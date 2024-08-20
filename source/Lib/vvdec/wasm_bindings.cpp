/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2018-2024, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVdeC Authors.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

     * Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

     * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.

     * Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from this
     software without specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.


------------------------------------------------------------------------------------------- */

#ifdef __EMSCRIPTEN__

#include "wasm_bindings.h"
#include "vvdec/vvdec.h"

#include <emscripten.h>
#include <emscripten/bind.h>

#ifdef TARGET_SIMD_WASM
#include <wasm_simd128.h>
#endif

#include <vector>
#include <iostream>

using namespace emscripten;

// helper-class so we don't need to manually work with double-pointers in JavaScript
struct vvdecFrameHandle
{
  vvdecFrame* frame = nullptr;

  val  getFrame() const { return val( frame ); }
  void reset() { frame = nullptr; }
};

extern "C" {
_Pragma( "GCC diagnostic push" );
_Pragma( "GCC diagnostic ignored \"-Wextern-c-compat\"" );

// dummy definition of vvdecDecoder, so we can use it in bindings (incomplete types don't work)
struct vvdecDecoder {};

_Pragma( "GCC diagnostic pop" )
}

// wrapper functions so we pass the vvdecFrame** doublepointer as a vvdecFrameHandle
static auto wrapped_vvdec_decode( vvdecDecoder* dec, vvdecAccessUnit* au, vvdecFrameHandle& handle ) { return vvdec_decode( dec, au, &handle.frame ); };
static auto wrapped_vvdec_flush ( vvdecDecoder* dec,                      vvdecFrameHandle& handle ) { return vvdec_flush ( dec,     &handle.frame ); };
// wrappers functions to return proper strings
static std::string wrapped_vvdec_get_nal_unit_type_name   ( vvdecNalType t )    { return vvdec_get_nal_unit_type_name   ( t );   }
static std::string wrapped_vvdec_get_version              ()                    { return vvdec_get_version              ();      }
static std::string wrapped_vvdec_get_dec_information      ( vvdecDecoder* dec ) { return vvdec_get_dec_information      ( dec ); }
static std::string wrapped_vvdec_get_last_error           ( vvdecDecoder* dec ) { return vvdec_get_last_error           ( dec ); }
static std::string wrapped_vvdec_get_last_additional_error( vvdecDecoder* dec ) { return vvdec_get_last_additional_error( dec ); }

static val getPlanesArray( const vvdecFrame& f )
{
  vvdecPlane* p0 = const_cast<vvdecPlane*>( &f.planes[0] );
  vvdecPlane* p1 = const_cast<vvdecPlane*>( &f.planes[1] );
  vvdecPlane* p2 = const_cast<vvdecPlane*>( &f.planes[2] );
  return val::array<val>( { val( p0 ), val( p1 ), val( p2 ) } );
}

// return a JavaScript TypedArray from raw plane pointers
static val getArrayView( const vvdecPlane& plane )
{
  int strideElems = plane.stride / plane.bytesPerSample;
  if( plane.bytesPerSample == 1 )
    return val( typed_memory_view<uint8_t>( strideElems * plane.height, plane.ptr ) );
  else if( plane.bytesPerSample == 2 )
    return val( typed_memory_view<uint16_t>( strideElems * plane.height, (uint16_t*) plane.ptr ) );
  else
    abort();
}

// this is probably not the intended way to use the embind API, but currently the only way to define a custom deleter
namespace emscripten
{
namespace internal
{
template<> void raw_destructor( vvdecAccessUnit* ptr ){ vvdec_accessUnit_free( ptr ); }
template<> void raw_destructor( vvdecDecoder* ptr )   { vvdec_decoder_close  ( ptr ); }
template<> void raw_destructor( vvdecParams* ptr )    { vvdec_params_free    ( ptr ); }
}  // namespace internal
}  // namespace emscripten

EMSCRIPTEN_BINDINGS( vvdec )
{
  // clang-format off
  function( "get_RGBA_image_JS", vvdec_get_RGBA_image_JS, allow_raw_pointers() );

  function( "get_version",            wrapped_vvdec_get_version,    allow_raw_pointers() );
  function( "get_nal_unit_type_name", vvdec_get_nal_unit_type_name, allow_raw_pointers() );
  function( "is_nal_unit_slice",      vvdec_is_nal_unit_slice,      allow_raw_pointers() );

  class_<vvdecAccessUnit>( "AccessUnit" )
    .constructor( vvdec_accessUnit_alloc )
    .function( "alloc_payload",     vvdec_accessUnit_alloc_payload, allow_raw_pointers() )
    .function( "free_payload",      vvdec_accessUnit_free_payload,  allow_raw_pointers() )
    .function( "default",           vvdec_accessUnit_default,       allow_raw_pointers() )
    .function( "get_nal_unit_type", vvdec_get_nal_unit_type,        allow_raw_pointers() )
    .property<val>( "payload",
                    []( const auto& au ) {
                      return val( typed_memory_view<uint8_t>( au.payloadSize, au.payload ) );
                    } )
    .property( "payloadSize",     &vvdecAccessUnit::payloadSize     )
    .property( "payloadUsedSize", &vvdecAccessUnit::payloadUsedSize )
    .property( "cts",             &vvdecAccessUnit::cts             )
    .property( "dts",             &vvdecAccessUnit::dts             )
    .property( "ctsValid",        &vvdecAccessUnit::ctsValid        )
    .property( "dtsValid",        &vvdecAccessUnit::dtsValid        )
    .property( "rap",             &vvdecAccessUnit::rap             )
  ;


  class_<vvdecDecoder>( "Decoder" )
    .constructor( vvdec_decoder_open )
    .function( "set_logging_callback",      vvdec_set_logging_callback,              allow_raw_pointers() ) //  this probably doesn't work yet
    .function( "find_frame_sei",            vvdec_find_frame_sei,                    allow_raw_pointers() )
    .function( "decode",                    wrapped_vvdec_decode,                    allow_raw_pointers() )
    .function( "flush",                     wrapped_vvdec_flush,                     allow_raw_pointers() )
    .function( "frame_unref",               vvdec_frame_unref,                       allow_raw_pointers() )
    .function( "get_hash_error_count",      vvdec_get_hash_error_count,              allow_raw_pointers() )
    .function( "get_dec_information",       wrapped_vvdec_get_dec_information,       allow_raw_pointers() )
    .function( "get_last_error",            wrapped_vvdec_get_last_error,            allow_raw_pointers() )
    .function( "get_last_additional_error", wrapped_vvdec_get_last_additional_error, allow_raw_pointers() )
  ;

  class_<vvdecParams>( "Params" )
    .constructor( vvdec_params_alloc )
    .property( "threads",            &vvdecParams::threads            )
    .property( "parseDelay",         &vvdecParams::parseDelay         )
    .property( "logLevel",           &vvdecParams::logLevel           )
    .property( "verifyPictureHash",  &vvdecParams::verifyPictureHash  )
    .property( "filmGrainSynthesis", &vvdecParams::filmGrainSynthesis )
    // .property( "simd",              &vvdecParams::simd              ) // SIMD is currently hardcoded to SSE42 for wasm
  ;

  class_<vvdecPlane>( "Plane" )
    .property( "ptr",            &getArrayView               )
    .property( "width",          &vvdecPlane::width          )
    .property( "height",         &vvdecPlane::height         )
    .property( "stride",         &vvdecPlane::stride         )
    .property( "bytesPerSample", &vvdecPlane::bytesPerSample )
  ;

  class_<vvdecFrame>( "Frame" )
    .property( "planes",         &getPlanesArray             )
    .property( "numPlanes",      &vvdecFrame::numPlanes      )
    .property( "width",          &vvdecFrame::width          )
    .property( "height",         &vvdecFrame::height         )
    .property( "bitDepth",       &vvdecFrame::bitDepth       )
    .property( "frameFormat",    &vvdecFrame::frameFormat    )
    .property( "colorFormat",    &vvdecFrame::colorFormat    )
    .property( "sequenceNumber", &vvdecFrame::sequenceNumber )
    .property( "cts",            &vvdecFrame::cts            )
    .property( "ctsValid",       &vvdecFrame::ctsValid       )
    .property<val>( "picAttributes", []( auto& f ){ return val( f.picAttributes ); } )
  ;

  // handle class to avoid having to manually work with double pointers in javascript
  class_<vvdecFrameHandle>( "FrameHandle" )
    .constructor()
    .function( "reset", &vvdecFrameHandle::reset )
    .property( "frame", &vvdecFrameHandle::getFrame )
  ;

  class_<vvdecPicAttributes>( "PicAttributes" )
    .property( "nalType",       &vvdecPicAttributes::nalType       )
    .property( "sliceType",     &vvdecPicAttributes::sliceType     )
    .property( "isRefPic",      &vvdecPicAttributes::isRefPic      )
    .property( "temporalLayer", &vvdecPicAttributes::temporalLayer )
    .property( "poc",           &vvdecPicAttributes::poc           )
    .property( "bits",          &vvdecPicAttributes::bits          )
    .property<val>( "vui", []( auto& a ){ return val( a.vui ); }   )
    .property<val>( "hrd", []( auto& a ){ return val( a.hrd ); }   )
  ;

  class_<vvdecVui>( "Vui" )
    .property( "aspectRatioInfoPresentFlag",     &vvdecVui::aspectRatioInfoPresentFlag     )
    .property( "aspectRatioConstantFlag",        &vvdecVui::aspectRatioConstantFlag        )
    .property( "nonPackedFlag",                  &vvdecVui::nonPackedFlag                  )
    .property( "nonProjectedFlag",               &vvdecVui::nonProjectedFlag               )
    .property( "aspectRatioIdc",                 &vvdecVui::aspectRatioIdc                 )
    .property( "sarWidth",                       &vvdecVui::sarWidth                       )
    .property( "sarHeight",                      &vvdecVui::sarHeight                      )
    .property( "colourDescriptionPresentFlag",   &vvdecVui::colourDescriptionPresentFlag   )
    .property( "colourPrimaries",                &vvdecVui::colourPrimaries                )
    .property( "transferCharacteristics",        &vvdecVui::transferCharacteristics        )
    .property( "matrixCoefficients",             &vvdecVui::matrixCoefficients             )
    .property( "progressiveSourceFlag",          &vvdecVui::progressiveSourceFlag          )
    .property( "interlacedSourceFlag",           &vvdecVui::interlacedSourceFlag           )
    .property( "chromaLocInfoPresentFlag",       &vvdecVui::chromaLocInfoPresentFlag       )
    .property( "chromaSampleLocTypeTopField",    &vvdecVui::chromaSampleLocTypeTopField    )
    .property( "chromaSampleLocTypeBottomField", &vvdecVui::chromaSampleLocTypeBottomField )
    .property( "chromaSampleLocType",            &vvdecVui::chromaSampleLocType            )
    .property( "overscanInfoPresentFlag",        &vvdecVui::overscanInfoPresentFlag        )
    .property( "overscanAppropriateFlag",        &vvdecVui::overscanAppropriateFlag        )
    .property( "videoSignalTypePresentFlag",     &vvdecVui::videoSignalTypePresentFlag     )
    .property( "videoFullRangeFlag",             &vvdecVui::videoFullRangeFlag             )
  ;

  class_<vvdecHrd>( "Hrd" )
    .property( "numUnitsInTick",                          &vvdecHrd::numUnitsInTick                          )
    .property( "timeScale",                               &vvdecHrd::timeScale                               )
    .property( "generalNalHrdParamsPresentFlag",          &vvdecHrd::generalNalHrdParamsPresentFlag          )
    .property( "generalVclHrdParamsPresentFlag",          &vvdecHrd::generalVclHrdParamsPresentFlag          )
    .property( "generalSamePicTimingInAllOlsFlag",        &vvdecHrd::generalSamePicTimingInAllOlsFlag        )
    .property( "tickDivisor",                             &vvdecHrd::tickDivisor                             )
    .property( "generalDecodingUnitHrdParamsPresentFlag", &vvdecHrd::generalDecodingUnitHrdParamsPresentFlag )
    .property( "bitRateScale",                            &vvdecHrd::bitRateScale                            )
    .property( "cpbSizeScale",                            &vvdecHrd::cpbSizeScale                            )
    .property( "cpbSizeDuScale",                          &vvdecHrd::cpbSizeDuScale                          )
    .property( "hrdCpbCnt",                               &vvdecHrd::hrdCpbCnt                               )
  ;

  enum_<vvdecLogLevel>( "LogLevel")
    .value( "SILENT",  VVDEC_SILENT  )
    .value( "ERROR",   VVDEC_ERROR   )
    .value( "WARNING", VVDEC_WARNING )
    .value( "INFO",    VVDEC_INFO    )
    .value( "NOTICE",  VVDEC_NOTICE  )
    .value( "VERBOSE", VVDEC_VERBOSE )
    .value( "DETAILS", VVDEC_DETAILS )
  ;

  enum_<vvdecSIMD_Extension>( "SIMD_Extension" )
    .value( "SIMD_DEFAULT", VVDEC_SIMD_DEFAULT )
    .value( "SIMD_SCALAR",  VVDEC_SIMD_SCALAR  )
    .value( "SIMD_WASM",    VVDEC_SIMD_WASM    )
  ;

  enum_<vvdecColorFormat>( "ColorFormat" )
    .value( "CF_INVALID",       VVDEC_CF_INVALID       )
    .value( "CF_YUV400_PLANAR", VVDEC_CF_YUV400_PLANAR )
    .value( "CF_YUV420_PLANAR", VVDEC_CF_YUV420_PLANAR )
    .value( "CF_YUV422_PLANAR", VVDEC_CF_YUV422_PLANAR )
    .value( "CF_YUV444_PLANAR", VVDEC_CF_YUV444_PLANAR )
  ;

  enum_<vvdecFrameFormat>( "FrameFormat" )
    .value( "FF_INVALID",     VVDEC_FF_INVALID     )
    .value( "FF_PROGRESSIVE", VVDEC_FF_PROGRESSIVE )
    .value( "FF_TOP_FIELD",   VVDEC_FF_TOP_FIELD   )
    .value( "FF_BOT_FIELD",   VVDEC_FF_BOT_FIELD   )
    .value( "FF_TOP_BOT",     VVDEC_FF_TOP_BOT     )
    .value( "FF_BOT_TOP",     VVDEC_FF_BOT_TOP     )
    .value( "FF_TOP_BOT_TOP", VVDEC_FF_TOP_BOT_TOP )
    .value( "FF_BOT_TOP_BOT", VVDEC_FF_BOT_TOP_BOT )
    .value( "FF_FRAME_DOUB",  VVDEC_FF_FRAME_DOUB  )
    .value( "FF_FRAME_TRIP",  VVDEC_FF_FRAME_TRIP  )
    .value( "FF_TOP_PW_PREV", VVDEC_FF_TOP_PW_PREV )
    .value( "FF_BOT_PW_PREV", VVDEC_FF_BOT_PW_PREV )
    .value( "FF_TOP_PW_NEXT", VVDEC_FF_TOP_PW_NEXT )
    .value( "FF_BOT_PW_NEXT", VVDEC_FF_BOT_PW_NEXT )
  ;

  enum_<vvdecSliceType>( "SliceType" )
    .value( "SLICETYPE_I",       VVDEC_SLICETYPE_I       )
    .value( "SLICETYPE_P",       VVDEC_SLICETYPE_P       )
    .value( "SLICETYPE_B",       VVDEC_SLICETYPE_B       )
    .value( "SLICETYPE_UNKNOWN", VVDEC_SLICETYPE_UNKNOWN )
  ;

  enum_<vvdecNalType>( "NalType" )
    .value( "NAL_UNIT_CODED_SLICE_TRAIL",      VVC_NAL_UNIT_CODED_SLICE_TRAIL      )
    .value( "NAL_UNIT_CODED_SLICE_STSA",       VVC_NAL_UNIT_CODED_SLICE_STSA       )
    .value( "NAL_UNIT_CODED_SLICE_RADL",       VVC_NAL_UNIT_CODED_SLICE_RADL       )
    .value( "NAL_UNIT_CODED_SLICE_RASL",       VVC_NAL_UNIT_CODED_SLICE_RASL       )

    .value( "NAL_UNIT_RESERVED_VCL_4",         VVC_NAL_UNIT_RESERVED_VCL_4         )
    .value( "NAL_UNIT_RESERVED_VCL_5",         VVC_NAL_UNIT_RESERVED_VCL_5         )
    .value( "NAL_UNIT_RESERVED_VCL_6",         VVC_NAL_UNIT_RESERVED_VCL_6         )

    .value( "NAL_UNIT_CODED_SLICE_IDR_W_RADL", VVC_NAL_UNIT_CODED_SLICE_IDR_W_RADL )
    .value( "NAL_UNIT_CODED_SLICE_IDR_N_LP",   VVC_NAL_UNIT_CODED_SLICE_IDR_N_LP   )
    .value( "NAL_UNIT_CODED_SLICE_CRA",        VVC_NAL_UNIT_CODED_SLICE_CRA        )
    .value( "NAL_UNIT_CODED_SLICE_GDR",        VVC_NAL_UNIT_CODED_SLICE_GDR        )

    .value( "NAL_UNIT_RESERVED_IRAP_VCL_11",   VVC_NAL_UNIT_RESERVED_IRAP_VCL_11   )
    .value( "NAL_UNIT_RESERVED_IRAP_VCL_12",   VVC_NAL_UNIT_RESERVED_IRAP_VCL_12   )

    .value( "NAL_UNIT_DCI",                    VVC_NAL_UNIT_DCI                    )
    .value( "NAL_UNIT_VPS",                    VVC_NAL_UNIT_VPS                    )
    .value( "NAL_UNIT_SPS",                    VVC_NAL_UNIT_SPS                    )
    .value( "NAL_UNIT_PPS",                    VVC_NAL_UNIT_PPS                    )
    .value( "NAL_UNIT_PREFIX_APS",             VVC_NAL_UNIT_PREFIX_APS             )
    .value( "NAL_UNIT_SUFFIX_APS",             VVC_NAL_UNIT_SUFFIX_APS             )
    .value( "NAL_UNIT_PH",                     VVC_NAL_UNIT_PH                     )
    .value( "NAL_UNIT_ACCESS_UNIT_DELIMITER",  VVC_NAL_UNIT_ACCESS_UNIT_DELIMITER  )
    .value( "NAL_UNIT_EOS",                    VVC_NAL_UNIT_EOS                    )
    .value( "NAL_UNIT_EOB",                    VVC_NAL_UNIT_EOB                    )
    .value( "NAL_UNIT_PREFIX_SEI",             VVC_NAL_UNIT_PREFIX_SEI             )
    .value( "NAL_UNIT_SUFFIX_SEI",             VVC_NAL_UNIT_SUFFIX_SEI             )
    .value( "NAL_UNIT_FD",                     VVC_NAL_UNIT_FD                     )

    .value( "NAL_UNIT_RESERVED_NVCL_26",       VVC_NAL_UNIT_RESERVED_NVCL_26       )
    .value( "NAL_UNIT_RESERVED_NVCL_27",       VVC_NAL_UNIT_RESERVED_NVCL_27       )

    .value( "NAL_UNIT_UNSPECIFIED_28",         VVC_NAL_UNIT_UNSPECIFIED_28         )
    .value( "NAL_UNIT_UNSPECIFIED_29",         VVC_NAL_UNIT_UNSPECIFIED_29         )
    .value( "NAL_UNIT_UNSPECIFIED_30",         VVC_NAL_UNIT_UNSPECIFIED_30         )
    .value( "NAL_UNIT_UNSPECIFIED_31",         VVC_NAL_UNIT_UNSPECIFIED_31         )
    .value( "NAL_UNIT_INVALID",                VVC_NAL_UNIT_INVALID                )
  ;

  enum_<vvdecComponentType>( "ComponentType" )
    .value( "CT_Y", VVDEC_CT_Y )
    .value( "CT_U", VVDEC_CT_U )
    .value( "CT_V", VVDEC_CT_V )
  ;
  // clang-format on
}

static const val Uint8ClampedArray = val::global( "Uint8ClampedArray" );
static const val ImageData         = val::global( "ImageData" );

template<class T>
void vvdec_get_RGBA_frame_impl( uint8_t* outBuffer, const vvdecFrame* frame );

val vvdec_get_RGBA_image_JS( intptr_t f )
{
  vvdecFrame* frame = (vvdecFrame*) f;
  if( frame->planes[0].bytesPerSample != frame->planes[1].bytesPerSample || frame->planes[0].bytesPerSample != frame->planes[2].bytesPerSample )
  {
    abort();
  }

  static std::vector<uint8_t> buffer( 4 * frame->width * frame->height, 255 );
  if( buffer.size() != 4 * frame->width * frame->height)
  {
    buffer.resize( 4 * frame->width * frame->height, 255 );
  }

  if( frame->planes[0].bytesPerSample == 1 )
    vvdec_get_RGBA_frame_impl<uint8_t>( buffer.data(), frame );
  else if( frame->planes[0].bytesPerSample == 2 )
    vvdec_get_RGBA_frame_impl<uint16_t>( buffer.data(), frame );
  else
    abort();

  val array = Uint8ClampedArray.new_( typed_memory_view<uint8_t>( buffer.size(), buffer.data() ) );
  return ImageData.new_( array, frame->width, frame->height );
}

template<class Tout, class Tin> static inline Tout clamp( Tin val )
{
  return std::min<Tin>( std::max<Tin>( std::numeric_limits<Tout>::min(), val ), std::numeric_limits<Tout>::max() );
}

template<class T>
void vvdec_get_RGBA_frame_impl( uint8_t* outBuffer, const vvdecFrame* frame )
{
  if( frame->colorFormat != VVDEC_CF_YUV420_PLANAR )
    abort();

  const auto width    = frame->width;
  const auto height   = frame->height;
  const auto strideY  = frame->planes[0].stride / sizeof( T );
  const auto strideUV = frame->planes[1].stride / sizeof( T );
  const auto bitDepth = frame->bitDepth;

  const int PR     = 10;   // Precision shift for float to int conversion
  const int shift  = bitDepth - 8;
  const int uvOffs = 1 << ( bitDepth - 1 );

  const T* frameY = (T*) frame->planes[0].ptr;
  const T* frameU = (T*) frame->planes[1].ptr;
  const T* frameV = (T*) frame->planes[2].ptr;

#ifdef TARGET_SIMD_WASM
  if( sizeof( T ) == 1 )
#endif
  {
    int iRGB = 0;
    for( unsigned y = 0; y < height; ++y )
    {
      for( unsigned x = 0; x < width; ++x )
      {
        // indices for Y, U, or V arrays
        const int iY  = x + y * strideY;
        const int iUV = x / 2 + y / 2 * strideUV;

        // input values (U&V converted to signed range)
        const int Y = frameY[iY];
        const int U = frameU[iUV] - uvOffs;
        const int V = frameV[iUV] - uvOffs;

        // integer RGB conversion
        const int16_t R = Y + ( (                                (int)( (1<<PR) * 1.403 ) * V ) >> PR );
        const int16_t G = Y - ( ( (int)( (1<<PR) * 0.344 ) * U + (int)( (1<<PR) * 0.714 ) * V ) >> PR );
        const int16_t B = Y + ( ( (int)( (1<<PR) * 1.770 ) * U                                ) >> PR );

        // convert from original bit depth to 8 bit and clamp
        outBuffer[iRGB++] = clamp<uint8_t>( R >> shift );
        outBuffer[iRGB++] = clamp<uint8_t>( G >> shift );
        outBuffer[iRGB++] = clamp<uint8_t>( B >> shift );
        // outBuffer[iRGB++] = 255;
        iRGB++;   // outBuffer is initialized with 255 on allocation, so we can skip setting alpha
      }
    }
  }
#ifdef TARGET_SIMD_WASM
  else
  {
    const v128_t uvOffs128 = wasm_i32x4_splat( uvOffs );

    // factors for the linear equations
    const v128_t fac1403 = wasm_i32x4_const_splat( 1.403 * ( 1 << PR ) );
    const v128_t fac344  = wasm_i32x4_const_splat( 0.344 * ( 1 << PR ) );
    const v128_t fac714  = wasm_i32x4_const_splat( 0.714 * ( 1 << PR ) );
    const v128_t fac1770 = wasm_i32x4_const_splat( 1.770 * ( 1 << PR ) );

    const v128_t alpha = wasm_i16x8_const_splat( 255 );

    for( unsigned y = 0; y < height; y += 2 )   // process two lines together
    {
      unsigned char* pRGB = outBuffer + 4 * width * y;
      for( unsigned x = 0; x < width; x += 8 )   // process 8 Pels per step
      {
        // indices for Y, U, or V arrays
        const int iY  = x + y * strideY;
        const int iUV = x / 2 + y / 2 * strideUV;

        // load UV, zero-extend to 4 * uin32_t, and  convert to signed range
        v128_t V = wasm_i32x4_sub( wasm_u32x4_load16x4( &frameV[iUV] ), uvOffs128 );
        v128_t U = wasm_i32x4_sub( wasm_u32x4_load16x4( &frameU[iUV] ), uvOffs128 );

        // linear equations
        v128_t UV_R =                                               wasm_i32x4_mul( V, fac1403 )  ;
        v128_t UV_G = wasm_i32x4_add( wasm_i32x4_mul( U,  fac344 ), wasm_i32x4_mul( V,  fac714 ) );
        v128_t UV_B =                 wasm_i32x4_mul( U, fac1770 )                                ;

        // shift back
        UV_R = wasm_i32x4_shr( UV_R, PR );
        UV_G = wasm_i32x4_shr( UV_G, PR );
        UV_B = wasm_i32x4_shr( UV_B, PR );

        // duplicate each U and V value
        UV_R = wasm_i16x8_shuffle( UV_R, UV_R, 0, 0, 2, 2, 4, 4, 6, 6 );
        UV_G = wasm_i16x8_shuffle( UV_G, UV_G, 0, 0, 2, 2, 4, 4, 6, 6 );
        UV_B = wasm_i16x8_shuffle( UV_B, UV_B, 0, 0, 2, 2, 4, 4, 6, 6 );

        // process two y lines
        for( int i = 0; i < 2; ++i )
        {
          // load 8 pels
          v128_t Y = wasm_v128_load( &frameY[iY + i * strideY] );

          v128_t R = wasm_i16x8_add_sat( Y, UV_R );
          v128_t G = wasm_i16x8_sub_sat( Y, UV_G );
          v128_t B = wasm_i16x8_add_sat( Y, UV_B );

          // shift right for 8 bit conversion
          R = wasm_i16x8_shr( R, shift );
          G = wasm_i16x8_shr( G, shift );
          B = wasm_i16x8_shr( B, shift );

          // narrow to actually use valid 8 bit range
          v128_t RG = wasm_u8x16_narrow_i16x8( R, G );
          v128_t BA = wasm_u8x16_narrow_i16x8( B, alpha );

          // interleave channels and store result
          wasm_v128_store( pRGB + i * width * 4,     wasm_i8x16_shuffle( RG, BA,
                                                                         0,  8, 16, 24,
                                                                         1,  9, 17, 25,
                                                                         2, 10, 18, 26,
                                                                         3, 11, 19, 27
                                                                         ) );
          wasm_v128_store( pRGB + i * width * 4 + 16, wasm_i8x16_shuffle( RG, BA,
                                                                          4, 12, 20, 28,
                                                                          5, 13, 21, 29,
                                                                          6, 14, 22, 30,
                                                                          7, 15, 23, 31
                                                                          ) );
        }
        pRGB += 32;
      }
    }
  }
#endif
}

#endif   // __EMSCRIPTEN__
