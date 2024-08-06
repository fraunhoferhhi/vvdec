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

#pragma once

#include "vvdec/vvdec.h"
#include "DecoderLib/DecLib.h"             // internal decoder

namespace vvdec {

class FilmGrain;

static const char * const vvdecNalTypeNames[] = { "NAL_UNIT_CODED_SLICE_TRAIL", "NAL_UNIT_CODED_SLICE_STSA", "NAL_UNIT_CODED_SLICE_RADL", "NAL_UNIT_CODED_SLICE_RASL",
                                                  "NAL_UNIT_RESERVED_VCL_4", "NAL_UNIT_RESERVED_VCL_5", "NAL_UNIT_RESERVED_VCL_6",
                                                  "NAL_UNIT_CODED_SLICE_IDR_W_RADL","NAL_UNIT_CODED_SLICE_IDR_N_LP","NAL_UNIT_CODED_SLICE_CRA","NAL_UNIT_CODED_SLICE_GDR",
                                                  "NAL_UNIT_RESERVED_IRAP_VCL_11","NAL_UNIT_RESERVED_IRAP_VCL_12",
                                                  "NAL_UNIT_DCI","NAL_UNIT_VPS","NAL_UNIT_SPS","NAL_UNIT_PPS",
                                                  "NAL_UNIT_PREFIX_APS","NAL_UNIT_SUFFIX_APS","NAL_UNIT_PH","NAL_UNIT_ACCESS_UNIT_DELIMITER",
                                                  "NAL_UNIT_EOS","NAL_UNIT_EOB","NAL_UNIT_PREFIX_SEI","NAL_UNIT_SUFFIX_SEI","NAL_UNIT_FD",
                                                  "NAL_UNIT_RESERVED_NVCL_26","NAL_UNIT_RESERVED_NVCL_27",
                                                  "NAL_UNIT_UNSPECIFIED_28","NAL_UNIT_UNSPECIFIED_29","NAL_UNIT_UNSPECIFIED_30","NAL_UNIT_UNSPECIFIED_31",
                                                  "NAL_UNIT_INVALID", 0 };

static const char * const vvdecErrorMsg[] = { "expected behavior",
                                              "unspecified malfunction",
                                              "decoder not initialized or tried to initialize multiple times",
                                              "internal allocation error",
                                              "decoder input data error",
                                              "allocated memory too small to receive decoded data",
                                              "inconsistent or invalid parameters",
                                              "unsupported request",
                                              "decoder requires restart",
                                              "unsupported CPU - SSE 4.1 needed",
                                              "more bitstream data needed. try again",
                                              "end of stream",
                                              "unknown error code", 0 };

/**
  \ingroup hhivvcdeclibExternalInterfaces
  The class HhiVvcDec provides the decoder user interface. The simplest way to use the decoder is to call init() to initialize an decoder instance with the
  the given VVCDecoderParameters. After initialization the decoding of the video is performed by using the decoder() method to hand over compressed packets (bitstream chunks) in decoding order
  and retrieve uncompressed pictures. The decoding can be end by calling flush() that causes the decoder to finish decoding of all pending packets.
  Finally calling uninit() releases all allocated resources held by the decoder internally.
*/
class VVDecImpl
{
public:

  enum VVDecInternalState
  {
    INTERNAL_STATE_UNINITIALIZED = 0,
    INTERNAL_STATE_INITIALIZED   = 1,
    INTERNAL_STATE_TUNING_IN     = 2,
    INTERNAL_STATE_DECODING      = 3,
    INTERNAL_STATE_FLUSHING      = 4,
    INTERNAL_STATE_FINALIZED     = 5,
    INTERNAL_STATE_RESTART_REQUIRED = 6,
    INTERNAL_STATE_NOT_SUPPORTED = 7
  };

public:

  VVDecImpl();
  ~VVDecImpl();

  class FrameStorage
  {
  public:
    int allocateStorage( size_t size )
    {
      if( size == 0 ){ return VVDEC_ERR_ALLOCATE; }
      m_ptr.reset( new unsigned char[size] );
      m_size = size;
      return 0;
    }

    int freeStorage()
    {
      if( !m_ptr ) { return VVDEC_ERR_ALLOCATE; }
      m_ptr.reset();
      m_size = 0;
      return 0;
    }

    unsigned char * getStorage()
    {
      return m_ptr.get();
    }

    bool isAllocated() { return !!m_ptr; }
    bool isExternAllocator() { return m_isExternAllocator; }
    void setExternAllocator() { m_isExternAllocator = true; }

  private:
    std::unique_ptr<unsigned char[]> m_ptr               = nullptr;   // pointer to plane buffer
    size_t                           m_size              = 0;
    bool                             m_isExternAllocator = false;
  };

public:

  int init( const vvdecParams& params, vvdecCreateBufferCallback callbackCreateBuf = nullptr, vvdecUnrefBufferCallback callbackUnrefBuf = nullptr );

  int uninit();
  int reset();

  void setLoggingCallback( vvdecLoggingCallback callback );

  int decode( vvdecAccessUnit& accessUnit, vvdecFrame** ppframe );

  int flush( vvdecFrame** ppcFrame );

  vvdecSEI* findFrameSei( vvdecSEIPayloadType payloadType, vvdecFrame *frame );

  int objectUnref( vvdecFrame* pframe );

  int getNumberOfErrorsPictureHashSEI( );

  int setAndRetErrorMsg( int Ret, std::string errString = "" );

  const char* getDecoderInfo();

  template<class MembFunc, class... Args>
  auto catchExceptions( MembFunc fn, Args... args );

  static const char* getErrorMsg( int nRet );
  static const char* getVersionNumber();

  static vvdecNalType getNalUnitType       ( vvdecAccessUnit& accessUnit );
  static const char* getNalUnitTypeAsString( vvdecNalType t );
  static bool isNalUnitSlice               ( vvdecNalType t );

  std::string                             m_cErrorString;
  std::string                             m_cAdditionalErrorString;

private:
  int xAddPicture                  ( Picture* pcPic );
  int xCreateFrame                 ( vvdecFrame&        frame,
                                     const CPelUnitBuf& rcPicBuf,
                                     uint32_t           uiWidth,
                                     uint32_t           uiHeight,
                                     const BitDepths&   rcBitDepths,
                                     bool               bCreateStorage,
                                     bool               origStride = false );

  void xUpdateFGC                  ( vvdecSEI *sei );
  void xAddGrain                   ( vvdecFrame *frame );

  static int xRetrieveNalStartCode ( unsigned char *pB, int iZerosInStartcode );
  static int xConvertPayloadToRBSP ( const uint8_t* payload, size_t payloadLen, InputBitstream* bitstream, bool isVclNalUnit );
  static int xReadNalUnitHeader    ( InputNALUnit& nalu );

  int xHandleOutput( Picture* pcPic );
  bool isFrameConverted( vvdecFrame* frame );

  static int copyComp( const unsigned char* pucSrc, unsigned char* pucDest, unsigned int uiWidth, unsigned int uiHeight, ptrdiff_t iStrideSrc, ptrdiff_t iStrideDest, int iBytesPerSample );

  void vvdec_picAttributes_default(vvdecPicAttributes *attributes);
  void vvdec_frame_default(vvdecFrame *frame);
  void vvdec_plane_default(vvdecPlane *plane);
  void vvdec_frame_reset(vvdecFrame *frame );

private:
  typedef std::tuple<vvdecFrame, Picture*> FrameListEntry;
  typedef std::map<uint64_t, FrameStorage> FrameStorageMap;
  typedef FrameStorageMap::value_type      FrameStorageMapType;

  bool                                     m_bInitialized   = false;
  VVDecInternalState                       m_eState         = INTERNAL_STATE_UNINITIALIZED;
  ErrHandlingFlags                         m_eErrHandlingFlags = ERR_HANDLING_TRY_CONTINUE;

  std::unique_ptr<DecLib>                  m_cDecLib;

  std::list<FrameListEntry>                m_rcFrameList;
  std::list<FrameListEntry>::iterator      m_pcFrameNext = m_rcFrameList.begin();

  FrameStorageMap                          m_cFrameStorageMap;       // map of frame storage class( converted frames)
  UserAllocator                            m_cUserAllocator;         // user allocator object, valid if buffers are managed external

  std::string                              m_sDecoderInfo;
  std::string                              m_sDecoderCapabilities;

  uint64_t                                 m_uiSeqNumber       = 0;
  uint64_t                                 m_uiSeqNumOutput    = 0;
  enum
  {
    FgcNone        = 0,
    FgcDontPersist = 1,
    FgcPersist     = 2
  }                                        m_filmGrainCharacteristicsState = FgcNone;
  bool                                     m_enableFilmGrain               = false;
  std::unique_ptr<FilmGrain>               m_filmGrainSynth;
};

template<class MembFunc, class... Args>
inline auto VVDecImpl::catchExceptions( MembFunc fn, Args... args )
{
  using TRet = decltype( ( this->*fn )( args... ) ); // return type of the wapped function

  // helper function to either return an error value or a null-pointer based on the wrapped function's return type
  static auto returnErrOrNullptr = []( intptr_t err )
  {
    if( std::is_pointer<TRet>() )
      return (TRet) 0;
    return (TRet) err;
  };

  try
  {
    return ( this->*fn )( args... );
  }
  catch( UnsupportedFeatureException& e )
  {
    m_cErrorString           = "unsupported feature exception";
    m_cAdditionalErrorString = std::string( "caught exception for unsupported feature: " ) + e.what();
    m_eState                 = INTERNAL_STATE_NOT_SUPPORTED;
    return returnErrOrNullptr( VVDEC_ERR_NOT_SUPPORTED );
  }
  catch( Exception& e )
  {
    m_cErrorString           = "decoder exception";
    m_cAdditionalErrorString = std::string( "caught decoder exception: " ) + e.what();
    m_eState                 = INTERNAL_STATE_RESTART_REQUIRED;
    return returnErrOrNullptr( VVDEC_ERR_RESTART_REQUIRED );
  }
  catch( std::overflow_error& e )
  {
    m_cErrorString           = "overflow exception";
    m_cAdditionalErrorString = std::string( "caught overflow exception: " ) + e.what();
    m_eState                 = INTERNAL_STATE_RESTART_REQUIRED;
    return returnErrOrNullptr( VVDEC_ERR_RESTART_REQUIRED );
  }
  catch( std::exception& e )
  {
    m_cErrorString           = "unknown exception";
    m_cAdditionalErrorString = std::string( "caught unknown exception: " ) + e.what();
    m_eState                 = INTERNAL_STATE_RESTART_REQUIRED;
    return returnErrOrNullptr( VVDEC_ERR_RESTART_REQUIRED );
  }
  catch( ... )
  {
    m_cErrorString           = "unknown exception";
    m_cAdditionalErrorString = "no details available";
    m_eState                 = INTERNAL_STATE_RESTART_REQUIRED;
    return returnErrOrNullptr( VVDEC_ERR_RESTART_REQUIRED );
  }
}

// this is needed by the vvdecapp to implement upscaling, but is not an official API for libvvdec
VVDEC_DECL void rescalePlane( const vvdecPlane&      srcPlane,
                              vvdecPlane&            dstPlane,
                              int                    planeComponent,
                              const vvdecColorFormat colorFormat,
                              int                    bitDepth,
                              const bool             horCollocatedChromaFlag,
                              const bool             verCollocatedChromaFlag );

}   // namespace vvdec
