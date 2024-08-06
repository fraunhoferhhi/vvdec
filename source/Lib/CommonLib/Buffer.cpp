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

/** \file     Buffer.cpp
 *  \brief    Low-overhead class describing 2D memory layout
 */

#define DONT_UNDEF_SIZE_AWARE_PER_EL_OP

// unit needs to come first due to a forward declaration

#include "Unit.h"
#include "Buffer.h"
#include "InterpolationFilter.h"
#include "Picture.h"
#include "Slice.h"

#if ENABLE_SIMD_OPT_BUFFER && defined( TARGET_SIMD_X86 )
#include "CommonDefX86.h"
#include <simde/x86/sse.h>
#endif

namespace vvdec
{

template< typename T >
void addAvgCore( const T* src1, ptrdiff_t src1Stride, const T* src2, ptrdiff_t src2Stride, T* dest, ptrdiff_t dstStride, int width, int height, int rshift, int offset, const ClpRng& clpRng )
{
#define ADD_AVG_CORE_OP( ADDR ) dest[ADDR] = ClipPel( rightShift( ( src1[ADDR] + src2[ADDR] + offset ), rshift ), clpRng )
#define ADD_AVG_CORE_INC    \
  src1 += src1Stride;       \
  src2 += src2Stride;       \
  dest +=  dstStride;       \

  SIZE_AWARE_PER_EL_OP( ADD_AVG_CORE_OP, ADD_AVG_CORE_INC );

#undef ADD_AVG_CORE_OP
#undef ADD_AVG_CORE_INC
}

template<typename T>
void reconstructCore( const T* src1, ptrdiff_t src1Stride, const T* src2, ptrdiff_t src2Stride, T* dest, ptrdiff_t dstStride, int width, int height, const ClpRng& clpRng )
{
#define RECO_CORE_OP( ADDR ) dest[ADDR] = ClipPel( src1[ADDR] + src2[ADDR], clpRng )
#define RECO_CORE_INC     \
  src1 += src1Stride;     \
  src2 += src2Stride;     \
  dest +=  dstStride;     \

  SIZE_AWARE_PER_EL_OP( RECO_CORE_OP, RECO_CORE_INC );

#undef RECO_CORE_OP
#undef RECO_CORE_INC
}


template<typename T>
void linTfCore( const T* src, ptrdiff_t srcStride, Pel *dst, ptrdiff_t dstStride, int width, int height, int scale, int shift, int offset, const ClpRng& clpRng, bool bClip )
{
#define LINTF_CORE_OP( ADDR ) dst[ADDR] = ( Pel ) bClip ? ClipPel( rightShift( scale * src[ADDR], shift ) + offset, clpRng ) : ( rightShift( scale * src[ADDR], shift ) + offset )
#define LINTF_CORE_INC  \
  src += srcStride;     \
  dst += dstStride;     \

  SIZE_AWARE_PER_EL_OP( LINTF_CORE_OP, LINTF_CORE_INC );

#undef LINTF_CORE_OP
#undef LINTF_CORE_INC
}

template<typename T>
void transpose4x4Core( const Pel* src, ptrdiff_t srcStride, Pel* dst, ptrdiff_t dstStride )
{
  for( int i = 0; i < 4; i++ )
  {
    for( int j = 0; j < 4; j++ )
    {
      dst[j * dstStride] = src[j];
    }

    dst++;
    src += srcStride;
  }
}

template<typename T>
void transpose8x8Core( const Pel* src, ptrdiff_t srcStride, Pel* dst, ptrdiff_t dstStride )
{
  for( int i = 0; i < 8; i++ )
  {
    for( int j = 0; j < 8; j++ )
    {
      dst[j * dstStride] = src[j];
    }

    dst++;
    src += srcStride;
  }
}

template<typename T>
void copyClipCore( const T* src, ptrdiff_t srcStride, Pel *dst, ptrdiff_t dstStride, int width, int height, const ClpRng& clpRng )
{
#define RECO_OP( ADDR ) dst[ADDR] = ClipPel( src[ADDR], clpRng )
#define RECO_INC      \
    src += srcStride; \
    dst += dstStride; \

  SIZE_AWARE_PER_EL_OP( RECO_OP, RECO_INC );

#undef RECO_OP
#undef RECO_INC
}

template<typename T>
void addWeightedAvgCore( const T* src1, ptrdiff_t src1Stride, const T* src2, ptrdiff_t src2Stride, T* dest, ptrdiff_t destStride, int width, int height, int rshift, int offset, int w0, int w1, const ClpRng& clpRng )
{
#define ADD_WGHT_AVG_OP( ADDR ) dest[ADDR] = ClipPel( rightShift( ( src1[ADDR]*w0 + src2[ADDR]*w1 + offset ), rshift ), clpRng )
#define ADD_WGHT_AVG_INC     \
    src1 += src1Stride; \
    src2 += src2Stride; \
    dest += destStride; \

  SIZE_AWARE_PER_EL_OP( ADD_WGHT_AVG_OP, ADD_WGHT_AVG_INC );

#undef ADD_WGHT_AVG_OP
#undef ADD_WGHT_AVG_INC
}

void copyBufferCore( const char *src, ptrdiff_t srcStride, char *dst, ptrdiff_t dstStride, int width, int height )
{
#if ENABLE_SIMD_OPT_BUFFER && defined( TARGET_SIMD_X86 )
  _mm_prefetch( (const char *) ( src ),             _MM_HINT_T0 );
  _mm_prefetch( (const char *) ( src + srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char *) ( dst ),             _MM_HINT_T0 );
  _mm_prefetch( (const char *) ( dst + dstStride ), _MM_HINT_T0 );

#endif
  if( width == srcStride && width == dstStride )
  {
    memcpy( dst, src, width * height );
    return;
  }

  for( int i = 0; i < height; i++ )
  {
#if ENABLE_SIMD_OPT_BUFFER && defined( TARGET_SIMD_X86 )
    _mm_prefetch( (const char *) ( src + srcStride ), _MM_HINT_T0 );
    _mm_prefetch( (const char *) ( dst + dstStride ), _MM_HINT_T0 );

#endif
    memcpy( dst, src, width );

    src += srcStride;
    dst += dstStride;
  }
}

void applyLutCore( Pel* ptr, ptrdiff_t ptrStride, int width, int height, const Pel* lut )
{
  //    const auto rsp_sgnl_op  = [=, &dst]( int ADDR ){ dst[ADDR] = lut[dst[ADDR]]; };
  //    const auto rsp_sgnl_inc = [=, &dst]            { dst += stride;              };

  //    size_aware_pel_op( rsp_sgnl_op, rsp_sgnl_inc, width, height );

    #define RSP_SGNL_OP( ADDR ) ptr[ADDR] = lut[ptr[ADDR]]
    #define RSP_SGNL_INC        ptr      += ptrStride;

      SIZE_AWARE_PER_EL_OP( RSP_SGNL_OP, RSP_SGNL_INC )

    #undef RSP_SGNL_OP
    #undef RSP_SGNL_INC
    
    return;
}

void fillN_CuCore( CodingUnit** ptr, ptrdiff_t ptrStride, int width, int height, CodingUnit* cuPtr )
{
  if( width == ptrStride )
  {
    std::fill_n( ptr, width * height, cuPtr );
  }
  else
  {
    CodingUnit** dst = ptr;

    for( int y = 0; y < height; y++, dst += ptrStride )
    {
      std::fill_n( dst, width, cuPtr );
    }
  }
}

void sampleRateConvCore( const std::pair<int, int> scalingRatio, const std::pair<int, int> compScale,
                         const Pel* orgSrc, const ptrdiff_t orgStride, const int orgWidth, const int orgHeight,
                         const int beforeScaleLeftOffset, const int beforeScaleTopOffset,
                         Pel* scaledSrc, const ptrdiff_t scaledStride, const int scaledWidth, const int scaledHeight,
                         const int afterScaleLeftOffset, const int afterScaleTopOffset,
                         const int bitDepth, const bool useLumaFilter,
                         const bool horCollocatedPositionFlag, const bool verCollocatedPositionFlag )
{
  if( orgWidth == scaledWidth && orgHeight == scaledHeight && scalingRatio == SCALE_1X && !beforeScaleLeftOffset && !beforeScaleTopOffset && !afterScaleLeftOffset && !afterScaleTopOffset )
  {
    g_pelBufOP.copyBuffer( ( const char * ) orgSrc, orgStride * sizeof( Pel ), ( char* ) scaledSrc, scaledStride * sizeof( Pel ), orgWidth * sizeof( Pel ), orgHeight );

    return;
  }

  const TFilterCoeff* filterHor = useLumaFilter ? &InterpolationFilter::m_lumaFilter[0][0] : &InterpolationFilter::m_chromaFilter[0][0];
  const TFilterCoeff* filterVer = useLumaFilter ? &InterpolationFilter::m_lumaFilter[0][0] : &InterpolationFilter::m_chromaFilter[0][0];

  const int numFracPositions  = useLumaFilter ? 15 : 31;
  const int numFracShift      = useLumaFilter ? 4 : 5;
  const int posShiftX         = SCALE_RATIO_BITS - numFracShift + compScale.first;
  const int posShiftY         = SCALE_RATIO_BITS - numFracShift + compScale.second;
  int addX  = (1 << (posShiftX - 1)) + (beforeScaleLeftOffset << SCALE_RATIO_BITS) + ((int( 1 - horCollocatedPositionFlag ) * 8 * (scalingRatio.first - SCALE_1X.first) + (1 << (2 + compScale.first))) >> (3 + compScale.first));
  int addY  = (1 << (posShiftY - 1)) + (beforeScaleTopOffset << SCALE_RATIO_BITS) + ((int( 1 - verCollocatedPositionFlag ) * 8 * (scalingRatio.second - SCALE_1X.second) + (1 << (2 + compScale.second))) >> (3 + compScale.second));

  const int filterLength = useLumaFilter ? NTAPS_LUMA : NTAPS_CHROMA;
  const int log2Norm     = 12;

  int* buf = new int[orgHeight * scaledWidth];
  int maxVal = (1 << bitDepth) - 1;

  CHECK( bitDepth > 17, "Overflow may happen!" );

  for( int i = 0; i < scaledWidth; i++ )
  {
    const Pel* org = orgSrc;
    int refPos = (((i << compScale.first) - afterScaleLeftOffset) * scalingRatio.first + addX) >> posShiftX;
    int integer = refPos >> numFracShift;
    int frac = refPos & numFracPositions;
    int* tmp = buf + i;

    for( int j = 0; j < orgHeight; j++ )
    {
      int sum = 0;
      const TFilterCoeff* f = filterHor + frac * filterLength;

      for( int k = 0; k < filterLength; k++ )
      {
        int xInt = std::min<int>( std::max( 0, integer + k - filterLength / 2 + 1 ), orgWidth - 1 );
        sum += f[k] * org[xInt]; // postpone horizontal filtering gain removal after vertical filtering
      }

      *tmp = sum;

      tmp += scaledWidth;
      org += orgStride;
    }
  }

  Pel* dst = scaledSrc;

  for( int j = 0; j < scaledHeight; j++ )
  {
    int refPos = (((j << compScale.second) - afterScaleTopOffset) * scalingRatio.second + addY) >> posShiftY;
    int integer = refPos >> numFracShift;
    int frac = refPos & numFracPositions;

    for( int i = 0; i < scaledWidth; i++ )
    {
      int sum = 0;
      int* tmp = buf + i;
      const TFilterCoeff* f = filterVer + frac * filterLength;

      for( int k = 0; k < filterLength; k++ )
      {
        int yInt = std::min<int>( std::max( 0, integer + k - filterLength / 2 + 1 ), orgHeight - 1 );
        sum += f[k] * tmp[yInt * scaledWidth];
      }

      dst[i] = std::min<int>( std::max( 0, (sum + (1 << (log2Norm - 1))) >> log2Norm ), maxVal );
    }

    dst += scaledStride;
  }

  delete[] buf;
}

void rspFwdCore( Pel* ptr, ptrdiff_t ptrStride, int width, int height, const int bd, const Pel OrgCW, const Pel* LmcsPivot, const Pel* ScaleCoeff, const Pel* InputPivot )
{
  int idxY;
  int shift = getLog2( OrgCW );

  //    const auto rsp_sgnl_op  = [=, &dst]( int ADDR ){ idxY = ( dst[ADDR] >> shift ); dst[ADDR] = static_cast<Pel>( ClipBD<int>( LmcsPivot[idxY] + ( ( ScaleCoeff[idxY] * ( dst[ADDR] - InputPivot[idxY] ) + ( 1 << 10 ) ) >> 11 ), bd ) ); };
  //    const auto rsp_sgnl_inc = [=, &dst]            { dst += stride; };

  //    size_aware_pel_op( rsp_sgnl_op, rsp_sgnl_inc, width, height );

#define RSP_FWD_OP( ADDR ) { idxY = ( ptr[ADDR] >> shift ); ptr[ADDR] = static_cast<Pel>( ClipBD<int>( LmcsPivot[idxY] + ( ( ScaleCoeff[idxY] * ( ptr[ADDR] - InputPivot[idxY] ) + ( 1 << 10 ) ) >> 11 ), bd ) ); }
#define RSP_FWD_INC        ptr      += ptrStride;

  SIZE_AWARE_PER_EL_OP( RSP_FWD_OP, RSP_FWD_INC )

#undef RSP_FWD_OP
#undef RSP_FWD_INC
}

PelBufferOps::PelBufferOps()
{
  addAvg4  = addAvgCore<Pel>;
  addAvg8  = addAvgCore<Pel>;
  addAvg16 = addAvgCore<Pel>;

  reco4 = reconstructCore<Pel>;
  reco8 = reconstructCore<Pel>;

  linTf4 = linTfCore<Pel>;
  linTf8 = linTfCore<Pel>;

  wghtAvg4 = addWeightedAvgCore<Pel>;
  wghtAvg8 = addWeightedAvgCore<Pel>;

  copyBuffer = copyBufferCore;

  transpose4x4 = transpose4x4Core<Pel>;
  transpose8x8 = transpose8x8Core<Pel>;

  applyLut = applyLutCore;
  rspFwd   = rspFwdCore;
  rspBcw   = nullptr;

  fillN_CU = fillN_CuCore;

  sampleRateConv = sampleRateConvCore;
}

PelBufferOps g_pelBufOP = PelBufferOps();

template<>
void AreaBuf<Pel>::addWeightedAvg(const AreaBuf<const Pel> &other1, const AreaBuf<const Pel> &other2, const ClpRng& clpRng, const int8_t bcwIdx)
{
  const int8_t w0 = getBcwWeight(bcwIdx, REF_PIC_LIST_0);
  const int8_t w1 = getBcwWeight(bcwIdx, REF_PIC_LIST_1);
  const int8_t log2WeightBase = g_BcwLog2WeightBase;
  const Pel* src0 = other1.buf;
  const Pel* src2 = other2.buf;
  Pel* dest = buf;

  const ptrdiff_t src1Stride = other1.stride;
  const ptrdiff_t src2Stride = other2.stride;
  const ptrdiff_t destStride = stride;
  const int clipbd    = clpRng.bd;
  const int shiftNum  = std::max<int>( 2, ( IF_INTERNAL_PREC - clipbd ) ) + log2WeightBase;
  const int offset    = ( 1 << ( shiftNum - 1 ) ) + ( IF_INTERNAL_OFFS << log2WeightBase );

  if( ( width & 7 ) == 0 )
  {
    g_pelBufOP.wghtAvg8( src0, src1Stride, src2, src2Stride, dest, destStride, width, height, shiftNum, offset, w0, w1, clpRng );
  }
  else if( ( width & 3 ) == 0 )
  {
    g_pelBufOP.wghtAvg4( src0, src1Stride, src2, src2Stride, dest, destStride, width, height, shiftNum, offset, w0, w1, clpRng );
  }
  else
  {
#define ADD_AVG_OP( ADDR ) dest[ADDR] = ClipPel( rightShift( ( src0[ADDR]*w0 + src2[ADDR]*w1 + offset ), shiftNum ), clpRng )
#define ADD_AVG_INC     \
    src0 += src1Stride; \
    src2 += src2Stride; \
    dest += destStride; \

    SIZE_AWARE_PER_EL_OP( ADD_AVG_OP, ADD_AVG_INC );

#undef ADD_AVG_OP
#undef ADD_AVG_INC
  }
}

template<>
void AreaBuf<Pel>::scaleSignal(const int scale, const ClpRng& clpRng)
{
  Pel* dst = buf;
  Pel* src = buf;
  int sign, absval;
  int maxAbsclipBD = ( 1 << clpRng.bd ) - 1;

  for (unsigned y = 0; y < height; y++)
  {
    for (unsigned x = 0; x < width; x++)
    {
      src[x] = Clip3<Pel>( -maxAbsclipBD - 1, maxAbsclipBD, src[x] );
      sign   = src[x] >= 0 ? 1 : -1;
      absval = sign * src[x];

      int val = sign * ((absval * scale + (1 << (CSCALE_FP_PREC - 1))) >> CSCALE_FP_PREC);

      if( sizeof( Pel ) == 2 ) // avoid overflow when storing data
      {
          val = Clip3<int>(-32768, 32767, val);
      }
      dst[x] = (Pel)val;
    }
    dst += stride;
    src += stride;
  }
}

template<>
void AreaBuf<Pel>::addAvg( const AreaBuf<const Pel> &other1, const AreaBuf<const Pel> &other2, const ClpRng& clpRng)
{
  const Pel* src0 = other1.buf;
  const Pel* src2 = other2.buf;
        Pel* dest =        buf;

  const ptrdiff_t src1Stride = other1.stride;
  const ptrdiff_t src2Stride = other2.stride;
  const ptrdiff_t destStride =        stride;
  const int       clipbd     = clpRng.bd;
  const int       shiftNum   = std::max<int>(2, (IF_INTERNAL_PREC - clipbd)) + 1;
  const int       offset     = (1 << (shiftNum - 1)) + 2 * IF_INTERNAL_OFFS;

  if( ( width & 15 ) == 0 )
  {
    g_pelBufOP.addAvg16( src0, src1Stride, src2, src2Stride, dest, destStride, width, height, shiftNum, offset, clpRng );
  }
  else if( ( width & 7 ) == 0 )
  {
    g_pelBufOP.addAvg8( src0, src1Stride, src2, src2Stride, dest, destStride, width, height, shiftNum, offset, clpRng );
  }
  else if( ( width & 3 ) == 0 )
  {
    g_pelBufOP.addAvg4( src0, src1Stride, src2, src2Stride, dest, destStride, width, height, shiftNum, offset, clpRng );
  }
  else
  {
#define ADD_AVG_OP( ADDR ) dest[ADDR] = ClipPel( rightShift( ( src0[ADDR] + src2[ADDR] + offset ), shiftNum ), clpRng )
#define ADD_AVG_INC     \
    src0 += src1Stride; \
    src2 += src2Stride; \
    dest += destStride; \

    SIZE_AWARE_PER_EL_OP( ADD_AVG_OP, ADD_AVG_INC );

#undef ADD_AVG_OP
#undef ADD_AVG_INC
  }
}

template<>
void AreaBuf<Pel>::reconstruct( const AreaBuf<const Pel> &pred, const AreaBuf<const Pel> &resi, const ClpRng& clpRng )
{
  const Pel* src1 = pred.buf;
  const Pel* src2 = resi.buf;
        Pel* dest =      buf;

  const ptrdiff_t src1Stride = pred.stride;
  const ptrdiff_t src2Stride = resi.stride;
  const ptrdiff_t destStride =      stride;

  if( ( width & 7 ) == 0 )
  {
    g_pelBufOP.reco8( src1, src1Stride, src2, src2Stride, dest, destStride, width, height, clpRng );
  }
  else if( ( width & 3 ) == 0 )
  {
    g_pelBufOP.reco4( src1, src1Stride, src2, src2Stride, dest, destStride, width, height, clpRng );
  }
  else
  {
#define RECO_OP( ADDR ) dest[ADDR] = ClipPel( src1[ADDR] + src2[ADDR], clpRng )
#define RECO_INC        \
    src1 += src1Stride; \
    src2 += src2Stride; \
    dest += destStride; \

    SIZE_AWARE_PER_EL_OP( RECO_OP, RECO_INC );

#undef RECO_OP
#undef RECO_INC
  }
}

template<>
void AreaBuf<Pel>::linearTransform( const int scale, const int shift, const int offset, bool bClip, const ClpRng& clpRng )
{
  const Pel* src = buf;
        Pel* dst = buf;

  if( width == 1 )
  {
    THROW_FATAL( "Blocks of width = 1 not supported" );
  }
  else if( ( width & 7 ) == 0 )
  {
    g_pelBufOP.linTf8( src, stride, dst, stride, width, height, scale, shift, offset, clpRng, bClip );
  }
  else if( ( width & 3 ) == 0 )
  {
    g_pelBufOP.linTf4( src, stride, dst, stride, width, height, scale, shift, offset, clpRng, bClip );
  }
  else
  {
#define LINTF_OP( ADDR ) dst[ADDR] = ( Pel ) bClip ? ClipPel( rightShift( scale * src[ADDR], shift ) + offset, clpRng ) : ( rightShift( scale * src[ADDR], shift ) + offset )
#define LINTF_INC        \
    src += stride;       \
    dst += stride;       \

    SIZE_AWARE_PER_EL_OP( LINTF_OP, LINTF_INC );

#undef RECO_OP
#undef RECO_INC
  }
}

#if ENABLE_SIMD_OPT_BUFFER && defined(TARGET_SIMD_X86)
template<>
void AreaBuf<Pel>::transposedFrom( const AreaBuf<const Pel> &other )
{
  CHECK( width != other.height || height != other.width, "Incompatible size" );

  if( ( ( width | height ) & 7 ) == 0 )
  {
    const Pel* src = other.buf;

    for( unsigned y = 0; y < other.height; y += 8 )
    {
      Pel* dst = buf + y;

      for( unsigned x = 0; x < other.width; x += 8 )
      {
        g_pelBufOP.transpose8x8( &src[x], other.stride, dst, stride );

        dst += 8 * stride;
      }

      src += 8 * other.stride;
    }
  }
  else if( ( ( width | height ) & 3 ) == 0 )
  {
    const Pel* src = other.buf;

    for( unsigned y = 0; y < other.height; y += 4 )
    {
      Pel* dst = buf + y;

      for( unsigned x = 0; x < other.width; x += 4 )
      {
        g_pelBufOP.transpose4x4( &src[x], other.stride, dst, stride );

        dst += 4 * stride;
      }

      src += 4 * other.stride;
    }
  }
  else
  {
          Pel* dst =       buf;
    const Pel* src = other.buf;
    width          = other.height;
    height         = other.width;
    stride         = stride < width ? width : stride;

    for( unsigned y = 0; y < other.height; y++ )
    {
      for( unsigned x = 0; x < other.width; x++ )
      {
        dst[y + x*stride] = src[x + y * other.stride];
      }
    }
  }
}
#endif

template<>
void AreaBuf<MotionInfo>::fill( const MotionInfo& val )
{
  if( width == stride )
  {
    std::fill_n( buf, width * height, val );
  }
  else
  {
    MotionInfo* dst = buf;

    for( int y = 0; y < height; y++, dst += stride )
    {
      std::fill_n( dst, width, val );
    }
  }
}

PelStorage::PelStorage()
{
  for( uint32_t i = 0; i < MAX_NUM_COMPONENT; i++ )
  {
    m_origin[i]        = nullptr;
    m_allocator[i]     = nullptr;
  }
}

PelStorage::~PelStorage()
{
  destroy();
}

void PelStorage::create( const UnitArea &_UnitArea )
{
  create( _UnitArea.chromaFormat, _UnitArea.blocks[0] );
}

void PelStorage::create( const ChromaFormat _chromaFormat, const Size& _size, const unsigned _maxCUSize, const unsigned _margin, const unsigned _alignmentByte, const bool _scaleChromaMargin, const UserAllocator* userAlloc )
{
  CHECK( !bufs.empty(), "Trying to re-create an already initialized buffer" );

  chromaFormat = _chromaFormat;

  const uint32_t numCh = getNumberValidComponents( _chromaFormat );

  unsigned extHeight = _size.height;
  unsigned extWidth  = _size.width;

  if( _maxCUSize )
  {
    extHeight = ( ( _size.height + _maxCUSize - 1 ) / _maxCUSize ) * _maxCUSize;
    extWidth  = ( ( _size.width  + _maxCUSize - 1 ) / _maxCUSize ) * _maxCUSize;
  }

  const unsigned _alignment = _alignmentByte / sizeof( Pel );

  for( uint32_t i = 0; i < numCh; i++ )
  {
    const ComponentID compID = ComponentID( i );
    const unsigned scaleX = getComponentScaleX( compID, _chromaFormat );
    const unsigned scaleY = getComponentScaleY( compID, _chromaFormat );

    unsigned scaledHeight = extHeight >> scaleY;
    unsigned scaledWidth  = extWidth  >> scaleX;
    unsigned ymargin      = _margin >> (_scaleChromaMargin?scaleY:0);
    unsigned xmargin      = _margin >> (_scaleChromaMargin?scaleX:0);

#if 1
    if( _alignment && xmargin )
    {
      xmargin = ( ( xmargin + _alignment - 1 ) / _alignment ) * _alignment;
    }

#endif
    SizeType totalWidth   = scaledWidth + 2 * xmargin;
    SizeType totalHeight  = scaledHeight +2 * ymargin;

    if( _alignment )
    {
      // make sure buffer lines are align
      CHECK( _alignmentByte != MEMORY_ALIGN_DEF_SIZE, "Unsupported alignment" );
      totalWidth = ( ( totalWidth + _alignment - 1 ) / _alignment ) * _alignment;
    }

#if ENABLE_SIMD_OPT_INTER
    uint32_t area = totalWidth * totalHeight + 1; // +1 for the extra Pel overread in prefetchPad_SSE, in case reading from the very bottom right of the picture
#else
    uint32_t area = totalWidth * totalHeight;
#endif
    CHECK( !area, "Trying to create a buffer with zero area" );

    m_origSi[i] = Size{ totalWidth, totalHeight };
    if( userAlloc && userAlloc->enabled )
    {
      m_origin[i] = ( Pel* ) userAlloc->create( userAlloc->opaque, (vvdecComponentType)i, sizeof(Pel)*area, MEMORY_ALIGN_DEF_SIZE, &m_allocator[i] );
      CHECK( m_origin[i] == nullptr, "external allocator callback failed (returned NULL)." );
      m_externAllocator = true;
      m_userAlloc       = userAlloc;
    }
    else
    {
      m_origin[i] = ( Pel* ) xMalloc( Pel, area );
    }
    Pel* topLeft = m_origin[i] + totalWidth * ymargin + xmargin;
    bufs.push_back( PelBuf( topLeft, totalWidth, _size.width >> scaleX, _size.height >> scaleY ) );
  }
}

void PelStorage::createFromBuf( PelUnitBuf buf )
{
  chromaFormat = buf.chromaFormat;

  const uint32_t numCh = getNumberValidComponents( chromaFormat );

  bufs.resize(numCh);

  for( uint32_t i = 0; i < numCh; i++ )
  {
    PelBuf cPelBuf = buf.get( ComponentID( i ) );
    bufs[i] = PelBuf( cPelBuf.bufAt( 0, 0 ), cPelBuf.stride, cPelBuf.width, cPelBuf.height );
  }
}

void PelStorage::swap( PelStorage& other )
{
  const uint32_t numCh = getNumberValidComponents( chromaFormat );

  for( uint32_t i = 0; i < numCh; i++ )
  {
    // check this otherwise it would turn out to get very weird
    CHECK( chromaFormat                   != other.chromaFormat                  , "Incompatible formats" );
    CHECK( get( ComponentID( i ) )        != other.get( ComponentID( i ) )       , "Incompatible formats" );
    CHECK( get( ComponentID( i ) ).stride != other.get( ComponentID( i ) ).stride, "Incompatible formats" );

    std::swap( bufs[i].buf,    other.bufs[i].buf );
    std::swap( bufs[i].stride, other.bufs[i].stride );
    std::swap( m_origin[i],    other.m_origin[i] );
    std::swap( m_allocator[i], other.m_allocator[i] );
  }
  std::swap( m_externAllocator, other.m_externAllocator );
  std::swap( m_userAlloc,       other.m_userAlloc );
}

void PelStorage::destroy()
{
  chromaFormat = NUM_CHROMA_FORMAT;
  for( uint32_t i = 0; i < MAX_NUM_COMPONENT; i++ )
  {
    if( m_origin[i] )
    {
      if ( !m_externAllocator )
      {
        xFree( m_origin[i] );
      }
      else if( m_allocator[i])
      {
        CHECK( m_userAlloc->unref == nullptr, "vvdecUnrefBufferCallback not valid, cannot unref picture buffer" )
        m_userAlloc->unref( m_userAlloc->opaque, m_allocator[i] );
      }
      m_origin[i] = nullptr;
    }
  }
  bufs.clear();
}

PelBuf PelStorage::getBuf( const ComponentID CompID )
{
  return bufs[CompID];
}

const CPelBuf PelStorage::getBuf( const ComponentID CompID ) const
{
  return bufs[CompID];
}

PelBuf PelStorage::getBuf( const CompArea &blk )
{
  const PelBuf& r = bufs[blk.compID()];

  CHECKD( rsAddr( blk.bottomRight(), r.stride ) >= ( ( r.height - 1 ) * r.stride + r.width ), "Trying to access a buf outside of bound!" );

  return PelBuf( r.buf + rsAddr( blk, r.stride ), r.stride, blk );
}

const CPelBuf PelStorage::getBuf( const CompArea &blk ) const
{
  const PelBuf& r = bufs[blk.compID()];
  return CPelBuf( r.buf + rsAddr( blk, r.stride ), r.stride, blk );
}

PelUnitBuf PelStorage::getBuf( const UnitArea &unit )
{
  return ( chromaFormat == CHROMA_400 ) ? PelUnitBuf( chromaFormat, getBuf( unit.Y() ) ) : PelUnitBuf( chromaFormat, getBuf( unit.Y() ), getBuf( unit.Cb() ), getBuf( unit.Cr() ) );
}

const CPelUnitBuf PelStorage::getBuf( const UnitArea &unit ) const
{
  return ( chromaFormat == CHROMA_400 ) ? CPelUnitBuf( chromaFormat, getBuf( unit.Y() ) ) : CPelUnitBuf( chromaFormat, getBuf( unit.Y() ), getBuf( unit.Cb() ), getBuf( unit.Cr() ) );
}

template<>
void UnitBuf<Pel>::colorSpaceConvert( const UnitBuf<Pel> &other, const ClpRng& clpRng )
{
  const Pel* pOrg0 = bufs[COMPONENT_Y ].buf;
  const Pel* pOrg1 = bufs[COMPONENT_Cb].buf;
  const Pel* pOrg2 = bufs[COMPONENT_Cr].buf;
  const ptrdiff_t strideOrg = bufs[COMPONENT_Y ].stride;

  Pel* pDst0 = other.bufs[COMPONENT_Y ].buf;
  Pel* pDst1 = other.bufs[COMPONENT_Cb].buf;
  Pel* pDst2 = other.bufs[COMPONENT_Cr].buf;
  const ptrdiff_t strideDst = other.bufs[COMPONENT_Y ].stride;

  int width  = bufs[COMPONENT_Y].width;
  int height = bufs[COMPONENT_Y].height;
  int maxAbsclipBD = (1 << (clpRng.bd + 1)) - 1;
  int y0, cg, co;

  CHECKD( bufs[COMPONENT_Y].stride != bufs[COMPONENT_Cb].stride || bufs[COMPONENT_Y].stride != bufs[COMPONENT_Cr].stride, "unequal stride for 444 content" );
  CHECKD( other.bufs[COMPONENT_Y].stride != other.bufs[COMPONENT_Cb].stride || other.bufs[COMPONENT_Y].stride != other.bufs[COMPONENT_Cr].stride, "unequal stride for 444 content" );
  CHECKD( bufs[COMPONENT_Y].width != other.bufs[COMPONENT_Y].width || bufs[COMPONENT_Y].height != other.bufs[COMPONENT_Y].height, "unequal block size" );

  for( int y = 0; y < height; y++ )
  {
    for( int x = 0; x < width; x++ )
    {
      y0 = pOrg0[x];
      cg = pOrg1[x];
      co = pOrg2[x];

      y0 = Clip3((-maxAbsclipBD - 1), maxAbsclipBD, y0);
      cg = Clip3((-maxAbsclipBD - 1), maxAbsclipBD, cg);
      co = Clip3((-maxAbsclipBD - 1), maxAbsclipBD, co);

      int t = y0 - (cg >> 1);
      pDst0[x] = cg + t;
      pDst1[x] = t - (co >> 1);
      pDst2[x] = co + pDst1[x];
    }

    pOrg0 += strideOrg;
    pOrg1 += strideOrg;
    pOrg2 += strideOrg;
    pDst0 += strideDst;
    pDst1 += strideDst;
    pDst2 += strideDst;
  }
}

template void UnitBuf<Pel>::writeToFile( std::string filename ) const;

template<typename T>
void UnitBuf<T>::writeToFile( std::string filename ) const
{
  FILE* f = fopen( filename.c_str(), "w" );
  CHECK_FATAL( f == nullptr, "writeToFile() cannot open file for writing" )

  for( auto& b: bufs )
  {
    for( unsigned y = 0; y < b.height; y++ )
    {
      fwrite( b.bufAt( 0, y ), sizeof( T ), b.width, f );
    }
  }

  fclose( f );
}

}
