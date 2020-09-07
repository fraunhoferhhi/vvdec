/* -----------------------------------------------------------------------------
Software Copyright License for the Fraunhofer Software Library VVdec

(c) Copyright (2018-2020) Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. 

1.    INTRODUCTION

The Fraunhofer Software Library VVdec (“Fraunhofer Versatile Video Decoding Library”) is software that implements (parts of) the Versatile Video Coding Standard - ITU-T H.266 | MPEG-I - Part 3 (ISO/IEC 23090-3) and related technology. 
The standard contains Fraunhofer patents as well as third-party patents. Patent licenses from third party standard patent right holders may be required for using the Fraunhofer Versatile Video Decoding Library. It is in your responsibility to obtain those if necessary. 

The Fraunhofer Versatile Video Decoding Library which mean any source code provided by Fraunhofer are made available under this software copyright license. 
It is based on the official ITU/ISO/IEC VVC Test Model (VTM) reference software whose copyright holders are indicated in the copyright notices of its source files. The VVC Test Model (VTM) reference software is licensed under the 3-Clause BSD License and therefore not subject of this software copyright license.

2.    COPYRIGHT LICENSE

Internal use of the Fraunhofer Versatile Video Decoding Library, in source and binary forms, with or without modification, is permitted without payment of copyright license fees for non-commercial purposes of evaluation, testing and academic research. 

No right or license, express or implied, is granted to any part of the Fraunhofer Versatile Video Decoding Library except and solely to the extent as expressly set forth herein. Any commercial use or exploitation of the Fraunhofer Versatile Video Decoding Library and/or any modifications thereto under this license are prohibited.

For any other use of the Fraunhofer Versatile Video Decoding Library than permitted by this software copyright license You need another license from Fraunhofer. In such case please contact Fraunhofer under the CONTACT INFORMATION below.

3.    LIMITED PATENT LICENSE

As mentioned under 1. Fraunhofer patents are implemented by the Fraunhofer Versatile Video Decoding Library. If You use the Fraunhofer Versatile Video Decoding Library in Germany, the use of those Fraunhofer patents for purposes of testing, evaluating and research and development is permitted within the statutory limitations of German patent law. However, if You use the Fraunhofer Versatile Video Decoding Library in a country where the use for research and development purposes is not permitted without a license, you must obtain an appropriate license from Fraunhofer. It is Your responsibility to check the legal requirements for any use of applicable patents.    

Fraunhofer provides no warranty of patent non-infringement with respect to the Fraunhofer Versatile Video Decoding Library.


4.    DISCLAIMER

The Fraunhofer Versatile Video Decoding Library is provided by Fraunhofer "AS IS" and WITHOUT ANY EXPRESS OR IMPLIED WARRANTIES, including but not limited to the implied warranties fitness for a particular purpose. IN NO EVENT SHALL FRAUNHOFER BE LIABLE for any direct, indirect, incidental, special, exemplary, or consequential damages, including but not limited to procurement of substitute goods or services; loss of use, data, or profits, or business interruption, however caused and on any theory of liability, whether in contract, strict liability, or tort (including negligence), arising in any way out of the use of the Fraunhofer Versatile Video Decoding Library, even if advised of the possibility of such damage.

5.    CONTACT INFORMATION

Fraunhofer Heinrich Hertz Institute
Attention: Video Coding & Analytics Department
Einsteinufer 37
10587 Berlin, Germany
www.hhi.fraunhofer.de/vvc
vvc@hhi.fraunhofer.de
------------------------------------------------------------------------------------------- */

/** \file     Buffer.cpp
 *  \brief    Low-overhead class describing 2D memory layout
 */

#define DONT_UNDEF_SIZE_AWARE_PER_EL_OP

// unit needs to come first due to a forward declaration
#include "Unit.h"
#include "Buffer.h"
#include "InterpolationFilter.h"

#include "CommonDefX86.h"

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
  _mm_prefetch( (const char *) ( src ),             _MM_HINT_T0 );
  _mm_prefetch( (const char *) ( src + srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char *) ( dst ),             _MM_HINT_T0 );
  _mm_prefetch( (const char *) ( dst + dstStride ), _MM_HINT_T0 );

  if( width == srcStride && width == dstStride )
  {
    memcpy( dst, src, width * height );
  }

  for( int i = 0; i < height; i++ )
  {
    _mm_prefetch( (const char *) ( src + srcStride ), _MM_HINT_T0 );
    _mm_prefetch( (const char *) ( dst + dstStride ), _MM_HINT_T0 );

    memcpy( dst, src, width );

    src += srcStride;
    dst += dstStride;
  }
}

template<int padSize>
void paddingCore(Pel *ptr, ptrdiff_t stride, int width, int height)
{
  /*left and right padding*/
  Pel *ptrTemp1 = ptr;
  Pel *ptrTemp2 = ptr + (width - 1);
  ptrdiff_t offset = 0;
  for (int i = 0; i < height; i++)
  {
    offset = stride * i;
    for (int j = 1; j <= padSize; j++)
    {
      *(ptrTemp1 - j + offset) = *(ptrTemp1 + offset);
      *(ptrTemp2 + j + offset) = *(ptrTemp2 + offset);
    }
  }
  /*Top and Bottom padding*/
  int numBytes = (width + padSize + padSize) * sizeof(Pel);
  ptrTemp1 = (ptr - padSize);
  ptrTemp2 = (ptr + (stride * (height - 1)) - padSize);
  for (int i = 1; i <= padSize; i++)
  {
    memcpy(ptrTemp1 - (i * stride), (ptrTemp1), numBytes);
    memcpy(ptrTemp2 + (i * stride), (ptrTemp2), numBytes);
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
  padding2 = paddingCore<2>;
  padding1 = paddingCore<1>;

  transpose4x4 = transpose4x4Core<Pel>;
  transpose8x8 = transpose8x8Core<Pel>;

  applyLut = applyLutCore;

  fillN_CU = fillN_CuCore;
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
void AreaBuf<Pel>::rspSignal( const Pel* lut )
{
  g_pelBufOP.applyLut( buf, stride, width, height, lut );
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
    THROW( "Blocks of width = 1 not supported" );
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

  if( ( width & 3 ) != 0 || ( height & 3 ) != 0 )
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
  else if( ( width & 7 ) != 0 || ( height & 7 ) != 0 )
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
    m_origin[i] = nullptr;
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

void PelStorage::create( const ChromaFormat _chromaFormat, const Size& _size, const unsigned _maxCUSize, const unsigned _margin, const unsigned _alignment, const bool _scaleChromaMargin )
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

  for( uint32_t i = 0; i < numCh; i++ )
  {
    const ComponentID compID = ComponentID( i );
    const unsigned scaleX = ::getComponentScaleX( compID, _chromaFormat );
    const unsigned scaleY = ::getComponentScaleY( compID, _chromaFormat );

    unsigned scaledHeight = extHeight >> scaleY;
    unsigned scaledWidth  = extWidth  >> scaleX;
    unsigned ymargin      = _margin >> (_scaleChromaMargin?scaleY:0);
    unsigned xmargin      = _margin >> (_scaleChromaMargin?scaleX:0);
    unsigned totalWidth   = scaledWidth + 2*xmargin;
    unsigned totalHeight  = scaledHeight +2*ymargin;

    if( _alignment )
    {
      // make sure buffer lines are align
      CHECK( _alignment != MEMORY_ALIGN_DEF_SIZE, "Unsupported alignment" );
      totalWidth = ( ( totalWidth + _alignment - 1 ) / _alignment ) * _alignment;
    }
    uint32_t area = totalWidth * totalHeight;
    CHECK( !area, "Trying to create a buffer with zero area" );

    m_origSi[i] = Size{ totalWidth, totalHeight };
    m_origin[i] = ( Pel* ) xMalloc( Pel, area );
    Pel* topLeft = m_origin[i] + totalWidth * ymargin + xmargin;
    bufs.push_back( PelBuf( topLeft, totalWidth, _size.width >> scaleX, _size.height >> scaleY ) );
  }
}

void PelStorage::createFromBuf( PelUnitBuf buf )
{
  chromaFormat = buf.chromaFormat;

  const uint32_t numCh = ::getNumberValidComponents( chromaFormat );

  bufs.resize(numCh);

  for( uint32_t i = 0; i < numCh; i++ )
  {
    PelBuf cPelBuf = buf.get( ComponentID( i ) );
    bufs[i] = PelBuf( cPelBuf.bufAt( 0, 0 ), cPelBuf.stride, cPelBuf.width, cPelBuf.height );
  }
}

void PelStorage::swap( PelStorage& other )
{
  const uint32_t numCh = ::getNumberValidComponents( chromaFormat );

  for( uint32_t i = 0; i < numCh; i++ )
  {
    // check this otherwise it would turn out to get very weird
    CHECK( chromaFormat                   != other.chromaFormat                  , "Incompatible formats" );
    CHECK( get( ComponentID( i ) )        != other.get( ComponentID( i ) )       , "Incompatible formats" );
    CHECK( get( ComponentID( i ) ).stride != other.get( ComponentID( i ) ).stride, "Incompatible formats" );

    std::swap( bufs[i].buf,    other.bufs[i].buf );
    std::swap( bufs[i].stride, other.bufs[i].stride );
    std::swap( m_origin[i],    other.m_origin[i] );
  }
}

void PelStorage::destroy()
{
  chromaFormat = NUM_CHROMA_FORMAT;
  for( uint32_t i = 0; i < MAX_NUM_COMPONENT; i++ )
  {
    if( m_origin[i] )
    {
      xFree( m_origin[i] );
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
  const PelBuf& r = bufs[blk.compID];

  CHECKD( rsAddr( blk.bottomRight(), r.stride ) >= ( ( r.height - 1 ) * r.stride + r.width ), "Trying to access a buf outside of bound!" );

  return PelBuf( r.buf + rsAddr( blk, r.stride ), r.stride, blk );
}

const CPelBuf PelStorage::getBuf( const CompArea &blk ) const
{
  const PelBuf& r = bufs[blk.compID];
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
