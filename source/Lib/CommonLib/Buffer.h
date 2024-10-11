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

/** \file     Buffer.h
 *  \brief    Low-overhead class describing 2D memory layout
 */

#pragma once

#include "Common.h"
#include "CommonDef.h"
#include "ChromaFormat.h"
#include "MotionInfo.h"

#include <string.h>
#include <type_traits>
#include <utility>

namespace vvdec
{

struct CodingUnit;

using namespace x86_simd;
using namespace arm_simd;

struct PelBufferOps
{
  PelBufferOps();

#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_OPT_BUFFER
  void initPelBufOpsX86();
  template<X86_VEXT vext>
  void _initPelBufOpsX86();
#endif

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_BUFFER
  void initPelBufOpsARM();
  template<ARM_VEXT vext>
  void _initPelBufOpsARM();
#endif

  void ( *addAvg4 )       ( const Pel* src0, ptrdiff_t src0Stride, const Pel* src1, ptrdiff_t src1Stride, Pel *dst, ptrdiff_t dstStride, int width, int height,            int shift, int offset,      const ClpRng& clpRng );
  void ( *addAvg8 )       ( const Pel* src0, ptrdiff_t src0Stride, const Pel* src1, ptrdiff_t src1Stride, Pel *dst, ptrdiff_t dstStride, int width, int height,            int shift, int offset,      const ClpRng& clpRng );
  void ( *addAvg16 )      ( const Pel* src0, ptrdiff_t src0Stride, const Pel* src1, ptrdiff_t src1Stride, Pel *dst, ptrdiff_t dstStride, int width, int height,            int shift, int offset,      const ClpRng& clpRng );
  void ( *reco4 )         ( const Pel* src0, ptrdiff_t src0Stride, const Pel* src1, ptrdiff_t src1Stride, Pel *dst, ptrdiff_t dstStride, int width, int height,                                        const ClpRng& clpRng );
  void ( *reco8 )         ( const Pel* src0, ptrdiff_t src0Stride, const Pel* src1, ptrdiff_t src1Stride, Pel *dst, ptrdiff_t dstStride, int width, int height,                                        const ClpRng& clpRng );
  void ( *linTf4 )        ( const Pel* src0, ptrdiff_t src0Stride,                                        Pel *dst, ptrdiff_t dstStride, int width, int height, int scale, int shift, int offset,      const ClpRng& clpRng, bool bClip );
  void ( *linTf8 )        ( const Pel* src0, ptrdiff_t src0Stride,                                        Pel *dst, ptrdiff_t dstStride, int width, int height, int scale, int shift, int offset,      const ClpRng& clpRng, bool bClip );
  void ( *wghtAvg4 )      ( const Pel* src0, ptrdiff_t src0Stride, const Pel* src1, ptrdiff_t src1Stride, Pel *dst, ptrdiff_t dstStride, int width, int height, int shift, int offset, int w0, int w1, const ClpRng& clpRng );
  void ( *wghtAvg8 )      ( const Pel* src0, ptrdiff_t src0Stride, const Pel* src1, ptrdiff_t src1Stride, Pel *dst, ptrdiff_t dstStride, int width, int height, int shift, int offset, int w0, int w1, const ClpRng& clpRng );
  void ( *copyBuffer  )    ( const char*src,  ptrdiff_t srcStride,        char* dst, ptrdiff_t  dstStride,                                int width, int height );
  void ( *transpose4x4 )  ( const Pel* src,  ptrdiff_t srcStride, Pel* dst, ptrdiff_t dstStride );
  void ( *transpose8x8 )  ( const Pel* src,  ptrdiff_t srcStride, Pel* dst, ptrdiff_t dstStride );
  void ( *applyLut )      (       Pel* ptr,  ptrdiff_t ptrStride, int width, int height, const Pel* lut );
  void ( *rspFwd )        (       Pel* ptr,  ptrdiff_t ptrStride, int width, int height, const int bd,                   const Pel OrgCW,  const Pel* LmcsPivot, const Pel* ScaleCoeff, const Pel* InputPivot );
  void ( *rspBcw )        (       Pel* ptr,  ptrdiff_t ptrStride, int width, int height, const int bd, const int minBin, const int maxBin, const Pel* LmcsPivot, const Pel* InvScCoeff, const Pel* InputPivot );
  void ( *fillN_CU )      (       CodingUnit** ptr, ptrdiff_t ptrStride, int width, int height, CodingUnit* cuPtr );

  void (*sampleRateConv)  ( const std::pair<int, int> scalingRatio, const std::pair<int, int> compScale,
                            const Pel* orgSrc, const ptrdiff_t orgStride, const int orgWidth, const int orgHeight,
                            const int beforeScaleLeftOffset, const int beforeScaleTopOffset,
                            Pel* scaledSrc, const ptrdiff_t scaledStride, const int scaledWidth, const int scaledHeight,
                            const int afterScaleLeftOffset, const int afterScaleTopOffset,
                            const int bitDepth, const bool useLumaFilter, const bool horCollocatedPositionFlag, const bool verCollocatedPositionFlag );
};

extern PelBufferOps g_pelBufOP;

void sampleRateConvCore( const std::pair<int, int> scalingRatio, const std::pair<int, int> compScale,
                         const Pel* orgSrc, const ptrdiff_t orgStride, const int orgWidth, const int orgHeight,
                         const int beforeScaleLeftOffset, const int beforeScaleTopOffset,
                         Pel* scaledSrc, const ptrdiff_t scaledStride, const int scaledWidth, const int scaledHeight,
                         const int afterScaleLeftOffset, const int afterScaleTopOffset,
                         const int bitDepth, const bool useLumaFilter,
                         const bool horCollocatedPositionFlag, const bool verCollocatedPositionFlag );

#define INCX( ptr, stride ) { ptr++; }
#define INCY( ptr, stride ) { ptr += ( stride ); }
#define OFFSETX( ptr, stride, x ) { ptr += ( x ); }
#define OFFSETY( ptr, stride, y ) { ptr += ( y ) * ( stride ); }
#define OFFSET( ptr, stride, x, y ) { ptr += ( x ) + ( y ) * ( stride ); }
#define GET_OFFSETX( ptr, stride, x ) ( ( ptr ) + ( x ) )
#define GET_OFFSETY( ptr, stride, y ) ( ( ptr ) + ( y ) * ( stride ) )
#define GET_OFFSET( ptr, stride, x, y ) ( ( ptr ) + ( x ) + ( y ) * ( stride ) )

class Window;
struct BitDepths;

template<typename T>
struct AreaBuf : public Size
{
  T*        buf;
  ptrdiff_t stride;

  AreaBuf()                                                                                     : Size(),                  buf( NULL ), stride( 0 )          { }
  AreaBuf( T *_buf, const Size &size )                                                          : Size( size ),            buf( _buf ), stride( size.width ) { }
  AreaBuf( T *_buf, const ptrdiff_t &_stride, const Size &size )                                : Size( size ),            buf( _buf ), stride( _stride )    { }
  AreaBuf( T *_buf, const SizeType &_width, const SizeType &_height )                           : Size( _width, _height ), buf( _buf ), stride( _width )     { }
  AreaBuf( T *_buf, const ptrdiff_t &_stride, const SizeType &_width, const SizeType &_height ) : Size( _width, _height ), buf( _buf ), stride( _stride )    { }

  AreaBuf( const AreaBuf& )  = default;
  AreaBuf(       AreaBuf&& ) = default;
  AreaBuf& operator=( const AreaBuf& )  = default;
  AreaBuf& operator=(       AreaBuf&& ) = default;

  // conversion from AreaBuf<const T> to AreaBuf<T>
  template<bool T_IS_CONST = std::is_const<T>::value>
  AreaBuf( const AreaBuf<typename std::remove_const_t<T>>& other, std::enable_if_t<T_IS_CONST>* = 0) : Size( other ), buf( other.buf ), stride( other.stride ) { }

  void fill                 ( const T &val );
  void memset               ( const int val );

  void copyFrom             ( const AreaBuf<const T> &other );
  
  void reconstruct          ( const AreaBuf<const T> &pred, const AreaBuf<const T> &resi, const ClpRng& clpRng);
  
  void subtract             ( const AreaBuf<const T> &other );
  void extendBorderPel      ( unsigned margin );
  void extendBorderPel      ( unsigned margin, bool left, bool right, bool top, bool bottom );
  void addWeightedAvg       ( const AreaBuf<const T> &other1, const AreaBuf<const T> &other2, const ClpRng& clpRng, const int8_t bcwIdx);
  void addAvg               ( const AreaBuf<const T> &other1, const AreaBuf<const T> &other2, const ClpRng& clpRng );
  void padBorderPel         ( unsigned marginX, unsigned marginY, int dir );
  
  void linearTransform      ( const int scale, const int shift, const int offset, bool bClip, const ClpRng& clpRng );

  void transposedFrom       ( const AreaBuf<const T> &other );

  void scaleSignal          ( const int scale, const ClpRng& clpRng );

        T& at( const int &x, const int &y )          { return buf[y * stride + x]; }
  const T& at( const int &x, const int &y ) const    { return buf[y * stride + x]; }

        T& at( const Position &pos )                 { return buf[pos.y * stride + pos.x]; }
  const T& at( const Position &pos ) const           { return buf[pos.y * stride + pos.x]; }


        T* bufAt( const int &x, const int &y )       { return GET_OFFSET( buf, stride,     x,     y ); }
  const T* bufAt( const int &x, const int &y ) const { return GET_OFFSET( buf, stride,     x,     y ); }
        T* bufAt( const Position& pos )              { return GET_OFFSET( buf, stride, pos.x, pos.y ); }
  const T* bufAt( const Position& pos ) const        { return GET_OFFSET( buf, stride, pos.x, pos.y ); }

  AreaBuf<      T> subBuf( const Area &area )                                                         { return subBuf( area.pos(), area.size() ); }
  AreaBuf<const T> subBuf( const Area &area )                                                   const { return subBuf( area.pos(), area.size() ); }
  AreaBuf<      T> subBuf( const Position &pos, const Size &size )                                    { return AreaBuf<      T>( bufAt( pos  ), stride, size   ); }
  AreaBuf<const T> subBuf( const Position &pos, const Size &size )                              const { return AreaBuf<const T>( bufAt( pos  ), stride, size   ); }
  AreaBuf<      T> subBuf( const int &x, const int &y, const unsigned &_w, const unsigned &_h )       { return AreaBuf<      T>( bufAt( x, y ), stride, _w, _h ); }
  AreaBuf<const T> subBuf( const int &x, const int &y, const unsigned &_w, const unsigned &_h ) const { return AreaBuf<const T>( bufAt( x, y ), stride, _w, _h ); }
};

typedef AreaBuf<      Pel>  PelBuf;
typedef AreaBuf<const Pel> CPelBuf;

typedef AreaBuf<      TCoeff>  CoeffBuf;
typedef AreaBuf<const TCoeff> CCoeffBuf;

typedef AreaBuf<TCoeffSig>        CoeffSigBuf;
typedef AreaBuf<const TCoeffSig> CCoeffSigBuf;

typedef AreaBuf<      MotionInfo>  MotionBuf;
typedef AreaBuf<const MotionInfo> CMotionBuf;

typedef AreaBuf<      LoopFilterParam>  LFPBuf;
typedef AreaBuf<const LoopFilterParam> CLFPBuf;

#define SIZE_AWARE_PER_EL_OP( OP, INC )                     \
if( ( width & 7 ) == 0 )                                    \
{                                                           \
  for( int y = 0; y < height; y++ )                         \
  {                                                         \
    for( int x = 0; x < width; x += 8 )                     \
    {                                                       \
      OP( x + 0 );                                          \
      OP( x + 1 );                                          \
      OP( x + 2 );                                          \
      OP( x + 3 );                                          \
      OP( x + 4 );                                          \
      OP( x + 5 );                                          \
      OP( x + 6 );                                          \
      OP( x + 7 );                                          \
    }                                                       \
                                                            \
    INC;                                                    \
  }                                                         \
}                                                           \
else if( ( width & 3 ) == 0 )                               \
{                                                           \
  for( int y = 0; y < height; y++ )                         \
  {                                                         \
    for( int x = 0; x < width; x += 4 )                     \
    {                                                       \
      OP( x + 0 );                                          \
      OP( x + 1 );                                          \
      OP( x + 2 );                                          \
      OP( x + 3 );                                          \
    }                                                       \
                                                            \
    INC;                                                    \
  }                                                         \
}                                                           \
else if( ( width & 1 ) == 0 )                               \
{                                                           \
  for( int y = 0; y < height; y++ )                         \
  {                                                         \
    for( int x = 0; x < width; x += 2 )                     \
    {                                                       \
      OP( x + 0 );                                          \
      OP( x + 1 );                                          \
    }                                                       \
                                                            \
    INC;                                                    \
  }                                                         \
}                                                           \
else                                                        \
{                                                           \
  for( int y = 0; y < height; y++ )                         \
  {                                                         \
    for( int x = 0; x < width; x++ )                        \
    {                                                       \
      OP( x );                                              \
    }                                                       \
                                                            \
    INC;                                                    \
  }                                                         \
}


template<typename TOP, typename TINC>
static inline void size_aware_pel_op( TOP op, TINC inc, int width, int height )
{
  if( ( width & 7 ) == 0 )
  {
    for( int y = 0; y < height; y++ )
    {
      for( int x = 0; x < width; x += 8 )
      {
        op( x + 0 );
        op( x + 1 );
        op( x + 2 );
        op( x + 3 );
        op( x + 4 );
        op( x + 5 );
        op( x + 6 );
        op( x + 7 );
      }

      inc();
    }
  }
  else if( ( width & 3 ) == 0 )
  {
    for( int y = 0; y < height; y++ )
    {
      for( int x = 0; x < width; x += 4 )
      {
        op( x + 0 );
        op( x + 1 );
        op( x + 2 );
        op( x + 3 );
      }

      inc();
    }
  }
  else if( ( width & 1 ) == 0 )
  {
    for( int y = 0; y < height; y++ )
    {
      for( int x = 0; x < width; x += 2 )
      {
        op( x + 0 );
        op( x + 1 );
      }

      inc();
    }
  }
  else
  {
    for( int y = 0; y < height; y++ )
    {
      for( int x = 0; x < width; x++ )
      {
        op( x );
      }

      inc();
    }
  }
}

template<>
void AreaBuf<MotionInfo>::fill( const MotionInfo& val );

template<typename T>
void AreaBuf<T>::fill(const T &val)
{
  if( T( 0 ) == val )
  {
    if( width == stride )
    {
      ::memset( buf, 0, width * height * sizeof( T ) );
    }
    else
    {
      T* dest = buf;
      size_t line = width * sizeof( T );

      for( unsigned y = 0; y < height; y++ )
      {
        ::memset( dest, 0, line );

        dest += stride;
      }
    }
  }
  else
  {
    if( width == stride )
    {
      std::fill_n( buf, width * height, val );
    }
    else
    {
      T* dest = buf;

      for( int y = 0; y < height; y++, dest += stride )
      {
        std::fill_n( dest, width, val );
      }
    }
  }
}

template<typename T>
void AreaBuf<T>::memset( const int val )
{
  GCC_WARNING_DISABLE_class_memaccess
  if( width == stride )
  {
    ::memset( buf, val, width * height * sizeof( T ) );
  }
  else
  {
    T* dest = buf;
    size_t line = width * sizeof( T );

    for( int y = 0; y < height; y++ )
    {
      ::memset( dest, val, line );

      dest += stride;
    }
  }
  GCC_WARNING_RESET
}

#if ENABLE_SIMD_OPT_BUFFER
template<typename T>
void AreaBuf<T>::copyFrom( const AreaBuf<const T> &other )
{
#if !defined(__GNUC__) || __GNUC__ > 5
  static_assert( std::is_trivially_copyable<T>::value, "Type T is not trivially_copyable" );
#endif

  g_pelBufOP.copyBuffer( (const char *) other.buf, sizeof( T ) * other.stride, (char *) buf, sizeof( T ) * stride, sizeof( T ) * width, height );
}
#else
template<typename T>
void AreaBuf<T>::copyFrom( const AreaBuf<const T> &other )
{
#if !defined(__GNUC__) || __GNUC__ > 5
  static_assert( std::is_trivially_copyable<T>::value, "Type T is not trivially_copyable" );
#endif

  CHECK_FATAL( width  != other.width,  "Incompatible size" );
  CHECK_FATAL( height != other.height, "Incompatible size" );

  if( buf == other.buf )
  {
    return;
  }

  if( ptrdiff_t( width ) == stride && stride == other.stride )
  {
    memcpy( buf, other.buf, width * height * sizeof( T ) );
  }
  else
  {
          T* dst              =       buf;
    const T* src              = other.buf;
    const ptrdiff_t srcStride = other.stride;

    for( unsigned y = 0; y < height; y++ )
    {
      memcpy( dst, src, width * sizeof( T ) );

      dst += stride;
      src += srcStride;
    }
  }
}
#endif


template<typename T>
void AreaBuf<T>::subtract( const AreaBuf<const T> &other )
{
  CHECK( width  != other.width,  "Incompatible size" );
  CHECK( height != other.height, "Incompatible size" );

        T* dest =       buf;
  const T* subs = other.buf;

#define SUBS_INC        \
  dest +=       stride; \
  subs += other.stride; \

#define SUBS_OP( ADDR ) dest[ADDR] -= subs[ADDR]

  SIZE_AWARE_PER_EL_OP( SUBS_OP, SUBS_INC );

#undef SUBS_OP
#undef SUBS_INC
}

template<typename T>
void AreaBuf<T>::reconstruct( const AreaBuf<const T> &pred, const AreaBuf<const T> &resi, const ClpRng& clpRng )
{
  THROW_FATAL( "Type not supported" );
}

template<>
void AreaBuf<Pel>::reconstruct( const AreaBuf<const Pel> &pred, const AreaBuf<const Pel> &resi, const ClpRng& clpRng );


template<typename T>
void AreaBuf<T>::addAvg( const AreaBuf<const T> &other1, const AreaBuf<const T> &other2, const ClpRng& clpRng )
{
  THROW_FATAL( "Type not supported" );
}

template<>
void AreaBuf<Pel>::addAvg( const AreaBuf<const Pel> &other1, const AreaBuf<const Pel> &other2, const ClpRng& clpRng );

template<typename T>
void AreaBuf<T>::linearTransform( const int scale, const int shift, const int offset, bool bClip, const ClpRng& clpRng )
{
  THROW_FATAL( "Type not supported" );
}

template<>
void AreaBuf<Pel>::linearTransform( const int scale, const int shift, const int offset, bool bClip, const ClpRng& clpRng );

template<typename T>
void AreaBuf<T>::extendBorderPel( unsigned margin )
{
  T*        p = buf;
  int       h = height;
  int       w = width;
  ptrdiff_t s = stride;

  CHECK( ( w + 2 * margin ) > s, "Size of buffer too small to extend" );
  // do left and right margins
  for( int y = 0; y < h; y++ )
  {
    for( int x = 0; x < margin; x++ )
    {
      *( p - margin + x ) = p[0];
      p[w + x]            = p[w - 1];
    }
    p += s;
  }

  // p is now the (0,height) (bottom left of image within bigger picture
  p -= ( s + margin );
  // p is now the (-margin, height-1)
  for( int y = 0; y < margin; y++ )
  {
    ::memcpy( p + ( y + 1 ) * s, p, sizeof( T ) * ( w + ( margin << 1 ) ) );
  }

  // pi is still (-marginX, height-1)
  p -= ( ( h - 1 ) * s );
  // pi is now (-marginX, 0)
  for( int y = 0; y < margin; y++ )
  {
    ::memcpy( p - ( y + 1 ) * s, p, sizeof( T ) * ( w + ( margin << 1 ) ) );
  }
}

template<typename T>
void AreaBuf<T>::extendBorderPel(unsigned margin, bool left, bool right, bool top, bool bottom)
{
  CHECK( ( width + left*margin + right*margin) > stride, "Size of buffer too small to extend" );
  // do left and right margins

  if( left && right )
  {
    T* p = buf;
    for( int y = 0; y < height; y++ )
    {
      for( int x = 0; x < margin; x++ )
      {
        p[-(int)margin + x] = p[0];
        p[width + x]   = p[width - 1];
      }
      p += stride;
    }
  }

  else if( left )
  {
    T* p = buf;
    for( int y = 0; y < height; y++ )
    {
      for( int x = 0; x < margin; x++ )
      {
        p[-(int)margin + x] = p[0];
      }
      p += stride;
    }
  }

  else if( right )
  {
    T* p = buf;
    for( int y = 0; y < height; y++ )
    {
      for( int x = 0; x < margin; x++ )
      {
        p[width + x] = p[width - 1];
      }
      p += stride;
    }
  }

  const int copylen = width + ( left ? margin : 0 ) + ( right ? margin : 0 );
  if( bottom )
  {
    T* p = buf + stride * height;
    if( left )
      p -= margin;

    // p is now the (-margin, height)
    for( int y = 0; y < margin; y++ )
    {
      ::memcpy( p + y * stride, p - stride, sizeof( T ) * copylen );
    }
  }

  if( top )
  {
    T* p = buf;
    if( left )
      p -= margin;

    // pi is now (-marginX, 0)
    for( int y = -(int)margin; y < 0; y++ )
    {
      ::memcpy( p + y * stride, p, sizeof( T ) * copylen );
    }
  }
}

template<typename T>
void AreaBuf<T>::padBorderPel( unsigned marginX, unsigned marginY, int dir )
{
  T*   p = buf;
  auto s = stride;
  int  h = height;
  int  w = width;

  CHECK( w  > s, "Size of buffer too small to extend" );

  // top-left margin
  if ( dir == 1 )
  {
    for( int y = 0; y < marginY; y++ )
    {
      for( int x = 0; x < marginX; x++ )
      {
        p[x] = p[marginX];
      }
      p += s;
    }
  }

  // bottom-right margin
  if ( dir == 2 )
  {
    p = buf + s * ( h - marginY ) + w - marginX;

    for( int y = 0; y < marginY; y++ )
    {
      for( int x = 0; x < marginX; x++ )
      {
        p[x] = p[-1];
      }
      p += s;
    }
  }
}

#if ENABLE_SIMD_OPT_BUFFER && defined(TARGET_SIMD_X86)
template<> void AreaBuf<Pel>::transposedFrom( const AreaBuf<const Pel> &other );
#endif

template<typename T>
void AreaBuf<T>::transposedFrom( const AreaBuf<const T> &other )
{
  CHECK( width * height != other.width * other.height, "Incompatible size" );

        T* dst  =       buf;
  const T* src  = other.buf;
  width         = other.height;
  height        = other.width;
  stride        = stride < width ? width : stride;

  for( unsigned y = 0; y < other.height; y++ )
  {
    for( unsigned x = 0; x < other.width; x++ )
    {
      dst[y + x*stride] = src[x + y * other.stride];
    }
  }
}

#ifndef DONT_UNDEF_SIZE_AWARE_PER_EL_OP
#undef SIZE_AWARE_PER_EL_OP
#endif // !DONT_UNDEF_SIZE_AWARE_PER_EL_OP

// ---------------------------------------------------------------------------
// UnitBuf struct
// ---------------------------------------------------------------------------

struct UnitArea;

template<typename T>
struct UnitBuf
{
  typedef static_vector<AreaBuf<T>,       MAX_NUM_COMPONENT> UnitBufBuffers;

  ChromaFormat chromaFormat;
  UnitBufBuffers bufs;

  UnitBuf() : chromaFormat( NUM_CHROMA_FORMAT ) { }
  UnitBuf( const ChromaFormat &_chromaFormat, const UnitBufBuffers&  _bufs ) : chromaFormat( _chromaFormat ), bufs( _bufs ) { }
  UnitBuf( const ChromaFormat &_chromaFormat,       UnitBufBuffers&& _bufs ) : chromaFormat( _chromaFormat ), bufs( std::forward<UnitBufBuffers>( _bufs ) ) { }
  UnitBuf( const ChromaFormat &_chromaFormat, const AreaBuf<T>  &blkY ) : chromaFormat( _chromaFormat ), bufs{ blkY } { }
  UnitBuf( const ChromaFormat &_chromaFormat,       AreaBuf<T> &&blkY ) : chromaFormat( _chromaFormat ), bufs{ std::forward<AreaBuf<T> >(blkY) } { }
  UnitBuf( const ChromaFormat &_chromaFormat, const AreaBuf<T>  &blkY, const AreaBuf<T>  &blkCb, const AreaBuf<T>  &blkCr ) : chromaFormat( _chromaFormat ), bufs{ blkY, blkCb, blkCr } { if( chromaFormat == CHROMA_400 ) bufs.resize( 1 ); }
  UnitBuf( const ChromaFormat &_chromaFormat,       AreaBuf<T> &&blkY,       AreaBuf<T> &&blkCb,       AreaBuf<T> &&blkCr ) : chromaFormat( _chromaFormat ), bufs{ std::forward<AreaBuf<T> >(blkY), std::forward<AreaBuf<T> >(blkCb), std::forward<AreaBuf<T> >(blkCr) } { if( chromaFormat == CHROMA_400 ) bufs.resize( 1 ); }

  UnitBuf( const UnitBuf& other )  = default;
  UnitBuf(       UnitBuf&& other ) = default;
  UnitBuf& operator=( const UnitBuf& other )  = default;
  UnitBuf& operator=(       UnitBuf&& other ) = default;

  // conversion from UnitBuf<const T> to UnitBuf<T>
  template<bool T_IS_COST = std::is_const<T>::value>
  UnitBuf( const UnitBuf<typename std::remove_const_t<T>>& other, std::enable_if_t<T_IS_COST>* = 0 ) : chromaFormat( other.chromaFormat ), bufs( other.bufs ) { }

        AreaBuf<T>& get( const ComponentID comp )        { return bufs[comp]; }
  const AreaBuf<T>& get( const ComponentID comp )  const { return bufs[comp]; }

        AreaBuf<T>& Y()        { return bufs[0]; }
  const AreaBuf<T>& Y()  const { return bufs[0]; }
        AreaBuf<T>& Cb()       { return bufs[1]; }
  const AreaBuf<T>& Cb() const { return bufs[1]; }
        AreaBuf<T>& Cr()       { return bufs[2]; }
  const AreaBuf<T>& Cr() const { return bufs[2]; }

  void fill                 ( const T &val );
  void copyFrom             ( const UnitBuf<const T> &other );
  void reconstruct          ( const UnitBuf<const T> &pred, const UnitBuf<const T> &resi, const ClpRngs& clpRngs );
  void subtract             ( const UnitBuf<const T> &other );
  void addWeightedAvg       ( const UnitBuf<      T> &other1, const UnitBuf<      T> &other2, const ClpRngs& clpRngs, const uint8_t bcwIdx = BCW_DEFAULT, const bool chromaOnly = false, const bool lumaOnly = false);
  void addAvg               ( const UnitBuf<      T> &other1, const UnitBuf<      T> &other2, const ClpRngs& clpRngs, const bool chromaOnly = false, const bool lumaOnly = false);
  void extendBorderPel      ( unsigned margin );
  void extendBorderPel      ( unsigned margin, bool left, bool right, bool top, bool bottom );
  void padBorderPel         ( unsigned margin, int dir );

        UnitBuf<      T> subBuf (const Area& subArea);
  const UnitBuf<const T> subBuf (const Area& subArea) const;
        UnitBuf<      T> subBuf (const UnitArea& subArea);
  const UnitBuf<const T> subBuf (const UnitArea& subArea) const;
  void colorSpaceConvert    ( const UnitBuf<T> &other, const ClpRng& clpRng );

  void writeToFile( std::string filename ) const;   // for debug purposes
};

typedef UnitBuf<      Pel>  PelUnitBuf;
typedef UnitBuf<const Pel> CPelUnitBuf;

typedef UnitBuf<      TCoeff>  CoeffUnitBuf;
typedef UnitBuf<const TCoeff> CCoeffUnitBuf;

}   // namespace vvdec

#include <Unit.h>

namespace vvdec
{
template<typename T>
void UnitBuf<T>::fill( const T &val )
{
  for( unsigned i = 0; i < bufs.size(); i++ )
  {
    bufs[i].fill( val );
  }
}

template<typename T>
void UnitBuf<T>::copyFrom( const UnitBuf<const T> &other )
{
  CHECK( chromaFormat != other.chromaFormat, "Incompatible formats" );

  for( unsigned i = 0; i < bufs.size(); i++ )
  {
    bufs[i].copyFrom( other.bufs[i] );
  }
}



template<typename T>
void UnitBuf<T>::subtract( const UnitBuf<const T> &other )
{
  CHECK( chromaFormat != other.chromaFormat, "Incompatible formats" );

  for( unsigned i = 0; i < bufs.size(); i++ )
  {
    bufs[i].subtract( other.bufs[i] );
  }
}

template<typename T>
void UnitBuf<T>::reconstruct(const UnitBuf<const T> &pred, const UnitBuf<const T> &resi, const ClpRngs& clpRngs)
{
  CHECK( chromaFormat != pred.chromaFormat, "Incompatible formats" );
  CHECK( chromaFormat != resi.chromaFormat, "Incompatible formats" );

  for( unsigned i = 0; i < bufs.size(); i++ )
  {
    bufs[i].reconstruct( pred.bufs[i], resi.bufs[i], clpRngs );
  }
}

template<typename T>
void UnitBuf<T>::addWeightedAvg(const UnitBuf<T> &other1, const UnitBuf<T> &other2, const ClpRngs& clpRngs, const uint8_t bcwIdx /* = BCW_DEFAULT */, const bool chromaOnly /* = false */, const bool lumaOnly /* = false */)
{
  const size_t istart = chromaOnly ? 1 : 0;
  const size_t iend   = lumaOnly   ? 1 : bufs.size();

  CHECK(lumaOnly && chromaOnly, "should not happen");

  for(size_t i = istart; i < iend; i++)
  {
    bufs[i].addWeightedAvg(other1.bufs[i], other2.bufs[i], clpRngs, bcwIdx);
  }
}

template<typename T>
void UnitBuf<T>::addAvg(const UnitBuf<T> &other1, const UnitBuf<T> &other2, const ClpRngs& clpRngs, const bool chromaOnly /* = false */, const bool lumaOnly /* = false */)
{
  const size_t istart = chromaOnly ? 1 : 0;
  const size_t iend   = lumaOnly   ? 1 : bufs.size();

  CHECK( lumaOnly && chromaOnly, "should not happen" );

  for( size_t i = istart; i < iend; i++)
  {
    bufs[i].addAvg( other1.bufs[i], other2.bufs[i], clpRngs );
  }
}

template<typename T>
void UnitBuf<T>::colorSpaceConvert( const UnitBuf<T> &other, const ClpRng& clpRng )
{
  THROW_FATAL( "Type not supported" );
}

template<>
void UnitBuf<Pel>::colorSpaceConvert( const UnitBuf<Pel> &other, const ClpRng& clpRng );

template<typename T>
void UnitBuf<T>::extendBorderPel( unsigned margin )
{
  for( unsigned i = 0; i < bufs.size(); i++ )
  {
    bufs[i].extendBorderPel( margin );
  }
}

template<typename T>
void UnitBuf<T>::extendBorderPel(unsigned margin, bool left, bool right, bool top, bool bottom)
{
  for( unsigned i = 0; i < bufs.size(); i++ )
  {
    bufs[i].extendBorderPel( margin, left, right, top, bottom );
  }
}

template<typename T>
void UnitBuf<T>::padBorderPel( unsigned margin, int dir )
{
  for( unsigned i = 0; i < bufs.size(); i++ )
  {
    bufs[i].padBorderPel( margin >> getComponentScaleX( ComponentID( i ), chromaFormat ), margin >> getComponentScaleY( ComponentID( i ), chromaFormat ), dir );
  }
}

template<typename T>
UnitBuf<T> UnitBuf<T>::subBuf( const UnitArea& subArea )
{
  UnitBuf<T> subBuf;
  subBuf.chromaFormat = chromaFormat;
  unsigned blockIdx = 0;

  for( auto &subAreaBuf : bufs )
  {
    subBuf.bufs.push_back( subAreaBuf.subBuf( subArea.blocks[blockIdx].pos(), subArea.blocks[blockIdx].size() ) );
    blockIdx++;
  }

  return subBuf;
}


template<typename T>
const UnitBuf<const T> UnitBuf<T>::subBuf( const UnitArea& subArea ) const
{
  UnitBuf<const T> subBuf;
  subBuf.chromaFormat = chromaFormat;
  unsigned blockIdx = 0;

  for( const auto &subAreaBuf : bufs )
  {
    subBuf.bufs.push_back( subAreaBuf.subBuf( subArea.blocks[blockIdx].pos(), subArea.blocks[blockIdx].size() ) );
    blockIdx++;
  }

  return subBuf;
}

template<typename T>
UnitBuf<T> UnitBuf<T>::subBuf( const Area & subArea )
{
  UnitBuf<T> subBuf;
  subBuf.chromaFormat = chromaFormat;
  unsigned blockIdx = 0;

  for( auto &subAreaBuf : bufs )
  {
    const int scaleX = getComponentScaleX( ComponentID(blockIdx), chromaFormat);
    const int scaleY = getComponentScaleY( ComponentID(blockIdx), chromaFormat);
    const Area scaledArea( subArea.pos().x >> scaleX, subArea.pos().y >> scaleY, subArea.size().width >> scaleX, subArea.size().height >> scaleY );
    subBuf.bufs.push_back( subAreaBuf.subBuf( scaledArea.pos(), scaledArea.size() ) );
    blockIdx++;
  }

  return subBuf;
}

template<typename T>
const UnitBuf<const T> UnitBuf<T>::subBuf( const Area & subArea ) const
{
  UnitBuf<T> subBuf;
  subBuf.chromaFormat = chromaFormat;
  unsigned blockIdx = 0;

  for( auto &subAreaBuf : bufs )
  {
    const int scaleX = getComponentScaleX( ComponentID(blockIdx), chromaFormat);
    const int scaleY = getComponentScaleY( ComponentID(blockIdx), chromaFormat);
    const Area scaledArea( subArea.pos().x >> scaleX, subArea.pos().y >> scaleY, subArea.size().width >> scaleX, subArea.size().height >> scaleY );
    subBuf.bufs.push_back( subAreaBuf.subBuf( scaledArea.pos(), scaledArea.size() ) );
    blockIdx++;
  }

  return subBuf;
}

// ---------------------------------------------------------------------------
// PelStorage struct (PelUnitBuf which allocates its own memory)
// ---------------------------------------------------------------------------

struct UnitArea;
struct CompArea;

struct PelStorage : public PelUnitBuf
{
  PelStorage();
  ~PelStorage();

  void swap( PelStorage& other );
  void createFromBuf( PelUnitBuf buf );
  void create( const UnitArea &_unit );
  void create( const ChromaFormat _chromaFormat, const Size& _size, const unsigned _maxCUSize = 0, const unsigned _margin = 0, const unsigned _alignment = 0, const bool _scaleChromaMargin = true, const UserAllocator* userAlloc = nullptr );
  void destroy();

         PelBuf getBuf( const CompArea &blk );
  const CPelBuf getBuf( const CompArea &blk ) const;

         PelBuf getBuf( const ComponentID CompID );
  const CPelBuf getBuf( const ComponentID CompID ) const;

         PelUnitBuf getBuf( const UnitArea &unit );
  const CPelUnitBuf getBuf( const UnitArea &unit ) const;
  Pel *getOrigin( const int id ) const { return m_origin[id]; }
  PelBuf getOriginBuf( const int id ) { return PelBuf( m_origin[id], m_origSi[id] ); }

  Size  getBufSize( const int id )      const { return  m_origSi[id];     }
  void *getBufAllocator( const int id ) const { return m_allocator[id];   }
  bool  isExternAllocator()             const { return m_externAllocator; }

  const UserAllocator* getUserAllocator() const { return m_userAlloc; }

private:

  Size    m_origSi[MAX_NUM_COMPONENT];
  Pel    *m_origin[MAX_NUM_COMPONENT];
  void   *m_allocator[MAX_NUM_COMPONENT];
  bool    m_externAllocator = false;
  const UserAllocator* m_userAlloc  = nullptr;
};

}   // namespace vvdec
