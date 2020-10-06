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

/** \file     Buffer.h
 *  \brief    Low-overhead class describing 2D memory layout
 */

#ifndef __BUFFER__
#define __BUFFER__

#include "Common.h"
#include "CommonDef.h"
#include "ChromaFormat.h"
#include "MotionInfo.h"

#include <string.h>
#include <type_traits>
#include <typeinfo>
#include <vector>

// ---------------------------------------------------------------------------
// AreaBuf struct
// ---------------------------------------------------------------------------

struct PelBufferOps
{
  PelBufferOps();

#if ENABLE_SIMD_OPT_BUFFER
#ifdef TARGET_SIMD_X86
  void initPelBufOpsX86();
  template<X86_VEXT vext>
  void _initPelBufOpsX86();

#endif
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
  void ( *copyBuffer )    ( const char*src,  ptrdiff_t srcStride,        char* dst, ptrdiff_t  dstStride,                                int width, int height );
  void ( *padding1 )      (       Pel *dst,  ptrdiff_t stride,                                                                           int width, int height );
  void ( *padding2 )      (       Pel *dst,  ptrdiff_t stride,                                                                           int width, int height );
  void ( *transpose4x4 )  ( const Pel* src,  ptrdiff_t srcStride, Pel* dst, ptrdiff_t dstStride );
  void ( *transpose8x8 )  ( const Pel* src,  ptrdiff_t srcStride, Pel* dst, ptrdiff_t dstStride );
  void ( *applyLut )      (       Pel* ptr,  ptrdiff_t ptrStride, int width, int height, const Pel* lut );
  void ( *fillN_CU )      (       CodingUnit** ptr, ptrdiff_t ptrStride, int width, int height, CodingUnit* cuPtr );
};

extern PelBufferOps g_pelBufOP;

#define INCX( ptr, stride ) { ptr++; }
#define INCY( ptr, stride ) { ptr += ( stride ); }
#define OFFSETX( ptr, stride, x ) { ptr += ( x ); }
#define OFFSETY( ptr, stride, y ) { ptr += ( y ) * ( stride ); }
#define OFFSET( ptr, stride, x, y ) { ptr += ( x ) + ( y ) * ( stride ); }
#define GET_OFFSETX( ptr, stride, x ) ( ( ptr ) + ( x ) )
#define GET_OFFSETY( ptr, stride, y ) ( ( ptr ) + ( y ) * ( stride ) )
#define GET_OFFSET( ptr, stride, x, y ) ( ( ptr ) + ( x ) + ( y ) * ( stride ) )

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
  AreaBuf( const AreaBuf<typename std::remove_const<T>::type >& other )                         : Size( other ),           buf( other.buf ), stride( other.stride ) { }

  void fill                 ( const T &val );
  void memset               ( const int val );

  void copyFrom             ( const AreaBuf<const T> &other ) const;
  
  void reconstruct          ( const AreaBuf<const T> &pred, const AreaBuf<const T> &resi, const ClpRng& clpRng);
  
  void subtract             ( const AreaBuf<const T> &other );
  void extendBorderPel      ( unsigned margin );
  void extendBorderPel      ( unsigned margin, bool left, bool right, bool top, bool bottom );
  void addWeightedAvg       ( const AreaBuf<const T> &other1, const AreaBuf<const T> &other2, const ClpRng& clpRng, const int8_t bcwIdx);
  void addAvg               ( const AreaBuf<const T> &other1, const AreaBuf<const T> &other2, const ClpRng& clpRng );

  void linearTransform      ( const int scale, const int shift, const int offset, bool bClip, const ClpRng& clpRng );

  void transposedFrom       ( const AreaBuf<const T> &other );

  void rspSignal            ( const Pel *lut );
  void scaleSignal          ( const int scale, const ClpRng& clpRng);

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

#if ENABLE_SIMD_OPT_BUFFER && defined( TARGET_SIMD_X86 )
template<typename T>
void AreaBuf<T>::copyFrom( const AreaBuf<const T> &other ) const
{
#if !defined(__GNUC__) || __GNUC__ > 5
  static_assert( std::is_trivially_copyable<T>::value, "Type T is not trivially_copyable" );
#endif

  g_pelBufOP.copyBuffer( (const char *) other.buf, sizeof( T ) * other.stride, (char *) buf, sizeof( T ) * stride, sizeof( T ) * width, height );
}
#else
template<typename T>
void AreaBuf<T>::copyFrom( const AreaBuf<const T> &other ) const
{
#if !defined(__GNUC__) || __GNUC__ > 5
  static_assert( std::is_trivially_copyable<T>::value, "Type T is not trivially_copyable" );
#endif

  CHECK( width  != other.width,  "Incompatible size" );
  CHECK( height != other.height, "Incompatible size" );

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
  THROW( "Type not supported" );
}

template<>
void AreaBuf<Pel>::reconstruct( const AreaBuf<const Pel> &pred, const AreaBuf<const Pel> &resi, const ClpRng& clpRng );


template<typename T>
void AreaBuf<T>::addAvg( const AreaBuf<const T> &other1, const AreaBuf<const T> &other2, const ClpRng& clpRng )
{
  THROW( "Type not supported" );
}

template<>
void AreaBuf<Pel>::addAvg( const AreaBuf<const Pel> &other1, const AreaBuf<const Pel> &other2, const ClpRng& clpRng );

template<typename T>
void AreaBuf<T>::linearTransform( const int scale, const int shift, const int offset, bool bClip, const ClpRng& clpRng )
{
  THROW( "Type not supported" );
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
  typedef static_vector<AreaBuf<const T>, MAX_NUM_COMPONENT> ConstUnitBufBuffers;

  ChromaFormat chromaFormat;
  UnitBufBuffers bufs;

  UnitBuf() : chromaFormat( NUM_CHROMA_FORMAT ) { }
  UnitBuf( const ChromaFormat &_chromaFormat, const UnitBufBuffers&  _bufs ) : chromaFormat( _chromaFormat ), bufs( _bufs ) { }
  UnitBuf( const ChromaFormat &_chromaFormat,       UnitBufBuffers&& _bufs ) : chromaFormat( _chromaFormat ), bufs( std::forward<UnitBufBuffers>( _bufs ) ) { }
  UnitBuf( const ChromaFormat &_chromaFormat, const AreaBuf<T>  &blkY ) : chromaFormat( _chromaFormat ), bufs{ blkY } { }
  UnitBuf( const ChromaFormat &_chromaFormat,       AreaBuf<T> &&blkY ) : chromaFormat( _chromaFormat ), bufs{ std::forward<AreaBuf<T> >(blkY) } { }
  UnitBuf( const ChromaFormat &_chromaFormat, const AreaBuf<T>  &blkY, const AreaBuf<T>  &blkCb, const AreaBuf<T>  &blkCr ) : chromaFormat( _chromaFormat ), bufs{ blkY, blkCb, blkCr } { if( chromaFormat == CHROMA_400 ) bufs.resize( 1 ); }
  UnitBuf( const ChromaFormat &_chromaFormat,       AreaBuf<T> &&blkY,       AreaBuf<T> &&blkCb,       AreaBuf<T> &&blkCr ) : chromaFormat( _chromaFormat ), bufs{ std::forward<AreaBuf<T> >(blkY), std::forward<AreaBuf<T> >(blkCb), std::forward<AreaBuf<T> >(blkCr) } { if( chromaFormat == CHROMA_400 ) bufs.resize( 1 ); }
  UnitBuf( const UnitBuf<typename std::remove_const<T>::type>& other ) : chromaFormat( other.chromaFormat ), bufs{}
  {
    // TODO: delete to avoid unneccessary copying
    for( auto &buf : other.bufs )
    {
      bufs.push_back( buf );
    }
  }

        AreaBuf<T>& get( const ComponentID comp )        { return bufs[comp]; }
  const AreaBuf<T>& get( const ComponentID comp )  const { return bufs[comp]; }

        AreaBuf<T>& Y()        { return bufs[0]; }
  const AreaBuf<T>& Y()  const { return bufs[0]; }
        AreaBuf<T>& Cb()       { return bufs[1]; }
  const AreaBuf<T>& Cb() const { return bufs[1]; }
        AreaBuf<T>& Cr()       { return bufs[2]; }
  const AreaBuf<T>& Cr() const { return bufs[2]; }

  void fill                 ( const T &val );
  void copyFrom             ( const UnitBuf<const T> &other ) const;
  void reconstruct          ( const UnitBuf<const T> &pred, const UnitBuf<const T> &resi, const ClpRngs& clpRngs );
  void subtract             ( const UnitBuf<const T> &other );
  void addWeightedAvg       ( const UnitBuf<      T> &other1, const UnitBuf<      T> &other2, const ClpRngs& clpRngs, const uint8_t bcwIdx = BCW_DEFAULT, const bool chromaOnly = false, const bool lumaOnly = false);
  void addAvg               ( const UnitBuf<      T> &other1, const UnitBuf<      T> &other2, const ClpRngs& clpRngs, const bool chromaOnly = false, const bool lumaOnly = false);
  void extendBorderPel      ( unsigned margin );
  void extendBorderPel      ( unsigned margin, bool left, bool right, bool top, bool bottom );

        UnitBuf<      T> subBuf (const Area& subArea);
  const UnitBuf<const T> subBuf (const Area& subArea) const;
        UnitBuf<      T> subBuf (const UnitArea& subArea);
  const UnitBuf<const T> subBuf (const UnitArea& subArea) const;
  void colorSpaceConvert    ( const UnitBuf<T> &other, const ClpRng& clpRng );
};

typedef UnitBuf<      Pel>  PelUnitBuf;
typedef UnitBuf<const Pel> CPelUnitBuf;

typedef UnitBuf<      TCoeff>  CoeffUnitBuf;
typedef UnitBuf<const TCoeff> CCoeffUnitBuf;

template<typename T>
void UnitBuf<T>::fill( const T &val )
{
  for( unsigned i = 0; i < bufs.size(); i++ )
  {
    bufs[i].fill( val );
  }
}

template<typename T>
void UnitBuf<T>::copyFrom( const UnitBuf<const T> &other ) const
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
  THROW( "Type not supported" );
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
  void create( const ChromaFormat _chromaFormat, const Size& _size, const unsigned _maxCUSize = 0, const unsigned _margin = 0, const unsigned _alignment = 0, const bool _scaleChromaMargin = true );
  void destroy();

         PelBuf getBuf( const CompArea &blk );
  const CPelBuf getBuf( const CompArea &blk ) const;

         PelBuf getBuf( const ComponentID CompID );
  const CPelBuf getBuf( const ComponentID CompID ) const;

         PelUnitBuf getBuf( const UnitArea &unit );
  const CPelUnitBuf getBuf( const UnitArea &unit ) const;
  Pel *getOrigin( const int id ) const { return m_origin[id]; }
  PelBuf getOriginBuf( const int id ) { return PelBuf( m_origin[id], m_origSi[id] ); }

private:

  Size m_origSi[MAX_NUM_COMPONENT];
  Pel *m_origin[MAX_NUM_COMPONENT];
};

#endif
