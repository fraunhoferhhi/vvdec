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

/** \file     Common.h
 *  \brief    Common 2D-geometrical structures
 */

#ifndef __COMMON__
#define __COMMON__

#include "CommonDef.h"

typedef int PosType;
typedef uint32_t SizeType;
struct Position
{
  PosType x;
  PosType y;

  constexpr Position()                                   : x(0),  y(0)  { }
  constexpr Position(const PosType _x, const PosType _y) : x(_x), y(_y) { }

  constexpr bool operator!=(const Position &other)  const { return x != other.x || y != other.y; }
  constexpr bool operator==(const Position &other)  const { return x == other.x && y == other.y; }

  constexpr Position offset(const Position pos)                 const { return Position(x + pos.x, y + pos.y); }
  constexpr Position offset(const PosType _x, const PosType _y) const { return Position(x + _x   , y + _y   ); }
  void     repositionTo(const Position newPos)              { x  = newPos.x; y  = newPos.y; }
  void     relativeTo  (const Position origin)              { x -= origin.x; y -= origin.y; }

  constexpr Position operator-( const Position &other )         const { return{ x - other.x, y - other.y }; }
};

struct Size
{
  SizeType width;
  SizeType height;

  constexpr Size()                                              : width(0),      height(0)       { }
  constexpr Size(const SizeType _width, const SizeType _height) : width(_width), height(_height) { }

  constexpr bool operator!=(const Size &other)      const { return (width != other.width) || (height != other.height); }
  constexpr bool operator==(const Size &other)      const { return (width == other.width) && (height == other.height); }
  constexpr uint32_t area()                             const { return (uint32_t) width * (uint32_t) height; }
};

struct Area : public Position, public Size
{
  constexpr Area()                                                                         : Position(),       Size()       { }
  constexpr Area(const Position &_pos, const Size &_size)                                  : Position(_pos),   Size(_size)  { }
  constexpr Area(const PosType _x, const PosType _y, const SizeType _w, const SizeType _h) : Position(_x, _y), Size(_w, _h) { }

                  Position& pos()                           { return *this; }
  constexpr const Position& pos()                     const { return *this; }
                  Size&     size()                          { return *this; }
  constexpr const Size&     size()                    const { return *this; }

  constexpr const Position& topLeft()                 const { return *this; }
  constexpr       Position  topRight()                const { return { (PosType) (x + width - 1), y                          }; }
  constexpr       Position  bottomLeft()              const { return { x                        , (PosType) (y + height - 1) }; }
  constexpr       Position  bottomRight()             const { return { (PosType) (x + width - 1), (PosType) (y + height - 1) }; }
  constexpr       Position  center()                  const { return { (PosType) (x + width / 2), (PosType) (y + height / 2) }; }

  constexpr bool contains(const Position &_pos)       const { return (_pos.x >= x) && (_pos.x < (x + width)) && (_pos.y >= y) && (_pos.y < (y + height)); }
            bool contains(const Area &_area)          const { return contains(_area.pos()) && contains(_area.bottomRight()); }

  constexpr bool operator!=(const Area &other)        const { return (Size::operator!=(other)) || (Position::operator!=(other)); }
  constexpr bool operator==(const Area &other)        const { return (Size::operator==(other)) && (Position::operator==(other)); }
};

struct UnitScale
{
  enum ScaliningType
  {
    UNIT_MAP,
    LF_PARAM_MAP,
    MI_MAP
  };

  constexpr UnitScale()                 : posx( 0), posy( 0), area(0) {}
  constexpr UnitScale( int sx, int sy ) : posx(sx), posy(sy), area(posx+posy) {}
  int posx = 0;
  int posy = 0;
  int area = 0;

  template<typename T> constexpr T scaleHor ( const T &in ) const { return in >> posx; }
  template<typename T> constexpr T scaleVer ( const T &in ) const { return in >> posy; }
  template<typename T> constexpr T scaleArea( const T &in ) const { return in >> area; }

  constexpr Position scale( const Position &pos  ) const { return { pos.x >> posx, pos.y >> posy }; }
  constexpr Size     scale( const Size     &size ) const { return { size.width >> posx, size.height >> posy }; }
  constexpr Area     scale( const Area    &_area ) const { return Area{ scale( _area.pos() ), scale( _area.size() ) }; }
};
namespace std
{
  template <>
  struct hash<Position> : public unary_function<Position, uint64_t>
  {
    uint64_t operator()(const Position& value) const
    {
      return (((uint64_t)value.x << 32) + value.y);
    }
  };

  template <>
  struct hash<Size> : public unary_function<Size, uint64_t>
  {
    uint64_t operator()(const Size& value) const
    {
      return (((uint64_t)value.width << 32) + value.height);
    }
  };
}
constexpr inline ptrdiff_t rsAddr(const Position &pos, const ptrdiff_t stride, const UnitScale &unitScale )
{
  return ( ptrdiff_t )(stride >> unitScale.posx) * ( ptrdiff_t )(pos.y >> unitScale.posy) + ( ptrdiff_t )(pos.x >> unitScale.posx);
}

constexpr inline ptrdiff_t rsAddr(const Position &pos, const Position &origin, const ptrdiff_t stride, const UnitScale &unitScale )
{
  return (stride >> unitScale.posx) * ((pos.y - origin.y) >> unitScale.posy) + ((pos.x - origin.x) >> unitScale.posx);
}

constexpr inline ptrdiff_t rsAddr(const Position &pos, const ptrdiff_t stride )
{
  return stride * ( ptrdiff_t )pos.y + ( ptrdiff_t )pos.x;
}

constexpr inline ptrdiff_t rsAddr(const Position &pos, const Position &origin, const ptrdiff_t stride )
{
  return stride * (pos.y - origin.y) + (pos.x - origin.x);
}

inline Area clipArea(const Area &_area, const Area &boundingBox)
{
  Area area = _area;

  if (area.x + area.width > boundingBox.x + boundingBox.width)
  {
    area.width = boundingBox.x + boundingBox.width - area.x;
  }

  if (area.y + area.height > boundingBox.y + boundingBox.height)
  {
    area.height = boundingBox.y + boundingBox.height - area.y;
  }

  return area;
}



#endif
