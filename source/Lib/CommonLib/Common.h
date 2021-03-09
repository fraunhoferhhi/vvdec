/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

For any license concerning other Intellectual Property rights than the software, 
especially patent licenses, a separate Agreement needs to be closed. 
For more information please contact:

Fraunhofer Heinrich Hertz Institute
Einsteinufer 37
10587 Berlin, Germany
www.hhi.fraunhofer.de/vvc
vvc@hhi.fraunhofer.de

Copyright (c) 2018-2021, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. 
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of Fraunhofer nor the names of its contributors may
   be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
THE POSSIBILITY OF SUCH DAMAGE.


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
