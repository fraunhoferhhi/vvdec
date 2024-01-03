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

/** \file     Unit.cpp
 *  \brief    defines unit as a set of blocks and basic unit types (coding, prediction, transform)
 */

#include "Unit.h"

#include "Buffer.h"
#include "Picture.h"
#include "ChromaFormat.h"

#include "UnitTools.h"
#include "UnitPartitioner.h"

namespace vvdec
{

 // ---------------------------------------------------------------------------
 // block method definitions
 // ---------------------------------------------------------------------------

Position CompArea::chromaPos( const ChromaFormat chromaFormat ) const
{
  if (isLuma(compID()))
  {
    uint32_t scaleX = getComponentScaleX(compID(), chromaFormat);
    uint32_t scaleY = getComponentScaleY(compID(), chromaFormat);

    return Position(x >> scaleX, y >> scaleY);
  }
  else
  {
    return *this;
  }
}

Size CompArea::lumaSize( const ChromaFormat chromaFormat ) const
{
  if( isChroma( compID() ) )
  {
    uint32_t scaleX = getComponentScaleX( compID(), chromaFormat );
    uint32_t scaleY = getComponentScaleY( compID(), chromaFormat );

    return Size( width << scaleX, height << scaleY );
  }
  else
  {
    return *this;
  }
}

Size CompArea::chromaSize( const ChromaFormat chromaFormat ) const
{
  if( isLuma( compID() ) )
  {
    uint32_t scaleX = getComponentScaleX( compID(), chromaFormat );
    uint32_t scaleY = getComponentScaleY( compID(), chromaFormat );

    return Size( width >> scaleX, height >> scaleY );
  }
  else
  {
    return *this;
  }
}

Position CompArea::lumaPos( const ChromaFormat chromaFormat ) const
{
  if( isChroma( compID() ) )
  {
    uint32_t scaleX = getComponentScaleX( compID(), chromaFormat );
    uint32_t scaleY = getComponentScaleY( compID(), chromaFormat );

    return Position( x << scaleX, y << scaleY );
  }
  else
  {
    return *this;
  }
}

Position CompArea::compPos( const ChromaFormat chromaFormat, const ComponentID compID ) const
{
  return isLuma( compID ) ? lumaPos( chromaFormat ) : chromaPos( chromaFormat );
}

Position CompArea::chanPos( const ChromaFormat chromaFormat, const ChannelType chType ) const
{
  return isLuma( chType ) ? lumaPos( chromaFormat ) : chromaPos( chromaFormat );
}

// ---------------------------------------------------------------------------
// unit method definitions
// ---------------------------------------------------------------------------

UnitArea::UnitArea(const ChromaFormat _chromaFormat) : chromaFormat(_chromaFormat) { }

UnitArea::UnitArea(const ChromaFormat _chromaFormat, const Area &_area) : chromaFormat(_chromaFormat)
{
  const uint32_t numCh = getNumberValidComponents( chromaFormat );

  blocks.resize_noinit( numCh );

  if( !numCh ) return;

  blocks[0]._compID      = COMPONENT_Y;
  blocks[0].x            = _area.x;
  blocks[0].y            = _area.y;
  blocks[0].width        = _area.width;
  blocks[0].height       = _area.height;

  if( numCh == 1 ) return;

  const int csx = getChannelTypeScaleX( CH_C, chromaFormat );
  const int csy = getChannelTypeScaleY( CH_C, chromaFormat );
  
  blocks[1]._compID      = COMPONENT_Cb;
  blocks[2]._compID      = COMPONENT_Cr;
  blocks[1].x            = blocks[2].x            = ( _area.x >> csx );
  blocks[1].y            = blocks[2].y            = ( _area.y >> csy );
  blocks[1].width        = blocks[2].width        = ( _area.width  >> csx );
  blocks[1].height       = blocks[2].height       = ( _area.height >> csy );
}

UnitArea::UnitArea(const ChromaFormat _chromaFormat, const CompArea &blkY) : chromaFormat(_chromaFormat), blocks { blkY } {}

UnitArea::UnitArea(const ChromaFormat _chromaFormat,       CompArea &&blkY) : chromaFormat(_chromaFormat), blocks { std::forward<CompArea>(blkY) } {}

UnitArea::UnitArea(const ChromaFormat _chromaFormat, const CompArea &blkY, const CompArea &blkCb, const CompArea &blkCr)  : chromaFormat(_chromaFormat), blocks { blkY, blkCb, blkCr } {}

UnitArea::UnitArea(const ChromaFormat _chromaFormat,       CompArea &&blkY,      CompArea &&blkCb,      CompArea &&blkCr) : chromaFormat(_chromaFormat), blocks { std::forward<CompArea>(blkY), std::forward<CompArea>(blkCb), std::forward<CompArea>(blkCr) } {}

bool UnitArea::contains(const UnitArea& other) const
{
  bool any = false;

  if( blocks[0].valid() && other.blocks[0].valid() )
  {
    any = true;
    if( !blocks[0].contains( other.blocks[0] ) ) return false;
  }

  if( blocks[1].valid() && other.blocks[1].valid() )
  {
    any = true;
    if( !blocks[1].contains( other.blocks[1] ) ) return false;
  }

  if( blocks[2].valid() && other.blocks[2].valid() )
  {
    any = true;
    if( !blocks[2].contains( other.blocks[2] ) ) return false;
  }

  return any;
}

bool UnitArea::contains( const UnitArea& other, const ChannelType chType ) const
{
  if( chType == CH_L && blocks[0].valid() && other.blocks[0].valid() )
  {
    if( !blocks[0].contains( other.blocks[0] ) ) return false;
    return true;
  }

  if( chType == CH_L ) return false;

  bool any = false;

  if( blocks[1].valid() && other.blocks[1].valid() )
  {
    any = true;
    if( !blocks[1].contains( other.blocks[1] ) ) return false;
  }

  if( blocks[2].valid() && other.blocks[2].valid() )
  {
    any = true;
    if( !blocks[2].contains( other.blocks[2] ) ) return false;
  }

  return any;
}

void UnitArea::repositionTo(const UnitArea& unitArea)
{
  for(uint32_t i = 0; i < blocks.size(); i++)
  {
    blocks[i].repositionTo(unitArea.blocks[i]);
  }
}

const UnitArea UnitArea::singleComp(const ComponentID compID) const
{
  UnitArea ret = *this;

  for( auto &blk : ret.blocks )
  {
    if( blk.compID() != compID )
    {
      new ( &blk ) CompArea();
    }
  }

  return ret;
}

const UnitArea UnitArea::singleChan(const ChannelType chType) const
{
#if 1
  UnitArea ret(chromaFormat);

  for (const auto &blk : blocks)
  {
    if (toChannelType( blk.compID() ) == chType)
    {
      ret.blocks.push_back(blk);
    }
    else
    {
      ret.blocks.push_back(CompArea());
    }
  }
#else
  UnitArea ret = *this;

  for( auto &blk : ret.blocks )
  {
    if( toChannelType( blk.compID ) != chType )
    {
      new ( &blk ) CompArea();
    }
  }
#endif
  return ret;
}

// ---------------------------------------------------------------------------
// coding unit method definitions
// ---------------------------------------------------------------------------

void CodingUnit::minInit( const UnitArea &unit )
{
  static_cast<UnitArea &>( *this ) = unit;

  intraDir[0] = DC_IDX;

  refIdx[0]   = refIdx[1] = -1;
}

// ---------------------------------------------------------------------------
// prediction unit method definitions
// ---------------------------------------------------------------------------

CodingUnit& CodingUnit::operator=( const MotionInfo& mi )
{
  setInterDir( mi.interDir() );

  for( uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++ )
  {
    refIdx[i] = mi.miRefIdx[i];
    mv [i][0] = mi.mv      [i];
  }

  return *this;
}

const MotionInfo& CodingUnit::getMotionInfo() const
{
  return ctuData->motion[cs->inCtuPos( lumaPos(), CH_L )];
}

const MotionInfo& CodingUnit::getMotionInfo( const Position& pos ) const
{
  CHECKD( !Y().contains( pos ), "Trying to access motion info outsied of PU" );
  return ctuData->motion[cs->inCtuPos( pos, CH_L )];
}

MotionBuf CodingUnit::getMotionBuf()
{
  return MotionBuf( &ctuData->motion[cs->inCtuPos( lumaPos(), CH_L )], cs->get4x4MapStride(), g_miScaling.scaleHor( lwidth() ), g_miScaling.scaleVer( lheight() ) );
}

CMotionBuf CodingUnit::getMotionBuf() const
{
  return CMotionBuf( &getMotionInfo(), cs->get4x4MapStride(), g_miScaling.scaleHor( lwidth() ), g_miScaling.scaleVer( lheight() ) );
}

}
