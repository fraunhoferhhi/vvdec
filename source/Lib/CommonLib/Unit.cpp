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

/** \file     Unit.cpp
 *  \brief    defines unit as a set of blocks and basic unit types (coding, prediction, transform)
 */

#include "Unit.h"

#include "Buffer.h"
#include "Picture.h"
#include "ChromaFormat.h"

#include "UnitTools.h"
#include "UnitPartitioner.h"

 // ---------------------------------------------------------------------------
 // block method definitions
 // ---------------------------------------------------------------------------

Position CompArea::chromaPos( const ChromaFormat chromaFormat ) const
{
  if (isLuma(compID))
  {
    uint32_t scaleX = getComponentScaleX(compID, chromaFormat);
    uint32_t scaleY = getComponentScaleY(compID, chromaFormat);

    return Position(x >> scaleX, y >> scaleY);
  }
  else
  {
    return *this;
  }
}

Size CompArea::lumaSize( const ChromaFormat chromaFormat ) const
{
  if( isChroma( compID ) )
  {
    uint32_t scaleX = getComponentScaleX( compID, chromaFormat );
    uint32_t scaleY = getComponentScaleY( compID, chromaFormat );

    return Size( width << scaleX, height << scaleY );
  }
  else
  {
    return *this;
  }
}

Size CompArea::chromaSize( const ChromaFormat chromaFormat ) const
{
  if( isLuma( compID ) )
  {
    uint32_t scaleX = getComponentScaleX( compID, chromaFormat );
    uint32_t scaleY = getComponentScaleY( compID, chromaFormat );

    return Size( width >> scaleX, height >> scaleY );
  }
  else
  {
    return *this;
  }
}

Position CompArea::lumaPos( const ChromaFormat chromaFormat ) const
{
  if( isChroma( compID ) )
  {
    uint32_t scaleX = getComponentScaleX( compID, chromaFormat );
    uint32_t scaleY = getComponentScaleY( compID, chromaFormat );

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

  blocks[0].compID       = COMPONENT_Y;
  blocks[0].x            = _area.x;
  blocks[0].y            = _area.y;
  blocks[0].width        = _area.width;
  blocks[0].height       = _area.height;

  if( numCh == 1 ) return;

  const int csx = getChannelTypeScaleX( CH_C, chromaFormat );
  const int csy = getChannelTypeScaleY( CH_C, chromaFormat );
  
  blocks[1].compID       = COMPONENT_Cb;
  blocks[2].compID       = COMPONENT_Cr;
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
    if( blk.compID != compID )
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
    if (toChannelType(blk.compID) == chType)
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

  setBcwIdx   ( BCW_DEFAULT );
  intraDir[0] = DC_IDX;

  refIdx[0]   = refIdx[1] = -1;
}

// ---------------------------------------------------------------------------
// prediction unit method definitions
// ---------------------------------------------------------------------------

CodingUnit& CodingUnit::operator=( const MotionInfo& mi )
{
  setInterDir( mi.interDir );

  for( uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++ )
  {
    refIdx[i] = mi.refIdx[i];
    mv [i][0] = mi.mv    [i];
  }

  return *this;
}

const MotionInfo& CodingUnit::getMotionInfo() const
{
  return cs->getMotionInfo( lumaPos() );
}

const MotionInfo& CodingUnit::getMotionInfo( const Position& pos ) const
{
  CHECKD( !Y().contains( pos ), "Trying to access motion info outsied of PU" );
  return cs->getMotionInfo( pos );
}

MotionBuf CodingUnit::getMotionBuf()
{
  return cs->getMotionBuf( *this );
}

CMotionBuf CodingUnit::getMotionBuf() const
{
  return cs->getMotionBuf( *this );
}

// ---------------------------------------------------------------------------
// XUCache: unit allocation cache
// ---------------------------------------------------------------------------

std::shared_ptr<CUCache> ThreadSafeCUCache::getCuCache()
{
  std::unique_lock<std::mutex> l(m_mutex);
  for( auto & c: m_cuCaches )
  {
    // we know the cache instance is available, when there is only one shared_ptr reference (our own) to the element
    if( c.unique() )
    {
      return c;
    }
  }
  // no cache instance available -> create a new one
  m_cuCaches.push_back( std::make_shared<CUCache>() );
  return m_cuCaches.back();
}

std::shared_ptr<TUCache> ThreadSafeCUCache::getTuCache()
{
  std::unique_lock<std::mutex> l( m_mutex );
  for( auto & t : m_tuCaches )
  {
    // we know the cache instance is available, when there is only one shared_ptr reference (our own) to the element
    if( t.unique() )
    {
      return t;
    }
  }
  // no cache instance available -> create a new one
  m_tuCaches.push_back( std::make_shared<TUCache>() );
  return m_tuCaches.back();
}
