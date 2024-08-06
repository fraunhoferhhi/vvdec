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

#include "PicListManager.h"

#include "Picture.h"

#ifndef DEBUG_PIC_ORDER
#  define DEBUG_PIC_ORDER 0
#endif
#if DEBUG_PIC_ORDER
#  define IF_DEBUG_PIC_ORDER( ... ) __VA_ARGS__
#else
#  define IF_DEBUG_PIC_ORDER( ... )
#endif

namespace vvdec
{
static std::ostream& operator<<( std::ostream& strm, PicList& picList )
{
  for( auto& p: picList )
  {
    char stateC = ' ';
    switch (p->progress) {
    case Picture::init:           stateC = ' '; break;
    case Picture::parsing:        stateC = 'p'; break;
    case Picture::parsed:         stateC = '.'; break;
    case Picture::reconstructing: stateC = 'x'; break;
    case Picture::reconstructed:
    case Picture::finished:       stateC = 'X';
      if( !p->neededForOutput )
      {
        stateC = 'o';
        if( p->lockedByApplication )   stateC = 'L';
        else if( p->dpbReferenceMark ) stateC = 'R';
      }
    }
    strm << p->poc << stateC << ' ';
  }

  return strm;
}

static std::ostream& operator<<( std::ostream& strm, PicListRange& picRange )
{
  for( auto& p: picRange )
  {
    strm << p->poc << ( p->progress >= Picture::finished ? "x" : " " ) << " ";
  }
  return strm;
}

static bool isIDR( NalUnitType type )
{
  return type == NAL_UNIT_CODED_SLICE_IDR_W_RADL || type == NAL_UNIT_CODED_SLICE_IDR_N_LP;
}
static bool isIDR( const Picture* pic )
{
  return isIDR( pic->eNalUnitType );
}

void PicListManager::deleteBuffers()
{
  for( auto& pcPic: m_cPicList )
  {
    pcPic->destroy();

    delete pcPic;
    pcPic = NULL;
  }
  m_cPicList.clear();
}

void PicListManager::create(int frameDelay, int decInstances, const UserAllocator& userAllocator )
{
  m_parseFrameDelay      = frameDelay;
  m_parallelDecInst      = decInstances;
  m_userAllocator        = userAllocator;
}

Picture* PicListManager::getNewPicBuffer( const SPS& sps, const PPS& pps, const uint32_t temporalLayer, const int layerId, const VPS* vps )
{
  CHECK_FATAL( m_parseFrameDelay < 0, "Parser frame delay is invalid" );

  Picture*  pcPic         = nullptr;
  const int iMaxRefPicNum = ( vps == nullptr || vps->m_numLayersInOls[vps->m_iTargetLayer] == 1 )
                              ? sps.getMaxDecPicBuffering( temporalLayer ) + 1
                              : vps->getMaxDecPicBuffering( temporalLayer );   // m_uiMaxDecPicBuffering has the space for the picture currently being decoded
  const unsigned int picMargin = 16 + sps.getMaxCUWidth();

  bool externAllocator = m_userAllocator.enabled;
  if ( externAllocator )
  {
    if (sps.getBitDepth() == 8 )
    {
      externAllocator = false;
    }
  }
  UserAllocator* userAllocator = externAllocator ? &m_userAllocator : nullptr;

  if( m_cPicList.size() < (uint32_t)iMaxRefPicNum + m_parseFrameDelay )
  {
    pcPic = new Picture();
    pcPic->create( sps.getChromaFormatIdc(),
                   Size( pps.getPicWidthInLumaSamples(), pps.getPicHeightInLumaSamples() ),
                   sps.getMaxCUWidth(),
                   picMargin,
                   layerId,
                   userAllocator );
    pcPic->createWrapAroundBuf( sps.getUseWrapAround(), sps.getMaxCUWidth() );
    m_cPicList.push_back( pcPic );

    return pcPic;
  }

  for( PicList::iterator itPic = m_cPicList.begin(); itPic != m_cPicList.end(); ++itPic )
  {
    Picture* pic = *itPic;
    if( pic->progress < Picture::finished || pic->stillReferenced || pic->dpbReferenceMark || pic->neededForOutput || pic->lockedByApplication )
    {
      continue;
    }

    if ( externAllocator )
    {
      pic->destroy();
      pic->create( sps.getChromaFormatIdc(),
                   Size( pps.getPicWidthInLumaSamples(), pps.getPicHeightInLumaSamples() ),
                   sps.getMaxCUWidth(),
                   picMargin,
                   layerId,
                   userAllocator );
      pic->createWrapAroundBuf( sps.getUseWrapAround(), sps.getMaxCUWidth() );
      pic->resetForUse( layerId );
    }

    // take the picture to be reused and move it to the end of the list
    move_to_end( itPic, m_cPicList );

    pcPic = pic;

    if ( externAllocator )
    {
      return pcPic;
    }

    break;
  }

  if( !pcPic )
  {
    // There is no room for this picture, either because of faulty encoder or dropped NAL. Extend the buffer.
    pcPic = new Picture();
    pcPic->create( sps.getChromaFormatIdc(),
                   Size( pps.getPicWidthInLumaSamples(), pps.getPicHeightInLumaSamples() ),
                   sps.getMaxCUWidth(),
                   picMargin,
                   layerId,
                   userAllocator );
    pcPic->createWrapAroundBuf( sps.getUseWrapAround(), sps.getMaxCUWidth() );
    m_cPicList.push_back( pcPic );

    return pcPic;
  }

  if( pcPic->lumaSize() != Size( pps.getPicWidthInLumaSamples(), pps.getPicHeightInLumaSamples() )
      || pcPic->cs->pcv->maxCUWidth != sps.getMaxCUWidth()
      || pcPic->cs->pcv->maxCUHeight != sps.getMaxCUHeight()
      || pcPic->layerId != layerId
      || pcPic->margin != picMargin
  )
  {
    pcPic->destroy();
    pcPic->create( sps.getChromaFormatIdc(),
                   Size( pps.getPicWidthInLumaSamples(), pps.getPicHeightInLumaSamples() ),
                   sps.getMaxCUWidth(),
                   picMargin,
                   layerId,
                   userAllocator );
    pcPic->createWrapAroundBuf( sps.getUseWrapAround(), sps.getMaxCUWidth() );
  }

  pcPic->resetForUse( layerId );

  return pcPic;
}

void PicListManager::markUnusedPicturesReusable()
{
  // collect all pictures, that are still referenced by an in-progress picture
  m_allRefPics.clear();
  m_allRefPics.reserve( m_cPicList.size() );
  for( auto& pic: m_cPicList )
  {
    if( pic->progress >= Picture::reconstructed )
    {
      continue;
    }

    for( auto& s: pic->slices )
    {
      for( auto l: { REF_PIC_LIST_0, REF_PIC_LIST_1 } )
      {
        for( int iRefIdx = 0; iRefIdx < s->getNumRefIdx( l ); iRefIdx++ )
        {
          const Picture* refPic = s->getRefPic( l, iRefIdx );
          m_allRefPics.insert( refPic );
        }
      }
    }
  }

  // remove stillReferenced flag from all others
  for( auto& pic: m_cPicList )
  {
    if( pic->progress < Picture::finished )   // only unmark pictures up to the first unfinished pic
    {
      break;
    }

    if( pic->stillReferenced && !pic->dpbReferenceMark && m_allRefPics.count( pic ) == 0 )
    {
      pic->stillReferenced = false;
      pic->m_subPicRefBufs.clear();
    }
  }
}

Picture* PicListManager::findClosestPic( int iLostPoc )
{
  int      closestPoc = INT32_MAX;
  Picture* closestPic = nullptr;
  for( auto& rpcPic: m_cPicList )
  {
    if( rpcPic->progress >= Picture::reconstructed && abs( rpcPic->getPOC() - iLostPoc ) < closestPoc
        && abs( rpcPic->getPOC() - iLostPoc ) != 0 )
    {
      closestPoc = abs( rpcPic->getPOC() - iLostPoc );
      closestPic = rpcPic;
    }
  }
  return closestPic;
}

Picture* PicListManager::getNextOutputPic( uint32_t numReorderPicsHighestTid,
                                           uint32_t maxDecPicBufferingHighestTid,
                                           bool     bFlush )
{
  (void)maxDecPicBufferingHighestTid; // unused

  if( m_cPicList.empty() )
  {
    return nullptr;
  }

  // find picture range up to the first random access point
  auto seqStart = m_cPicList.cbegin();
  auto seqEnd   = m_cPicList.cend();

  bool foundOutputPic = false;
  for( auto itPic = seqStart; itPic != m_cPicList.cend(); ++itPic )
  {
    if( !( *itPic )->neededForOutput && ( *itPic )->progress >= Picture::reconstructed )
    {
      continue;
    }

    if( ( *itPic )->progress < Picture::finished )
    {
      seqEnd = itPic;
      break;
    }

    if( isIDR( *itPic ) && !(*itPic)->getMixedNaluTypesInPicFlag())
    {
      if( !foundOutputPic ) // if there was no picture needed for output before the first RAP,
      {                     // we begin the range at the RAP...
        seqStart = itPic;
      }
      else                  // ...otherwise it ends at the RAP
      {
        seqEnd = itPic;
        break;
      }
    }

    foundOutputPic |= (*itPic)->neededForOutput;

    // ignore pictures, that are not needed for output or referenced any more
    if( !foundOutputPic && !(*itPic)->dpbReferenceMark )
    {
      seqStart = itPic;
    }
  }
  if( !foundOutputPic )
  {
    return nullptr;
  }

  PicListRange picRange{ seqStart, seqEnd };

  IF_DEBUG_PIC_ORDER( std::cout << "list:  " << m_cPicList << std::flush );
//  IF_DEBUG_PIC_ORDER( std::cout << std::endl << "range: " << picRange << std::flush );

  if( m_tuneInDelay <= numReorderPicsHighestTid + m_parallelDecInst + 1 && !bFlush )
  {
    IF_DEBUG_PIC_ORDER( std::cout << std::endl );
    ++m_tuneInDelay;
    return nullptr;
  }

  // when there is an IDR picture coming up, we can flush all pictures before that
  if( seqEnd != m_cPicList.cend() && isIDR( *seqEnd ) && !(*seqEnd)->getMixedNaluTypesInPicFlag() )
  {
    bFlush = true;
    IF_DEBUG_PIC_ORDER( std::cout << " flush" );
  }

  unsigned numPicsNotYetDisplayed = 0;
  unsigned dpbFullness            = 0;
  if( !bFlush )
  {
    for( auto& pcPic: picRange )
    {
      if( pcPic->neededForOutput && pcPic->progress >= Picture::finished )
      {
        numPicsNotYetDisplayed++;
        dpbFullness++;
      }
      else if( pcPic->dpbReferenceMark && pcPic->progress >= Picture::finished )
      {
        dpbFullness++;
      }
    }
  }

  IF_DEBUG_PIC_ORDER( std::cout << "   " << numPicsNotYetDisplayed << '/' << numReorderPicsHighestTid << " "<< dpbFullness << '/' << maxDecPicBufferingHighestTid << "   " );
  (void)dpbFullness;

  Picture * lowestPOCPic = nullptr;
  if( numPicsNotYetDisplayed > numReorderPicsHighestTid + ( m_firstOutputPic ? MAX_OUT_OF_ORDER_PICS : 0 )
//      || dpbFullness > maxDecPicBufferingHighestTid
      || bFlush )
  {

    for( auto& pcPic: picRange )
    {
      //CHECK( pcPic->fieldPic, "Interlaced not suported" );

      if( pcPic->neededForOutput && pcPic->progress >= Picture::finished &&
          ( lowestPOCPic==nullptr || pcPic->poc < lowestPOCPic->poc ) )
      {
        lowestPOCPic = pcPic;
      }
    }
  }

  if( lowestPOCPic )
  {
    m_firstOutputPic                  = false;
    lowestPOCPic->lockedByApplication = true;
    lowestPOCPic->neededForOutput     = false;

    IF_DEBUG_PIC_ORDER( std::cout << " ==> " << lowestPOCPic->poc );
  }
  IF_DEBUG_PIC_ORDER( std::cout << std::endl );

  return lowestPOCPic;
}

void PicListManager::releasePicture( Picture* pic )
{
  if( pic )
  {
    pic->lockedByApplication = false;
  }
}

}   // namespace vvdec
