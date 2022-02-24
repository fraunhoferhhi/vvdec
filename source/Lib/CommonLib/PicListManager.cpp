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

Copyright (c) 2018-2022, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. 
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

#include "PicListManager.h"

#include "Picture.h"

#ifndef DEBUG_PIC_ORDER
#define DEBUG_PIC_ORDER 0
#endif
#if DEBUG_PIC_ORDER
# define IF_DEBUG_PIC_ORDER(...)__VA_ARGS__
#else
# define IF_DEBUG_PIC_ORDER(...)
#endif

namespace vvdec
{

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

void PicListManager::create(int frameDelay, int decInstances)
{
  m_parseFrameDelay = frameDelay;
  m_parallelDecInst = decInstances;
}

PicListRange PicListManager::getPicListRange( const Picture* pic ) const
{
  if( !pic )
  {
    return PicListRange{ m_cPicList.cbegin(), m_cPicList.cend() };
  }

  auto seqStart = m_cPicList.cbegin();
  for( auto itPic = m_cPicList.cbegin(); itPic != m_cPicList.cend(); ++itPic )
  {
    if( isIDR( *itPic ) && !(*itPic)->getMixedNaluTypesInPicFlag() )
    {
      seqStart = itPic;
    }

    if( *itPic == pic )
    {
      break;
    }
  }

  return PicListRange{ seqStart, m_cPicList.end() };
}

Picture* PicListManager::getNewPicBuffer( const SPS& sps, const PPS& pps, const uint32_t temporalLayer, const int layerId, const VPS* vps )
{
  CHECK( m_parseFrameDelay < 0, "Parser frame delay is invalid" );

  Picture*  pcPic         = nullptr;
  const int iMaxRefPicNum = ( vps == nullptr || vps->m_numLayersInOls[vps->m_iTargetLayer] == 1 )
                              ? sps.getMaxDecPicBuffering( temporalLayer ) + 1
                              : vps->getMaxDecPicBuffering( temporalLayer );   // m_uiMaxDecPicBuffering has the space for the picture currently being decoded
  if( m_cPicList.size() < (uint32_t)iMaxRefPicNum + m_parseFrameDelay )
  {
    pcPic = new Picture();
    pcPic->create( sps.getChromaFormatIdc(),
                   Size( pps.getPicWidthInLumaSamples(), pps.getPicHeightInLumaSamples() ),
                   sps.getMaxCUWidth(),
                   sps.getMaxCUWidth() + 16,
                   layerId );
    pcPic->createWrapAroundBuf( sps.getUseWrapAround(), sps.getMaxCUWidth() );
    m_cPicList.push_back( pcPic );

    return pcPic;
  }

  for( PicList::iterator itPic = m_cPicList.begin(); itPic != m_cPicList.end(); ++itPic )
  {
    Picture* pic = *itPic;
    if( pic->progress < Picture::reconstructed || pic->referenced || pic->neededForOutput || pic->lockedByApplication )
    {
      continue;
    }

    // take the picture to be reused and move it to the end of the list
    move_to_end( itPic, m_cPicList );
    pcPic = pic;
    break;
  }

  if( !pcPic )
  {
    // There is no room for this picture, either because of faulty encoder or dropped NAL. Extend the buffer.
    pcPic = new Picture();
    pcPic->create( sps.getChromaFormatIdc(),
                   Size( pps.getPicWidthInLumaSamples(), pps.getPicHeightInLumaSamples() ),
                   sps.getMaxCUWidth(),
                   sps.getMaxCUWidth() + 16,
                   layerId );
    pcPic->createWrapAroundBuf( sps.getUseWrapAround(), sps.getMaxCUWidth() );
    m_cPicList.push_back( pcPic );

    return pcPic;
  }

  if( pcPic->lumaSize() != Size( pps.getPicWidthInLumaSamples(), pps.getPicHeightInLumaSamples() )
      || pcPic->cs->pcv->maxCUWidth != sps.getMaxCUWidth()
      || pcPic->cs->pcv->maxCUHeight != sps.getMaxCUHeight()
      || pcPic->layerId != layerId
  )
  {
    pcPic->destroy();
    pcPic->create( sps.getChromaFormatIdc(),
                   Size( pps.getPicWidthInLumaSamples(), pps.getPicHeightInLumaSamples() ),
                   sps.getMaxCUWidth(),
                   sps.getMaxCUWidth() + 16,
                   layerId );
    pcPic->createWrapAroundBuf( sps.getUseWrapAround(), sps.getMaxCUWidth() );
  }

  pcPic->resetForUse();

  return pcPic;
}

static bool findInRefPicList( Picture* checkRefPic, const ReferencePictureList* rpl, int currPicPoc )
{
  // loop through all pictures in the Reference Picture Set
  // to see if the picture should be kept as reference picture
  for( int i = 0; i < rpl->getNumberOfShorttermPictures() + rpl->getNumberOfLongtermPictures(); i++ )
  {
    if( !rpl->isRefPicLongterm( i ) )
    {
      if( checkRefPic->poc == currPicPoc + rpl->getRefPicIdentifier( i ) )
      {
        checkRefPic->longTerm = false;
        return true;
      }
    }
    else
    {
      int refPoc = rpl->calcLTRefPOC( currPicPoc, checkRefPic->cs->sps->getBitsForPOC(), i );
      if( isLTPocEqual( checkRefPic->poc, refPoc, checkRefPic->cs->sps->getBitsForPOC(), rpl->getDeltaPocMSBPresentFlag( i ) ) )
      {
        checkRefPic->longTerm = true;
        return true;
      }
    }
  }
  return false;
}

void PicListManager::applyDoneReferencePictureMarking()
{
  Picture* lastDonePic = nullptr;
  for( auto& p: m_cPicList )
  {
    if( p->done.isBlocked() )
    {
      break;
    }
    lastDonePic = p;
  }
  if( !lastDonePic )
  {
    return;
  }

  const Picture* picRangeStart = *getPicListRange( lastDonePic ).begin();
  bool           inPicRange    = false;

  for( auto& itPic: m_cPicList )
  {
    if( itPic == lastDonePic )
    {
      // only check up to the last finished picture
      return;
    }
    if( !itPic->referenced )
    {
      // already marked as not references
      continue;
    }

    inPicRange |= ( itPic == picRangeStart );   // all pictures before the current valid picture-range can also be marked as not needed for referenece

    bool isReference = false;
    if( !( isIDR( lastDonePic ) && !lastDonePic->getMixedNaluTypesInPicFlag() ) || !inPicRange )
    {
      for( auto& slice: lastDonePic->slices )
      {
        if( findInRefPicList( itPic, slice->getRPL0(), lastDonePic->getPOC() ) || findInRefPicList( itPic, slice->getRPL1(), lastDonePic->getPOC() ) )
        {
          isReference = true;
          break;
        }
      }
    }

    // mark the picture as "unused for reference" if it is not in
    // the Reference Picture List
    CHECK( itPic->progress < Picture::reconstructed, "all pictures, for which we apply reference pic marking should have been reconstructed" )
    if( !isReference )
    {
      itPic->referenced = false;
      itPic->longTerm   = false;
      itPic->wasLost    = false;
      itPic->m_subPicRefBufs.clear();
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
    if( (*itPic)->progress < Picture::finished )
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
    if( !foundOutputPic && !(*itPic)->referenced )
    {
      seqStart = itPic;
    }
  }
  if( !foundOutputPic )
  {
    return nullptr;
  }

  PicListRange picRange{ seqStart, seqEnd };

#if DEBUG_PIC_ORDER
  std::cout << "list:  ";
  for( auto& p: m_cPicList )
  {
    char stateC = ' ';
    switch (p->progress) {
    case Picture::parsing:        stateC = 'p'; break;
    case Picture::parsed:         stateC = '.'; break;
    case Picture::reconstructing: stateC = 'x'; break;
    default:
    case Picture::reconstructed:
    case Picture::finished:       stateC = 'X';
      if( !p->neededForOutput )
      {
        stateC = 'o';
        if( p->lockedByApplication ) stateC = 'L';
        else if( p->referenced )     stateC = 'R';
      }
    }
    std::cout << p->poc << stateC << ' ';
  }
//  std::cout << std::endl;

//  std::cout << "range: ";
//  for( auto& p: picRange )
//  {
//    std::cout << p->poc << ( p->progress >= Picture::finished ? "x" : " " ) << " ";
//  }
  std::cout << std::flush;
#endif

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
      else if( pcPic->referenced && pcPic->progress >= Picture::finished )
      {
        dpbFullness++;
      }
    }
  }

  IF_DEBUG_PIC_ORDER( std::cout << "   " << numPicsNotYetDisplayed << '/' << numReorderPicsHighestTid << " "<< dpbFullness << '/' << maxDecPicBufferingHighestTid << "   " );

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

}
