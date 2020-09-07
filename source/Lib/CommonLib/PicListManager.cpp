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


static bool isIRAP( NalUnitType type )
{
  return type == NAL_UNIT_CODED_SLICE_IDR_W_RADL || type == NAL_UNIT_CODED_SLICE_IDR_N_LP;
  // TODO: shouldn't this include CRAs? But then AI decoding is broken
//  return type >= NAL_UNIT_CODED_SLICE_BLA_W_LP && type <= NAL_UNIT_RESERVED_IRAP_VCL23;
}

static bool isIRAP( const Picture * pic )
{
  return isIRAP( pic->eNalUnitType );
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
    if( isIRAP( *itPic ) )
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

#if JVET_Q0814_DPB
Picture* PicListManager::getNewPicBuffer( const SPS& sps,const PPS& pps, const uint32_t temporalLayer, const int layerId, const VPS* vps )
#else
Picture* PicListManager::getNewPicBuffer( const SPS& sps,const PPS& pps, const uint32_t temporalLayer, const int layerId )
#endif
{
  CHECK( m_parseFrameDelay < 0, "Parser frame delay is invalid" );

  Picture*  pcPic         = nullptr;
#if JVET_Q0814_DPB
  const int iMaxRefPicNum = ( vps == nullptr || vps->m_numLayersInOls[vps->m_iTargetLayer] == 1 ) ? sps.getMaxDecPicBuffering( temporalLayer ) + 1 : vps->getMaxDecPicBuffering( temporalLayer );     // m_uiMaxDecPicBuffering has the space for the picture currently being decoded
#else
  const int iMaxRefPicNum = sps.getMaxDecPicBuffering( temporalLayer ) + 1;   // m_uiMaxDecPicBuffering has the space for the picture currently being decoded
#endif
  if( m_cPicList.size() < (uint32_t)iMaxRefPicNum + m_parseFrameDelay )
  {
    pcPic = new Picture();
    pcPic->create( sps.getChromaFormatIdc(),
                   Size( pps.getPicWidthInLumaSamples(), pps.getPicHeightInLumaSamples() ),
                   sps.getMaxCUWidth(),
                   sps.getMaxCUWidth() + 16
                 , layerId
                  );
    m_cPicList.push_back( pcPic );

    return pcPic;
  }

  for( PicList::iterator itPic = m_cPicList.begin(); itPic != m_cPicList.end(); ++itPic )
  {
    Picture* pic = *itPic;
    if( !pic->referenced && !pic->neededForOutput && !pic->lockedByApplication )
    {
      // take the picture to be reused and move it to the end of the list
      if (pic->cs->picHeader)
      {
          delete(pic->cs->picHeader);
      }
      m_cPicList.erase( itPic );
      m_cPicList.push_back( pic );
      pcPic = pic;
      break;
    }
  }

  if( !pcPic )
  {
    // There is no room for this picture, either because of faulty encoder or dropped NAL. Extend the buffer.
    pcPic = new Picture();
    pcPic->create( sps.getChromaFormatIdc(),
                   Size( pps.getPicWidthInLumaSamples(), pps.getPicHeightInLumaSamples() ),
                   sps.getMaxCUWidth(),
                   sps.getMaxCUWidth() + 16
                 , layerId
                  );
    m_cPicList.push_back( pcPic );

    return pcPic;
  }

  if( !pcPic->Y().Size::operator==( Size( pps.getPicWidthInLumaSamples(), pps.getPicHeightInLumaSamples() ) )
      || pcPic->cs->pcv->maxCUWidth != sps.getMaxCUWidth() || pcPic->cs->pcv->maxCUHeight != sps.getMaxCUHeight()
#if JVET_R0058
      || pcPic->layerId != layerId
#endif
    )
  {
    pcPic->destroy();
    pcPic->create( sps.getChromaFormatIdc(),
                   Size( pps.getPicWidthInLumaSamples(), pps.getPicHeightInLumaSamples() ),
                   sps.getMaxCUWidth(),
                   sps.getMaxCUWidth() + 16
                 , layerId
                  );
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
      int pocCycle     = 1 << ( checkRefPic->cs->sps->getBitsForPOC() );
      int refPocMasked = checkRefPic->poc & ( pocCycle - 1 );
      if( checkRefPic->longTerm && refPocMasked == rpl->getRefPicIdentifier( i ) )
      {
        checkRefPic->longTerm = true;
        return true;
      }
    }
  }
  return false;
}

void PicListManager::applyReferencePictureListBasedMarking( const Picture* currPic, const ReferencePictureList* rpl0, const ReferencePictureList* rpl1 )
{
  const bool noNeedToCheck = currPic->eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP ||
                             currPic->eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL;

  // loop through all pictures in the reference picture buffer
  for( auto& itPic: m_cPicList )
  {
    if( !itPic->referenced )
      continue;

    bool isReference = 0;
    if( !noNeedToCheck )
    {
      isReference = findInRefPicList( itPic, rpl0, currPic->getPOC() );
      if( !isReference )
      {
        isReference = findInRefPicList( itPic, rpl1, currPic->getPOC());
      }
    }

    // mark the picture as "unused for reference" if it is not in
    // the Reference Picture List
    if( itPic->reconstructed && itPic->poc != currPic->getPOC() && !isReference )
    {
      itPic->referenced = false;
      itPic->longTerm   = false;
      itPic->wasLost    = false;
    }
  }
}

void PicListManager::applyDoneReferencePictureMarking()
{
  Picture* lastDonePic = nullptr;

  PicList::iterator firstNotDonePic = std::find_if( m_cPicList.begin(), m_cPicList.end(), []( Picture* p ) { return p->done.isBlocked(); } );
  if( firstNotDonePic == m_cPicList.begin() )
  {
    // nothing done, yet
    return;
  }
  if( firstNotDonePic == m_cPicList.end() )
  {
    // all done
    lastDonePic = m_cPicList.back();
  }
  else
  {
    --firstNotDonePic;
    lastDonePic = *firstNotDonePic;
  }

  const bool noNeedToCheck = lastDonePic->eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP ||
                             lastDonePic->eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL;

  for( auto& itPic: m_cPicList )
  {
    if( !itPic->referenced )
    {
      continue;
    }
    if( itPic == lastDonePic )
    {
      return;
    }

    bool isReference = false;
    if( !noNeedToCheck )
    {
      for( auto& slice: lastDonePic->slices )
      {
        isReference = findInRefPicList( itPic, slice->getRPL0(), lastDonePic->getPOC() );
        if( !isReference )
        {
          isReference = findInRefPicList( itPic, slice->getRPL1(), lastDonePic->getPOC() );
        }
      }
    }

    // mark the picture as "unused for reference" if it is not in
    // the Reference Picture List
    CHECK( !itPic->reconstructed, "all pictures, for which we apply reference pic marking should have been reconstructed" )
    if( itPic->poc != lastDonePic->getPOC() && !isReference )
    {
      itPic->referenced = false;
      itPic->longTerm   = false;
      itPic->wasLost    = false;
    }
  }
}

Picture* PicListManager::findClosestPic( int iLostPoc )
{
  int      closestPoc = INT32_MAX;
  Picture* closestPic = nullptr;
  for( auto& rpcPic: m_cPicList )
  {
    if( rpcPic->reconstructed && abs( rpcPic->getPOC() - iLostPoc ) < closestPoc
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
    if( !(*itPic)->reconstructed )
    {
      seqEnd = itPic;
      break;
    }

    if( isIRAP( *itPic ) )
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
    if     ( !p->neededForOutput )                  stateC = 'o';
    else if( p->reconstructed )                     stateC = 'X';
    else if( p->inProgress )                        stateC = 'x';
    else if( !p->slices[0]->parseDone.isBlocked() ) stateC = '.';

    std::cout << p->poc << stateC << ' ';
  }
//  std::cout << std::endl;

//  std::cout << "range: ";
//  for( auto& p: picRange )
//  {
//    std::cout << p->poc << ( p->reconstructed ? "x" : " " ) << " ";
//  }
  std::cout << std::flush;
#endif

  if( m_tuneInDelay <= numReorderPicsHighestTid + m_parallelDecInst + 1 && !bFlush )
  {
    IF_DEBUG_PIC_ORDER( std::cout << std::endl );
    ++m_tuneInDelay;
    return nullptr;
  }

  // when there is an IRAP picture coming up, we can flush all pictures before that
  if( seqEnd != m_cPicList.cend() && isIRAP( *seqEnd ) )
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
      if( pcPic->neededForOutput && pcPic->reconstructed )
      {
        numPicsNotYetDisplayed++;
        dpbFullness++;
      }
      else if( pcPic->referenced && pcPic->reconstructed )   // !reconstructed means parsing started but not decoding
      {
        dpbFullness++;
      }
    }
  }

  IF_DEBUG_PIC_ORDER( std::cout << "   " << numPicsNotYetDisplayed << '/' << numReorderPicsHighestTid << " "<< dpbFullness << '/' << maxDecPicBufferingHighestTid << "   " );

  Picture * lowestPOCPic = nullptr;
  if( numPicsNotYetDisplayed > numReorderPicsHighestTid + ( m_firstOutputPic ? m_parallelDecInst + 1 : 0 )
//      || dpbFullness > maxDecPicBufferingHighestTid
      || bFlush )
  {

    for( auto& pcPic: picRange )
    {
      CHECK( pcPic->fieldPic, "Interlaced not suported" );

      if( pcPic->neededForOutput && pcPic->reconstructed &&
          ( lowestPOCPic==nullptr || pcPic->poc < lowestPOCPic->poc ) )
      {
        lowestPOCPic = pcPic;
      }
    }
  }

  if( lowestPOCPic )
  {
    m_firstOutputPic = false;
    lowestPOCPic->lockedByApplication = true;
    lowestPOCPic->neededForOutput = false;

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
