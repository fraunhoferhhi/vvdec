/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2018-2022, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVdeC Authors.
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

#include "DecLibParser.h"
#include "DecLib.h"
#include "NALread.h"

#include "CommonLib/dtrace_next.h"
#include "CommonLib/dtrace_buffer.h"

#include "Utilities/ThreadPool.h"

#include <unordered_map>
#include <utility>

namespace vvdec
{
template<class TArr, class TElem = decltype( std::declval<TArr>()[0] )>
void fill_array( TArr& array, const TElem& val )
{
  std::fill( std::begin( array ), std::end( array ), val );
}

#ifdef TRACE_ENABLE_ITT
extern __itt_domain* itt_domain_prs;
extern __itt_domain* itt_domain_oth;

extern __itt_string_handle* itt_handle_parse;
extern __itt_string_handle* itt_handle_start;

#define ITT_TASKSTART( d, t ) __itt_task_begin( ( d ), __itt_null, __itt_null, ( t ) )
#define ITT_TASKEND( d, t )   __itt_task_end  ( ( d ) )
#else
#define ITT_TASKSTART( d, t )
#define ITT_TASKEND( d, t )
#endif

static const int INIT_POC = -MAX_INT;

DecLibParser::~DecLibParser()
{
  m_prefixSEINALUs.clear();

  destroy();
}

void DecLibParser::create( ThreadPool* tp, int parserFrameDelay, int numReconInst, int numDecThreads )
{
  m_threadPool        = tp;
  m_parseFrameDelay   = parserFrameDelay;
  m_numDecThreads     = numDecThreads;
  m_maxPicReconSkip   = numReconInst - 1;

  m_apcSlicePilot     = new Slice;
  m_picHeader         = std::make_shared<PicHeader>();
  m_uiSliceSegmentIdx = 0;

  fill_array( m_associatedIRAPType,     NAL_UNIT_INVALID );
  fill_array( m_pocCRA,                 INIT_POC );
  fill_array( m_gdrRecoveryPointPocVal, INIT_POC );
  fill_array( m_gdrRecovered,           false );

  m_cSliceDecoder.setContextStateVec( numDecThreads );
}

void DecLibParser::destroy()
{
  SEI_internal::deleteSEIs( m_seiMessageList );

  m_threadPool    = nullptr;

  delete m_apcSlicePilot;
  m_apcSlicePilot = nullptr;
  
  m_picHeader.reset();
  
  m_cSliceDecoder.destroy();
  
  if( m_dci )
  {
    delete m_dci;
    m_dci = NULL;
  }
}

void DecLibParser::recreateLostPicture( Picture* pcPic )
{
  auto closestPic = m_picListManager.findClosestPic( pcPic->poc );
  CHECK( !closestPic,                                   "found no Picture to replace lost Picture" );
  CHECK( closestPic->progress < Picture::reconstructed, "closest Picture is not yet reconstructed" )
  if( closestPic )
  {
    // the next not-lost picture in the parseFrameList should be the one, that referenced this picture
    auto referencedBy = std::find_if( m_parseFrameList.begin(), m_parseFrameList.end(),
                                      []( Picture* p ) { return !p->wasLost; } );
    msg( INFO, "copying picture %d to %d (%d)\n",
         closestPic->getPOC(),
         pcPic->poc,
         ( referencedBy != m_parseFrameList.end() ) ? ( *referencedBy )->poc : -1 );

    pcPic->getRecoBuf().copyFrom( closestPic->getRecoBuf() );
    for( int i = 0; i < pcPic->cs->m_ctuDataSize; i++ )
    {
      memcpy( pcPic->cs->m_ctuData[i].colMotion, closestPic->cs->m_ctuData[i].colMotion, sizeof( ColocatedMotionInfo ) * pcPic->cs->pcv->num8x8CtuBlks );
    }

    pcPic->slices[0]->copySliceInfo( closestPic->slices[0] );
    pcPic->slices[0]->setPOC( pcPic->poc );

    pcPic->progress = Picture::reconstructed;

    pcPic->parseDone.unlock();
    pcPic->done.unlock();
  }
}

Picture* DecLibParser::parse( InputNALUnit& nalu, int* pSkipFrame, int iTargetLayer )
{
  // ignore all NAL units of layers > 0
  if( m_iTargetLayer >= 0 && nalu.m_nuhLayerId != m_iTargetLayer )   // TBC: ignore bitstreams whose nuh_layer_id is not the target layer id
  {
    msg( WARNING, "Warning: found NAL unit with nuh_layer_id equal to %d. Ignoring.\n", nalu.m_nuhLayerId );
    return nullptr;
  }

  if( !nalu.isVcl() )
  {
    if( nalu.m_nalUnitType == NAL_UNIT_SUFFIX_APS || nalu.m_nalUnitType == NAL_UNIT_SUFFIX_SEI )
    {
      if( m_pcParsePic )
      {
        m_pcParsePic->bits += nalu.m_bits;
      }
    }
    else
    {
      m_nonVCLbits += nalu.m_bits;
    }
  }

  GCC_EXTRA_WARNING_switch_enum;
  switch( nalu.m_nalUnitType )
  {
  case NAL_UNIT_CODED_SLICE_TRAIL:
  case NAL_UNIT_CODED_SLICE_STSA:
  case NAL_UNIT_CODED_SLICE_RADL:
  case NAL_UNIT_CODED_SLICE_RASL:
  case NAL_UNIT_CODED_SLICE_IDR_W_RADL:
  case NAL_UNIT_CODED_SLICE_IDR_N_LP:
  case NAL_UNIT_CODED_SLICE_CRA:
  case NAL_UNIT_CODED_SLICE_GDR:
  {
    const auto ret = xDecodeSliceHead( nalu, pSkipFrame );

    m_parseNewPicture = ( ret == NewPicture );

    if( ret == NewPicture && m_parseFrameList.size() > m_parseFrameDelay )
    {
      Picture* pic = getNextDecodablePicture();
      return pic;
    }
    return nullptr;
  }

  case NAL_UNIT_OPI:
    // NOT IMPLEMENTED
    return nullptr;

  case NAL_UNIT_DCI:
    xDecodeDCI( nalu );
    return nullptr;

  case NAL_UNIT_VPS:
    xDecodeVPS( nalu );
    // m_vps->m_iTargetLayer = iTargetOlsIdx;
    return nullptr;

  case NAL_UNIT_SPS:
    xDecodeSPS( nalu );
    return nullptr;

  case NAL_UNIT_PPS:
    xDecodePPS( nalu );
    return nullptr;

  case NAL_UNIT_PREFIX_APS:
  case NAL_UNIT_SUFFIX_APS:
    xDecodeAPS( nalu );
    return nullptr;

  case NAL_UNIT_PH:
    xDecodePicHeader( nalu );
    return nullptr;

  case NAL_UNIT_ACCESS_UNIT_DELIMITER:
  {
    AUDReader audReader;
    uint32_t  picType;
    audReader.parseAccessUnitDelimiter( &( nalu.getBitstream() ), picType );
    msg( NOTICE, "Note: found NAL_UNIT_ACCESS_UNIT_DELIMITER\n" );
    return nullptr;
  }

  case NAL_UNIT_EOS:
    m_associatedIRAPType[nalu.m_nuhLayerId]     = NAL_UNIT_INVALID;
    m_pocCRA[nalu.m_nuhLayerId]                 = INIT_POC;
    m_gdrRecoveryPointPocVal[nalu.m_nuhLayerId] = INIT_POC;
    m_gdrRecovered[nalu.m_nuhLayerId]           = false;
    m_pocRandomAccess                           = MAX_INT;
    m_prevPOC                                   = MAX_INT;
    m_prevSliceSkipped                          = false;
    m_skippedPOC                                = 0;
    setFirstSliceInSequence( true, nalu.m_nuhLayerId );
    setFirstSliceInPicture( true );
    m_picListManager.restart();
    return nullptr;

  case NAL_UNIT_EOB:
    return nullptr;

  case NAL_UNIT_PREFIX_SEI:
    // Buffer up prefix SEI messages until SPS of associated VCL is known.
    m_prefixSEINALUs.emplace_back( nalu );
    m_pictureSeiNalus.emplace_back( nalu );
    return nullptr;

  case NAL_UNIT_SUFFIX_SEI:
    if( m_pcParsePic )
    {
      m_pictureSeiNalus.emplace_back( nalu );
      const SPS* sps = m_parameterSetManager.getActiveSPS();
      const VPS* vps = m_parameterSetManager.getVPS( sps->getVPSId() );
      m_seiReader.parseSEImessage( &( nalu.getBitstream() ), m_pcParsePic->seiMessageList, nalu.m_nalUnitType, nalu.m_nuhLayerId, nalu.m_temporalId, vps, sps, m_HRD, m_pDecodedSEIOutputStream );

      if( m_parseFrameDelay == 0 )   // else it has to be done in finishPicture()
      {
        // if parallel parsing is disabled, wait for the picture to finish
        if( m_threadPool->numThreads() == 0 )
        {
          m_threadPool->processTasksOnMainThread();
        }
        m_pcParsePic->done.wait();
        m_decLib.checkPictureHashSEI( m_pcParsePic );
      }
    }
    else
    {
      msg( NOTICE, "Note: received suffix SEI but no picture currently active.\n" );
    }
    return nullptr;

  case NAL_UNIT_FD:
    return nullptr;

  case NAL_UNIT_RESERVED_VCL_4:
  case NAL_UNIT_RESERVED_VCL_5:
  case NAL_UNIT_RESERVED_VCL_6:
  case NAL_UNIT_RESERVED_IRAP_VCL_11:
    msg( NOTICE, "Note: found reserved VCL NAL unit.\n" );
    xParsePrefixSEIsForUnknownVCLNal();
    return nullptr;
  case NAL_UNIT_RESERVED_NVCL_26:
  case NAL_UNIT_RESERVED_NVCL_27:
    msg( NOTICE, "Note: found reserved NAL unit.\n" );
    return nullptr;
  case NAL_UNIT_UNSPECIFIED_28:
  case NAL_UNIT_UNSPECIFIED_29:
  case NAL_UNIT_UNSPECIFIED_30:
  case NAL_UNIT_UNSPECIFIED_31:
    msg( NOTICE, "Note: found unspecified NAL unit.\n" );
    return nullptr;
  case NAL_UNIT_INVALID:
  default:
    THROW( "Invalid NAL unit type" );
    break;
  }
  GCC_WARNING_RESET;

  return nullptr;
}

Picture* DecLibParser::getNextDecodablePicture()
{
  if( m_parseFrameList.empty() )
  {
    return nullptr;
  }

  if( m_threadPool->numThreads() == 0 || m_parseFrameDelay == 0 )
  {
    // adhere to strict decoding order if running singlethreaded
    Picture * pic = m_parseFrameList.front();
    m_parseFrameList.pop_front();
    return pic;
  }

  if( m_parseFrameList.front()->skippedDecCount >= MAX_OUT_OF_ORDER_PICS )
  {
    Picture* pic = m_parseFrameList.front();
    m_parseFrameList.pop_front();
    return pic;
  }

  // try to find next picture, that is parsed and has all reference pictures decoded
  for( auto picIt = m_parseFrameList.begin(); picIt != m_parseFrameList.end(); ++picIt )
  {
    Picture* pic = *picIt;

    if( pic->parseDone.isBlocked() )
      continue;

    bool allRefPicsDone = true;
    for( const Slice* slice: pic->slices )
    {
      if( slice->isIntra() )
      {
        continue;
      }
      for( int iDir = REF_PIC_LIST_0; iDir < NUM_REF_PIC_LIST_01 && allRefPicsDone; ++iDir )
      {
        for( int iRefIdx = 0; iRefIdx < slice->getNumRefIdx( (RefPicList) iDir ) && allRefPicsDone; iRefIdx++ )
        {
          const Picture* refPic = slice->getRefPic( (RefPicList) iDir, iRefIdx );
          if( refPic->done.isBlocked() )
          {
            allRefPicsDone = false;
            break;
          }
        }
      }
    }

    if( allRefPicsDone )
    {
      // increase skip count for all previous pictures
      for( auto& skipped: PicListRange{ m_parseFrameList.begin(), picIt } )
      {
        skipped->skippedDecCount++;
      }

      m_parseFrameList.erase( picIt );
      return pic;
    }

//    if( pic->getTLayer() < m_parseFrameList.front()->getTLayer() )
//    {
//      break;
//    }
  }

  // if no picture has all reference-pictures decoded, use next pic in (regular) decoding order.
  Picture * pic = m_parseFrameList.front();
  m_parseFrameList.pop_front();
  return pic;
}

void DecLibParser::checkAPSInPictureUnit()
{
  bool firstVCLFound  = false;
  bool suffixAPSFound = false;

  for( auto &nalu : m_pictureUnitNals )
  {
    if( NALUnit::isVclNalUnitType(nalu) )
    {
      firstVCLFound = true;
      CHECK( suffixAPSFound, "When any suffix APS NAL units are present in a PU, they shall follow the last VCL unit of the PU" );
    }
    else if( nalu == NAL_UNIT_PREFIX_APS )
    {
      CHECK( firstVCLFound, "When any prefix APS NAL units are present in a PU, they shall precede the first VCL unit of the PU");
    }
    else if( nalu == NAL_UNIT_SUFFIX_APS )
    {
      suffixAPSFound = true;
    }
  }
}

DecLibParser::SliceHeadResult DecLibParser::xDecodeSliceHead( InputNALUnit& nalu, int* pSkipFrame )
{
  m_apcSlicePilot->initSlice(); // the slice pilot is an object to prepare for a new slice
                                // it is not associated with picture, sps or pps structures.


  m_apcSlicePilot->setNalUnitType   ( nalu.m_nalUnitType );
  m_apcSlicePilot->setNalUnitLayerId( nalu.m_nuhLayerId  );
  m_apcSlicePilot->setTLayer        ( nalu.m_temporalId  );

  m_HLSReader.setBitstream( &nalu.getBitstream() );

  m_bFirstSliceInPicture = true;
  m_HLSReader.parseSliceHeader( m_apcSlicePilot, m_picHeader.get(), &m_parameterSetManager, m_prevTid0POC, m_pcParsePic, m_bFirstSliceInPicture );

  if( pSkipFrame && *pSkipFrame )
  {
    --( *pSkipFrame );   // decrement the counter
    m_prevSliceSkipped = true;
    m_skippedPOC       = m_apcSlicePilot->getPOC();
    return SkipPicture;
  }

  CHECK( m_bFirstSliceInPicture != ( m_apcSlicePilot->getCtuAddrInSlice( 0 ) == 0 ), "first slice in picture should start at CTU-addr 0" );

  if( m_bFirstSliceInPicture )
  {
    m_uiSliceSegmentIdx = 0;
    m_apcSlicePilot->setPicHeader   ( m_picHeader.get() );
  }
  else // if it turns out, this was not the first slice in the picture, we need to parse the header again
  {
    CHECK( m_uiSliceSegmentIdx == 0, "slice segment idx should only be zero for first slice in picture" );
    CHECK( !m_pcParsePic->getMixedNaluTypesInPicFlag() && nalu.m_nalUnitType != m_pcParsePic->slices[m_uiSliceSegmentIdx - 1]->getNalUnitType(),
           "The value of NAL unit type shall be the same for all coded slice NAL units of a picture if pps_mixed_nalu_types_in_pic_flag is not set" )
    m_apcSlicePilot->copySliceInfo  ( m_pcParsePic->slices[m_uiSliceSegmentIdx-1] );

    m_apcSlicePilot->setNalUnitType   ( nalu.m_nalUnitType );
    m_apcSlicePilot->setNalUnitLayerId( nalu.m_nuhLayerId );
    m_apcSlicePilot->setTLayer        ( nalu.m_temporalId );

    nalu.getBitstream().resetToStart();
    nalu.readNalUnitHeader();
    m_HLSReader.setBitstream        ( &nalu.getBitstream() );

    m_HLSReader.parseSliceHeader( m_apcSlicePilot, m_picHeader.get(), &m_parameterSetManager, m_prevTid0POC, m_pcParsePic, m_bFirstSliceInPicture );
  }
  
  PPS *pps = m_parameterSetManager.getPPS( m_apcSlicePilot->getPicHeader()->getPPSId() );
  CHECK( pps == 0, "No PPS present" );
  SPS *sps = m_parameterSetManager.getSPS( pps->getSPSId() );
  CHECK( sps == 0, "No SPS present" );
  VPS *vps = m_parameterSetManager.getVPS( sps->getVPSId() );
  CHECK( sps->getVPSId() > 0 && vps == 0, "Invalid VPS" );
  if( sps->getVPSId() == 0 && m_prevLayerID != MAX_INT )
  {
    CHECK( m_prevLayerID != nalu.m_nuhLayerId, "All VCL NAL unit in the CVS shall have the same value of nuh_layer_id "
                                               "when sps_video_parameter_set_id is equal to 0" );
  }

  if( vps != nullptr && ( vps->getIndependentLayerFlag( nalu.m_nuhLayerId ) == 0 ) )
  {
    bool pocIsSet = false;
    for( auto auNALit = m_accessUnitPicInfo.begin(); auNALit != m_accessUnitPicInfo.end(); auNALit++ )
    {
      for( int iRefIdx = 0; iRefIdx < m_apcSlicePilot->getNumRefIdx( REF_PIC_LIST_0 ) && !pocIsSet; iRefIdx++ )
      {
        if( m_apcSlicePilot->getRefPic( REF_PIC_LIST_0, iRefIdx ) && m_apcSlicePilot->getRefPic( REF_PIC_LIST_0, iRefIdx )->getPOC() == ( *auNALit ).m_POC )
        {
          m_apcSlicePilot->setPOC( m_apcSlicePilot->getRefPic( REF_PIC_LIST_0, iRefIdx )->getPOC() );
          pocIsSet = true;
        }
      }
      for( int iRefIdx = 0; iRefIdx < m_apcSlicePilot->getNumRefIdx( REF_PIC_LIST_1 ) && !pocIsSet; iRefIdx++ )
      {
        if( m_apcSlicePilot->getRefPic( REF_PIC_LIST_1, iRefIdx ) && m_apcSlicePilot->getRefPic( REF_PIC_LIST_1, iRefIdx )->getPOC() == ( *auNALit ).m_POC )
        {
          m_apcSlicePilot->setPOC( m_apcSlicePilot->getRefPic( REF_PIC_LIST_1, iRefIdx )->getPOC() );
          pocIsSet = true;
        }
      }
    }
  }

  // update independent slice index
  uint32_t uiIndependentSliceIdx = 0;
  if( !m_bFirstSliceInPicture )
  {
    uiIndependentSliceIdx = m_pcParsePic->slices[m_uiSliceSegmentIdx - 1]->getIndependentSliceIdx();
    uiIndependentSliceIdx++;
  }
  m_apcSlicePilot->setIndependentSliceIdx( uiIndependentSliceIdx );

  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "poc", m_apcSlicePilot->getPOC() ) );

#if !DISABLE_CHECK_NO_OUTPUT_PRIOR_PICS_FLAG
  if( ( m_bFirstSliceInPicture || m_apcSlicePilot->isCRAorGDR() ) && getNoOutputPriorPicsFlag() )
  {
    checkNoOutputPriorPics();
    setNoOutputPriorPicsFlag( false );
  }
#endif

  if( m_bFirstSliceInPicture )
  {
    const auto pictureType = m_apcSlicePilot->getNalUnitType();

    if( !pps->getMixedNaluTypesInPicFlag()
        && ( pictureType == NAL_UNIT_CODED_SLICE_IDR_W_RADL
             || pictureType == NAL_UNIT_CODED_SLICE_IDR_N_LP
             || pictureType == NAL_UNIT_CODED_SLICE_CRA
             || pictureType == NAL_UNIT_CODED_SLICE_GDR ) )
    {
      m_pocCRA            [nalu.m_nuhLayerId] = m_apcSlicePilot->getPOC();
      m_associatedIRAPType[nalu.m_nuhLayerId] = pictureType;
    }
  }

  xUpdatePreviousTid0POC( m_apcSlicePilot );

  m_apcSlicePilot->setAssociatedIRAPPOC ( m_pocCRA            [nalu.m_nuhLayerId] );
  m_apcSlicePilot->setAssociatedIRAPType( m_associatedIRAPType[nalu.m_nuhLayerId] );

  // For inference of NoOutputOfPriorPicsFlag
  //
  // TODO: (GH) Currently setting NoOutputBeforeRecoveryFlag is more or less disabled as a workaround for missing
  //       output pictures du to parallel parsing. See: https://github.com/fraunhoferhhi/vvdec/issues/17
  //       This should be implemented properly, so we can correctly detect the start of a new CLVS.
  //
  if( /*m_apcSlicePilot->getRapPicFlag() ||*/ m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR )
  {
    if( !pps->getMixedNaluTypesInPicFlag() )
    {
      if( m_bFirstSliceInSequence[nalu.m_nuhLayerId] )
      {
        m_picHeader->setNoOutputBeforeRecoveryFlag( true );
      }
      else if( m_apcSlicePilot->getIdrPicFlag() )
      {
        m_picHeader->setNoOutputBeforeRecoveryFlag( true );
      }
      else if( m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA )
      {
        m_picHeader->setNoOutputBeforeRecoveryFlag( m_picHeader->getHandleCraAsCvsStartFlag() );
      }
      else if( m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR )
      {
        m_picHeader->setNoOutputBeforeRecoveryFlag( m_picHeader->getHandleGdrAsCvsStartFlag() );
      }
    }

    //the inference for NoOutputOfPriorPicsFlag
    if( !m_bFirstSliceInBitstream && m_picHeader->getNoOutputBeforeRecoveryFlag() )
    {
      m_apcSlicePilot->setNoOutputOfPriorPicsFlag( true );
    }
    else
    {
      m_apcSlicePilot->setNoOutputOfPriorPicsFlag( false );
    }

    if( m_apcSlicePilot->isCRAorGDR() )
    {
      m_lastNoOutputBeforeRecoveryFlag[nalu.m_nuhLayerId] = m_picHeader->getNoOutputBeforeRecoveryFlag();
    }


    if( m_apcSlicePilot->getNoOutputOfPriorPicsFlag() )
    {
      m_lastPOCNoOutputPriorPics = m_apcSlicePilot->getPOC();
      m_isNoOutputPriorPics = true;
    }
    else
    {
      m_isNoOutputPriorPics = false;
    }
  }

  //For inference of PicOutputFlag
  if( !pps->getMixedNaluTypesInPicFlag() && ( m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL ) )
  {
    if( m_lastNoOutputBeforeRecoveryFlag[nalu.m_nuhLayerId] )
    {
      m_picHeader->setPicOutputFlag( false );
    }
  }
  if( sps->getVPSId() > 0 )
  {
    VPS *vps = m_parameterSetManager.getVPS( sps->getVPSId() );
    CHECK( vps == 0, "No VPS present" );
    if( ( vps->getOlsModeIdc() == 0
          && vps->getGeneralLayerIdx( nalu.m_nuhLayerId ) < ( vps->getMaxLayers() - 1 )
          && vps->getOlsOutputLayerFlag( vps->m_iTargetLayer, vps->getMaxLayers() - 1 ) == 1 )
        || ( vps->getOlsModeIdc() == 2
             && vps->getOlsOutputLayerFlag( vps->m_iTargetLayer, vps->getGeneralLayerIdx( nalu.m_nuhLayerId ) ) == 0 ) )
    {
      m_picHeader->setPicOutputFlag( false );
    }
  }

  if( !pps->getMixedNaluTypesInPicFlag() && m_apcSlicePilot->isCRAorGDR() && m_lastNoOutputBeforeRecoveryFlag[nalu.m_nuhLayerId] )
  {
    int iMaxPOClsb = 1 << sps->getBitsForPOC();
    m_apcSlicePilot->setPOC( m_apcSlicePilot->getPOC() & ( iMaxPOClsb - 1 ) );
    xUpdatePreviousTid0POC ( m_apcSlicePilot );
  }

  AccessUnitPicInfo picInfo;
  picInfo.m_nalUnitType = nalu.m_nalUnitType;
  picInfo.m_nuhLayerId  = nalu.m_nuhLayerId;
  picInfo.m_temporalId  = nalu.m_temporalId;
  picInfo.m_POC         = m_apcSlicePilot->getPOC();
  m_accessUnitPicInfo.push_back( picInfo );

  // Skip pictures due to random access
  if( isRandomAccessSkipPicture() )
  {
    m_prevSliceSkipped = true;
    m_skippedPOC       = m_apcSlicePilot->getPOC();
    return SkipPicture;
  }

  // clear previous slice skipped flag
  m_prevSliceSkipped = false;

  //we should only get a different poc for a new picture (with CTU address==0)
  if(  m_apcSlicePilot->getPOC() != m_prevPOC
   && !m_bFirstSliceInSequence[nalu.m_nuhLayerId]
   &&  m_apcSlicePilot->getFirstCtuRsAddrInSlice() != 0 )
  {
    msg( WARNING, "Warning, the first slice of a picture might have been lost!\n");
  }

  m_prevLayerID = nalu.m_nuhLayerId;

  if( !pps->pcv )
  {
    pps->pcv = std::make_unique<PreCalcValues>( *sps, *pps );
  }

  //detect lost reference picture and insert copy of earlier frame.
  for( const auto rplIdx: { REF_PIC_LIST_0, REF_PIC_LIST_1 } )
  {
    const auto* rpl = m_apcSlicePilot->getRPL( rplIdx );

    int lostPoc         = MAX_INT;
    int lostRefPicIndex = 0;
    while( !m_apcSlicePilot->checkThatAllRefPicsAreAvailable( m_picListManager.getPicListRange( m_picListManager.getBackPic() ),
                                                              rpl,
                                                              m_apcSlicePilot->getNumRefIdx( rplIdx ),
                                                              &lostPoc,
                                                              &lostRefPicIndex ) )
    {
      if( !pps->getMixedNaluTypesInPicFlag()
          && ( ( m_apcSlicePilot->isIDR() && ( sps->getIDRRefParamListPresent() || pps->getRplInfoInPhFlag() ) )
               || ( m_apcSlicePilot->isCRAorGDR() && m_picHeader->getNoOutputBeforeRecoveryFlag() ) ) )
      {
        if( !rpl->isInterLayerRefPic( lostRefPicIndex ) )
        {
          prepareUnavailablePicture( pps,
                                     lostPoc,
                                     m_apcSlicePilot->getNalUnitLayerId(),
                                     rpl->isRefPicLongterm( lostRefPicIndex ),
                                     m_apcSlicePilot->getTLayer() );
        }
      }
      else
      {
        m_parseFrameList.push_back( prepareLostPicture( lostPoc, m_apcSlicePilot->getTLayer() ) );
      }
    }
  }

  Slice* slice = xDecodeSliceMain( nalu );

  // WARNING: don't use m_apcSlicePilot or m_picHeader after this point, because they have been reallocated

  m_pcParsePic->neededForOutput = m_pcParsePic->picHeader->getPicOutputFlag();
  if( pps->getNumSlicesInPic() == 0 || m_uiSliceSegmentIdx == pps->getNumSlicesInPic() - 1 )
  {
#if 0
    // TODO for VPS support:
    if( sps->getVPSId() > 0 && NOT IN OUTPUT LAYER SET )
    {
      m_pcParsePic->neededForOutput = false;
    }
    else
#endif
    {
      if( !m_gdrRecovered[nalu.m_nuhLayerId] && slice->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR && m_gdrRecoveryPointPocVal[nalu.m_nuhLayerId] == INIT_POC )
      {
        m_gdrRecoveryPointPocVal[nalu.m_nuhLayerId] = m_pcParsePic->poc + m_pcParsePic->picHeader->getRecoveryPocCnt();
      }

      if( !m_gdrRecovered[nalu.m_nuhLayerId]
          && ( m_gdrRecoveryPointPocVal[nalu.m_nuhLayerId] == m_pcParsePic->poc || m_pcParsePic->picHeader->getRecoveryPocCnt() == 0 ) )
      {
        m_gdrRecovered          [nalu.m_nuhLayerId] = true;
        m_gdrRecoveryPointPocVal[nalu.m_nuhLayerId] = INIT_POC;
      }

      const bool is_recovering_picture = slice->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_GDR && m_pcParsePic->poc < m_gdrRecoveryPointPocVal[nalu.m_nuhLayerId];
      if( slice->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR && m_gdrRecovered[nalu.m_nuhLayerId] )
      {
        m_pcParsePic->neededForOutput = true;
      }
      else if( slice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL && m_lastNoOutputBeforeRecoveryFlag[nalu.m_nuhLayerId] )
      {
        m_pcParsePic->neededForOutput = false;
      }
      else if( ( slice->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR && m_pcParsePic->picHeader->getNoOutputBeforeRecoveryFlag() )
               || ( is_recovering_picture && ( !m_gdrRecovered[nalu.m_nuhLayerId] || m_lastNoOutputBeforeRecoveryFlag[nalu.m_nuhLayerId] ) ) )
      {
        m_pcParsePic->neededForOutput = false;
      }
    }
  }

  if( slice->getFirstCtuRsAddrInSlice() == 0 && !m_bFirstSliceInPicture )
  {
    if( m_prevPOC >= m_pocRandomAccess )
    {
      DTRACE_UPDATE( g_trace_ctx, std::make_pair( "final", 0 ) );
    }
    m_prevPOC = slice->getPOC();
  }
  else
  {
    DTRACE_UPDATE( g_trace_ctx, std::make_pair( "final", 1 ) );
  }

  m_prevPOC = slice->getPOC();

  auto ret = ContinueParsing;

  const unsigned lastCtuInSlice = slice->getCtuAddrInSlice( slice->getNumCtuInSlice() - 1 );

  if( lastCtuInSlice == slice->getPPS()->pcv->sizeInCtus - 1 )
  {
    ret = NewPicture;
    xUpdateRasInit( slice );
  }

  ++m_uiSliceSegmentIdx;
  m_bFirstSliceInPicture                     = false;
  m_bFirstSliceInSequence[nalu.m_nuhLayerId] = false;
  m_bFirstSliceInBitstream                   = false;

  return ret;
}

Slice*  DecLibParser::xDecodeSliceMain( InputNALUnit &nalu )
{
  ITT_TASKSTART( itt_domain_oth, itt_handle_start );
  // actual decoding starts here
  m_pcParsePic   = xActivateParameterSets( nalu.m_nuhLayerId );
  Slice* pcSlice = m_pcParsePic->slices[m_uiSliceSegmentIdx];
  m_pcParsePic->poc          = pcSlice->getPOC();
  m_pcParsePic->layer        = pcSlice->getTLayer();
  m_pcParsePic->referenced   = !pcSlice->getPicHeader()->getNonReferencePictureFlag();
  m_pcParsePic->eNalUnitType = nalu.m_nalUnitType;
  m_pcParsePic->cts          = nalu.m_cts;
  m_pcParsePic->dts          = nalu.m_dts;
  m_pcParsePic->rap          = nalu.m_rap;
  m_pcParsePic->bits        += nalu.m_bits + m_nonVCLbits;
  m_pcParsePic->layerId      = nalu.m_nuhLayerId;
  m_pcParsePic->subLayerNonReferencePictureDueToSTSA = false;

  m_nonVCLbits = 0;

  CHECK( m_pcParsePic->layer != nalu.m_temporalId,
         "Currently parsed pic should have the same temporal layer as the NAL unit" );

  if( pcSlice->getSPS()->getProfileTierLevel()->getConstraintInfo()->getNoApsConstraintFlag() )
  {
    bool flag = pcSlice->getSPS()->getUseCCALF() || pcSlice->getPicHeader()->getNumAlfAps() || pcSlice->getPicHeader()->getAlfEnabledFlag( COMPONENT_Cb ) || pcSlice->getPicHeader()->getAlfEnabledFlag( COMPONENT_Cr );
    CHECK( flag,
           "When no_aps_constraint_flag is equal to 1, the values of ph_num_alf_aps_ids_luma, sh_num_alf_aps_ids_luma, ph_alf_cb_flag, ph_alf_cr_flag, sh_alf_cb_flag, sh_alf_cr_flag, and sps_ccalf_enabled_flag shall all be equal to 0" )
  }

  if( pcSlice->getNalUnitLayerId() != pcSlice->getSPS()->getLayerId() )
  {
    CHECK( pcSlice->getSPS()->getLayerId() > pcSlice->getNalUnitLayerId(), "Layer Id of SPS cannot be greater than layer Id of VCL NAL unit the refer to it" );
    CHECK( pcSlice->getSPS()->getVPSId() == 0, "VPSId of the referred SPS cannot be 0 when layer Id of SPS and layer Id of current slice are different" );
    for (int i = 0; i < pcSlice->getVPS()->getNumOutputLayerSets(); i++ )
    {
      bool isCurrLayerInOls = false;
      bool isRefLayerInOls = false;
      int j = pcSlice->getVPS()->getNumLayersInOls(i) - 1;
      for (; j >= 0; j--)
      {
        if( pcSlice->getVPS()->getLayerIdInOls(i, j) == pcSlice->getNalUnitLayerId() )
        {
          isCurrLayerInOls = true;
        }
        if( pcSlice->getVPS()->getLayerIdInOls(i, j) == pcSlice->getSPS()->getLayerId() )
        {
          isRefLayerInOls = true;
        }
      }
      CHECK( isCurrLayerInOls && !isRefLayerInOls, "When VCL NAl unit in layer A refers to SPS in layer B, all OLS that contains layer A shall also contains layer B" );
    }
  }
  if( pcSlice->getNalUnitLayerId() != pcSlice->getPPS()->getLayerId() )
  {
    CHECK( pcSlice->getPPS()->getLayerId() > pcSlice->getNalUnitLayerId(), "Layer Id of PPS cannot be greater than layer Id of VCL NAL unit the refer to it" );
    CHECK( pcSlice->getSPS()->getVPSId() == 0, "VPSId of the referred SPS cannot be 0 when layer Id of PPS and layer Id of current slice are different" );
    for (int i = 0; i < pcSlice->getVPS()->getNumOutputLayerSets(); i++ )
    {
      bool isCurrLayerInOls = false;
      bool isRefLayerInOls = false;
      int j = pcSlice->getVPS()->getNumLayersInOls(i) - 1;
      for (; j >= 0; j--)
      {
        if( pcSlice->getVPS()->getLayerIdInOls(i, j) == pcSlice->getNalUnitLayerId() )
        {
          isCurrLayerInOls = true;
        }
        if( pcSlice->getVPS()->getLayerIdInOls(i, j) == pcSlice->getPPS()->getLayerId() )
        {
          isRefLayerInOls = true;
        }
      }
      CHECK( isCurrLayerInOls && !isRefLayerInOls, "When VCL NAl unit in layer A refers to PPS in layer B, all OLS that contains layer A shall also contains layer B" );
    }
  }

  if( m_bFirstSliceInPicture )
  {
    m_pcParsePic->setDecodingOrderNumber( m_decodingOrderCounter );
    m_decodingOrderCounter++;

    pcSlice->getPic()->subPictures.clear();
    pcSlice->getPic()->sliceSubpicIdx.clear();

    for( int subPicIdx = 0; subPicIdx < pcSlice->getSPS()->getNumSubPics(); subPicIdx++ )
    {
      pcSlice->getPic()->subPictures.push_back( pcSlice->getPPS()->getSubPic( subPicIdx ) );
    }
  }

  pcSlice->getPic()->sliceSubpicIdx.push_back(pcSlice->getPPS()->getSubPicIdxFromSubPicId(pcSlice->getSliceSubPicId()));

  // When decoding the slice header, the stored start and end addresses were actually RS addresses, not TS addresses.
  // Now, having set up the maps, convert them to the correct form.

#if !DISABLE_CONFROMANCE_CHECK
  pcSlice->checkCRA( m_pocCRA[nalu.m_nuhLayerId], m_associatedIRAPType[nalu.m_nuhLayerId], m_picListManager.getPicListRange( m_pcParsePic ) );
#endif
  pcSlice->constructRefPicLists( m_picListManager.getPicListRange( m_pcParsePic ) );
#if !DISABLE_CONFROMANCE_CHECK
  pcSlice->checkRPL(pcSlice->getRPL0(), pcSlice->getRPL1(), m_associatedIRAPDecodingOrderNumber, m_picListManager.getPicListRange( m_pcParsePic ) );
  pcSlice->checkSTSA( m_picListManager.getPicListRange( m_pcParsePic ) );
#endif

#if TBC
  pcSlice->setPrevGDRSubpicPOC(m_prevGDRSubpicPOC[nalu.m_nuhLayerId][currSubPicIdx]);
  pcSlice->setPrevIRAPSubpicPOC(m_prevIRAPSubpicPOC[nalu.m_nuhLayerId][currSubPicIdx]);
  pcSlice->setPrevIRAPSubpicType(m_prevIRAPSubpicType[nalu.m_nuhLayerId][currSubPicIdx]);
  pcSlice->checkSubpicTypeConstraints(m_cListPic, pcSlice->getRPL0(), pcSlice->getRPL1(), m_prevIRAPSubpicDecOrderNo[nalu.m_nuhLayerId][currSubPicIdx]);
#endif

  if( m_pcParsePic->cs->vps && !m_pcParsePic->cs->vps->getIndependentLayerFlag( m_pcParsePic->cs->vps->getGeneralLayerIdx( nalu.m_nuhLayerId ) ) && m_pcParsePic->cs->pps->getNumSubPics() > 1 )
  {
    CU::checkConformanceILRP(pcSlice);
  }

  pcSlice->scaleRefPicList( m_pcParsePic->cs->picHeader );


    if (!pcSlice->isIntra())
    {
      const int iCurrPOC  = pcSlice->getPOC();

      bool bLowDelay = true;
      for( int iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx( REF_PIC_LIST_0 ) && bLowDelay; iRefIdx++ )
      {
        if( pcSlice->getRefPic( REF_PIC_LIST_0, iRefIdx )->getPOC() > iCurrPOC )
        {
          bLowDelay = false;
        }
      }
      if( pcSlice->isInterB() )
      {
        for( int iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx( REF_PIC_LIST_1 ) && bLowDelay; iRefIdx++ )
        {
          if( pcSlice->getRefPic( REF_PIC_LIST_1, iRefIdx )->getPOC() > iCurrPOC )
          {
            bLowDelay = false;
          }
        }
      }

      pcSlice->setCheckLDC( bLowDelay );
    }

    if( pcSlice->getSPS()->getUseSMVD() && !pcSlice->getCheckLDC() && !pcSlice->getPicHeader()->getMvdL1ZeroFlag() )
    {
      int currPOC = pcSlice->getPOC();

      int forwardPOC = currPOC;
      int backwardPOC = currPOC;
      int ref = 0;
      int refIdx0 = -1;
      int refIdx1 = -1;

      // search nearest forward POC in List 0
      for ( ref = 0; ref < pcSlice->getNumRefIdx( REF_PIC_LIST_0 ); ref++ )
      {
        int poc = pcSlice->getRefPic( REF_PIC_LIST_0, ref )->getPOC();
        const bool isRefLongTerm = pcSlice->getRefPic( REF_PIC_LIST_0, ref )->longTerm;
        if( poc < currPOC && ( poc > forwardPOC || refIdx0 == -1 ) && !isRefLongTerm )
        {
          forwardPOC = poc;
          refIdx0 = ref;
        }
      }

      // search nearest backward POC in List 1
      for ( ref = 0; ref < pcSlice->getNumRefIdx( REF_PIC_LIST_1 ); ref++ )
      {
        int poc = pcSlice->getRefPic( REF_PIC_LIST_1, ref )->getPOC();
        const bool isRefLongTerm = pcSlice->getRefPic( REF_PIC_LIST_1, ref )->longTerm;
        if( poc > currPOC && ( poc < backwardPOC || refIdx1 == -1 ) && !isRefLongTerm )
        {
          backwardPOC = poc;
          refIdx1 = ref;
        }
      }

      if ( !(forwardPOC < currPOC && backwardPOC > currPOC) )
      {
        forwardPOC = currPOC;
        backwardPOC = currPOC;
        refIdx0 = -1;
        refIdx1 = -1;

        // search nearest backward POC in List 0
        for ( ref = 0; ref < pcSlice->getNumRefIdx( REF_PIC_LIST_0 ); ref++ )
        {
          int poc = pcSlice->getRefPic( REF_PIC_LIST_0, ref )->getPOC();
          const bool isRefLongTerm = pcSlice->getRefPic( REF_PIC_LIST_0, ref )->longTerm;
          if( poc > currPOC && ( poc < backwardPOC || refIdx0 == -1 ) && !isRefLongTerm )
          {
            backwardPOC = poc;
            refIdx0 = ref;
          }
        }

        // search nearest forward POC in List 1
        for ( ref = 0; ref < pcSlice->getNumRefIdx( REF_PIC_LIST_1 ); ref++ )
        {
          int poc = pcSlice->getRefPic( REF_PIC_LIST_1, ref )->getPOC();
          const bool isRefLongTerm = pcSlice->getRefPic( REF_PIC_LIST_1, ref )->longTerm;
          if( poc < currPOC && ( poc > forwardPOC || refIdx1 == -1 ) && !isRefLongTerm )
          {
            forwardPOC = poc;
            refIdx1 = ref;
          }
        }
      }

      if ( forwardPOC < currPOC && backwardPOC > currPOC )
      {
        pcSlice->setBiDirPred( true, refIdx0, refIdx1 );
      }
      else
      {
        pcSlice->setBiDirPred( false, -1, -1 );
      }
    }
    else
    {
      pcSlice->setBiDirPred( false, -1, -1 );
    }

    //---------------
    pcSlice->setRefPOCList();
    NalUnitInfo naluInfo;
    naluInfo.m_nalUnitType = nalu.m_nalUnitType;
    naluInfo.m_nuhLayerId = nalu.m_nuhLayerId;
    naluInfo.m_firstCTUinSlice = pcSlice->getFirstCtuRsAddrInSlice();
    naluInfo.m_POC = pcSlice->getPOC();
    xCheckMixedNalUnit( pcSlice, nalu );
    m_nalUnitInfo[naluInfo.m_nuhLayerId].push_back( naluInfo );

  if( m_bFirstSliceInPicture )
  {
#if RECO_WHILE_PARSE
    for( int ctu = 0; ctu < pcSlice->getPPS()->pcv->sizeInCtus; ctu++ )
    {
      m_pcParsePic->ctuParsedBarrier[ctu].lock();
    }

#endif
    m_parseFrameList.push_back( m_pcParsePic );
  }  

  ITT_TASKEND( itt_domain_oth, itt_handle_start );

  static const auto parseTask = []( int threadId, Slice* slice )
  {
    auto& decLib    = slice->parseTaskParams.decLibParser;
    auto& bitstream = slice->parseTaskParams.bitstream;
    auto* pic       = slice->getPic();

    try
    {
      pic->startProcessingTimer();

      //  Decode a picture
      ITT_TASKSTART( itt_domain_prs, itt_handle_parse );
      decLib->m_cSliceDecoder.parseSlice( slice, &bitstream, threadId );
      ITT_TASKEND( itt_domain_prs, itt_handle_parse );

      pic->stopProcessingTimer();

      bitstream.clearFifo();
      bitstream.clearEmulationPreventionByteLocation();
    }
    catch( ... )
    {
      pic->parseDone.setException( std::current_exception() );
#if RECO_WHILE_PARSE
      for( auto& b: pic->ctuParsedBarrier )
      {
        b.setException( std::current_exception() );
      }
#endif
      std::rethrow_exception( std::current_exception() );
    }
    return true;
  };

  pcSlice->parseTaskParams.init( this, std::move( nalu.getBitstream() ) );
  pcSlice->parseDone.lock();

  if( m_threadPool && m_threadPool->numThreads() > 1 )
  {
    if( m_uiSliceSegmentIdx > 0 )
    {
      m_threadPool->addBarrierTask<Slice>( parseTask, pcSlice, nullptr, &pcSlice->parseDone, CBarrierVec{ &pcSlice->getPic()->slices[m_uiSliceSegmentIdx - 1]->parseDone } );
    }
    else
    {
      m_threadPool->addBarrierTask<Slice>( parseTask, pcSlice, nullptr, &pcSlice->parseDone );
    }
  }
  else
  {
    parseTask( 0, pcSlice );
    pcSlice->parseDone.unlock();
    if( m_pcParsePic->slices.size() != 1 && !m_pcParsePic->parseDone.isBlocked() && m_numDecThreads == 0 )
    {
      while( !m_threadPool->processTasksOnMainThread() );
    }
  }

  return pcSlice;
}

Picture * DecLibParser::xActivateParameterSets( const int layerId )
{
  Picture * pcPic = nullptr;

  if( m_bFirstSliceInPicture )
  {
    auto paramSets = m_parameterSetManager.xActivateParameterSets( m_apcSlicePilot, m_picHeader.get() );
    const SPS*  sps     = paramSets.sps;
    const PPS*  pps     = paramSets.pps;
          APS** alfApss = paramSets.alfAPSs->data();
          APS*  lmcsAPS = paramSets.lmcsAps;
          APS*  scalingListAPS = paramSets.scalingListAps;

    xParsePrefixSEImessages();

#if RExt__HIGH_BIT_DEPTH_SUPPORT == 0
    if( /* sps->getSpsRangeExtension().getExtendedPrecisionProcessingFlag() ||*/ sps->getBitDepth( CHANNEL_TYPE_LUMA ) > 12 || sps->getBitDepth( CHANNEL_TYPE_CHROMA ) > 12 )
    {
      THROW( "High bit depth support must be enabled at compile-time in order to decode this bitstream\n" );
    }
#endif

    //  Get a new picture buffer. This will also set up m_pcPic, and therefore give us a SPS and PPS pointer that we can use.
    pcPic = m_picListManager.getNewPicBuffer( *sps, *pps, m_apcSlicePilot->getTLayer(), layerId, m_parameterSetManager.getVPS( sps->getVPSId() ) );
    // assign these fields already, because they are needed by PicListManager::getPicListRange() and Slice::applyReferencePictureSet()
    pcPic->poc          = m_apcSlicePilot->getPOC();
    pcPic->eNalUnitType = m_apcSlicePilot->getNalUnitType();

#if !DISABLE_CONFROMANCE_CHECK
    m_apcSlicePilot->checkLeadingPictureRestrictions( m_picListManager.getPicListRange( pcPic ) );
#endif
    pcPic->finalInit( &m_cuChunkCache, &m_tuChunkCache, sps, pps, m_picHeader.get(), alfApss, lmcsAPS, scalingListAPS );

    // Set Field/Frame coding mode
    bool isField    = false;
    bool isTopField = false;
    if(!m_seiMessageList.empty())
    {
      // Check if any new Frame Field Info SEI has arrived
      seiMessages frameFieldSEIs = SEI_internal::getSeisByType( m_seiMessageList, VVDEC_FRAME_FIELD_INFO );
      if(!frameFieldSEIs.empty())
      {
        const vvdecSEIFrameFieldInfo* ff = (vvdecSEIFrameFieldInfo*) frameFieldSEIs.front()->payload;
        isField    = ff->fieldPicFlag;
        isTopField = isField && (!ff->bottomFieldFlag);
      }
    }
    pcPic->fieldPic = isField;
    pcPic->topField = isTopField;

    // transfer any SEI messages that have been received to the picture
    pcPic->seiMessageList = m_seiMessageList;
    m_seiMessageList.clear();
  }
  else
  {
    pcPic = m_pcParsePic;
  }

  // make the slice-pilot a real slice, and set up the slice-pilot for the next slice
  Slice* pSlice = pcPic->allocateNewSlice( &m_apcSlicePilot );
  CHECK( pcPic->slices.size() != ( m_uiSliceSegmentIdx + 1 ), "Invalid number of slices" );

  if( m_bFirstSliceInPicture )
  {
    pcPic->setPicHead( m_picHeader );
    pSlice->setPicHeader( m_picHeader.get() );
    m_picHeader = std::make_shared<PicHeader>();
  }

  const VPS*  vps     = pSlice->getVPS();
  const SPS*  sps     = pSlice->getSPS();
  const PPS*  pps     = pSlice->getPPS();

  if( !m_bFirstSliceInPicture )
  {
    APS*  lmcsAPS = pSlice->getPicHeader()->getLmcsAPS().get();

    // check that the current active PPS has not changed...
    if( sps->getChangedFlag() )
    {
      EXIT( "Error - a new SPS has been decoded while processing a picture" );
    }
    if( pps->getChangedFlag() )
    {
      EXIT( "Error - a new PPS has been decoded while processing a picture" );
    }
    for( int i = 0; i < ALF_CTB_MAX_NUM_APS; i++ )
    {
      APS* aps = m_parameterSetManager.getAPS_nothrow( i, ALF_APS );
      if( aps && aps->getChangedFlag() )
      {
        EXIT("Error - a new APS has been decoded while processing a picture");
      }
    }

    if( lmcsAPS && lmcsAPS->getChangedFlag() )
    {
      EXIT("Error - a new LMCS APS has been decoded while processing a picture");
    }

    xParsePrefixSEImessages();

    // Check if any new SEI has arrived
    if( !m_seiMessageList.empty() )
    {
      // Currently only decoding Unit SEI message occurring between VCL NALUs copied
      seiMessages& picSEI = pcPic->seiMessageList;
      seiMessages decodingUnitInfos = SEI_internal::extractSeisByType( m_seiMessageList, VVDEC_DECODING_UNIT_INFO );
      picSEI.insert( picSEI.end(), decodingUnitInfos.begin(), decodingUnitInfos.end() );
      SEI_internal::deleteSEIs   ( m_seiMessageList );
    }
  }

  CHECK( !sps->getGDREnabledFlag() && m_picHeader->getGdrPicFlag(),
         "When gdr_enabled_flag is equal to 0, the value of gdr_pic_flag shall be equal to 0 " );
  CHECK( !sps->getUseWP() && pps->getUseWP(),
         "When sps_weighted_pred_flag is equal to 0, the value of pps_weighted_pred_flag shall be equal to 0." );
  CHECK( !sps->getUseWPBiPred() && pps->getWPBiPred(),
         "When sps_weighted_bipred_flag is equal to 0, the value of pps_weighted_bipred_flag shall be equal to 0." );

  CHECK( !sps->getResChangeInClvsEnabledFlag() && pps->getPicWidthInLumaSamples() != sps->getMaxPicWidthInLumaSamples(),
         "When res_change_in_clvs_allowed_flag equal to 0, the value of pic_width_in_luma_samples shall be equal to pic_width_max_in_luma_samples." );
  CHECK( !sps->getResChangeInClvsEnabledFlag() && pps->getPicHeightInLumaSamples() != sps->getMaxPicHeightInLumaSamples(),
         "When res_change_in_clvs_allowed_flag equal to 0, the value of pic_height_in_luma_samples shall be equal to pic_height_max_in_luma_samples." );
  CHECK( sps->getResChangeInClvsEnabledFlag() &&sps->getSubPicInfoPresentFlag() != 0,
         "When res_change_in_clvs_allowed_flag is equal to 1, the value of subpic_info_present_flag shall be equal to 0.");
  CHECK( sps->getResChangeInClvsEnabledFlag() && sps->getVirtualBoundariesEnabledFlag(),
         "when the value of res_change_in_clvs_allowed_flag is equal to 1, the value of sps_virtual_boundaries_present_flag shall be equal to 0" );
  if( vps != nullptr && vps->m_numOutputLayersInOls[vps->m_iTargetLayer] > 1 )
  {
    CHECK( sps->getMaxPicWidthInLumaSamples( ) > vps->getOlsDpbPicSize( vps->m_iTargetLayer ).width,
           "pic_width_max_in_luma_samples shall be less than or equal to the value of ols_dpb_pic_width[ i ]" );
    CHECK( sps->getMaxPicHeightInLumaSamples() > vps->getOlsDpbPicSize( vps->m_iTargetLayer ).height,
           "pic_height_max_in_luma_samples shall be less than or equal to the value of ols_dpb_pic_height[ i ]" );
  }

  CHECK( sps->getProfileTierLevel()->getConstraintInfo()->getOneTilePerPicConstraintFlag() &&pps->getNumTiles() != 1,
         "When one_tile_per_pic_constraint_flag is equal to 1, each picture shall contain only one tile" );
  CHECK( sps->getProfileTierLevel()->getConstraintInfo()->getOneSlicePerPicConstraintFlag() && pps->getRectSliceFlag() && pps->getNumSlicesInPic() != 1,
          "When one_slice_per_pic_constraint_flag is equal to 1 and if pps_rect_slice_flag is equal to 1, the value of num_slices_in_pic_minus1 shall be equal to 0" );
  CHECK( sps->getProfileTierLevel()->getConstraintInfo()->getNoRprConstraintFlag() && sps->getRprEnabledFlag(),
         "When gci_no_ref_pic_resampling_constraint_flag is equal to 1, the value of sps_ref_pic_resampling_enabled_flag shall be equal to 0" );
  CHECK( sps->getProfileTierLevel()->getConstraintInfo()->getNoResChangeInClvsConstraintFlag() && sps->getResChangeInClvsEnabledFlag(),
         "When gci_no_res_change_in_clvs_constraint_flag is equal to 1, the value of sps_res_change_in_clvs_allowed_flag shall be equal to 0" );
  CHECK( sps->getProfileTierLevel()->getConstraintInfo()->getNoIdrRplConstraintFlag() && sps->getIDRRefParamListPresent(),
          "When gci_no_idr_rpl_constraint_flag equal to 1 , the value of sps_idr_rpl_present_flag shall be equal to 0" );
  CHECK( sps->getProfileTierLevel()->getConstraintInfo()->getNoMixedNaluTypesInPicConstraintFlag() && pps->getMixedNaluTypesInPicFlag(),
          "When gci_no_mixed_nalu_types_in_pic_constraint_flag equal to 1, the value of pps_mixed_nalu_types_in_pic_flag shall be equal to 0" );
  CHECK( sps->getProfileTierLevel()->getConstraintInfo()->getNoRectSliceConstraintFlag() && pps->getRectSliceFlag(),
          "When gci_no_rectangular_slice_constraint_flag equal to 1, the value of pps_rect_slice_flag shall be equal to 0" );
  CHECK( sps->getProfileTierLevel()->getConstraintInfo()->getOneSlicePerSubpicConstraintFlag() && !pps->getSingleSlicePerSubPicFlag(),
          "When gci_one_slice_per_subpic_constraint_flag equal to 1, the value of pps_single_slice_per_subpic_flag shall be equal to 1" );
  CHECK( sps->getProfileTierLevel()->getConstraintInfo()->getNoSubpicInfoConstraintFlag() && sps->getSubPicInfoPresentFlag(),
          "When gci_no_subpic_info_constraint_flag is equal to 1, the value of sps_subpic_info_present_flag shall be equal to 0" );

  if( sps->getProfileTierLevel()->getConstraintInfo()->getNoMttConstraintFlag() )
  {
    CHECK( sps->getMaxBTDepth() || sps->getMaxBTDepthI() || sps->getMaxBTDepthIChroma(),
           "When gci_no_mtt_constraint_flag is equal to 1, the values of sps_max_mtt_hierarchy_depth_intra_slice_luma, sps_max_mtt_hierarchy_depth_inter_slice and sps_max_mtt_hierarchy_depth_intra_slice_chroma shall be equal to 0" );
  }
  if( sps->getProfileTierLevel()->getConstraintInfo()->getNoWeightedPredictionConstraintFlag() )
  {
    CHECK( sps->getUseWP() || sps->getUseWPBiPred(),
           "When gci_no_weighted_prediction_constraint_flag is equal to 1, the values of sps_weighted_pred_flag and sps_weighted_bipred_flag shall be equal to 0" );
  }

  CHECK( sps->getProfileTierLevel()->getConstraintInfo()->getNoChromaQpOffsetConstraintFlag() && pps->getCuChromaQpOffsetEnabledFlag(),
         "When gci_no_ChromaQpOffset_constraint_flag is equal to 1, the values of pps_cu_chroma_qp_offset_list_enabled_flag shall be equal to 0" );

  CHECK( sps->getCTUSize() > ( 1 << sps->getProfileTierLevel()->getConstraintInfo()->getMaxLog2CtuSizeConstraintIdc() ),
         "The CTU size specified by sps_log2_ctu_size_minus5 shall not exceed the constraint specified by gci_three_minus_max_log2_ctu_size_constraint_idc" );
  CHECK( sps->getProfileTierLevel()->getConstraintInfo()->getNoLumaTransformSize64ConstraintFlag() && sps->getLog2MaxTbSize() != 5,
         "When gci_no_luma_transform_size_64_constraint_flag is equal to 1, the value of sps_max_luma_transform_size_64_flag shall be equal to 0" );

  // TODO: fix MT static maps
  static std::unordered_map<int, int> m_layerChromaFormat;
  static std::unordered_map<int, int> m_layerBitDepth;

  if( vps != nullptr && vps->getMaxLayers() > 1 )
  {
    int curLayerIdx          = vps->getGeneralLayerIdx(layerId);
    int curLayerChromaFormat = sps->getChromaFormatIdc();
    int curLayerBitDepth     = sps->getBitDepth(CHANNEL_TYPE_LUMA);

    if( pSlice->isClvssPu() && m_bFirstSliceInPicture )
    {
      m_layerChromaFormat[curLayerIdx] = curLayerChromaFormat;
      m_layerBitDepth    [curLayerIdx] = curLayerBitDepth;
    }
    else
    {
      CHECK( m_layerChromaFormat[curLayerIdx] != curLayerChromaFormat,
             "Different chroma format in the same layer." );
      CHECK( m_layerBitDepth    [curLayerIdx] != curLayerBitDepth,
             "Different bit-depth in the same layer." );
    }

    for( int i = 0; i < curLayerIdx; i++ )
    {
      if( vps->getDirectRefLayerFlag( curLayerIdx, i ) )
      {
        int refLayerChromaFormat = m_layerChromaFormat[i];
        CHECK( curLayerChromaFormat != refLayerChromaFormat,
               "The chroma formats of the current layer and the reference layer are different" );
        int refLayerBitDepth = m_layerBitDepth[i];
        CHECK( curLayerBitDepth != refLayerBitDepth,
               "The bit-depth of the current layer and the reference layer are different" );
      }
    }
  }

  const int minCuSize = 1 << sps->getLog2MinCodingBlockSize();
  CHECK( ( pps->getPicWidthInLumaSamples()  % ( std::max( 8, minCuSize) ) ) != 0,
         "Coded frame width must be a multiple of Max(8, the minimum unit size)" );
  CHECK( ( pps->getPicHeightInLumaSamples() % ( std::max( 8, minCuSize) ) ) != 0,
         "Coded frame height must be a multiple of Max(8, the minimum unit size)" );

  if( sps->getCTUSize() + 2 * ( 1 << sps->getLog2MinCodingBlockSize() ) > pps->getPicWidthInLumaSamples() )
  {
    CHECK( pps->getUseWrapAround(),
           "Wraparound shall be disabled when the value of ( CtbSizeY / MinCbSizeY + 1) is greater than or equal to ( pic_width_in_luma_samples / MinCbSizeY - 1 )" );
  }

  if( pSlice->getPicHeader()->getGdrOrIrapPicFlag() && !pSlice->getPicHeader()->getGdrPicFlag() && ( !vps || vps->getIndependentLayerFlag( vps->getGeneralLayerIdx( layerId ) ) ) )
  {
    CHECK( pSlice->getPicHeader()->getPicInterSliceAllowedFlag(),
          "When gdr_or_irap_pic_flag is equal to 1 and gdr_pic_flag is equal to 0 and vps_independent_layer_flag[ GeneralLayerIdx[ nuh_layer_id ] ] is equal to 1, ph_inter_slice_allowed_flag shall be equal to 0" );
  }

  if( sps->getVPSId() && vps->m_numLayersInOls[vps->m_iTargetLayer] == 1 )
  {
    CHECK( !sps->getPtlDpbHrdParamsPresentFlag(),
           "When sps_video_parameter_set_id is greater than 0 and there is an OLS that contains only one layer with nuh_layer_id equal to the nuh_layer_id of the SPS, the value of sps_ptl_dpb_hrd_params_present_flag shall be equal to 1" );
  }

  ProfileLevelTierFeatures ptlFeature;
  ptlFeature.extractPTLInformation(*sps);

  const LevelTierFeatures* ltFeature = ptlFeature.getLevelTierFeatures();
  const ProfileFeatures*   pFeature  = ptlFeature.getProfileFeatures();

  CHECK( ltFeature && pps->getNumTileColumns() > ltFeature->maxTileCols,
         "Num tile columns signaled in PPS exceed level limits" );
  CHECK( ltFeature && pps->getNumTiles() > ltFeature->maxTilesPerAu,
         "Num tiles signaled in PPS exceed level limits" );
  CHECK( pFeature && sps->getBitDepth( CHANNEL_TYPE_LUMA ) > pFeature->maxBitDepth,
         "Bit depth exceed profile limit" );
  CHECK( pFeature && sps->getChromaFormatIdc() > pFeature->maxChromaFormat,
         "Chroma format exceed profile limit" );

  return pcPic;
}

Picture* DecLibParser::prepareLostPicture( int iLostPoc, const int layerId )
{
  msg( INFO, "inserting lost poc : %d\n", iLostPoc );

  Picture* cFillPic = m_picListManager.getNewPicBuffer( *m_parameterSetManager.getFirstSPS(), *m_parameterSetManager.getFirstPPS(), 0, layerId, m_parameterSetManager.getVPS( m_parameterSetManager.getFirstSPS()->getVPSId() ) );
  cFillPic->finalInit( &m_cuChunkCache, &m_tuChunkCache, m_parameterSetManager.getFirstSPS(), m_parameterSetManager.getFirstPPS(), m_picHeader.get(), m_parameterSetManager.getAlfAPSs().data(), nullptr, nullptr, false ); //TODO: check this
  cFillPic->cs->allocTempInternals();
  cFillPic->cs->initStructData();

  int         iTLayer  = m_apcSlicePilot->getTLayer();   // TLayer needs to be <= TLayer of referencing frame
  bool        isIRAP   = false;
  NalUnitType naluType = NAL_UNIT_CODED_SLICE_TRAIL;
  if( m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL ||
      m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL )
  {
    if( m_apcSlicePilot->getAssociatedIRAPPOC() < m_apcSlicePilot->getPOC()
        && iLostPoc > m_apcSlicePilot->getPOC() )
    {
      iTLayer = 0;
      isIRAP  = true;
      if( iLostPoc == 0 && m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL )
      {
        naluType = NAL_UNIT_CODED_SLICE_IDR_W_RADL;
      }
      else
      {
        naluType = NAL_UNIT_CODED_SLICE_CRA;
      }
    }
  }
  else if( iLostPoc == 0 )
  {
    iTLayer  = 0;
    isIRAP   = true;
    naluType = NAL_UNIT_CODED_SLICE_IDR_N_LP;
  }

  if( cFillPic->slices.empty() )
  {
    cFillPic->allocateNewSlice();
  }
  cFillPic->slices[0]->initSlice();
  cFillPic->slices[0]->setPOC( iLostPoc );
  cFillPic->slices[0]->setTLayer( iTLayer );
  cFillPic->slices[0]->setNalUnitType( naluType );
  if( (cFillPic->slices[0]->getTLayer() == 0) && (cFillPic->slices[0]->getNalUnitType() != NAL_UNIT_CODED_SLICE_RASL) && (cFillPic->slices[0]->getNalUnitType() != NAL_UNIT_CODED_SLICE_RADL) )
  {
    m_prevTid0POC = cFillPic->slices[0]->getPOC();
  }
  cFillPic->layer           = iTLayer;
  cFillPic->referenced      = true;
  cFillPic->neededForOutput = false;
  cFillPic->wasLost         = true;
  cFillPic->progress        = Picture::parsed;
  cFillPic->poc             = iLostPoc;
  cFillPic->eNalUnitType    = naluType;
  cFillPic->rap             = isIRAP;
  if( isIRAP )
  {
    m_apcSlicePilot->setAssociatedIRAPPOC( iLostPoc );
    m_apcSlicePilot->setAssociatedIRAPType( naluType );

    m_pocRandomAccess = iLostPoc;
  }
  return cFillPic;
}

void DecLibParser::prepareUnavailablePicture( const PPS *pps, int iUnavailablePoc, const int layerId, const bool longTermFlag, const int temporalId )
{
  msg( INFO, "inserting unavailable poc : %d\n", iUnavailablePoc );
  Picture* cFillPic = m_picListManager.getNewPicBuffer( *m_parameterSetManager.getFirstSPS(), *m_parameterSetManager.getFirstPPS(), 0, layerId, m_parameterSetManager.getVPS( m_parameterSetManager.getFirstSPS()->getVPSId() ) );
  APS* nullAlfApss[ALF_CTB_MAX_NUM_APS] = { nullptr, };
  cFillPic->finalInit( &m_cuChunkCache, &m_tuChunkCache, m_parameterSetManager.getFirstSPS(), m_parameterSetManager.getFirstPPS(), m_picHeader.get(), nullAlfApss, nullptr, nullptr, false ); //TODO: check this
  cFillPic->cs->allocTempInternals();
  cFillPic->cs->initStructData();

  cFillPic->allocateNewSlice();
  cFillPic->slices[0]->initSlice();

  uint32_t yFill = 1 << (m_parameterSetManager.getFirstSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 1);
  uint32_t cFill = 1 << (m_parameterSetManager.getFirstSPS()->getBitDepth(CHANNEL_TYPE_CHROMA) - 1);
  cFillPic->getRecoBuf().Y().fill(yFill);
  cFillPic->getRecoBuf().Cb().fill(cFill);
  cFillPic->getRecoBuf().Cr().fill(cFill);

  cFillPic->referenced = true;
//  cFillPic->interLayerRefPicFlag = false; //always false and never gets checked
  cFillPic->longTerm = longTermFlag;
  cFillPic->slices[0]->setPOC(iUnavailablePoc);
//  cFillPic->wasLost = true;
  cFillPic->poc = iUnavailablePoc;
  if( (cFillPic->slices[0]->getTLayer() == 0) && (cFillPic->slices[0]->getNalUnitType() != NAL_UNIT_CODED_SLICE_RASL) && (cFillPic->slices[0]->getNalUnitType() != NAL_UNIT_CODED_SLICE_RADL) )
  {
    m_prevTid0POC = cFillPic->slices[0]->getPOC();
  }
  cFillPic->progress        = Picture::reconstructed;
  cFillPic->neededForOutput = false;
  // picture header is not derived for generated reference picture
  cFillPic->slices[0]->setPicHeader( nullptr );
  cFillPic->layer = temporalId;
  cFillPic->nonReferencePictureFlag = false;
  cFillPic->slices[0]->setPPS( pps );

  cFillPic->parseDone.unlock();
  cFillPic->done.unlock();
}

void DecLibParser::xParsePrefixSEImessages()
{
  while( !m_prefixSEINALUs.empty() )
  {
    InputNALUnit& nalu = m_prefixSEINALUs.front();
    const SPS *sps = m_parameterSetManager.getActiveSPS();
    const VPS *vps = m_parameterSetManager.getVPS(sps->getVPSId());
    m_seiReader.parseSEImessage( &(nalu.getBitstream()), m_seiMessageList, nalu.m_nalUnitType, nalu.m_nuhLayerId, nalu.m_temporalId, vps, sps, m_HRD, m_pDecodedSEIOutputStream );
    m_prefixSEINALUs.pop_front();
  }
}

void DecLibParser::xParsePrefixSEIsForUnknownVCLNal()
{
  while (!m_prefixSEINALUs.empty())
  {
    // do nothing?
    msg( NOTICE, "Discarding Prefix SEI associated with unknown VCL NAL unit.\n");
    m_prefixSEINALUs.pop_front();
  }
  // TODO: discard following suffix SEIs as well?
}

void DecLibParser::xDecodePicHeader( InputNALUnit& nalu )
{
  m_HLSReader.setBitstream( &nalu.getBitstream() );
  m_HLSReader.parsePictureHeader( m_picHeader.get(), &m_parameterSetManager, true );
  m_picHeader->setValid();
}

void DecLibParser::xDecodeVPS( InputNALUnit& nalu )
{
  std::unique_ptr<VPS> vps( new VPS() );
  m_HLSReader.setBitstream( &nalu.getBitstream() );
  m_HLSReader.parseVPS( vps.get() );
  m_parameterSetManager.storeVPS( vps.release(), nalu.getBitstream().getFifo() );

  // VPS is parsed, but completely ingored
}

void DecLibParser::xDecodeDCI( InputNALUnit& nalu )
{
  m_HLSReader.setBitstream(&nalu.getBitstream());

  CHECK(nalu.m_temporalId, "The value of TemporalId of DCI NAL units shall be equal to 0");
  if( !m_dci )
  {
    m_dci = new DCI;
    m_HLSReader.parseDCI( m_dci );
  }
  else
  {
    DCI dupDCI;
    m_HLSReader.parseDCI( &dupDCI );
    CHECK( !m_dci->IsIndenticalDCI( dupDCI ), "Two signaled DCIs are different");
  }
}

void DecLibParser::xDecodeSPS( InputNALUnit& nalu )
{
  std::unique_ptr<SPS> sps( new SPS() );
  m_HLSReader.setBitstream( &nalu.getBitstream() );
  m_HLSReader.parseSPS( sps.get(), &m_parameterSetManager );
  sps->setLayerId( nalu.m_nuhLayerId );
  DTRACE( g_trace_ctx, D_QP_PER_CTU, "CTU Size: %dx%d", sps->getMaxCUWidth(), sps->getMaxCUHeight() );  // don't move after storeSPS, because SPS could have been deleted
  m_parameterSetManager.storeSPS( sps.release(), nalu.getBitstream().getFifo() );
}

void DecLibParser::xDecodePPS( InputNALUnit& nalu )
{
  std::unique_ptr<PPS> pps( new PPS() );
  m_HLSReader.setBitstream( &nalu.getBitstream() );
  m_HLSReader.parsePPS( pps.get(), &m_parameterSetManager );
  pps->setLayerId( nalu.m_nuhLayerId );
  m_parameterSetManager.storePPS( pps.release(), nalu.getBitstream().getFifo() );
}

void DecLibParser::xDecodeAPS( InputNALUnit& nalu )
{
  std::unique_ptr<APS> aps( new APS() );
  m_HLSReader.setBitstream( &nalu.getBitstream() );
  m_HLSReader.parseAPS( aps.get() );
  aps->setTemporalId( nalu.m_temporalId );
  aps->setLayerId( nalu.m_nuhLayerId );
  aps->setHasPrefixNalUnitType( nalu.m_nalUnitType == NAL_UNIT_PREFIX_APS );
  m_parameterSetManager.storeAPS( aps.release(), nalu.getBitstream().getFifo() );
}

void DecLibParser::xUpdatePreviousTid0POC(Slice * pSlice)
{
  if( pSlice->getTLayer() == 0 && pSlice->getNalUnitType() != NAL_UNIT_CODED_SLICE_RASL && pSlice->getNalUnitType() != NAL_UNIT_CODED_SLICE_RADL
      && !pSlice->getPicHeader()->getNonReferencePictureFlag() )
  {
    m_prevTid0POC = pSlice->getPOC();
  }
}

void DecLibParser::xUpdateRasInit(Slice* slice)
{
  slice->setPendingRasInit( false );
  if ( slice->getPOC() > m_lastRasPoc )
  {
    m_lastRasPoc = MAX_INT;
    slice->setPendingRasInit( true );
  }
  if ( slice->isIRAP() )
  {
    m_lastRasPoc = slice->getPOC();
  }
}

void DecLibParser::checkNoOutputPriorPics()
{
  if( !m_isNoOutputPriorPics )
  {
    return;
  }

  auto pcListPic = m_picListManager.getPicListRange( m_picListManager.getBackPic() );
  for( auto& pcPicTmp: pcListPic )
  {
    if( pcPicTmp->progress >= Picture::reconstructed && pcPicTmp->getPOC() < m_lastPOCNoOutputPriorPics )
    {
      pcPicTmp->neededForOutput = false;
    }
  }
}

/** Function for checking if picture should be skipped because of random access. This function checks the skipping of pictures in the case of -s option random access.
 *  All pictures prior to the random access point indicated by the counter iSkipFrame are skipped.
 *  It also checks the type of Nal unit type at the random access point.
 *  If the random access point is CRA/CRANT/BLA/BLANT, TFD pictures with POC less than the POC of the random access point are skipped.
 *  If the random access point is IDR all pictures after the random access point are decoded.
 *  If the random access point is none of the above, a warning is issues, and decoding of pictures with POC
 *  equal to or greater than the random access point POC is attempted. For non IDR/CRA/BLA random
 *  access point there is no guarantee that the decoder will not crash.
 */
bool DecLibParser::isRandomAccessSkipPicture()
{
  if (m_pocRandomAccess == MAX_INT) // start of random access point, m_pocRandomAccess has not been set yet.
  {
#if GDR_ADJ
    if (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR )
#else
    if (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA )
#endif
    {
      // set the POC random access since we need to skip the reordered pictures in the case of CRA/CRANT/BLA/BLANT.
      m_pocRandomAccess = m_apcSlicePilot->getPOC();
    }
    else if ( m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
           || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP )
    {
      m_pocRandomAccess = -MAX_INT; // no need to skip the reordered pictures in IDR, they are decodable.
    }
    else
    {
      if(!m_warningMessageSkipPicture)
      {
        msg( WARNING, "Warning: this is not a valid random access point and the data is discarded until the first CRA picture\n");
        m_warningMessageSkipPicture = true;
      }
      return true;
    }
  }
  // skip the reordered pictures, if necessary
  else if (m_apcSlicePilot->getPOC() < m_pocRandomAccess && (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL))
  {
    return true;
  }
  // if we reach here, then the picture is not skipped.
  return false;
}

void DecLibParser::xCheckMixedNalUnit( Slice* pcSlice, InputNALUnit &nalu )
{
  if( pcSlice->getPPS()->getMixedNaluTypesInPicFlag() )
  {
    CHECK(pcSlice->getPPS()->getNumSlicesInPic() < 2, "mixed nal unit type picture, but with less than 2 slices");

    CHECK( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR, "picture with mixed NAL unit type cannot have GDR slice");

    //Check that if current slice is IRAP type, the other type of NAL can only be TRAIL_NUT
    if( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA )
    {
      for( int i = 0; i < m_uiSliceSegmentIdx; i++ )
      {
        Slice* PreSlice = m_pcParsePic->slices[i];
        CHECK( (pcSlice->getNalUnitType() != PreSlice->getNalUnitType()) && (PreSlice->getNalUnitType() != NAL_UNIT_CODED_SLICE_TRAIL), "In a mixed NAL unt type picture, an IRAP slice can be mixed with Trail slice(s) only");
      }
    }

    // if this is the last slice of the picture, check whether that there are at least two different NAL unit types in the picture
    if( pcSlice->getPPS()->getNumSlicesInPic() == (m_uiSliceSegmentIdx + 1) )
    {
      bool hasDiffTypes = false;
      for( int i = 1; !hasDiffTypes && i <= m_uiSliceSegmentIdx; i++ )
      {
        Slice* slice1 = m_pcParsePic->slices[i-1];
        Slice* slice2 = m_pcParsePic->slices[i];
        if( slice1->getNalUnitType() != slice2->getNalUnitType())
        {
          hasDiffTypes = true;
        }
      }
      CHECK( !hasDiffTypes, "VCL NAL units of the picture shall have two or more different nal_unit_type values");
    }

    const unsigned  ctuRsAddr = pcSlice->getCtuAddrInSlice(0);
    const unsigned  ctuXPosInCtus = ctuRsAddr % pcSlice->getPPS()->getPicWidthInCtu();
    const unsigned  ctuYPosInCtus = ctuRsAddr / pcSlice->getPPS()->getPicWidthInCtu();
    const unsigned  maxCUSize = pcSlice->getSPS()->getMaxCUWidth();
    Position pos(ctuXPosInCtus*maxCUSize, ctuYPosInCtus*maxCUSize);
    const SubPic &curSubPic = pcSlice->getPPS()->getSubPicFromPos(pos);

    // check subpicture constraints
    if ((pcSlice->getNalUnitType() >= NAL_UNIT_CODED_SLICE_IDR_W_RADL) && (pcSlice->getNalUnitType() <= NAL_UNIT_CODED_SLICE_CRA))
    {
      CHECK(curSubPic.getTreatedAsPicFlag() != true, "a slice of IDR_W_RADL to CRA_NUT shall have its subpic's sub_pic_treated_as_pic_flag equal to 1");
    }
    else
    {
      // check reference list constraint
      if (!m_nalUnitInfo[nalu.m_nuhLayerId].empty())
      {
        //find out the closest IRAP nal unit that are in the same layer and in the corresponding subpicture
        NalUnitInfo *latestIRAPNalUnit = nullptr;
        int size = (int)m_nalUnitInfo[nalu.m_nuhLayerId].size();
        int naluIdx;
        for (naluIdx = size - 1; naluIdx >= 0; naluIdx--)
        {
          NalUnitInfo *iterNalu = &m_nalUnitInfo[nalu.m_nuhLayerId][naluIdx];
          bool isIRAPSlice = iterNalu->m_nalUnitType >= NAL_UNIT_CODED_SLICE_IDR_W_RADL && iterNalu->m_nalUnitType <= NAL_UNIT_CODED_SLICE_CRA;
          if (isIRAPSlice)
          {
            latestIRAPNalUnit = iterNalu;
            break;
          }
        }
        if (latestIRAPNalUnit != nullptr && ((latestIRAPNalUnit->m_nalUnitType >= NAL_UNIT_CODED_SLICE_IDR_W_RADL && latestIRAPNalUnit->m_nalUnitType <= NAL_UNIT_CODED_SLICE_IDR_N_LP)
            || (latestIRAPNalUnit->m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA && pcSlice->getPOC() > latestIRAPNalUnit->m_POC)))
        {
          // clear the nalu unit before the latest IRAP slice
          m_nalUnitInfo[nalu.m_nuhLayerId].erase(m_nalUnitInfo[nalu.m_nuhLayerId].begin(), m_nalUnitInfo[nalu.m_nuhLayerId].begin() + naluIdx);

          const unsigned  ctuRsAddrIRAP = latestIRAPNalUnit->m_firstCTUinSlice;
          const unsigned  ctuXPosInCtusIRAP = ctuRsAddrIRAP % pcSlice->getPPS()->getPicWidthInCtu();
          const unsigned  ctuYPosInCtusIRAP = ctuRsAddrIRAP / pcSlice->getPPS()->getPicWidthInCtu();
          Position posIRAP(ctuXPosInCtusIRAP*maxCUSize, ctuYPosInCtusIRAP*maxCUSize);
          bool isInCorrespondingSubpic = curSubPic.isContainingPos(posIRAP);
          if (isInCorrespondingSubpic)
          {
            // check RefPicList[0]
            for (int refIdx = 0; refIdx < pcSlice->getNumRefIdx(REF_PIC_LIST_0); refIdx++)
            {
              int POC = pcSlice->getRefPOC(REF_PIC_LIST_0, refIdx);
              bool notInPOCAfterIRAP = true;
              // check all ref pics of the current slice are from poc after the IRAP slice
              for (auto iterNalu : m_nalUnitInfo[nalu.m_nuhLayerId])
              {
                if (POC == iterNalu.m_POC)
                  notInPOCAfterIRAP = false;
              }
              CHECK(notInPOCAfterIRAP, "all reference pictures of a slice after the IRAP picture are from pictures after the IRAP");
            }
            // check RefPicList[1]
            for (int refIdx = 0; refIdx < pcSlice->getNumRefIdx(REF_PIC_LIST_1); refIdx++)
            {
              int POC = pcSlice->getRefPOC(REF_PIC_LIST_1, refIdx);
              bool notInPOCAfterIRAP = true;
              // check all ref pics of the current slice are from poc after the IRAP slice
              for (auto iterNalu : m_nalUnitInfo[nalu.m_nuhLayerId])
              {
                if (POC == iterNalu.m_POC)
                  notInPOCAfterIRAP = false;
              }
              CHECK(notInPOCAfterIRAP, "all reference pictures of a slice after the IRAP picture are from pictures after the IRAP");
            }
          }
        }
      }
    }
  }
  else // all slices shall have the same nal unit type
  {
    bool sameNalUnitType = true;
    for( int i = 0; i < m_uiSliceSegmentIdx; i++ )
    {
      Slice *PreSlice = m_pcParsePic->slices[i];
      if( PreSlice->getNalUnitType() != pcSlice->getNalUnitType() )
      {
        sameNalUnitType = false;
      }
    }
    CHECK(!sameNalUnitType, "mixed_nalu_types_in_pic_flag is zero, but have different nal unit types");
  }
}

}
