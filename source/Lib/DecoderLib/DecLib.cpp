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

/** \file     DecLib.cpp
    \brief    decoder wrapper class
*/

#include "DecLib.h"

#include "CommonLib/dtrace_next.h"
#include "CommonLib/dtrace_buffer.h"
#include "CommonLib/TimeProfiler.h"

#include "NALread.h"

#ifdef TRACE_ENABLE_ITT
std::vector<__itt_domain*> itt_domain_decInst;
__itt_domain* itt_domain_dec           = __itt_domain_create( "Decode" );
__itt_domain* itt_domain_prs           = __itt_domain_create( "Parse" );
__itt_domain* itt_domain_oth           = __itt_domain_create( "Other" );

__itt_string_handle* itt_handle_alf    = __itt_string_handle_create( "ALF_CTU" );
__itt_string_handle* itt_handle_presao = __itt_string_handle_create( "PreSAO_Line" );
__itt_string_handle* itt_handle_sao    = __itt_string_handle_create( "SAO_CTU" );
__itt_string_handle* itt_handle_lfl    = __itt_string_handle_create( "LFL_CTU" );
__itt_string_handle* itt_handle_intra  = __itt_string_handle_create( "Intra_CTU" );
__itt_string_handle* itt_handle_inter  = __itt_string_handle_create( "Inter_CTU" );
__itt_string_handle* itt_handle_mider  = __itt_string_handle_create( "MI-Der_CTU" );
__itt_string_handle* itt_handle_lfcl   = __itt_string_handle_create( "Prep_ClearLF" );
__itt_string_handle* itt_handle_ext    = __itt_string_handle_create( "Prep_ExtBrdr" );
__itt_string_handle* itt_handle_dmvr   = __itt_string_handle_create( "MI-DMVR" );
__itt_string_handle* itt_handle_rsp    = __itt_string_handle_create( "Reshape_CTU" );

__itt_string_handle* itt_handle_parse  = __itt_string_handle_create( "Parse_Slice" );

__itt_string_handle* itt_handle_start  = __itt_string_handle_create( "Start_Pic" );
__itt_string_handle* itt_handle_done   = __itt_string_handle_create( "Pic_Done" );
__itt_string_handle* itt_handle_finish = __itt_string_handle_create( "Finish_Pic" );

__itt_string_handle* itt_handle_schedTasks = __itt_string_handle_create( "Scheduling_Tasks" );
__itt_string_handle* itt_handle_waitTasks  = __itt_string_handle_create( "Wait_for_Dec_Tasks" );


// create global domain for DecLib
__itt_domain* itt_domain_glb    = __itt_domain_create ( "Global" );
// create a global counter
__itt_counter itt_frame_counter = __itt_counter_create( "FrameNumber", "Global" );

#define ITT_TASKSTART( d, t ) __itt_task_begin( ( d ), __itt_null, __itt_null, ( t ) )
#define ITT_TASKEND( d, t )   __itt_task_end  ( ( d ) )
#else
#define ITT_TASKSTART( d, t )
#define ITT_TASKEND( d, t )
#endif

//! \ingroup DecoderLib
//! \{

DecLib::DecLib()
{
#ifdef TRACE_ENABLE_ITT
  itt_domain_dec->flags = 1;
  itt_domain_prs->flags = 1;
  itt_domain_glb->flags = 1;
  itt_domain_oth->flags = 1;
#endif
}

void DecLib::create(int numDecThreads, int parserFrameDelay)
{
  // run constructor again to ensure all variables, especially in DecLibParser have been reset
  this->~DecLib();
  new( this ) DecLib;

  if( numDecThreads < 0 )
  {
    numDecThreads = std::thread::hardware_concurrency();
  }

  m_decodeThreadPool.reset( new NoMallocThreadPool( numDecThreads, "DecThread" ) );

  if( parserFrameDelay < 0 )
  {
    CHECK( numDecThreads < 0, "invalid number of threads" );
    parserFrameDelay = numDecThreads;
  }
  m_parseFrameDelay = parserFrameDelay;

  m_picListManager.create( m_parseFrameDelay, ( int ) m_decLibRecon.size() );
  m_decLibParser.create  ( m_decodeThreadPool.get(), m_parseFrameDelay, ( int ) m_decLibRecon.size(), numDecThreads );
    
  int id=0;
  for( auto &dec: m_decLibRecon )
  {
    dec.create( m_decodeThreadPool.get(), id++ );
  }

  std::stringstream cssCap;
  cssCap << "THREADS="     << numDecThreads << "; "
         << "PARSE_DELAY=" << parserFrameDelay << "; ";
#if ENABLE_SIMD_OPT
  std::string cSIMD;
  cssCap << "SIMD=" << read_x86_extension( cSIMD );
#else
  cssCap << "SIMD=NONE";
#endif

  m_sDecoderCapabilities = cssCap.str();
  msg( INFO, "[%s]\n", m_sDecoderCapabilities.c_str() );

  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "final", 1 ) );
}

void DecLib::destroy()
{
  if( m_decodeThreadPool )
  {
    m_decodeThreadPool->shutdown( true );
    m_decodeThreadPool.reset();
  }

  m_decLibParser.destroy();
  for( auto &dec: m_decLibRecon )
  {
    dec.destroy();
  }

  m_picListManager.deleteBuffers();
}

#if JVET_P0288_PIC_OUTPUT
Picture* DecLib::decode( InputNALUnit& nalu, int* pSkipFrame, int iTargetLayer )
#else
Picture* DecLib::decode( InputNALUnit& nalu, int* pSkipFrame )
#endif
{
  PROFILER_SCOPE_AND_STAGE( 1, g_timeProfiler, P_NALU_SLICE_PIC_HL );
  Picture * pcParsedPic = nullptr;
  if( m_iMaxTemporalLayer >= 0 && nalu.m_temporalId > m_iMaxTemporalLayer )
  {
    pcParsedPic = nullptr;
  }
  else
  {
#if JVET_P0288_PIC_OUTPUT
    pcParsedPic = m_decLibParser.parse( nalu, pSkipFrame, iTargetLayer );
#else
    pcParsedPic = m_decLibParser.parse( nalu, pSkipFrame );
#endif
  }

  if( pcParsedPic )
  {
    this->decompressPicture( pcParsedPic );
  }

  if(      pcParsedPic
      ||   nalu.m_nalUnitType == NAL_UNIT_EOS
      || ( nalu.m_nalUnitType >= NAL_UNIT_CODED_SLICE_TRAIL      && nalu.m_nalUnitType <= NAL_UNIT_RESERVED_IRAP_VCL_12 )
      || ( nalu.m_nalUnitType >= NAL_UNIT_CODED_SLICE_IDR_W_RADL && nalu.m_nalUnitType <= NAL_UNIT_CODED_SLICE_GDR )
    )
  {
    Picture* outPic = getNextOutputPic( false );
    CHECK_WARN( m_checkMissingOutput && !outPic, "missing output picture" ); // we don't need this CHECK in flushPic(), because flushPic() is usually only called until the first nullptr is returned
    if( outPic )
    {
      m_checkMissingOutput = true;
    }
    return outPic;
  }

  return nullptr;
}

Picture* DecLib::flushPic()
{
  // at end of file, fill the decompression queue and decode pictures until we get one out
  while( Picture* pcParsedPic = m_decLibParser.getNextDecodablePicture() )
  {
    this->decompressPicture( pcParsedPic );

    if( Picture* outPic = getNextOutputPic( false ) )
    {
      return outPic;
    }
  }

  // first try to get a picture without waiting for the decoder
  if( Picture* outPic = getNextOutputPic( false ) )
  {
    return outPic;
  }

  // if no picture is done, actually block and wait
  if( Picture* outPic = getNextOutputPic( true ) )
  {
    return outPic;
  }

  // At the very end reset parser state
  InputNALUnit eosNAL;
  eosNAL.m_nalUnitType = NAL_UNIT_EOS;
  m_decLibParser.parse( eosNAL, nullptr );
  m_checkMissingOutput = false;

  return nullptr;
}

#if JVET_R0270
int DecLib::finishPicture( Picture* pcPic, MsgLevel msgl, bool associatedWithNewClvs )
#else
int DecLib::finishPicture( Picture* pcPic, MsgLevel msgl )
#endif
{
#ifdef TRACE_ENABLE_ITT
  // increment Framecounter
  __itt_counter_inc( itt_frame_counter );
#endif

  Slice*  pcSlice = pcPic->cs->slice;
  if( pcPic->wasLost )
  {
    msg( msgl, "POC %4d TId: %1d LOST\n", pcPic->poc, pcSlice->getTLayer() );
    pcPic->reconstructed = true;
    return pcPic->poc;
  }

  ITT_TASKSTART( itt_domain_oth, itt_handle_finish );

  char c = ( pcSlice->isIntra() ? 'I' : pcSlice->isInterP() ? 'P' : 'B' );
  if( !pcPic->referenced )
  {
    c += 32;  // tolower
  }

  //-- For time output for each slice
  msg( msgl, "POC %4d LId: %2d TId: %1d ( %c-SLICE, QP%3d ) ", pcPic->poc, pcPic->layerId,
         pcSlice->getTLayer(),
         c,
         pcSlice->getSliceQp() );
  msg( msgl, "[DT %6.3f] ", pcSlice->getProcessingTime() );

  for (int iRefList = 0; iRefList < 2; iRefList++)
  {
    msg( msgl, "[L%d ", iRefList);
    for (int iRefIndex = 0; iRefIndex < pcSlice->getNumRefIdx(RefPicList(iRefList)); iRefIndex++)
    {
      msg( msgl, "%d ", pcSlice->getRefPOC(RefPicList(iRefList), iRefIndex));
    }
    msg( msgl, "] ");
  }

  msg( msgl, "\n");

//  pcPic->neededForOutput = (pcSlice->getPicHeader()->getPicOutputFlag() ? true : false);
#if JVET_R0270
  if (associatedWithNewClvs && pcPic->neededForOutput)
  {
    if (!pcSlice->getPPS()->getMixedNaluTypesInPicFlag() && pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL)
    {
      pcPic->neededForOutput = false;
    }
    else if (pcSlice->getPPS()->getMixedNaluTypesInPicFlag())
    {
      bool isRaslPic = true;
      for (int i = 0; isRaslPic && i < pcPic->numSlices; i++)
      {
        if (!(pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL))
        {
          isRaslPic = false;
        }
      }
      if (isRaslPic)
      {
        pcPic->neededForOutput = false;
      }
    }
  }
#endif

  m_picListManager.applyDoneReferencePictureMarking();

#if JVET_Q0044_SLICE_IDX_WITH_SUBPICS
  m_maxDecSubPicIdx = 0;
  m_maxDecSliceAddrInSubPic = -1;
#endif

  if( m_parseFrameDelay > 0 )
  {
    checkPictureHashSEI( pcPic );
  }

  ITT_TASKEND( itt_domain_oth, itt_handle_finish );

  return pcSlice->getPOC();
}

void DecLib::checkPictureHashSEI( Picture* pcPic )
{
  if( !m_decodedPictureHashSEIEnabled )
  {
    return;
  }

  CHECK( !pcPic->reconstructed, "picture not reconstructed" );

  SEIMessages                  pictureHashes = getSeisByType( pcPic->SEIs, SEI::DECODED_PICTURE_HASH );
  const SEIDecodedPictureHash* hash          = pictureHashes.size() > 0 ? (SEIDecodedPictureHash*)pictureHashes.front() : nullptr;
  if( pictureHashes.empty() )
  {
    msg( WARNING, "Warning: missing decoded picture hash SEI message.\n" );
    return;
  }
  if( pictureHashes.size() > 1 )
  {
    msg( WARNING, "Warning: Got multiple decoded picture hash SEI messages. Using first." );
  }
  msg( INFO, "         " );
  m_numberOfChecksumErrorsDetected += calcAndPrintHashStatus( pcPic->getRecoBuf(), hash, pcPic->cs->slice->getSPS()->getBitDepths(), INFO );
  msg( INFO, "\n" );
}

Picture* DecLib::getNextOutputPic( bool bFlush )
{
  if( bFlush )
  {
    // wait for last pictures in bitstream
    for( auto & dec: m_decLibRecon )
    {
      Picture* donePic = dec.waitForPrevDecompressedPic();
      if( donePic )
      {
        finishPicture( donePic );
      }
    }
  }
  if( m_picListManager.getFrontPic() == nullptr )
  {
    return nullptr;
  }

  const SPS* activeSPS      = m_picListManager.getFrontPic()->cs->sps.get();
  const int  maxNrSublayers = activeSPS->getMaxTLayers();

  int numReorderPicsHighestTid;
  int maxDecPicBufferingHighestTid;
  if( m_iMaxTemporalLayer == -1 || m_iMaxTemporalLayer >= maxNrSublayers )
  {
    numReorderPicsHighestTid     = activeSPS->getNumReorderPics( maxNrSublayers - 1 );
    maxDecPicBufferingHighestTid = activeSPS->getMaxDecPicBuffering( maxNrSublayers - 1 );
  }
  else
  {
    numReorderPicsHighestTid     = activeSPS->getNumReorderPics( m_iMaxTemporalLayer );
    maxDecPicBufferingHighestTid = activeSPS->getMaxDecPicBuffering( m_iMaxTemporalLayer );
  }

  Picture* outPic = m_picListManager.getNextOutputPic( numReorderPicsHighestTid, maxDecPicBufferingHighestTid, bFlush );
  CHECK( outPic && outPic->done.isBlocked(), "next output-pic is not done yet." );
  return outPic;
}

void DecLib::decompressPicture( Picture* pcPic )
{
  DecLibRecon* decLibInstance = &m_decLibRecon.front();
  move_to_end( m_decLibRecon.begin(), m_decLibRecon );

  while( pcPic->wasLost )
  {
    Picture* donePic = decLibInstance->waitForPrevDecompressedPic();
    if( donePic )
    {
      finishPicture( donePic );
    }

    m_decLibParser.recreateLostPicture( pcPic );
    finishPicture( pcPic );

    pcPic = m_decLibParser.getNextDecodablePicture();
    if( !pcPic )
    {
      msg(WARNING, "a lost picture was filled in, but no following picture is available for decoding.");
      return;
    }
  }

  Picture* donePic = decLibInstance->waitForPrevDecompressedPic();

  decLibInstance->decompressPicture( pcPic );

  if( donePic )
  {
    finishPicture( donePic );
  }
}

void DecLib::checkNalUnitConstraints( uint32_t naluType )
{
  const ConstraintInfo *cInfo = NULL;
  if (m_decLibParser.getParameterSetManager().getActiveSPS() != NULL && m_decLibParser.getParameterSetManager().getActiveSPS()->getProfileTierLevel() != NULL)
  {
    cInfo = m_decLibParser.getParameterSetManager().getActiveSPS()->getProfileTierLevel()->getConstraintInfo();
    if( cInfo != NULL )
    {
      xCheckNalUnitConstraintFlags( cInfo, naluType );
    }
  }
}

void DecLib::xCheckNalUnitConstraintFlags( const ConstraintInfo *cInfo, uint32_t naluType )
{
  if( cInfo != NULL )
  {
    CHECK( cInfo->getNoTrailConstraintFlag() && naluType == NAL_UNIT_CODED_SLICE_TRAIL,
           "Non-conforming bitstream. no_trail_constraint_flag is equal to 1 but bitstream contains NAL unit of type TRAIL_NUT." );
    CHECK( cInfo->getNoStsaConstraintFlag()  && naluType == NAL_UNIT_CODED_SLICE_STSA,
           "Non-conforming bitstream. no_stsa_constraint_flag is equal to 1 but bitstream contains NAL unit of type STSA_NUT." );
    CHECK( cInfo->getNoRaslConstraintFlag()  && naluType == NAL_UNIT_CODED_SLICE_RASL,
           "Non-conforming bitstream. no_rasl_constraint_flag is equal to 1 but bitstream contains NAL unit of type RASL_NUT." );
    CHECK( cInfo->getNoRadlConstraintFlag()  && naluType == NAL_UNIT_CODED_SLICE_RADL,
           "Non-conforming bitstream. no_radl_constraint_flag is equal to 1 but bitstream contains NAL unit of type RADL_NUT." );
    CHECK( cInfo->getNoIdrConstraintFlag()   && naluType == NAL_UNIT_CODED_SLICE_IDR_W_RADL,
           "Non-conforming bitstream. no_idr_constraint_flag is equal to 1 but bitstream contains NAL unit of type IDR_W_RADL." );
    CHECK( cInfo->getNoIdrConstraintFlag()   && naluType == NAL_UNIT_CODED_SLICE_IDR_N_LP,
           "Non-conforming bitstream. no_idr_constraint_flag is equal to 1 but bitstream contains NAL unit of type IDR_N_LP." );
    CHECK( cInfo->getNoCraConstraintFlag()   && naluType == NAL_UNIT_CODED_SLICE_CRA,
           "Non-conforming bitstream. no_cra_constraint_flag is equal to 1 but bitstream contains NAL unit of type CRA_NUT." );
    CHECK( cInfo->getNoGdrConstraintFlag()   && naluType == NAL_UNIT_CODED_SLICE_GDR,
           "Non-conforming bitstream. no_gdr_constraint_flag is equal to 1 but bitstream contains NAL unit of type GDR_NUT." );
    CHECK( cInfo->getNoApsConstraintFlag()   && naluType == NAL_UNIT_PREFIX_APS,
           "Non-conforming bitstream. no_aps_constraint_flag is equal to 1 but bitstream contains NAL unit of type APS_PREFIX_NUT." );
    CHECK( cInfo->getNoApsConstraintFlag()   && naluType == NAL_UNIT_SUFFIX_APS,
           "Non-conforming bitstream. no_aps_constraint_flag is equal to 1 but bitstream contains NAL unit of type APS_SUFFIX_NUT." );
  }
}

#define SEI_REPETITION_CONSTRAINT_LIST_SIZE  21

/**
 - Count the number of identical SEI messages in the current picture
 */
void DecLib::checkSeiInPictureUnit()
{
  std::vector<std::tuple<int, uint32_t, uint8_t*>> seiList;

  // payload types subject to constrained SEI repetition
  int picUnitRepConSeiList[SEI_REPETITION_CONSTRAINT_LIST_SIZE] = { 0, 1, 19, 45, 129, 132, 133, 137, 144, 145, 147, 148, 149, 150, 153, 154, 155, 156, 168, 203, 204};
  
  // extract SEI messages from NAL units
  for( auto &sei : m_pictureSeiNalus )
  {
    InputBitstream bs = sei->getBitstream();

    do
    {
      int payloadType = 0;
      uint32_t val = 0;

      do
      {
        bs.readByte(val);
        payloadType += val;
      } while (val==0xFF);

      uint32_t payloadSize = 0;
      do
      {
        bs.readByte(val);
        payloadSize += val;
      } while (val==0xFF);
    
      uint8_t *payload = new uint8_t[payloadSize];
      for( uint32_t i = 0; i < payloadSize; i++ )
      {
        bs.readByte(val);
        payload[i] = (uint8_t)val;
      }
      seiList.push_back(std::tuple<int, uint32_t, uint8_t*>(payloadType, payloadSize, payload));
    }
    while( bs.getNumBitsLeft() > 8 );
  }

  // count repeated messages in list
  for( uint32_t i = 0; i < seiList.size(); i++ )
  {
    int      k, count = 1;
    int      payloadType1 = std::get<0>(seiList[i]);
    uint32_t payloadSize1 = std::get<1>(seiList[i]);
    uint8_t  *payload1    = std::get<2>(seiList[i]);

    // only consider SEI payload types in the PicUnitRepConSeiList
    for( k=0; k<SEI_REPETITION_CONSTRAINT_LIST_SIZE; k++ )
    {
      if( payloadType1 == picUnitRepConSeiList[k] )
      {
        break;
      }
    }
    if( k >= SEI_REPETITION_CONSTRAINT_LIST_SIZE )
    {
      continue;
    }

    // compare current SEI message with remaining messages in the list
    for( uint32_t j = i+1; j < seiList.size(); j++ )
    {
      int      payloadType2 = std::get<0>(seiList[j]);
      uint32_t payloadSize2 = std::get<1>(seiList[j]);
      uint8_t  *payload2    = std::get<2>(seiList[j]);
      
      // check for identical SEI type, size, and payload
      if( payloadType1 == payloadType2 && payloadSize1 == payloadSize2 )
      {
        if( memcmp(payload1, payload2, payloadSize1*sizeof(uint8_t)) == 0 )
        {
          count++;
        }
      }
    }
    CHECK(count > 4, "There shall be less than or equal to 4 identical sei_payload( ) syntax structures within a picture unit.");
  }

  // free SEI message list memory
  for( uint32_t i = 0; i < seiList.size(); i++ )
  {
    uint8_t *payload = std::get<2>(seiList[i]);
    delete   payload;
  }
  seiList.clear();
}

/**
 - Reset list of SEI NAL units from the current picture
 */
void DecLib::resetPictureSeiNalus()
{
  while( !m_pictureSeiNalus.empty() )
  {
    delete m_pictureSeiNalus.front();
    m_pictureSeiNalus.pop_front();
  }
}

void DecLib::checkAPSInPictureUnit()
{
  bool firstVCLFound = false;
  bool suffixAPSFound = false;
  for (auto &nalu : m_pictureUnitNals)
  {
    if (NALUnit::isVclNalUnitType(nalu))
    {
      firstVCLFound = true;
      CHECK( suffixAPSFound, "When any suffix APS NAL units are present in a PU, they shall follow the last VCL unit of the PU" );
    }
    else if (nalu == NAL_UNIT_PREFIX_APS)
    {
      CHECK( firstVCLFound, "When any prefix APS NAL units are present in a PU, they shall precede the first VCL unit of the PU");
    }
    else if (nalu == NAL_UNIT_SUFFIX_APS)
    {
      suffixAPSFound = true;
    }
  }
}

//! \}
