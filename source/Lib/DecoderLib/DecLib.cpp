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

/** \file     DecLib.cpp
    \brief    decoder wrapper class
*/

#include "DecLib.h"

#include "CommonLib/dtrace_next.h"
#include "CommonLib/dtrace_buffer.h"
#include "CommonLib/TimeProfiler.h"

#include "CommonLib/x86/CommonDefX86.h"

#include "NALread.h"


namespace vvdec
{

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

void DecLib::create( int numDecThreads, int parserFrameDelay, const UserAllocator& userAllocator, ErrHandlingFlags errHandlingFlags )
{
  // run constructor again to ensure all variables, especially in DecLibParser have been reset
  this->~DecLib();
  new( this ) DecLib;

  if( numDecThreads < 0 )
  {
    numDecThreads = std::thread::hardware_concurrency();
  }

  m_decodeThreadPool.reset( new ThreadPool( numDecThreads, "DecThread" ) );

  if( parserFrameDelay < 0 )
  {
    CHECK_FATAL( numDecThreads < 0, "invalid number of threads" );
    parserFrameDelay = std::min<int>( ( numDecThreads * DEFAULT_PARSE_DELAY_FACTOR ) >> 4, DEFAULT_PARSE_DELAY_MAX );
  }
  m_parseFrameDelay = parserFrameDelay;

  bool upscalingEnabled = false;
  m_picListManager.create( m_parseFrameDelay, (int) m_decLibRecon.size(), userAllocator );
  m_decLibParser.create  ( m_decodeThreadPool.get(), m_parseFrameDelay, (int) m_decLibRecon.size(), numDecThreads, errHandlingFlags );

  int id=0;
  for( auto &dec: m_decLibRecon )
  {
    dec.create( m_decodeThreadPool.get(), id++, upscalingEnabled );
  }

  std::stringstream cssCap;
  cssCap << "THREADS="     << numDecThreads << "; "
         << "PARSE_DELAY=" << parserFrameDelay << "; ";
#if ENABLE_SIMD_OPT
#  if defined( TARGET_SIMD_X86 )
  cssCap << "SIMD=" << read_x86_extension_name();
#  else
  cssCap << "SIMD=SCALAR";
#  endif
#else
  cssCap << "SIMD=NONE";
#endif

  m_sDecoderCapabilities = cssCap.str();

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

Picture* DecLib::decode( InputNALUnit& nalu )
{
  PROFILER_SCOPE_AND_STAGE( 1, g_timeProfiler, P_NALU_SLICE_PIC_HL );

  bool newPic = false;
  if( m_iMaxTemporalLayer < 0 || nalu.m_temporalId <= m_iMaxTemporalLayer )
  {
    newPic = m_decLibParser.parse( nalu );
  }

  if( newPic )
  {
    Picture* pcParsedPic = m_decLibParser.getNextDecodablePicture();
    if( pcParsedPic )
    {
      while( pcParsedPic->error || pcParsedPic->wasLost || pcParsedPic->parseDone.hasException() )
      {
        CHECK_FATAL( pcParsedPic->progress >= Picture::reconstructing, "The error picture shouldn't be in reconstructing state yet." );

        std::exception_ptr parsing_exception = pcParsedPic->parseDone.hasException() ? pcParsedPic->parseDone.getException() : nullptr;
        if( parsing_exception )
        {   // the exception has not been thrown out of the library. Do that after preparing this picture for referencing
          pcParsedPic->error = true;
          pcParsedPic->waitForAllTasks();
          pcParsedPic->parseDone.clearException();
        }

        pcParsedPic->waitForAllTasks();

        if( pcParsedPic->progress < Picture::parsing )
        {
          // we don't know if all structures are there yet, so we init them
          pcParsedPic->ensureUsableAsRef();
        }
        pcParsedPic->fillGrey( m_decLibParser.getParameterSetManager().getFirstSPS() );

        // need to finish picture here, because it won't go through declibRecon
        finishPicture( pcParsedPic );

        // this exception has not been thrown outside (error must have happened in slice parsing task)
        if( parsing_exception )
        {
          CHECK_FATAL( pcParsedPic->exceptionThrownOut, "The exception shouldn't have been thrown out already." );
          pcParsedPic->exceptionThrownOut = true;
          std::rethrow_exception( parsing_exception );
        }

        // try again to get a picture, that we can reconstruct now
        pcParsedPic = m_decLibParser.getNextDecodablePicture();
      }

      reconPicture( pcParsedPic );
    }
  }

  if( newPic || nalu.m_nalUnitType == NAL_UNIT_EOS )
  {
    Picture* outPic = getNextOutputPic( false );
    if( outPic )
    {
      CHECK_WARN( outPic->progress < Picture::finished, "Picture should have been finished by now. Blocking and finishing..." );
      if( outPic->progress < Picture::finished )
      {
        blockAndFinishPictures( outPic );

        CHECK( outPic->progress < Picture::finished, "Picture still not finished. Something is really broken." );
      }

      m_checkMissingOutput = true;
    }

    // warn if we don't produce an output picture for every incoming picture
    CHECK_WARN( m_checkMissingOutput && !outPic, "missing output picture" ); // this CHECK_FATAL is not needed in flushPic(), because in flushPic() the nullptr signals the end of the bitstream
    return outPic;
  }

  return nullptr;
}

Picture* DecLib::flushPic()
{
  Picture* outPic = getNextOutputPic( false );
  // at end of file, fill the decompression queue and decode pictures until the next output-picture is finished
  while( Picture* pcParsedPic = m_decLibParser.getNextDecodablePicture() )
  {
    // reconPicture() blocks and finishes one picture on each call
    reconPicture( pcParsedPic );

    if( !outPic )
    {
      outPic = getNextOutputPic( false );
    }
    if( outPic && outPic->progress == Picture::finished )
    {
      return outPic;
    }
  }

  if( outPic && outPic->progress == Picture::finished )
  {
    return outPic;
  }

  // if all pictures have been parsed, but not finished, iteratively wait for and finish next pictures
  blockAndFinishPictures( outPic );
  if( !outPic )
  {
    outPic = getNextOutputPic( false );
  }
  if( outPic && outPic->progress == Picture::finished )
  {
    return outPic;
  }

  CHECK( outPic, "we shouldn't be holding an output picture here" );
  // flush remaining pictures without considering num reorder pics
  outPic = getNextOutputPic( true );
  if( outPic )
  {
    CHECK( outPic->progress != Picture::finished, "all pictures should have been finished by now" );
    // outPic->referenced = false;
    return outPic;
  }

  // At the very end reset parser state
  InputNALUnit eosNAL;
  eosNAL.m_nalUnitType = NAL_UNIT_EOS;
  m_decLibParser.parse( eosNAL );
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

  Slice*  pcSlice = pcPic->slices[0];
  if( pcPic->wasLost || pcPic->error || pcPic->reconDone.hasException() )
  {
    msg( msgl, "POC %4d LId: %2d TId: %1d %s\n", pcPic->poc, pcPic->layerId, pcSlice->getTLayer(), pcPic->wasLost ? "LOST" : "ERROR" );
    pcPic->progress = Picture::finished;

    // if the picture has an exception set (originating from thread-pool tasks), don't return here, but rethrow the exception
    try
    {
      pcPic->parseDone.checkAndRethrowException();
      pcPic->reconDone.checkAndRethrowException();
    }
    catch( ... )
    {
      pcPic->waitForAllTasks();

      // need to clear exception so we can use it as reference picture
      pcPic->reconDone.clearException();
      pcPic->reconDone.unlock();
      pcPic->error = true;
      if( !pcPic->exceptionThrownOut )
      {
        pcPic->exceptionThrownOut = true;
        throw;
      }
    }

    return pcPic->poc;
  }

  ITT_TASKSTART( itt_domain_oth, itt_handle_finish );

  char c = ( pcSlice->isIntra() ? 'I' : pcSlice->isInterP() ? 'P' : 'B' );
  if( !pcPic->isReferencePic )
  {
    c += 32;  // tolower
  }

  //-- For time output for each slice
  msg( msgl, "POC %4d LId: %2d TId: %1d ( %c-SLICE, QP%3d ) ", pcPic->poc, pcPic->layerId,
         pcSlice->getTLayer(),
         c,
         pcSlice->getSliceQp() );
  msg( msgl, "[DT %6.3f] ", pcPic->getProcessingTime() );

  for (int iRefList = 0; iRefList < 2; iRefList++)
  {
    msg( msgl, "[L%d ", iRefList);
    for (int iRefIndex = 0; iRefIndex < pcSlice->getNumRefIdx(RefPicList(iRefList)); iRefIndex++)
    {
#if RPR_OUTPUT_MSG
      const std::pair<int, int>& scaleRatio = pcSlice->getScalingRatio( RefPicList( iRefList ), iRefIndex );
      
      if( pcSlice->getPicHeader()->getEnableTMVPFlag() && pcSlice->getColFromL0Flag() == bool(1 - iRefList) && pcSlice->getColRefIdx() == iRefIndex )
      {
        if( scaleRatio.first != 1 << SCALE_RATIO_BITS || scaleRatio.second != 1 << SCALE_RATIO_BITS )
        {
          msg( msgl, "%dc(%1.2lfx, %1.2lfx) ", pcSlice->getRefPOC( RefPicList( iRefList ), iRefIndex ), double( scaleRatio.first ) / ( 1 << SCALE_RATIO_BITS ), double( scaleRatio.second ) / ( 1 << SCALE_RATIO_BITS ) );
        }
        else
        {
          msg( msgl, "%dc ", pcSlice->getRefPOC( RefPicList( iRefList ), iRefIndex ) );
        }
      }
      else
      {
        if( scaleRatio.first != 1 << SCALE_RATIO_BITS || scaleRatio.second != 1 << SCALE_RATIO_BITS )
        {
          msg( msgl, "%d(%1.2lfx, %1.2lfx) ", pcSlice->getRefPOC( RefPicList( iRefList ), iRefIndex ), double( scaleRatio.first ) / ( 1 << SCALE_RATIO_BITS ), double( scaleRatio.second ) / ( 1 << SCALE_RATIO_BITS ) );
        }
        else
        {
      msg( msgl, "%d ", pcSlice->getRefPOC(RefPicList(iRefList), iRefIndex));
    }
      }
#else
      msg( msgl, "%d ", pcSlice->getRefPOC(RefPicList(iRefList), iRefIndex));
#endif
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

  m_picListManager.markUnusedPicturesReusable();

  if( m_parseFrameDelay > 0 )
  {
    checkPictureHashSEI( pcPic );
  }

  ITT_TASKEND( itt_domain_oth, itt_handle_finish );

  pcPic->progress = Picture::finished;

  return pcSlice->getPOC();
}

void DecLib::checkPictureHashSEI( Picture* pcPic )
{
  if( !m_decodedPictureHashSEIEnabled )
  {
    return;
  }
  if( !pcPic->neededForOutput || pcPic->picCheckedDPH )
  {
    return;
  }

  CHECK( pcPic->progress < Picture::reconstructed, "picture not reconstructed" );

  seiMessages pictureHashes = SEI_internal::getSeisByType( pcPic->seiMessageList, VVDEC_DECODED_PICTURE_HASH );
  if( !pictureHashes.empty() )
  {
    if( pictureHashes.size() > 1 )
    {
      msg( WARNING, "Warning: Got multiple decoded picture hash SEI messages. Using first." );
    }

    const vvdecSEIDecodedPictureHash* hash = (vvdecSEIDecodedPictureHash*)pictureHashes.front()->payload;

    msg( INFO, "         " );
    const int hashErrors              = calcAndPrintHashStatus( pcPic->getRecoBuf(), hash, pcPic->cs->sps->getBitDepths(), INFO );
    m_numberOfChecksumErrorsDetected += hashErrors;
    pcPic->dphMismatch                = !!hashErrors;
    pcPic->picCheckedDPH              = true;
    msg( INFO, "\n" );
  }
  else
  {
    if( pcPic->subPictures.empty() )
    {
      msg( WARNING, "Warning: missing decoded picture hash SEI message.\n" );
      return;
    }

    seiMessages scalableNestingSeis = SEI_internal::getSeisByType( pcPic->seiMessageList, VVDEC_SCALABLE_NESTING );
    for( auto* seiIt: scalableNestingSeis )
    {
      CHECK( seiIt->payloadType != VVDEC_SCALABLE_NESTING, "expected nesting SEI" );

      const vvdecSEIScalableNesting* nestingSei = (vvdecSEIScalableNesting*) seiIt->payload;
      if( !nestingSei->snSubpicFlag )
      {
        continue;
      }

      for( int i = 0; i < nestingSei->snNumSEIs; ++i )
      {
        auto& nestedSei = nestingSei->nestedSEIs[i];
        CHECK( nestedSei == nullptr, "missing nested sei" );
        if( nestedSei && nestedSei->payloadType != VVDEC_DECODED_PICTURE_HASH )
          continue;

        const vvdecSEIDecodedPictureHash* hash = (vvdecSEIDecodedPictureHash*) nestedSei->payload;

        if( pcPic->subpicsCheckedDPH.empty() )
        {
          pcPic->subpicsCheckedDPH.resize( pcPic->subPictures.size(), false );
        }
        else
        {
          CHECK( pcPic->subpicsCheckedDPH.size() != pcPic->subPictures.size(), "Picture::subpicsCheckedDPH not properly initialized" );
        }

        for( int j = 0; j < nestingSei->snNumSubpics; ++j )
        {
          uint32_t subpicId = nestingSei->snSubpicId[j];

          for( auto& subPic: pcPic->subPictures )
          {
            if( subPic.getSubPicID() != subpicId )
              continue;

            auto subPicIdx = subPic.getSubPicIdx();
            if( pcPic->subpicsCheckedDPH[subPicIdx] )
              continue;

            const UnitArea area                 = UnitArea( pcPic->chromaFormat, subPic.getLumaArea() );
            const int      hashErrors           = calcAndPrintHashStatus( pcPic->cs->getRecoBuf( area ), hash, pcPic->cs->sps->getBitDepths(), INFO );
            m_numberOfChecksumErrorsDetected   += hashErrors;
            pcPic->dphMismatch                 |= !!hashErrors;
            pcPic->subpicsCheckedDPH[subPicIdx] = true;
            msg( INFO, "\n" );
          }
        }
      }
    }

    size_t checkedSubpicCount = std::count( pcPic->subpicsCheckedDPH.cbegin(), pcPic->subpicsCheckedDPH.cend(), true );
    pcPic->picCheckedDPH      = ( checkedSubpicCount == pcPic->subPictures.size() );   // mark when all subpics have been checked

    if( m_parseFrameDelay )
    {
      // this warning is only enabled, when running with parse delay enabled, because otherwise we don't know here if the last DPH Suffix-SEI has already been
      // parsed
      if( checkedSubpicCount != pcPic->subPictures.size() )
      {
        msg( WARNING, "Warning: missing decoded picture hash SEI message for SubPics (%u/%u).\n", checkedSubpicCount, pcPic->subPictures.size() );
      }
    }
  }
}

Picture* DecLib::getNextOutputPic( bool bFlush )
{
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

  return m_picListManager.getNextOutputPic( numReorderPicsHighestTid, maxDecPicBufferingHighestTid, bFlush );
}

void DecLib::reconPicture( Picture* pcPic )
{
  CHECK_FATAL( std::any_of( m_decLibRecon.begin(), m_decLibRecon.end(), [=]( auto& rec ) { return rec.getCurrPic() == pcPic; } ),
         "(Reused) Picture structure is still in progress in decLibRecon." );

  DecLibRecon* reconInstance = &m_decLibRecon.front();
  move_to_end( m_decLibRecon.begin(), m_decLibRecon );

  Picture* donePic = reconInstance->waitForPrevDecompressedPic();
  try
  {
    reconInstance->decompressPicture( pcPic );
  }
  catch( ... )
  {
    pcPic->reconDone.setException( std::current_exception() );
    pcPic->error = true;
  }

  if( donePic )
  {
    finishPicture( donePic );
  }
}

void DecLib::blockAndFinishPictures( Picture* pcPic )
{
  // find Recon instance where current picture (if not null) is active and ensure the picture gets finished
  // otherwise all pictures get finished
  for( auto& recon: m_decLibRecon )
  {
    if( pcPic && recon.getCurrPic() != pcPic )
    {
      continue;
    }

    if( Picture* donePic = recon.waitForPrevDecompressedPic() )
    {
      finishPicture( donePic );
    }
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
    InputBitstream bs = sei.getBitstream();

    do
    {
      int payloadType = 0;
      uint32_t val = 0;

      do
      {
        val = bs.readByte();
        payloadType += val;
      } while (val==0xFF);

      uint32_t payloadSize = 0;
      do
      {
        val = bs.readByte();
        payloadSize += val;
      } while (val==0xFF);

      uint8_t *payload = new uint8_t[payloadSize];
      for( uint32_t i = 0; i < payloadSize; i++ )
      {
        val = bs.readByte();
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
  m_pictureSeiNalus.clear();
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

}
