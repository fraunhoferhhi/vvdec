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

/** \file     DecLibRecon.cpp
    \brief    decoder class
*/

#include "DecLib.h"

#include "CommonLib/TrQuant.h"
#if ENABLE_SIMD_TCOEFF_OPS
#include "CommonLib/TrQuant_EMT.h"
#endif
#include "CommonLib/InterPrediction.h"
#include "CommonLib/IntraPrediction.h"
#include "CommonLib/Unit.h"
#include "CommonLib/Buffer.h"
#include "CommonLib/UnitTools.h"

#include "CommonLib/dtrace_next.h"
#include "CommonLib/dtrace_buffer.h"

namespace vvdec
{

#ifdef TRACE_ENABLE_ITT
extern __itt_domain*              itt_domain_dec;
extern std::vector<__itt_domain*> itt_domain_decInst;

extern __itt_string_handle* itt_handle_alf;
extern __itt_string_handle* itt_handle_presao;
extern __itt_string_handle* itt_handle_sao;
extern __itt_string_handle* itt_handle_lfl;
extern __itt_string_handle* itt_handle_intra;
extern __itt_string_handle* itt_handle_inter;
extern __itt_string_handle* itt_handle_mider;
extern __itt_string_handle* itt_handle_lfcl;
extern __itt_string_handle* itt_handle_ext;
extern __itt_string_handle* itt_handle_dmvr;
extern __itt_string_handle* itt_handle_rsp;

extern __itt_string_handle* itt_handle_schedTasks;
extern __itt_string_handle* itt_handle_waitTasks;

// create global domain for DecLib
extern __itt_domain* itt_domain_glb;
// create a global counter
extern __itt_counter itt_frame_counter;

#define ITT_TASKSTART( d, t ) __itt_task_begin( ( d ), __itt_null, __itt_null, ( t ) )
#define ITT_TASKEND( d, t )   __itt_task_end  ( ( d ) )
#else
#define ITT_TASKSTART( d, t )
#define ITT_TASKEND( d, t )
#endif

//! \ingroup DecoderLib
//! \{

void CommonTaskParam::reset( CodingStructure& cs, TaskType ctuStartState, int tasksPerLine, bool doALF )
{
  this->cs = &cs;

  const int heightInCtus = cs.pcv->heightInCtus;
  CHECKD( !ctuStates.empty() && std::any_of( ctuStates.begin(), ctuStates.end(), []( CtuState& s ) { return s != DONE; } ), "some CTUs of previous pic not done" );
  ctuStates = std::vector<CtuState>( heightInCtus * tasksPerLine );
  for( auto& ctu: ctuStates )
  {
    ctu.store( ctuStartState );
  }


  this->perLineMiHist = std::vector<MotionHist>( heightInCtus );
  this->dmvrTriggers  = std::vector<Barrier>   ( heightInCtus );

  this->doALF        = doALF;
  this->alfPrepared.lock();
}

DecLibRecon::DecLibRecon()
{
#if ENABLE_SIMD_OPT_BUFFER && defined( TARGET_SIMD_X86 )
  g_pelBufOP.initPelBufOpsX86();
#endif
#if ENABLE_SIMD_TCOEFF_OPS && defined( TARGET_SIMD_X86 )
  g_tCoeffOps.initTCoeffOpsX86();
#endif

}

void DecLibRecon::create( ThreadPool* threadPool, unsigned instanceId )
{
  // run constructor again to ensure all variables, especially in DecLibParser have been reset
  this->~DecLibRecon();
  new( this ) DecLibRecon;


#if TRACE_ENABLE_ITT
  if( itt_domain_decInst.size() < instanceId + 1 )
  {
    std::string name( "DecLibRecon " + std::to_string( instanceId ) );
    itt_domain_decInst.push_back( __itt_domain_create( name.c_str() ) );
    itt_domain_decInst.back()->flags = 1;

    CHECK( itt_domain_decInst.back() != itt_domain_decInst[instanceId], "current decLibRecon ITT-Domain is not the last in vector. Instances created in the wrong order?" );
  }
  m_itt_decInst = itt_domain_decInst[instanceId];
#endif

  m_decodeThreadPool = threadPool;
  m_numDecThreads    = std::max( 1, threadPool ? threadPool->numThreads() : 1 );

  m_cIntraPred = new IntraPrediction[m_numDecThreads];
  m_cInterPred = new InterPrediction[m_numDecThreads];
  m_cTrQuant   = new TrQuant        [m_numDecThreads];
  m_cCuDecoder = new DecCu          [m_numDecThreads];
  m_cReshaper  = new Reshape        [m_numDecThreads];
}

void DecLibRecon::destroy()
{
  m_decodeThreadPool = nullptr;

  delete[] m_cIntraPred; m_cIntraPred = nullptr;
  delete[] m_cInterPred; m_cInterPred = nullptr;
  delete[] m_cTrQuant;   m_cTrQuant   = nullptr;
  delete[] m_cCuDecoder; m_cCuDecoder = nullptr;
  delete[] m_cReshaper;  m_cReshaper  = nullptr;
}

void DecLibRecon::borderExtPic( Picture* pic )
{
  pic->borderExtStarted = true;

  const bool wrapAround = pic->cs->sps->getUseWrapAround();
  if( wrapAround )
  {
    // copy reconstruction buffer to wrapAround buffer. All other border-extension tasks depend on this task.
    static auto copyTask = []( int, Picture* picture ) {
      ITT_TASKSTART( itt_domain_dec, itt_handle_ext );
      picture->getRecoBuf( true ).copyFrom( picture->getRecoBuf() );
      ITT_TASKEND( itt_domain_dec, itt_handle_ext );
      return true;
    };
    pic->m_copyWrapBufDone.lock();
    m_decodeThreadPool->addBarrierTask<Picture>( copyTask,
                                                 pic,
                                                 nullptr,
                                                 &pic->m_copyWrapBufDone,
                                                 { &pic->done} );
  }

  // start actual border extension tasks
  {
    static auto task = []( int, Picture* picture ) {
      ITT_TASKSTART( itt_domain_dec, itt_handle_ext );
      picture->extendPicBorder( true, false, false, false );
      ITT_TASKEND( itt_domain_dec, itt_handle_ext );
      return true;
    };
    m_decodeThreadPool->addBarrierTask<Picture>( task,
                                                 pic,
                                                 &pic->m_borderExtTaskCounter,
                                                 nullptr,
                                                 { wrapAround ? &pic->m_copyWrapBufDone : &pic->done} );
  }

  {
    static auto task = []( int, Picture* picture ) {
      ITT_TASKSTART( itt_domain_dec, itt_handle_ext );
      picture->extendPicBorder( false, true, false, false );
      ITT_TASKEND( itt_domain_dec, itt_handle_ext );
      return true;
    };
    m_decodeThreadPool->addBarrierTask<Picture>( task,
                                                 pic,
                                                 &pic->m_borderExtTaskCounter,
                                                 nullptr,
                                                 { wrapAround ? &pic->m_copyWrapBufDone : &pic->done} );
  }

  {
    static auto task = []( int, Picture* picture ) {
      ITT_TASKSTART( itt_domain_dec, itt_handle_ext );
      picture->extendPicBorder( false, false, true, false, CH_L );
      ITT_TASKEND( itt_domain_dec, itt_handle_ext );
      return true;
    };
    m_decodeThreadPool->addBarrierTask<Picture>( task,
                                                 pic,
                                                 &pic->m_borderExtTaskCounter,
                                                 nullptr,
                                                 { wrapAround ? &pic->m_copyWrapBufDone : &pic->done} );
  }
  {
    static auto task = []( int, Picture* picture ) {
      ITT_TASKSTART( itt_domain_dec, itt_handle_ext );
      picture->extendPicBorder( false, false, false, true, CH_L );
      ITT_TASKEND( itt_domain_dec, itt_handle_ext );
      return true;
    };
    m_decodeThreadPool->addBarrierTask<Picture>( task,
                                                 pic,
                                                 &pic->m_borderExtTaskCounter,
                                                 nullptr,
                                                 { wrapAround ? &pic->m_copyWrapBufDone : &pic->done} );
  }

  {
    static auto task = []( int, Picture* picture ) {
      ITT_TASKSTART( itt_domain_dec, itt_handle_ext );
      picture->extendPicBorder( false, false, true, false, CH_C );
      ITT_TASKEND( itt_domain_dec, itt_handle_ext );
      return true;
    };
    m_decodeThreadPool->addBarrierTask<Picture>( task,
                                                 pic,
                                                 &pic->m_borderExtTaskCounter,
                                                 nullptr,
                                                 { wrapAround ? &pic->m_copyWrapBufDone : &pic->done} );
  }
  {
    static auto task = []( int, Picture* picture ) {
      ITT_TASKSTART( itt_domain_dec, itt_handle_ext );
      picture->extendPicBorder( false, false, false, true, CH_C );
      ITT_TASKEND( itt_domain_dec, itt_handle_ext );
      return true;
    };
    m_decodeThreadPool->addBarrierTask<Picture>( task,
                                                 pic,
                                                 &pic->m_borderExtTaskCounter,
                                                 nullptr,
                                                 { wrapAround ? &pic->m_copyWrapBufDone : &pic->done} );
  }
}

void DecLibRecon::createSubPicRefBufs( Picture* pic )
{
  pic->subPicExtStarted = true;

  const PPS* pps       = pic->cs->pps.get();
  const SPS* sps       = pic->cs->sps.get();
  const int  numSubPic = pps->getNumSubPics();

  pic->m_subPicRefBufs.resize( numSubPic );
  for( int i = 0; i < numSubPic; ++i )
  {
    const SubPic& currSubPic = pps->getSubPic( i );
    const Area    subPicArea( currSubPic.getSubPicLeft(),
                              currSubPic.getSubPicTop(),
                              currSubPic.getSubPicWidthInLumaSample(),
                              currSubPic.getSubPicHeightInLumaSample() );

    pic->m_subPicRefBufs[i].create( pic->chromaFormat, Size( subPicArea ), sps->getMaxCUWidth(), pic->margin, MEMORY_ALIGN_DEF_SIZE );

    static auto task = []( int, SubPicExtTask* t ) {
      t->subPicBuf->copyFrom( t->picture->getRecoBuf().subBuf( t->subPicArea ) );
      t->picture->extendPicBorderBuf( *t->subPicBuf );
      return true;
    };
    m_subPicExtTasks.emplace_back( SubPicExtTask{ pic, &pic->m_subPicRefBufs[i], subPicArea } );
    m_decodeThreadPool->addBarrierTask<SubPicExtTask>( task, &m_subPicExtTasks.back(), &pic->m_borderExtTaskCounter, nullptr, { &pic->done } );
  }
}

void DecLibRecon::decompressPicture( Picture* pcPic )
{
  CodingStructure& cs = *pcPic->cs;

  pcPic->inProgress = true;

#ifdef TRACE_ENABLE_ITT
  // mark start of frame
    pcPic->m_itt_decLibInst = m_itt_decInst;
  __itt_frame_begin_v3( pcPic->m_itt_decLibInst, nullptr );
#endif

  // Initialise the various objects for the new set of settings
  const SPS * sps = cs.sps.get();
  const PPS * pps = cs.pps.get();

  for( int i = 0; i < m_numDecThreads; i++ )
  {
    if( sps->getUseReshaper() )
    {
      m_cReshaper[i].createDec( sps->getBitDepth( CHANNEL_TYPE_LUMA ) );
      m_cReshaper[i].initSlice( pcPic->slices[0]->getNalUnitLayerId(), *pcPic->slices[0]->getPicHeader(), *pcPic->slices[0]->getVPS() );
    }

    m_cIntraPred[i].init( sps->getChromaFormatIdc(), sps->getBitDepth( CHANNEL_TYPE_LUMA ) );
    m_cInterPred[i].init( &m_cRdCost, sps->getChromaFormatIdc(), sps->getMaxCUHeight() );

    // Recursive structure
    m_cTrQuant[i]  .init( pcPic );
    m_cCuDecoder[i].init( &m_cIntraPred[i], &m_cInterPred[i], &m_cReshaper[i], &m_cTrQuant[i] );
  }

  const uint32_t  log2SaoOffsetScaleLuma   = (uint32_t) std::max(0, sps->getBitDepth(CHANNEL_TYPE_LUMA  ) - MAX_SAO_TRUNCATED_BITDEPTH);
  const uint32_t  log2SaoOffsetScaleChroma = (uint32_t) std::max(0, sps->getBitDepth(CHANNEL_TYPE_CHROMA) - MAX_SAO_TRUNCATED_BITDEPTH);
  const int maxDepth = getLog2(sps->getMaxCUWidth()) - pps->pcv->minCUWidthLog2;
  m_cSAO.create( pps->getPicWidthInLumaSamples(),
                 pps->getPicHeightInLumaSamples(),
                 sps->getChromaFormatIdc(),
                 sps->getMaxCUWidth(),
                 sps->getMaxCUHeight(),
                 maxDepth,
                 log2SaoOffsetScaleLuma,
                 log2SaoOffsetScaleChroma
               );

  if( sps->getUseALF() )
  {
    m_cALF.create( cs.picHeader, sps, pps, m_numDecThreads );
  }

  const int widthInCtus = cs.pcv->widthInCtus;
  const int heightInCtus = cs.pcv->heightInCtus;

  if( sps->getIBCFlag() )
  {
    cs.initVIbcBuf( heightInCtus, sps->getChromaFormatIdc(), sps->getMaxCUHeight() );
  }
  pcPic->startProcessingTimer();

  if( m_decodeThreadPool->numThreads() > 0 )
  {
    ITT_TASKSTART( itt_domain_dec, itt_handle_schedTasks );
  }

  picBarriers.clear();
#if ALLOW_MIDER_LF_DURING_PICEXT
  CBarrierVec  picExtBarriers;
#else
  CBarrierVec &picExtBarriers = picBarriers;
#endif

  const int numSubPic = cs.pps->getNumSubPics();
  if( numSubPic > 1 )
  {
    m_subPicExtTasks.clear();
    m_subPicExtTasks.reserve( pcPic->slices.size() * MAX_NUM_REF_PICS * numSubPic );
  }

  std::vector<Picture*> borderExtRefPics;
  for( const Slice* slice : pcPic->slices )
  {
    if( slice->isIntra() )
    {
      continue;
    }

    for( int iDir = REF_PIC_LIST_0; iDir < NUM_REF_PIC_LIST_01; ++iDir )
    {
      for( int iRefIdx = 0; iRefIdx < slice->getNumRefIdx( ( RefPicList ) iDir ); iRefIdx++ )
      {
        Picture* refPic = slice->getNoConstRefPic( ( RefPicList ) iDir, iRefIdx );

        if( std::find( borderExtRefPics.cbegin(), borderExtRefPics.cend(), refPic ) == borderExtRefPics.cend() )
        {
          borderExtRefPics.push_back( refPic );
        }
      }
    }
  }

  for( Picture* refPic : borderExtRefPics )
  {
    if( !refPic->borderExtStarted )
    {
      // TODO: (GH) Can we bypass this border extension, when all subpics (>1) are treated as pics?
      borderExtPic( refPic );
    }

    if( !refPic->subPicExtStarted && numSubPic > 1 && refPic->m_subPicRefBufs.size() != numSubPic )
    {
      CHECK( !refPic->m_subPicRefBufs.empty(), "Wrong number of subpics already present in reference picture" );
      CHECK( cs.sps->getUseWrapAround(), "Wraparound + subpics not implemented" );

      createSubPicRefBufs( refPic );
    }

    if( refPic->m_borderExtTaskCounter.isBlocked() &&
        std::find( picExtBarriers.cbegin(), picExtBarriers.cend(), refPic->m_borderExtTaskCounter.donePtr() ) == picExtBarriers.cend() )
    {
      picExtBarriers.push_back( refPic->m_borderExtTaskCounter.donePtr() );
    }

    if( refPic->m_dmvrTaskCounter.isBlocked() &&
        std::find( picBarriers.cbegin(), picBarriers.cend(), refPic->m_dmvrTaskCounter.donePtr() ) == picBarriers.cend() )
    {
      picBarriers.push_back( refPic->m_dmvrTaskCounter.donePtr() );
    }
  }

  if( m_decodeThreadPool->numThreads() == 0 && (
       std::any_of( picExtBarriers.cbegin(), picExtBarriers.cend(), []( const Barrier* b ) { return b->isBlocked(); } ) ||
       std::any_of( picBarriers   .cbegin(), picBarriers   .cend(), []( const Barrier* b ) { return b->isBlocked(); } ) ) )
  {
    m_decodeThreadPool->processTasksOnMainThread();
  }

  const bool isIntra = std::all_of( pcPic->slices.begin(), pcPic->slices.end(), []( const Slice* pcSlice ) { return pcSlice->isIntra(); } );

  const int numColPerTask = std::max( std::min( widthInCtus, ( widthInCtus / std::max( m_numDecThreads * ( isIntra ? 2 : 1 ), 1 ) ) + ( isIntra ? 0 : 1 ) ), 1 );
  const int numTasksPerLine = widthInCtus / numColPerTask + !!( widthInCtus % numColPerTask );

#if ALLOW_MIDER_LF_DURING_PICEXT
  pcPic->refPicExtDepBarriers = std::move( picExtBarriers );
#endif
#if !RECO_WHILE_PARSE
  picBarriers.push_back( &cs.slice->parseDone );

#endif
  const TaskType ctuStartState = MIDER;
  const bool     doALF         = cs.sps->getUseALF() && !AdaptiveLoopFilter::getAlfSkipPic( cs );
  commonTaskParam.reset( cs, ctuStartState, numTasksPerLine, doALF );

  tasksDMVR = std::vector<LineTaskParam>( heightInCtus, LineTaskParam{ commonTaskParam, -1 } );
  tasksCtu  = std::vector<CtuTaskParam >( heightInCtus * numTasksPerLine, CtuTaskParam{ commonTaskParam, -1, -1, {} } );

  pcPic->done.lock();

#if 0
  // schedule in raster scan order
  for( int line = 0; line < heightInCtus; ++line )
  {
    for( int col = 0; col < widthInCtus;  ++col )
    {
#else
  // schedule in zig-zag scan order
  for( int i = 0; i < numTasksPerLine + heightInCtus; ++i )
  {
    int line = 0;
    for( int col = i; col >= 0; --col, ++line )
    {
#endif
      if( line < heightInCtus && col < numTasksPerLine )
      {
        CBarrierVec ctuBarriesrs = picBarriers;

#if RECO_WHILE_PARSE
        const int ctuStart = col * numColPerTask;
        const int ctuEnd   = std::min( ctuStart + numColPerTask, widthInCtus );
        for( int ctu = ctuStart; ctu < ctuEnd; ctu++ )
        {
          ctuBarriesrs.push_back( &pcPic->ctuParsedBarrier[line * widthInCtus + ctu] );
        }
#endif
        CtuTaskParam* param = &tasksCtu[line * numTasksPerLine + col];
        param->line         = line;
        param->col          = col;
        param->numColPerTask   = numColPerTask;
        param->numTasksPerLine = numTasksPerLine;

        m_decodeThreadPool->addBarrierTask<CtuTaskParam>( ctuTask<false>,
                                                          param,
                                                          &pcPic->m_ctuTaskCounter,
                                                          nullptr,
                                                          std::move( ctuBarriesrs ),
                                                          ctuTask<true> );
      }
    }
  }

  if( commonTaskParam.doALF )
  {
    AdaptiveLoopFilter::preparePic( cs );
    commonTaskParam.alfPrepared.unlock();
  }

  {
    static auto doneTask = []( int, Picture* picture )
    {
      CodingStructure& cs = *picture->cs;
      if( cs.sps->getUseALF() && !AdaptiveLoopFilter::getAlfSkipPic( cs ) )
      {
        AdaptiveLoopFilter::swapBufs( cs );
      }

      picture->reconstructed = true;
#ifdef TRACE_ENABLE_ITT
      // mark end of frame
      __itt_frame_end_v3( picture->m_itt_decLibInst, nullptr );
#endif
      picture->stopProcessingTimer();

      return true;
    };
    m_decodeThreadPool->addBarrierTask<Picture>( doneTask, pcPic, nullptr, &pcPic->done, { pcPic->m_ctuTaskCounter.donePtr() } );
  }

  if( pcPic->referenced )
  {
    static auto task = []( int tid, LineTaskParam* param )
    {
      ITT_TASKSTART( itt_domain_dec, itt_handle_dmvr );
      auto& cs = *param->common.cs;
      for( int col = 0; col < cs.pcv->widthInCtus; col++ )
      {
        param->common.decLib.m_cCuDecoder[tid].TaskDeriveDMVRMotionInfo( cs, getCtuArea( cs, col, param->line, true ) );
      }
      ITT_TASKEND( itt_domain_dec, itt_handle_dmvr );
      return true;
    };

    for( int taskLineDMVR = 0; taskLineDMVR < heightInCtus; taskLineDMVR++ )
    {
      auto param  = &tasksDMVR[taskLineDMVR];
      param->line = taskLineDMVR;
      m_decodeThreadPool->addBarrierTask<LineTaskParam>( task,
                                                         param,
                                                         &pcPic->m_dmvrTaskCounter,
                                                         nullptr,
                                                         { &commonTaskParam.dmvrTriggers[taskLineDMVR], &pcPic->parseDone } );
    }

    {
      // dummy task to propagate exceptions from the ctu-decoding tasks to the dmvrTaskCounter
      static auto dummyTask = []( int, void* ) { return true; };
      m_decodeThreadPool->addBarrierTask<void>( dummyTask, nullptr, &pcPic->m_dmvrTaskCounter, nullptr, { pcPic->m_ctuTaskCounter.donePtr() } );
    }
  }

  if( m_decodeThreadPool->numThreads() == 0 )
  {
  }
  else
  {
    ITT_TASKEND( itt_domain_dec, itt_handle_schedTasks );
  }

  m_currDecompPic = pcPic;
}

Picture* DecLibRecon::waitForPrevDecompressedPic()
{
  if( !m_currDecompPic )
    return nullptr;

  ITT_TASKSTART( itt_domain_dec, itt_handle_waitTasks );
  if( m_decodeThreadPool->numThreads() == 0 )
  {
    m_decodeThreadPool->processTasksOnMainThread();
    CHECK( m_currDecompPic->m_dmvrTaskCounter.isBlocked() || m_currDecompPic->done.isBlocked(), "can't make progress. some dependecy has not been finished" );
  }
  m_currDecompPic->m_dmvrTaskCounter.wait();
  m_currDecompPic->done.wait();
  ITT_TASKEND( itt_domain_dec, itt_handle_waitTasks );

  m_currDecompPic->inProgress = false;
  return std::exchange( m_currDecompPic, nullptr );
}

template<bool onlyCheckReadyState>
bool DecLibRecon::ctuTask( int tid, CtuTaskParam* param )
{
  const int       col          = param->col;
  const int       line         = param->line;

  auto&           cs           = *param->common.cs;
  auto&           decLib       = param->common.decLib;
  const int       widthInCtus  = param->numTasksPerLine;
  const int       heightInCtus = cs.pcv->heightInCtus;

  CtuState&       thisCtuState =  param->common.ctuStates[line * widthInCtus + col];
  const CtuState* thisLine     = &param->common.ctuStates[line * widthInCtus];
  const CtuState* lineAbove    = thisLine - widthInCtus;
  const CtuState* lineBelow    = thisLine + widthInCtus;

  const int       ctuStart     = col * param->numColPerTask;
  const int       ctuEnd       = std::min<int>( ctuStart + param->numColPerTask, cs.pcv->widthInCtus );

  try
  {
    if( cs.picture->m_ctuTaskCounter.hasException() )
    {
      std::rethrow_exception( cs.picture->m_ctuTaskCounter.getException() );
    }

    switch( thisCtuState.load() )
    {
      // all case statements fall through to continue with next task, unless they return false due to unsatisfied preconditions

    case MIDER:
    {
      if( col > 0 && thisLine[col - 1] <= MIDER )
        return false;
      if( line > 0 && lineAbove[std::min( col + 1, widthInCtus - 1 )] <= MIDER )
        return false;
      if( onlyCheckReadyState )
        return true;

      ITT_TASKSTART( itt_domain_dec, itt_handle_mider );

      for( int ctu = ctuStart; ctu < ctuEnd; ctu++ )
      {
        CtuData& ctuData = cs.getCtuData( ctu, line );
        GCC_WARNING_DISABLE_class_memaccess
        memset( ctuData.motion, 0, sizeof( CtuData::motion ) );
        GCC_WARNING_RESET

        if( !ctuData.slice->isIntra() || cs.sps->getIBCFlag() )
        {
          const UnitArea ctuArea = getCtuArea( cs, ctu, line, true );
          decLib.m_cCuDecoder[tid].TaskDeriveCtuMotionInfo( cs, ctuArea, param->common.perLineMiHist[line] );
        }
      }
      thisCtuState = ( TaskType )( MIDER + 1 );

      ITT_TASKEND( itt_domain_dec, itt_handle_mider );
    }

    case LF_INIT:
    {
      if( onlyCheckReadyState )
        return true;

      ITT_TASKSTART( itt_domain_dec, itt_handle_lfcl );

      for( int ctu = ctuStart; ctu < ctuEnd; ctu++ )
      {
        CtuData& ctuData = cs.getCtuData( ctu, line );
        memset( ctuData.lfParam, 0, sizeof( CtuData::lfParam ) );

        const UnitArea  ctuArea  = getCtuArea( cs, ctu, line, true );
        decLib.m_cLoopFilter.calcFilterStrengthsCTU( cs, ctuArea );
      }

      thisCtuState = ( TaskType )( LF_INIT + 1 );

      ITT_TASKEND( itt_domain_dec, itt_handle_lfcl );
    }

    case INTER:
    {
      if( std::all_of( cs.picture->slices.begin(), cs.picture->slices.end(), []( const Slice* pcSlice ) { return pcSlice->isIntra(); } ) )
      {
        // not really necessary, but only for optimizing the wave-fronts
        if( col > 1 && thisLine[col - 2] <= INTER )
          return false;
        if( line > 0 && lineAbove[col] <= INTER )
          return false;
      }

      if( std::any_of( cs.picture->refPicExtDepBarriers.cbegin(), cs.picture->refPicExtDepBarriers.cend(), []( const Barrier* b ) { return b->isBlocked(); } ) )
      {
        return false;
      }

      if( onlyCheckReadyState )
        return true;

      ITT_TASKSTART( itt_domain_dec, itt_handle_inter );

      for( int ctu = ctuStart; ctu < ctuEnd; ctu++ )
      {
        const CtuData& ctuData = cs.getCtuData( ctu, line );
        const UnitArea ctuArea = getCtuArea( cs, ctu, line, true );

        decLib.m_cCuDecoder[tid].TaskTrafoCtu( cs, ctuArea );

        if( !ctuData.slice->isIntra() )
        {
          decLib.m_cCuDecoder[tid].TaskInterCtu( cs, ctuArea );
        }
      }

      thisCtuState = ( TaskType )( INTER + 1 );

      ITT_TASKEND( itt_domain_dec, itt_handle_inter );
    }

    case INTRA:
    {
      if( col > 0 && thisLine[col - 1] <= INTRA )
        return false;
      if( line > 0 && lineAbove[std::min( col + 1, widthInCtus - 1 )] <= INTRA )
        return false;
      if( onlyCheckReadyState )
        return true;

      ITT_TASKSTART( itt_domain_dec, itt_handle_intra );

      for( int ctu = ctuStart; ctu < ctuEnd; ctu++ )
      {
        const UnitArea  ctuArea = getCtuArea( cs, ctu, line, true );
        decLib.m_cCuDecoder[tid].TaskCriticalIntraKernel( cs, ctuArea );
      }

      thisCtuState = ( TaskType )( INTRA + 1 );

      ITT_TASKEND( itt_domain_dec, itt_handle_intra );
    }

    case RSP:
    {
      // RIRZIIIII
      // IIIIIXXXX
      //
      // - Z can be reshaped when it is no more an intra prediction source for X in the next line


      if     ( line + 1 < heightInCtus && col + 1 < widthInCtus && lineBelow[col + 1] < RSP )
        return false;
      else if( line + 1 < heightInCtus &&                          lineBelow[col]     < RSP )
        return false;
      else if(                            col + 1 < widthInCtus && thisLine [col + 1] < RSP ) // need this for the last line
        return false;

      if( onlyCheckReadyState )
        return true;

      ITT_TASKSTART( itt_domain_dec, itt_handle_rsp );

      for( int ctu = ctuStart; ctu < ctuEnd; ctu++ )
      {
        decLib.m_cReshaper[tid].rspCtu( cs, ctu, line, 0 );
      }

      ITT_TASKEND( itt_domain_dec, itt_handle_rsp );

      thisCtuState = ( TaskType )( RSP + 1 );
    }

    case LF_V:
    {
      if( col > 0 && thisLine[col - 1] < LF_V )
        return false;
      if( onlyCheckReadyState )
        return true;

      ITT_TASKSTART( itt_domain_dec, itt_handle_lfl );

      for( int ctu = ctuStart; ctu < ctuEnd; ctu++ )
      {
        decLib.m_cLoopFilter.loopFilterCTU( cs, MAX_NUM_CHANNEL_TYPE, ctu, line, 0, EDGE_VER );
      }

      thisCtuState = ( TaskType )( LF_V + 1 );

      ITT_TASKEND( itt_domain_dec, itt_handle_lfl );
    }

    case LF_H:
    {
      if( line > 0 && lineAbove[col] < LF_H )
        return false;

      if( line > 0 && col + 1 < widthInCtus && lineAbove[col + 1] < LF_H )
        return false;

      if( col + 1 < widthInCtus && thisLine[col + 1] < LF_H )
        return false;

      if( onlyCheckReadyState )
        return true;

      ITT_TASKSTART( itt_domain_dec, itt_handle_lfl );

      for( int ctu = ctuStart; ctu < ctuEnd; ctu++ )
      {
        decLib.m_cLoopFilter.loopFilterCTU( cs, MAX_NUM_CHANNEL_TYPE, ctu, line, 0, EDGE_HOR );
      }

      thisCtuState = ( TaskType )( LF_H + 1 );

      ITT_TASKEND( itt_domain_dec, itt_handle_lfl );
    }

    case PRESAO:
    {
      // only last CTU processes full line
      if( col == widthInCtus - 1 )
      {
        if( line > 0 && lineAbove[col] <= PRESAO )
          return false;

        for( int c = 0; c < widthInCtus; ++c )
        {
          if( thisLine[c] < PRESAO )
            return false;

          if( line + 1 < heightInCtus && lineBelow[c] < PRESAO )
            return false;
        }
        if( onlyCheckReadyState )
          return true;

        ITT_TASKSTART( itt_domain_dec, itt_handle_presao );

        if( cs.sps->getUseSAO() )
        {
          decLib.m_cSAO.SAOPrepareCTULine( cs, getLineArea( cs, line, true ) );
        }
        param->common.dmvrTriggers[line].unlock();

        ITT_TASKEND( itt_domain_dec, itt_handle_presao );
      }
      else if( thisLine[widthInCtus - 1] <= PRESAO )   // wait for last CTU to finish PRESAO
      {
        return false;
      }
      if( onlyCheckReadyState )
        return true;

      thisCtuState = ( TaskType )( PRESAO + 1 );
    }

    case SAO:
    {
      if( onlyCheckReadyState )
        return true;

      // only last CTU processes full line
      if( cs.sps->getUseSAO() )
      {
        ITT_TASKSTART( itt_domain_dec, itt_handle_sao );

        for( int ctu = ctuStart; ctu < ctuEnd; ctu++ )
        {
          const UnitArea  ctuArea = getCtuArea( cs, ctu, line, true );
          decLib.m_cSAO.SAOProcessCTU( cs, ctuArea );
        }

        ITT_TASKEND( itt_domain_dec, itt_handle_sao );
      }
      if( param->common.doALF )
      {
        ITT_TASKSTART( itt_domain_dec, itt_handle_alf );

        for( int ctu = ctuStart; ctu < ctuEnd; ctu++ )
        {
          AdaptiveLoopFilter::prepareCTU( cs, ctu, line );
        }

        ITT_TASKEND( itt_domain_dec, itt_handle_alf );
      }

      thisCtuState = ( TaskType )( SAO + 1 );
    }

    case ALF:
    {
      if( param->common.doALF )
      {
        const bool a = line > 0;
        const bool b = line + 1 < heightInCtus;
        const bool c = col > 0;
        const bool d = col + 1 < widthInCtus;

        if( param->common.alfPrepared.isBlocked() )
          return false;

        if( a )
        {
          if( c && lineAbove[col - 1] < ALF ) return false;
          if(      lineAbove[col    ] < ALF ) return false;
          if( d && lineAbove[col + 1] < ALF ) return false;
        }

        if( b )
        {
          if( c && lineBelow[col - 1] < ALF ) return false;
          if(      lineBelow[col    ] < ALF ) return false;
          if( d && lineBelow[col + 1] < ALF ) return false;
        }

        if( c && thisLine[col - 1] < ALF ) return false;
        if( d && thisLine[col + 1] < ALF ) return false;

        if( onlyCheckReadyState )
          return true;

        ITT_TASKSTART( itt_domain_dec, itt_handle_alf );
        for( int ctu = ctuStart; ctu < ctuEnd; ctu++ )
        {
          decLib.m_cALF.processCTU( cs, ctu, line, tid );
        }
        ITT_TASKEND( itt_domain_dec, itt_handle_alf );
      }
      else if( onlyCheckReadyState )
        return true;

      thisCtuState = ( TaskType )( ALF + 1 );
    }

    default:
      CHECKD( thisCtuState != DONE, "Wrong CTU state" );
    }   // end switch
  }
  catch( ... )
  {
    for( auto& t: param->common.dmvrTriggers )
    {
      t.setException( std::current_exception() );
    }
    std::rethrow_exception( std::current_exception() );
  }

  return true;
}

}
