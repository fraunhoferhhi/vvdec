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

/** \file     DecLibRecon.h
    \brief    decoder class (header)
*/

#pragma once

#include "CommonLib/CommonDef.h"
#include "CommonLib/AdaptiveLoopFilter.h"
#include "CommonLib/LoopFilter.h"
#include "CommonLib/InterPrediction.h"
#include "CommonLib/IntraPrediction.h"
#include "CommonLib/Picture.h"
#include "CommonLib/RdCost.h"
#include "CommonLib/Reshape.h"
#include "CommonLib/SampleAdaptiveOffset.h"
#include "CommonLib/TrQuant.h"

#include "DecCu.h"
#include "Utilities/ThreadPool.h"


namespace vvdec
{

class DecLibRecon;
class InterPrediction;
class TrQuant;

//! \ingroup DecoderLib
//! \{
// ====================================================================================================================
// Class definition
// ====================================================================================================================


enum TaskType
{
  /*TRAFO=-1,*/ MIDER, MIDER_cont, LF_INIT, INTER, INTRA, INTRA_cont, RSP, LF_V, LF_V_cont, LF_H, PRESAO, SAO, SAO_cont, ALF, DONE, DMVR
};
using CtuState = std::atomic<TaskType>;

struct CommonTaskParam
{
  DecLibRecon&             decLib;
  CodingStructure*         cs           = nullptr;
  std::vector<CtuState>    ctuStates;
  std::vector<MotionHist>  perLineMiHist;

  bool                     doALF        = true;

  explicit CommonTaskParam( DecLibRecon* dec ) : decLib( *dec ) {}
  void reset( CodingStructure& cs, TaskType ctuStartState, int tasksPerLine, bool doALF );
};

struct SubPicExtTask
{
  Picture*    picture   = nullptr;
  PelStorage* subPicBuf = nullptr;
  Area        subPicArea;
};

struct LineTaskParam
{
  CommonTaskParam& common;
  int              line;
};

struct CtuTaskParam
{
  CommonTaskParam& common;
  int              taskLine;
  int              taskCol;
  int              ctuStart;
  int              ctuEnd;
  int              numColPerTask;
  int              numTasksPerLine;
};

struct FinishPicTaskParam
{
  DecLibRecon* decLib;
  Picture*     pic;

  FinishPicTaskParam() : decLib( nullptr ), pic( nullptr ) {}
  FinishPicTaskParam( DecLibRecon* _dec, Picture* _pic ) : decLib( _dec ), pic( _pic ) {}
};

struct PerThreadResource
{
  IntraPrediction m_cIntraPred;
  InterPrediction m_cInterPred;
  TrQuant         m_cTrQuant;
  Reshape         m_cReshaper;
  DecCu           m_cCuDecoder;

  explicit PerThreadResource() : m_cTrQuant( &m_cInterPred ) {}
  explicit PerThreadResource( TrQuant& trQuant0 ) : m_cTrQuant( &m_cInterPred, &trQuant0 ) {}
};

/// decoder class
class DecLibRecon
{
private:
  PerThreadResource  **m_pcThreadResource;
  RdCost               m_cRdCost;
  LoopFilter           m_cLoopFilter;
  SampleAdaptiveOffset m_cSAO;
  AdaptiveLoopFilter   m_cALF;

  int                  m_numDecThreads = 0;
  ThreadPool*          m_decodeThreadPool;
  bool                 m_upscaleOutputEnabled = false;

  Picture*             m_currDecompPic = nullptr;
#if TRACE_ENABLE_ITT
  __itt_domain*        m_itt_decInst = nullptr;
#endif

  std::unique_ptr<Pel[], AlignedDeleter<Pel>> 
                       m_predBuf;
  ptrdiff_t            m_predBufSize     = 0;
  Mv*                  m_dmvrMvCache     = nullptr;
  size_t               m_dmvrMvCacheSize = 0;
  ptrdiff_t            m_num4x4Elements  = 0;
  LoopFilterParam*     m_loopFilterParam = nullptr;
  MotionInfo*          m_motionInfo      = nullptr;

  PelStorage           m_fltBuf;

  CommonTaskParam            commonTaskParam{ this };
  std::vector<SubPicExtTask> m_subPicExtTasks;
  std::vector<LineTaskParam> tasksFinishMotion;
  std::vector<CtuTaskParam>  tasksCtu;
  FinishPicTaskParam         taskFinishPic;
  CBarrierVec                picBarriers;

public:
  DecLibRecon();
  ~DecLibRecon() = default;
  CLASS_COPY_MOVE_DELETE( DecLibRecon )

  void create( ThreadPool* threadPool, unsigned instanceId, bool upscaleOutputEnabled );
  void destroy();

  void     decompressPicture( Picture* pcPic );
  Picture* waitForPrevDecompressedPic();
  void     cleanupOnException( std::exception_ptr exception );
  Picture* getCurrPic() const { return m_currDecompPic; }
  void     swapBufs( CodingStructure& cs );


private:
  void borderExtPic       ( Picture* pic, const Picture* currPic );
  void createSubPicRefBufs( Picture* pic, const Picture* currPic );

  template<bool checkReadyState=false>
  static bool ctuTask( int tid, CtuTaskParam* param );
};

}
