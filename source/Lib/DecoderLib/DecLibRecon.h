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

/** \file     DecLibRecon.h
    \brief    decoder class (header)
*/

#pragma once

#include "CommonLib/CommonDef.h"
#include "CommonLib/Picture.h"
#include "CommonLib/RdCost.h"
#include "CommonLib/Reshape.h"
#include "CommonLib/LoopFilter.h"
#include "CommonLib/AdaptiveLoopFilter.h"
#include "CommonLib/SampleAdaptiveOffset.h"

#include "Utilities/ThreadPool.h"

namespace vvdec
{

class DecLibRecon;
class IntraPrediction;
class InterPrediction;
class TrQuant;
class DecCu;

//! \ingroup DecoderLib
//! \{
// ====================================================================================================================
// Class definition
// ====================================================================================================================


enum TaskType
{
  /*TRAFO=-1,*/ MIDER, LF_INIT, INTER, INTRA, RSP, LF_V, LF_H, PRESAO, SAO, ALF, DONE, DMVR
};
using CtuState = std::atomic<TaskType>;

struct CommonTaskParam
{
  DecLibRecon&             decLib;
  CodingStructure*         cs           = nullptr;
  std::vector<CtuState>    ctuStates;
  std::vector<MotionHist>  perLineMiHist;
  std::vector<Barrier>     finishMotionTriggers;

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
  int              line;
  int              col;
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
  DecLibRecon( const DecLibRecon& )  = delete;
  DecLibRecon( const DecLibRecon&& ) = delete;

  void create( ThreadPool* threadPool, unsigned instanceId );
  void destroy();

  void     decompressPicture( Picture* pcPic );
  Picture* waitForPrevDecompressedPic();
  Picture* getCurrPic() const { return m_currDecompPic; }
  void     swapBufs( CodingStructure& cs );


private:
  void borderExtPic ( Picture* pic );
  void createSubPicRefBufs( Picture* pic );

  template<bool checkReadyState=false>
  static bool ctuTask( int tid, CtuTaskParam* param );
};

}
