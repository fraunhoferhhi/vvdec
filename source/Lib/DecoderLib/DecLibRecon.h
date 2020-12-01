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

Copyright (c) 2018-2020, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. 
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

#ifndef DECLIB_RECON_H
#define DECLIB_RECON_H

#include "CommonLib/CommonDef.h"
#include "CommonLib/Picture.h"
#include "CommonLib/RdCost.h"
#include "CommonLib/Reshape.h"
#include "CommonLib/LoopFilter.h"
#include "CommonLib/AdaptiveLoopFilter.h"
#include "CommonLib/SampleAdaptiveOffset.h"

#include "Utilities/NoMallocThreadPool.h"

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
  std::vector<Barrier>     dmvrTriggers;

  bool                     doALF        = true;
  Barrier                  alfPrepared;

  explicit CommonTaskParam( DecLibRecon* dec ) : decLib( *dec ) {}
  void reset( CodingStructure& cs, TaskType ctuStartState, int tasksPerLine, bool doALF );
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

/// decoder class
class DecLibRecon
{
private:
  // functional classes
  IntraPrediction*     m_cIntraPred = nullptr;
  InterPrediction*     m_cInterPred = nullptr;
  TrQuant*             m_cTrQuant   = nullptr;
  DecCu*               m_cCuDecoder = nullptr;
  RdCost               m_cRdCost;
  Reshape*             m_cReshaper  = nullptr;   ///< reshaper class
  LoopFilter           m_cLoopFilter;
  SampleAdaptiveOffset m_cSAO;
  AdaptiveLoopFilter   m_cALF;

  int                  m_numDecThreads = 0;
  NoMallocThreadPool*  m_decodeThreadPool;

  Picture*             m_currDecompPic = nullptr;
#if TRACE_ENABLE_ITT
  __itt_domain*        m_itt_decInst = nullptr;
#endif

  CommonTaskParam            commonTaskParam{ this };
  std::vector<LineTaskParam> tasksDMVR;
  std::vector<CtuTaskParam>  tasksCtu;
  CBarrierVec                picBarriers;

public:
  DecLibRecon();
  ~DecLibRecon() = default;
  DecLibRecon( const DecLibRecon& )  = delete;
  DecLibRecon( const DecLibRecon&& ) = delete;

  void create( NoMallocThreadPool* threadPool, unsigned instanceId );
  void destroy();

  void     decompressPicture( Picture* pcPic );
  Picture* waitForPrevDecompressedPic();
  Picture* getCurrPic() const { return m_currDecompPic; }


private:
  void borderExtPic ( Picture* pic );

  template<bool checkReadyState=false>
  static bool ctuTask( int tid, CtuTaskParam* param );
};

//! \}

#endif   // DECLIB_RECON_H
