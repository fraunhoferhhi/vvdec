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

/** \file     DecCu.h
    \brief    CU decoder class (header)
*/

#ifndef __DECCU__
#define __DECCU__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "CABACReader.h"

#include "CommonLib/TrQuant.h"
#include "CommonLib/InterPrediction.h"
#include "CommonLib/IntraPrediction.h"
#include "CommonLib/LoopFilter.h"
#include "CommonLib/Unit.h"
#include "CommonLib/Reshape.h"

//! \ingroup DecoderLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// CU decoder class
class DecCu
{
  // Task definition
public:
  void TaskDeriveCtuMotionInfo ( CodingStructure& cs, const UnitArea& ctuArea,  MotionHist& hist );
  void TaskInterCtu            ( CodingStructure& cs, const UnitArea& ctuArea );
  void TaskTrafoCtu            ( CodingStructure& cs, const UnitArea& ctuArea );
  void TaskCriticalIntraKernel ( CodingStructure& cs, const UnitArea& ctuArea );
  void TaskDeriveDMVRMotionInfo( CodingStructure& cs, const UnitArea& ctuArea );

public:
  DecCu();
  virtual ~DecCu();

  /// initialize access channels
  void  init              ( IntraPrediction* pcIntra, InterPrediction* pcInter, Reshape* pcReshape, TrQuant* pcTrQuant );

  void  create();
  void  destroy();

protected:
  /// reconstruct Ctu information
  void predAndReco        ( CodingUnit&      cu, bool doCiipIntra = false );
  void finishLMCSAndReco  ( CodingUnit&      cu );
  void reconstructResi    ( CodingUnit&      cu );

  void xIntraRecACT       ( CodingUnit&      cu );
  void xDeriveCUMV        ( CodingUnit&      cu, MotionHist& hist );

private:
  Reshape*          m_pcReshape;
  IntraPrediction*  m_pcIntraPred;
  InterPrediction*  m_pcInterPred;
  TrQuant*          m_pcTrQuant;

  MergeCtx          m_geoMrgCtx;
};

//! \}

#endif

