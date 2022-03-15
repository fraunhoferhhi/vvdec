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

/** \file     WeightPrediction.h
    \brief    weighting prediction class (header)
*/

#pragma once

#include "CommonDef.h"
#include "Slice.h"
#include "Unit.h"
#include "Buffer.h"

namespace vvdec
{

// forward declarations
struct WPScalingParam;

// ====================================================================================================================
// Class definition
// ====================================================================================================================
/// weighting prediction class
class WeightPrediction
{
public:
  WeightPrediction();

  void  getWpScaling(           const Slice                *slice,
                                const int                  &iRefIdx0,
                                const int                  &iRefIdx1,
                                      WPScalingParam       *wp0,
                                      WPScalingParam       *wp1 );

  void addWeightBi(             const PelUnitBuf           &pcYuvSrc0,
                                const PelUnitBuf           &pcYuvSrc1,
                                const ClpRngs              &clpRngs,
                                const WPScalingParam *const wp0,
                                const WPScalingParam *const wp1,
                                      PelUnitBuf           &rpcYuvDst,
                                const bool                  bRoundLuma = true
                                );

  void  addWeightUni(           const PelUnitBuf           &pcYuvSrc0,
                                const ClpRngs              &clpRngs,
                                const WPScalingParam *const wp0,
                                      PelUnitBuf           &rpcYuvDst
                                );

  void  xWeightedPredictionUni( const PredictionUnit       &pu,
                                const PelUnitBuf           &pcYuvSrc,
                                const RefPicList           &eRefPicList,
                                      PelUnitBuf           &pcYuvPred,
                                const int                   iRefIdx=-1
                                );

  void  xWeightedPredictionBi(  const PredictionUnit       &pu,
                                const PelUnitBuf           &pcYuvSrc0,
                                const PelUnitBuf           &pcYuvSrc1,
                                      PelUnitBuf           &pcYuvDst
                                );
};

}
