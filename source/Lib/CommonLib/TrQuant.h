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

/** \file     TrQuant.h
    \brief    transform and quantization class (header)
*/

#pragma once

#include "CommonDef.h"
#include "Unit.h"
#include "Quant.h"

namespace vvdec
{
using namespace x86_simd;

typedef void InvTrans(const TCoeff*, TCoeff*, int, int, int, int, bool, const TCoeff, const TCoeff);

// ====================================================================================================================
// Class definition
// ====================================================================================================================


/// transform and quantization class
class TrQuant : Quant
{
public:
  TrQuant( class InterPrediction* ip, const TrQuant* other = nullptr );

  // initialize class
  void init      ( const Picture *pic );
  void getTrTypes( const TransformUnit &tu, const ComponentID compID, int &trTypeHor, int &trTypeVer );

  void invLfnstNxN( int* src, int* dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize );

  uint32_t getLFNSTIntraMode( int wideAngPredMode );
  bool     getTransposeFlag ( uint32_t intraMode  );

protected:

  void xInvLfnst      (       TransformUnit &tu, const ComponentID &compID );

public:

  void invTransformNxN(       TransformUnit &tu, const ComponentID &compID, PelBuf &pResi, const QpParam &cQPs );
  void invTransformICT( const TransformUnit &tu, PelBuf &resCb, PelBuf &resCr );



protected:
  bool     m_bEnc;
  bool     m_useTransformSkipFast;

  bool     m_rectTUs;

  bool     m_scalingListEnabledFlag;

  TCoeff  *m_blk;
  TCoeff  *m_tmp;
  TCoeff  *m_dqnt;

private:
  TCoeff   m_tempInMatrix [ 48 ];
  TCoeff   m_tempOutMatrix[ 48 ];
  static const int maxAbsIctMode = 3;
  void   (*m_invICTMem[1+2*maxAbsIctMode])(PelBuf&,PelBuf&);
  void  (**m_invICT)(PelBuf&,PelBuf&);
  void ( *m_invLfnstNxN )( int* src, int* dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize );

  // dequantization
  void xDeQuant( const TransformUnit &tu,
                       CoeffBuf      &dstCoeff,
                 const ComponentID   &compID,
                 const QpParam       &cQP      );

  // inverse transform
  void xIT     ( const TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pCoeff, PelBuf &pResidual );

  // inverse skipping transform
  void xITransformSkip(
                 const CCoeffBuf     &plCoef,
                       PelBuf        &pResidual,
                 const TransformUnit &tu,
                 const ComponentID   &component);


#ifdef TARGET_SIMD_X86
  template<X86_VEXT vext>
  void _initTrQuantX86();
  void initTrQuantX86();
#endif
};// END CLASS DEFINITION TrQuant

}   // namespace vvdec
