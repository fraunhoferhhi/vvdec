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

/** \file     Quant.h
    \brief    base quantization class (header)
*/

#pragma once

#include "CommonDef.h"
#include "Unit.h"

namespace vvdec
{

struct Picture;
class ScalingList;

using namespace x86_simd;

// ====================================================================================================================
// Constants
// ====================================================================================================================

#define QP_BITS                 15

// ====================================================================================================================
// Class definition
// ====================================================================================================================
struct TrQuantParams
{
  int     rightShift;
  int     qScale;
};

/// QP struct
class QpParam
{
public:
  short  Qps[2];
  int8_t pers[2];
  int8_t rems[2];

  QpParam(const TransformUnit& tu, const ComponentID &compID, const bool allowACTQpoffset = true);

  int Qp ( const bool ts ) const { return Qps [ts?1:0]; }
  int per( const bool ts ) const { return pers[ts?1:0]; }
  int rem( const bool ts ) const { return rems[ts?1:0]; }

}; // END STRUCT DEFINITION QpParam

/// transform and quantization class
class Quant
{
public:
  int* getDequantCoeff           ( uint32_t list, uint32_t sizeX, uint32_t sizeY ) { return m_dequantCoef[sizeX][sizeY][list]; };  //!< get DeQuant Coefficent

  void setUseScalingList         ( bool bUseScalingList ) { m_scalingListEnabledFlag = bUseScalingList; };
  bool getUseScalingList         ( bool isTransformSkip, const bool lfnstApplied, const bool disableScalingMatrixForLFNSTBlks, const bool disableSMforACT)
  {
    return( m_scalingListEnabledFlag && !isTransformSkip && (!lfnstApplied || !disableScalingMatrixForLFNSTBlks) && !disableSMforACT);
  }
  void setScalingListDec         ( const ScalingList &scalingList);
  bool processScalingListDec     ( const int *coeff, int *dequantcoeff, uint32_t height, uint32_t width, uint32_t ratio, int sizuNum, uint32_t dc);

  Quant( const Quant* other );
  ~Quant();

  // initialize class
  void init( const Picture *pic );
public:
  // de-quantization
  void dequant           ( const TransformUnit &tu, CoeffBuf &dstCoeff, const ComponentID &compID, const QpParam &cQP );

private:
  void xInitScalingList   ( const Quant* other );
  void xDestroyScalingList();
  bool xSetScalingListDec( const ScalingList &scalingList, uint32_t list, uint32_t size, uint32_t scalingListId );
  void xSetRecScalingListDec( const ScalingList &scalingList, uint32_t list, uint32_t sizew, uint32_t sizeh, uint32_t scalingListId );
  bool m_scalingListEnabledFlag;

  int *m_dequantCoef   [SCALING_LIST_SIZE_NUM][SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM]; ///< array of dequantization matrix coefficient 4x4
  int *m_dequantCoefBuf;
  bool m_ownDequantCoeff;


  void ( *DeQuant    ) (const int maxX,const int restX,const int maxY,const int scale,const TCoeffSig*const piQCoef,const size_t piQCfStride,TCoeff   *const piCoef,const int rightShift,const int inputMaximum,const TCoeff transformMaximum);
  void ( *DeQuantPCM ) (const int maxX,const int restX,const int maxY,const int scale,      TCoeff   *const piQCoef,const size_t piQCfStride,TCoeff   *const piCoef,const int rightShift,const int inputMaximum,const TCoeff transformMaximum);

#if ENABLE_SIMD_OPT_QUANT && defined( TARGET_SIMD_X86 )

  void initQuantX86();
  template <X86_VEXT vext>
  void _initQuantX86();
#endif
};// END CLASS DEFINITION Quant

}   // namespace vvdec
