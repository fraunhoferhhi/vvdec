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

/** \file     Rom.h
    \brief    global variables & functions (header)
*/

#pragma once

#include "CommonDef.h"
#include "Common.h"

#if ENABLE_TIME_PROFILING || ENABLE_TIME_PROFILING_EXTENDED
#  include "TimeProfiler.h"
#endif

#include <atomic>

namespace vvdec
{


//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Initialize / destroy functions
// ====================================================================================================================

void         initROM();
void         destroyROM();

extern std::atomic_bool romInitialized;

// ====================================================================================================================
// Data structure related table & variable
// ====================================================================================================================

// flexible conversion from relative to absolute index
extern const uint16_t   g_log2SbbSize   [MAX_LOG2_TU_SIZE_PLUS_ONE][MAX_LOG2_TU_SIZE_PLUS_ONE][2];
extern const uint16_t   g_scanOrderBuf  [32258];
extern const uint16_t*  g_scanOrder     [SCAN_NUMBER_OF_GROUP_TYPES][MAX_LOG2_TU_SIZE_PLUS_ONE][MAX_LOG2_TU_SIZE_PLUS_ONE];

extern const uint16_t   g_coefTopLeftDiagScan8x8[MAX_LOG2_TU_SIZE_PLUS_ONE][64];

extern const int g_InvQuantScales[2/*0=4^n blocks, 1=2*4^n blocks*/][SCALING_LIST_REM_NUM];          // IQ(QP%6)

static const int g_numTransformMatrixSizes = 6;
static const int g_transformMatrixShift    = 6;


// ====================================================================================================================
// Scanning order & context mapping table
// ====================================================================================================================

extern const uint32_t   ctxIndMap4x4[4*4];

extern const uint32_t   g_uiGroupIdx[ MAX_TU_SIZE_FOR_PROFILE ];
extern const uint32_t   g_uiMinInGroup[ LAST_SIGNIFICANT_GROUPS ];
extern const uint32_t   g_auiGoRiceParsCoeff     [ 32 ];
static inline uint32_t g_auiGoRicePosCoeff0(int st, uint32_t ricePar)
{
  return (st < 2 ? 1 : 2) << ricePar;
}
extern const uint32_t   g_auiGoRiceRange[ MAX_GR_ORDER_RESIDUAL ];                  //!< maximum value coded with Rice codes

// ====================================================================================================================
// Intra prediction table
// ====================================================================================================================

extern const uint8_t  g_chroma422IntraAngleMappingTable[NUM_INTRA_MODE];

// ====================================================================================================================
// Mode-Dependent DST Matrices
// ====================================================================================================================


extern const TMatrixCoeff g_trCoreDCT2P2 [  2][  2];
extern const TMatrixCoeff g_trCoreDCT2P4 [  4][  4];
extern const TMatrixCoeff g_trCoreDCT2P8 [  8][  8];
extern const TMatrixCoeff g_trCoreDCT2P16[ 16][ 16];
extern const TMatrixCoeff g_trCoreDCT2P32[ 32][ 32];
extern const TMatrixCoeff g_trCoreDCT2P64[ 64][ 64];

extern const TMatrixCoeff g_trCoreDCT8P4 [  4][  4];
extern const TMatrixCoeff g_trCoreDCT8P8 [  8][  8];
extern const TMatrixCoeff g_trCoreDCT8P16[ 16][ 16];
extern const TMatrixCoeff g_trCoreDCT8P32[ 32][ 32];

extern const TMatrixCoeff g_trCoreDST7P4 [  4][  4];
extern const TMatrixCoeff g_trCoreDST7P8 [  8][  8];
extern const TMatrixCoeff g_trCoreDST7P16[ 16][ 16];
extern const TMatrixCoeff g_trCoreDST7P32[ 32][ 32];

extern const     int8_t   g_lfnst8x8[ 4 ][ 2 ][ 48 ][ 16 ];
extern const     int8_t   g_lfnst4x4[ 4 ][ 2 ][ 16 ][ 16 ];

extern const     uint8_t  g_lfnstLut[ NUM_INTRA_MODE + NUM_EXT_LUMA_MODE - 1 ];

// ====================================================================================================================
// Misc.
// ====================================================================================================================
#if !( ENABLE_SIMD_LOG2 && defined( TARGET_SIMD_X86 ) )
extern int8_t          g_aucLog2            [MAX_CU_SIZE + 1];
extern int8_t          g_aucNextLog2        [MAX_CU_SIZE + 1];
extern int8_t          g_aucPrevLog2        [MAX_CU_SIZE + 1];
#endif

extern const int       g_ictModes[2][4];

class SizeIndexInfoLog2
{
public:
  constexpr static inline SizeType numAllWidths()           { return MAX_LOG2_TU_SIZE_PLUS_ONE; }
  constexpr static inline SizeType numAllHeights()          { return MAX_LOG2_TU_SIZE_PLUS_ONE; }
  constexpr static inline SizeType sizeFrom( SizeType idx ) { return (1 << idx); }
            static inline SizeType idxFrom( SizeType size ) { return getLog2(size); }
};

extern const SizeIndexInfoLog2 g_sizeIdxInfo;

extern const UnitScale g_miScaling; // scaling object for motion scaling
extern const UnitScale g_colMiScaling;


#if ENABLE_TIME_PROFILING || ENABLE_TIME_PROFILING_EXTENDED
#  if ENABLE_TIME_PROFILING
extern TimeProfiler* g_timeProfiler;
#  else
extern TimeProfiler2D* g_timeProfiler;
#  endif
#endif

const char* nalUnitTypeToString(NalUnitType type);

extern const int g_quantTSDefault4x4   [4*4];
extern const int g_quantIntraDefault8x8[8*8];
extern const int g_quantInterDefault8x8[8*8];

extern const uint32_t g_vvcScalingListSizeX[SCALING_LIST_SIZE_NUM];
extern const uint32_t g_scalingListId[SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM];

extern MsgLevel g_verbosity;

extern const int8_t g_BcwLog2WeightBase;
extern const int8_t g_BcwWeightBase;
extern const int8_t g_BcwWeights[BCW_NUM];
extern const int8_t g_BcwParsingOrder[BCW_NUM];
extern const int8_t g_BcwInternFwd[BCW_NUM];
extern const int8_t g_BcwInternBcw[BCW_NUM];

int8_t getBcwWeight(uint8_t bcwIdx, uint8_t uhRefFrmList);

constexpr uint8_t g_tbMax[257] = { 0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
                                   4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
                                   5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
                                   6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
                                   6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7,
                                   7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
                                   7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
                                   7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
                                   7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
                                   7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 8 };

//! \}



extern const int g_alfNumCoeff[ALF_NUM_OF_FILTER_TYPES];

const int g_IBCBufferSize = 256 * 128;

void initGeoTemplate();

extern int16_t         g_weightOffset       [GEO_NUM_PARTITION_MODE][GEO_NUM_CU_SIZE][GEO_NUM_CU_SIZE][2];
extern int16_t         g_GeoParams          [GEO_NUM_PARTITION_MODE][2];
extern int16_t         g_globalGeoWeights   [GEO_NUM_PRESTORED_MASK][GEO_WEIGHT_MASK_SIZE * GEO_WEIGHT_MASK_SIZE];
extern const int8_t    g_angle2mask         [GEO_NUM_ANGLES];
extern const int8_t    g_Dis                [GEO_NUM_ANGLES];
extern const int8_t    g_angle2mirror       [GEO_NUM_ANGLES];

}   // namespace vvdec
