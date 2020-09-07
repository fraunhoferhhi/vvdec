/* -----------------------------------------------------------------------------
Software Copyright License for the Fraunhofer Software Library VVdec

(c) Copyright (2018-2020) Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. 

1.    INTRODUCTION

The Fraunhofer Software Library VVdec (“Fraunhofer Versatile Video Decoding Library”) is software that implements (parts of) the Versatile Video Coding Standard - ITU-T H.266 | MPEG-I - Part 3 (ISO/IEC 23090-3) and related technology. 
The standard contains Fraunhofer patents as well as third-party patents. Patent licenses from third party standard patent right holders may be required for using the Fraunhofer Versatile Video Decoding Library. It is in your responsibility to obtain those if necessary. 

The Fraunhofer Versatile Video Decoding Library which mean any source code provided by Fraunhofer are made available under this software copyright license. 
It is based on the official ITU/ISO/IEC VVC Test Model (VTM) reference software whose copyright holders are indicated in the copyright notices of its source files. The VVC Test Model (VTM) reference software is licensed under the 3-Clause BSD License and therefore not subject of this software copyright license.

2.    COPYRIGHT LICENSE

Internal use of the Fraunhofer Versatile Video Decoding Library, in source and binary forms, with or without modification, is permitted without payment of copyright license fees for non-commercial purposes of evaluation, testing and academic research. 

No right or license, express or implied, is granted to any part of the Fraunhofer Versatile Video Decoding Library except and solely to the extent as expressly set forth herein. Any commercial use or exploitation of the Fraunhofer Versatile Video Decoding Library and/or any modifications thereto under this license are prohibited.

For any other use of the Fraunhofer Versatile Video Decoding Library than permitted by this software copyright license You need another license from Fraunhofer. In such case please contact Fraunhofer under the CONTACT INFORMATION below.

3.    LIMITED PATENT LICENSE

As mentioned under 1. Fraunhofer patents are implemented by the Fraunhofer Versatile Video Decoding Library. If You use the Fraunhofer Versatile Video Decoding Library in Germany, the use of those Fraunhofer patents for purposes of testing, evaluating and research and development is permitted within the statutory limitations of German patent law. However, if You use the Fraunhofer Versatile Video Decoding Library in a country where the use for research and development purposes is not permitted without a license, you must obtain an appropriate license from Fraunhofer. It is Your responsibility to check the legal requirements for any use of applicable patents.    

Fraunhofer provides no warranty of patent non-infringement with respect to the Fraunhofer Versatile Video Decoding Library.


4.    DISCLAIMER

The Fraunhofer Versatile Video Decoding Library is provided by Fraunhofer "AS IS" and WITHOUT ANY EXPRESS OR IMPLIED WARRANTIES, including but not limited to the implied warranties fitness for a particular purpose. IN NO EVENT SHALL FRAUNHOFER BE LIABLE for any direct, indirect, incidental, special, exemplary, or consequential damages, including but not limited to procurement of substitute goods or services; loss of use, data, or profits, or business interruption, however caused and on any theory of liability, whether in contract, strict liability, or tort (including negligence), arising in any way out of the use of the Fraunhofer Versatile Video Decoding Library, even if advised of the possibility of such damage.

5.    CONTACT INFORMATION

Fraunhofer Heinrich Hertz Institute
Attention: Video Coding & Analytics Department
Einsteinufer 37
10587 Berlin, Germany
www.hhi.fraunhofer.de/vvc
vvc@hhi.fraunhofer.de
------------------------------------------------------------------------------------------- */

/** \file     Rom.h
    \brief    global variables & functions (header)
*/

#ifndef __ROM__
#define __ROM__

#include "CommonDef.h"
#include "Common.h"

#include <stdio.h>
#include <iostream>


//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Initialize / destroy functions
// ====================================================================================================================

void         initROM();
void         destroyROM();

// ====================================================================================================================
// Data structure related table & variable
// ====================================================================================================================

// flexible conversion from relative to absolute index
extern const uint32_t   g_log2SbbSize   [MAX_CU_DEPTH+1][MAX_CU_DEPTH+1][2];
extern       uint32_t*  g_scanOrder     [SCAN_NUMBER_OF_GROUP_TYPES][SCAN_NUMBER_OF_TYPES][MAX_CU_SIZE / 2 + 1][MAX_CU_SIZE / 2 + 1];

extern       uint32_t*  g_coefTopLeftDiagScan8x8   [ MAX_CU_SIZE / 2 + 1 ];
extern       uint8_t*   g_coefTopLeftDiagScan8x8pos[ MAX_CU_SIZE / 2 + 1 ];

extern const int g_QuantScales   [2/*0=4^n blocks, 1=2*4^n blocks*/][SCALING_LIST_REM_NUM];          // Q(QP%6)
extern const int g_InvQuantScales[2/*0=4^n blocks, 1=2*4^n blocks*/][SCALING_LIST_REM_NUM];          // IQ(QP%6)

static const int g_numTransformMatrixSizes = 6;
static const int g_transformMatrixShift    = 6;


// ====================================================================================================================
// Scanning order & context mapping table
// ====================================================================================================================

extern const uint32_t   ctxIndMap4x4[4*4];

extern const uint32_t   g_uiGroupIdx[ MAX_TU_SIZE ];
extern const uint32_t   g_uiMinInGroup[ LAST_SIGNIFICANT_GROUPS ];
extern const uint32_t   g_auiGoRiceParsCoeff     [ 32 ];
inline uint32_t g_auiGoRicePosCoeff0(int st, uint32_t ricePar)
{
  return (st < 2 ? 1 : 2) << ricePar;
}
extern const uint32_t   g_auiGoRiceRange[ MAX_GR_ORDER_RESIDUAL ];                  //!< maximum value coded with Rice codes

// ====================================================================================================================
// Intra prediction table
// ====================================================================================================================

extern const uint8_t  g_aucIntraModeNumFast_UseMPM_2D[7 - MIN_CU_LOG2 + 1][7 - MIN_CU_LOG2 + 1];
extern const uint8_t  g_aucIntraModeNumFast_UseMPM   [MAX_CU_DEPTH];
extern const uint8_t  g_aucIntraModeNumFast_NotUseMPM[MAX_CU_DEPTH];

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

extern const     int8_t   g_lfnst8x8[ 4 ][ 2 ][ 16 ][ 48 ];
extern const     int8_t   g_lfnst4x4[ 4 ][ 2 ][ 16 ][ 16 ];

extern const     uint8_t  g_lfnstLut[ NUM_INTRA_MODE + NUM_EXT_LUMA_MODE - 1 ];

// ====================================================================================================================
// Misc.
// ====================================================================================================================
#if !( ENABLE_SIMD_LOG2 && defined( TARGET_SIMD_X86 ) )
extern int8_t          g_aucLog2                       [MAX_CU_SIZE + 1];
extern int8_t          g_aucNextLog2        [MAX_CU_SIZE + 1];
extern int8_t          g_aucPrevLog2        [MAX_CU_SIZE + 1];
#endif
extern const int8_t    i2Log2Tab[257];

extern const int       g_ictModes[2][4];

class SizeIndexInfoLog2
{
public:
  constexpr inline SizeType numAllWidths()            const { return 8; }
  constexpr inline SizeType numAllHeights()           const { return 8; }
  constexpr inline SizeType numWidths()               const { return 6; }
  constexpr inline SizeType numHeights()              const { return 6; }
  constexpr inline SizeType sizeFrom( SizeType idx )  const { return (1 << idx); }
            inline SizeType idxFrom( SizeType size )  const { return getLog2(size); }
  constexpr inline bool     isCuSize( SizeType size ) const { return size >= ( 1 << MIN_CU_LOG2 ); }
};

extern const SizeIndexInfoLog2 g_sizeIdxInfo;

inline bool isNonLog2BlockSize( const Size& size )
{
  return ( ( 1 << g_sizeIdxInfo.idxFrom(size.width) ) != size.width ) || ( ( 1 << g_sizeIdxInfo.idxFrom(size.height) ) != size.height );
}

extern const UnitScale g_miScaling; // scaling object for motion scaling

/*! Sophisticated Trace-logging */
#if ENABLE_TRACING
#include "dtrace.h"
extern CDTrace* g_trace_ctx;
#endif

#if ENABLE_TIME_PROFILING || ENABLE_TIME_PROFILING_EXTENDED
#include "TimeProfiler.h"
#if ENABLE_TIME_PROFILING
extern TimeProfiler *g_timeProfiler;
#else
extern TimeProfiler2D *g_timeProfiler;
#endif
#endif

const char* nalUnitTypeToString(NalUnitType type);

extern const char *MatrixType   [SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM];
extern const char *MatrixType_DC[SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM];

extern const int g_quantTSDefault4x4   [4*4];
extern const int g_quantIntraDefault8x8[8*8];
extern const int g_quantInterDefault8x8[8*8];

extern const uint32_t g_vvcScalingListSize [SCALING_LIST_SIZE_NUM];
extern const uint32_t g_vvcScalingListSizeX[SCALING_LIST_SIZE_NUM];
#if JVET_R0166_SCALING_LISTS_CHROMA_444
extern const uint32_t g_scalingListId[SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM];
#endif

extern MsgLevel g_verbosity;

extern const int g_aiNonLMPosThrs[];

extern const int8_t g_BcwLog2WeightBase;
extern const int8_t g_BcwWeightBase;
extern const int8_t g_BcwWeights[BCW_NUM];
extern const int8_t g_BcwSearchOrder[BCW_NUM];
extern       int8_t g_BcwCodingOrder[BCW_NUM];
extern       int8_t g_BcwParsingOrder[BCW_NUM];

class CodingStructure;
int8_t getBcwWeight(uint8_t bcwIdx, uint8_t uhRefFrmList);
void resetBcwCodingOrder(bool bRunDecoding, const CodingStructure &cs);
uint32_t deriveWeightIdxBits(uint8_t bcwIdx);

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
extern int16_t** g_GeoParams;
extern int16_t*  g_globalGeoWeights   [GEO_NUM_PRESTORED_MASK];
extern int16_t*  g_globalGeoEncSADmask[GEO_NUM_PRESTORED_MASK];
extern int16_t   g_weightOffset       [GEO_NUM_PARTITION_MODE][GEO_NUM_CU_SIZE][GEO_NUM_CU_SIZE][2];
extern int8_t    g_angle2mask         [GEO_NUM_ANGLES];
extern int8_t    g_Dis[GEO_NUM_ANGLES];
extern int8_t    g_angle2mirror[GEO_NUM_ANGLES];
#endif  //__TCOMROM__

