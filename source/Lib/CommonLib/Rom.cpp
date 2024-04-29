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

/** \file     Rom.cpp
    \brief    global variables & functions
*/

#include "Rom.h"
#include "UnitTools.h"

#if ENABLE_TRACING
#include "dtrace.h"

#include <string>
#endif

#include <memory.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iomanip>

#include "vvdec/vvdec.h"

namespace vvdec
{

// ====================================================================================================================
// Initialize / destroy functions
// ====================================================================================================================


#if ENABLE_TIME_PROFILING
TimeProfiler *g_timeProfiler = nullptr;
#elif ENABLE_TIME_PROFILING_EXTENDED
TimeProfiler2D *g_timeProfiler = nullptr;
#endif

//! \ingroup CommonLib
//! \{

std::atomic_bool romInitialized(0);

MsgLevel g_verbosity = VERBOSE;
void    *g_context = nullptr;
std::function<void( void*, int, const char*, va_list )> g_msgFnc = default_msgFnc;

const char* nalUnitTypeToString(NalUnitType type)
{
  GCC_EXTRA_WARNING_switch_enum
  switch (type)
  {
  case NAL_UNIT_CODED_SLICE_TRAIL:      return "TRAIL";
  case NAL_UNIT_CODED_SLICE_STSA:       return "STSA";
  case NAL_UNIT_CODED_SLICE_RADL:       return "RADL";
  case NAL_UNIT_CODED_SLICE_RASL:       return "RASL";
  case NAL_UNIT_CODED_SLICE_IDR_W_RADL: return "IDR_W_RADL";
  case NAL_UNIT_CODED_SLICE_IDR_N_LP:   return "IDR_N_LP";
  case NAL_UNIT_CODED_SLICE_CRA:        return "CRA";
  case NAL_UNIT_CODED_SLICE_GDR:        return "GDR";
  case NAL_UNIT_OPI:                    return "OPI";
  case NAL_UNIT_DCI:                    return "DCI";
  case NAL_UNIT_VPS:                    return "VPS";
  case NAL_UNIT_SPS:                    return "SPS";
  case NAL_UNIT_PPS:                    return "PPS";
  case NAL_UNIT_PREFIX_APS:             return "Prefix APS";
  case NAL_UNIT_SUFFIX_APS:             return "Suffix APS";
  case NAL_UNIT_PH:                     return "PH";
  case NAL_UNIT_ACCESS_UNIT_DELIMITER:  return "AUD";
  case NAL_UNIT_EOS:                    return "EOS";
  case NAL_UNIT_EOB:                    return "EOB";
  case NAL_UNIT_PREFIX_SEI:             return "Prefix SEI";
  case NAL_UNIT_SUFFIX_SEI:             return "Suffix SEI";
  case NAL_UNIT_FD:                     return "FD";
  case NAL_UNIT_RESERVED_VCL_4:
  case NAL_UNIT_RESERVED_VCL_5:
  case NAL_UNIT_RESERVED_VCL_6:
  case NAL_UNIT_RESERVED_IRAP_VCL_11:
  case NAL_UNIT_RESERVED_NVCL_26:
  case NAL_UNIT_RESERVED_NVCL_27:
  case NAL_UNIT_UNSPECIFIED_28:
  case NAL_UNIT_UNSPECIFIED_29:
  case NAL_UNIT_UNSPECIFIED_30:
  case NAL_UNIT_UNSPECIFIED_31:
  case NAL_UNIT_INVALID:
  default:                              return "UNK";
  }
  GCC_WARNING_RESET
}

class ScanGenerator
{
private:
  uint32_t m_line, m_column;
  const uint32_t m_blockWidth, m_blockHeight;
  const uint32_t m_stride;

public:
  ScanGenerator(uint32_t blockWidth, uint32_t blockHeight, uint32_t stride)
    : m_line(0), m_column(0), m_blockWidth(blockWidth), m_blockHeight(blockHeight), m_stride(stride)
  { }

  uint32_t GetCurrentX() const { return m_column; }
  uint32_t GetCurrentY() const { return m_line; }

  uint32_t GetNextIndex(uint32_t blockOffsetX, uint32_t blockOffsetY)
  {
    const uint32_t rtn = ((m_line + blockOffsetY) * m_stride) + m_column + blockOffsetX;

    //advance line and column to the next position
    //------------------------------------------------
    if ((m_column == m_blockWidth - 1) || (m_line == 0)) //if we reach the end of a rank, go diagonally down to the next one
    {
      m_line += m_column + 1;
      m_column = 0;

      if (m_line >= m_blockHeight) //if that takes us outside the block, adjust so that we are back on the bottom row
      {
        m_column += m_line - (m_blockHeight - 1);
        m_line = m_blockHeight - 1;
      }
    }
    else
    {
      m_column++;
      m_line--;
    }

      //------------------------------------------------

    return rtn;
  }
};
const int8_t g_BcwLog2WeightBase = 3;
const int8_t g_BcwWeightBase = (1 << g_BcwLog2WeightBase);
const int8_t g_BcwWeights[BCW_NUM] = { -2, 3, 4, 5, 10 };
const int8_t g_BcwParsingOrder[BCW_NUM] = { 2, 3, 1, 4, 0 };

const int8_t g_BcwInternFwd[BCW_NUM] = { 1, 2, 0, 3, 4 };
const int8_t g_BcwInternBcw[BCW_NUM] = { 2, 0, 1, 3, 4 };

int8_t getBcwWeight(uint8_t bcwIdx, uint8_t uhRefFrmList)
{
  // Weghts for the model: P0 + w * (P1 - P0) = (1-w) * P0 + w * P1
  // Retuning  1-w for P0 or w for P1
  return (uhRefFrmList == REF_PIC_LIST_0 ? g_BcwWeightBase - g_BcwWeights[bcwIdx] : g_BcwWeights[bcwIdx]);
}

const uint16_t g_log2SbbSize[MAX_LOG2_TU_SIZE_PLUS_ONE][MAX_LOG2_TU_SIZE_PLUS_ONE][2] =
//===== luma/chroma =====
{
  { { 0,0 },{ 0,1 },{ 0,2 },{ 0,3 },{ 0,4 },{ 0,4 },{ 0,4 } },
  { { 1,0 },{ 1,1 },{ 1,1 },{ 1,3 },{ 1,3 },{ 1,3 },{ 1,3 } },
  { { 2,0 },{ 1,1 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 } },
  { { 3,0 },{ 3,1 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 } },
  { { 4,0 },{ 3,1 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 } },
  { { 4,0 },{ 3,1 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 } },
  { { 4,0 },{ 3,1 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 } }
};

// initialize ROM variables
void initROM()
{
#if !( ENABLE_SIMD_LOG2 && defined( TARGET_SIMD_X86 ) )
  int c;

#endif
#if !( ENABLE_SIMD_LOG2 && defined( TARGET_SIMD_X86 ) )
  // g_aucConvertToBit[ x ]: log2(x/4), if x=4 -> 0, x=8 -> 1, x=16 -> 2, ...
  // g_aucLog2[ x ]: log2(x), if x=1 -> 0, x=2 -> 1, x=4 -> 2, x=8 -> 3, x=16 -> 4, ...
  ::memset(g_aucLog2, 0, sizeof(g_aucLog2));
  c = 0;
  for( int i = 0, n = 0; i <= MAX_CU_SIZE; i++ )
  {
    g_aucNextLog2[i] = i <= 1 ? 0 : c + 1;

    if( i == ( 1 << n ) )
    {
      c = n;
      n++;
    }

    g_aucPrevLog2[i] = c;
    g_aucLog2    [i] = c;
  }

#endif

  // This is the hack for having global variables in a shared library. Having global
  // variables in a shared library is a very bad idea in general. The clean solution
  // would be:
  //  A: Have a struct that contains all the global variables and pass a const reference
  //     to them into each class/function that needs them.
  //  B: For really const values just precalculate them all and put them intot a header
  //     to include everywhere where needed.
  bool expected = false;

  if( !romInitialized.compare_exchange_strong( expected, true ) )
  {
    return;
  }

  //const SizeIndexInfoLog2 &sizeInfo = g_sizeIdxInfo;

  // initialize scan orders
  //for (uint32_t blockHeightIdx = 0; blockHeightIdx < sizeInfo.numAllHeights(); blockHeightIdx++)
  //{
  //  for (uint32_t blockWidthIdx = 0; blockWidthIdx < sizeInfo.numAllWidths(); blockWidthIdx++)
  //  {
  //    const uint32_t blockWidth  = sizeInfo.sizeFrom(blockWidthIdx);
  //    const uint32_t blockHeight = sizeInfo.sizeFrom(blockHeightIdx);
  //    const uint32_t totalValues = blockWidth * blockHeight;
  //
  //    //--------------------------------------------------------------------------------------------------
  //
  //    //non-grouped scan orders
  //
  //    {
  //      ScanGenerator fullBlockScan( blockWidth, blockHeight, blockWidth );
  //      for( uint32_t scanPosition = 0; scanPosition < totalValues; scanPosition++ )
  //      {
  //        const int rasterPos = fullBlockScan.GetNextIndex( 0, 0 );
  //        g_scanOrder[SCAN_UNGROUPED][blockWidthIdx][blockHeightIdx][scanPosition] = rasterPos;
  //      }
  //    }
  //
  //    //--------------------------------------------------------------------------------------------------
  //
  //    //grouped scan orders
  //    const uint32_t* log2Sbb        = g_log2SbbSize[ getLog2(blockWidth) ][ getLog2(blockHeight) ];
  //    const uint32_t  log2CGWidth    = log2Sbb[0];
  //    const uint32_t  log2CGHeight   = log2Sbb[1];
  //
  //    const uint32_t  groupWidth     = 1 << log2CGWidth;
  //    const uint32_t  groupHeight    = 1 << log2CGHeight;
  //    const uint32_t  widthInGroups  = std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, blockWidth)  >> log2CGWidth;
  //    const uint32_t  heightInGroups = std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, blockHeight) >> log2CGHeight;
  //
  //    const uint32_t  groupSize      = groupWidth    * groupHeight;
  //    const uint32_t  totalGroups    = widthInGroups * heightInGroups;
  //
  //    if ( blockWidth > JVET_C0024_ZERO_OUT_TH || blockHeight > JVET_C0024_ZERO_OUT_TH )
  //    {
  //      for (uint32_t i = 0; i < totalValues; i++)
  //      {
  //        g_scanOrder[SCAN_GROUPED_4x4][blockWidthIdx][blockHeightIdx]   [i] = totalValues - 1;
  //      }
  //    }
  //
  //    ScanGenerator fullBlockScan(widthInGroups, heightInGroups, groupWidth);
  //
  //    for (uint32_t groupIndex = 0; groupIndex < totalGroups; groupIndex++)
  //    {
  //      const uint32_t groupPositionY  = fullBlockScan.GetCurrentY();
  //      const uint32_t groupPositionX  = fullBlockScan.GetCurrentX();
  //      const uint32_t groupOffsetX    = groupPositionX * groupWidth;
  //      const uint32_t groupOffsetY    = groupPositionY * groupHeight;
  //      const uint32_t groupOffsetScan = groupIndex     * groupSize;
  //
  //      ScanGenerator groupScan(groupWidth, groupHeight, blockWidth);
  //
  //      for (uint32_t scanPosition = 0; scanPosition < groupSize; scanPosition++)
  //      {
  //        const int rasterPos = groupScan.GetNextIndex( groupOffsetX, groupOffsetY );
  //        g_scanOrder[SCAN_GROUPED_4x4][blockWidthIdx][blockHeightIdx][groupOffsetScan + scanPosition] = rasterPos;
  //      }
  //
  //      fullBlockScan.GetNextIndex(0, 0);
  //    }
  //
  //    //--------------------------------------------------------------------------------------------------
  //  }
  //}

  //// initialize CoefTopLeftDiagScan8x8 for LFNST
  //for( uint32_t blockWidthIdx = 0; blockWidthIdx < sizeInfo.numAllWidths(); blockWidthIdx++ )
  //{
  //  const uint32_t blockWidth = sizeInfo.sizeFrom( blockWidthIdx );
  //
  //  const static uint8_t g_auiXYDiagScan8x8[ 64 ][ 2 ] =
  //  {
  //    { 0, 0 }, { 0, 1 }, { 1, 0 }, { 0, 2 }, { 1, 1 }, { 2, 0 }, { 0, 3 }, { 1, 2 },
  //    { 2, 1 }, { 3, 0 }, { 1, 3 }, { 2, 2 }, { 3, 1 }, { 2, 3 }, { 3, 2 }, { 3, 3 },
  //    { 0, 4 }, { 0, 5 }, { 1, 4 }, { 0, 6 }, { 1, 5 }, { 2, 4 }, { 0, 7 }, { 1, 6 },
  //    { 2, 5 }, { 3, 4 }, { 1, 7 }, { 2, 6 }, { 3, 5 }, { 2, 7 }, { 3, 6 }, { 3, 7 },
  //    { 4, 0 }, { 4, 1 }, { 5, 0 }, { 4, 2 }, { 5, 1 }, { 6, 0 }, { 4, 3 }, { 5, 2 },
  //    { 6, 1 }, { 7, 0 }, { 5, 3 }, { 6, 2 }, { 7, 1 }, { 6, 3 }, { 7, 2 }, { 7, 3 },
  //    { 4, 4 }, { 4, 5 }, { 5, 4 }, { 4, 6 }, { 5, 5 }, { 6, 4 }, { 4, 7 }, { 5, 6 },
  //    { 6, 5 }, { 7, 4 }, { 5, 7 }, { 6, 6 }, { 7, 5 }, { 6, 7 }, { 7, 6 }, { 7, 7 }
  //  };
  //  for( int i = 0; i < 64; i++ )
  //  {
  //    g_coefTopLeftDiagScan8x8[ blockWidthIdx ][ i ] = g_auiXYDiagScan8x8[ i ][ 0 ] + g_auiXYDiagScan8x8[ i ][ 1 ] * blockWidth;
  //    uint8_t val = g_auiXYDiagScan8x8[ i ][ 0 ];
  //    val <<= 4;
  //    val += g_auiXYDiagScan8x8[ i ][ 1 ];
  //  }
  //}
  //
  //for( uint32_t blockWidthIdx = 0; blockWidthIdx < sizeInfo.numAllWidths(); blockWidthIdx++ )
  //{
  //  printf( "  {" );
  //  for( int i = 0; i < 64; i++ )
  //  {
  //    printf( "%d, ", g_coefTopLeftDiagScan8x8[blockWidthIdx][i] );
  //  }
  //  printf( "},\n" );
  //}

  //printf( "    " );
  //for( int i = 0; i < 32258; i++ )
  //{
  //  printf( "%d, ", g_scanOrderBuf[i] );
  //  if( ( i + 1 ) % 50 == 0 ) printf( "\n    " );
  //}

  initGeoTemplate();
}

void destroyROM()
{
  //dont deinitialize ROM, since it has no allocations, can be reused as is
  //romInitialized--;
  //if (romInitialized > 0)
  //{
  //  return;
  //}
}

// ====================================================================================================================
// Data structure related table & variable
// ====================================================================================================================

const int g_InvQuantScales[2][SCALING_LIST_REM_NUM] = // can be represented as a 9 element table
{
  { 40,45,51,57,64,72 },
  { 57,64,72,80,90,102 } // Note: last 3 values of second row == double of the first 3 values of the first row
};

//--------------------------------------------------------------------------------------------------
//structures

//EMT threshold


//--------------------------------------------------------------------------------------------------
//coefficients
//--------------------------------------------------------------------------------------------------
// ====================================================================================================================
// Intra prediction
// ====================================================================================================================

const uint8_t g_chroma422IntraAngleMappingTable[NUM_INTRA_MODE] =
//                                    *                                H                              *                                D      *   *   *   *       *   *   *                   *        V       *                   *   *   *      *   *   *   *
//0, 1,  2,  3,  4,  5,  6,  7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, DM
{ 0, 1, 61, 62, 63, 64, 65, 66, 2, 3,  5,  6,  8, 10, 12, 13, 14, 16, 18, 20, 22, 23, 24, 26, 28, 30, 31, 33, 34, 35, 36, 37, 38, 39, 40, 41, 41, 42, 43, 43, 44, 44, 45, 45, 46, 47, 48, 48, 49, 49, 50, 51, 51, 52, 52, 53, 54, 55, 55, 56, 56, 57, 57, 58, 59, 59, 60, DM_CHROMA_IDX };



// ====================================================================================================================
// Misc.
// ====================================================================================================================
const SizeIndexInfoLog2   g_sizeIdxInfo;
#if !( ENABLE_SIMD_LOG2 && defined( TARGET_SIMD_X86 ) )
int8_t                    g_aucLog2    [MAX_CU_SIZE + 1];
int8_t                    g_aucNextLog2[MAX_CU_SIZE + 1];
int8_t                    g_aucPrevLog2[MAX_CU_SIZE + 1];
#endif

const int                 g_ictModes[2][4] = { { 0, 3, 1, 2 }, { 0, -3, -1, -2 } };

const UnitScale g_miScaling   ( MIN_CU_LOG2,     MIN_CU_LOG2 );
const UnitScale g_colMiScaling( MIN_CU_LOG2 + 1, MIN_CU_LOG2 + 1 ); // collocated motion has an 8x8 scaling


// ====================================================================================================================
// Scanning order & context model mapping
// ====================================================================================================================

// scanning order table

const uint16_t* g_scanOrder[SCAN_NUMBER_OF_GROUP_TYPES][MAX_LOG2_TU_SIZE_PLUS_ONE][MAX_LOG2_TU_SIZE_PLUS_ONE] =
{
  {
    { g_scanOrderBuf + 0, g_scanOrderBuf + 1, g_scanOrderBuf + 3, g_scanOrderBuf + 7, g_scanOrderBuf + 15, g_scanOrderBuf + 31, g_scanOrderBuf + 63, },
    { g_scanOrderBuf + 127, g_scanOrderBuf + 129, g_scanOrderBuf + 133, g_scanOrderBuf + 141, g_scanOrderBuf + 157, g_scanOrderBuf + 189, g_scanOrderBuf + 253, },
    { g_scanOrderBuf + 381, g_scanOrderBuf + 385, g_scanOrderBuf + 393, g_scanOrderBuf + 409, g_scanOrderBuf + 441, g_scanOrderBuf + 505, g_scanOrderBuf + 633, },
    { g_scanOrderBuf + 889, g_scanOrderBuf + 897, g_scanOrderBuf + 913, g_scanOrderBuf + 945, g_scanOrderBuf + 1009, g_scanOrderBuf + 1137, g_scanOrderBuf + 1393, },
    { g_scanOrderBuf + 1905, g_scanOrderBuf + 1921, g_scanOrderBuf + 1953, g_scanOrderBuf + 2017, g_scanOrderBuf + 2145, g_scanOrderBuf + 2401, g_scanOrderBuf + 2913, },
    { g_scanOrderBuf + 3937, g_scanOrderBuf + 3969, g_scanOrderBuf + 4033, g_scanOrderBuf + 4161, g_scanOrderBuf + 4417, g_scanOrderBuf + 4929, g_scanOrderBuf + 5953, },
    { g_scanOrderBuf + 8001, g_scanOrderBuf + 8065, g_scanOrderBuf + 8193, g_scanOrderBuf + 8449, g_scanOrderBuf + 8961, g_scanOrderBuf + 9985, g_scanOrderBuf + 12033, },
  },
  {
    { g_scanOrderBuf + 16129, g_scanOrderBuf + 16130, g_scanOrderBuf + 16132, g_scanOrderBuf + 16136, g_scanOrderBuf + 16144, g_scanOrderBuf + 16160, g_scanOrderBuf + 16192, },
    { g_scanOrderBuf + 16256, g_scanOrderBuf + 16258, g_scanOrderBuf + 16262, g_scanOrderBuf + 16270, g_scanOrderBuf + 16286, g_scanOrderBuf + 16318, g_scanOrderBuf + 16382, },
    { g_scanOrderBuf + 16510, g_scanOrderBuf + 16514, g_scanOrderBuf + 16522, g_scanOrderBuf + 16538, g_scanOrderBuf + 16570, g_scanOrderBuf + 16634, g_scanOrderBuf + 16762, },
    { g_scanOrderBuf + 17018, g_scanOrderBuf + 17026, g_scanOrderBuf + 17042, g_scanOrderBuf + 17074, g_scanOrderBuf + 17138, g_scanOrderBuf + 17266, g_scanOrderBuf + 17522, },
    { g_scanOrderBuf + 18034, g_scanOrderBuf + 18050, g_scanOrderBuf + 18082, g_scanOrderBuf + 18146, g_scanOrderBuf + 18274, g_scanOrderBuf + 18530, g_scanOrderBuf + 19042, },
    { g_scanOrderBuf + 20066, g_scanOrderBuf + 20098, g_scanOrderBuf + 20162, g_scanOrderBuf + 20290, g_scanOrderBuf + 20546, g_scanOrderBuf + 21058, g_scanOrderBuf + 22082, },
    { g_scanOrderBuf + 24130, g_scanOrderBuf + 24194, g_scanOrderBuf + 24322, g_scanOrderBuf + 24578, g_scanOrderBuf + 25090, g_scanOrderBuf + 26114, g_scanOrderBuf + 28162, },
  }
};

const uint16_t g_coefTopLeftDiagScan8x8[MAX_LOG2_TU_SIZE_PLUS_ONE][64] =
{
  {0, 1, 1, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 5, 5, 6, 4, 5, 5, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 10, 4, 5, 5, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 10, 8, 9, 9, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 13, 13, 14, },
  {0, 2, 1, 4, 3, 2, 6, 5, 4, 3, 7, 6, 5, 8, 7, 9, 8, 10, 9, 12, 11, 10, 14, 13, 12, 11, 15, 14, 13, 16, 15, 17, 4, 6, 5, 8, 7, 6, 10, 9, 8, 7, 11, 10, 9, 12, 11, 13, 12, 14, 13, 16, 15, 14, 18, 17, 16, 15, 19, 18, 17, 20, 19, 21, },
  {0, 4, 1, 8, 5, 2, 12, 9, 6, 3, 13, 10, 7, 14, 11, 15, 16, 20, 17, 24, 21, 18, 28, 25, 22, 19, 29, 26, 23, 30, 27, 31, 4, 8, 5, 12, 9, 6, 16, 13, 10, 7, 17, 14, 11, 18, 15, 19, 20, 24, 21, 28, 25, 22, 32, 29, 26, 23, 33, 30, 27, 34, 31, 35, },
  {0, 8, 1, 16, 9, 2, 24, 17, 10, 3, 25, 18, 11, 26, 19, 27, 32, 40, 33, 48, 41, 34, 56, 49, 42, 35, 57, 50, 43, 58, 51, 59, 4, 12, 5, 20, 13, 6, 28, 21, 14, 7, 29, 22, 15, 30, 23, 31, 36, 44, 37, 52, 45, 38, 60, 53, 46, 39, 61, 54, 47, 62, 55, 63, },
  {0, 16, 1, 32, 17, 2, 48, 33, 18, 3, 49, 34, 19, 50, 35, 51, 64, 80, 65, 96, 81, 66, 112, 97, 82, 67, 113, 98, 83, 114, 99, 115, 4, 20, 5, 36, 21, 6, 52, 37, 22, 7, 53, 38, 23, 54, 39, 55, 68, 84, 69, 100, 85, 70, 116, 101, 86, 71, 117, 102, 87, 118, 103, 119, },
  {0, 32, 1, 64, 33, 2, 96, 65, 34, 3, 97, 66, 35, 98, 67, 99, 128, 160, 129, 192, 161, 130, 224, 193, 162, 131, 225, 194, 163, 226, 195, 227, 4, 36, 5, 68, 37, 6, 100, 69, 38, 7, 101, 70, 39, 102, 71, 103, 132, 164, 133, 196, 165, 134, 228, 197, 166, 135, 229, 198, 167, 230, 199, 231, },
  {0, 64, 1, 128, 65, 2, 192, 129, 66, 3, 193, 130, 67, 194, 131, 195, 256, 320, 257, 384, 321, 258, 448, 385, 322, 259, 449, 386, 323, 450, 387, 451, 4, 68, 5, 132, 69, 6, 196, 133, 70, 7, 197, 134, 71, 198, 135, 199, 260, 324, 261, 388, 325, 262, 452, 389, 326, 263, 453, 390, 327, 454, 391, 455, },
};

const uint32_t ctxIndMap4x4[4 * 4] =
{
  0, 1, 4, 5,
  2, 3, 4, 5,
  6, 6, 8, 8,
  7, 7, 8, 8
};


const uint32_t g_uiMinInGroup[LAST_SIGNIFICANT_GROUPS] = { 0,1,2,3,4,6,8,12,16,24,32,48,64,96 };
const uint32_t g_uiGroupIdx[MAX_TU_SIZE_FOR_PROFILE] = { 0,1,2,3,4,4,5,5,6,6,6,6,7,7,7,7,8,8,8,8,8,8,8,8,9,9,9,9,9,9,9,9, 10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11};
const uint32_t g_auiGoRiceParsCoeff[32] =
{
  0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3
};

const int g_quantTSDefault4x4[4 * 4] =
{
  16,16,16,16,
  16,16,16,16,
  16,16,16,16,
  16,16,16,16
};

const int g_quantIntraDefault8x8[8 * 8] =
{
  16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16
};

const int g_quantInterDefault8x8[8 * 8] =
{
  16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16
};

const uint32_t g_vvcScalingListSizeX[SCALING_LIST_SIZE_NUM] = { 1, 2,  4,  8,  16,   32,   64 };

const uint32_t g_scalingListId[SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM] =
{
  {  0,  0,  0,  0,  0,  0},  // SCALING_LIST_1x1
  {  0,  0,  0,  0,  0,  1},  // SCALING_LIST_2x2
  {  2,  3,  4,  5,  6,  7},  // SCALING_LIST_4x4
  {  8,  9, 10, 11, 12, 13},  // SCALING_LIST_8x8
  { 14, 15, 16, 17, 18, 19},  // SCALING_LIST_16x16
  { 20, 21, 22, 23, 24, 25},  // SCALING_LIST_32x32
  { 26, 21, 22, 27, 24, 25},  // SCALING_LIST_64x64
};



const int g_alfNumCoeff[ALF_NUM_OF_FILTER_TYPES] = { 7, 13, 8 };

void initGeoTemplate()
{
  int modeIdx = 0;
  for (int angleIdx = 0; angleIdx < GEO_NUM_ANGLES; angleIdx++)
  {
    for (int distanceIdx = 0; distanceIdx < GEO_NUM_DISTANCES; distanceIdx++)
    {
      if ((distanceIdx == 0 && angleIdx >= 16)
          || ((distanceIdx == 2 || distanceIdx == 0) && (g_angle2mask[angleIdx] == 0 || g_angle2mask[angleIdx] == 5))
          || g_angle2mask[angleIdx] == -1)
        continue;
      g_GeoParams[modeIdx][0] = (int16_t) angleIdx;
      g_GeoParams[modeIdx][1] = (int16_t) distanceIdx;
      modeIdx++;
    }
  }
  for (int angleIdx = 0; angleIdx < (GEO_NUM_ANGLES >> 2) + 1; angleIdx++)
  {
    if (g_angle2mask[angleIdx] == -1)
      continue;

    int     distanceX = angleIdx;
    int     distanceY = (distanceX + (GEO_NUM_ANGLES >> 2)) % GEO_NUM_ANGLES;
    int16_t rho       = (g_Dis[distanceX] *(1<<(GEO_MAX_CU_LOG2 + 1))) + (g_Dis[distanceY] *(1<<(GEO_MAX_CU_LOG2 + 1)));
    static const int16_t maskOffset = (2 * GEO_MAX_CU_SIZE - GEO_WEIGHT_MASK_SIZE) >> 1;
    int                  index      = 0;
    for (int y = 0; y < GEO_WEIGHT_MASK_SIZE; y++)
    {
      int16_t lookUpY = (((y + maskOffset) << 1) + 1) * g_Dis[distanceY];
      for (int x = 0; x < GEO_WEIGHT_MASK_SIZE; x++, index++)
      {
        int16_t sx_i                                         = ((x + maskOffset) << 1) + 1;
        int16_t weightIdx                                    = sx_i * g_Dis[distanceX] + lookUpY - rho;
        int     weightLinearIdx                              = 32 + weightIdx;
        g_globalGeoWeights[g_angle2mask[angleIdx]][index]    = Clip3(0, 8, (weightLinearIdx + 4) >> 3);
      }
    }
  }

  for (int hIdx = 0; hIdx < GEO_NUM_CU_SIZE; hIdx++)
  {
    int16_t height = 1 << (hIdx + GEO_MIN_CU_LOG2);
    for (int wIdx = 0; wIdx < GEO_NUM_CU_SIZE; wIdx++)
    {
      int16_t width = 1 << (wIdx + GEO_MIN_CU_LOG2);
      for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
      {
        int16_t angle    = g_GeoParams[splitDir][0];
        int16_t distance = g_GeoParams[splitDir][1];
        int16_t offsetX  = (GEO_WEIGHT_MASK_SIZE - width) >> 1;
        int16_t offsetY  = (GEO_WEIGHT_MASK_SIZE - height) >> 1;
        if (distance > 0)
        {
          if (angle % 16 == 8 || (angle % 16 != 0 && height >= width))
          {
            offsetY += angle < 16 ? ((distance * (int32_t) height) >> 3) : -((distance * (int32_t) height) >> 3);
          }
          else
          {
            offsetX += angle < 16 ? ((distance * (int32_t) width) >> 3) : -((distance * (int32_t) width) >> 3);
          }
        }
        g_weightOffset[splitDir][hIdx][wIdx][0] = offsetX;
        g_weightOffset[splitDir][hIdx][wIdx][1] = offsetY;
      }
    }
  }
}

int16_t   g_GeoParams           [GEO_NUM_PARTITION_MODE][2];
int16_t   g_globalGeoWeights    [GEO_NUM_PRESTORED_MASK]   [GEO_WEIGHT_MASK_SIZE * GEO_WEIGHT_MASK_SIZE];

int16_t         g_weightOffset[GEO_NUM_PARTITION_MODE][GEO_NUM_CU_SIZE][GEO_NUM_CU_SIZE][2];
const int8_t    g_angle2mask  [GEO_NUM_ANGLES] = { 0, -1, 1, 2, 3, 4, -1, -1, 5, -1, -1, 4, 3, 2, 1, -1, 0, -1, 1, 2, 3, 4, -1, -1, 5, -1, -1, 4, 3, 2, 1, -1 };
const int8_t    g_Dis         [GEO_NUM_ANGLES] = { 8, 8, 8, 8, 4, 4, 2, 1, 0, -1, -2, -4, -4, -8, -8, -8, -8, -8, -8, -8, -4, -4, -2, -1, 0, 1, 2, 4, 4, 8, 8, 8 };
const int8_t    g_angle2mirror[GEO_NUM_ANGLES] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2 };

const uint16_t g_scanOrderBuf[32258] = {
  0, 0, 1, 0, 1, 2, 3, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
  19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36,
  37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 0, 1, 0, 2, 1, 3, 0, 2, 1, 4, 3, 6, 5, 7, 0, 2, 1, 4, 3, 6, 5, 8, 7,
  10, 9, 12, 11, 14, 13, 15, 0, 2, 1, 4, 3, 6, 5, 8, 7, 10, 9, 12, 11, 14, 13, 16, 15, 18, 17, 20, 19, 22, 21, 24, 23, 26, 25, 28, 27, 30, 29, 31, 0, 2, 1, 4, 3, 6, 5, 8, 7, 10, 9,
  12, 11, 14, 13, 16, 15, 18, 17, 20, 19, 22, 21, 24, 23, 26, 25, 28, 27, 30, 29, 32, 31, 34, 33, 36, 35, 38, 37, 40, 39, 42, 41, 44, 43, 46, 45, 48, 47, 50, 49, 52, 51, 54, 53, 56, 55, 58, 57, 60, 59,
  62, 61, 63, 0, 2, 1, 4, 3, 6, 5, 8, 7, 10, 9, 12, 11, 14, 13, 16, 15, 18, 17, 20, 19, 22, 21, 24, 23, 26, 25, 28, 27, 30, 29, 32, 31, 34, 33, 36, 35, 38, 37, 40, 39, 42, 41, 44, 43, 46, 45,
  48, 47, 50, 49, 52, 51, 54, 53, 56, 55, 58, 57, 60, 59, 62, 61, 64, 63, 66, 65, 68, 67, 70, 69, 72, 71, 74, 73, 76, 75, 78, 77, 80, 79, 82, 81, 84, 83, 86, 85, 88, 87, 90, 89, 92, 91, 94, 93, 96, 95,
  98, 97, 100, 99, 102, 101, 104, 103, 106, 105, 108, 107, 110, 109, 112, 111, 114, 113, 116, 115, 118, 117, 120, 119, 122, 121, 124, 123, 126, 125, 127, 0, 1, 2, 3, 0, 4, 1, 5, 2, 6, 3, 7, 0, 4, 1, 8, 5, 2, 12,
  9, 6, 3, 13, 10, 7, 14, 11, 15, 0, 4, 1, 8, 5, 2, 12, 9, 6, 3, 16, 13, 10, 7, 20, 17, 14, 11, 24, 21, 18, 15, 28, 25, 22, 19, 29, 26, 23, 30, 27, 31, 0, 4, 1, 8, 5, 2, 12, 9, 6,
  3, 16, 13, 10, 7, 20, 17, 14, 11, 24, 21, 18, 15, 28, 25, 22, 19, 32, 29, 26, 23, 36, 33, 30, 27, 40, 37, 34, 31, 44, 41, 38, 35, 48, 45, 42, 39, 52, 49, 46, 43, 56, 53, 50, 47, 60, 57, 54, 51, 61,
  58, 55, 62, 59, 63, 0, 4, 1, 8, 5, 2, 12, 9, 6, 3, 16, 13, 10, 7, 20, 17, 14, 11, 24, 21, 18, 15, 28, 25, 22, 19, 32, 29, 26, 23, 36, 33, 30, 27, 40, 37, 34, 31, 44, 41, 38, 35, 48, 45, 42,
  39, 52, 49, 46, 43, 56, 53, 50, 47, 60, 57, 54, 51, 64, 61, 58, 55, 68, 65, 62, 59, 72, 69, 66, 63, 76, 73, 70, 67, 80, 77, 74, 71, 84, 81, 78, 75, 88, 85, 82, 79, 92, 89, 86, 83, 96, 93, 90, 87, 100,
  97, 94, 91, 104, 101, 98, 95, 108, 105, 102, 99, 112, 109, 106, 103, 116, 113, 110, 107, 120, 117, 114, 111, 124, 121, 118, 115, 125, 122, 119, 126, 123, 127, 0, 4, 1, 8, 5, 2, 12, 9, 6, 3, 16, 13, 10, 7, 20, 17, 14,
  11, 24, 21, 18, 15, 28, 25, 22, 19, 32, 29, 26, 23, 36, 33, 30, 27, 40, 37, 34, 31, 44, 41, 38, 35, 48, 45, 42, 39, 52, 49, 46, 43, 56, 53, 50, 47, 60, 57, 54, 51, 64, 61, 58, 55, 68, 65, 62, 59, 72,
  69, 66, 63, 76, 73, 70, 67, 80, 77, 74, 71, 84, 81, 78, 75, 88, 85, 82, 79, 92, 89, 86, 83, 96, 93, 90, 87, 100, 97, 94, 91, 104, 101, 98, 95, 108, 105, 102, 99, 112, 109, 106, 103, 116, 113, 110, 107, 120, 117, 114,
  111, 124, 121, 118, 115, 128, 125, 122, 119, 132, 129, 126, 123, 136, 133, 130, 127, 140, 137, 134, 131, 144, 141, 138, 135, 148, 145, 142, 139, 152, 149, 146, 143, 156, 153, 150, 147, 160, 157, 154, 151, 164, 161, 158, 155, 168, 165, 162, 159, 172,
  169, 166, 163, 176, 173, 170, 167, 180, 177, 174, 171, 184, 181, 178, 175, 188, 185, 182, 179, 192, 189, 186, 183, 196, 193, 190, 187, 200, 197, 194, 191, 204, 201, 198, 195, 208, 205, 202, 199, 212, 209, 206, 203, 216, 213, 210, 207, 220, 217, 214,
  211, 224, 221, 218, 215, 228, 225, 222, 219, 232, 229, 226, 223, 236, 233, 230, 227, 240, 237, 234, 231, 244, 241, 238, 235, 248, 245, 242, 239, 252, 249, 246, 243, 253, 250, 247, 254, 251, 255, 0, 1, 2, 3, 4, 5, 6, 7, 0, 8, 1,
  9, 2, 10, 3, 11, 4, 12, 5, 13, 6, 14, 7, 15, 0, 8, 1, 16, 9, 2, 24, 17, 10, 3, 25, 18, 11, 4, 26, 19, 12, 5, 27, 20, 13, 6, 28, 21, 14, 7, 29, 22, 15, 30, 23, 31, 0, 8, 1, 16, 9,
  2, 24, 17, 10, 3, 32, 25, 18, 11, 4, 40, 33, 26, 19, 12, 5, 48, 41, 34, 27, 20, 13, 6, 56, 49, 42, 35, 28, 21, 14, 7, 57, 50, 43, 36, 29, 22, 15, 58, 51, 44, 37, 30, 23, 59, 52, 45, 38, 31, 60,
  53, 46, 39, 61, 54, 47, 62, 55, 63, 0, 8, 1, 16, 9, 2, 24, 17, 10, 3, 32, 25, 18, 11, 4, 40, 33, 26, 19, 12, 5, 48, 41, 34, 27, 20, 13, 6, 56, 49, 42, 35, 28, 21, 14, 7, 64, 57, 50, 43, 36,
  29, 22, 15, 72, 65, 58, 51, 44, 37, 30, 23, 80, 73, 66, 59, 52, 45, 38, 31, 88, 81, 74, 67, 60, 53, 46, 39, 96, 89, 82, 75, 68, 61, 54, 47, 104, 97, 90, 83, 76, 69, 62, 55, 112, 105, 98, 91, 84, 77, 70,
  63, 120, 113, 106, 99, 92, 85, 78, 71, 121, 114, 107, 100, 93, 86, 79, 122, 115, 108, 101, 94, 87, 123, 116, 109, 102, 95, 124, 117, 110, 103, 125, 118, 111, 126, 119, 127, 0, 8, 1, 16, 9, 2, 24, 17, 10, 3, 32, 25, 18,
  11, 4, 40, 33, 26, 19, 12, 5, 48, 41, 34, 27, 20, 13, 6, 56, 49, 42, 35, 28, 21, 14, 7, 64, 57, 50, 43, 36, 29, 22, 15, 72, 65, 58, 51, 44, 37, 30, 23, 80, 73, 66, 59, 52, 45, 38, 31, 88, 81, 74,
  67, 60, 53, 46, 39, 96, 89, 82, 75, 68, 61, 54, 47, 104, 97, 90, 83, 76, 69, 62, 55, 112, 105, 98, 91, 84, 77, 70, 63, 120, 113, 106, 99, 92, 85, 78, 71, 128, 121, 114, 107, 100, 93, 86, 79, 136, 129, 122, 115, 108,
  101, 94, 87, 144, 137, 130, 123, 116, 109, 102, 95, 152, 145, 138, 131, 124, 117, 110, 103, 160, 153, 146, 139, 132, 125, 118, 111, 168, 161, 154, 147, 140, 133, 126, 119, 176, 169, 162, 155, 148, 141, 134, 127, 184, 177, 170, 163, 156, 149, 142,
  135, 192, 185, 178, 171, 164, 157, 150, 143, 200, 193, 186, 179, 172, 165, 158, 151, 208, 201, 194, 187, 180, 173, 166, 159, 216, 209, 202, 195, 188, 181, 174, 167, 224, 217, 210, 203, 196, 189, 182, 175, 232, 225, 218, 211, 204, 197, 190, 183, 240,
  233, 226, 219, 212, 205, 198, 191, 248, 241, 234, 227, 220, 213, 206, 199, 249, 242, 235, 228, 221, 214, 207, 250, 243, 236, 229, 222, 215, 251, 244, 237, 230, 223, 252, 245, 238, 231, 253, 246, 239, 254, 247, 255, 0, 8, 1, 16, 9, 2, 24,
  17, 10, 3, 32, 25, 18, 11, 4, 40, 33, 26, 19, 12, 5, 48, 41, 34, 27, 20, 13, 6, 56, 49, 42, 35, 28, 21, 14, 7, 64, 57, 50, 43, 36, 29, 22, 15, 72, 65, 58, 51, 44, 37, 30, 23, 80, 73, 66, 59, 52,
  45, 38, 31, 88, 81, 74, 67, 60, 53, 46, 39, 96, 89, 82, 75, 68, 61, 54, 47, 104, 97, 90, 83, 76, 69, 62, 55, 112, 105, 98, 91, 84, 77, 70, 63, 120, 113, 106, 99, 92, 85, 78, 71, 128, 121, 114, 107, 100, 93, 86,
  79, 136, 129, 122, 115, 108, 101, 94, 87, 144, 137, 130, 123, 116, 109, 102, 95, 152, 145, 138, 131, 124, 117, 110, 103, 160, 153, 146, 139, 132, 125, 118, 111, 168, 161, 154, 147, 140, 133, 126, 119, 176, 169, 162, 155, 148, 141, 134, 127, 184,
  177, 170, 163, 156, 149, 142, 135, 192, 185, 178, 171, 164, 157, 150, 143, 200, 193, 186, 179, 172, 165, 158, 151, 208, 201, 194, 187, 180, 173, 166, 159, 216, 209, 202, 195, 188, 181, 174, 167, 224, 217, 210, 203, 196, 189, 182, 175, 232, 225, 218,
  211, 204, 197, 190, 183, 240, 233, 226, 219, 212, 205, 198, 191, 248, 241, 234, 227, 220, 213, 206, 199, 256, 249, 242, 235, 228, 221, 214, 207, 264, 257, 250, 243, 236, 229, 222, 215, 272, 265, 258, 251, 244, 237, 230, 223, 280, 273, 266, 259, 252,
  245, 238, 231, 288, 281, 274, 267, 260, 253, 246, 239, 296, 289, 282, 275, 268, 261, 254, 247, 304, 297, 290, 283, 276, 269, 262, 255, 312, 305, 298, 291, 284, 277, 270, 263, 320, 313, 306, 299, 292, 285, 278, 271, 328, 321, 314, 307, 300, 293, 286,
  279, 336, 329, 322, 315, 308, 301, 294, 287, 344, 337, 330, 323, 316, 309, 302, 295, 352, 345, 338, 331, 324, 317, 310, 303, 360, 353, 346, 339, 332, 325, 318, 311, 368, 361, 354, 347, 340, 333, 326, 319, 376, 369, 362, 355, 348, 341, 334, 327, 384,
  377, 370, 363, 356, 349, 342, 335, 392, 385, 378, 371, 364, 357, 350, 343, 400, 393, 386, 379, 372, 365, 358, 351, 408, 401, 394, 387, 380, 373, 366, 359, 416, 409, 402, 395, 388, 381, 374, 367, 424, 417, 410, 403, 396, 389, 382, 375, 432, 425, 418,
  411, 404, 397, 390, 383, 440, 433, 426, 419, 412, 405, 398, 391, 448, 441, 434, 427, 420, 413, 406, 399, 456, 449, 442, 435, 428, 421, 414, 407, 464, 457, 450, 443, 436, 429, 422, 415, 472, 465, 458, 451, 444, 437, 430, 423, 480, 473, 466, 459, 452,
  445, 438, 431, 488, 481, 474, 467, 460, 453, 446, 439, 496, 489, 482, 475, 468, 461, 454, 447, 504, 497, 490, 483, 476, 469, 462, 455, 505, 498, 491, 484, 477, 470, 463, 506, 499, 492, 485, 478, 471, 507, 500, 493, 486, 479, 508, 501, 494, 487, 509,
  502, 495, 510, 503, 511, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 0, 16, 1, 17, 2, 18, 3, 19, 4, 20, 5, 21, 6, 22, 7, 23, 8, 24, 9, 25, 10, 26, 11, 27, 12, 28, 13, 29, 14,
  30, 15, 31, 0, 16, 1, 32, 17, 2, 48, 33, 18, 3, 49, 34, 19, 4, 50, 35, 20, 5, 51, 36, 21, 6, 52, 37, 22, 7, 53, 38, 23, 8, 54, 39, 24, 9, 55, 40, 25, 10, 56, 41, 26, 11, 57, 42, 27, 12, 58,
  43, 28, 13, 59, 44, 29, 14, 60, 45, 30, 15, 61, 46, 31, 62, 47, 63, 0, 16, 1, 32, 17, 2, 48, 33, 18, 3, 64, 49, 34, 19, 4, 80, 65, 50, 35, 20, 5, 96, 81, 66, 51, 36, 21, 6, 112, 97, 82, 67, 52,
  37, 22, 7, 113, 98, 83, 68, 53, 38, 23, 8, 114, 99, 84, 69, 54, 39, 24, 9, 115, 100, 85, 70, 55, 40, 25, 10, 116, 101, 86, 71, 56, 41, 26, 11, 117, 102, 87, 72, 57, 42, 27, 12, 118, 103, 88, 73, 58, 43, 28,
  13, 119, 104, 89, 74, 59, 44, 29, 14, 120, 105, 90, 75, 60, 45, 30, 15, 121, 106, 91, 76, 61, 46, 31, 122, 107, 92, 77, 62, 47, 123, 108, 93, 78, 63, 124, 109, 94, 79, 125, 110, 95, 126, 111, 127, 0, 16, 1, 32, 17,
  2, 48, 33, 18, 3, 64, 49, 34, 19, 4, 80, 65, 50, 35, 20, 5, 96, 81, 66, 51, 36, 21, 6, 112, 97, 82, 67, 52, 37, 22, 7, 128, 113, 98, 83, 68, 53, 38, 23, 8, 144, 129, 114, 99, 84, 69, 54, 39, 24, 9,
  160, 145, 130, 115, 100, 85, 70, 55, 40, 25, 10, 176, 161, 146, 131, 116, 101, 86, 71, 56, 41, 26, 11, 192, 177, 162, 147, 132, 117, 102, 87, 72, 57, 42, 27, 12, 208, 193, 178, 163, 148, 133, 118, 103, 88, 73, 58, 43, 28, 13,
  224, 209, 194, 179, 164, 149, 134, 119, 104, 89, 74, 59, 44, 29, 14, 240, 225, 210, 195, 180, 165, 150, 135, 120, 105, 90, 75, 60, 45, 30, 15, 241, 226, 211, 196, 181, 166, 151, 136, 121, 106, 91, 76, 61, 46, 31, 242, 227, 212, 197,
  182, 167, 152, 137, 122, 107, 92, 77, 62, 47, 243, 228, 213, 198, 183, 168, 153, 138, 123, 108, 93, 78, 63, 244, 229, 214, 199, 184, 169, 154, 139, 124, 109, 94, 79, 245, 230, 215, 200, 185, 170, 155, 140, 125, 110, 95, 246, 231, 216, 201,
  186, 171, 156, 141, 126, 111, 247, 232, 217, 202, 187, 172, 157, 142, 127, 248, 233, 218, 203, 188, 173, 158, 143, 249, 234, 219, 204, 189, 174, 159, 250, 235, 220, 205, 190, 175, 251, 236, 221, 206, 191, 252, 237, 222, 207, 253, 238, 223, 254, 239,
  255, 0, 16, 1, 32, 17, 2, 48, 33, 18, 3, 64, 49, 34, 19, 4, 80, 65, 50, 35, 20, 5, 96, 81, 66, 51, 36, 21, 6, 112, 97, 82, 67, 52, 37, 22, 7, 128, 113, 98, 83, 68, 53, 38, 23, 8, 144, 129, 114, 99,
  84, 69, 54, 39, 24, 9, 160, 145, 130, 115, 100, 85, 70, 55, 40, 25, 10, 176, 161, 146, 131, 116, 101, 86, 71, 56, 41, 26, 11, 192, 177, 162, 147, 132, 117, 102, 87, 72, 57, 42, 27, 12, 208, 193, 178, 163, 148, 133, 118, 103,
  88, 73, 58, 43, 28, 13, 224, 209, 194, 179, 164, 149, 134, 119, 104, 89, 74, 59, 44, 29, 14, 240, 225, 210, 195, 180, 165, 150, 135, 120, 105, 90, 75, 60, 45, 30, 15, 256, 241, 226, 211, 196, 181, 166, 151, 136, 121, 106, 91, 76,
  61, 46, 31, 272, 257, 242, 227, 212, 197, 182, 167, 152, 137, 122, 107, 92, 77, 62, 47, 288, 273, 258, 243, 228, 213, 198, 183, 168, 153, 138, 123, 108, 93, 78, 63, 304, 289, 274, 259, 244, 229, 214, 199, 184, 169, 154, 139, 124, 109, 94,
  79, 320, 305, 290, 275, 260, 245, 230, 215, 200, 185, 170, 155, 140, 125, 110, 95, 336, 321, 306, 291, 276, 261, 246, 231, 216, 201, 186, 171, 156, 141, 126, 111, 352, 337, 322, 307, 292, 277, 262, 247, 232, 217, 202, 187, 172, 157, 142, 127, 368,
  353, 338, 323, 308, 293, 278, 263, 248, 233, 218, 203, 188, 173, 158, 143, 384, 369, 354, 339, 324, 309, 294, 279, 264, 249, 234, 219, 204, 189, 174, 159, 400, 385, 370, 355, 340, 325, 310, 295, 280, 265, 250, 235, 220, 205, 190, 175, 416, 401, 386,
  371, 356, 341, 326, 311, 296, 281, 266, 251, 236, 221, 206, 191, 432, 417, 402, 387, 372, 357, 342, 327, 312, 297, 282, 267, 252, 237, 222, 207, 448, 433, 418, 403, 388, 373, 358, 343, 328, 313, 298, 283, 268, 253, 238, 223, 464, 449, 434, 419, 404,
  389, 374, 359, 344, 329, 314, 299, 284, 269, 254, 239, 480, 465, 450, 435, 420, 405, 390, 375, 360, 345, 330, 315, 300, 285, 270, 255, 496, 481, 466, 451, 436, 421, 406, 391, 376, 361, 346, 331, 316, 301, 286, 271, 497, 482, 467, 452, 437, 422, 407,
  392, 377, 362, 347, 332, 317, 302, 287, 498, 483, 468, 453, 438, 423, 408, 393, 378, 363, 348, 333, 318, 303, 499, 484, 469, 454, 439, 424, 409, 394, 379, 364, 349, 334, 319, 500, 485, 470, 455, 440, 425, 410, 395, 380, 365, 350, 335, 501, 486, 471,
  456, 441, 426, 411, 396, 381, 366, 351, 502, 487, 472, 457, 442, 427, 412, 397, 382, 367, 503, 488, 473, 458, 443, 428, 413, 398, 383, 504, 489, 474, 459, 444, 429, 414, 399, 505, 490, 475, 460, 445, 430, 415, 506, 491, 476, 461, 446, 431, 507, 492,
  477, 462, 447, 508, 493, 478, 463, 509, 494, 479, 510, 495, 511, 0, 16, 1, 32, 17, 2, 48, 33, 18, 3, 64, 49, 34, 19, 4, 80, 65, 50, 35, 20, 5, 96, 81, 66, 51, 36, 21, 6, 112, 97, 82, 67, 52, 37, 22, 7, 128,
  113, 98, 83, 68, 53, 38, 23, 8, 144, 129, 114, 99, 84, 69, 54, 39, 24, 9, 160, 145, 130, 115, 100, 85, 70, 55, 40, 25, 10, 176, 161, 146, 131, 116, 101, 86, 71, 56, 41, 26, 11, 192, 177, 162, 147, 132, 117, 102, 87, 72,
  57, 42, 27, 12, 208, 193, 178, 163, 148, 133, 118, 103, 88, 73, 58, 43, 28, 13, 224, 209, 194, 179, 164, 149, 134, 119, 104, 89, 74, 59, 44, 29, 14, 240, 225, 210, 195, 180, 165, 150, 135, 120, 105, 90, 75, 60, 45, 30, 15, 256,
  241, 226, 211, 196, 181, 166, 151, 136, 121, 106, 91, 76, 61, 46, 31, 272, 257, 242, 227, 212, 197, 182, 167, 152, 137, 122, 107, 92, 77, 62, 47, 288, 273, 258, 243, 228, 213, 198, 183, 168, 153, 138, 123, 108, 93, 78, 63, 304, 289, 274,
  259, 244, 229, 214, 199, 184, 169, 154, 139, 124, 109, 94, 79, 320, 305, 290, 275, 260, 245, 230, 215, 200, 185, 170, 155, 140, 125, 110, 95, 336, 321, 306, 291, 276, 261, 246, 231, 216, 201, 186, 171, 156, 141, 126, 111, 352, 337, 322, 307, 292,
  277, 262, 247, 232, 217, 202, 187, 172, 157, 142, 127, 368, 353, 338, 323, 308, 293, 278, 263, 248, 233, 218, 203, 188, 173, 158, 143, 384, 369, 354, 339, 324, 309, 294, 279, 264, 249, 234, 219, 204, 189, 174, 159, 400, 385, 370, 355, 340, 325, 310,
  295, 280, 265, 250, 235, 220, 205, 190, 175, 416, 401, 386, 371, 356, 341, 326, 311, 296, 281, 266, 251, 236, 221, 206, 191, 432, 417, 402, 387, 372, 357, 342, 327, 312, 297, 282, 267, 252, 237, 222, 207, 448, 433, 418, 403, 388, 373, 358, 343, 328,
  313, 298, 283, 268, 253, 238, 223, 464, 449, 434, 419, 404, 389, 374, 359, 344, 329, 314, 299, 284, 269, 254, 239, 480, 465, 450, 435, 420, 405, 390, 375, 360, 345, 330, 315, 300, 285, 270, 255, 496, 481, 466, 451, 436, 421, 406, 391, 376, 361, 346,
  331, 316, 301, 286, 271, 512, 497, 482, 467, 452, 437, 422, 407, 392, 377, 362, 347, 332, 317, 302, 287, 528, 513, 498, 483, 468, 453, 438, 423, 408, 393, 378, 363, 348, 333, 318, 303, 544, 529, 514, 499, 484, 469, 454, 439, 424, 409, 394, 379, 364,
  349, 334, 319, 560, 545, 530, 515, 500, 485, 470, 455, 440, 425, 410, 395, 380, 365, 350, 335, 576, 561, 546, 531, 516, 501, 486, 471, 456, 441, 426, 411, 396, 381, 366, 351, 592, 577, 562, 547, 532, 517, 502, 487, 472, 457, 442, 427, 412, 397, 382,
  367, 608, 593, 578, 563, 548, 533, 518, 503, 488, 473, 458, 443, 428, 413, 398, 383, 624, 609, 594, 579, 564, 549, 534, 519, 504, 489, 474, 459, 444, 429, 414, 399, 640, 625, 610, 595, 580, 565, 550, 535, 520, 505, 490, 475, 460, 445, 430, 415, 656,
  641, 626, 611, 596, 581, 566, 551, 536, 521, 506, 491, 476, 461, 446, 431, 672, 657, 642, 627, 612, 597, 582, 567, 552, 537, 522, 507, 492, 477, 462, 447, 688, 673, 658, 643, 628, 613, 598, 583, 568, 553, 538, 523, 508, 493, 478, 463, 704, 689, 674,
  659, 644, 629, 614, 599, 584, 569, 554, 539, 524, 509, 494, 479, 720, 705, 690, 675, 660, 645, 630, 615, 600, 585, 570, 555, 540, 525, 510, 495, 736, 721, 706, 691, 676, 661, 646, 631, 616, 601, 586, 571, 556, 541, 526, 511, 752, 737, 722, 707, 692,
  677, 662, 647, 632, 617, 602, 587, 572, 557, 542, 527, 768, 753, 738, 723, 708, 693, 678, 663, 648, 633, 618, 603, 588, 573, 558, 543, 784, 769, 754, 739, 724, 709, 694, 679, 664, 649, 634, 619, 604, 589, 574, 559, 800, 785, 770, 755, 740, 725, 710,
  695, 680, 665, 650, 635, 620, 605, 590, 575, 816, 801, 786, 771, 756, 741, 726, 711, 696, 681, 666, 651, 636, 621, 606, 591, 832, 817, 802, 787, 772, 757, 742, 727, 712, 697, 682, 667, 652, 637, 622, 607, 848, 833, 818, 803, 788, 773, 758, 743, 728,
  713, 698, 683, 668, 653, 638, 623, 864, 849, 834, 819, 804, 789, 774, 759, 744, 729, 714, 699, 684, 669, 654, 639, 880, 865, 850, 835, 820, 805, 790, 775, 760, 745, 730, 715, 700, 685, 670, 655, 896, 881, 866, 851, 836, 821, 806, 791, 776, 761, 746,
  731, 716, 701, 686, 671, 912, 897, 882, 867, 852, 837, 822, 807, 792, 777, 762, 747, 732, 717, 702, 687, 928, 913, 898, 883, 868, 853, 838, 823, 808, 793, 778, 763, 748, 733, 718, 703, 944, 929, 914, 899, 884, 869, 854, 839, 824, 809, 794, 779, 764,
  749, 734, 719, 960, 945, 930, 915, 900, 885, 870, 855, 840, 825, 810, 795, 780, 765, 750, 735, 976, 961, 946, 931, 916, 901, 886, 871, 856, 841, 826, 811, 796, 781, 766, 751, 992, 977, 962, 947, 932, 917, 902, 887, 872, 857, 842, 827, 812, 797, 782,
  767, 1008, 993, 978, 963, 948, 933, 918, 903, 888, 873, 858, 843, 828, 813, 798, 783, 1009, 994, 979, 964, 949, 934, 919, 904, 889, 874, 859, 844, 829, 814, 799, 1010, 995, 980, 965, 950, 935, 920, 905, 890, 875, 860, 845, 830, 815, 1011, 996, 981, 966,
  951, 936, 921, 906, 891, 876, 861, 846, 831, 1012, 997, 982, 967, 952, 937, 922, 907, 892, 877, 862, 847, 1013, 998, 983, 968, 953, 938, 923, 908, 893, 878, 863, 1014, 999, 984, 969, 954, 939, 924, 909, 894, 879, 1015, 1000, 985, 970, 955, 940, 925, 910,
  895, 1016, 1001, 986, 971, 956, 941, 926, 911, 1017, 1002, 987, 972, 957, 942, 927, 1018, 1003, 988, 973, 958, 943, 1019, 1004, 989, 974, 959, 1020, 1005, 990, 975, 1021, 1006, 991, 1022, 1007, 1023, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
  13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 0, 32, 1, 33, 2, 34, 3, 35, 4, 36, 5, 37, 6, 38, 7, 39, 8, 40, 9, 41, 10, 42, 11, 43, 12, 44, 13, 45, 14, 46, 15,
  47, 16, 48, 17, 49, 18, 50, 19, 51, 20, 52, 21, 53, 22, 54, 23, 55, 24, 56, 25, 57, 26, 58, 27, 59, 28, 60, 29, 61, 30, 62, 31, 63, 0, 32, 1, 64, 33, 2, 96, 65, 34, 3, 97, 66, 35, 4, 98, 67, 36,
  5, 99, 68, 37, 6, 100, 69, 38, 7, 101, 70, 39, 8, 102, 71, 40, 9, 103, 72, 41, 10, 104, 73, 42, 11, 105, 74, 43, 12, 106, 75, 44, 13, 107, 76, 45, 14, 108, 77, 46, 15, 109, 78, 47, 16, 110, 79, 48, 17, 111,
  80, 49, 18, 112, 81, 50, 19, 113, 82, 51, 20, 114, 83, 52, 21, 115, 84, 53, 22, 116, 85, 54, 23, 117, 86, 55, 24, 118, 87, 56, 25, 119, 88, 57, 26, 120, 89, 58, 27, 121, 90, 59, 28, 122, 91, 60, 29, 123, 92, 61,
  30, 124, 93, 62, 31, 125, 94, 63, 126, 95, 127, 0, 32, 1, 64, 33, 2, 96, 65, 34, 3, 128, 97, 66, 35, 4, 160, 129, 98, 67, 36, 5, 192, 161, 130, 99, 68, 37, 6, 224, 193, 162, 131, 100, 69, 38, 7, 225, 194, 163,
  132, 101, 70, 39, 8, 226, 195, 164, 133, 102, 71, 40, 9, 227, 196, 165, 134, 103, 72, 41, 10, 228, 197, 166, 135, 104, 73, 42, 11, 229, 198, 167, 136, 105, 74, 43, 12, 230, 199, 168, 137, 106, 75, 44, 13, 231, 200, 169, 138, 107,
  76, 45, 14, 232, 201, 170, 139, 108, 77, 46, 15, 233, 202, 171, 140, 109, 78, 47, 16, 234, 203, 172, 141, 110, 79, 48, 17, 235, 204, 173, 142, 111, 80, 49, 18, 236, 205, 174, 143, 112, 81, 50, 19, 237, 206, 175, 144, 113, 82, 51,
  20, 238, 207, 176, 145, 114, 83, 52, 21, 239, 208, 177, 146, 115, 84, 53, 22, 240, 209, 178, 147, 116, 85, 54, 23, 241, 210, 179, 148, 117, 86, 55, 24, 242, 211, 180, 149, 118, 87, 56, 25, 243, 212, 181, 150, 119, 88, 57, 26, 244,
  213, 182, 151, 120, 89, 58, 27, 245, 214, 183, 152, 121, 90, 59, 28, 246, 215, 184, 153, 122, 91, 60, 29, 247, 216, 185, 154, 123, 92, 61, 30, 248, 217, 186, 155, 124, 93, 62, 31, 249, 218, 187, 156, 125, 94, 63, 250, 219, 188, 157,
  126, 95, 251, 220, 189, 158, 127, 252, 221, 190, 159, 253, 222, 191, 254, 223, 255, 0, 32, 1, 64, 33, 2, 96, 65, 34, 3, 128, 97, 66, 35, 4, 160, 129, 98, 67, 36, 5, 192, 161, 130, 99, 68, 37, 6, 224, 193, 162, 131, 100,
  69, 38, 7, 256, 225, 194, 163, 132, 101, 70, 39, 8, 288, 257, 226, 195, 164, 133, 102, 71, 40, 9, 320, 289, 258, 227, 196, 165, 134, 103, 72, 41, 10, 352, 321, 290, 259, 228, 197, 166, 135, 104, 73, 42, 11, 384, 353, 322, 291, 260,
  229, 198, 167, 136, 105, 74, 43, 12, 416, 385, 354, 323, 292, 261, 230, 199, 168, 137, 106, 75, 44, 13, 448, 417, 386, 355, 324, 293, 262, 231, 200, 169, 138, 107, 76, 45, 14, 480, 449, 418, 387, 356, 325, 294, 263, 232, 201, 170, 139, 108,
  77, 46, 15, 481, 450, 419, 388, 357, 326, 295, 264, 233, 202, 171, 140, 109, 78, 47, 16, 482, 451, 420, 389, 358, 327, 296, 265, 234, 203, 172, 141, 110, 79, 48, 17, 483, 452, 421, 390, 359, 328, 297, 266, 235, 204, 173, 142, 111, 80, 49,
  18, 484, 453, 422, 391, 360, 329, 298, 267, 236, 205, 174, 143, 112, 81, 50, 19, 485, 454, 423, 392, 361, 330, 299, 268, 237, 206, 175, 144, 113, 82, 51, 20, 486, 455, 424, 393, 362, 331, 300, 269, 238, 207, 176, 145, 114, 83, 52, 21, 487,
  456, 425, 394, 363, 332, 301, 270, 239, 208, 177, 146, 115, 84, 53, 22, 488, 457, 426, 395, 364, 333, 302, 271, 240, 209, 178, 147, 116, 85, 54, 23, 489, 458, 427, 396, 365, 334, 303, 272, 241, 210, 179, 148, 117, 86, 55, 24, 490, 459, 428,
  397, 366, 335, 304, 273, 242, 211, 180, 149, 118, 87, 56, 25, 491, 460, 429, 398, 367, 336, 305, 274, 243, 212, 181, 150, 119, 88, 57, 26, 492, 461, 430, 399, 368, 337, 306, 275, 244, 213, 182, 151, 120, 89, 58, 27, 493, 462, 431, 400, 369,
  338, 307, 276, 245, 214, 183, 152, 121, 90, 59, 28, 494, 463, 432, 401, 370, 339, 308, 277, 246, 215, 184, 153, 122, 91, 60, 29, 495, 464, 433, 402, 371, 340, 309, 278, 247, 216, 185, 154, 123, 92, 61, 30, 496, 465, 434, 403, 372, 341, 310,
  279, 248, 217, 186, 155, 124, 93, 62, 31, 497, 466, 435, 404, 373, 342, 311, 280, 249, 218, 187, 156, 125, 94, 63, 498, 467, 436, 405, 374, 343, 312, 281, 250, 219, 188, 157, 126, 95, 499, 468, 437, 406, 375, 344, 313, 282, 251, 220, 189, 158,
  127, 500, 469, 438, 407, 376, 345, 314, 283, 252, 221, 190, 159, 501, 470, 439, 408, 377, 346, 315, 284, 253, 222, 191, 502, 471, 440, 409, 378, 347, 316, 285, 254, 223, 503, 472, 441, 410, 379, 348, 317, 286, 255, 504, 473, 442, 411, 380, 349, 318,
  287, 505, 474, 443, 412, 381, 350, 319, 506, 475, 444, 413, 382, 351, 507, 476, 445, 414, 383, 508, 477, 446, 415, 509, 478, 447, 510, 479, 511, 0, 32, 1, 64, 33, 2, 96, 65, 34, 3, 128, 97, 66, 35, 4, 160, 129, 98, 67, 36, 5,
  192, 161, 130, 99, 68, 37, 6, 224, 193, 162, 131, 100, 69, 38, 7, 256, 225, 194, 163, 132, 101, 70, 39, 8, 288, 257, 226, 195, 164, 133, 102, 71, 40, 9, 320, 289, 258, 227, 196, 165, 134, 103, 72, 41, 10, 352, 321, 290, 259, 228,
  197, 166, 135, 104, 73, 42, 11, 384, 353, 322, 291, 260, 229, 198, 167, 136, 105, 74, 43, 12, 416, 385, 354, 323, 292, 261, 230, 199, 168, 137, 106, 75, 44, 13, 448, 417, 386, 355, 324, 293, 262, 231, 200, 169, 138, 107, 76, 45, 14, 480,
  449, 418, 387, 356, 325, 294, 263, 232, 201, 170, 139, 108, 77, 46, 15, 512, 481, 450, 419, 388, 357, 326, 295, 264, 233, 202, 171, 140, 109, 78, 47, 16, 544, 513, 482, 451, 420, 389, 358, 327, 296, 265, 234, 203, 172, 141, 110, 79, 48, 17,
  576, 545, 514, 483, 452, 421, 390, 359, 328, 297, 266, 235, 204, 173, 142, 111, 80, 49, 18, 608, 577, 546, 515, 484, 453, 422, 391, 360, 329, 298, 267, 236, 205, 174, 143, 112, 81, 50, 19, 640, 609, 578, 547, 516, 485, 454, 423, 392, 361, 330,
  299, 268, 237, 206, 175, 144, 113, 82, 51, 20, 672, 641, 610, 579, 548, 517, 486, 455, 424, 393, 362, 331, 300, 269, 238, 207, 176, 145, 114, 83, 52, 21, 704, 673, 642, 611, 580, 549, 518, 487, 456, 425, 394, 363, 332, 301, 270, 239, 208, 177,
  146, 115, 84, 53, 22, 736, 705, 674, 643, 612, 581, 550, 519, 488, 457, 426, 395, 364, 333, 302, 271, 240, 209, 178, 147, 116, 85, 54, 23, 768, 737, 706, 675, 644, 613, 582, 551, 520, 489, 458, 427, 396, 365, 334, 303, 272, 241, 210, 179, 148,
  117, 86, 55, 24, 800, 769, 738, 707, 676, 645, 614, 583, 552, 521, 490, 459, 428, 397, 366, 335, 304, 273, 242, 211, 180, 149, 118, 87, 56, 25, 832, 801, 770, 739, 708, 677, 646, 615, 584, 553, 522, 491, 460, 429, 398, 367, 336, 305, 274, 243,
  212, 181, 150, 119, 88, 57, 26, 864, 833, 802, 771, 740, 709, 678, 647, 616, 585, 554, 523, 492, 461, 430, 399, 368, 337, 306, 275, 244, 213, 182, 151, 120, 89, 58, 27, 896, 865, 834, 803, 772, 741, 710, 679, 648, 617, 586, 555, 524, 493, 462,
  431, 400, 369, 338, 307, 276, 245, 214, 183, 152, 121, 90, 59, 28, 928, 897, 866, 835, 804, 773, 742, 711, 680, 649, 618, 587, 556, 525, 494, 463, 432, 401, 370, 339, 308, 277, 246, 215, 184, 153, 122, 91, 60, 29, 960, 929, 898, 867, 836, 805,
  774, 743, 712, 681, 650, 619, 588, 557, 526, 495, 464, 433, 402, 371, 340, 309, 278, 247, 216, 185, 154, 123, 92, 61, 30, 992, 961, 930, 899, 868, 837, 806, 775, 744, 713, 682, 651, 620, 589, 558, 527, 496, 465, 434, 403, 372, 341, 310, 279, 248,
  217, 186, 155, 124, 93, 62, 31, 993, 962, 931, 900, 869, 838, 807, 776, 745, 714, 683, 652, 621, 590, 559, 528, 497, 466, 435, 404, 373, 342, 311, 280, 249, 218, 187, 156, 125, 94, 63, 994, 963, 932, 901, 870, 839, 808, 777, 746, 715, 684, 653,
  622, 591, 560, 529, 498, 467, 436, 405, 374, 343, 312, 281, 250, 219, 188, 157, 126, 95, 995, 964, 933, 902, 871, 840, 809, 778, 747, 716, 685, 654, 623, 592, 561, 530, 499, 468, 437, 406, 375, 344, 313, 282, 251, 220, 189, 158, 127, 996, 965, 934,
  903, 872, 841, 810, 779, 748, 717, 686, 655, 624, 593, 562, 531, 500, 469, 438, 407, 376, 345, 314, 283, 252, 221, 190, 159, 997, 966, 935, 904, 873, 842, 811, 780, 749, 718, 687, 656, 625, 594, 563, 532, 501, 470, 439, 408, 377, 346, 315, 284, 253,
  222, 191, 998, 967, 936, 905, 874, 843, 812, 781, 750, 719, 688, 657, 626, 595, 564, 533, 502, 471, 440, 409, 378, 347, 316, 285, 254, 223, 999, 968, 937, 906, 875, 844, 813, 782, 751, 720, 689, 658, 627, 596, 565, 534, 503, 472, 441, 410, 379, 348,
  317, 286, 255, 1000, 969, 938, 907, 876, 845, 814, 783, 752, 721, 690, 659, 628, 597, 566, 535, 504, 473, 442, 411, 380, 349, 318, 287, 1001, 970, 939, 908, 877, 846, 815, 784, 753, 722, 691, 660, 629, 598, 567, 536, 505, 474, 443, 412, 381, 350, 319,
  1002, 971, 940, 909, 878, 847, 816, 785, 754, 723, 692, 661, 630, 599, 568, 537, 506, 475, 444, 413, 382, 351, 1003, 972, 941, 910, 879, 848, 817, 786, 755, 724, 693, 662, 631, 600, 569, 538, 507, 476, 445, 414, 383, 1004, 973, 942, 911, 880, 849, 818,
  787, 756, 725, 694, 663, 632, 601, 570, 539, 508, 477, 446, 415, 1005, 974, 943, 912, 881, 850, 819, 788, 757, 726, 695, 664, 633, 602, 571, 540, 509, 478, 447, 1006, 975, 944, 913, 882, 851, 820, 789, 758, 727, 696, 665, 634, 603, 572, 541, 510, 479,
  1007, 976, 945, 914, 883, 852, 821, 790, 759, 728, 697, 666, 635, 604, 573, 542, 511, 1008, 977, 946, 915, 884, 853, 822, 791, 760, 729, 698, 667, 636, 605, 574, 543, 1009, 978, 947, 916, 885, 854, 823, 792, 761, 730, 699, 668, 637, 606, 575, 1010, 979,
  948, 917, 886, 855, 824, 793, 762, 731, 700, 669, 638, 607, 1011, 980, 949, 918, 887, 856, 825, 794, 763, 732, 701, 670, 639, 1012, 981, 950, 919, 888, 857, 826, 795, 764, 733, 702, 671, 1013, 982, 951, 920, 889, 858, 827, 796, 765, 734, 703, 1014, 983,
  952, 921, 890, 859, 828, 797, 766, 735, 1015, 984, 953, 922, 891, 860, 829, 798, 767, 1016, 985, 954, 923, 892, 861, 830, 799, 1017, 986, 955, 924, 893, 862, 831, 1018, 987, 956, 925, 894, 863, 1019, 988, 957, 926, 895, 1020, 989, 958, 927, 1021, 990, 959,
  1022, 991, 1023, 0, 32, 1, 64, 33, 2, 96, 65, 34, 3, 128, 97, 66, 35, 4, 160, 129, 98, 67, 36, 5, 192, 161, 130, 99, 68, 37, 6, 224, 193, 162, 131, 100, 69, 38, 7, 256, 225, 194, 163, 132, 101, 70, 39, 8, 288, 257,
  226, 195, 164, 133, 102, 71, 40, 9, 320, 289, 258, 227, 196, 165, 134, 103, 72, 41, 10, 352, 321, 290, 259, 228, 197, 166, 135, 104, 73, 42, 11, 384, 353, 322, 291, 260, 229, 198, 167, 136, 105, 74, 43, 12, 416, 385, 354, 323, 292, 261,
  230, 199, 168, 137, 106, 75, 44, 13, 448, 417, 386, 355, 324, 293, 262, 231, 200, 169, 138, 107, 76, 45, 14, 480, 449, 418, 387, 356, 325, 294, 263, 232, 201, 170, 139, 108, 77, 46, 15, 512, 481, 450, 419, 388, 357, 326, 295, 264, 233, 202,
  171, 140, 109, 78, 47, 16, 544, 513, 482, 451, 420, 389, 358, 327, 296, 265, 234, 203, 172, 141, 110, 79, 48, 17, 576, 545, 514, 483, 452, 421, 390, 359, 328, 297, 266, 235, 204, 173, 142, 111, 80, 49, 18, 608, 577, 546, 515, 484, 453, 422,
  391, 360, 329, 298, 267, 236, 205, 174, 143, 112, 81, 50, 19, 640, 609, 578, 547, 516, 485, 454, 423, 392, 361, 330, 299, 268, 237, 206, 175, 144, 113, 82, 51, 20, 672, 641, 610, 579, 548, 517, 486, 455, 424, 393, 362, 331, 300, 269, 238, 207,
  176, 145, 114, 83, 52, 21, 704, 673, 642, 611, 580, 549, 518, 487, 456, 425, 394, 363, 332, 301, 270, 239, 208, 177, 146, 115, 84, 53, 22, 736, 705, 674, 643, 612, 581, 550, 519, 488, 457, 426, 395, 364, 333, 302, 271, 240, 209, 178, 147, 116,
  85, 54, 23, 768, 737, 706, 675, 644, 613, 582, 551, 520, 489, 458, 427, 396, 365, 334, 303, 272, 241, 210, 179, 148, 117, 86, 55, 24, 800, 769, 738, 707, 676, 645, 614, 583, 552, 521, 490, 459, 428, 397, 366, 335, 304, 273, 242, 211, 180, 149,
  118, 87, 56, 25, 832, 801, 770, 739, 708, 677, 646, 615, 584, 553, 522, 491, 460, 429, 398, 367, 336, 305, 274, 243, 212, 181, 150, 119, 88, 57, 26, 864, 833, 802, 771, 740, 709, 678, 647, 616, 585, 554, 523, 492, 461, 430, 399, 368, 337, 306,
  275, 244, 213, 182, 151, 120, 89, 58, 27, 896, 865, 834, 803, 772, 741, 710, 679, 648, 617, 586, 555, 524, 493, 462, 431, 400, 369, 338, 307, 276, 245, 214, 183, 152, 121, 90, 59, 28, 928, 897, 866, 835, 804, 773, 742, 711, 680, 649, 618, 587,
  556, 525, 494, 463, 432, 401, 370, 339, 308, 277, 246, 215, 184, 153, 122, 91, 60, 29, 960, 929, 898, 867, 836, 805, 774, 743, 712, 681, 650, 619, 588, 557, 526, 495, 464, 433, 402, 371, 340, 309, 278, 247, 216, 185, 154, 123, 92, 61, 30, 992,
  961, 930, 899, 868, 837, 806, 775, 744, 713, 682, 651, 620, 589, 558, 527, 496, 465, 434, 403, 372, 341, 310, 279, 248, 217, 186, 155, 124, 93, 62, 31, 1024, 993, 962, 931, 900, 869, 838, 807, 776, 745, 714, 683, 652, 621, 590, 559, 528, 497, 466,
  435, 404, 373, 342, 311, 280, 249, 218, 187, 156, 125, 94, 63, 1056, 1025, 994, 963, 932, 901, 870, 839, 808, 777, 746, 715, 684, 653, 622, 591, 560, 529, 498, 467, 436, 405, 374, 343, 312, 281, 250, 219, 188, 157, 126, 95, 1088, 1057, 1026, 995, 964,
  933, 902, 871, 840, 809, 778, 747, 716, 685, 654, 623, 592, 561, 530, 499, 468, 437, 406, 375, 344, 313, 282, 251, 220, 189, 158, 127, 1120, 1089, 1058, 1027, 996, 965, 934, 903, 872, 841, 810, 779, 748, 717, 686, 655, 624, 593, 562, 531, 500, 469, 438,
  407, 376, 345, 314, 283, 252, 221, 190, 159, 1152, 1121, 1090, 1059, 1028, 997, 966, 935, 904, 873, 842, 811, 780, 749, 718, 687, 656, 625, 594, 563, 532, 501, 470, 439, 408, 377, 346, 315, 284, 253, 222, 191, 1184, 1153, 1122, 1091, 1060, 1029, 998, 967, 936,
  905, 874, 843, 812, 781, 750, 719, 688, 657, 626, 595, 564, 533, 502, 471, 440, 409, 378, 347, 316, 285, 254, 223, 1216, 1185, 1154, 1123, 1092, 1061, 1030, 999, 968, 937, 906, 875, 844, 813, 782, 751, 720, 689, 658, 627, 596, 565, 534, 503, 472, 441, 410,
  379, 348, 317, 286, 255, 1248, 1217, 1186, 1155, 1124, 1093, 1062, 1031, 1000, 969, 938, 907, 876, 845, 814, 783, 752, 721, 690, 659, 628, 597, 566, 535, 504, 473, 442, 411, 380, 349, 318, 287, 1280, 1249, 1218, 1187, 1156, 1125, 1094, 1063, 1032, 1001, 970, 939, 908,
  877, 846, 815, 784, 753, 722, 691, 660, 629, 598, 567, 536, 505, 474, 443, 412, 381, 350, 319, 1312, 1281, 1250, 1219, 1188, 1157, 1126, 1095, 1064, 1033, 1002, 971, 940, 909, 878, 847, 816, 785, 754, 723, 692, 661, 630, 599, 568, 537, 506, 475, 444, 413, 382,
  351, 1344, 1313, 1282, 1251, 1220, 1189, 1158, 1127, 1096, 1065, 1034, 1003, 972, 941, 910, 879, 848, 817, 786, 755, 724, 693, 662, 631, 600, 569, 538, 507, 476, 445, 414, 383, 1376, 1345, 1314, 1283, 1252, 1221, 1190, 1159, 1128, 1097, 1066, 1035, 1004, 973, 942, 911, 880,
  849, 818, 787, 756, 725, 694, 663, 632, 601, 570, 539, 508, 477, 446, 415, 1408, 1377, 1346, 1315, 1284, 1253, 1222, 1191, 1160, 1129, 1098, 1067, 1036, 1005, 974, 943, 912, 881, 850, 819, 788, 757, 726, 695, 664, 633, 602, 571, 540, 509, 478, 447, 1440, 1409, 1378,
  1347, 1316, 1285, 1254, 1223, 1192, 1161, 1130, 1099, 1068, 1037, 1006, 975, 944, 913, 882, 851, 820, 789, 758, 727, 696, 665, 634, 603, 572, 541, 510, 479, 1472, 1441, 1410, 1379, 1348, 1317, 1286, 1255, 1224, 1193, 1162, 1131, 1100, 1069, 1038, 1007, 976, 945, 914, 883, 852,
  821, 790, 759, 728, 697, 666, 635, 604, 573, 542, 511, 1504, 1473, 1442, 1411, 1380, 1349, 1318, 1287, 1256, 1225, 1194, 1163, 1132, 1101, 1070, 1039, 1008, 977, 946, 915, 884, 853, 822, 791, 760, 729, 698, 667, 636, 605, 574, 543, 1536, 1505, 1474, 1443, 1412, 1381, 1350,
  1319, 1288, 1257, 1226, 1195, 1164, 1133, 1102, 1071, 1040, 1009, 978, 947, 916, 885, 854, 823, 792, 761, 730, 699, 668, 637, 606, 575, 1568, 1537, 1506, 1475, 1444, 1413, 1382, 1351, 1320, 1289, 1258, 1227, 1196, 1165, 1134, 1103, 1072, 1041, 1010, 979, 948, 917, 886, 855, 824,
  793, 762, 731, 700, 669, 638, 607, 1600, 1569, 1538, 1507, 1476, 1445, 1414, 1383, 1352, 1321, 1290, 1259, 1228, 1197, 1166, 1135, 1104, 1073, 1042, 1011, 980, 949, 918, 887, 856, 825, 794, 763, 732, 701, 670, 639, 1632, 1601, 1570, 1539, 1508, 1477, 1446, 1415, 1384, 1353, 1322,
  1291, 1260, 1229, 1198, 1167, 1136, 1105, 1074, 1043, 1012, 981, 950, 919, 888, 857, 826, 795, 764, 733, 702, 671, 1664, 1633, 1602, 1571, 1540, 1509, 1478, 1447, 1416, 1385, 1354, 1323, 1292, 1261, 1230, 1199, 1168, 1137, 1106, 1075, 1044, 1013, 982, 951, 920, 889, 858, 827, 796,
  765, 734, 703, 1696, 1665, 1634, 1603, 1572, 1541, 1510, 1479, 1448, 1417, 1386, 1355, 1324, 1293, 1262, 1231, 1200, 1169, 1138, 1107, 1076, 1045, 1014, 983, 952, 921, 890, 859, 828, 797, 766, 735, 1728, 1697, 1666, 1635, 1604, 1573, 1542, 1511, 1480, 1449, 1418, 1387, 1356, 1325, 1294,
  1263, 1232, 1201, 1170, 1139, 1108, 1077, 1046, 1015, 984, 953, 922, 891, 860, 829, 798, 767, 1760, 1729, 1698, 1667, 1636, 1605, 1574, 1543, 1512, 1481, 1450, 1419, 1388, 1357, 1326, 1295, 1264, 1233, 1202, 1171, 1140, 1109, 1078, 1047, 1016, 985, 954, 923, 892, 861, 830, 799, 1792,
  1761, 1730, 1699, 1668, 1637, 1606, 1575, 1544, 1513, 1482, 1451, 1420, 1389, 1358, 1327, 1296, 1265, 1234, 1203, 1172, 1141, 1110, 1079, 1048, 1017, 986, 955, 924, 893, 862, 831, 1824, 1793, 1762, 1731, 1700, 1669, 1638, 1607, 1576, 1545, 1514, 1483, 1452, 1421, 1390, 1359, 1328, 1297, 1266,
  1235, 1204, 1173, 1142, 1111, 1080, 1049, 1018, 987, 956, 925, 894, 863, 1856, 1825, 1794, 1763, 1732, 1701, 1670, 1639, 1608, 1577, 1546, 1515, 1484, 1453, 1422, 1391, 1360, 1329, 1298, 1267, 1236, 1205, 1174, 1143, 1112, 1081, 1050, 1019, 988, 957, 926, 895, 1888, 1857, 1826, 1795, 1764,
  1733, 1702, 1671, 1640, 1609, 1578, 1547, 1516, 1485, 1454, 1423, 1392, 1361, 1330, 1299, 1268, 1237, 1206, 1175, 1144, 1113, 1082, 1051, 1020, 989, 958, 927, 1920, 1889, 1858, 1827, 1796, 1765, 1734, 1703, 1672, 1641, 1610, 1579, 1548, 1517, 1486, 1455, 1424, 1393, 1362, 1331, 1300, 1269, 1238,
  1207, 1176, 1145, 1114, 1083, 1052, 1021, 990, 959, 1952, 1921, 1890, 1859, 1828, 1797, 1766, 1735, 1704, 1673, 1642, 1611, 1580, 1549, 1518, 1487, 1456, 1425, 1394, 1363, 1332, 1301, 1270, 1239, 1208, 1177, 1146, 1115, 1084, 1053, 1022, 991, 1984, 1953, 1922, 1891, 1860, 1829, 1798, 1767, 1736,
  1705, 1674, 1643, 1612, 1581, 1550, 1519, 1488, 1457, 1426, 1395, 1364, 1333, 1302, 1271, 1240, 1209, 1178, 1147, 1116, 1085, 1054, 1023, 2016, 1985, 1954, 1923, 1892, 1861, 1830, 1799, 1768, 1737, 1706, 1675, 1644, 1613, 1582, 1551, 1520, 1489, 1458, 1427, 1396, 1365, 1334, 1303, 1272, 1241, 1210,
  1179, 1148, 1117, 1086, 1055, 2017, 1986, 1955, 1924, 1893, 1862, 1831, 1800, 1769, 1738, 1707, 1676, 1645, 1614, 1583, 1552, 1521, 1490, 1459, 1428, 1397, 1366, 1335, 1304, 1273, 1242, 1211, 1180, 1149, 1118, 1087, 2018, 1987, 1956, 1925, 1894, 1863, 1832, 1801, 1770, 1739, 1708, 1677, 1646, 1615,
  1584, 1553, 1522, 1491, 1460, 1429, 1398, 1367, 1336, 1305, 1274, 1243, 1212, 1181, 1150, 1119, 2019, 1988, 1957, 1926, 1895, 1864, 1833, 1802, 1771, 1740, 1709, 1678, 1647, 1616, 1585, 1554, 1523, 1492, 1461, 1430, 1399, 1368, 1337, 1306, 1275, 1244, 1213, 1182, 1151, 2020, 1989, 1958, 1927, 1896,
  1865, 1834, 1803, 1772, 1741, 1710, 1679, 1648, 1617, 1586, 1555, 1524, 1493, 1462, 1431, 1400, 1369, 1338, 1307, 1276, 1245, 1214, 1183, 2021, 1990, 1959, 1928, 1897, 1866, 1835, 1804, 1773, 1742, 1711, 1680, 1649, 1618, 1587, 1556, 1525, 1494, 1463, 1432, 1401, 1370, 1339, 1308, 1277, 1246, 1215,
  2022, 1991, 1960, 1929, 1898, 1867, 1836, 1805, 1774, 1743, 1712, 1681, 1650, 1619, 1588, 1557, 1526, 1495, 1464, 1433, 1402, 1371, 1340, 1309, 1278, 1247, 2023, 1992, 1961, 1930, 1899, 1868, 1837, 1806, 1775, 1744, 1713, 1682, 1651, 1620, 1589, 1558, 1527, 1496, 1465, 1434, 1403, 1372, 1341, 1310,
  1279, 2024, 1993, 1962, 1931, 1900, 1869, 1838, 1807, 1776, 1745, 1714, 1683, 1652, 1621, 1590, 1559, 1528, 1497, 1466, 1435, 1404, 1373, 1342, 1311, 2025, 1994, 1963, 1932, 1901, 1870, 1839, 1808, 1777, 1746, 1715, 1684, 1653, 1622, 1591, 1560, 1529, 1498, 1467, 1436, 1405, 1374, 1343, 2026, 1995,
  1964, 1933, 1902, 1871, 1840, 1809, 1778, 1747, 1716, 1685, 1654, 1623, 1592, 1561, 1530, 1499, 1468, 1437, 1406, 1375, 2027, 1996, 1965, 1934, 1903, 1872, 1841, 1810, 1779, 1748, 1717, 1686, 1655, 1624, 1593, 1562, 1531, 1500, 1469, 1438, 1407, 2028, 1997, 1966, 1935, 1904, 1873, 1842, 1811, 1780,
  1749, 1718, 1687, 1656, 1625, 1594, 1563, 1532, 1501, 1470, 1439, 2029, 1998, 1967, 1936, 1905, 1874, 1843, 1812, 1781, 1750, 1719, 1688, 1657, 1626, 1595, 1564, 1533, 1502, 1471, 2030, 1999, 1968, 1937, 1906, 1875, 1844, 1813, 1782, 1751, 1720, 1689, 1658, 1627, 1596, 1565, 1534, 1503, 2031, 2000,
  1969, 1938, 1907, 1876, 1845, 1814, 1783, 1752, 1721, 1690, 1659, 1628, 1597, 1566, 1535, 2032, 2001, 1970, 1939, 1908, 1877, 1846, 1815, 1784, 1753, 1722, 1691, 1660, 1629, 1598, 1567, 2033, 2002, 1971, 1940, 1909, 1878, 1847, 1816, 1785, 1754, 1723, 1692, 1661, 1630, 1599, 2034, 2003, 1972, 1941,
  1910, 1879, 1848, 1817, 1786, 1755, 1724, 1693, 1662, 1631, 2035, 2004, 1973, 1942, 1911, 1880, 1849, 1818, 1787, 1756, 1725, 1694, 1663, 2036, 2005, 1974, 1943, 1912, 1881, 1850, 1819, 1788, 1757, 1726, 1695, 2037, 2006, 1975, 1944, 1913, 1882, 1851, 1820, 1789, 1758, 1727, 2038, 2007, 1976, 1945,
  1914, 1883, 1852, 1821, 1790, 1759, 2039, 2008, 1977, 1946, 1915, 1884, 1853, 1822, 1791, 2040, 2009, 1978, 1947, 1916, 1885, 1854, 1823, 2041, 2010, 1979, 1948, 1917, 1886, 1855, 2042, 2011, 1980, 1949, 1918, 1887, 2043, 2012, 1981, 1950, 1919, 2044, 2013, 1982, 1951, 2045, 2014, 1983, 2046, 2015,
  2047, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48,
  49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 0, 64, 1, 65, 2, 66, 3, 67, 4, 68, 5, 69, 6, 70, 7, 71, 8, 72, 9, 73, 10, 74, 11, 75, 12, 76, 13, 77, 14, 78, 15, 79, 16, 80, 17,
  81, 18, 82, 19, 83, 20, 84, 21, 85, 22, 86, 23, 87, 24, 88, 25, 89, 26, 90, 27, 91, 28, 92, 29, 93, 30, 94, 31, 95, 32, 96, 33, 97, 34, 98, 35, 99, 36, 100, 37, 101, 38, 102, 39, 103, 40, 104, 41, 105, 42,
  106, 43, 107, 44, 108, 45, 109, 46, 110, 47, 111, 48, 112, 49, 113, 50, 114, 51, 115, 52, 116, 53, 117, 54, 118, 55, 119, 56, 120, 57, 121, 58, 122, 59, 123, 60, 124, 61, 125, 62, 126, 63, 127, 0, 64, 1, 128, 65, 2, 192,
  129, 66, 3, 193, 130, 67, 4, 194, 131, 68, 5, 195, 132, 69, 6, 196, 133, 70, 7, 197, 134, 71, 8, 198, 135, 72, 9, 199, 136, 73, 10, 200, 137, 74, 11, 201, 138, 75, 12, 202, 139, 76, 13, 203, 140, 77, 14, 204, 141, 78,
  15, 205, 142, 79, 16, 206, 143, 80, 17, 207, 144, 81, 18, 208, 145, 82, 19, 209, 146, 83, 20, 210, 147, 84, 21, 211, 148, 85, 22, 212, 149, 86, 23, 213, 150, 87, 24, 214, 151, 88, 25, 215, 152, 89, 26, 216, 153, 90, 27, 217,
  154, 91, 28, 218, 155, 92, 29, 219, 156, 93, 30, 220, 157, 94, 31, 221, 158, 95, 32, 222, 159, 96, 33, 223, 160, 97, 34, 224, 161, 98, 35, 225, 162, 99, 36, 226, 163, 100, 37, 227, 164, 101, 38, 228, 165, 102, 39, 229, 166, 103,
  40, 230, 167, 104, 41, 231, 168, 105, 42, 232, 169, 106, 43, 233, 170, 107, 44, 234, 171, 108, 45, 235, 172, 109, 46, 236, 173, 110, 47, 237, 174, 111, 48, 238, 175, 112, 49, 239, 176, 113, 50, 240, 177, 114, 51, 241, 178, 115, 52, 242,
  179, 116, 53, 243, 180, 117, 54, 244, 181, 118, 55, 245, 182, 119, 56, 246, 183, 120, 57, 247, 184, 121, 58, 248, 185, 122, 59, 249, 186, 123, 60, 250, 187, 124, 61, 251, 188, 125, 62, 252, 189, 126, 63, 253, 190, 127, 254, 191, 255, 0,
  64, 1, 128, 65, 2, 192, 129, 66, 3, 256, 193, 130, 67, 4, 320, 257, 194, 131, 68, 5, 384, 321, 258, 195, 132, 69, 6, 448, 385, 322, 259, 196, 133, 70, 7, 449, 386, 323, 260, 197, 134, 71, 8, 450, 387, 324, 261, 198, 135, 72,
  9, 451, 388, 325, 262, 199, 136, 73, 10, 452, 389, 326, 263, 200, 137, 74, 11, 453, 390, 327, 264, 201, 138, 75, 12, 454, 391, 328, 265, 202, 139, 76, 13, 455, 392, 329, 266, 203, 140, 77, 14, 456, 393, 330, 267, 204, 141, 78, 15, 457,
  394, 331, 268, 205, 142, 79, 16, 458, 395, 332, 269, 206, 143, 80, 17, 459, 396, 333, 270, 207, 144, 81, 18, 460, 397, 334, 271, 208, 145, 82, 19, 461, 398, 335, 272, 209, 146, 83, 20, 462, 399, 336, 273, 210, 147, 84, 21, 463, 400, 337,
  274, 211, 148, 85, 22, 464, 401, 338, 275, 212, 149, 86, 23, 465, 402, 339, 276, 213, 150, 87, 24, 466, 403, 340, 277, 214, 151, 88, 25, 467, 404, 341, 278, 215, 152, 89, 26, 468, 405, 342, 279, 216, 153, 90, 27, 469, 406, 343, 280, 217,
  154, 91, 28, 470, 407, 344, 281, 218, 155, 92, 29, 471, 408, 345, 282, 219, 156, 93, 30, 472, 409, 346, 283, 220, 157, 94, 31, 473, 410, 347, 284, 221, 158, 95, 32, 474, 411, 348, 285, 222, 159, 96, 33, 475, 412, 349, 286, 223, 160, 97,
  34, 476, 413, 350, 287, 224, 161, 98, 35, 477, 414, 351, 288, 225, 162, 99, 36, 478, 415, 352, 289, 226, 163, 100, 37, 479, 416, 353, 290, 227, 164, 101, 38, 480, 417, 354, 291, 228, 165, 102, 39, 481, 418, 355, 292, 229, 166, 103, 40, 482,
  419, 356, 293, 230, 167, 104, 41, 483, 420, 357, 294, 231, 168, 105, 42, 484, 421, 358, 295, 232, 169, 106, 43, 485, 422, 359, 296, 233, 170, 107, 44, 486, 423, 360, 297, 234, 171, 108, 45, 487, 424, 361, 298, 235, 172, 109, 46, 488, 425, 362,
  299, 236, 173, 110, 47, 489, 426, 363, 300, 237, 174, 111, 48, 490, 427, 364, 301, 238, 175, 112, 49, 491, 428, 365, 302, 239, 176, 113, 50, 492, 429, 366, 303, 240, 177, 114, 51, 493, 430, 367, 304, 241, 178, 115, 52, 494, 431, 368, 305, 242,
  179, 116, 53, 495, 432, 369, 306, 243, 180, 117, 54, 496, 433, 370, 307, 244, 181, 118, 55, 497, 434, 371, 308, 245, 182, 119, 56, 498, 435, 372, 309, 246, 183, 120, 57, 499, 436, 373, 310, 247, 184, 121, 58, 500, 437, 374, 311, 248, 185, 122,
  59, 501, 438, 375, 312, 249, 186, 123, 60, 502, 439, 376, 313, 250, 187, 124, 61, 503, 440, 377, 314, 251, 188, 125, 62, 504, 441, 378, 315, 252, 189, 126, 63, 505, 442, 379, 316, 253, 190, 127, 506, 443, 380, 317, 254, 191, 507, 444, 381, 318,
  255, 508, 445, 382, 319, 509, 446, 383, 510, 447, 511, 0, 64, 1, 128, 65, 2, 192, 129, 66, 3, 256, 193, 130, 67, 4, 320, 257, 194, 131, 68, 5, 384, 321, 258, 195, 132, 69, 6, 448, 385, 322, 259, 196, 133, 70, 7, 512, 449, 386,
  323, 260, 197, 134, 71, 8, 576, 513, 450, 387, 324, 261, 198, 135, 72, 9, 640, 577, 514, 451, 388, 325, 262, 199, 136, 73, 10, 704, 641, 578, 515, 452, 389, 326, 263, 200, 137, 74, 11, 768, 705, 642, 579, 516, 453, 390, 327, 264, 201, 138,
  75, 12, 832, 769, 706, 643, 580, 517, 454, 391, 328, 265, 202, 139, 76, 13, 896, 833, 770, 707, 644, 581, 518, 455, 392, 329, 266, 203, 140, 77, 14, 960, 897, 834, 771, 708, 645, 582, 519, 456, 393, 330, 267, 204, 141, 78, 15, 961, 898, 835,
  772, 709, 646, 583, 520, 457, 394, 331, 268, 205, 142, 79, 16, 962, 899, 836, 773, 710, 647, 584, 521, 458, 395, 332, 269, 206, 143, 80, 17, 963, 900, 837, 774, 711, 648, 585, 522, 459, 396, 333, 270, 207, 144, 81, 18, 964, 901, 838, 775, 712,
  649, 586, 523, 460, 397, 334, 271, 208, 145, 82, 19, 965, 902, 839, 776, 713, 650, 587, 524, 461, 398, 335, 272, 209, 146, 83, 20, 966, 903, 840, 777, 714, 651, 588, 525, 462, 399, 336, 273, 210, 147, 84, 21, 967, 904, 841, 778, 715, 652, 589,
  526, 463, 400, 337, 274, 211, 148, 85, 22, 968, 905, 842, 779, 716, 653, 590, 527, 464, 401, 338, 275, 212, 149, 86, 23, 969, 906, 843, 780, 717, 654, 591, 528, 465, 402, 339, 276, 213, 150, 87, 24, 970, 907, 844, 781, 718, 655, 592, 529, 466,
  403, 340, 277, 214, 151, 88, 25, 971, 908, 845, 782, 719, 656, 593, 530, 467, 404, 341, 278, 215, 152, 89, 26, 972, 909, 846, 783, 720, 657, 594, 531, 468, 405, 342, 279, 216, 153, 90, 27, 973, 910, 847, 784, 721, 658, 595, 532, 469, 406, 343,
  280, 217, 154, 91, 28, 974, 911, 848, 785, 722, 659, 596, 533, 470, 407, 344, 281, 218, 155, 92, 29, 975, 912, 849, 786, 723, 660, 597, 534, 471, 408, 345, 282, 219, 156, 93, 30, 976, 913, 850, 787, 724, 661, 598, 535, 472, 409, 346, 283, 220,
  157, 94, 31, 977, 914, 851, 788, 725, 662, 599, 536, 473, 410, 347, 284, 221, 158, 95, 32, 978, 915, 852, 789, 726, 663, 600, 537, 474, 411, 348, 285, 222, 159, 96, 33, 979, 916, 853, 790, 727, 664, 601, 538, 475, 412, 349, 286, 223, 160, 97,
  34, 980, 917, 854, 791, 728, 665, 602, 539, 476, 413, 350, 287, 224, 161, 98, 35, 981, 918, 855, 792, 729, 666, 603, 540, 477, 414, 351, 288, 225, 162, 99, 36, 982, 919, 856, 793, 730, 667, 604, 541, 478, 415, 352, 289, 226, 163, 100, 37, 983,
  920, 857, 794, 731, 668, 605, 542, 479, 416, 353, 290, 227, 164, 101, 38, 984, 921, 858, 795, 732, 669, 606, 543, 480, 417, 354, 291, 228, 165, 102, 39, 985, 922, 859, 796, 733, 670, 607, 544, 481, 418, 355, 292, 229, 166, 103, 40, 986, 923, 860,
  797, 734, 671, 608, 545, 482, 419, 356, 293, 230, 167, 104, 41, 987, 924, 861, 798, 735, 672, 609, 546, 483, 420, 357, 294, 231, 168, 105, 42, 988, 925, 862, 799, 736, 673, 610, 547, 484, 421, 358, 295, 232, 169, 106, 43, 989, 926, 863, 800, 737,
  674, 611, 548, 485, 422, 359, 296, 233, 170, 107, 44, 990, 927, 864, 801, 738, 675, 612, 549, 486, 423, 360, 297, 234, 171, 108, 45, 991, 928, 865, 802, 739, 676, 613, 550, 487, 424, 361, 298, 235, 172, 109, 46, 992, 929, 866, 803, 740, 677, 614,
  551, 488, 425, 362, 299, 236, 173, 110, 47, 993, 930, 867, 804, 741, 678, 615, 552, 489, 426, 363, 300, 237, 174, 111, 48, 994, 931, 868, 805, 742, 679, 616, 553, 490, 427, 364, 301, 238, 175, 112, 49, 995, 932, 869, 806, 743, 680, 617, 554, 491,
  428, 365, 302, 239, 176, 113, 50, 996, 933, 870, 807, 744, 681, 618, 555, 492, 429, 366, 303, 240, 177, 114, 51, 997, 934, 871, 808, 745, 682, 619, 556, 493, 430, 367, 304, 241, 178, 115, 52, 998, 935, 872, 809, 746, 683, 620, 557, 494, 431, 368,
  305, 242, 179, 116, 53, 999, 936, 873, 810, 747, 684, 621, 558, 495, 432, 369, 306, 243, 180, 117, 54, 1000, 937, 874, 811, 748, 685, 622, 559, 496, 433, 370, 307, 244, 181, 118, 55, 1001, 938, 875, 812, 749, 686, 623, 560, 497, 434, 371, 308, 245,
  182, 119, 56, 1002, 939, 876, 813, 750, 687, 624, 561, 498, 435, 372, 309, 246, 183, 120, 57, 1003, 940, 877, 814, 751, 688, 625, 562, 499, 436, 373, 310, 247, 184, 121, 58, 1004, 941, 878, 815, 752, 689, 626, 563, 500, 437, 374, 311, 248, 185, 122,
  59, 1005, 942, 879, 816, 753, 690, 627, 564, 501, 438, 375, 312, 249, 186, 123, 60, 1006, 943, 880, 817, 754, 691, 628, 565, 502, 439, 376, 313, 250, 187, 124, 61, 1007, 944, 881, 818, 755, 692, 629, 566, 503, 440, 377, 314, 251, 188, 125, 62, 1008,
  945, 882, 819, 756, 693, 630, 567, 504, 441, 378, 315, 252, 189, 126, 63, 1009, 946, 883, 820, 757, 694, 631, 568, 505, 442, 379, 316, 253, 190, 127, 1010, 947, 884, 821, 758, 695, 632, 569, 506, 443, 380, 317, 254, 191, 1011, 948, 885, 822, 759, 696,
  633, 570, 507, 444, 381, 318, 255, 1012, 949, 886, 823, 760, 697, 634, 571, 508, 445, 382, 319, 1013, 950, 887, 824, 761, 698, 635, 572, 509, 446, 383, 1014, 951, 888, 825, 762, 699, 636, 573, 510, 447, 1015, 952, 889, 826, 763, 700, 637, 574, 511, 1016,
  953, 890, 827, 764, 701, 638, 575, 1017, 954, 891, 828, 765, 702, 639, 1018, 955, 892, 829, 766, 703, 1019, 956, 893, 830, 767, 1020, 957, 894, 831, 1021, 958, 895, 1022, 959, 1023, 0, 64, 1, 128, 65, 2, 192, 129, 66, 3, 256, 193, 130, 67, 4,
  320, 257, 194, 131, 68, 5, 384, 321, 258, 195, 132, 69, 6, 448, 385, 322, 259, 196, 133, 70, 7, 512, 449, 386, 323, 260, 197, 134, 71, 8, 576, 513, 450, 387, 324, 261, 198, 135, 72, 9, 640, 577, 514, 451, 388, 325, 262, 199, 136, 73,
  10, 704, 641, 578, 515, 452, 389, 326, 263, 200, 137, 74, 11, 768, 705, 642, 579, 516, 453, 390, 327, 264, 201, 138, 75, 12, 832, 769, 706, 643, 580, 517, 454, 391, 328, 265, 202, 139, 76, 13, 896, 833, 770, 707, 644, 581, 518, 455, 392, 329,
  266, 203, 140, 77, 14, 960, 897, 834, 771, 708, 645, 582, 519, 456, 393, 330, 267, 204, 141, 78, 15, 1024, 961, 898, 835, 772, 709, 646, 583, 520, 457, 394, 331, 268, 205, 142, 79, 16, 1088, 1025, 962, 899, 836, 773, 710, 647, 584, 521, 458, 395,
  332, 269, 206, 143, 80, 17, 1152, 1089, 1026, 963, 900, 837, 774, 711, 648, 585, 522, 459, 396, 333, 270, 207, 144, 81, 18, 1216, 1153, 1090, 1027, 964, 901, 838, 775, 712, 649, 586, 523, 460, 397, 334, 271, 208, 145, 82, 19, 1280, 1217, 1154, 1091, 1028,
  965, 902, 839, 776, 713, 650, 587, 524, 461, 398, 335, 272, 209, 146, 83, 20, 1344, 1281, 1218, 1155, 1092, 1029, 966, 903, 840, 777, 714, 651, 588, 525, 462, 399, 336, 273, 210, 147, 84, 21, 1408, 1345, 1282, 1219, 1156, 1093, 1030, 967, 904, 841, 778, 715,
  652, 589, 526, 463, 400, 337, 274, 211, 148, 85, 22, 1472, 1409, 1346, 1283, 1220, 1157, 1094, 1031, 968, 905, 842, 779, 716, 653, 590, 527, 464, 401, 338, 275, 212, 149, 86, 23, 1536, 1473, 1410, 1347, 1284, 1221, 1158, 1095, 1032, 969, 906, 843, 780, 717, 654,
  591, 528, 465, 402, 339, 276, 213, 150, 87, 24, 1600, 1537, 1474, 1411, 1348, 1285, 1222, 1159, 1096, 1033, 970, 907, 844, 781, 718, 655, 592, 529, 466, 403, 340, 277, 214, 151, 88, 25, 1664, 1601, 1538, 1475, 1412, 1349, 1286, 1223, 1160, 1097, 1034, 971, 908, 845,
  782, 719, 656, 593, 530, 467, 404, 341, 278, 215, 152, 89, 26, 1728, 1665, 1602, 1539, 1476, 1413, 1350, 1287, 1224, 1161, 1098, 1035, 972, 909, 846, 783, 720, 657, 594, 531, 468, 405, 342, 279, 216, 153, 90, 27, 1792, 1729, 1666, 1603, 1540, 1477, 1414, 1351, 1288,
  1225, 1162, 1099, 1036, 973, 910, 847, 784, 721, 658, 595, 532, 469, 406, 343, 280, 217, 154, 91, 28, 1856, 1793, 1730, 1667, 1604, 1541, 1478, 1415, 1352, 1289, 1226, 1163, 1100, 1037, 974, 911, 848, 785, 722, 659, 596, 533, 470, 407, 344, 281, 218, 155, 92, 29,
  1920, 1857, 1794, 1731, 1668, 1605, 1542, 1479, 1416, 1353, 1290, 1227, 1164, 1101, 1038, 975, 912, 849, 786, 723, 660, 597, 534, 471, 408, 345, 282, 219, 156, 93, 30, 1984, 1921, 1858, 1795, 1732, 1669, 1606, 1543, 1480, 1417, 1354, 1291, 1228, 1165, 1102, 1039, 976, 913, 850,
  787, 724, 661, 598, 535, 472, 409, 346, 283, 220, 157, 94, 31, 1985, 1922, 1859, 1796, 1733, 1670, 1607, 1544, 1481, 1418, 1355, 1292, 1229, 1166, 1103, 1040, 977, 914, 851, 788, 725, 662, 599, 536, 473, 410, 347, 284, 221, 158, 95, 32, 1986, 1923, 1860, 1797, 1734,
  1671, 1608, 1545, 1482, 1419, 1356, 1293, 1230, 1167, 1104, 1041, 978, 915, 852, 789, 726, 663, 600, 537, 474, 411, 348, 285, 222, 159, 96, 33, 1987, 1924, 1861, 1798, 1735, 1672, 1609, 1546, 1483, 1420, 1357, 1294, 1231, 1168, 1105, 1042, 979, 916, 853, 790, 727, 664, 601,
  538, 475, 412, 349, 286, 223, 160, 97, 34, 1988, 1925, 1862, 1799, 1736, 1673, 1610, 1547, 1484, 1421, 1358, 1295, 1232, 1169, 1106, 1043, 980, 917, 854, 791, 728, 665, 602, 539, 476, 413, 350, 287, 224, 161, 98, 35, 1989, 1926, 1863, 1800, 1737, 1674, 1611, 1548, 1485,
  1422, 1359, 1296, 1233, 1170, 1107, 1044, 981, 918, 855, 792, 729, 666, 603, 540, 477, 414, 351, 288, 225, 162, 99, 36, 1990, 1927, 1864, 1801, 1738, 1675, 1612, 1549, 1486, 1423, 1360, 1297, 1234, 1171, 1108, 1045, 982, 919, 856, 793, 730, 667, 604, 541, 478, 415, 352,
  289, 226, 163, 100, 37, 1991, 1928, 1865, 1802, 1739, 1676, 1613, 1550, 1487, 1424, 1361, 1298, 1235, 1172, 1109, 1046, 983, 920, 857, 794, 731, 668, 605, 542, 479, 416, 353, 290, 227, 164, 101, 38, 1992, 1929, 1866, 1803, 1740, 1677, 1614, 1551, 1488, 1425, 1362, 1299, 1236,
  1173, 1110, 1047, 984, 921, 858, 795, 732, 669, 606, 543, 480, 417, 354, 291, 228, 165, 102, 39, 1993, 1930, 1867, 1804, 1741, 1678, 1615, 1552, 1489, 1426, 1363, 1300, 1237, 1174, 1111, 1048, 985, 922, 859, 796, 733, 670, 607, 544, 481, 418, 355, 292, 229, 166, 103,
  40, 1994, 1931, 1868, 1805, 1742, 1679, 1616, 1553, 1490, 1427, 1364, 1301, 1238, 1175, 1112, 1049, 986, 923, 860, 797, 734, 671, 608, 545, 482, 419, 356, 293, 230, 167, 104, 41, 1995, 1932, 1869, 1806, 1743, 1680, 1617, 1554, 1491, 1428, 1365, 1302, 1239, 1176, 1113, 1050, 987,
  924, 861, 798, 735, 672, 609, 546, 483, 420, 357, 294, 231, 168, 105, 42, 1996, 1933, 1870, 1807, 1744, 1681, 1618, 1555, 1492, 1429, 1366, 1303, 1240, 1177, 1114, 1051, 988, 925, 862, 799, 736, 673, 610, 547, 484, 421, 358, 295, 232, 169, 106, 43, 1997, 1934, 1871,
  1808, 1745, 1682, 1619, 1556, 1493, 1430, 1367, 1304, 1241, 1178, 1115, 1052, 989, 926, 863, 800, 737, 674, 611, 548, 485, 422, 359, 296, 233, 170, 107, 44, 1998, 1935, 1872, 1809, 1746, 1683, 1620, 1557, 1494, 1431, 1368, 1305, 1242, 1179, 1116, 1053, 990, 927, 864, 801, 738,
  675, 612, 549, 486, 423, 360, 297, 234, 171, 108, 45, 1999, 1936, 1873, 1810, 1747, 1684, 1621, 1558, 1495, 1432, 1369, 1306, 1243, 1180, 1117, 1054, 991, 928, 865, 802, 739, 676, 613, 550, 487, 424, 361, 298, 235, 172, 109, 46, 2000, 1937, 1874, 1811, 1748, 1685, 1622,
  1559, 1496, 1433, 1370, 1307, 1244, 1181, 1118, 1055, 992, 929, 866, 803, 740, 677, 614, 551, 488, 425, 362, 299, 236, 173, 110, 47, 2001, 1938, 1875, 1812, 1749, 1686, 1623, 1560, 1497, 1434, 1371, 1308, 1245, 1182, 1119, 1056, 993, 930, 867, 804, 741, 678, 615, 552, 489,
  426, 363, 300, 237, 174, 111, 48, 2002, 1939, 1876, 1813, 1750, 1687, 1624, 1561, 1498, 1435, 1372, 1309, 1246, 1183, 1120, 1057, 994, 931, 868, 805, 742, 679, 616, 553, 490, 427, 364, 301, 238, 175, 112, 49, 2003, 1940, 1877, 1814, 1751, 1688, 1625, 1562, 1499, 1436, 1373,
  1310, 1247, 1184, 1121, 1058, 995, 932, 869, 806, 743, 680, 617, 554, 491, 428, 365, 302, 239, 176, 113, 50, 2004, 1941, 1878, 1815, 1752, 1689, 1626, 1563, 1500, 1437, 1374, 1311, 1248, 1185, 1122, 1059, 996, 933, 870, 807, 744, 681, 618, 555, 492, 429, 366, 303, 240,
  177, 114, 51, 2005, 1942, 1879, 1816, 1753, 1690, 1627, 1564, 1501, 1438, 1375, 1312, 1249, 1186, 1123, 1060, 997, 934, 871, 808, 745, 682, 619, 556, 493, 430, 367, 304, 241, 178, 115, 52, 2006, 1943, 1880, 1817, 1754, 1691, 1628, 1565, 1502, 1439, 1376, 1313, 1250, 1187, 1124,
  1061, 998, 935, 872, 809, 746, 683, 620, 557, 494, 431, 368, 305, 242, 179, 116, 53, 2007, 1944, 1881, 1818, 1755, 1692, 1629, 1566, 1503, 1440, 1377, 1314, 1251, 1188, 1125, 1062, 999, 936, 873, 810, 747, 684, 621, 558, 495, 432, 369, 306, 243, 180, 117, 54, 2008,
  1945, 1882, 1819, 1756, 1693, 1630, 1567, 1504, 1441, 1378, 1315, 1252, 1189, 1126, 1063, 1000, 937, 874, 811, 748, 685, 622, 559, 496, 433, 370, 307, 244, 181, 118, 55, 2009, 1946, 1883, 1820, 1757, 1694, 1631, 1568, 1505, 1442, 1379, 1316, 1253, 1190, 1127, 1064, 1001, 938, 875,
  812, 749, 686, 623, 560, 497, 434, 371, 308, 245, 182, 119, 56, 2010, 1947, 1884, 1821, 1758, 1695, 1632, 1569, 1506, 1443, 1380, 1317, 1254, 1191, 1128, 1065, 1002, 939, 876, 813, 750, 687, 624, 561, 498, 435, 372, 309, 246, 183, 120, 57, 2011, 1948, 1885, 1822, 1759,
  1696, 1633, 1570, 1507, 1444, 1381, 1318, 1255, 1192, 1129, 1066, 1003, 940, 877, 814, 751, 688, 625, 562, 499, 436, 373, 310, 247, 184, 121, 58, 2012, 1949, 1886, 1823, 1760, 1697, 1634, 1571, 1508, 1445, 1382, 1319, 1256, 1193, 1130, 1067, 1004, 941, 878, 815, 752, 689, 626,
  563, 500, 437, 374, 311, 248, 185, 122, 59, 2013, 1950, 1887, 1824, 1761, 1698, 1635, 1572, 1509, 1446, 1383, 1320, 1257, 1194, 1131, 1068, 1005, 942, 879, 816, 753, 690, 627, 564, 501, 438, 375, 312, 249, 186, 123, 60, 2014, 1951, 1888, 1825, 1762, 1699, 1636, 1573, 1510,
  1447, 1384, 1321, 1258, 1195, 1132, 1069, 1006, 943, 880, 817, 754, 691, 628, 565, 502, 439, 376, 313, 250, 187, 124, 61, 2015, 1952, 1889, 1826, 1763, 1700, 1637, 1574, 1511, 1448, 1385, 1322, 1259, 1196, 1133, 1070, 1007, 944, 881, 818, 755, 692, 629, 566, 503, 440, 377,
  314, 251, 188, 125, 62, 2016, 1953, 1890, 1827, 1764, 1701, 1638, 1575, 1512, 1449, 1386, 1323, 1260, 1197, 1134, 1071, 1008, 945, 882, 819, 756, 693, 630, 567, 504, 441, 378, 315, 252, 189, 126, 63, 2017, 1954, 1891, 1828, 1765, 1702, 1639, 1576, 1513, 1450, 1387, 1324, 1261,
  1198, 1135, 1072, 1009, 946, 883, 820, 757, 694, 631, 568, 505, 442, 379, 316, 253, 190, 127, 2018, 1955, 1892, 1829, 1766, 1703, 1640, 1577, 1514, 1451, 1388, 1325, 1262, 1199, 1136, 1073, 1010, 947, 884, 821, 758, 695, 632, 569, 506, 443, 380, 317, 254, 191, 2019, 1956,
  1893, 1830, 1767, 1704, 1641, 1578, 1515, 1452, 1389, 1326, 1263, 1200, 1137, 1074, 1011, 948, 885, 822, 759, 696, 633, 570, 507, 444, 381, 318, 255, 2020, 1957, 1894, 1831, 1768, 1705, 1642, 1579, 1516, 1453, 1390, 1327, 1264, 1201, 1138, 1075, 1012, 949, 886, 823, 760, 697, 634,
  571, 508, 445, 382, 319, 2021, 1958, 1895, 1832, 1769, 1706, 1643, 1580, 1517, 1454, 1391, 1328, 1265, 1202, 1139, 1076, 1013, 950, 887, 824, 761, 698, 635, 572, 509, 446, 383, 2022, 1959, 1896, 1833, 1770, 1707, 1644, 1581, 1518, 1455, 1392, 1329, 1266, 1203, 1140, 1077, 1014, 951,
  888, 825, 762, 699, 636, 573, 510, 447, 2023, 1960, 1897, 1834, 1771, 1708, 1645, 1582, 1519, 1456, 1393, 1330, 1267, 1204, 1141, 1078, 1015, 952, 889, 826, 763, 700, 637, 574, 511, 2024, 1961, 1898, 1835, 1772, 1709, 1646, 1583, 1520, 1457, 1394, 1331, 1268, 1205, 1142, 1079, 1016,
  953, 890, 827, 764, 701, 638, 575, 2025, 1962, 1899, 1836, 1773, 1710, 1647, 1584, 1521, 1458, 1395, 1332, 1269, 1206, 1143, 1080, 1017, 954, 891, 828, 765, 702, 639, 2026, 1963, 1900, 1837, 1774, 1711, 1648, 1585, 1522, 1459, 1396, 1333, 1270, 1207, 1144, 1081, 1018, 955, 892, 829,
  766, 703, 2027, 1964, 1901, 1838, 1775, 1712, 1649, 1586, 1523, 1460, 1397, 1334, 1271, 1208, 1145, 1082, 1019, 956, 893, 830, 767, 2028, 1965, 1902, 1839, 1776, 1713, 1650, 1587, 1524, 1461, 1398, 1335, 1272, 1209, 1146, 1083, 1020, 957, 894, 831, 2029, 1966, 1903, 1840, 1777, 1714, 1651,
  1588, 1525, 1462, 1399, 1336, 1273, 1210, 1147, 1084, 1021, 958, 895, 2030, 1967, 1904, 1841, 1778, 1715, 1652, 1589, 1526, 1463, 1400, 1337, 1274, 1211, 1148, 1085, 1022, 959, 2031, 1968, 1905, 1842, 1779, 1716, 1653, 1590, 1527, 1464, 1401, 1338, 1275, 1212, 1149, 1086, 1023, 2032, 1969, 1906,
  1843, 1780, 1717, 1654, 1591, 1528, 1465, 1402, 1339, 1276, 1213, 1150, 1087, 2033, 1970, 1907, 1844, 1781, 1718, 1655, 1592, 1529, 1466, 1403, 1340, 1277, 1214, 1151, 2034, 1971, 1908, 1845, 1782, 1719, 1656, 1593, 1530, 1467, 1404, 1341, 1278, 1215, 2035, 1972, 1909, 1846, 1783, 1720, 1657, 1594,
  1531, 1468, 1405, 1342, 1279, 2036, 1973, 1910, 1847, 1784, 1721, 1658, 1595, 1532, 1469, 1406, 1343, 2037, 1974, 1911, 1848, 1785, 1722, 1659, 1596, 1533, 1470, 1407, 2038, 1975, 1912, 1849, 1786, 1723, 1660, 1597, 1534, 1471, 2039, 1976, 1913, 1850, 1787, 1724, 1661, 1598, 1535, 2040, 1977, 1914,
  1851, 1788, 1725, 1662, 1599, 2041, 1978, 1915, 1852, 1789, 1726, 1663, 2042, 1979, 1916, 1853, 1790, 1727, 2043, 1980, 1917, 1854, 1791, 2044, 1981, 1918, 1855, 2045, 1982, 1919, 2046, 1983, 2047, 0, 64, 1, 128, 65, 2, 192, 129, 66, 3, 256, 193, 130, 67, 4, 320, 257,
  194, 131, 68, 5, 384, 321, 258, 195, 132, 69, 6, 448, 385, 322, 259, 196, 133, 70, 7, 512, 449, 386, 323, 260, 197, 134, 71, 8, 576, 513, 450, 387, 324, 261, 198, 135, 72, 9, 640, 577, 514, 451, 388, 325, 262, 199, 136, 73, 10, 704,
  641, 578, 515, 452, 389, 326, 263, 200, 137, 74, 11, 768, 705, 642, 579, 516, 453, 390, 327, 264, 201, 138, 75, 12, 832, 769, 706, 643, 580, 517, 454, 391, 328, 265, 202, 139, 76, 13, 896, 833, 770, 707, 644, 581, 518, 455, 392, 329, 266, 203,
  140, 77, 14, 960, 897, 834, 771, 708, 645, 582, 519, 456, 393, 330, 267, 204, 141, 78, 15, 1024, 961, 898, 835, 772, 709, 646, 583, 520, 457, 394, 331, 268, 205, 142, 79, 16, 1088, 1025, 962, 899, 836, 773, 710, 647, 584, 521, 458, 395, 332, 269,
  206, 143, 80, 17, 1152, 1089, 1026, 963, 900, 837, 774, 711, 648, 585, 522, 459, 396, 333, 270, 207, 144, 81, 18, 1216, 1153, 1090, 1027, 964, 901, 838, 775, 712, 649, 586, 523, 460, 397, 334, 271, 208, 145, 82, 19, 1280, 1217, 1154, 1091, 1028, 965, 902,
  839, 776, 713, 650, 587, 524, 461, 398, 335, 272, 209, 146, 83, 20, 1344, 1281, 1218, 1155, 1092, 1029, 966, 903, 840, 777, 714, 651, 588, 525, 462, 399, 336, 273, 210, 147, 84, 21, 1408, 1345, 1282, 1219, 1156, 1093, 1030, 967, 904, 841, 778, 715, 652, 589,
  526, 463, 400, 337, 274, 211, 148, 85, 22, 1472, 1409, 1346, 1283, 1220, 1157, 1094, 1031, 968, 905, 842, 779, 716, 653, 590, 527, 464, 401, 338, 275, 212, 149, 86, 23, 1536, 1473, 1410, 1347, 1284, 1221, 1158, 1095, 1032, 969, 906, 843, 780, 717, 654, 591, 528,
  465, 402, 339, 276, 213, 150, 87, 24, 1600, 1537, 1474, 1411, 1348, 1285, 1222, 1159, 1096, 1033, 970, 907, 844, 781, 718, 655, 592, 529, 466, 403, 340, 277, 214, 151, 88, 25, 1664, 1601, 1538, 1475, 1412, 1349, 1286, 1223, 1160, 1097, 1034, 971, 908, 845, 782, 719,
  656, 593, 530, 467, 404, 341, 278, 215, 152, 89, 26, 1728, 1665, 1602, 1539, 1476, 1413, 1350, 1287, 1224, 1161, 1098, 1035, 972, 909, 846, 783, 720, 657, 594, 531, 468, 405, 342, 279, 216, 153, 90, 27, 1792, 1729, 1666, 1603, 1540, 1477, 1414, 1351, 1288, 1225, 1162,
  1099, 1036, 973, 910, 847, 784, 721, 658, 595, 532, 469, 406, 343, 280, 217, 154, 91, 28, 1856, 1793, 1730, 1667, 1604, 1541, 1478, 1415, 1352, 1289, 1226, 1163, 1100, 1037, 974, 911, 848, 785, 722, 659, 596, 533, 470, 407, 344, 281, 218, 155, 92, 29, 1920, 1857,
  1794, 1731, 1668, 1605, 1542, 1479, 1416, 1353, 1290, 1227, 1164, 1101, 1038, 975, 912, 849, 786, 723, 660, 597, 534, 471, 408, 345, 282, 219, 156, 93, 30, 1984, 1921, 1858, 1795, 1732, 1669, 1606, 1543, 1480, 1417, 1354, 1291, 1228, 1165, 1102, 1039, 976, 913, 850, 787, 724,
  661, 598, 535, 472, 409, 346, 283, 220, 157, 94, 31, 2048, 1985, 1922, 1859, 1796, 1733, 1670, 1607, 1544, 1481, 1418, 1355, 1292, 1229, 1166, 1103, 1040, 977, 914, 851, 788, 725, 662, 599, 536, 473, 410, 347, 284, 221, 158, 95, 32, 2112, 2049, 1986, 1923, 1860, 1797,
  1734, 1671, 1608, 1545, 1482, 1419, 1356, 1293, 1230, 1167, 1104, 1041, 978, 915, 852, 789, 726, 663, 600, 537, 474, 411, 348, 285, 222, 159, 96, 33, 2176, 2113, 2050, 1987, 1924, 1861, 1798, 1735, 1672, 1609, 1546, 1483, 1420, 1357, 1294, 1231, 1168, 1105, 1042, 979, 916, 853,
  790, 727, 664, 601, 538, 475, 412, 349, 286, 223, 160, 97, 34, 2240, 2177, 2114, 2051, 1988, 1925, 1862, 1799, 1736, 1673, 1610, 1547, 1484, 1421, 1358, 1295, 1232, 1169, 1106, 1043, 980, 917, 854, 791, 728, 665, 602, 539, 476, 413, 350, 287, 224, 161, 98, 35, 2304,
  2241, 2178, 2115, 2052, 1989, 1926, 1863, 1800, 1737, 1674, 1611, 1548, 1485, 1422, 1359, 1296, 1233, 1170, 1107, 1044, 981, 918, 855, 792, 729, 666, 603, 540, 477, 414, 351, 288, 225, 162, 99, 36, 2368, 2305, 2242, 2179, 2116, 2053, 1990, 1927, 1864, 1801, 1738, 1675, 1612, 1549,
  1486, 1423, 1360, 1297, 1234, 1171, 1108, 1045, 982, 919, 856, 793, 730, 667, 604, 541, 478, 415, 352, 289, 226, 163, 100, 37, 2432, 2369, 2306, 2243, 2180, 2117, 2054, 1991, 1928, 1865, 1802, 1739, 1676, 1613, 1550, 1487, 1424, 1361, 1298, 1235, 1172, 1109, 1046, 983, 920, 857,
  794, 731, 668, 605, 542, 479, 416, 353, 290, 227, 164, 101, 38, 2496, 2433, 2370, 2307, 2244, 2181, 2118, 2055, 1992, 1929, 1866, 1803, 1740, 1677, 1614, 1551, 1488, 1425, 1362, 1299, 1236, 1173, 1110, 1047, 984, 921, 858, 795, 732, 669, 606, 543, 480, 417, 354, 291, 228,
  165, 102, 39, 2560, 2497, 2434, 2371, 2308, 2245, 2182, 2119, 2056, 1993, 1930, 1867, 1804, 1741, 1678, 1615, 1552, 1489, 1426, 1363, 1300, 1237, 1174, 1111, 1048, 985, 922, 859, 796, 733, 670, 607, 544, 481, 418, 355, 292, 229, 166, 103, 40, 2624, 2561, 2498, 2435, 2372, 2309,
  2246, 2183, 2120, 2057, 1994, 1931, 1868, 1805, 1742, 1679, 1616, 1553, 1490, 1427, 1364, 1301, 1238, 1175, 1112, 1049, 986, 923, 860, 797, 734, 671, 608, 545, 482, 419, 356, 293, 230, 167, 104, 41, 2688, 2625, 2562, 2499, 2436, 2373, 2310, 2247, 2184, 2121, 2058, 1995, 1932, 1869,
  1806, 1743, 1680, 1617, 1554, 1491, 1428, 1365, 1302, 1239, 1176, 1113, 1050, 987, 924, 861, 798, 735, 672, 609, 546, 483, 420, 357, 294, 231, 168, 105, 42, 2752, 2689, 2626, 2563, 2500, 2437, 2374, 2311, 2248, 2185, 2122, 2059, 1996, 1933, 1870, 1807, 1744, 1681, 1618, 1555, 1492,
  1429, 1366, 1303, 1240, 1177, 1114, 1051, 988, 925, 862, 799, 736, 673, 610, 547, 484, 421, 358, 295, 232, 169, 106, 43, 2816, 2753, 2690, 2627, 2564, 2501, 2438, 2375, 2312, 2249, 2186, 2123, 2060, 1997, 1934, 1871, 1808, 1745, 1682, 1619, 1556, 1493, 1430, 1367, 1304, 1241, 1178,
  1115, 1052, 989, 926, 863, 800, 737, 674, 611, 548, 485, 422, 359, 296, 233, 170, 107, 44, 2880, 2817, 2754, 2691, 2628, 2565, 2502, 2439, 2376, 2313, 2250, 2187, 2124, 2061, 1998, 1935, 1872, 1809, 1746, 1683, 1620, 1557, 1494, 1431, 1368, 1305, 1242, 1179, 1116, 1053, 990, 927,
  864, 801, 738, 675, 612, 549, 486, 423, 360, 297, 234, 171, 108, 45, 2944, 2881, 2818, 2755, 2692, 2629, 2566, 2503, 2440, 2377, 2314, 2251, 2188, 2125, 2062, 1999, 1936, 1873, 1810, 1747, 1684, 1621, 1558, 1495, 1432, 1369, 1306, 1243, 1180, 1117, 1054, 991, 928, 865, 802, 739,
  676, 613, 550, 487, 424, 361, 298, 235, 172, 109, 46, 3008, 2945, 2882, 2819, 2756, 2693, 2630, 2567, 2504, 2441, 2378, 2315, 2252, 2189, 2126, 2063, 2000, 1937, 1874, 1811, 1748, 1685, 1622, 1559, 1496, 1433, 1370, 1307, 1244, 1181, 1118, 1055, 992, 929, 866, 803, 740, 677, 614,
  551, 488, 425, 362, 299, 236, 173, 110, 47, 3072, 3009, 2946, 2883, 2820, 2757, 2694, 2631, 2568, 2505, 2442, 2379, 2316, 2253, 2190, 2127, 2064, 2001, 1938, 1875, 1812, 1749, 1686, 1623, 1560, 1497, 1434, 1371, 1308, 1245, 1182, 1119, 1056, 993, 930, 867, 804, 741, 678, 615, 552,
  489, 426, 363, 300, 237, 174, 111, 48, 3136, 3073, 3010, 2947, 2884, 2821, 2758, 2695, 2632, 2569, 2506, 2443, 2380, 2317, 2254, 2191, 2128, 2065, 2002, 1939, 1876, 1813, 1750, 1687, 1624, 1561, 1498, 1435, 1372, 1309, 1246, 1183, 1120, 1057, 994, 931, 868, 805, 742, 679, 616, 553,
  490, 427, 364, 301, 238, 175, 112, 49, 3200, 3137, 3074, 3011, 2948, 2885, 2822, 2759, 2696, 2633, 2570, 2507, 2444, 2381, 2318, 2255, 2192, 2129, 2066, 2003, 1940, 1877, 1814, 1751, 1688, 1625, 1562, 1499, 1436, 1373, 1310, 1247, 1184, 1121, 1058, 995, 932, 869, 806, 743, 680, 617,
  554, 491, 428, 365, 302, 239, 176, 113, 50, 3264, 3201, 3138, 3075, 3012, 2949, 2886, 2823, 2760, 2697, 2634, 2571, 2508, 2445, 2382, 2319, 2256, 2193, 2130, 2067, 2004, 1941, 1878, 1815, 1752, 1689, 1626, 1563, 1500, 1437, 1374, 1311, 1248, 1185, 1122, 1059, 996, 933, 870, 807, 744,
  681, 618, 555, 492, 429, 366, 303, 240, 177, 114, 51, 3328, 3265, 3202, 3139, 3076, 3013, 2950, 2887, 2824, 2761, 2698, 2635, 2572, 2509, 2446, 2383, 2320, 2257, 2194, 2131, 2068, 2005, 1942, 1879, 1816, 1753, 1690, 1627, 1564, 1501, 1438, 1375, 1312, 1249, 1186, 1123, 1060, 997, 934,
  871, 808, 745, 682, 619, 556, 493, 430, 367, 304, 241, 178, 115, 52, 3392, 3329, 3266, 3203, 3140, 3077, 3014, 2951, 2888, 2825, 2762, 2699, 2636, 2573, 2510, 2447, 2384, 2321, 2258, 2195, 2132, 2069, 2006, 1943, 1880, 1817, 1754, 1691, 1628, 1565, 1502, 1439, 1376, 1313, 1250, 1187,
  1124, 1061, 998, 935, 872, 809, 746, 683, 620, 557, 494, 431, 368, 305, 242, 179, 116, 53, 3456, 3393, 3330, 3267, 3204, 3141, 3078, 3015, 2952, 2889, 2826, 2763, 2700, 2637, 2574, 2511, 2448, 2385, 2322, 2259, 2196, 2133, 2070, 2007, 1944, 1881, 1818, 1755, 1692, 1629, 1566, 1503,
  1440, 1377, 1314, 1251, 1188, 1125, 1062, 999, 936, 873, 810, 747, 684, 621, 558, 495, 432, 369, 306, 243, 180, 117, 54, 3520, 3457, 3394, 3331, 3268, 3205, 3142, 3079, 3016, 2953, 2890, 2827, 2764, 2701, 2638, 2575, 2512, 2449, 2386, 2323, 2260, 2197, 2134, 2071, 2008, 1945, 1882,
  1819, 1756, 1693, 1630, 1567, 1504, 1441, 1378, 1315, 1252, 1189, 1126, 1063, 1000, 937, 874, 811, 748, 685, 622, 559, 496, 433, 370, 307, 244, 181, 118, 55, 3584, 3521, 3458, 3395, 3332, 3269, 3206, 3143, 3080, 3017, 2954, 2891, 2828, 2765, 2702, 2639, 2576, 2513, 2450, 2387, 2324,
  2261, 2198, 2135, 2072, 2009, 1946, 1883, 1820, 1757, 1694, 1631, 1568, 1505, 1442, 1379, 1316, 1253, 1190, 1127, 1064, 1001, 938, 875, 812, 749, 686, 623, 560, 497, 434, 371, 308, 245, 182, 119, 56, 3648, 3585, 3522, 3459, 3396, 3333, 3270, 3207, 3144, 3081, 3018, 2955, 2892, 2829,
  2766, 2703, 2640, 2577, 2514, 2451, 2388, 2325, 2262, 2199, 2136, 2073, 2010, 1947, 1884, 1821, 1758, 1695, 1632, 1569, 1506, 1443, 1380, 1317, 1254, 1191, 1128, 1065, 1002, 939, 876, 813, 750, 687, 624, 561, 498, 435, 372, 309, 246, 183, 120, 57, 3712, 3649, 3586, 3523, 3460, 3397,
  3334, 3271, 3208, 3145, 3082, 3019, 2956, 2893, 2830, 2767, 2704, 2641, 2578, 2515, 2452, 2389, 2326, 2263, 2200, 2137, 2074, 2011, 1948, 1885, 1822, 1759, 1696, 1633, 1570, 1507, 1444, 1381, 1318, 1255, 1192, 1129, 1066, 1003, 940, 877, 814, 751, 688, 625, 562, 499, 436, 373, 310, 247,
  184, 121, 58, 3776, 3713, 3650, 3587, 3524, 3461, 3398, 3335, 3272, 3209, 3146, 3083, 3020, 2957, 2894, 2831, 2768, 2705, 2642, 2579, 2516, 2453, 2390, 2327, 2264, 2201, 2138, 2075, 2012, 1949, 1886, 1823, 1760, 1697, 1634, 1571, 1508, 1445, 1382, 1319, 1256, 1193, 1130, 1067, 1004, 941, 878,
  815, 752, 689, 626, 563, 500, 437, 374, 311, 248, 185, 122, 59, 3840, 3777, 3714, 3651, 3588, 3525, 3462, 3399, 3336, 3273, 3210, 3147, 3084, 3021, 2958, 2895, 2832, 2769, 2706, 2643, 2580, 2517, 2454, 2391, 2328, 2265, 2202, 2139, 2076, 2013, 1950, 1887, 1824, 1761, 1698, 1635, 1572,
  1509, 1446, 1383, 1320, 1257, 1194, 1131, 1068, 1005, 942, 879, 816, 753, 690, 627, 564, 501, 438, 375, 312, 249, 186, 123, 60, 3904, 3841, 3778, 3715, 3652, 3589, 3526, 3463, 3400, 3337, 3274, 3211, 3148, 3085, 3022, 2959, 2896, 2833, 2770, 2707, 2644, 2581, 2518, 2455, 2392, 2329,
  2266, 2203, 2140, 2077, 2014, 1951, 1888, 1825, 1762, 1699, 1636, 1573, 1510, 1447, 1384, 1321, 1258, 1195, 1132, 1069, 1006, 943, 880, 817, 754, 691, 628, 565, 502, 439, 376, 313, 250, 187, 124, 61, 3968, 3905, 3842, 3779, 3716, 3653, 3590, 3527, 3464, 3401, 3338, 3275, 3212, 3149,
  3086, 3023, 2960, 2897, 2834, 2771, 2708, 2645, 2582, 2519, 2456, 2393, 2330, 2267, 2204, 2141, 2078, 2015, 1952, 1889, 1826, 1763, 1700, 1637, 1574, 1511, 1448, 1385, 1322, 1259, 1196, 1133, 1070, 1007, 944, 881, 818, 755, 692, 629, 566, 503, 440, 377, 314, 251, 188, 125, 62, 4032,
  3969, 3906, 3843, 3780, 3717, 3654, 3591, 3528, 3465, 3402, 3339, 3276, 3213, 3150, 3087, 3024, 2961, 2898, 2835, 2772, 2709, 2646, 2583, 2520, 2457, 2394, 2331, 2268, 2205, 2142, 2079, 2016, 1953, 1890, 1827, 1764, 1701, 1638, 1575, 1512, 1449, 1386, 1323, 1260, 1197, 1134, 1071, 1008, 945, 882,
  819, 756, 693, 630, 567, 504, 441, 378, 315, 252, 189, 126, 63, 4033, 3970, 3907, 3844, 3781, 3718, 3655, 3592, 3529, 3466, 3403, 3340, 3277, 3214, 3151, 3088, 3025, 2962, 2899, 2836, 2773, 2710, 2647, 2584, 2521, 2458, 2395, 2332, 2269, 2206, 2143, 2080, 2017, 1954, 1891, 1828, 1765,
  1702, 1639, 1576, 1513, 1450, 1387, 1324, 1261, 1198, 1135, 1072, 1009, 946, 883, 820, 757, 694, 631, 568, 505, 442, 379, 316, 253, 190, 127, 4034, 3971, 3908, 3845, 3782, 3719, 3656, 3593, 3530, 3467, 3404, 3341, 3278, 3215, 3152, 3089, 3026, 2963, 2900, 2837, 2774, 2711, 2648, 2585,
  2522, 2459, 2396, 2333, 2270, 2207, 2144, 2081, 2018, 1955, 1892, 1829, 1766, 1703, 1640, 1577, 1514, 1451, 1388, 1325, 1262, 1199, 1136, 1073, 1010, 947, 884, 821, 758, 695, 632, 569, 506, 443, 380, 317, 254, 191, 4035, 3972, 3909, 3846, 3783, 3720, 3657, 3594, 3531, 3468, 3405, 3342,
  3279, 3216, 3153, 3090, 3027, 2964, 2901, 2838, 2775, 2712, 2649, 2586, 2523, 2460, 2397, 2334, 2271, 2208, 2145, 2082, 2019, 1956, 1893, 1830, 1767, 1704, 1641, 1578, 1515, 1452, 1389, 1326, 1263, 1200, 1137, 1074, 1011, 948, 885, 822, 759, 696, 633, 570, 507, 444, 381, 318, 255, 4036,
  3973, 3910, 3847, 3784, 3721, 3658, 3595, 3532, 3469, 3406, 3343, 3280, 3217, 3154, 3091, 3028, 2965, 2902, 2839, 2776, 2713, 2650, 2587, 2524, 2461, 2398, 2335, 2272, 2209, 2146, 2083, 2020, 1957, 1894, 1831, 1768, 1705, 1642, 1579, 1516, 1453, 1390, 1327, 1264, 1201, 1138, 1075, 1012, 949, 886,
  823, 760, 697, 634, 571, 508, 445, 382, 319, 4037, 3974, 3911, 3848, 3785, 3722, 3659, 3596, 3533, 3470, 3407, 3344, 3281, 3218, 3155, 3092, 3029, 2966, 2903, 2840, 2777, 2714, 2651, 2588, 2525, 2462, 2399, 2336, 2273, 2210, 2147, 2084, 2021, 1958, 1895, 1832, 1769, 1706, 1643, 1580, 1517,
  1454, 1391, 1328, 1265, 1202, 1139, 1076, 1013, 950, 887, 824, 761, 698, 635, 572, 509, 446, 383, 4038, 3975, 3912, 3849, 3786, 3723, 3660, 3597, 3534, 3471, 3408, 3345, 3282, 3219, 3156, 3093, 3030, 2967, 2904, 2841, 2778, 2715, 2652, 2589, 2526, 2463, 2400, 2337, 2274, 2211, 2148, 2085,
  2022, 1959, 1896, 1833, 1770, 1707, 1644, 1581, 1518, 1455, 1392, 1329, 1266, 1203, 1140, 1077, 1014, 951, 888, 825, 762, 699, 636, 573, 510, 447, 4039, 3976, 3913, 3850, 3787, 3724, 3661, 3598, 3535, 3472, 3409, 3346, 3283, 3220, 3157, 3094, 3031, 2968, 2905, 2842, 2779, 2716, 2653, 2590,
  2527, 2464, 2401, 2338, 2275, 2212, 2149, 2086, 2023, 1960, 1897, 1834, 1771, 1708, 1645, 1582, 1519, 1456, 1393, 1330, 1267, 1204, 1141, 1078, 1015, 952, 889, 826, 763, 700, 637, 574, 511, 4040, 3977, 3914, 3851, 3788, 3725, 3662, 3599, 3536, 3473, 3410, 3347, 3284, 3221, 3158, 3095, 3032,
  2969, 2906, 2843, 2780, 2717, 2654, 2591, 2528, 2465, 2402, 2339, 2276, 2213, 2150, 2087, 2024, 1961, 1898, 1835, 1772, 1709, 1646, 1583, 1520, 1457, 1394, 1331, 1268, 1205, 1142, 1079, 1016, 953, 890, 827, 764, 701, 638, 575, 4041, 3978, 3915, 3852, 3789, 3726, 3663, 3600, 3537, 3474, 3411,
  3348, 3285, 3222, 3159, 3096, 3033, 2970, 2907, 2844, 2781, 2718, 2655, 2592, 2529, 2466, 2403, 2340, 2277, 2214, 2151, 2088, 2025, 1962, 1899, 1836, 1773, 1710, 1647, 1584, 1521, 1458, 1395, 1332, 1269, 1206, 1143, 1080, 1017, 954, 891, 828, 765, 702, 639, 4042, 3979, 3916, 3853, 3790, 3727,
  3664, 3601, 3538, 3475, 3412, 3349, 3286, 3223, 3160, 3097, 3034, 2971, 2908, 2845, 2782, 2719, 2656, 2593, 2530, 2467, 2404, 2341, 2278, 2215, 2152, 2089, 2026, 1963, 1900, 1837, 1774, 1711, 1648, 1585, 1522, 1459, 1396, 1333, 1270, 1207, 1144, 1081, 1018, 955, 892, 829, 766, 703, 4043, 3980,
  3917, 3854, 3791, 3728, 3665, 3602, 3539, 3476, 3413, 3350, 3287, 3224, 3161, 3098, 3035, 2972, 2909, 2846, 2783, 2720, 2657, 2594, 2531, 2468, 2405, 2342, 2279, 2216, 2153, 2090, 2027, 1964, 1901, 1838, 1775, 1712, 1649, 1586, 1523, 1460, 1397, 1334, 1271, 1208, 1145, 1082, 1019, 956, 893, 830,
  767, 4044, 3981, 3918, 3855, 3792, 3729, 3666, 3603, 3540, 3477, 3414, 3351, 3288, 3225, 3162, 3099, 3036, 2973, 2910, 2847, 2784, 2721, 2658, 2595, 2532, 2469, 2406, 2343, 2280, 2217, 2154, 2091, 2028, 1965, 1902, 1839, 1776, 1713, 1650, 1587, 1524, 1461, 1398, 1335, 1272, 1209, 1146, 1083, 1020,
  957, 894, 831, 4045, 3982, 3919, 3856, 3793, 3730, 3667, 3604, 3541, 3478, 3415, 3352, 3289, 3226, 3163, 3100, 3037, 2974, 2911, 2848, 2785, 2722, 2659, 2596, 2533, 2470, 2407, 2344, 2281, 2218, 2155, 2092, 2029, 1966, 1903, 1840, 1777, 1714, 1651, 1588, 1525, 1462, 1399, 1336, 1273, 1210, 1147,
  1084, 1021, 958, 895, 4046, 3983, 3920, 3857, 3794, 3731, 3668, 3605, 3542, 3479, 3416, 3353, 3290, 3227, 3164, 3101, 3038, 2975, 2912, 2849, 2786, 2723, 2660, 2597, 2534, 2471, 2408, 2345, 2282, 2219, 2156, 2093, 2030, 1967, 1904, 1841, 1778, 1715, 1652, 1589, 1526, 1463, 1400, 1337, 1274, 1211,
  1148, 1085, 1022, 959, 4047, 3984, 3921, 3858, 3795, 3732, 3669, 3606, 3543, 3480, 3417, 3354, 3291, 3228, 3165, 3102, 3039, 2976, 2913, 2850, 2787, 2724, 2661, 2598, 2535, 2472, 2409, 2346, 2283, 2220, 2157, 2094, 2031, 1968, 1905, 1842, 1779, 1716, 1653, 1590, 1527, 1464, 1401, 1338, 1275, 1212,
  1149, 1086, 1023, 4048, 3985, 3922, 3859, 3796, 3733, 3670, 3607, 3544, 3481, 3418, 3355, 3292, 3229, 3166, 3103, 3040, 2977, 2914, 2851, 2788, 2725, 2662, 2599, 2536, 2473, 2410, 2347, 2284, 2221, 2158, 2095, 2032, 1969, 1906, 1843, 1780, 1717, 1654, 1591, 1528, 1465, 1402, 1339, 1276, 1213, 1150,
  1087, 4049, 3986, 3923, 3860, 3797, 3734, 3671, 3608, 3545, 3482, 3419, 3356, 3293, 3230, 3167, 3104, 3041, 2978, 2915, 2852, 2789, 2726, 2663, 2600, 2537, 2474, 2411, 2348, 2285, 2222, 2159, 2096, 2033, 1970, 1907, 1844, 1781, 1718, 1655, 1592, 1529, 1466, 1403, 1340, 1277, 1214, 1151, 4050, 3987,
  3924, 3861, 3798, 3735, 3672, 3609, 3546, 3483, 3420, 3357, 3294, 3231, 3168, 3105, 3042, 2979, 2916, 2853, 2790, 2727, 2664, 2601, 2538, 2475, 2412, 2349, 2286, 2223, 2160, 2097, 2034, 1971, 1908, 1845, 1782, 1719, 1656, 1593, 1530, 1467, 1404, 1341, 1278, 1215, 4051, 3988, 3925, 3862, 3799, 3736,
  3673, 3610, 3547, 3484, 3421, 3358, 3295, 3232, 3169, 3106, 3043, 2980, 2917, 2854, 2791, 2728, 2665, 2602, 2539, 2476, 2413, 2350, 2287, 2224, 2161, 2098, 2035, 1972, 1909, 1846, 1783, 1720, 1657, 1594, 1531, 1468, 1405, 1342, 1279, 4052, 3989, 3926, 3863, 3800, 3737, 3674, 3611, 3548, 3485, 3422,
  3359, 3296, 3233, 3170, 3107, 3044, 2981, 2918, 2855, 2792, 2729, 2666, 2603, 2540, 2477, 2414, 2351, 2288, 2225, 2162, 2099, 2036, 1973, 1910, 1847, 1784, 1721, 1658, 1595, 1532, 1469, 1406, 1343, 4053, 3990, 3927, 3864, 3801, 3738, 3675, 3612, 3549, 3486, 3423, 3360, 3297, 3234, 3171, 3108, 3045,
  2982, 2919, 2856, 2793, 2730, 2667, 2604, 2541, 2478, 2415, 2352, 2289, 2226, 2163, 2100, 2037, 1974, 1911, 1848, 1785, 1722, 1659, 1596, 1533, 1470, 1407, 4054, 3991, 3928, 3865, 3802, 3739, 3676, 3613, 3550, 3487, 3424, 3361, 3298, 3235, 3172, 3109, 3046, 2983, 2920, 2857, 2794, 2731, 2668, 2605,
  2542, 2479, 2416, 2353, 2290, 2227, 2164, 2101, 2038, 1975, 1912, 1849, 1786, 1723, 1660, 1597, 1534, 1471, 4055, 3992, 3929, 3866, 3803, 3740, 3677, 3614, 3551, 3488, 3425, 3362, 3299, 3236, 3173, 3110, 3047, 2984, 2921, 2858, 2795, 2732, 2669, 2606, 2543, 2480, 2417, 2354, 2291, 2228, 2165, 2102,
  2039, 1976, 1913, 1850, 1787, 1724, 1661, 1598, 1535, 4056, 3993, 3930, 3867, 3804, 3741, 3678, 3615, 3552, 3489, 3426, 3363, 3300, 3237, 3174, 3111, 3048, 2985, 2922, 2859, 2796, 2733, 2670, 2607, 2544, 2481, 2418, 2355, 2292, 2229, 2166, 2103, 2040, 1977, 1914, 1851, 1788, 1725, 1662, 1599, 4057,
  3994, 3931, 3868, 3805, 3742, 3679, 3616, 3553, 3490, 3427, 3364, 3301, 3238, 3175, 3112, 3049, 2986, 2923, 2860, 2797, 2734, 2671, 2608, 2545, 2482, 2419, 2356, 2293, 2230, 2167, 2104, 2041, 1978, 1915, 1852, 1789, 1726, 1663, 4058, 3995, 3932, 3869, 3806, 3743, 3680, 3617, 3554, 3491, 3428, 3365,
  3302, 3239, 3176, 3113, 3050, 2987, 2924, 2861, 2798, 2735, 2672, 2609, 2546, 2483, 2420, 2357, 2294, 2231, 2168, 2105, 2042, 1979, 1916, 1853, 1790, 1727, 4059, 3996, 3933, 3870, 3807, 3744, 3681, 3618, 3555, 3492, 3429, 3366, 3303, 3240, 3177, 3114, 3051, 2988, 2925, 2862, 2799, 2736, 2673, 2610,
  2547, 2484, 2421, 2358, 2295, 2232, 2169, 2106, 2043, 1980, 1917, 1854, 1791, 4060, 3997, 3934, 3871, 3808, 3745, 3682, 3619, 3556, 3493, 3430, 3367, 3304, 3241, 3178, 3115, 3052, 2989, 2926, 2863, 2800, 2737, 2674, 2611, 2548, 2485, 2422, 2359, 2296, 2233, 2170, 2107, 2044, 1981, 1918, 1855, 4061,
  3998, 3935, 3872, 3809, 3746, 3683, 3620, 3557, 3494, 3431, 3368, 3305, 3242, 3179, 3116, 3053, 2990, 2927, 2864, 2801, 2738, 2675, 2612, 2549, 2486, 2423, 2360, 2297, 2234, 2171, 2108, 2045, 1982, 1919, 4062, 3999, 3936, 3873, 3810, 3747, 3684, 3621, 3558, 3495, 3432, 3369, 3306, 3243, 3180, 3117,
  3054, 2991, 2928, 2865, 2802, 2739, 2676, 2613, 2550, 2487, 2424, 2361, 2298, 2235, 2172, 2109, 2046, 1983, 4063, 4000, 3937, 3874, 3811, 3748, 3685, 3622, 3559, 3496, 3433, 3370, 3307, 3244, 3181, 3118, 3055, 2992, 2929, 2866, 2803, 2740, 2677, 2614, 2551, 2488, 2425, 2362, 2299, 2236, 2173, 2110,
  2047, 4064, 4001, 3938, 3875, 3812, 3749, 3686, 3623, 3560, 3497, 3434, 3371, 3308, 3245, 3182, 3119, 3056, 2993, 2930, 2867, 2804, 2741, 2678, 2615, 2552, 2489, 2426, 2363, 2300, 2237, 2174, 2111, 4065, 4002, 3939, 3876, 3813, 3750, 3687, 3624, 3561, 3498, 3435, 3372, 3309, 3246, 3183, 3120, 3057,
  2994, 2931, 2868, 2805, 2742, 2679, 2616, 2553, 2490, 2427, 2364, 2301, 2238, 2175, 4066, 4003, 3940, 3877, 3814, 3751, 3688, 3625, 3562, 3499, 3436, 3373, 3310, 3247, 3184, 3121, 3058, 2995, 2932, 2869, 2806, 2743, 2680, 2617, 2554, 2491, 2428, 2365, 2302, 2239, 4067, 4004, 3941, 3878, 3815, 3752,
  3689, 3626, 3563, 3500, 3437, 3374, 3311, 3248, 3185, 3122, 3059, 2996, 2933, 2870, 2807, 2744, 2681, 2618, 2555, 2492, 2429, 2366, 2303, 4068, 4005, 3942, 3879, 3816, 3753, 3690, 3627, 3564, 3501, 3438, 3375, 3312, 3249, 3186, 3123, 3060, 2997, 2934, 2871, 2808, 2745, 2682, 2619, 2556, 2493, 2430,
  2367, 4069, 4006, 3943, 3880, 3817, 3754, 3691, 3628, 3565, 3502, 3439, 3376, 3313, 3250, 3187, 3124, 3061, 2998, 2935, 2872, 2809, 2746, 2683, 2620, 2557, 2494, 2431, 4070, 4007, 3944, 3881, 3818, 3755, 3692, 3629, 3566, 3503, 3440, 3377, 3314, 3251, 3188, 3125, 3062, 2999, 2936, 2873, 2810, 2747,
  2684, 2621, 2558, 2495, 4071, 4008, 3945, 3882, 3819, 3756, 3693, 3630, 3567, 3504, 3441, 3378, 3315, 3252, 3189, 3126, 3063, 3000, 2937, 2874, 2811, 2748, 2685, 2622, 2559, 4072, 4009, 3946, 3883, 3820, 3757, 3694, 3631, 3568, 3505, 3442, 3379, 3316, 3253, 3190, 3127, 3064, 3001, 2938, 2875, 2812,
  2749, 2686, 2623, 4073, 4010, 3947, 3884, 3821, 3758, 3695, 3632, 3569, 3506, 3443, 3380, 3317, 3254, 3191, 3128, 3065, 3002, 2939, 2876, 2813, 2750, 2687, 4074, 4011, 3948, 3885, 3822, 3759, 3696, 3633, 3570, 3507, 3444, 3381, 3318, 3255, 3192, 3129, 3066, 3003, 2940, 2877, 2814, 2751, 4075, 4012,
  3949, 3886, 3823, 3760, 3697, 3634, 3571, 3508, 3445, 3382, 3319, 3256, 3193, 3130, 3067, 3004, 2941, 2878, 2815, 4076, 4013, 3950, 3887, 3824, 3761, 3698, 3635, 3572, 3509, 3446, 3383, 3320, 3257, 3194, 3131, 3068, 3005, 2942, 2879, 4077, 4014, 3951, 3888, 3825, 3762, 3699, 3636, 3573, 3510, 3447,
  3384, 3321, 3258, 3195, 3132, 3069, 3006, 2943, 4078, 4015, 3952, 3889, 3826, 3763, 3700, 3637, 3574, 3511, 3448, 3385, 3322, 3259, 3196, 3133, 3070, 3007, 4079, 4016, 3953, 3890, 3827, 3764, 3701, 3638, 3575, 3512, 3449, 3386, 3323, 3260, 3197, 3134, 3071, 4080, 4017, 3954, 3891, 3828, 3765, 3702,
  3639, 3576, 3513, 3450, 3387, 3324, 3261, 3198, 3135, 4081, 4018, 3955, 3892, 3829, 3766, 3703, 3640, 3577, 3514, 3451, 3388, 3325, 3262, 3199, 4082, 4019, 3956, 3893, 3830, 3767, 3704, 3641, 3578, 3515, 3452, 3389, 3326, 3263, 4083, 4020, 3957, 3894, 3831, 3768, 3705, 3642, 3579, 3516, 3453, 3390,
  3327, 4084, 4021, 3958, 3895, 3832, 3769, 3706, 3643, 3580, 3517, 3454, 3391, 4085, 4022, 3959, 3896, 3833, 3770, 3707, 3644, 3581, 3518, 3455, 4086, 4023, 3960, 3897, 3834, 3771, 3708, 3645, 3582, 3519, 4087, 4024, 3961, 3898, 3835, 3772, 3709, 3646, 3583, 4088, 4025, 3962, 3899, 3836, 3773, 3710,
  3647, 4089, 4026, 3963, 3900, 3837, 3774, 3711, 4090, 4027, 3964, 3901, 3838, 3775, 4091, 4028, 3965, 3902, 3839, 4092, 4029, 3966, 3903, 4093, 4030, 3967, 4094, 4031, 4095, 0, 0, 1, 0, 1, 2, 3, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5,
  6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 0, 1, 2, 3, 4, 5, 6, 7,
  8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63,
  63, 63, 63, 63, 63, 63, 0, 1, 0, 2, 1, 3, 0, 2, 1, 3, 4, 6, 5, 7, 0, 2, 1, 4, 3, 6, 5, 8, 7, 10, 9, 12, 11, 14, 13, 15, 0, 2, 1, 4, 3, 6, 5, 8, 7, 10, 9, 12, 11, 14,
  13, 15, 16, 18, 17, 20, 19, 22, 21, 24, 23, 26, 25, 28, 27, 30, 29, 31, 0, 2, 1, 4, 3, 6, 5, 8, 7, 10, 9, 12, 11, 14, 13, 15, 16, 18, 17, 20, 19, 22, 21, 24, 23, 26, 25, 28, 27, 30, 29, 31,
  32, 34, 33, 36, 35, 38, 37, 40, 39, 42, 41, 44, 43, 46, 45, 47, 48, 50, 49, 52, 51, 54, 53, 56, 55, 58, 57, 60, 59, 62, 61, 63, 0, 2, 1, 4, 3, 6, 5, 8, 7, 10, 9, 12, 11, 14, 13, 15, 16, 18,
  17, 20, 19, 22, 21, 24, 23, 26, 25, 28, 27, 30, 29, 31, 32, 34, 33, 36, 35, 38, 37, 40, 39, 42, 41, 44, 43, 46, 45, 47, 48, 50, 49, 52, 51, 54, 53, 56, 55, 58, 57, 60, 59, 62, 61, 63, 127, 127, 127, 127,
  127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
  127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 0, 1, 2, 3, 0, 4, 1, 5, 2, 6, 3, 7, 0, 4, 1, 8, 5, 2, 12, 9, 6, 3, 13, 10, 7, 14, 11, 15, 0, 4, 1, 8, 5, 2, 12, 9, 6, 3, 13, 10,
  7, 14, 11, 15, 16, 20, 17, 24, 21, 18, 28, 25, 22, 19, 29, 26, 23, 30, 27, 31, 0, 4, 1, 8, 5, 2, 12, 9, 6, 3, 13, 10, 7, 14, 11, 15, 16, 20, 17, 24, 21, 18, 28, 25, 22, 19, 29, 26, 23, 30,
  27, 31, 32, 36, 33, 40, 37, 34, 44, 41, 38, 35, 45, 42, 39, 46, 43, 47, 48, 52, 49, 56, 53, 50, 60, 57, 54, 51, 61, 58, 55, 62, 59, 63, 0, 4, 1, 8, 5, 2, 12, 9, 6, 3, 13, 10, 7, 14, 11, 15,
  16, 20, 17, 24, 21, 18, 28, 25, 22, 19, 29, 26, 23, 30, 27, 31, 32, 36, 33, 40, 37, 34, 44, 41, 38, 35, 45, 42, 39, 46, 43, 47, 48, 52, 49, 56, 53, 50, 60, 57, 54, 51, 61, 58, 55, 62, 59, 63, 64, 68,
  65, 72, 69, 66, 76, 73, 70, 67, 77, 74, 71, 78, 75, 79, 80, 84, 81, 88, 85, 82, 92, 89, 86, 83, 93, 90, 87, 94, 91, 95, 96, 100, 97, 104, 101, 98, 108, 105, 102, 99, 109, 106, 103, 110, 107, 111, 112, 116, 113, 120,
  117, 114, 124, 121, 118, 115, 125, 122, 119, 126, 123, 127, 0, 4, 1, 8, 5, 2, 12, 9, 6, 3, 13, 10, 7, 14, 11, 15, 16, 20, 17, 24, 21, 18, 28, 25, 22, 19, 29, 26, 23, 30, 27, 31, 32, 36, 33, 40, 37, 34,
  44, 41, 38, 35, 45, 42, 39, 46, 43, 47, 48, 52, 49, 56, 53, 50, 60, 57, 54, 51, 61, 58, 55, 62, 59, 63, 64, 68, 65, 72, 69, 66, 76, 73, 70, 67, 77, 74, 71, 78, 75, 79, 80, 84, 81, 88, 85, 82, 92, 89,
  86, 83, 93, 90, 87, 94, 91, 95, 96, 100, 97, 104, 101, 98, 108, 105, 102, 99, 109, 106, 103, 110, 107, 111, 112, 116, 113, 120, 117, 114, 124, 121, 118, 115, 125, 122, 119, 126, 123, 127, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 1, 2, 3, 4, 5, 6, 7, 0, 8, 1, 9, 2, 10, 3, 11, 4, 12, 5, 13, 6, 14, 7, 15, 0, 8, 1, 16, 9, 2, 24, 17,
  10, 3, 25, 18, 11, 26, 19, 27, 4, 12, 5, 20, 13, 6, 28, 21, 14, 7, 29, 22, 15, 30, 23, 31, 0, 8, 1, 16, 9, 2, 24, 17, 10, 3, 25, 18, 11, 26, 19, 27, 32, 40, 33, 48, 41, 34, 56, 49, 42, 35,
  57, 50, 43, 58, 51, 59, 4, 12, 5, 20, 13, 6, 28, 21, 14, 7, 29, 22, 15, 30, 23, 31, 36, 44, 37, 52, 45, 38, 60, 53, 46, 39, 61, 54, 47, 62, 55, 63, 0, 8, 1, 16, 9, 2, 24, 17, 10, 3, 25, 18,
  11, 26, 19, 27, 32, 40, 33, 48, 41, 34, 56, 49, 42, 35, 57, 50, 43, 58, 51, 59, 4, 12, 5, 20, 13, 6, 28, 21, 14, 7, 29, 22, 15, 30, 23, 31, 64, 72, 65, 80, 73, 66, 88, 81, 74, 67, 89, 82, 75, 90,
  83, 91, 36, 44, 37, 52, 45, 38, 60, 53, 46, 39, 61, 54, 47, 62, 55, 63, 96, 104, 97, 112, 105, 98, 120, 113, 106, 99, 121, 114, 107, 122, 115, 123, 68, 76, 69, 84, 77, 70, 92, 85, 78, 71, 93, 86, 79, 94, 87, 95,
  100, 108, 101, 116, 109, 102, 124, 117, 110, 103, 125, 118, 111, 126, 119, 127, 0, 8, 1, 16, 9, 2, 24, 17, 10, 3, 25, 18, 11, 26, 19, 27, 32, 40, 33, 48, 41, 34, 56, 49, 42, 35, 57, 50, 43, 58, 51, 59, 4, 12,
  5, 20, 13, 6, 28, 21, 14, 7, 29, 22, 15, 30, 23, 31, 64, 72, 65, 80, 73, 66, 88, 81, 74, 67, 89, 82, 75, 90, 83, 91, 36, 44, 37, 52, 45, 38, 60, 53, 46, 39, 61, 54, 47, 62, 55, 63, 96, 104, 97, 112,
  105, 98, 120, 113, 106, 99, 121, 114, 107, 122, 115, 123, 68, 76, 69, 84, 77, 70, 92, 85, 78, 71, 93, 86, 79, 94, 87, 95, 128, 136, 129, 144, 137, 130, 152, 145, 138, 131, 153, 146, 139, 154, 147, 155, 100, 108, 101, 116, 109, 102,
  124, 117, 110, 103, 125, 118, 111, 126, 119, 127, 160, 168, 161, 176, 169, 162, 184, 177, 170, 163, 185, 178, 171, 186, 179, 187, 132, 140, 133, 148, 141, 134, 156, 149, 142, 135, 157, 150, 143, 158, 151, 159, 192, 200, 193, 208, 201, 194, 216, 209,
  202, 195, 217, 210, 203, 218, 211, 219, 164, 172, 165, 180, 173, 166, 188, 181, 174, 167, 189, 182, 175, 190, 183, 191, 224, 232, 225, 240, 233, 226, 248, 241, 234, 227, 249, 242, 235, 250, 243, 251, 196, 204, 197, 212, 205, 198, 220, 213, 206, 199,
  221, 214, 207, 222, 215, 223, 228, 236, 229, 244, 237, 230, 252, 245, 238, 231, 253, 246, 239, 254, 247, 255, 0, 8, 1, 16, 9, 2, 24, 17, 10, 3, 25, 18, 11, 26, 19, 27, 32, 40, 33, 48, 41, 34, 56, 49, 42, 35, 57, 50,
  43, 58, 51, 59, 4, 12, 5, 20, 13, 6, 28, 21, 14, 7, 29, 22, 15, 30, 23, 31, 64, 72, 65, 80, 73, 66, 88, 81, 74, 67, 89, 82, 75, 90, 83, 91, 36, 44, 37, 52, 45, 38, 60, 53, 46, 39, 61, 54, 47, 62,
  55, 63, 96, 104, 97, 112, 105, 98, 120, 113, 106, 99, 121, 114, 107, 122, 115, 123, 68, 76, 69, 84, 77, 70, 92, 85, 78, 71, 93, 86, 79, 94, 87, 95, 128, 136, 129, 144, 137, 130, 152, 145, 138, 131, 153, 146, 139, 154, 147, 155,
  100, 108, 101, 116, 109, 102, 124, 117, 110, 103, 125, 118, 111, 126, 119, 127, 160, 168, 161, 176, 169, 162, 184, 177, 170, 163, 185, 178, 171, 186, 179, 187, 132, 140, 133, 148, 141, 134, 156, 149, 142, 135, 157, 150, 143, 158, 151, 159, 192, 200,
  193, 208, 201, 194, 216, 209, 202, 195, 217, 210, 203, 218, 211, 219, 164, 172, 165, 180, 173, 166, 188, 181, 174, 167, 189, 182, 175, 190, 183, 191, 224, 232, 225, 240, 233, 226, 248, 241, 234, 227, 249, 242, 235, 250, 243, 251, 196, 204, 197, 212,
  205, 198, 220, 213, 206, 199, 221, 214, 207, 222, 215, 223, 228, 236, 229, 244, 237, 230, 252, 245, 238, 231, 253, 246, 239, 254, 247, 255, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511,
  511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511,
  511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511,
  511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511,
  511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511,
  511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
  0, 16, 1, 17, 2, 18, 3, 19, 4, 20, 5, 21, 6, 22, 7, 23, 8, 24, 9, 25, 10, 26, 11, 27, 12, 28, 13, 29, 14, 30, 15, 31, 0, 16, 1, 32, 17, 2, 48, 33, 18, 3, 49, 34, 19, 50, 35, 51, 4, 20,
  5, 36, 21, 6, 52, 37, 22, 7, 53, 38, 23, 54, 39, 55, 8, 24, 9, 40, 25, 10, 56, 41, 26, 11, 57, 42, 27, 58, 43, 59, 12, 28, 13, 44, 29, 14, 60, 45, 30, 15, 61, 46, 31, 62, 47, 63, 0, 16, 1, 32,
  17, 2, 48, 33, 18, 3, 49, 34, 19, 50, 35, 51, 64, 80, 65, 96, 81, 66, 112, 97, 82, 67, 113, 98, 83, 114, 99, 115, 4, 20, 5, 36, 21, 6, 52, 37, 22, 7, 53, 38, 23, 54, 39, 55, 68, 84, 69, 100, 85, 70,
  116, 101, 86, 71, 117, 102, 87, 118, 103, 119, 8, 24, 9, 40, 25, 10, 56, 41, 26, 11, 57, 42, 27, 58, 43, 59, 72, 88, 73, 104, 89, 74, 120, 105, 90, 75, 121, 106, 91, 122, 107, 123, 12, 28, 13, 44, 29, 14, 60, 45,
  30, 15, 61, 46, 31, 62, 47, 63, 76, 92, 77, 108, 93, 78, 124, 109, 94, 79, 125, 110, 95, 126, 111, 127, 0, 16, 1, 32, 17, 2, 48, 33, 18, 3, 49, 34, 19, 50, 35, 51, 64, 80, 65, 96, 81, 66, 112, 97, 82, 67,
  113, 98, 83, 114, 99, 115, 4, 20, 5, 36, 21, 6, 52, 37, 22, 7, 53, 38, 23, 54, 39, 55, 128, 144, 129, 160, 145, 130, 176, 161, 146, 131, 177, 162, 147, 178, 163, 179, 68, 84, 69, 100, 85, 70, 116, 101, 86, 71, 117, 102,
  87, 118, 103, 119, 8, 24, 9, 40, 25, 10, 56, 41, 26, 11, 57, 42, 27, 58, 43, 59, 192, 208, 193, 224, 209, 194, 240, 225, 210, 195, 241, 226, 211, 242, 227, 243, 132, 148, 133, 164, 149, 134, 180, 165, 150, 135, 181, 166, 151, 182,
  167, 183, 72, 88, 73, 104, 89, 74, 120, 105, 90, 75, 121, 106, 91, 122, 107, 123, 12, 28, 13, 44, 29, 14, 60, 45, 30, 15, 61, 46, 31, 62, 47, 63, 196, 212, 197, 228, 213, 198, 244, 229, 214, 199, 245, 230, 215, 246, 231, 247,
  136, 152, 137, 168, 153, 138, 184, 169, 154, 139, 185, 170, 155, 186, 171, 187, 76, 92, 77, 108, 93, 78, 124, 109, 94, 79, 125, 110, 95, 126, 111, 127, 200, 216, 201, 232, 217, 202, 248, 233, 218, 203, 249, 234, 219, 250, 235, 251, 140, 156,
  141, 172, 157, 142, 188, 173, 158, 143, 189, 174, 159, 190, 175, 191, 204, 220, 205, 236, 221, 206, 252, 237, 222, 207, 253, 238, 223, 254, 239, 255, 0, 16, 1, 32, 17, 2, 48, 33, 18, 3, 49, 34, 19, 50, 35, 51, 64, 80, 65, 96,
  81, 66, 112, 97, 82, 67, 113, 98, 83, 114, 99, 115, 4, 20, 5, 36, 21, 6, 52, 37, 22, 7, 53, 38, 23, 54, 39, 55, 128, 144, 129, 160, 145, 130, 176, 161, 146, 131, 177, 162, 147, 178, 163, 179, 68, 84, 69, 100, 85, 70,
  116, 101, 86, 71, 117, 102, 87, 118, 103, 119, 8, 24, 9, 40, 25, 10, 56, 41, 26, 11, 57, 42, 27, 58, 43, 59, 192, 208, 193, 224, 209, 194, 240, 225, 210, 195, 241, 226, 211, 242, 227, 243, 132, 148, 133, 164, 149, 134, 180, 165,
  150, 135, 181, 166, 151, 182, 167, 183, 72, 88, 73, 104, 89, 74, 120, 105, 90, 75, 121, 106, 91, 122, 107, 123, 12, 28, 13, 44, 29, 14, 60, 45, 30, 15, 61, 46, 31, 62, 47, 63, 256, 272, 257, 288, 273, 258, 304, 289, 274, 259,
  305, 290, 275, 306, 291, 307, 196, 212, 197, 228, 213, 198, 244, 229, 214, 199, 245, 230, 215, 246, 231, 247, 136, 152, 137, 168, 153, 138, 184, 169, 154, 139, 185, 170, 155, 186, 171, 187, 76, 92, 77, 108, 93, 78, 124, 109, 94, 79, 125, 110,
  95, 126, 111, 127, 320, 336, 321, 352, 337, 322, 368, 353, 338, 323, 369, 354, 339, 370, 355, 371, 260, 276, 261, 292, 277, 262, 308, 293, 278, 263, 309, 294, 279, 310, 295, 311, 200, 216, 201, 232, 217, 202, 248, 233, 218, 203, 249, 234, 219, 250,
  235, 251, 140, 156, 141, 172, 157, 142, 188, 173, 158, 143, 189, 174, 159, 190, 175, 191, 384, 400, 385, 416, 401, 386, 432, 417, 402, 387, 433, 418, 403, 434, 419, 435, 324, 340, 325, 356, 341, 326, 372, 357, 342, 327, 373, 358, 343, 374, 359, 375,
  264, 280, 265, 296, 281, 266, 312, 297, 282, 267, 313, 298, 283, 314, 299, 315, 204, 220, 205, 236, 221, 206, 252, 237, 222, 207, 253, 238, 223, 254, 239, 255, 448, 464, 449, 480, 465, 450, 496, 481, 466, 451, 497, 482, 467, 498, 483, 499, 388, 404,
  389, 420, 405, 390, 436, 421, 406, 391, 437, 422, 407, 438, 423, 439, 328, 344, 329, 360, 345, 330, 376, 361, 346, 331, 377, 362, 347, 378, 363, 379, 268, 284, 269, 300, 285, 270, 316, 301, 286, 271, 317, 302, 287, 318, 303, 319, 452, 468, 453, 484,
  469, 454, 500, 485, 470, 455, 501, 486, 471, 502, 487, 503, 392, 408, 393, 424, 409, 394, 440, 425, 410, 395, 441, 426, 411, 442, 427, 443, 332, 348, 333, 364, 349, 334, 380, 365, 350, 335, 381, 366, 351, 382, 367, 383, 456, 472, 457, 488, 473, 458,
  504, 489, 474, 459, 505, 490, 475, 506, 491, 507, 396, 412, 397, 428, 413, 398, 444, 429, 414, 399, 445, 430, 415, 446, 431, 447, 460, 476, 461, 492, 477, 462, 508, 493, 478, 463, 509, 494, 479, 510, 495, 511, 0, 16, 1, 32, 17, 2, 48, 33,
  18, 3, 49, 34, 19, 50, 35, 51, 64, 80, 65, 96, 81, 66, 112, 97, 82, 67, 113, 98, 83, 114, 99, 115, 4, 20, 5, 36, 21, 6, 52, 37, 22, 7, 53, 38, 23, 54, 39, 55, 128, 144, 129, 160, 145, 130, 176, 161, 146, 131,
  177, 162, 147, 178, 163, 179, 68, 84, 69, 100, 85, 70, 116, 101, 86, 71, 117, 102, 87, 118, 103, 119, 8, 24, 9, 40, 25, 10, 56, 41, 26, 11, 57, 42, 27, 58, 43, 59, 192, 208, 193, 224, 209, 194, 240, 225, 210, 195, 241, 226,
  211, 242, 227, 243, 132, 148, 133, 164, 149, 134, 180, 165, 150, 135, 181, 166, 151, 182, 167, 183, 72, 88, 73, 104, 89, 74, 120, 105, 90, 75, 121, 106, 91, 122, 107, 123, 12, 28, 13, 44, 29, 14, 60, 45, 30, 15, 61, 46, 31, 62,
  47, 63, 256, 272, 257, 288, 273, 258, 304, 289, 274, 259, 305, 290, 275, 306, 291, 307, 196, 212, 197, 228, 213, 198, 244, 229, 214, 199, 245, 230, 215, 246, 231, 247, 136, 152, 137, 168, 153, 138, 184, 169, 154, 139, 185, 170, 155, 186, 171, 187,
  76, 92, 77, 108, 93, 78, 124, 109, 94, 79, 125, 110, 95, 126, 111, 127, 320, 336, 321, 352, 337, 322, 368, 353, 338, 323, 369, 354, 339, 370, 355, 371, 260, 276, 261, 292, 277, 262, 308, 293, 278, 263, 309, 294, 279, 310, 295, 311, 200, 216,
  201, 232, 217, 202, 248, 233, 218, 203, 249, 234, 219, 250, 235, 251, 140, 156, 141, 172, 157, 142, 188, 173, 158, 143, 189, 174, 159, 190, 175, 191, 384, 400, 385, 416, 401, 386, 432, 417, 402, 387, 433, 418, 403, 434, 419, 435, 324, 340, 325, 356,
  341, 326, 372, 357, 342, 327, 373, 358, 343, 374, 359, 375, 264, 280, 265, 296, 281, 266, 312, 297, 282, 267, 313, 298, 283, 314, 299, 315, 204, 220, 205, 236, 221, 206, 252, 237, 222, 207, 253, 238, 223, 254, 239, 255, 448, 464, 449, 480, 465, 450,
  496, 481, 466, 451, 497, 482, 467, 498, 483, 499, 388, 404, 389, 420, 405, 390, 436, 421, 406, 391, 437, 422, 407, 438, 423, 439, 328, 344, 329, 360, 345, 330, 376, 361, 346, 331, 377, 362, 347, 378, 363, 379, 268, 284, 269, 300, 285, 270, 316, 301,
  286, 271, 317, 302, 287, 318, 303, 319, 452, 468, 453, 484, 469, 454, 500, 485, 470, 455, 501, 486, 471, 502, 487, 503, 392, 408, 393, 424, 409, 394, 440, 425, 410, 395, 441, 426, 411, 442, 427, 443, 332, 348, 333, 364, 349, 334, 380, 365, 350, 335,
  381, 366, 351, 382, 367, 383, 456, 472, 457, 488, 473, 458, 504, 489, 474, 459, 505, 490, 475, 506, 491, 507, 396, 412, 397, 428, 413, 398, 444, 429, 414, 399, 445, 430, 415, 446, 431, 447, 460, 476, 461, 492, 477, 462, 508, 493, 478, 463, 509, 494,
  479, 510, 495, 511, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 0, 32,
  1, 33, 2, 34, 3, 35, 4, 36, 5, 37, 6, 38, 7, 39, 8, 40, 9, 41, 10, 42, 11, 43, 12, 44, 13, 45, 14, 46, 15, 47, 16, 48, 17, 49, 18, 50, 19, 51, 20, 52, 21, 53, 22, 54, 23, 55, 24, 56, 25, 57,
  26, 58, 27, 59, 28, 60, 29, 61, 30, 62, 31, 63, 0, 32, 1, 64, 33, 2, 96, 65, 34, 3, 97, 66, 35, 98, 67, 99, 4, 36, 5, 68, 37, 6, 100, 69, 38, 7, 101, 70, 39, 102, 71, 103, 8, 40, 9, 72, 41, 10,
  104, 73, 42, 11, 105, 74, 43, 106, 75, 107, 12, 44, 13, 76, 45, 14, 108, 77, 46, 15, 109, 78, 47, 110, 79, 111, 16, 48, 17, 80, 49, 18, 112, 81, 50, 19, 113, 82, 51, 114, 83, 115, 20, 52, 21, 84, 53, 22, 116, 85,
  54, 23, 117, 86, 55, 118, 87, 119, 24, 56, 25, 88, 57, 26, 120, 89, 58, 27, 121, 90, 59, 122, 91, 123, 28, 60, 29, 92, 61, 30, 124, 93, 62, 31, 125, 94, 63, 126, 95, 127, 0, 32, 1, 64, 33, 2, 96, 65, 34, 3,
  97, 66, 35, 98, 67, 99, 128, 160, 129, 192, 161, 130, 224, 193, 162, 131, 225, 194, 163, 226, 195, 227, 4, 36, 5, 68, 37, 6, 100, 69, 38, 7, 101, 70, 39, 102, 71, 103, 132, 164, 133, 196, 165, 134, 228, 197, 166, 135, 229, 198,
  167, 230, 199, 231, 8, 40, 9, 72, 41, 10, 104, 73, 42, 11, 105, 74, 43, 106, 75, 107, 136, 168, 137, 200, 169, 138, 232, 201, 170, 139, 233, 202, 171, 234, 203, 235, 12, 44, 13, 76, 45, 14, 108, 77, 46, 15, 109, 78, 47, 110,
  79, 111, 140, 172, 141, 204, 173, 142, 236, 205, 174, 143, 237, 206, 175, 238, 207, 239, 16, 48, 17, 80, 49, 18, 112, 81, 50, 19, 113, 82, 51, 114, 83, 115, 144, 176, 145, 208, 177, 146, 240, 209, 178, 147, 241, 210, 179, 242, 211, 243,
  20, 52, 21, 84, 53, 22, 116, 85, 54, 23, 117, 86, 55, 118, 87, 119, 148, 180, 149, 212, 181, 150, 244, 213, 182, 151, 245, 214, 183, 246, 215, 247, 24, 56, 25, 88, 57, 26, 120, 89, 58, 27, 121, 90, 59, 122, 91, 123, 152, 184,
  153, 216, 185, 154, 248, 217, 186, 155, 249, 218, 187, 250, 219, 251, 28, 60, 29, 92, 61, 30, 124, 93, 62, 31, 125, 94, 63, 126, 95, 127, 156, 188, 157, 220, 189, 158, 252, 221, 190, 159, 253, 222, 191, 254, 223, 255, 0, 32, 1, 64,
  33, 2, 96, 65, 34, 3, 97, 66, 35, 98, 67, 99, 128, 160, 129, 192, 161, 130, 224, 193, 162, 131, 225, 194, 163, 226, 195, 227, 4, 36, 5, 68, 37, 6, 100, 69, 38, 7, 101, 70, 39, 102, 71, 103, 256, 288, 257, 320, 289, 258,
  352, 321, 290, 259, 353, 322, 291, 354, 323, 355, 132, 164, 133, 196, 165, 134, 228, 197, 166, 135, 229, 198, 167, 230, 199, 231, 8, 40, 9, 72, 41, 10, 104, 73, 42, 11, 105, 74, 43, 106, 75, 107, 384, 416, 385, 448, 417, 386, 480, 449,
  418, 387, 481, 450, 419, 482, 451, 483, 260, 292, 261, 324, 293, 262, 356, 325, 294, 263, 357, 326, 295, 358, 327, 359, 136, 168, 137, 200, 169, 138, 232, 201, 170, 139, 233, 202, 171, 234, 203, 235, 12, 44, 13, 76, 45, 14, 108, 77, 46, 15,
  109, 78, 47, 110, 79, 111, 388, 420, 389, 452, 421, 390, 484, 453, 422, 391, 485, 454, 423, 486, 455, 487, 264, 296, 265, 328, 297, 266, 360, 329, 298, 267, 361, 330, 299, 362, 331, 363, 140, 172, 141, 204, 173, 142, 236, 205, 174, 143, 237, 206,
  175, 238, 207, 239, 16, 48, 17, 80, 49, 18, 112, 81, 50, 19, 113, 82, 51, 114, 83, 115, 392, 424, 393, 456, 425, 394, 488, 457, 426, 395, 489, 458, 427, 490, 459, 491, 268, 300, 269, 332, 301, 270, 364, 333, 302, 271, 365, 334, 303, 366,
  335, 367, 144, 176, 145, 208, 177, 146, 240, 209, 178, 147, 241, 210, 179, 242, 211, 243, 20, 52, 21, 84, 53, 22, 116, 85, 54, 23, 117, 86, 55, 118, 87, 119, 396, 428, 397, 460, 429, 398, 492, 461, 430, 399, 493, 462, 431, 494, 463, 495,
  272, 304, 273, 336, 305, 274, 368, 337, 306, 275, 369, 338, 307, 370, 339, 371, 148, 180, 149, 212, 181, 150, 244, 213, 182, 151, 245, 214, 183, 246, 215, 247, 24, 56, 25, 88, 57, 26, 120, 89, 58, 27, 121, 90, 59, 122, 91, 123, 400, 432,
  401, 464, 433, 402, 496, 465, 434, 403, 497, 466, 435, 498, 467, 499, 276, 308, 277, 340, 309, 278, 372, 341, 310, 279, 373, 342, 311, 374, 343, 375, 152, 184, 153, 216, 185, 154, 248, 217, 186, 155, 249, 218, 187, 250, 219, 251, 28, 60, 29, 92,
  61, 30, 124, 93, 62, 31, 125, 94, 63, 126, 95, 127, 404, 436, 405, 468, 437, 406, 500, 469, 438, 407, 501, 470, 439, 502, 471, 503, 280, 312, 281, 344, 313, 282, 376, 345, 314, 283, 377, 346, 315, 378, 347, 379, 156, 188, 157, 220, 189, 158,
  252, 221, 190, 159, 253, 222, 191, 254, 223, 255, 408, 440, 409, 472, 441, 410, 504, 473, 442, 411, 505, 474, 443, 506, 475, 507, 284, 316, 285, 348, 317, 286, 380, 349, 318, 287, 381, 350, 319, 382, 351, 383, 412, 444, 413, 476, 445, 414, 508, 477,
  446, 415, 509, 478, 447, 510, 479, 511, 0, 32, 1, 64, 33, 2, 96, 65, 34, 3, 97, 66, 35, 98, 67, 99, 128, 160, 129, 192, 161, 130, 224, 193, 162, 131, 225, 194, 163, 226, 195, 227, 4, 36, 5, 68, 37, 6, 100, 69, 38, 7,
  101, 70, 39, 102, 71, 103, 256, 288, 257, 320, 289, 258, 352, 321, 290, 259, 353, 322, 291, 354, 323, 355, 132, 164, 133, 196, 165, 134, 228, 197, 166, 135, 229, 198, 167, 230, 199, 231, 8, 40, 9, 72, 41, 10, 104, 73, 42, 11, 105, 74,
  43, 106, 75, 107, 384, 416, 385, 448, 417, 386, 480, 449, 418, 387, 481, 450, 419, 482, 451, 483, 260, 292, 261, 324, 293, 262, 356, 325, 294, 263, 357, 326, 295, 358, 327, 359, 136, 168, 137, 200, 169, 138, 232, 201, 170, 139, 233, 202, 171, 234,
  203, 235, 12, 44, 13, 76, 45, 14, 108, 77, 46, 15, 109, 78, 47, 110, 79, 111, 512, 544, 513, 576, 545, 514, 608, 577, 546, 515, 609, 578, 547, 610, 579, 611, 388, 420, 389, 452, 421, 390, 484, 453, 422, 391, 485, 454, 423, 486, 455, 487,
  264, 296, 265, 328, 297, 266, 360, 329, 298, 267, 361, 330, 299, 362, 331, 363, 140, 172, 141, 204, 173, 142, 236, 205, 174, 143, 237, 206, 175, 238, 207, 239, 16, 48, 17, 80, 49, 18, 112, 81, 50, 19, 113, 82, 51, 114, 83, 115, 640, 672,
  641, 704, 673, 642, 736, 705, 674, 643, 737, 706, 675, 738, 707, 739, 516, 548, 517, 580, 549, 518, 612, 581, 550, 519, 613, 582, 551, 614, 583, 615, 392, 424, 393, 456, 425, 394, 488, 457, 426, 395, 489, 458, 427, 490, 459, 491, 268, 300, 269, 332,
  301, 270, 364, 333, 302, 271, 365, 334, 303, 366, 335, 367, 144, 176, 145, 208, 177, 146, 240, 209, 178, 147, 241, 210, 179, 242, 211, 243, 20, 52, 21, 84, 53, 22, 116, 85, 54, 23, 117, 86, 55, 118, 87, 119, 768, 800, 769, 832, 801, 770,
  864, 833, 802, 771, 865, 834, 803, 866, 835, 867, 644, 676, 645, 708, 677, 646, 740, 709, 678, 647, 741, 710, 679, 742, 711, 743, 520, 552, 521, 584, 553, 522, 616, 585, 554, 523, 617, 586, 555, 618, 587, 619, 396, 428, 397, 460, 429, 398, 492, 461,
  430, 399, 493, 462, 431, 494, 463, 495, 272, 304, 273, 336, 305, 274, 368, 337, 306, 275, 369, 338, 307, 370, 339, 371, 148, 180, 149, 212, 181, 150, 244, 213, 182, 151, 245, 214, 183, 246, 215, 247, 24, 56, 25, 88, 57, 26, 120, 89, 58, 27,
  121, 90, 59, 122, 91, 123, 896, 928, 897, 960, 929, 898, 992, 961, 930, 899, 993, 962, 931, 994, 963, 995, 772, 804, 773, 836, 805, 774, 868, 837, 806, 775, 869, 838, 807, 870, 839, 871, 648, 680, 649, 712, 681, 650, 744, 713, 682, 651, 745, 714,
  683, 746, 715, 747, 524, 556, 525, 588, 557, 526, 620, 589, 558, 527, 621, 590, 559, 622, 591, 623, 400, 432, 401, 464, 433, 402, 496, 465, 434, 403, 497, 466, 435, 498, 467, 499, 276, 308, 277, 340, 309, 278, 372, 341, 310, 279, 373, 342, 311, 374,
  343, 375, 152, 184, 153, 216, 185, 154, 248, 217, 186, 155, 249, 218, 187, 250, 219, 251, 28, 60, 29, 92, 61, 30, 124, 93, 62, 31, 125, 94, 63, 126, 95, 127, 900, 932, 901, 964, 933, 902, 996, 965, 934, 903, 997, 966, 935, 998, 967, 999,
  776, 808, 777, 840, 809, 778, 872, 841, 810, 779, 873, 842, 811, 874, 843, 875, 652, 684, 653, 716, 685, 654, 748, 717, 686, 655, 749, 718, 687, 750, 719, 751, 528, 560, 529, 592, 561, 530, 624, 593, 562, 531, 625, 594, 563, 626, 595, 627, 404, 436,
  405, 468, 437, 406, 500, 469, 438, 407, 501, 470, 439, 502, 471, 503, 280, 312, 281, 344, 313, 282, 376, 345, 314, 283, 377, 346, 315, 378, 347, 379, 156, 188, 157, 220, 189, 158, 252, 221, 190, 159, 253, 222, 191, 254, 223, 255, 904, 936, 905, 968,
  937, 906, 1000, 969, 938, 907, 1001, 970, 939, 1002, 971, 1003, 780, 812, 781, 844, 813, 782, 876, 845, 814, 783, 877, 846, 815, 878, 847, 879, 656, 688, 657, 720, 689, 658, 752, 721, 690, 659, 753, 722, 691, 754, 723, 755, 532, 564, 533, 596, 565, 534,
  628, 597, 566, 535, 629, 598, 567, 630, 599, 631, 408, 440, 409, 472, 441, 410, 504, 473, 442, 411, 505, 474, 443, 506, 475, 507, 284, 316, 285, 348, 317, 286, 380, 349, 318, 287, 381, 350, 319, 382, 351, 383, 908, 940, 909, 972, 941, 910, 1004, 973,
  942, 911, 1005, 974, 943, 1006, 975, 1007, 784, 816, 785, 848, 817, 786, 880, 849, 818, 787, 881, 850, 819, 882, 851, 883, 660, 692, 661, 724, 693, 662, 756, 725, 694, 663, 757, 726, 695, 758, 727, 759, 536, 568, 537, 600, 569, 538, 632, 601, 570, 539,
  633, 602, 571, 634, 603, 635, 412, 444, 413, 476, 445, 414, 508, 477, 446, 415, 509, 478, 447, 510, 479, 511, 912, 944, 913, 976, 945, 914, 1008, 977, 946, 915, 1009, 978, 947, 1010, 979, 1011, 788, 820, 789, 852, 821, 790, 884, 853, 822, 791, 885, 854,
  823, 886, 855, 887, 664, 696, 665, 728, 697, 666, 760, 729, 698, 667, 761, 730, 699, 762, 731, 763, 540, 572, 541, 604, 573, 542, 636, 605, 574, 543, 637, 606, 575, 638, 607, 639, 916, 948, 917, 980, 949, 918, 1012, 981, 950, 919, 1013, 982, 951, 1014,
  983, 1015, 792, 824, 793, 856, 825, 794, 888, 857, 826, 795, 889, 858, 827, 890, 859, 891, 668, 700, 669, 732, 701, 670, 764, 733, 702, 671, 765, 734, 703, 766, 735, 767, 920, 952, 921, 984, 953, 922, 1016, 985, 954, 923, 1017, 986, 955, 1018, 987, 1019,
  796, 828, 797, 860, 829, 798, 892, 861, 830, 799, 893, 862, 831, 894, 863, 895, 924, 956, 925, 988, 957, 926, 1020, 989, 958, 927, 1021, 990, 959, 1022, 991, 1023, 0, 32, 1, 64, 33, 2, 96, 65, 34, 3, 97, 66, 35, 98, 67, 99, 128, 160,
  129, 192, 161, 130, 224, 193, 162, 131, 225, 194, 163, 226, 195, 227, 4, 36, 5, 68, 37, 6, 100, 69, 38, 7, 101, 70, 39, 102, 71, 103, 256, 288, 257, 320, 289, 258, 352, 321, 290, 259, 353, 322, 291, 354, 323, 355, 132, 164, 133, 196,
  165, 134, 228, 197, 166, 135, 229, 198, 167, 230, 199, 231, 8, 40, 9, 72, 41, 10, 104, 73, 42, 11, 105, 74, 43, 106, 75, 107, 384, 416, 385, 448, 417, 386, 480, 449, 418, 387, 481, 450, 419, 482, 451, 483, 260, 292, 261, 324, 293, 262,
  356, 325, 294, 263, 357, 326, 295, 358, 327, 359, 136, 168, 137, 200, 169, 138, 232, 201, 170, 139, 233, 202, 171, 234, 203, 235, 12, 44, 13, 76, 45, 14, 108, 77, 46, 15, 109, 78, 47, 110, 79, 111, 512, 544, 513, 576, 545, 514, 608, 577,
  546, 515, 609, 578, 547, 610, 579, 611, 388, 420, 389, 452, 421, 390, 484, 453, 422, 391, 485, 454, 423, 486, 455, 487, 264, 296, 265, 328, 297, 266, 360, 329, 298, 267, 361, 330, 299, 362, 331, 363, 140, 172, 141, 204, 173, 142, 236, 205, 174, 143,
  237, 206, 175, 238, 207, 239, 16, 48, 17, 80, 49, 18, 112, 81, 50, 19, 113, 82, 51, 114, 83, 115, 640, 672, 641, 704, 673, 642, 736, 705, 674, 643, 737, 706, 675, 738, 707, 739, 516, 548, 517, 580, 549, 518, 612, 581, 550, 519, 613, 582,
  551, 614, 583, 615, 392, 424, 393, 456, 425, 394, 488, 457, 426, 395, 489, 458, 427, 490, 459, 491, 268, 300, 269, 332, 301, 270, 364, 333, 302, 271, 365, 334, 303, 366, 335, 367, 144, 176, 145, 208, 177, 146, 240, 209, 178, 147, 241, 210, 179, 242,
  211, 243, 20, 52, 21, 84, 53, 22, 116, 85, 54, 23, 117, 86, 55, 118, 87, 119, 768, 800, 769, 832, 801, 770, 864, 833, 802, 771, 865, 834, 803, 866, 835, 867, 644, 676, 645, 708, 677, 646, 740, 709, 678, 647, 741, 710, 679, 742, 711, 743,
  520, 552, 521, 584, 553, 522, 616, 585, 554, 523, 617, 586, 555, 618, 587, 619, 396, 428, 397, 460, 429, 398, 492, 461, 430, 399, 493, 462, 431, 494, 463, 495, 272, 304, 273, 336, 305, 274, 368, 337, 306, 275, 369, 338, 307, 370, 339, 371, 148, 180,
  149, 212, 181, 150, 244, 213, 182, 151, 245, 214, 183, 246, 215, 247, 24, 56, 25, 88, 57, 26, 120, 89, 58, 27, 121, 90, 59, 122, 91, 123, 896, 928, 897, 960, 929, 898, 992, 961, 930, 899, 993, 962, 931, 994, 963, 995, 772, 804, 773, 836,
  805, 774, 868, 837, 806, 775, 869, 838, 807, 870, 839, 871, 648, 680, 649, 712, 681, 650, 744, 713, 682, 651, 745, 714, 683, 746, 715, 747, 524, 556, 525, 588, 557, 526, 620, 589, 558, 527, 621, 590, 559, 622, 591, 623, 400, 432, 401, 464, 433, 402,
  496, 465, 434, 403, 497, 466, 435, 498, 467, 499, 276, 308, 277, 340, 309, 278, 372, 341, 310, 279, 373, 342, 311, 374, 343, 375, 152, 184, 153, 216, 185, 154, 248, 217, 186, 155, 249, 218, 187, 250, 219, 251, 28, 60, 29, 92, 61, 30, 124, 93,
  62, 31, 125, 94, 63, 126, 95, 127, 900, 932, 901, 964, 933, 902, 996, 965, 934, 903, 997, 966, 935, 998, 967, 999, 776, 808, 777, 840, 809, 778, 872, 841, 810, 779, 873, 842, 811, 874, 843, 875, 652, 684, 653, 716, 685, 654, 748, 717, 686, 655,
  749, 718, 687, 750, 719, 751, 528, 560, 529, 592, 561, 530, 624, 593, 562, 531, 625, 594, 563, 626, 595, 627, 404, 436, 405, 468, 437, 406, 500, 469, 438, 407, 501, 470, 439, 502, 471, 503, 280, 312, 281, 344, 313, 282, 376, 345, 314, 283, 377, 346,
  315, 378, 347, 379, 156, 188, 157, 220, 189, 158, 252, 221, 190, 159, 253, 222, 191, 254, 223, 255, 904, 936, 905, 968, 937, 906, 1000, 969, 938, 907, 1001, 970, 939, 1002, 971, 1003, 780, 812, 781, 844, 813, 782, 876, 845, 814, 783, 877, 846, 815, 878,
  847, 879, 656, 688, 657, 720, 689, 658, 752, 721, 690, 659, 753, 722, 691, 754, 723, 755, 532, 564, 533, 596, 565, 534, 628, 597, 566, 535, 629, 598, 567, 630, 599, 631, 408, 440, 409, 472, 441, 410, 504, 473, 442, 411, 505, 474, 443, 506, 475, 507,
  284, 316, 285, 348, 317, 286, 380, 349, 318, 287, 381, 350, 319, 382, 351, 383, 908, 940, 909, 972, 941, 910, 1004, 973, 942, 911, 1005, 974, 943, 1006, 975, 1007, 784, 816, 785, 848, 817, 786, 880, 849, 818, 787, 881, 850, 819, 882, 851, 883, 660, 692,
  661, 724, 693, 662, 756, 725, 694, 663, 757, 726, 695, 758, 727, 759, 536, 568, 537, 600, 569, 538, 632, 601, 570, 539, 633, 602, 571, 634, 603, 635, 412, 444, 413, 476, 445, 414, 508, 477, 446, 415, 509, 478, 447, 510, 479, 511, 912, 944, 913, 976,
  945, 914, 1008, 977, 946, 915, 1009, 978, 947, 1010, 979, 1011, 788, 820, 789, 852, 821, 790, 884, 853, 822, 791, 885, 854, 823, 886, 855, 887, 664, 696, 665, 728, 697, 666, 760, 729, 698, 667, 761, 730, 699, 762, 731, 763, 540, 572, 541, 604, 573, 542,
  636, 605, 574, 543, 637, 606, 575, 638, 607, 639, 916, 948, 917, 980, 949, 918, 1012, 981, 950, 919, 1013, 982, 951, 1014, 983, 1015, 792, 824, 793, 856, 825, 794, 888, 857, 826, 795, 889, 858, 827, 890, 859, 891, 668, 700, 669, 732, 701, 670, 764, 733,
  702, 671, 765, 734, 703, 766, 735, 767, 920, 952, 921, 984, 953, 922, 1016, 985, 954, 923, 1017, 986, 955, 1018, 987, 1019, 796, 828, 797, 860, 829, 798, 892, 861, 830, 799, 893, 862, 831, 894, 863, 895, 924, 956, 925, 988, 957, 926, 1020, 989, 958, 927,
  1021, 990, 959, 1022, 991, 1023, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
  20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 0, 64, 1, 65, 2, 66,
  3, 67, 4, 68, 5, 69, 6, 70, 7, 71, 8, 72, 9, 73, 10, 74, 11, 75, 12, 76, 13, 77, 14, 78, 15, 79, 16, 80, 17, 81, 18, 82, 19, 83, 20, 84, 21, 85, 22, 86, 23, 87, 24, 88, 25, 89, 26, 90, 27, 91,
  28, 92, 29, 93, 30, 94, 31, 95, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
  127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 0, 64, 1, 128, 65, 2, 192, 129, 66, 3, 193, 130, 67, 194, 131, 195, 4, 68, 5, 132, 69, 6, 196, 133, 70, 7, 197, 134,
  71, 198, 135, 199, 8, 72, 9, 136, 73, 10, 200, 137, 74, 11, 201, 138, 75, 202, 139, 203, 12, 76, 13, 140, 77, 14, 204, 141, 78, 15, 205, 142, 79, 206, 143, 207, 16, 80, 17, 144, 81, 18, 208, 145, 82, 19, 209, 146, 83, 210,
  147, 211, 20, 84, 21, 148, 85, 22, 212, 149, 86, 23, 213, 150, 87, 214, 151, 215, 24, 88, 25, 152, 89, 26, 216, 153, 90, 27, 217, 154, 91, 218, 155, 219, 28, 92, 29, 156, 93, 30, 220, 157, 94, 31, 221, 158, 95, 222, 159, 223,
  255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 64, 1, 128, 65, 2, 192, 129, 66, 3, 193, 130, 67, 194, 131, 195, 256, 320, 257, 384, 321, 258,
  448, 385, 322, 259, 449, 386, 323, 450, 387, 451, 4, 68, 5, 132, 69, 6, 196, 133, 70, 7, 197, 134, 71, 198, 135, 199, 260, 324, 261, 388, 325, 262, 452, 389, 326, 263, 453, 390, 327, 454, 391, 455, 8, 72, 9, 136, 73, 10, 200, 137,
  74, 11, 201, 138, 75, 202, 139, 203, 264, 328, 265, 392, 329, 266, 456, 393, 330, 267, 457, 394, 331, 458, 395, 459, 12, 76, 13, 140, 77, 14, 204, 141, 78, 15, 205, 142, 79, 206, 143, 207, 268, 332, 269, 396, 333, 270, 460, 397, 334, 271,
  461, 398, 335, 462, 399, 463, 16, 80, 17, 144, 81, 18, 208, 145, 82, 19, 209, 146, 83, 210, 147, 211, 272, 336, 273, 400, 337, 274, 464, 401, 338, 275, 465, 402, 339, 466, 403, 467, 20, 84, 21, 148, 85, 22, 212, 149, 86, 23, 213, 150,
  87, 214, 151, 215, 276, 340, 277, 404, 341, 278, 468, 405, 342, 279, 469, 406, 343, 470, 407, 471, 24, 88, 25, 152, 89, 26, 216, 153, 90, 27, 217, 154, 91, 218, 155, 219, 280, 344, 281, 408, 345, 282, 472, 409, 346, 283, 473, 410, 347, 474,
  411, 475, 28, 92, 29, 156, 93, 30, 220, 157, 94, 31, 221, 158, 95, 222, 159, 223, 284, 348, 285, 412, 349, 286, 476, 413, 350, 287, 477, 414, 351, 478, 415, 479, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511,
  511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511,
  511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511,
  511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511,
  511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511,
  511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 0, 64, 1, 128, 65, 2, 192, 129, 66, 3,
  193, 130, 67, 194, 131, 195, 256, 320, 257, 384, 321, 258, 448, 385, 322, 259, 449, 386, 323, 450, 387, 451, 4, 68, 5, 132, 69, 6, 196, 133, 70, 7, 197, 134, 71, 198, 135, 199, 512, 576, 513, 640, 577, 514, 704, 641, 578, 515, 705, 642,
  579, 706, 643, 707, 260, 324, 261, 388, 325, 262, 452, 389, 326, 263, 453, 390, 327, 454, 391, 455, 8, 72, 9, 136, 73, 10, 200, 137, 74, 11, 201, 138, 75, 202, 139, 203, 768, 832, 769, 896, 833, 770, 960, 897, 834, 771, 961, 898, 835, 962,
  899, 963, 516, 580, 517, 644, 581, 518, 708, 645, 582, 519, 709, 646, 583, 710, 647, 711, 264, 328, 265, 392, 329, 266, 456, 393, 330, 267, 457, 394, 331, 458, 395, 459, 12, 76, 13, 140, 77, 14, 204, 141, 78, 15, 205, 142, 79, 206, 143, 207,
  772, 836, 773, 900, 837, 774, 964, 901, 838, 775, 965, 902, 839, 966, 903, 967, 520, 584, 521, 648, 585, 522, 712, 649, 586, 523, 713, 650, 587, 714, 651, 715, 268, 332, 269, 396, 333, 270, 460, 397, 334, 271, 461, 398, 335, 462, 399, 463, 16, 80,
  17, 144, 81, 18, 208, 145, 82, 19, 209, 146, 83, 210, 147, 211, 776, 840, 777, 904, 841, 778, 968, 905, 842, 779, 969, 906, 843, 970, 907, 971, 524, 588, 525, 652, 589, 526, 716, 653, 590, 527, 717, 654, 591, 718, 655, 719, 272, 336, 273, 400,
  337, 274, 464, 401, 338, 275, 465, 402, 339, 466, 403, 467, 20, 84, 21, 148, 85, 22, 212, 149, 86, 23, 213, 150, 87, 214, 151, 215, 780, 844, 781, 908, 845, 782, 972, 909, 846, 783, 973, 910, 847, 974, 911, 975, 528, 592, 529, 656, 593, 530,
  720, 657, 594, 531, 721, 658, 595, 722, 659, 723, 276, 340, 277, 404, 341, 278, 468, 405, 342, 279, 469, 406, 343, 470, 407, 471, 24, 88, 25, 152, 89, 26, 216, 153, 90, 27, 217, 154, 91, 218, 155, 219, 784, 848, 785, 912, 849, 786, 976, 913,
  850, 787, 977, 914, 851, 978, 915, 979, 532, 596, 533, 660, 597, 534, 724, 661, 598, 535, 725, 662, 599, 726, 663, 727, 280, 344, 281, 408, 345, 282, 472, 409, 346, 283, 473, 410, 347, 474, 411, 475, 28, 92, 29, 156, 93, 30, 220, 157, 94, 31,
  221, 158, 95, 222, 159, 223, 788, 852, 789, 916, 853, 790, 980, 917, 854, 791, 981, 918, 855, 982, 919, 983, 536, 600, 537, 664, 601, 538, 728, 665, 602, 539, 729, 666, 603, 730, 667, 731, 284, 348, 285, 412, 349, 286, 476, 413, 350, 287, 477, 414,
  351, 478, 415, 479, 792, 856, 793, 920, 857, 794, 984, 921, 858, 795, 985, 922, 859, 986, 923, 987, 540, 604, 541, 668, 605, 542, 732, 669, 606, 543, 733, 670, 607, 734, 671, 735, 796, 860, 797, 924, 861, 798, 988, 925, 862, 799, 989, 926, 863, 990,
  927, 991, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
  1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 0, 64, 1, 128, 65, 2, 192, 129, 66, 3, 193, 130, 67, 194, 131, 195, 256, 320, 257, 384, 321, 258, 448, 385, 322, 259, 449, 386, 323, 450, 387, 451, 4, 68, 5, 132,
  69, 6, 196, 133, 70, 7, 197, 134, 71, 198, 135, 199, 512, 576, 513, 640, 577, 514, 704, 641, 578, 515, 705, 642, 579, 706, 643, 707, 260, 324, 261, 388, 325, 262, 452, 389, 326, 263, 453, 390, 327, 454, 391, 455, 8, 72, 9, 136, 73, 10,
  200, 137, 74, 11, 201, 138, 75, 202, 139, 203, 768, 832, 769, 896, 833, 770, 960, 897, 834, 771, 961, 898, 835, 962, 899, 963, 516, 580, 517, 644, 581, 518, 708, 645, 582, 519, 709, 646, 583, 710, 647, 711, 264, 328, 265, 392, 329, 266, 456, 393,
  330, 267, 457, 394, 331, 458, 395, 459, 12, 76, 13, 140, 77, 14, 204, 141, 78, 15, 205, 142, 79, 206, 143, 207, 1024, 1088, 1025, 1152, 1089, 1026, 1216, 1153, 1090, 1027, 1217, 1154, 1091, 1218, 1155, 1219, 772, 836, 773, 900, 837, 774, 964, 901, 838, 775,
  965, 902, 839, 966, 903, 967, 520, 584, 521, 648, 585, 522, 712, 649, 586, 523, 713, 650, 587, 714, 651, 715, 268, 332, 269, 396, 333, 270, 460, 397, 334, 271, 461, 398, 335, 462, 399, 463, 16, 80, 17, 144, 81, 18, 208, 145, 82, 19, 209, 146,
  83, 210, 147, 211, 1280, 1344, 1281, 1408, 1345, 1282, 1472, 1409, 1346, 1283, 1473, 1410, 1347, 1474, 1411, 1475, 1028, 1092, 1029, 1156, 1093, 1030, 1220, 1157, 1094, 1031, 1221, 1158, 1095, 1222, 1159, 1223, 776, 840, 777, 904, 841, 778, 968, 905, 842, 779, 969, 906, 843, 970,
  907, 971, 524, 588, 525, 652, 589, 526, 716, 653, 590, 527, 717, 654, 591, 718, 655, 719, 272, 336, 273, 400, 337, 274, 464, 401, 338, 275, 465, 402, 339, 466, 403, 467, 20, 84, 21, 148, 85, 22, 212, 149, 86, 23, 213, 150, 87, 214, 151, 215,
  1536, 1600, 1537, 1664, 1601, 1538, 1728, 1665, 1602, 1539, 1729, 1666, 1603, 1730, 1667, 1731, 1284, 1348, 1285, 1412, 1349, 1286, 1476, 1413, 1350, 1287, 1477, 1414, 1351, 1478, 1415, 1479, 1032, 1096, 1033, 1160, 1097, 1034, 1224, 1161, 1098, 1035, 1225, 1162, 1099, 1226, 1163, 1227, 780, 844,
  781, 908, 845, 782, 972, 909, 846, 783, 973, 910, 847, 974, 911, 975, 528, 592, 529, 656, 593, 530, 720, 657, 594, 531, 721, 658, 595, 722, 659, 723, 276, 340, 277, 404, 341, 278, 468, 405, 342, 279, 469, 406, 343, 470, 407, 471, 24, 88, 25, 152,
  89, 26, 216, 153, 90, 27, 217, 154, 91, 218, 155, 219, 1792, 1856, 1793, 1920, 1857, 1794, 1984, 1921, 1858, 1795, 1985, 1922, 1859, 1986, 1923, 1987, 1540, 1604, 1541, 1668, 1605, 1542, 1732, 1669, 1606, 1543, 1733, 1670, 1607, 1734, 1671, 1735, 1288, 1352, 1289, 1416, 1353, 1290,
  1480, 1417, 1354, 1291, 1481, 1418, 1355, 1482, 1419, 1483, 1036, 1100, 1037, 1164, 1101, 1038, 1228, 1165, 1102, 1039, 1229, 1166, 1103, 1230, 1167, 1231, 784, 848, 785, 912, 849, 786, 976, 913, 850, 787, 977, 914, 851, 978, 915, 979, 532, 596, 533, 660, 597, 534, 724, 661,
  598, 535, 725, 662, 599, 726, 663, 727, 280, 344, 281, 408, 345, 282, 472, 409, 346, 283, 473, 410, 347, 474, 411, 475, 28, 92, 29, 156, 93, 30, 220, 157, 94, 31, 221, 158, 95, 222, 159, 223, 1796, 1860, 1797, 1924, 1861, 1798, 1988, 1925, 1862, 1799,
  1989, 1926, 1863, 1990, 1927, 1991, 1544, 1608, 1545, 1672, 1609, 1546, 1736, 1673, 1610, 1547, 1737, 1674, 1611, 1738, 1675, 1739, 1292, 1356, 1293, 1420, 1357, 1294, 1484, 1421, 1358, 1295, 1485, 1422, 1359, 1486, 1423, 1487, 1040, 1104, 1041, 1168, 1105, 1042, 1232, 1169, 1106, 1043, 1233, 1170,
  1107, 1234, 1171, 1235, 788, 852, 789, 916, 853, 790, 980, 917, 854, 791, 981, 918, 855, 982, 919, 983, 536, 600, 537, 664, 601, 538, 728, 665, 602, 539, 729, 666, 603, 730, 667, 731, 284, 348, 285, 412, 349, 286, 476, 413, 350, 287, 477, 414, 351, 478,
  415, 479, 1800, 1864, 1801, 1928, 1865, 1802, 1992, 1929, 1866, 1803, 1993, 1930, 1867, 1994, 1931, 1995, 1548, 1612, 1549, 1676, 1613, 1550, 1740, 1677, 1614, 1551, 1741, 1678, 1615, 1742, 1679, 1743, 1296, 1360, 1297, 1424, 1361, 1298, 1488, 1425, 1362, 1299, 1489, 1426, 1363, 1490, 1427, 1491,
  1044, 1108, 1045, 1172, 1109, 1046, 1236, 1173, 1110, 1047, 1237, 1174, 1111, 1238, 1175, 1239, 792, 856, 793, 920, 857, 794, 984, 921, 858, 795, 985, 922, 859, 986, 923, 987, 540, 604, 541, 668, 605, 542, 732, 669, 606, 543, 733, 670, 607, 734, 671, 735, 1804, 1868,
  1805, 1932, 1869, 1806, 1996, 1933, 1870, 1807, 1997, 1934, 1871, 1998, 1935, 1999, 1552, 1616, 1553, 1680, 1617, 1554, 1744, 1681, 1618, 1555, 1745, 1682, 1619, 1746, 1683, 1747, 1300, 1364, 1301, 1428, 1365, 1302, 1492, 1429, 1366, 1303, 1493, 1430, 1367, 1494, 1431, 1495, 1048, 1112, 1049, 1176,
  1113, 1050, 1240, 1177, 1114, 1051, 1241, 1178, 1115, 1242, 1179, 1243, 796, 860, 797, 924, 861, 798, 988, 925, 862, 799, 989, 926, 863, 990, 927, 991, 1808, 1872, 1809, 1936, 1873, 1810, 2000, 1937, 1874, 1811, 2001, 1938, 1875, 2002, 1939, 2003, 1556, 1620, 1557, 1684, 1621, 1558,
  1748, 1685, 1622, 1559, 1749, 1686, 1623, 1750, 1687, 1751, 1304, 1368, 1305, 1432, 1369, 1306, 1496, 1433, 1370, 1307, 1497, 1434, 1371, 1498, 1435, 1499, 1052, 1116, 1053, 1180, 1117, 1054, 1244, 1181, 1118, 1055, 1245, 1182, 1119, 1246, 1183, 1247, 1812, 1876, 1813, 1940, 1877, 1814, 2004, 1941,
  1878, 1815, 2005, 1942, 1879, 2006, 1943, 2007, 1560, 1624, 1561, 1688, 1625, 1562, 1752, 1689, 1626, 1563, 1753, 1690, 1627, 1754, 1691, 1755, 1308, 1372, 1309, 1436, 1373, 1310, 1500, 1437, 1374, 1311, 1501, 1438, 1375, 1502, 1439, 1503, 1816, 1880, 1817, 1944, 1881, 1818, 2008, 1945, 1882, 1819,
  2009, 1946, 1883, 2010, 1947, 2011, 1564, 1628, 1565, 1692, 1629, 1566, 1756, 1693, 1630, 1567, 1757, 1694, 1631, 1758, 1695, 1759, 1820, 1884, 1821, 1948, 1885, 1822, 2012, 1949, 1886, 1823, 2013, 1950, 1887, 2014, 1951, 2015, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
  2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 0, 64, 1, 128, 65, 2, 192, 129, 66, 3, 193, 130, 67, 194, 131, 195, 256, 320, 257, 384, 321, 258, 448, 385, 322, 259, 449, 386, 323, 450, 387, 451, 4, 68, 5, 132, 69, 6,
  196, 133, 70, 7, 197, 134, 71, 198, 135, 199, 512, 576, 513, 640, 577, 514, 704, 641, 578, 515, 705, 642, 579, 706, 643, 707, 260, 324, 261, 388, 325, 262, 452, 389, 326, 263, 453, 390, 327, 454, 391, 455, 8, 72, 9, 136, 73, 10, 200, 137,
  74, 11, 201, 138, 75, 202, 139, 203, 768, 832, 769, 896, 833, 770, 960, 897, 834, 771, 961, 898, 835, 962, 899, 963, 516, 580, 517, 644, 581, 518, 708, 645, 582, 519, 709, 646, 583, 710, 647, 711, 264, 328, 265, 392, 329, 266, 456, 393, 330, 267,
  457, 394, 331, 458, 395, 459, 12, 76, 13, 140, 77, 14, 204, 141, 78, 15, 205, 142, 79, 206, 143, 207, 1024, 1088, 1025, 1152, 1089, 1026, 1216, 1153, 1090, 1027, 1217, 1154, 1091, 1218, 1155, 1219, 772, 836, 773, 900, 837, 774, 964, 901, 838, 775, 965, 902,
  839, 966, 903, 967, 520, 584, 521, 648, 585, 522, 712, 649, 586, 523, 713, 650, 587, 714, 651, 715, 268, 332, 269, 396, 333, 270, 460, 397, 334, 271, 461, 398, 335, 462, 399, 463, 16, 80, 17, 144, 81, 18, 208, 145, 82, 19, 209, 146, 83, 210,
  147, 211, 1280, 1344, 1281, 1408, 1345, 1282, 1472, 1409, 1346, 1283, 1473, 1410, 1347, 1474, 1411, 1475, 1028, 1092, 1029, 1156, 1093, 1030, 1220, 1157, 1094, 1031, 1221, 1158, 1095, 1222, 1159, 1223, 776, 840, 777, 904, 841, 778, 968, 905, 842, 779, 969, 906, 843, 970, 907, 971,
  524, 588, 525, 652, 589, 526, 716, 653, 590, 527, 717, 654, 591, 718, 655, 719, 272, 336, 273, 400, 337, 274, 464, 401, 338, 275, 465, 402, 339, 466, 403, 467, 20, 84, 21, 148, 85, 22, 212, 149, 86, 23, 213, 150, 87, 214, 151, 215, 1536, 1600,
  1537, 1664, 1601, 1538, 1728, 1665, 1602, 1539, 1729, 1666, 1603, 1730, 1667, 1731, 1284, 1348, 1285, 1412, 1349, 1286, 1476, 1413, 1350, 1287, 1477, 1414, 1351, 1478, 1415, 1479, 1032, 1096, 1033, 1160, 1097, 1034, 1224, 1161, 1098, 1035, 1225, 1162, 1099, 1226, 1163, 1227, 780, 844, 781, 908,
  845, 782, 972, 909, 846, 783, 973, 910, 847, 974, 911, 975, 528, 592, 529, 656, 593, 530, 720, 657, 594, 531, 721, 658, 595, 722, 659, 723, 276, 340, 277, 404, 341, 278, 468, 405, 342, 279, 469, 406, 343, 470, 407, 471, 24, 88, 25, 152, 89, 26,
  216, 153, 90, 27, 217, 154, 91, 218, 155, 219, 1792, 1856, 1793, 1920, 1857, 1794, 1984, 1921, 1858, 1795, 1985, 1922, 1859, 1986, 1923, 1987, 1540, 1604, 1541, 1668, 1605, 1542, 1732, 1669, 1606, 1543, 1733, 1670, 1607, 1734, 1671, 1735, 1288, 1352, 1289, 1416, 1353, 1290, 1480, 1417,
  1354, 1291, 1481, 1418, 1355, 1482, 1419, 1483, 1036, 1100, 1037, 1164, 1101, 1038, 1228, 1165, 1102, 1039, 1229, 1166, 1103, 1230, 1167, 1231, 784, 848, 785, 912, 849, 786, 976, 913, 850, 787, 977, 914, 851, 978, 915, 979, 532, 596, 533, 660, 597, 534, 724, 661, 598, 535,
  725, 662, 599, 726, 663, 727, 280, 344, 281, 408, 345, 282, 472, 409, 346, 283, 473, 410, 347, 474, 411, 475, 28, 92, 29, 156, 93, 30, 220, 157, 94, 31, 221, 158, 95, 222, 159, 223, 1796, 1860, 1797, 1924, 1861, 1798, 1988, 1925, 1862, 1799, 1989, 1926,
  1863, 1990, 1927, 1991, 1544, 1608, 1545, 1672, 1609, 1546, 1736, 1673, 1610, 1547, 1737, 1674, 1611, 1738, 1675, 1739, 1292, 1356, 1293, 1420, 1357, 1294, 1484, 1421, 1358, 1295, 1485, 1422, 1359, 1486, 1423, 1487, 1040, 1104, 1041, 1168, 1105, 1042, 1232, 1169, 1106, 1043, 1233, 1170, 1107, 1234,
  1171, 1235, 788, 852, 789, 916, 853, 790, 980, 917, 854, 791, 981, 918, 855, 982, 919, 983, 536, 600, 537, 664, 601, 538, 728, 665, 602, 539, 729, 666, 603, 730, 667, 731, 284, 348, 285, 412, 349, 286, 476, 413, 350, 287, 477, 414, 351, 478, 415, 479,
  1800, 1864, 1801, 1928, 1865, 1802, 1992, 1929, 1866, 1803, 1993, 1930, 1867, 1994, 1931, 1995, 1548, 1612, 1549, 1676, 1613, 1550, 1740, 1677, 1614, 1551, 1741, 1678, 1615, 1742, 1679, 1743, 1296, 1360, 1297, 1424, 1361, 1298, 1488, 1425, 1362, 1299, 1489, 1426, 1363, 1490, 1427, 1491, 1044, 1108,
  1045, 1172, 1109, 1046, 1236, 1173, 1110, 1047, 1237, 1174, 1111, 1238, 1175, 1239, 792, 856, 793, 920, 857, 794, 984, 921, 858, 795, 985, 922, 859, 986, 923, 987, 540, 604, 541, 668, 605, 542, 732, 669, 606, 543, 733, 670, 607, 734, 671, 735, 1804, 1868, 1805, 1932,
  1869, 1806, 1996, 1933, 1870, 1807, 1997, 1934, 1871, 1998, 1935, 1999, 1552, 1616, 1553, 1680, 1617, 1554, 1744, 1681, 1618, 1555, 1745, 1682, 1619, 1746, 1683, 1747, 1300, 1364, 1301, 1428, 1365, 1302, 1492, 1429, 1366, 1303, 1493, 1430, 1367, 1494, 1431, 1495, 1048, 1112, 1049, 1176, 1113, 1050,
  1240, 1177, 1114, 1051, 1241, 1178, 1115, 1242, 1179, 1243, 796, 860, 797, 924, 861, 798, 988, 925, 862, 799, 989, 926, 863, 990, 927, 991, 1808, 1872, 1809, 1936, 1873, 1810, 2000, 1937, 1874, 1811, 2001, 1938, 1875, 2002, 1939, 2003, 1556, 1620, 1557, 1684, 1621, 1558, 1748, 1685,
  1622, 1559, 1749, 1686, 1623, 1750, 1687, 1751, 1304, 1368, 1305, 1432, 1369, 1306, 1496, 1433, 1370, 1307, 1497, 1434, 1371, 1498, 1435, 1499, 1052, 1116, 1053, 1180, 1117, 1054, 1244, 1181, 1118, 1055, 1245, 1182, 1119, 1246, 1183, 1247, 1812, 1876, 1813, 1940, 1877, 1814, 2004, 1941, 1878, 1815,
  2005, 1942, 1879, 2006, 1943, 2007, 1560, 1624, 1561, 1688, 1625, 1562, 1752, 1689, 1626, 1563, 1753, 1690, 1627, 1754, 1691, 1755, 1308, 1372, 1309, 1436, 1373, 1310, 1500, 1437, 1374, 1311, 1501, 1438, 1375, 1502, 1439, 1503, 1816, 1880, 1817, 1944, 1881, 1818, 2008, 1945, 1882, 1819, 2009, 1946,
  1883, 2010, 1947, 2011, 1564, 1628, 1565, 1692, 1629, 1566, 1756, 1693, 1630, 1567, 1757, 1694, 1631, 1758, 1695, 1759, 1820, 1884, 1821, 1948, 1885, 1822, 2012, 1949, 1886, 1823, 2013, 1950, 1887, 2014, 1951, 2015, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
};
}
