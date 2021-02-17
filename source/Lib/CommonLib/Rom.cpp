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

/** \file     Rom.cpp
    \brief    global variables & functions
*/

#include "Rom.h"
#include "UnitTools.h"

#include <memory.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iomanip>

// ====================================================================================================================
// Initialize / destroy functions
// ====================================================================================================================

#if ENABLE_TRACING
CDTrace *g_trace_ctx = NULL;
std::string sTracingRule="D_HEADER:poc<=1";
std::string sTracingFile="tracefile_dec.txt";
bool   bTracingChannelsList = false;
#endif

#if ENABLE_TIME_PROFILING
TimeProfiler *g_timeProfiler = nullptr;
#elif ENABLE_TIME_PROFILING_EXTENDED
TimeProfiler2D *g_timeProfiler = nullptr;
#endif

//! \ingroup CommonLib
//! \{

std::atomic<int> romInitialized(0);

MsgLevel g_verbosity = VERBOSE;

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
  case NAL_UNIT_RESERVED_IRAP_VCL_12:
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
  const CoeffScanType m_scanType;

public:
  ScanGenerator(uint32_t blockWidth, uint32_t blockHeight, uint32_t stride, CoeffScanType scanType)
    : m_line(0), m_column(0), m_blockWidth(blockWidth), m_blockHeight(blockHeight), m_stride(stride), m_scanType(scanType)
  { }

  uint32_t GetCurrentX() const { return m_column; }
  uint32_t GetCurrentY() const { return m_line; }

  uint32_t GetNextIndex(uint32_t blockOffsetX, uint32_t blockOffsetY)
  {
    const uint32_t rtn = ((m_line + blockOffsetY) * m_stride) + m_column + blockOffsetX;

    //advance line and column to the next position
    switch (m_scanType)
    {
      //------------------------------------------------

      case SCAN_DIAG:

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
        break;

      //------------------------------------------------

      default:

        THROW("ERROR: Unknown scan type \"" << m_scanType << "\"in ScanGenerator::GetNextIndex");
        break;
    }

    return rtn;
  }
};
const int8_t g_BcwLog2WeightBase = 3;
const int8_t g_BcwWeightBase = (1 << g_BcwLog2WeightBase);
const int8_t g_BcwWeights[BCW_NUM] = { -2, 3, 4, 5, 10 };
const int8_t g_BcwParsingOrder[BCW_NUM] = { 2, 3, 1, 4, 0 };

int8_t getBcwWeight(uint8_t bcwIdx, uint8_t uhRefFrmList)
{
  // Weghts for the model: P0 + w * (P1 - P0) = (1-w) * P0 + w * P1
  // Retuning  1-w for P0 or w for P1
  return (uhRefFrmList == REF_PIC_LIST_0 ? g_BcwWeightBase - g_BcwWeights[bcwIdx] : g_BcwWeights[bcwIdx]);
}

const uint32_t g_log2SbbSize[MAX_LOG2_TU_SIZE_PLUS_ONE][MAX_LOG2_TU_SIZE_PLUS_ONE][2] =
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
#if RExt__HIGH_BIT_DEPTH_SUPPORT || !( ENABLE_SIMD_LOG2 && defined( TARGET_SIMD_X86 ) )
  int c;

#endif
#if RExt__HIGH_BIT_DEPTH_SUPPORT
  {
    c = 64;
    const double s = sqrt((double)c) * (64 << COM16_C806_TRANS_PREC);


    for (int k = 0; k < c; k++)
    {
      for (int n = 0; n < c; n++)
      {
        double w0, v;
        const double PI = 3.14159265358979323846;

        // DCT-II
        w0 = k == 0 ? sqrt(0.5) : 1;
        v = cos(PI*(n + 0.5)*k / c) * w0 * sqrt(2.0 / c);
        short sv = (short)(s * v + (v > 0 ? 0.5 : -0.5));
        if (g_aiT64[0][0][c*c + k*c + n] != sv)
        {
          msg(WARNING, "trap");
        }
      }
    }
  }

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
  if (romInitialized > 0)
  {
    romInitialized++;
    return;
  }
  romInitialized++;

  const SizeIndexInfoLog2 &sizeInfo = g_sizeIdxInfo;

  {
  // initialize scan orders
  for (uint32_t blockHeightIdx = 0; blockHeightIdx < sizeInfo.numAllHeights(); blockHeightIdx++)
  {
    for (uint32_t blockWidthIdx = 0; blockWidthIdx < sizeInfo.numAllWidths(); blockWidthIdx++)
    {
      const uint32_t blockWidth  = sizeInfo.sizeFrom(blockWidthIdx);
      const uint32_t blockHeight = sizeInfo.sizeFrom(blockHeightIdx);
      const uint32_t totalValues = blockWidth * blockHeight;

      //--------------------------------------------------------------------------------------------------

      //non-grouped scan orders

      for (uint32_t scanTypeIndex = 0; scanTypeIndex < SCAN_NUMBER_OF_TYPES; scanTypeIndex++)
      {
        const CoeffScanType scanType = CoeffScanType(scanTypeIndex);

        CHECK( g_scanOrder[SCAN_UNGROUPED][scanType][blockWidthIdx][blockHeightIdx], "already initialized" );
        g_scanOrder [SCAN_UNGROUPED][scanType][blockWidthIdx][blockHeightIdx]    = new uint32_t[totalValues];

        ScanGenerator fullBlockScan(blockWidth, blockHeight, blockWidth, scanType);
        for (uint32_t scanPosition = 0; scanPosition < totalValues; scanPosition++)
        {
          const int rasterPos = fullBlockScan.GetNextIndex( 0, 0 );
          g_scanOrder[SCAN_UNGROUPED][scanType][blockWidthIdx][blockHeightIdx][scanPosition] = rasterPos;
        }
      }

      //--------------------------------------------------------------------------------------------------

      //grouped scan orders
      const uint32_t* log2Sbb        = g_log2SbbSize[ getLog2(blockWidth) ][ getLog2(blockHeight) ];
      const uint32_t  log2CGWidth    = log2Sbb[0];
      const uint32_t  log2CGHeight   = log2Sbb[1];

      const uint32_t  groupWidth     = 1 << log2CGWidth;
      const uint32_t  groupHeight    = 1 << log2CGHeight;
      const uint32_t  widthInGroups  = std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, blockWidth)  >> log2CGWidth;
      const uint32_t  heightInGroups = std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, blockHeight) >> log2CGHeight;

      const uint32_t  groupSize      = groupWidth    * groupHeight;
      const uint32_t  totalGroups    = widthInGroups * heightInGroups;

      for (uint32_t scanTypeIndex = 0; scanTypeIndex < SCAN_NUMBER_OF_TYPES; scanTypeIndex++)
      {
        const CoeffScanType scanType = CoeffScanType(scanTypeIndex);

        CHECK( g_scanOrder[SCAN_GROUPED_4x4][scanType][blockWidthIdx][blockHeightIdx], "already initialized" );
        g_scanOrder[SCAN_GROUPED_4x4][scanType][blockWidthIdx][blockHeightIdx] = new uint32_t[totalValues];
        if ( blockWidth > JVET_C0024_ZERO_OUT_TH || blockHeight > JVET_C0024_ZERO_OUT_TH )
        {
          for (uint32_t i = 0; i < totalValues; i++)
          {
            g_scanOrder     [SCAN_GROUPED_4x4][scanType][blockWidthIdx][blockHeightIdx]   [i] = totalValues - 1;
          }
        }

        ScanGenerator fullBlockScan(widthInGroups, heightInGroups, groupWidth, scanType);

        for (uint32_t groupIndex = 0; groupIndex < totalGroups; groupIndex++)
        {
          const uint32_t groupPositionY  = fullBlockScan.GetCurrentY();
          const uint32_t groupPositionX  = fullBlockScan.GetCurrentX();
          const uint32_t groupOffsetX    = groupPositionX * groupWidth;
          const uint32_t groupOffsetY    = groupPositionY * groupHeight;
          const uint32_t groupOffsetScan = groupIndex     * groupSize;

          ScanGenerator groupScan(groupWidth, groupHeight, blockWidth, scanType);

          for (uint32_t scanPosition = 0; scanPosition < groupSize; scanPosition++)
          {
            const int rasterPos = groupScan.GetNextIndex( groupOffsetX, groupOffsetY );
            g_scanOrder[SCAN_GROUPED_4x4][scanType][blockWidthIdx][blockHeightIdx][groupOffsetScan + scanPosition] = rasterPos;
          }

          fullBlockScan.GetNextIndex(0, 0);
        }
      }

      //--------------------------------------------------------------------------------------------------
    }
  }
  }

  // initialize CoefTopLeftDiagScan8x8 for LFNST
  for( uint32_t blockWidthIdx = 0; blockWidthIdx < sizeInfo.numAllWidths(); blockWidthIdx++ )
  {
    const uint32_t blockWidth = sizeInfo.sizeFrom( blockWidthIdx );

    const static uint8_t g_auiXYDiagScan8x8[ 64 ][ 2 ] =
    {
      { 0, 0 }, { 0, 1 }, { 1, 0 }, { 0, 2 }, { 1, 1 }, { 2, 0 }, { 0, 3 }, { 1, 2 },
      { 2, 1 }, { 3, 0 }, { 1, 3 }, { 2, 2 }, { 3, 1 }, { 2, 3 }, { 3, 2 }, { 3, 3 },
      { 0, 4 }, { 0, 5 }, { 1, 4 }, { 0, 6 }, { 1, 5 }, { 2, 4 }, { 0, 7 }, { 1, 6 },
      { 2, 5 }, { 3, 4 }, { 1, 7 }, { 2, 6 }, { 3, 5 }, { 2, 7 }, { 3, 6 }, { 3, 7 },
      { 4, 0 }, { 4, 1 }, { 5, 0 }, { 4, 2 }, { 5, 1 }, { 6, 0 }, { 4, 3 }, { 5, 2 },
      { 6, 1 }, { 7, 0 }, { 5, 3 }, { 6, 2 }, { 7, 1 }, { 6, 3 }, { 7, 2 }, { 7, 3 },
      { 4, 4 }, { 4, 5 }, { 5, 4 }, { 4, 6 }, { 5, 5 }, { 6, 4 }, { 4, 7 }, { 5, 6 },
      { 6, 5 }, { 7, 4 }, { 5, 7 }, { 6, 6 }, { 7, 5 }, { 6, 7 }, { 7, 6 }, { 7, 7 }
    };
    CHECK( g_coefTopLeftDiagScan8x8[blockWidthIdx], "already initialized" );
    CHECK( g_coefTopLeftDiagScan8x8pos[blockWidthIdx], "already initialized" );
    g_coefTopLeftDiagScan8x8[ blockWidthIdx ] = new uint32_t[64];
    g_coefTopLeftDiagScan8x8pos[ blockWidthIdx ] = new uint8_t[64];
    for( int i = 0; i < 64; i++ )
    {
      g_coefTopLeftDiagScan8x8[ blockWidthIdx ][ i ] = g_auiXYDiagScan8x8[ i ][ 0 ] + g_auiXYDiagScan8x8[ i ][ 1 ] * blockWidth;
      uint8_t val = g_auiXYDiagScan8x8[ i ][ 0 ];
      val <<= 4;
      val += g_auiXYDiagScan8x8[ i ][ 1 ];
      g_coefTopLeftDiagScan8x8pos[ blockWidthIdx ][ i ] = val;
    }
  }

  initGeoTemplate();
}

void destroyROM()
{
  romInitialized--;
  if (romInitialized > 0)
  {
    return;
  }

  const SizeIndexInfoLog2 &sizeInfo = g_sizeIdxInfo;
  // initialize CoefTopLeftDiagScan8x8 for LFNST
  for( uint32_t blockWidthIdx = 0; blockWidthIdx < sizeInfo.numAllWidths(); blockWidthIdx++ )
  {
    delete[] g_coefTopLeftDiagScan8x8[ blockWidthIdx ];
    delete[] g_coefTopLeftDiagScan8x8pos[ blockWidthIdx ];
    g_coefTopLeftDiagScan8x8[ blockWidthIdx ] = nullptr;
    g_coefTopLeftDiagScan8x8pos[ blockWidthIdx ] = nullptr;
  }

  unsigned numWidths  = g_sizeIdxInfo.numAllWidths();
  unsigned numHeights = g_sizeIdxInfo.numAllHeights();

  for (uint32_t groupTypeIndex = 0; groupTypeIndex < SCAN_NUMBER_OF_GROUP_TYPES; groupTypeIndex++)
  {
    for (uint32_t scanOrderIndex = 0; scanOrderIndex < SCAN_NUMBER_OF_TYPES; scanOrderIndex++)
    {
      for (uint32_t blockWidthIdx = 0; blockWidthIdx < numWidths; blockWidthIdx++)
      {
        for (uint32_t blockHeightIdx = 0; blockHeightIdx < numHeights; blockHeightIdx++)
        {
          delete[] g_scanOrder[groupTypeIndex][scanOrderIndex][blockWidthIdx][blockHeightIdx];
          g_scanOrder[groupTypeIndex][scanOrderIndex][blockWidthIdx][blockHeightIdx] = nullptr;
        }
      }
    }
  }

  for( int modeIdx = 0; modeIdx < GEO_NUM_PARTITION_MODE; modeIdx++ )
  {
    delete[] g_GeoParams[modeIdx];
    g_GeoParams[modeIdx] = nullptr;
  }
  delete[] g_GeoParams;
  for( int i = 0; i < GEO_NUM_PRESTORED_MASK; i++ )
  {
    delete[] g_globalGeoWeights   [i];
    g_globalGeoWeights   [i] = nullptr;
  }
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
#if !ENABLE_SIMD_LOG2
int8_t                    g_aucLog2    [MAX_CU_SIZE + 1];
int8_t                    g_aucNextLog2[MAX_CU_SIZE + 1];
int8_t                    g_aucPrevLog2[MAX_CU_SIZE + 1];
#endif

const int                 g_ictModes[2][4] = { { 0, 3, 1, 2 }, { 0, -3, -1, -2 } };

const UnitScale g_miScaling( MIN_CU_LOG2, MIN_CU_LOG2 );


// ====================================================================================================================
// Scanning order & context model mapping
// ====================================================================================================================

// scanning order table
uint32_t* g_scanOrder     [SCAN_NUMBER_OF_GROUP_TYPES][SCAN_NUMBER_OF_TYPES][MAX_LOG2_TU_SIZE_PLUS_ONE][MAX_LOG2_TU_SIZE_PLUS_ONE];
uint32_t* g_coefTopLeftDiagScan8x8[ MAX_CU_SIZE / 2 + 1 ];
uint8_t* g_coefTopLeftDiagScan8x8pos[ MAX_CU_SIZE / 2 + 1 ];

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

#if JVET_R0166_SCALING_LISTS_CHROMA_444
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
#endif



const int g_alfNumCoeff[ALF_NUM_OF_FILTER_TYPES] = { 7, 13, 8 };

void initGeoTemplate()
{
  g_GeoParams = new int16_t*[GEO_NUM_PARTITION_MODE];
  int modeIdx = 0;
  for( int angleIdx = 0; angleIdx < GEO_NUM_ANGLES; angleIdx++ )
  {
    for( int distanceIdx = 0; distanceIdx < GEO_NUM_DISTANCES; distanceIdx++ )
    {
      if( (distanceIdx == 0 && angleIdx >= 16)
        || ((distanceIdx == 2 || distanceIdx == 0) && (g_angle2mask[angleIdx] == 0 || g_angle2mask[angleIdx] == 5))
        || g_angle2mask[angleIdx] == -1 )
        continue;
      g_GeoParams[modeIdx]    = new int16_t[2];
      g_GeoParams[modeIdx][0] = (int16_t)angleIdx;
      g_GeoParams[modeIdx][1] = (int16_t)distanceIdx;
      modeIdx++;
    }
  }
  for (int angleIdx = 0; angleIdx < (GEO_NUM_ANGLES >> 2) + 1; angleIdx++)
  {
    if (g_angle2mask[angleIdx] == -1)
      continue;
    g_globalGeoWeights[g_angle2mask[angleIdx]] = new int16_t[GEO_WEIGHT_MASK_SIZE * GEO_WEIGHT_MASK_SIZE];
  
    int distanceX = angleIdx;
    int distanceY = (distanceX + (GEO_NUM_ANGLES >> 2)) % GEO_NUM_ANGLES;
    int16_t rho = (g_Dis[distanceX] << (GEO_MAX_CU_LOG2+1)) + (g_Dis[distanceY] << (GEO_MAX_CU_LOG2 + 1));
    static const int16_t maskOffset = (2*GEO_MAX_CU_SIZE - GEO_WEIGHT_MASK_SIZE) >> 1;
    int index = 0;
    for( int y = 0; y < GEO_WEIGHT_MASK_SIZE; y++ )
    {
      int16_t lookUpY = (((y + maskOffset) << 1) + 1) * g_Dis[distanceY];
      for( int x = 0; x < GEO_WEIGHT_MASK_SIZE; x++, index++ )
      {
        int16_t sx_i = ((x + maskOffset) << 1) + 1;
        int16_t weightIdx = sx_i * g_Dis[distanceX] + lookUpY - rho;
        int weightLinearIdx = 32 + weightIdx;
        g_globalGeoWeights[g_angle2mask[angleIdx]][index] = Clip3(0, 8, (weightLinearIdx + 4) >> 3);
      }
    }
  }

  for( int hIdx = 0; hIdx < GEO_NUM_CU_SIZE; hIdx++ )
  {
    int16_t height = 1 << ( hIdx + GEO_MIN_CU_LOG2);
    for( int wIdx = 0; wIdx < GEO_NUM_CU_SIZE; wIdx++ )
    {
      int16_t width = 1 << (wIdx + GEO_MIN_CU_LOG2);
      for( int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++ )
      {
        int16_t angle         = g_GeoParams[splitDir][0];
        int16_t distance      = g_GeoParams[splitDir][1];
        int16_t offsetX       = (GEO_WEIGHT_MASK_SIZE - width) >> 1;
        int16_t offsetY       = (GEO_WEIGHT_MASK_SIZE - height) >> 1;
        if( distance > 0 )
        {
          if( angle % 16 == 8 || (angle % 16 != 0 && height >= width) )
          {
            offsetY += angle < 16 ? ((distance * (int32_t)height) >> 3) : -((distance * (int32_t)height) >> 3);
          }
          else
          {
            offsetX += angle < 16 ? ((distance * (int32_t)width) >> 3) : -((distance * (int32_t)width) >> 3);
          }
        }
        g_weightOffset[splitDir][hIdx][wIdx][0] = offsetX;
        g_weightOffset[splitDir][hIdx][wIdx][1] = offsetY;
      }
    }
  }
}
int16_t** g_GeoParams;
int16_t*  g_globalGeoWeights   [GEO_NUM_PRESTORED_MASK];
int16_t   g_weightOffset       [GEO_NUM_PARTITION_MODE][GEO_NUM_CU_SIZE][GEO_NUM_CU_SIZE][2];
int8_t    g_angle2mask[GEO_NUM_ANGLES] = { 0, -1, 1, 2, 3, 4, -1, -1, 5, -1, -1, 4, 3, 2, 1, -1, 0, -1, 1, 2, 3, 4, -1, -1, 5, -1, -1, 4, 3, 2, 1, -1 };
int8_t    g_Dis[GEO_NUM_ANGLES] = { 8, 8, 8, 8, 4, 4, 2, 1, 0, -1, -2, -4, -4, -8, -8, -8, -8, -8, -8, -8, -4, -4, -2, -1, 0, 1, 2, 4, 4, 8, 8, 8 };
int8_t    g_angle2mirror[GEO_NUM_ANGLES] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2 };
//! \}
