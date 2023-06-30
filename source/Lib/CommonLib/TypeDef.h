/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2018-2023, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVdeC Authors.
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

/** \file     TypeDef.h
    \brief    Define macros, basic types, new types and enumerations
*/

#pragma once

#ifndef COMMONDEF_H
#error Include CommonDef.h not TypeDef.h
#endif

#include <vector>
#include <sstream>
#include <cstddef>
#include <cstring>
#include <cassert>
#include <mutex>

namespace vvdec
{

#if __SANITIZE_ADDRESS__
// macro to enable workarounds for address-sanitizer false-positives
# define ASAN_WORKAROUND                                  1
#endif

#define RECO_WHILE_PARSE                                  1
#define ALLOW_MIDER_LF_DURING_PICEXT                      1
#define MAX_THREADS                                       64 // maximum number of threads

#define MAX_OUT_OF_ORDER_PICS                             3 // maximum number of pictures, that are reconstructed out of order

#define JVET_O1170_CHECK_BV_AT_DECODER                    0 // For decoder to check if a BV is valid or not

#define DISABLE_CONFROMANCE_CHECK                         1
#define DISABLE_CHECK_NO_OUTPUT_PRIOR_PICS_FLAG           0

#define RPR_FIX                                           1
#define RPR_YUV_OUTPUT                                    1
#define RPR_OUTPUT_MSG                                    1

#define GDR_ADJ                                           1
#define ALF_FIX                                           1

#define TBC                                               0

#define ALF_PRE_TRANSPOSE                                 1

#define JVET_R0270                                        TBC // JVET-S0270: Treating picture with mixed RASL and RADL slices as RASL picture
#define JVET_S0155_EOS_NALU_CHECK                         TBC // JVET-S0155: Constraints on EOS NAL units

// ====================================================================================================================
// NEXT software switches
// ====================================================================================================================

#if 0                                                       // set to 1 to avoid valgrind errors concerning uninitilized memory
#define VALGRIND_MEMCLEAR( ref )                           memset(ref,0,sizeof(ref))
#define VALGRIND_MEMZERO( ref,size )                       memset(ref,0,size)
#else
#define VALGRIND_MEMCLEAR( ref )
#define VALGRIND_MEMZERO( ref,size )
#endif

#ifndef ENABLE_TRACING
#define ENABLE_TRACING                                    0 // DISABLE by default (enable only when debugging)
#endif // ! ENABLE_TRACING

#ifndef ENABLE_TIME_PROFILING
#define ENABLE_TIME_PROFILING                             0 // DISABLED by default (can be enabled by project configuration or make command) 
#endif
#ifndef ENABLE_TIME_PROFILING_PIC_TYPES
#define ENABLE_TIME_PROFILING_PIC_TYPES                   0 // DISABLED by default (can be enabled by project configuration or make command) 
#endif
#ifndef ENABLE_TIME_PROFILING_CU_SHAPES
#define ENABLE_TIME_PROFILING_CU_SHAPES                   0 // DISABLED by default (can be enabled by project configuration or make command) 
#endif
#define ENABLE_TIME_PROFILING_EXTENDED                    ( ENABLE_TIME_PROFILING_PIC_TYPES || ENABLE_TIME_PROFILING_CU_SHAPES )

// ====================================================================================================================
// Debugging
// ====================================================================================================================

#define PRINT_MACRO_VALUES                                1 ///< When enabled, the encoder prints out a list of the non-environment-variable controlled macros and their values on startup

// ====================================================================================================================
// Tool Switches - transitory (these macros are likely to be removed in future revisions)
// ====================================================================================================================

#define DECODER_CHECK_SUBSTREAM_AND_SLICE_TRAILING_BYTES  1 ///< TODO: integrate this macro into a broader conformance checking system.
#define U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI    1 ///< Alternative transfer characteristics SEI message (JCTVC-U0033, with syntax naming from V1005)

// ====================================================================================================================
// Tool Switches
// ====================================================================================================================


// This can be enabled by the makefile
#ifndef RExt__HIGH_BIT_DEPTH_SUPPORT
#define RExt__HIGH_BIT_DEPTH_SUPPORT                      0 ///< 0 (default) use data type definitions for 8-10 bit video, 1 = use larger data types to allow for up to 16-bit video (originally developed as part of N0188)
#endif

// SIMD optimizations
#define SIMD_ENABLE                                       1
#define ENABLE_SIMD_OPT                                 ( SIMD_ENABLE && !RExt__HIGH_BIT_DEPTH_SUPPORT )    ///< SIMD optimizations, no impact on RD performance
#define ENABLE_SIMD_OPT_MCIF                            ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for the interpolation filter, no impact on RD performance
#define ENABLE_SIMD_OPT_BUFFER                          ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for the buffer operations, no impact on RD performance
#define ENABLE_SIMD_OPT_DIST                            ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for the distortion calculations(SAD,SSE,HADAMARD), no impact on RD performance
#define ENABLE_SIMD_OPT_ALF                             ( 1 && ENABLE_SIMD_OPT /*&& !ALF_FIX*/ )                            ///< SIMD optimization for ALF
#define ENABLE_SIMD_OPT_INTRAPRED                       ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for Intra Prediction
#define ENABLE_SIMD_OPT_QUANT                           ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for Quant/Dequant
#if ENABLE_SIMD_OPT_BUFFER
#define ENABLE_SIMD_OPT_GBI                               1                                                 ///< SIMD optimization for GBi
#endif
#define ENABLE_SIMD_OPT_INTER                           ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for BIO
#define ENABLE_SIMD_OPT_PICTURE                         ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for Picture Padding
#define ENABLE_SIMD_OPT_SAO                             ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for BIO
#define ENABLE_SIMD_TCOEFF_OPS                          ( 1 && ENABLE_SIMD_OPT )
#define ENABLE_SIMD_LOG2                                ( 1 && ENABLE_SIMD_OPT )
#define ENABLE_SIMD_DBLF                                ( 1 && ENABLE_SIMD_OPT )

#if defined( TARGET_SIMD_X86 ) && !defined( REAL_TARGET_X86 )
#  define SIMD_EVERYWHERE_EXTENSION_LEVEL                 SSE42
#endif

// End of SIMD optimizations

#define LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET         1 /// JVET-L0414 (CE11.2.2) with explicit signalling of num interval, threshold and qpOffset

// ====================================================================================================================
// Derived macros
// ====================================================================================================================

#if RExt__HIGH_BIT_DEPTH_SUPPORT
#define FULL_NBIT                                         1 ///< When enabled, use distortion measure derived from all bits of source data, otherwise discard (bitDepth - 8) least-significant bits of distortion
#else
#define FULL_NBIT                                         1 ///< When enabled, use distortion measure derived from all bits of source data, otherwise discard (bitDepth - 8) least-significant bits of distortion
#endif

#if FULL_NBIT
#define DISTORTION_PRECISION_ADJUSTMENT(x)                0
#else
#define DISTORTION_ESTIMATION_BITS                        8
#define DISTORTION_PRECISION_ADJUSTMENT(x)                ((x>DISTORTION_ESTIMATION_BITS)? ((x)-DISTORTION_ESTIMATION_BITS) : 0)
#endif

// ====================================================================================================================
// Error checks
// ====================================================================================================================


// ====================================================================================================================
// Named numerical types
// ====================================================================================================================

#if RExt__HIGH_BIT_DEPTH_SUPPORT
typedef       int             Pel;               ///< pixel type
typedef       int64_t         TCoeff;            ///< transform coefficient
typedef       int             TMatrixCoeff;      ///< transform matrix coefficient
typedef       int16_t         TFilterCoeff;      ///< filter coefficient
typedef       int64_t         Intermediate_Int;  ///< used as intermediate value in calculations
#else
typedef       int16_t         Pel;               ///< pixel type
typedef       int             TCoeff;            ///< transform coefficient
typedef       int16_t         TCoeffSig;
typedef       int16_t         TMatrixCoeff;      ///< transform matrix coefficient
typedef       int16_t         TFilterCoeff;      ///< filter coefficient
typedef       int             Intermediate_Int;  ///< used as intermediate value in calculations
#endif

typedef       uint32_t        Distortion;        ///< distortion measurement

typedef       uint16_t        SplitSeries;       ///< used to encoded the splits that caused a particular CU size

// ====================================================================================================================
// Enumeration
// ====================================================================================================================

enum ApsTypeValues
{
  ALF_APS  = 0,
  LMCS_APS = 1,
  SCALING_LIST_APS = 2,
};

// EMT transform tags
enum TransType : uint8_t
{
  DCT2           = 0,
  DCT8           = 1,
  DST7           = 2,
  NUM_TRANS_TYPE
};

enum MTSIdx : uint8_t
{
  MTS_DCT2_DCT2 = 0,
  MTS_SKIP = 1,
  MTS_DST7_DST7 = 2,
  MTS_DCT8_DST7 = 3,
  MTS_DST7_DCT8 = 4,
  MTS_DCT8_DCT8 = 5
};

enum ISPType : uint8_t
{
  NOT_INTRA_SUBPARTITIONS = 0,
  HOR_INTRA_SUBPARTITIONS = 1,
  VER_INTRA_SUBPARTITIONS = 2,
  NUM_INTRA_SUBPARTITIONS_MODES
};

enum SbtIdx : uint8_t
{
  SBT_OFF_DCT  = 0,
  SBT_VER_HALF = 1,
  SBT_HOR_HALF = 2,
  SBT_VER_QUAD = 3,
  SBT_HOR_QUAD = 4,
  NUMBER_SBT_IDX
};

enum SbtPos : uint8_t
{
  SBT_POS0 = 0,
  SBT_POS1 = 1,
  NUMBER_SBT_POS
};

/// supported slice type
enum SliceType
{
  B_SLICE               = 0,
  P_SLICE               = 1,
  I_SLICE               = 2,
  NUMBER_OF_SLICE_TYPES
};

/// chroma formats (according to semantics of chroma_format_idc)
enum ChromaFormat : uint8_t
{
  CHROMA_400        = 0,
  CHROMA_420        = 1,
  CHROMA_422        = 2,
  CHROMA_444        = 3,
  NUM_CHROMA_FORMAT
};

enum ChannelType : uint8_t
{
  CHANNEL_TYPE_LUMA    = 0,
  CHANNEL_TYPE_CHROMA  = 1,
  MAX_NUM_CHANNEL_TYPE
};

enum TreeType : uint8_t
{
  TREE_D = 0, //default tree status (for single-tree slice, TREE_D means joint tree; for dual-tree I slice, TREE_D means TREE_L for luma and TREE_C for chroma)
  TREE_L = 1, //separate tree only contains luma (may split)
  TREE_C = 2, //separate tree only contains chroma (not split), to avoid small chroma block
};

enum ModeType : uint8_t
{
  MODE_TYPE_ALL = 0, //all modes can try
  MODE_TYPE_INTER = 1, //can try inter
  MODE_TYPE_INTRA = 2, //can try intra, ibc, palette
};

#define CH_L CHANNEL_TYPE_LUMA
#define CH_C CHANNEL_TYPE_CHROMA

enum ComponentID : uint8_t
{
  COMPONENT_Y         = 0,
  COMPONENT_Cb        = 1,
  COMPONENT_Cr        = 2,
  MAX_NUM_COMPONENT,
  JOINT_CbCr          = MAX_NUM_COMPONENT,
  MAX_NUM_TBLOCKS     = MAX_NUM_COMPONENT
};

#define MAP_CHROMA(c) (ComponentID(c))

enum DeblockEdgeDir : uint8_t
{
  EDGE_VER     = 0,
  EDGE_HOR     = 1,
  NUM_EDGE_DIR
};

/// supported prediction type
enum PredMode : uint8_t
{
  MODE_INTER                 = 0,     ///< inter-prediction mode
  MODE_INTRA                 = 1,     ///< intra-prediction mode
  MODE_IBC                   = 2,     ///< ibc-prediction mode
  NUMBER_OF_PREDICTION_MODES
};

/// reference list index
enum RefPicList : uint8_t
{
  REF_PIC_LIST_0               = 0,   ///< reference list 0
  REF_PIC_LIST_1               = 1,   ///< reference list 1
  NUM_REF_PIC_LIST_01          = 2,
  REF_PIC_LIST_X               = 100  ///< special mark
};

#define L0 REF_PIC_LIST_0
#define L1 REF_PIC_LIST_1

/// distortion function index
enum DFunc
{
  DF_SAD8            = 0,      ///<   8xM SAD
  DF_SAD16           = 1,      ///<  16xM SAD

  DF_TOTAL_FUNCTIONS = DF_SAD16 + 1
};

/// motion vector predictor direction used in AMVP
enum MvpDir : uint8_t
{
  MD_LEFT = 0,          ///< MVP of left block
  MD_ABOVE,             ///< MVP of above block
  MD_ABOVE_RIGHT,       ///< MVP of above right block
  MD_BELOW_LEFT,        ///< MVP of below left block
  MD_ABOVE_LEFT         ///< MVP of above left block
};

enum CoeffScanGroupType
{
  SCAN_UNGROUPED   = 0,
  SCAN_GROUPED_4x4 = 1,
  SCAN_NUMBER_OF_GROUP_TYPES = 2
};

enum ScalingListSize
{
  SCALING_LIST_1x1 = 0,
  SCALING_LIST_2x2,
  SCALING_LIST_4x4,
  SCALING_LIST_8x8,
  SCALING_LIST_16x16,
  SCALING_LIST_32x32,
  SCALING_LIST_64x64,
  SCALING_LIST_SIZE_NUM,
  //for user define matrix
  SCALING_LIST_FIRST_CODED = SCALING_LIST_2x2,
  SCALING_LIST_LAST_CODED = SCALING_LIST_64x64
};

enum ScalingList1dStartIdx
{
  SCALING_LIST_1D_START_2x2    = 0,
  SCALING_LIST_1D_START_4x4    = 2,
  SCALING_LIST_1D_START_8x8    = 8,
  SCALING_LIST_1D_START_16x16  = 14,
  SCALING_LIST_1D_START_32x32  = 20,
  SCALING_LIST_1D_START_64x64  = 26,
};

enum SAOMode //mode
{
  SAO_MODE_OFF = 0,
  SAO_MODE_NEW,
  SAO_MODE_MERGE,
  NUM_SAO_MODES
};

enum SAOModeMergeTypes
{
  SAO_MERGE_LEFT =0,
  SAO_MERGE_ABOVE,
  NUM_SAO_MERGE_TYPES
};


enum SAOModeNewTypes
{
  SAO_TYPE_START_EO =0,
  SAO_TYPE_EO_0 = SAO_TYPE_START_EO,
  SAO_TYPE_EO_90,
  SAO_TYPE_EO_135,
  SAO_TYPE_EO_45,

  SAO_TYPE_START_BO,
  SAO_TYPE_BO = SAO_TYPE_START_BO,

  NUM_SAO_NEW_TYPES
};
#define NUM_SAO_EO_TYPES_LOG2 2

enum SAOEOClasses
{
  SAO_CLASS_EO_FULL_VALLEY = 0,
  SAO_CLASS_EO_HALF_VALLEY = 1,
  SAO_CLASS_EO_PLAIN       = 2,
  SAO_CLASS_EO_HALF_PEAK   = 3,
  SAO_CLASS_EO_FULL_PEAK   = 4,
  NUM_SAO_EO_CLASSES,
};

#define NUM_SAO_BO_CLASSES_LOG2  5
#define NUM_SAO_BO_CLASSES       (1<<NUM_SAO_BO_CLASSES_LOG2)

namespace Profile
{
  enum Name
  {
    NONE                                 = 0,
    STILL_PICTURE                        = 64,
    MAIN_10                              = 1,
    MAIN_10_STILL_PICTURE                = MAIN_10 | STILL_PICTURE,
    MULTILAYER_MAIN_10                   = 17,
    MULTILAYER_MAIN_10_STILL_PICTURE     = MULTILAYER_MAIN_10 | STILL_PICTURE,
    MAIN_10_444                          = 33,
    MAIN_10_444_STILL_PICTURE            = MAIN_10_444 | STILL_PICTURE,
    MULTILAYER_MAIN_10_444               = 49,
    MULTILAYER_MAIN_10_444_STILL_PICTURE = MULTILAYER_MAIN_10_444 | STILL_PICTURE,
  };
}

enum Tier
{
  MAIN = 0,
  HIGH = 1,
  NUMBER_OF_TIERS=2
};

enum SPSExtensionFlagIndex
{
  SPS_EXT__REXT           = 0,
//SPS_EXT__MVHEVC         = 1, //for use in future versions
//SPS_EXT__SHVC           = 2, //for use in future versions
  SPS_EXT__NEXT           = 3,
  NUM_SPS_EXTENSION_FLAGS = 8
};

enum PPSExtensionFlagIndex
{
  PPS_EXT__REXT           = 0,
//PPS_EXT__MVHEVC         = 1, //for use in future versions
//PPS_EXT__SHVC           = 2, //for use in future versions
  NUM_PPS_EXTENSION_FLAGS = 8
};

// TODO: Existing names used for the different NAL unit types can be altered to better reflect the names in the spec.
//       However, the names in the spec are not yet stable at this point. Once the names are stable, a cleanup
//       effort can be done without use of macros to alter the names used to indicate the different NAL unit types.
enum NalUnitType
{
  NAL_UNIT_CODED_SLICE_TRAIL = 0,   // 0
  NAL_UNIT_CODED_SLICE_STSA,        // 1
  NAL_UNIT_CODED_SLICE_RADL,        // 2
  NAL_UNIT_CODED_SLICE_RASL,        // 3

  NAL_UNIT_RESERVED_VCL_4,
  NAL_UNIT_RESERVED_VCL_5,
  NAL_UNIT_RESERVED_VCL_6,

  NAL_UNIT_CODED_SLICE_IDR_W_RADL,  // 7
  NAL_UNIT_CODED_SLICE_IDR_N_LP,    // 8
  NAL_UNIT_CODED_SLICE_CRA,         // 9
  NAL_UNIT_CODED_SLICE_GDR,         // 10

  NAL_UNIT_RESERVED_IRAP_VCL_11,

  NAL_UNIT_OPI,                     // 12
  NAL_UNIT_DCI,                     // 13
  NAL_UNIT_VPS,                     // 14
  NAL_UNIT_SPS,                     // 15
  NAL_UNIT_PPS,                     // 16
  NAL_UNIT_PREFIX_APS,              // 17
  NAL_UNIT_SUFFIX_APS,              // 18
  NAL_UNIT_PH,                      // 19
  NAL_UNIT_ACCESS_UNIT_DELIMITER,   // 20
  NAL_UNIT_EOS,                     // 21
  NAL_UNIT_EOB,                     // 22
  NAL_UNIT_PREFIX_SEI,              // 23
  NAL_UNIT_SUFFIX_SEI,              // 24
  NAL_UNIT_FD,                      // 25

  NAL_UNIT_RESERVED_NVCL_26,
  NAL_UNIT_RESERVED_NVCL_27,

  NAL_UNIT_UNSPECIFIED_28,
  NAL_UNIT_UNSPECIFIED_29,
  NAL_UNIT_UNSPECIFIED_30,
  NAL_UNIT_UNSPECIFIED_31,
  NAL_UNIT_INVALID
};

enum MergeType : uint8_t
{
  MRG_TYPE_DEFAULT_N        = 0, // 0
  MRG_TYPE_SUBPU_ATMVP,
  MRG_TYPE_IBC,
  NUM_MRG_TYPE                   // 5
};

enum TriangleSplit : uint8_t
{
  TRIANGLE_DIR_135 = 0,
  TRIANGLE_DIR_45,
  TRIANGLE_DIR_NUM
};

enum AffineModel : uint8_t
{
  AFFINEMODEL_4PARAM,
  AFFINEMODEL_6PARAM,
  AFFINE_MODEL_NUM
};

enum ImvMode : uint8_t
{
  IMV_OFF = 0,
  IMV_FPEL,
  IMV_4PEL,
  IMV_HPEL,
  NUM_IMV_MODES
};


// ====================================================================================================================
// Type definition
// ====================================================================================================================

/// parameters for adaptive loop filter
class PicSym;

#define MAX_NUM_SAO_CLASSES  32  //(NUM_SAO_EO_GROUPS > NUM_SAO_BO_GROUPS)?NUM_SAO_EO_GROUPS:NUM_SAO_BO_GROUPS

struct SAOOffset
{
  SAOMode modeIdc;       // NEW, MERGE, OFF
  int     typeIdc;       // union of SAOModeMergeTypes and SAOModeNewTypes, depending on modeIdc.
  int     typeAuxInfo;   // BO: starting band index
  int     offset[MAX_NUM_SAO_CLASSES];

  void reset()
  {
    modeIdc     = SAO_MODE_OFF;
    typeIdc     = -1;
    typeAuxInfo = -1;
    ::memset( offset, 0, sizeof( int ) * MAX_NUM_SAO_CLASSES );
  }

  const SAOOffset& operator=( const SAOOffset& src )
  {
    modeIdc     = src.modeIdc;
    typeIdc     = src.typeIdc;
    typeAuxInfo = src.typeAuxInfo;
    ::memcpy( offset, src.offset, sizeof( int ) * MAX_NUM_SAO_CLASSES );

    return *this;
  }
};

struct SAOBlkParam
{
  void reset()
  {
    for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
    {
      offsetParam[compIdx].reset();
    }
  }

  const SAOBlkParam& operator=( const SAOBlkParam& src )
  {
    for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
    {
      offsetParam[compIdx] = src.offsetParam[compIdx];
    }
    return *this;
  }

  SAOOffset&       operator[]( int compIdx ) { return offsetParam[compIdx]; }
  const SAOOffset& operator[]( int compIdx ) const { return offsetParam[compIdx]; }

private:
  SAOOffset offsetParam[MAX_NUM_COMPONENT];
};

struct BitDepths
{
  int recon[MAX_NUM_CHANNEL_TYPE] = {8, 8}; ///< the bit depth as indicated in the SPS
};

/// parameters for deblocking filter
struct LFCUParam
{
//  bool internalEdge;                     ///< indicates internal edge
  bool leftEdge;                         ///< indicates left edge
  bool topEdge;                          ///< indicates top edge
};


struct SEITimeSet
{
  SEITimeSet() : clockTimeStampFlag(false),
                     numUnitFieldBasedFlag(false),
                     countingType(0),
                     fullTimeStampFlag(false),
                     discontinuityFlag(false),
                     cntDroppedFlag(false),
                     numberOfFrames(0),
                     secondsValue(0),
                     minutesValue(0),
                     hoursValue(0),
                     secondsFlag(false),
                     minutesFlag(false),
                     hoursFlag(false),
                     timeOffsetLength(0),
                     timeOffsetValue(0)
  { }
  bool clockTimeStampFlag;
  bool numUnitFieldBasedFlag;
  int  countingType;
  bool fullTimeStampFlag;
  bool discontinuityFlag;
  bool cntDroppedFlag;
  int  numberOfFrames;
  int  secondsValue;
  int  minutesValue;
  int  hoursValue;
  bool secondsFlag;
  bool minutesFlag;
  bool hoursFlag;
  int  timeOffsetLength;
  int  timeOffsetValue;
};

struct SEIMasteringDisplay
{
  uint32_t maxLuminance;
  uint32_t minLuminance;
  uint16_t primaries[3][2];
  uint16_t whitePoint[2];
};

class ChromaCbfs
{
public:
  ChromaCbfs()
    : Cb( false ), Cr( false )
  {}
public:
  bool sigChroma( ChromaFormat chromaFormat ) const
  {
    if( chromaFormat == CHROMA_400 )
    {
      return false;
    }
    return   ( Cb || Cr );
  }
  bool& cbf( ComponentID compID )
  {
    bool *cbfs[MAX_NUM_TBLOCKS] = { nullptr, &Cb, &Cr };

    return *cbfs[compID];
  }
public:
  bool Cb;
  bool Cr;
};

struct LoopFilterParam
{
  int8_t   qp[3];
  uint8_t  bs;
  uint8_t  sideMaxFiltLength;
  uint8_t  flags;

  bool filterEdge( ChannelType chType ) const { return ( flags >> chType ) & 1; }
  // chroma max filter lenght
  bool filterCMFL()                     const { return ( flags >>      5 ) & 1; }

  void setFilterEdge( ChannelType chType, int f ) { flags = ( flags & ~( 1 << chType ) ) | ( f << chType ); }
  void setFilterCMFL(                     int f ) { flags = ( flags & ~( 1 <<      5 ) ) | ( f <<      5 ); }
};

enum MsgLevel
{
  SILENT  = 0,
  ERROR   = 1,
  WARNING = 2,
  INFO    = 3,
  NOTICE  = 4,
  VERBOSE = 5,
  DETAILS = 6
};

enum ErrHandlingFlags
{
  // Keep in sync with vvdecErrHandlingFlags from vvdec.h
  ERR_HANDLING_OFF          = 0,   // no special internal error-handling besides tuning in to GDR streams
  ERR_HANDLING_TRY_CONTINUE = 1,   // try to continue decoding after parsing errors or missing pictures
#if 0
  // NOT YET IMPLEMENTED
  ERR_HANDLING_COPY_CLOSEST = 2    // replace missing reference pictures with the closest available picture (otherwise grey frame)
#endif
};

// ---------------------------------------------------------------------------
// exception class
// ---------------------------------------------------------------------------

#ifdef assert
#undef assert
#endif

#define assert dont_use_assert_use_CHECK_instead

class Exception : public std::exception
{
public:
  explicit Exception( const std::string& _s ) : m_str( _s ) {}
  virtual ~Exception() noexcept = default;

  virtual const char* what() const noexcept                 { return m_str.c_str(); }
  template<typename T> Exception& operator<<( const T& t )  { std::ostringstream oss; oss << t; m_str += oss.str(); return *this; }
private:
  std::string m_str;
};

class RecoverableException : public Exception
{
public:
  explicit RecoverableException( const std::string& _s ) : Exception( _s ) {}
  virtual ~RecoverableException() noexcept = default;

  RecoverableException& operator=( const RecoverableException& _e )        { Exception::operator=( _e ); return *this; }
  template<typename T> RecoverableException& operator<<( const T& t )      { Exception::operator<<( t ); return *this; }
};

class UnsupportedFeatureException : public Exception
{
public:
  explicit UnsupportedFeatureException( const std::string& _s ) : Exception( _s ) {}
  UnsupportedFeatureException( const UnsupportedFeatureException& _e )                   = default;
  virtual ~UnsupportedFeatureException() noexcept                                 = default;
  UnsupportedFeatureException& operator=( const UnsupportedFeatureException& _e )        { Exception::operator=( _e ); return *this; }
  template<typename T> UnsupportedFeatureException& operator<<( const T& t )      { Exception::operator<<( t ); return *this; }
};

#if !defined( __PRETTY_FUNCTION__ ) && !defined( __GNUC__ )
# define __PRETTY_FUNCTION__ __FUNCSIG__
#endif

#if !NDEBUG   // for non MSVC compiler, define _DEBUG if in debug mode to have same behavior between MSVC and others in debug
#  ifndef _DEBUG
#    define _DEBUG 1
#  endif
#endif

// if a check fails with THROW or CHECK, please check if ported correctly from assert in revision 1196)
#define THROW(x)               throw( Exception( "\nERROR: In function \"" ) << __PRETTY_FUNCTION__ << "\" in " << __FILE__ << ":" << __LINE__ << ": " << x )
#define ABORT(x)               { std::cerr << "\nERROR: In function \"" << __FUNCTION__ << "\" in " << __FILE__ << ":" << __LINE__ << ": " << x << std::endl; abort(); }
#define THROW_RECOVERABLE(x)   throw( RecoverableException( "\nERROR: In function \"" ) << __PRETTY_FUNCTION__ << "\" in " << __FILE__ << ":" << __LINE__ << ": " << x )
#define THROW_UNSUPPORTED(x)   throw( UnsupportedFeatureException( "\nERROR: In function \"" ) << __PRETTY_FUNCTION__ << "\" in " << __FILE__ << ":" << __LINE__ << ": " << x )
#define CHECK(c,x)             if(c){ THROW( x << "\nERROR CONDITION: " << #c ); }
#define CHECK_RECOVERABLE(c,x) if(c){ THROW_RECOVERABLE( x << "\nERROR CONDITION: " << #c ); }
#define CHECK_WARN(c,x)        if(c){ std::cerr << "\nWARNING: In function \"" << __PRETTY_FUNCTION__ << "\" in " << __FILE__ << ":" << __LINE__ << ": " << x << "\nERROR CONDITION: " << #c << std::endl; }
#define CHECK_UNSUPPORTED(c,x) if(c){ THROW_UNSUPPORTED( x << "\nERROR CONDITION: " << #c ); }
#define EXIT(x)                throw( Exception( "\n" ) << x << "\n" )
#define CHECK_NULLPTR(_ptr)    CHECK( !( _ptr ), "Accessing an empty pointer!" )

#if defined( _DEBUG )
#define CHECKD(c,x)            if(c){ ABORT( x << "\nERROR CONDITION: " << #c ); }
#else
#define CHECKD(c,x)
#endif   // _DEBUG

#define CHECKD_NULLPTR( _ptr ) CHECKD( !( _ptr ), "Accessing an empty pointer!" )

// ---------------------------------------------------------------------------
// static vector
// ---------------------------------------------------------------------------

template<typename T, size_t N>
class static_vector
{
  T _arr[ N ];
  size_t _size = 0;

public:

  typedef T         value_type;
  typedef size_t    size_type;
  typedef ptrdiff_t difference_type;
  typedef T&        reference;
  typedef T const&  const_reference;
  typedef T*        pointer;
  typedef T const*  const_pointer;
  typedef T*        iterator;
  typedef T const*  const_iterator;

  static const size_type max_num_elements = N;

  static_vector()                                        = default;
  static_vector( const static_vector<T, N>& )            = default;
  static_vector( static_vector<T, N>&& )                 = default;
  static_vector& operator=( const static_vector<T, N>& ) = default;
  static_vector& operator=( static_vector<T, N>&& )      = default;

  static_vector( size_t N_ ) : _size( N_ )                     { CHECKD( _size > N, "capacity exceeded" ); }
  static_vector( size_t N_, const T& _val ) : _size( 0 )       { resize( N_, _val ); }
  template<typename It>
  static_vector( It _it1, It _it2 ) : _size( 0 )               { while( _it1 < _it2 ) _arr[ _size++ ] = *_it1++; }
  template<typename Iterable,
           typename IS_ITERABLE = decltype( std::cbegin( std::declval<Iterable>() ), std::cend( std::declval<Iterable>() ) )>
  explicit static_vector( const Iterable& iterable ) : _size( 0 ) { for( auto& e: iterable ) { push_back( e ); } }

  static_vector( std::initializer_list<T> _il ) : _size( 0 )
  {
    typename std::initializer_list<T>::iterator _src1 = _il.begin();
    typename std::initializer_list<T>::iterator _src2 = _il.end();

    while( _src1 < _src2 ) _arr[ _size++ ] = *_src1++;

    CHECKD( _size > N, "capacity exceeded" );
  }
  static_vector& operator=( std::initializer_list<T> _il )
  {
    _size = 0;

    typename std::initializer_list<T>::iterator _src1 = _il.begin();
    typename std::initializer_list<T>::iterator _src2 = _il.end();

    while( _src1 < _src2 ) _arr[ _size++ ] = *_src1++;

    CHECKD( _size > N, "capacity exceeded" );
    return *this;
  }

  void resize_noinit( size_t N_ )               { CHECKD( N_ > N, "capacity exceeded" ); _size = N_; }
  void resize( size_t N_ )                      { CHECKD( N_ > N, "capacity exceeded" ); while(_size < N_) _arr[ _size++ ] = T() ; _size = N_; }
  void resize( size_t N_, const T& _val )       { CHECKD( N_ > N, "capacity exceeded" ); while(_size < N_) _arr[ _size++ ] = _val; _size = N_; }
  void reserve( size_t N_ )                     { CHECKD( N_ > N, "capacity exceeded" ); }
  void push_back( const T& _val )               { CHECKD( _size >= N, "capacity exceeded" ); _arr[ _size++ ] = _val; }
  void push_back( T&& val )                     { CHECKD( _size >= N, "capacity exceeded" ); _arr[ _size++ ] = std::forward<T>( val ); }
  void pop_back()                               { CHECKD( _size == 0, "calling pop_back on an empty vector" ); _size--; }
  void pop_front()                              { CHECKD( _size == 0, "calling pop_front on an empty vector" ); _size--; for( int i = 0; i < _size; i++ ) _arr[i] = _arr[i + 1]; }
  void clear()                                  { _size = 0; }
  reference       at( size_t _i )               { CHECKD( _i >= _size, "Trying to access an out-of-bound-element" ); return _arr[ _i ]; }
  const_reference at( size_t _i ) const         { CHECKD( _i >= _size, "Trying to access an out-of-bound-element" ); return _arr[ _i ]; }
  reference       operator[]( size_t _i )       { CHECKD( _i >= _size, "Trying to access an out-of-bound-element" ); return _arr[ _i ]; }
  const_reference operator[]( size_t _i ) const { CHECKD( _i >= _size, "Trying to access an out-of-bound-element" ); return _arr[ _i ]; }
  reference       front()                       { CHECKD( _size == 0, "Trying to access the first element of an empty vector" ); return _arr[ 0 ]; }
  const_reference front() const                 { CHECKD( _size == 0, "Trying to access the first element of an empty vector" ); return _arr[ 0 ]; }
  reference       back()                        { CHECKD( _size == 0, "Trying to access the last element of an empty vector" );  return _arr[ _size - 1 ]; }
  const_reference back() const                  { CHECKD( _size == 0, "Trying to access the last element of an empty vector" );  return _arr[ _size - 1 ]; }
  pointer         data()                        { return _arr; }
  const_pointer   data() const                  { return _arr; }
  iterator        begin()                       { return _arr; }
  const_iterator  begin() const                 { return _arr; }
  const_iterator  cbegin() const                { return _arr; }
  iterator        end()                         { return _arr + _size; }
  const_iterator  end() const                   { return _arr + _size; };
  const_iterator  cend() const                  { return _arr + _size; };
  size_type       size() const                  { return _size; };
  size_type       byte_size() const             { return _size * sizeof( T ); }
  bool            empty() const                 { return _size == 0; }

  size_type       capacity() const              { return N; }
  size_type       max_size() const              { return N; }
  size_type       byte_capacity() const         { return sizeof(_arr); }

  void            erase( const_iterator _pos )  { iterator it   = begin() + ( _pos - 1 - begin() );
                                                  iterator last = end() - 1;
                                                  while( ++it != last ) *it = *( it + 1 );
                                                  _size--; }
};

#define SIGN(x) ( (x) >= 0 ? 1 : -1 )

#define MAX_NUM_ALF_CLASSES             25
#define MAX_NUM_ALF_LUMA_COEFF          13
#define MAX_NUM_ALF_CHROMA_COEFF        7
#define MAX_ALF_FILTER_LENGTH           7
#define MAX_NUM_ALF_COEFF               (MAX_ALF_FILTER_LENGTH * MAX_ALF_FILTER_LENGTH / 2 + 1)
#define MAX_NUM_ALF_TRANSPOSE_ID        4
#define MAX_ALF_PADDING_SIZE            4
#define MAX_NUM_ALF_ALTERNATIVES_CHROMA 8
#define MAX_NUM_CC_ALF_FILTERS          4
#define MAX_NUM_CC_ALF_CHROMA_COEFF     8
#define CCALF_DYNAMIC_RANGE             6
#define CCALF_BITS_PER_COEFF_LEVEL      3

enum AlfFilterType
{
  ALF_FILTER_5 = 0,
  ALF_FILTER_7,
  CC_ALF,
  ALF_NUM_OF_FILTER_TYPES
};

struct AlfSliceParam
{
  bool             enabledFlag        [MAX_NUM_COMPONENT];                          // alf_slice_enable_flag, alf_chroma_idc
  bool             nonLinearFlagLuma;
  bool             nonLinearFlagChroma;
  short            lumaCoeff          [MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF]; // alf_coeff_luma_delta[i][j]
  short            lumaClipp          [MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF]; // alf_clipp_luma_[i][j]
  int              numAlternativesChroma;                                                  // alf_chroma_num_alts_minus_one + 1
  short            chromaCoeff        [MAX_NUM_ALF_ALTERNATIVES_CHROMA * MAX_NUM_ALF_CHROMA_COEFF]; // alf_coeff_chroma[i]
  short            chromaClipp        [MAX_NUM_ALF_ALTERNATIVES_CHROMA * MAX_NUM_ALF_CHROMA_COEFF]; // alf_clipp_chroma[i]
  short            filterCoeffDeltaIdx[MAX_NUM_ALF_CLASSES];                        // filter_coeff_delta[i]
  bool             filterCoeffFlag    [MAX_NUM_ALF_CLASSES];                        // filter_coefficient_flag[i]
  int              numLumaFilters;                                                  // number_of_filters_minus1 + 1
  bool             coeffDeltaFlag;                                                  // alf_coefficients_delta_flag
  
  bool             lumaCoeffSummed = false;
  bool             lumaFinalDone   = false;
  bool             chrmFinalDone   = false;
#if ALF_PRE_TRANSPOSE
  short            lumaCoeffFinal     [MAX_NUM_ALF_TRANSPOSE_ID * MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
  short            lumaClippFinal     [MAX_NUM_ALF_TRANSPOSE_ID * MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
#else
  short            lumaCoeffFinal     [MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
  short            lumaClippFinal     [MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
#endif
  short            chrmClippFinal     [MAX_NUM_ALF_ALTERNATIVES_CHROMA * MAX_NUM_ALF_CHROMA_COEFF];
  int              tLayer;
  bool             newFilterFlag      [MAX_NUM_CHANNEL_TYPE];

  mutable std::mutex recostructMutex;   // this must be the last member, so we can clear the rest of the struct using memset()

  AlfSliceParam()
  {
    static_assert( offsetof( AlfSliceParam, recostructMutex ) + sizeof( recostructMutex ) == sizeof( AlfSliceParam ), "recostructMutex must be last member" );
    static_assert( std::is_standard_layout<AlfSliceParam>::value, "AlfSliceParam must be standard layout type for offsetof to work" );

    GCC_WARNING_DISABLE_class_memaccess
      memset( this, 0, offsetof( AlfSliceParam, recostructMutex ) );
    GCC_WARNING_RESET

    // GH: why are numLumaFilters and numAlternativesChroma not set to 1 as in reset()?
  }

  void reset()
  {
    GCC_WARNING_DISABLE_class_memaccess
      memset( this, 0, offsetof( AlfSliceParam, recostructMutex ) );
    GCC_WARNING_RESET

    numLumaFilters = 1;
    numAlternativesChroma = 1;
  }

  const AlfSliceParam& operator=( const AlfSliceParam& src )
  {
    GCC_WARNING_DISABLE_class_memaccess
      std::memcpy( this, &src, offsetof( AlfSliceParam, recostructMutex ) );
    GCC_WARNING_RESET

    return *this;
  }
};

struct CcAlfFilterParam
{
  bool    ccAlfFilterEnabled[2];
  bool    ccAlfFilterIdxEnabled[2][MAX_NUM_CC_ALF_FILTERS];
  uint8_t ccAlfFilterCount[2];
  short   ccAlfCoeff[2][MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF];
  int     newCcAlfFilter[2];
  int     numberValidComponents;

  CcAlfFilterParam()
  {
    memset( this, 0, sizeof( *this ) );
  }

  void reset()
  {
    std::memset( this, 0, sizeof( CcAlfFilterParam ) );
    ccAlfFilterCount[0] = ccAlfFilterCount[1] = MAX_NUM_CC_ALF_FILTERS;
    numberValidComponents = 3;
    newCcAlfFilter[0] = newCcAlfFilter[1] = 0;
  }
  
  const CcAlfFilterParam& operator = ( const CcAlfFilterParam& src )
  {
    std::memcpy( this, &src, sizeof( CcAlfFilterParam ) );
    return *this;
  }
};

// ---------------------------------------------------------------------------
// general helpers
// ---------------------------------------------------------------------------
// move an element within a list to the list's end without moving or copying the contained object
template<typename TList>
static void move_to_end( typename TList::const_iterator it, TList& list )
{
#ifdef _DEBUG
  const auto* oldAddr = &( *it );
#endif

  list.splice( list.cend(), list, it );

  CHECKD( &list.back() != oldAddr, "moving failed" );
}

}
