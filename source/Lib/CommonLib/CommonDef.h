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

/** \file     CommonDef.h
    \brief    Defines version information, constants and small in-line functions
*/

#pragma once

#define COMMONDEF_H

#include <algorithm>
#include <limits>
#include <cmath>   // needed for std::log2()
#include <cstdarg>
#include <functional>
#include <mutex>

#if defined( __x86_64__ ) || defined( _M_X64 ) || defined( __i386__ ) || defined( __i386 ) || defined( _M_IX86 )
# define REAL_TARGET_X86 1
#elif defined( __aarch64__ ) || defined( _M_ARM64 ) || defined( __arm__ ) || defined( _M_ARM )
# define REAL_TARGET_ARM 1
#elif defined( __wasm__ ) || defined( __wasm32__ )
# define REAL_TARGET_WASM 1
#endif

#ifdef _WIN32
#  include <intrin.h>
#endif

#if defined( __INTEL_COMPILER )
#pragma warning( disable : 1786 )
#pragma warning( disable : 3175 )   // unrecognized gcc optimization level
#pragma warning( disable :  177 )
#endif

#if _MSC_VER > 1000
// disable "signed and unsigned mismatch"
#pragma warning( disable : 4018 )
// disable bool coercion "performance warning"
#pragma warning( disable : 4800 )

#pragma warning( disable : 4996 )
#endif // _MSC_VER > 1000


// MS Visual Studio before 2017 does not support required C++14 features
#ifdef _MSC_VER
#if _MSC_VER < 1910
#error "MS Visual Studio version not supported. Please upgrade to Visual Studio 2017 or higher (or use other compilers)"
#endif
#endif

#define _CRT_SECURE_NO_WARNINGS 1

// macros to selectively disable some usually useful warnings
#if defined __GNUC__ && !defined __clang__
# define GCC_WARNING_RESET                       _Pragma("GCC diagnostic pop");

# define GCC_EXTRA_WARNING_switch_enum           _Pragma("GCC diagnostic push"); _Pragma("GCC diagnostic error \"-Wswitch-enum\"");
#else
# define GCC_WARNING_RESET

# define GCC_EXTRA_WARNING_switch_enum
#endif

#if __GNUC__ >= 8 && !defined __clang__
# define GCC_WARNING_DISABLE_maybe_uninitialized _Pragma("GCC diagnostic push"); _Pragma("GCC diagnostic ignored \"-Wmaybe-uninitialized\"");
# define GCC_WARNING_DISABLE_class_memaccess     _Pragma("GCC diagnostic push"); _Pragma("GCC diagnostic ignored \"-Wclass-memaccess\"");
#else
# define GCC_WARNING_DISABLE_maybe_uninitialized
# define GCC_WARNING_DISABLE_class_memaccess
#endif

#define CLASS_COPY_MOVE_DEFAULT( Class )      \
  Class( const Class& )            = default; \
  Class( Class&& )                 = default; \
  Class& operator=( const Class& ) = default; \
  Class& operator=( Class&& )      = default;

#define CLASS_COPY_MOVE_DELETE( Class )      \
  Class( const Class& )            = delete; \
  Class( Class&& )                 = delete; \
  Class& operator=( const Class& ) = delete; \
  Class& operator=( Class&& )      = delete;

#include "TypeDef.h"
#include "vvdec/version.h"

namespace vvdec
{

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Common constants
// ====================================================================================================================
static const uint64_t MAX_UINT64 = std::numeric_limits<uint64_t>::max();
static const uint32_t MAX_UINT   = std::numeric_limits<uint32_t>::max();
static const int      MAX_INT    = std::numeric_limits<int>     ::max();
static const uint8_t  MAX_UCHAR  = std::numeric_limits<int8_t>  ::max();
static const uint8_t  MAX_SCHAR  = std::numeric_limits<uint8_t> ::max();

// ====================================================================================================================
// Coding tool configuration
// ====================================================================================================================
// Most of these should not be changed - they resolve the meaning of otherwise magic numbers.

static const int MAX_NUM_REF_PICS =                                16; ///< max. number of pictures used for reference
static const int MAX_NUM_REF =                                     16; ///< max. number of entries in picture reference list
static const int MAX_QP =                                          63;
static const int NOT_VALID =                                       -1;
static const int MI_NOT_VALID =                                    -1;
static const int MF_NOT_VALID =                                    -1;
static const int MH_NOT_VALID =                                    -1;
static const int CO_NOT_VALID =                                    -1;

static const int AMVP_MAX_NUM_CANDS =                               2; ///< AMVP: advanced motion vector prediction - max number of final candidates
static const int AMVP_MAX_NUM_CANDS_MEM =                           3; ///< AMVP: advanced motion vector prediction - max number of candidates
static const int AMVP_DECIMATION_FACTOR =                           2;
static const int MRG_MAX_NUM_CANDS =                                6; ///< MERGE
static const int AFFINE_MRG_MAX_NUM_CANDS =                         5; ///< AFFINE MERGE
static const int IBC_MRG_MAX_NUM_CANDS =                            6; ///< IBC MERGE

static const int MAX_TLAYER =                                       7; ///< Explicit temporal layer QP offset - max number of temporal layer

static const int MIN_TB_LOG2_SIZEY = 2;
static const int MAX_TB_LOG2_SIZEY = 6;

static const int MIN_TB_SIZEY = 1 << MIN_TB_LOG2_SIZEY;
static const int MAX_TB_SIZEY = 1 << MAX_TB_LOG2_SIZEY;

static const int MAX_NUM_PICS_IN_SOP =                           1024;

static const int MAX_NESTING_NUM_OPS =                           1024;
static const int MAX_NESTING_NUM_LAYER =                           64;

static const int MAX_VPS_NUM_HRD_PARAMETERS =                       1;
static const int MAX_VPS_LAYERS =                                  64;
static const int MAX_VPS_SUBLAYERS =                                7;
static const int MAX_NUM_OLSS =                                   256;
static const int MAX_VPS_OLS_MODE_IDC =                             2;

static const int MIP_MAX_WIDTH =                                   MAX_TB_SIZEY;
static const int MIP_MAX_HEIGHT =                                  MAX_TB_SIZEY;

static const int ALF_FIXED_FILTER_NUM        =                     64;
static const int ALF_CTB_MAX_NUM_APS         =                      8;
static const int NUM_FIXED_FILTER_SETS       =                     16;
static const int NUM_TOTAL_FILTER_SETS       =                     NUM_FIXED_FILTER_SETS + ALF_CTB_MAX_NUM_APS;

static const int MAX_BDOF_APPLICATION_REGION =                     16;

static const int MAX_CPB_CNT =                                     32; ///< Upper bound of (cpb_cnt_minus1 + 1)
static const int MAX_NUM_LAYER_IDS =                               64;
static const int COEF_REMAIN_BIN_REDUCTION =                        5; ///< indicates the level at which the VLC transitions from Golomb-Rice to TU+EG(k)
static const int CU_DQP_TU_CMAX =                                   5; ///< max number bins for truncated unary
static const int CU_DQP_EG_k =                                      0; ///< expgolomb order

static const int SBH_THRESHOLD =                                    4; ///< value of the fixed SBH controlling threshold

static const int MAX_NUM_VPS =                                     16;
static const int MAX_NUM_DPS =                                     16;
static const int MAX_NUM_SPS =                                     16;
static const int MAX_NUM_PPS =                                     64;
static const int MAX_NUM_APS =                                     32;  //Currently APS ID has 5 bits
static const int NUM_APS_TYPE_LEN =                                 3;  //Currently APS Type has 3 bits
static const int MAX_NUM_APS_TYPE =                                 8;  //Currently APS Type has 3 bits so the max type is 8

static const int MAX_TILE_COLS =                                   20;  ///< Maximum number of tile columns
static const int MAX_TILES =                                      440;  ///< Maximum number of tiles
static const int MAX_SLICES =                                     600;  ///< Maximum number of slices per picture
static const int MLS_GRP_NUM =                                   1024; ///< Max number of coefficient groups, max(16, 256)

static const int MLS_CG_SIZE =                                      4; ///< Coefficient group size of 4x4; = MLS_CG_LOG2_WIDTH + MLS_CG_LOG2_HEIGHT


static const int MAX_REF_LINE_IDX =                                 3; //highest refLine offset in the list
static const int MRL_NUM_REF_LINES =                                3; //number of candidates in the array
static const int MULTI_REF_LINE_IDX[4] =               { 0, 1, 2, 0 };

static const int PRED_REG_MIN_WIDTH =                               4;  // Minimum prediction region width for ISP subblocks

static const int NUM_LUMA_MODE =                                   67; ///< Planar + DC + 65 directional mode (4*16 + 1)
static const int NUM_LMC_MODE =                                    1 + 2; ///< LMC + MDLM_T + MDLM_L
static const int NUM_INTRA_MODE = (NUM_LUMA_MODE + NUM_LMC_MODE);

static const int NUM_EXT_LUMA_MODE =                               28;

static const int NUM_DIR =           (((NUM_LUMA_MODE - 3) >> 2) + 1);
static const int PLANAR_IDX =                                       0; ///< index for intra PLANAR mode
static const int DC_IDX =                                           1; ///< index for intra DC     mode
static const int HOR_IDX =                    (1 * (NUM_DIR - 1) + 2); ///< index for intra HORIZONTAL mode
static const int DIA_IDX =                    (2 * (NUM_DIR - 1) + 2); ///< index for intra DIAGONAL   mode
static const int VER_IDX =                    (3 * (NUM_DIR - 1) + 2); ///< index for intra VERTICAL   mode
static const int VDIA_IDX =                   (4 * (NUM_DIR - 1) + 2); ///< index for intra VDIAGONAL  mode
static const int BDPCM_IDX =                  (5 * (NUM_DIR - 1) + 2); ///< index for intra VDIAGONAL  mode

static const int NUM_CHROMA_MODE = (5 + NUM_LMC_MODE); ///< total number of chroma modes
static const int LM_CHROMA_IDX = NUM_LUMA_MODE; ///< chroma mode index for derived from LM mode
static const int MDLM_L_IDX =                          LM_CHROMA_IDX + 1; ///< MDLM_L
static const int MDLM_T_IDX =                          LM_CHROMA_IDX + 2; ///< MDLM_T
static const int DM_CHROMA_IDX =                       NUM_INTRA_MODE; ///< chroma mode index for derived from luma intra mode

static const uint32_t  MTS_INTRA_MAX_CU_SIZE =                         32; ///< Max Intra CU size applying EMT, supported values: 8, 16, 32, 64, 128
static const uint32_t  MTS_INTER_MAX_CU_SIZE =                         32; ///< Max Inter CU size applying EMT, supported values: 8, 16, 32, 64, 128
static const int NUM_MOST_PROBABLE_MODES = 6;

static const int MAX_NUM_MIP_MODE =                                32; ///< maximum number of MIP pred. modes
static const int MAX_LFNST_COEF_NUM =                              16;

static const int LFNST_LAST_SIG_LUMA =                              1;
static const int LFNST_LAST_SIG_CHROMA =                            1;

static const int NUM_LFNST_NUM_PER_SET =                            3;

static const int CABAC_INIT_PRESENT_FLAG =                          1;

static const int MV_FRACTIONAL_BITS_INTERNAL                      = 4;
static const int MV_FRACTIONAL_BITS_SIGNAL                        = 2;
static const int MV_FRACTIONAL_BITS_DIFF = MV_FRACTIONAL_BITS_INTERNAL - MV_FRACTIONAL_BITS_SIGNAL;
static const int LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS = 1 << MV_FRACTIONAL_BITS_INTERNAL;
static const int CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS = 1 << (MV_FRACTIONAL_BITS_INTERNAL + 1);

static const int MAX_NUM_SUB_PICS =                               255;
static const int MAX_NUM_LONG_TERM_REF_PICS =                      33;
static const int NUM_LONG_TERM_REF_PIC_SPS =                        0;


static const int MAX_QP_OFFSET_LIST_SIZE =                          6; ///< Maximum size of QP offset list is 6 entries
static const int MAX_NUM_CQP_MAPPING_TABLES =                       3; ///< Maximum number of chroma QP mapping tables (Cb, Cr and joint Cb-Cr)
static const int MIN_QP_VALUE_FOR_16_BIT   =                      -48; ////< Minimum value for QP (-6*(bitdepth - 8) ) for bit depth 16 ; actual minimum QP value is bit depth dependent
static const int MAX_NUM_QP_VALUES =    MAX_QP + 1 - MIN_QP_VALUE_FOR_16_BIT; ////< Maximum number of QP values possible - bit depth dependent

static const int MAX_TIMECODE_SEI_SETS =                            3; ///< Maximum number of time sets

static const int MAX_CU_DEPTH =                                     7; ///< log2(CTUSize)
static const int MAX_CU_SIZE =                        1<<MAX_CU_DEPTH;
static const int MIN_CU_LOG2 =                                      2;
static const int MIN_PU_SIZE =                                      4;
static const int MIN_TU_SIZE =                                      4;
static const int MAX_LOG2_TU_SIZE_PLUS_ONE =                        7; ///< log2(MAX_TU_SIZE) + 1
static const int MAX_NUM_PARTS_IN_CTU =                         ( ( MAX_CU_SIZE * MAX_CU_SIZE ) >> ( MIN_CU_LOG2 << 1 ) );
static const int MAX_TU_SIZE_FOR_PROFILE =                         64;
static const int MAX_LOG2_DIFF_CU_TR_SIZE =                         2;
static const int MAX_CU_TILING_PARTITIONS = 1 << ( MAX_LOG2_DIFF_CU_TR_SIZE << 1 );

static const int JVET_C0024_ZERO_OUT_TH =                          32;

static const int MAX_NUM_PART_IDXS_IN_CTU_WIDTH = MAX_CU_SIZE/MIN_PU_SIZE; ///< maximum number of partition indices across the width of a CTU (or height of a CTU)
static const int SCALING_LIST_REM_NUM =                             6;

static const int IQUANT_SHIFT =                                     6;
static const int SCALE_BITS =                                      15; ///< Precision for fractional bit estimates

static const int SCALING_LIST_NUM = MAX_NUM_COMPONENT * (NUMBER_OF_PREDICTION_MODES - 1); ///< list number for quantization matrix

static const int SCALING_LIST_START_VALUE =                         8; ///< start value for dpcm mode
static const int MAX_MATRIX_COEF_NUM =                             64; ///< max coefficient number for quantization matrix
static const int MAX_MATRIX_SIZE_NUM =                              8; ///< max size number for quantization matrix
static const int SCALING_LIST_BITS =                                8; ///< bit depth of scaling list entries
static const int LOG2_SCALING_LIST_NEUTRAL_VALUE =                  4; ///< log2 of the value that, when used in a scaling list, has no effect on quantisation
static const int SCALING_LIST_DC =                                 16; ///< default DC value

static const int LAST_SIGNIFICANT_GROUPS =                         14;
static const int MAX_GR_ORDER_RESIDUAL =                           10;

static const int AFFINE_MIN_BLOCK_SIZE =                            4; ///< Minimum affine MC block size

static const int MMVD_REFINE_STEP =                                 8; ///< max number of distance step
static const int MMVD_MAX_REFINE_NUM =                              (MMVD_REFINE_STEP * 4); ///< max number of candidate from a base candidate
static const int MMVD_BASE_MV_NUM =                                 2; ///< max number of base candidate

static const int MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT_LUMA =      28;
static const int MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT_CHROMA =    28;

static const int BIO_EXTEND_SIZE              =                     1;
static const int BIO_ALIGN_SIZE               =                     8;
static const int BIO_TEMP_BUFFER_SIZE         =                     (MAX_BDOF_APPLICATION_REGION + 2 * BIO_ALIGN_SIZE) * (MAX_BDOF_APPLICATION_REGION + 2 * BIO_ALIGN_SIZE);

static const int PROF_BORDER_EXT_W            =                     1;
static const int PROF_BORDER_EXT_H            =                     1;
static const int BCW_NUM =                                          5; ///< the number of weight options
static const int BCW_DEFAULT =                                      0; ///< Default weighting index representing for w=0.5, in the internal domain
static const int BCW_SIZE_CONSTRAINT =                            256; ///< disabling Bcw if cu size is smaller than 256
static const int MAX_NUM_HMVP_CANDS =                              (MRG_MAX_NUM_CANDS-1); ///< maximum number of HMVP candidates to be stored and used in merge list
static const int MAX_NUM_HMVP_AVMPCANDS =                          4; ///< maximum number of HMVP candidates to be used in AMVP list

static const int ALF_VB_POS_ABOVE_CTUROW_LUMA =                     4;
static const int ALF_VB_POS_ABOVE_CTUROW_CHMA =                     2;

static const int DMVR_SUBCU_WIDTH       =                           16;
static const int DMVR_SUBCU_HEIGHT      =                           16;
static const int DMVR_SUBCU_WIDTH_LOG2  =                           4;
static const int DMVR_SUBCU_HEIGHT_LOG2 =                           4;
static const int MAX_NUM_SUBCU_DMVR     =                           ((MAX_CU_SIZE * MAX_CU_SIZE) >> (DMVR_SUBCU_WIDTH_LOG2 + DMVR_SUBCU_HEIGHT_LOG2));
static const int DMVR_NUM_ITERATION     =                           2;

static const int    MIN_DUALTREE_CHROMA_WIDTH  =                    4;
static const int    MIN_DUALTREE_CHROMA_SIZE   =                   16;

static const SplitSeries SPLIT_DMULT        =                       5;
static const SplitSeries SPLIT_MASK         =                      31;      ///< = (1 << SPLIT_BITS) - 1
static const int COM16_C806_TRANS_PREC =                            0;


static const int NTAPS_LUMA               =                         8; ///< Number of taps for luma
static const int NTAPS_CHROMA             =                         4; ///< Number of taps for chroma
#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
static const int MAX_LADF_INTERVALS       =                         5; /// max number of luma adaptive deblocking filter qp offset intervals
#endif

static const int NTAPS_BILINEAR           =                         2; ///< Number of taps for bilinear filter

static const int ATMVP_SUB_BLOCK_SIZE =                             3; ///< sub-block size for ATMVP
static const int GEO_MAX_NUM_UNI_CANDS =                            6;
static const int GEO_MAX_NUM_CANDS = GEO_MAX_NUM_UNI_CANDS * (GEO_MAX_NUM_UNI_CANDS - 1);
static const int GEO_MIN_CU_LOG2 =                                  3;
static const int GEO_MAX_CU_LOG2 =                                  6;
static const int GEO_MIN_CU_SIZE =               1 << GEO_MIN_CU_LOG2;
static const int GEO_MAX_CU_SIZE =               1 << GEO_MAX_CU_LOG2;
static const int GEO_NUM_CU_SIZE = ( GEO_MAX_CU_LOG2 - GEO_MIN_CU_LOG2 ) + 1;
static const int GEO_NUM_PARTITION_MODE =                          64;
static const int GEO_NUM_ANGLES =                                  32;
static const int GEO_NUM_DISTANCES =                                4;
static const int GEO_NUM_PRESTORED_MASK =                           6;
static const int GEO_WEIGHT_MASK_SIZE = 3 * (GEO_MAX_CU_SIZE >> 3) * 2 + GEO_MAX_CU_SIZE;
static const int GEO_MV_MASK_SIZE =         GEO_WEIGHT_MASK_SIZE >> 2;

static const int LDT_MODE_TYPE_INHERIT =                            0; ///< No need to signal mode_constraint_flag, and the modeType of the region is inherited from its parent node
static const int LDT_MODE_TYPE_INFER =                              1; ///< No need to signal mode_constraint_flag, and the modeType of the region is inferred as MODE_TYPE_INTRA
static const int LDT_MODE_TYPE_SIGNAL =                             2; ///< Need to signal mode_constraint_flag, and the modeType of the region is determined by the flag

static constexpr int MV_EXPONENT_BITCOUNT    =                      4;
static constexpr int MV_MANTISSA_BITCOUNT    =                      6;
static constexpr int MV_MANTISSA_UPPER_LIMIT =                      ((1 << (MV_MANTISSA_BITCOUNT - 1)) - 1);
static constexpr int MV_MANTISSA_LIMIT       =                      (1 << (MV_MANTISSA_BITCOUNT - 1));
static constexpr int MV_EXPONENT_MASK        =                      ((1 << MV_EXPONENT_BITCOUNT) - 1);

static constexpr int MV_BITS =                                      18;

static const int MVD_MAX =                            (1 << 17) - 1;
static const int MVD_MIN =                               -(1 << 17);

static const int PIC_CODE_CW_BINS =                              16;
static const int FP_PREC =                                       11;
static const int CSCALE_FP_PREC =                                11;
static const int SCALE_RATIO_BITS =                              14;
static const int MAX_SCALING_RATIO =                              2;  // max scaling ratio allowed in the software, it is used to allocated an internla buffer in the rescaling
static const std::pair<int, int> SCALE_1X = std::pair<int, int>( 1 << SCALE_RATIO_BITS, 1 << SCALE_RATIO_BITS );  // scale ratio 1x
static const int DELTA_QP_ACT[4] =                  { -5, 1, 3, 1 };

// ====================================================================================================================
// Macro functions
// ====================================================================================================================

template<typename T>
struct ClpRngTemplate
{
  T min() const { return 0; }
  T max() const { return ( ( 1 << bd ) - 1 );}
  int bd;
};

typedef ClpRngTemplate<Pel> ClpRng;
typedef ClpRng ClpRngs;

template <typename T> constexpr static inline T Clip3  ( const T minVal, const T maxVal, const T a) { return std::min<T> (std::max<T> (minVal, a) , maxVal); }  ///< general min/max clip
template <typename T> constexpr static inline T ClipBD ( const T x, const int bitDepth )            { return Clip3( T( 0 ), T( ( 1 << bitDepth ) - 1 ), x ); }
template <typename T> constexpr static inline T ClipPel( const T a, const ClpRng& clpRng )          { return ClipBD( a, clpRng.bd ); }  ///< clip reconstruction

static void default_msgFnc( void *, int level, const char* fmt, va_list args )
{
  vfprintf(stderr, fmt, args);
}

extern MsgLevel g_verbosity;
extern void    *g_context;
extern std::function<void( void*, int, const char*, va_list )> g_msgFnc;


static inline void msg( MsgLevel level, const char* fmt, ... )
{
  if ( vvdec::g_msgFnc && vvdec::g_verbosity >= level )
  {
    static std::mutex _msgMutex;
    std::unique_lock<std::mutex> _lock( _msgMutex );
    va_list args;
    va_start( args, fmt );
    vvdec::g_msgFnc( vvdec::g_context, level, fmt, args );
    va_end( args );
  }
}

#define MEMORY_ALIGN_DEF_SIZE       32  // for use with avx2 (256 bit)
#define CACHE_MEM_ALIGN_SIZE      1024

#define ALIGNED_MALLOC              1   ///< use 32-byte aligned malloc/free

#if ALIGNED_MALLOC

#  if( _WIN32 && ( _MSC_VER > 1300 ) ) || defined( __MINGW64_VERSION_MAJOR )
#    define xMalloc( type, len ) (type*) _aligned_malloc( sizeof( type ) * ( len ), MEMORY_ALIGN_DEF_SIZE )
#    define xFree( ptr )         _aligned_free( ptr )
#  elif defined( __MINGW32__ )
#    define xMalloc( type, len ) (type*) __mingw_aligned_malloc( sizeof( type ) * ( len ), MEMORY_ALIGN_DEF_SIZE )
#    define xFree( ptr )         __mingw_aligned_free( ptr )
#  else
#    define xMalloc( type, len ) detail::aligned_malloc<type>( len, MEMORY_ALIGN_DEF_SIZE )
#    define xFree( ptr )         free( ptr )
namespace detail
{
  template<typename T>
  static inline T* aligned_malloc( size_t len, size_t alignement )
  {
    T* p = NULL;
    if( posix_memalign( (void**) &p, alignement, sizeof( T ) * ( len ) ) )
    {
      THROW_FATAL( "posix_memalign failed" );
    }
    return p;
  }
}   // namespace detail
#  endif

#else   // !ALIGNED_MALLOC
#  define xMalloc( type, len ) (type*) malloc( sizeof( type ) * ( len ) )
#  define xFree( ptr )         free( ptr )
#endif   // !ALIGNED_MALLOC

template<class T>
struct AlignedDeleter
{
  void operator()( T* p ) const { xFree( p ); };
};

template<class T>
struct AlignedAllocator
{
  using value_type = T;

  using propagate_on_container_move_assignment = std::true_type;
  using is_always_equal                        = std::true_type;

  AlignedAllocator()  = default;
  ~AlignedAllocator() = default;
  CLASS_COPY_MOVE_DEFAULT( AlignedAllocator )

  template<class U>
  constexpr AlignedAllocator( const AlignedAllocator<U>& ) noexcept {}

  T* allocate( std::size_t n )
  {
    if( n > std::numeric_limits<std::size_t>::max() / sizeof( T ) )
    {
      throw std::bad_array_new_length();
    }

    if( T* p = xMalloc( T, n ) )
    {
      return p;
    }

    throw std::bad_alloc();
  }

  void deallocate( T* p, std::size_t ) noexcept { xFree( p ); }
};
template<typename T1, typename T2>
static bool operator==( const AlignedAllocator<T1>&, const AlignedAllocator<T2>& ) noexcept { return true; }
template<typename T1, typename T2>
static bool operator!=( const AlignedAllocator<T1>&, const AlignedAllocator<T2>& ) noexcept { return false; }

using AlignedByteVec = std::vector<uint8_t, AlignedAllocator<uint8_t>>;

#if defined _MSC_VER
#  define ALIGN_DATA( nBytes, v ) __declspec( align( nBytes ) ) v
#else
#  define ALIGN_DATA( nBytes, v ) v __attribute__( ( aligned( nBytes ) ) )
#endif

#if defined(__GNUC__) && !defined(__clang__)
#    define GCC_VERSION_AT_LEAST(x,y) (__GNUC__ > x || __GNUC__ == x && __GNUC_MINOR__ >= y)
#else
#    define GCC_VERSION_AT_LEAST(x,y) 0
#endif

#ifdef __clang__
#    define CLANG_VERSION_AT_LEAST(x,y) (__clang_major__ > x || __clang_major__ == x && __clang_minor__ >= y)
#else
#    define CLANG_VERSION_AT_LEAST(x,y) 0
#endif

#if defined( __GNUC__ )
#if __has_attribute( no_sanitize )
#    define NO_THREAD_SANITIZE __attribute__( ( no_sanitize( "thread" ) ) )
#else
#    define NO_THREAD_SANITIZE
#endif
#else
#    define NO_THREAD_SANITIZE
#endif

#ifdef __GNUC__
#    define ALWAYS_INLINE __attribute__((always_inline)) inline
#elif defined _MSC_VER
#    define ALWAYS_INLINE __forceinline
#else
#    define ALWAYS_INLINE
#endif

#if ENABLE_SIMD_OPT

//necessary to be able to compare with SIMD_EVERYWHERE_EXTENSION_LEVEL in the preprocessor
#define X86_SIMD_UNDEFINED -1
#define X86_SIMD_SCALAR     0
#define X86_SIMD_SSE41      1
#define X86_SIMD_SSE42      2
#define X86_SIMD_AVX        3
#define X86_SIMD_AVX2       4
#define X86_SIMD_AVX512     5

namespace x86_simd
{
#  ifdef TARGET_SIMD_X86
  typedef enum
  {
    UNDEFINED = X86_SIMD_UNDEFINED,
    SCALAR    = X86_SIMD_SCALAR,
    SSE41     = X86_SIMD_SSE41,
    SSE42     = X86_SIMD_SSE42,
    AVX       = X86_SIMD_AVX,
    AVX2      = X86_SIMD_AVX2,
    AVX512    = X86_SIMD_AVX512,
  } X86_VEXT;
#  endif   // TARGET_SIMD_X86
}   // namespace x86_simd

namespace arm_simd
{
#  ifdef TARGET_SIMD_ARM
  typedef enum
  {
    UNDEFINED = -1,
    SCALAR    = 0,
    NEON,
  } ARM_VEXT;
#  endif   // TARGET_SIMD_ARM
}   // namespace arm_simd

#endif   // ENABLE_SIMD_OPT

template <typename ValueType> static inline ValueType rightShift      (const ValueType value, const int shift) { return (shift >= 0) ? ( value                                  >> shift) : ( value                                   << -shift); }
template <typename ValueType> static inline ValueType rightShift_round(const ValueType value, const int shift) { return (shift >= 0) ? ((value + (ValueType(1) << (shift - 1))) >> shift) : ( value                                   << -shift); }

#if defined( _WIN32 )
static inline unsigned int bit_scan_reverse( int a )
{
  unsigned long idx = 0;
  _BitScanReverse( &idx, a );
  return idx;
}
#elif defined( __GNUC__ )
static inline unsigned int bit_scan_reverse( int a )
{
  return __builtin_clz( a ) ^ ( 8 * sizeof( a ) - 1 );
}
#endif

#if ENABLE_SIMD_LOG2
static inline int getLog2( int val )
{
  return bit_scan_reverse( val );
}
#else
extern int8_t g_aucLog2[MAX_CU_SIZE + 1];
static inline int getLog2( int val )
{
  CHECKD( g_aucLog2[2] != 1, "g_aucLog2[] has not been initialized yet." );
  if( val > 0 && val < (int) sizeof( g_aucLog2 ) )
  {
    return g_aucLog2[val];
  }
  return std::log2( val );
}
#endif

//CASE-BREAK for breakpoints
#if defined ( _MSC_VER ) && defined ( _DEBUG )
#define _CASE(_x) if(_x)
#define _BREAK while(0);
#define _AREA_AT(_a,_x,_y,_w,_h)  (_a.x==_x && _a.y==_y && _a.width==_w && _a.height==_h)
#define _AREA_CONTAINS(_a,_x,_y)  (_a.contains( Position{ _x, _y} ))
#define _UNIT_AREA_AT(_a,_x,_y,_w,_h) (_a.Y().x==_x && _a.Y().y==_y && _a.Y().width==_w && _a.Y().height==_h)
#else
#define _CASE(...)
#define _BREAK
#define _AREA_AT(...)
#define _AREA_CONTAINS(_a,_x,_y)
#define _UNIT_AREA_AT(_a,_x,_y,_w,_h)
#endif

#ifdef TRACE_ENABLE_ITT
}
# include <ittnotify.h>
namespace vvdec {

# define ITT_TASKSTART( d, t ) __itt_task_begin( ( d ), __itt_null, __itt_null, ( t ) )
# define ITT_TASKEND( d, t )   __itt_task_end  ( ( d ) )

# define ITT_SYNCPREP( p ) __itt_sync_prepare  ( & p )
# define ITT_SYNCACQ( p )  __itt_sync_acquired ( & p )
# define ITT_SYNCREL( p )  __itt_sync_releasing( & p )

# define ITT_COUNTSET( c, v ) __itt_counter_set_value( c, &v )
# define ITT_COUNTINC( c )    __itt_counter_inc( c )
# define ITT_COUNTDEC( c )    __itt_counter_dec( c )
# define ITT_COUNTADD( c, v ) __itt_counter_inc_delta( c, &v )
# define ITT_COUNTSUB( c, v ) __itt_counter_dec_delta( c, &v )
#else //!TRACE_ENABLE_ITT
# define ITT_TASKSTART( d, t )
# define ITT_TASKEND( d, t )

# define ITT_SYNCPREP( p )
# define ITT_SYNCACQ( p )
# define ITT_SYNCREL( p )

# define ITT_COUNTSET( c, v )
# define ITT_COUNTINC( c )
# define ITT_COUNTDEC( c )
# define ITT_COUNTADD( c, v )
# define ITT_COUNTSUB( c, v )
#endif //!TRACE_ENABLE_ITT

//! \}

}   // namespace vvdec
