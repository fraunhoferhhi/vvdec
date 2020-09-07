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

/** \file     TypeDef.h
    \brief    Define macros, basic types, new types and enumerations
*/

#ifndef __TYPEDEF__
#define __TYPEDEF__

#ifndef __COMMONDEF__
#error Include CommonDef.h not TypeDef.h
#endif

#include <vector>
#include <utility>
#include <sstream>
#include <cstddef>
#include <cstring>
#include <assert.h>
#include <cassert>


#if __SANITIZE_ADDRESS__
// macro to enable workarounds for address-sanitizer false-positives
# define ASAN_WORKAROUND                                  1
#endif

#define RECO_WHILE_PARSE                                  1
#define ALLOW_MIDER_LF_DURING_PICEXT                      1

#define JVET_O1170_CHECK_BV_AT_DECODER                    0 // For decoder to check if a BV is valid or not

#define DISABLE_CONFROMANCE_CHECK                         1

#define TBD                                               1
#define TBT                                               1
#define TBC                                               0

#define DISABLE_CHECK_NO_OUTPUT_PRIOR_PICS_FLAG           1

#define JVET_O1143_SUBPIC_BOUNDARY                        TBT //to be tested // treat subpicture boundary as picture boundary
#if JVET_O1143_SUBPIC_BOUNDARY
#define JVET_O1143_LPF_ACROSS_SUBPIC_BOUNDARY             1
#define JVET_O1143_MV_ACROSS_SUBPIC_BOUNDARY              1
#endif

#define JVET_P0365_SCALING_MATRIX_LFNST                   TBD //enc crash // JVET-P0365: Signal flag to indicate whether scaling matrices are used for LFNST-coded blocks
#define JVET_P0117_PTL_SCALABILITY                        TBT //to be tested // JVET-P0117: sps_ptl_dpb_hrd_params_present_flag related syntax change, others in JVET-Q0786
#define JVET_P0101_POC_MULTILAYER                         TBT //to be tested // POC derivation for pictures in dependent layers
#define JVET_P0288_PIC_OUTPUT                             TBT //to be tested // JVET-P0288: Set the value of PictureOutputFlag

#define JVET_Q0044_SLICE_IDX_WITH_SUBPICS                 TBT //to be tested // JVET-Q0044: slice index with subpictures
#define JVET_Q0786_PTL_only                               TBT //to be tested // JVET-Q0786: modifications to VPS syntax - PTL part only (signal PTL for single layer OLSs)
#define JVET_Q0438_MONOCHROME_BUGFIXES                    TBT //to be tested with 4:0:0 // JVET-Q0438: Monochrome bug fixes
#define JVET_Q0818_PT_SEI                                 TBT //to be tested // JVET-Q0818: add display_elemental_periods_minus1 to picture timing SEI message
#define JVET_Q0814_DPB                                    TBT //to be tested // JVET-Q0814: DPB capacity is based on picture units regardless of the resoltuion
#define JVET_Q0114_ASPECT5_GCI_FLAG                       TBT //to be tested // JVET-Q0114 Aspect 5: Add a general constraint on no reference picture resampling

#define JVET_R0058                                        TBT //to be tested // JVET-R0058: the combination of RPR, subpictures, and scalability
#define JVET_R0162_WRAPAROUND_OFFSET_SIGNALING            TBT //to be tested // JVET-R0162 proposal 1 : signal "picture width minus wraparound offset" instead of "wraparound offset"
#define JVET_R0188                                        TBT // JVET-R0188: Signalling slice_width_in_tiles_minus1[i] and slice_height_in_tiles_minus1[i]
#define JVET_R0203_IRAP_LEADING_CONSTRAINT                TBT //to be tested // JVET-R0203: Constraint that IRAP NAL unit type cannot be mixed with RASL_NUT / RADL_NUT
#define JVET_Q0764_WRAP_AROUND_WITH_RPR                   TBT // JVET-Q0764: Combination of wrap around offset and RPR
#define JVET_R0483_SH_TSRC_DISABLED_FLAG_CLEANUP          TBT //to be tested // JVET-R0483 Comb 4: R0049 + R0271, only R0049 method 3 aspect (Skip signaling sh_ts_residual_coding_disabled_flag when sps_transform_skip_enabled_flag = 0, also proposed in R0068, R0097, R0142, R0153) as R0271 has its own macro
#define JVET_R0380_SCALING_MATRIX_DISABLE_YCC_OR_RGB      TBT //to be tested // JVET-R0380 solution3-3: Disable scaling matrix for blocks coded in alternative colour space.
#define R0091_CONSTRAINT_SLICE_ORDER                      TBT //to be tested // JVET-R0091: constraint slice signalling order to be the same as slice coding order
#define JVET_R0165_OPTIONAL_ENTRY_POINT                   TBT //to be tested // JVET-R0165: Optional entry point offset
#define JVET_R0166_SCALING_LISTS_CHROMA_444               TBT //to be tested // JVET-R0166: Scaling list for Chroma 444
#define JVET_R0191_ASPECT3                                TBT // JVET-R0191#3: Modify the upper range of vps_num_dpb_params and num_ols_hrd_params_minus1 to be total number of OLSs minus the number of single-layer OLSs
                                                              //               Constrain that each PTL, DPB, and HRD params in VPS are referred to at least once
#define JVET_R0276_REORDERED_SUBPICS                      TBT // JVET-R0276: reference picture constraint for reordered sub-pictures
#define JVET_R0350_MIP_CHROMA_444_SINGLETREE              TBT //to be tested // JVET-R0350: MIP for chroma in case of 4:4:4 format and single tree
#define JVET_R0294_SUBPIC_HASH                            TBT //to be tested // JVET-R0294: Allow decoded picture hash SEI messages to be nested in subpicture context

#define JVET_S0058_GCI                                    TBT //to be tested // no_mtt_constraint_flag and no_weightedpred_constraint_flag
#define JVET_R0341_GCI                                    TBT //to be tested // JVET-R0341: on constraint flag for local chroma QP control
#define JVET_R0270                                        TBC // JVET-S0270: Treating picture with mixed RASL and RADL slices as RASL picture
#define JVET_S0258_SUBPIC_CONSTRAINTS                     TBT //to be tested // JVET-S0258: sub-picture constraints
#define JVET_S0234_ACT_CRS_FIX                            TBT //to be tested // JVET-S0234: perform chroma residual scaling in RGB domain when ACT is on
#define JVET_S0123_IDR_UNAVAILABLE_REFERENCE              TBT //to be tested // JVET-S0123: Invoke the generation of unavailable reference picture for an IDR picture that has RPLs.
                                                              //             Change the process for deriving empty RPLs when sps_idr_rpl_present_flag is equal to 0 and nal_unit_type is equal to IDR_W_RADL or IDR_N_LP to involve pps_rpl_info_in_ph_flag.
#define JVET_S0124_UNAVAILABLE_REFERENCE                  TBT //to be tested // JVET-S0124: Add TemporalId, ph_non_ref_pic_flag, and ph_pic_parameter_set_id for generating unavailable reference pictures
#define JVET_S0063_VPS_SIGNALLING                         TBT //to be tested // Modifications to VPS signalling - conditionally signal vps_num_ptls_minus1
#define JVET_S0155_EOS_NALU_CHECK                         TBC // JVET-S0155: Constraints on EOS NAL units
#define JVET_S0071_SAME_SIZE_SUBPIC_LAYOUT                TBT //to be tested // JVET-S0071 : shortcut when all subpictures have the same size
#define JVET_S0098_SLI_FRACTION                           TBT //to be tested // JVET-S0098 Item 3: Add non_subpic_layers_fraction syntax element
#define JVET_S0048_SCALING_OFFSET                         TBT //to be tested // JVET-S0048 Aspect2: change the constraint on the value ranges of scaling window offsets to be more flexible
#define JVET_S0248_HRD_CLEANUP                            TBT //to be tested // JVET-S0248 Aspect7: When bp_alt_cpb_params_present_flag is equal to 1, the value of bp_du_hrd_params_present_flag shall be equal to 0.
#define JVET_S0100_ASPECT3                                TBT //to be tested // JVET-S0100 Aspect 3: constraints on vps_dpb_max_tid and vps_hrd_max_tid depending on vps_ptl_max_tid
#define JVET_S0064_SEI_BUFFERING_PERIOD_CLEANUP           TBT //to be tested // JVET-S0064: Conditionally signal bp_sublayer_dpb_output_offsets_present_flag
#define JVET_S0185_PROPOSAL2_SEI_CLEANUP                  TBT //to be tested // JVET-S0185_PROPOSAL2: Move signalling of syntax element bp_alt_cpb_params_present_flag
#define JVET_S0181_PROPOSAL1                              TBT //to be tested // JVET-0181_Proposal1: Conditionally signal bp_sublayer_initial_cpb_removal_delay_present_flag
#define JVET_S0177_SCALABLE_NESTING_SEI                   TBT //to be tested // JVET-S0177: Constraints on the scalable nesting SEI message
#define JVET_S0178_GENERAL_SEI_CHECK                      TBT //to be tested // JVET-S0178: General SEI semantics and constraints
#define JVET_S0176_SLI_SEI                                TBT //to be tested // JVET-S0176: On the subpicture level information SEI message
#define JVET_S0181_PROPOSAL2_BUFFERING_PERIOD_CLEANUP     TBT //to be tested // JVET-S0181 Proposal2: Move signalling of bp_max_sublayers_minus1 and conditionally signal bp_cpb_removal_delay_deltas_present_flag, bp_num_cpb_removal_delay_deltas_minus1, and bp_cpb_removal_delay
#define JVET_S0050_GCI                                    TBT //to be tested // JVET-S0050: Signal new GCI flags no_virtual_boundaries_constraint_flag and no_explicit_scaling_list_constraint_flag
                                                                   //             Constrain the value of one_subpic_per_pic_constraint_flag, one_slice_per_pic_constraint_flag and no_aps_constraint_flag
                                                                   //             Remove all constraints that require GCI fields to be equal to a value that imposes a constraint
#define JVET_S0138_GCI_PTL                                TBT //to be tested // JVET-S_Notes_d9: move frame_only_constraint_flag and single_layer_constraint_flag into PTL for easy access
#define JVET_S0113_S0195_GCI                              TBT //to be tested // JVET-S0113: no_rectangular_slice_constraint_flag to constrain pps_rect_slice_flag
                                                            //             one_slice_per_subpicture_constraint_flag to constrain pps_single_slice_per_subpic_flag
                                                            // JVET-S0195: replace one_subpic_per_pic_constraint_flag with no_subpic_info_constraint_flag and its semantics
                                                            //             add no_idr_rpl_constraint_flag
#define JVET_S0182_RPL_SIGNALLING                         TBT //to be tested // JVET-S0182: modifications to rpl information signalling
#define JVET_S0185_PROPOSAl1_PICTURE_TIMING_CLEANUP       TBT //to be tested // JVET-S0185: Proposal 1, put syntax element pt_cpb_removal_delay_minus1[] first, followed by similar information for sub-layers, followed by pt_dpb_output_delay
#define JVET_S0183_VPS_INFORMATION_SIGNALLING             TBT //to be tested // JVET-S0183: Proposal 1, signal vps_num_output_layer_sets_minus1 as vps_num_output_layer_sets_minus2
#define JVET_S0105_GCI_REORDER_IN_CATEGORY                TBT //to be tested // JVET-S0105: reorder and categorize GCI flags (assumes the following macros set to 1: JVET_S0050_GCI, JVET_S0113_S0195_GCI, JVET_S0066_GCI, JVET_S0138_GCI_PTL, JVET_S0058_GCI, JVET_R0341_GCI, JVET_Q0114_ASPECT5_GCI_FLAG)

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
#define ENABLE_SIMD_OPT_ALF                             ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for ALF
#define ENABLE_SIMD_OPT_INTRAPRED                       ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for Intra Prediction
#define ENABLE_SIMD_OPT_QUANT                           ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for Quant/Dequant
#if ENABLE_SIMD_OPT_BUFFER
#define ENABLE_SIMD_OPT_GBI                               1                                                 ///< SIMD optimization for GBi
#endif
#define ENABLE_SIMD_OPT_BIO                             ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for BIO
#define ENABLE_SIMD_OPT_PICTURE                         ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for Picture Padding
#define ENABLE_SIMD_OPT_SAO                             ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for BIO
#define ENABLE_SIMD_TCOEFF_OPS                          ( 1 && ENABLE_SIMD_OPT )
#define ENABLE_SIMD_LOG2                                ( 1 && ENABLE_SIMD_OPT )
#define ENABLE_SIMD_DBLF                                ( 1 && ENABLE_SIMD_OPT )

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

typedef       uint64_t        Distortion;        ///< distortion measurement

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
enum TransType : int8_t
{
  DCT2           = 0,
  DCT8           = 1,
  DST7           = 2,
  NUM_TRANS_TYPE
};

enum MTSIdx
{
  MTS_DCT2_DCT2 = 0,
  MTS_SKIP = 1,
  MTS_DST7_DST7 = 2,
  MTS_DCT8_DST7 = 3,
  MTS_DST7_DCT8 = 4,
  MTS_DCT8_DCT8 = 5
};

enum ISPType : int8_t
{
  NOT_INTRA_SUBPARTITIONS = 0,
  HOR_INTRA_SUBPARTITIONS = 1,
  VER_INTRA_SUBPARTITIONS = 2,
  NUM_INTRA_SUBPARTITIONS_MODES
};

enum SbtIdx : int8_t
{
  SBT_OFF_DCT  = 0,
  SBT_VER_HALF = 1,
  SBT_HOR_HALF = 2,
  SBT_VER_QUAD = 3,
  SBT_HOR_QUAD = 4,
  NUMBER_SBT_IDX
};

enum SbtPos : int8_t
{
  SBT_POS0 = 0,
  SBT_POS1 = 1,
  NUMBER_SBT_POS
};

enum SbtMode : int8_t
{
  SBT_VER_H0 = 0,
  SBT_VER_H1 = 1,
  SBT_HOR_H0 = 2,
  SBT_HOR_H1 = 3,
  SBT_VER_Q0 = 4,
  SBT_VER_Q1 = 5,
  SBT_HOR_Q0 = 6,
  SBT_HOR_Q1 = 7,
  NUMBER_SBT_MODE
};

enum RDPCMMode : int8_t
{
  RDPCM_OFF             = 0,
  RDPCM_HOR             = 1,
  RDPCM_VER             = 2,
  NUMBER_OF_RDPCM_MODES
};

enum RDPCMSignallingMode
{
  RDPCM_SIGNAL_IMPLICIT            = 0,
  RDPCM_SIGNAL_EXPLICIT            = 1,
  NUMBER_OF_RDPCM_SIGNALLING_MODES
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
enum ChromaFormat : int8_t
{
  CHROMA_400        = 0,
  CHROMA_420        = 1,
  CHROMA_422        = 2,
  CHROMA_444        = 3,
  NUM_CHROMA_FORMAT
};

enum ChannelType : int8_t
{
  CHANNEL_TYPE_LUMA    = 0,
  CHANNEL_TYPE_CHROMA  = 1,
  MAX_NUM_CHANNEL_TYPE
};

enum TreeType : int8_t
{
  TREE_D = 0, //default tree status (for single-tree slice, TREE_D means joint tree; for dual-tree I slice, TREE_D means TREE_L for luma and TREE_C for chroma)
  TREE_L = 1, //separate tree only contains luma (may split)
  TREE_C = 2, //separate tree only contains chroma (not split), to avoid small chroma block
};

enum ModeType : int8_t
{
  MODE_TYPE_ALL = 0, //all modes can try
  MODE_TYPE_INTER = 1, //can try inter
  MODE_TYPE_INTRA = 2, //can try intra, ibc, palette
};

#define CH_L CHANNEL_TYPE_LUMA
#define CH_C CHANNEL_TYPE_CHROMA

enum ComponentID : int8_t
{
  COMPONENT_Y         = 0,
  COMPONENT_Cb        = 1,
  COMPONENT_Cr        = 2,
  MAX_NUM_COMPONENT,
  JOINT_CbCr          = MAX_NUM_COMPONENT,
  MAX_NUM_TBLOCKS     = MAX_NUM_COMPONENT
};

#define MAP_CHROMA(c) (ComponentID(c))

enum InputColourSpaceConversion // defined in terms of conversion prior to input of encoder.
{
  IPCOLOURSPACE_UNCHANGED               = 0,
  IPCOLOURSPACE_YCbCrtoYCrCb            = 1, // Mainly used for debug!
  IPCOLOURSPACE_YCbCrtoYYY              = 2, // Mainly used for debug!
  IPCOLOURSPACE_RGBtoGBR                = 3,
  NUMBER_INPUT_COLOUR_SPACE_CONVERSIONS = 4
};

enum MATRIX_COEFFICIENTS // Table E.5 (Matrix coefficients)
{
  MATRIX_COEFFICIENTS_RGB                           = 0,
  MATRIX_COEFFICIENTS_BT709                         = 1,
  MATRIX_COEFFICIENTS_UNSPECIFIED                   = 2,
  MATRIX_COEFFICIENTS_RESERVED_BY_ITUISOIEC         = 3,
  MATRIX_COEFFICIENTS_USFCCT47                      = 4,
  MATRIX_COEFFICIENTS_BT601_625                     = 5,
  MATRIX_COEFFICIENTS_BT601_525                     = 6,
  MATRIX_COEFFICIENTS_SMPTE240                      = 7,
  MATRIX_COEFFICIENTS_YCGCO                         = 8,
  MATRIX_COEFFICIENTS_BT2020_NON_CONSTANT_LUMINANCE = 9,
  MATRIX_COEFFICIENTS_BT2020_CONSTANT_LUMINANCE     = 10,
};

enum DeblockEdgeDir : int8_t
{
  EDGE_VER     = 0,
  EDGE_HOR     = 1,
  NUM_EDGE_DIR
};

/// supported prediction type
enum PredMode : int8_t
{
  MODE_INTER                 = 0,     ///< inter-prediction mode
  MODE_INTRA                 = 1,     ///< intra-prediction mode
  MODE_IBC                   = 2,     ///< ibc-prediction mode
  NUMBER_OF_PREDICTION_MODES
};

/// reference list index
enum RefPicList : int8_t
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
  DF_SAD             = 8,             ///< general size SAD
  DF_SAD2            = DF_SAD+1,      ///<   2xM SAD
  DF_SAD4            = DF_SAD+2,      ///<   4xM SAD
  DF_SAD8            = DF_SAD+3,      ///<   8xM SAD
  DF_SAD16           = DF_SAD+4,      ///<  16xM SAD
  DF_SAD32           = DF_SAD+5,      ///<  32xM SAD
  DF_SAD64           = DF_SAD+6,      ///<  64xM SAD
  DF_SAD16N          = DF_SAD+7,      ///< 16NxM SAD

  DF_TOTAL_FUNCTIONS = DF_SAD16N + 1
};

/// motion vector predictor direction used in AMVP
enum MvpDir : int8_t
{
  MD_LEFT = 0,          ///< MVP of left block
  MD_ABOVE,             ///< MVP of above block
  MD_ABOVE_RIGHT,       ///< MVP of above right block
  MD_BELOW_LEFT,        ///< MVP of below left block
  MD_ABOVE_LEFT         ///< MVP of above left block
};

/// coefficient scanning type used in ACS
enum CoeffScanType
{
  SCAN_DIAG = 0,        ///< up-right diagonal scan
  SCAN_NUMBER_OF_TYPES
};

enum CoeffScanGroupType
{
  SCAN_UNGROUPED   = 0,
  SCAN_GROUPED_4x4 = 1,
  SCAN_NUMBER_OF_GROUP_TYPES = 2
};

enum ScalingListMode
{
  SCALING_LIST_OFF,
  SCALING_LIST_DEFAULT,
  SCALING_LIST_FILE_READ
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
  SCALING_LIST_128x128,
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

// For use with decoded picture hash SEI messages, generated by encoder.
enum HashType
{
  HASHTYPE_MD5             = 0,
  HASHTYPE_CRC             = 1,
  HASHTYPE_CHECKSUM        = 2,
  HASHTYPE_NONE            = 3,
  NUMBER_OF_HASHTYPES      = 4
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

namespace Level
{
  enum Tier
  {
    MAIN = 0,
    HIGH = 1,
    NUMBER_OF_TIERS=2
  };

  enum Name
  {
    // code = (level * 30)
    NONE     = 0,
    LEVEL1   = 16,
    LEVEL2   = 32,
    LEVEL2_1 = 35,
    LEVEL3   = 48,
    LEVEL3_1 = 51,
    LEVEL4   = 64,
    LEVEL4_1 = 67,
    LEVEL5   = 80,
    LEVEL5_1 = 83,
    LEVEL5_2 = 86,
    LEVEL6   = 96,
    LEVEL6_1 = 99,
    LEVEL6_2 = 102,
    LEVEL15_5 = 255,
  };
}

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
  NAL_UNIT_RESERVED_IRAP_VCL_12,
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

enum MergeType : int8_t
{
  MRG_TYPE_DEFAULT_N        = 0, // 0
  MRG_TYPE_SUBPU_ATMVP,
  MRG_TYPE_IBC,
  NUM_MRG_TYPE                   // 5
};

enum TriangleSplit : int8_t
{
  TRIANGLE_DIR_135 = 0,
  TRIANGLE_DIR_45,
  TRIANGLE_DIR_NUM
};

enum AffineModel : int8_t
{
  AFFINEMODEL_4PARAM,
  AFFINEMODEL_6PARAM,
  AFFINE_MODEL_NUM
};

enum ImvMode : int8_t
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

  SAOOffset() { reset(); }
  ~SAOOffset() {}

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
  SAOBlkParam() { reset(); }
  ~SAOBlkParam() {}

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



struct PictureHash
{
  std::vector<uint8_t> hash;

  bool operator==(const PictureHash &other) const
  {
    if (other.hash.size() != hash.size())
    {
      return false;
    }
    for(uint32_t i=0; i<uint32_t(hash.size()); i++)
    {
      if (other.hash[i] != hash[i])
      {
        return false;
      }
    }
    return true;
  }

  bool operator!=(const PictureHash &other) const
  {
    return !(*this == other);
  }
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
  bool     colourVolumeSEIEnabled;
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
  Exception( const std::string& _s ) : m_str( _s ) { }
  Exception( const Exception& _e ) : std::exception( _e ), m_str( _e.m_str ) { }
  virtual ~Exception() noexcept { };
  virtual const char* what() const noexcept { return m_str.c_str(); }
  Exception& operator=( const Exception& _e ) { std::exception::operator=( _e ); m_str = _e.m_str; return *this; }
  template<typename T> Exception& operator<<( T t ) { std::ostringstream oss; oss << t; m_str += oss.str(); return *this; }
private:
  std::string m_str;
};

// if a check fails with THROW or CHECK, please check if ported correctly from assert in revision 1196)
#define THROW(x)            throw( Exception( "\nERROR: In function \"" ) << __FUNCTION__ << "\" in " << __FILE__ << ":" << __LINE__ << ": " << x )
//#define THROW(x)            { std::cerr << "\nERROR: In function \"" << __FUNCTION__ << "\" in " << __FILE__ << ":" << __LINE__ << ": " << x << std::endl; abort(); }
#define CHECK(c,x)          if(c){ THROW(x); }
#define CHECK_WARN(c,x)     if(c){ std::cerr << "\nWARNING: In function \"" << __FUNCTION__ << "\" in " << __FILE__ << ":" << __LINE__ << ": " << x << std::endl; }
#define EXIT(x)             throw( Exception( "\n" ) << x << "\n" )
#define CHECK_NULLPTR(_ptr) CHECK( !( _ptr ), "Accessing an empty pointer!" )

#if !NDEBUG  // for non MSVC compiler, define _DEBUG if in debug mode to have same behavior between MSVC and others in debug
#ifndef _DEBUG
#define _DEBUG 1
#endif
#endif

#if defined( _DEBUG )
#define CHECKD(c,x)         if(c){ THROW(x); }
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
  size_t _size;

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

  static_vector() : _size( 0 )                                 { }
  static_vector( size_t N_ ) : _size( N_ )                     { }
  static_vector( size_t N_, const T& _val ) : _size( 0 )       { resize( N_, _val ); }
  template<typename It>
  static_vector( It _it1, It _it2 ) : _size( 0 )               { while( _it1 < _it2 ) _arr[ _size++ ] = *_it1++; }
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

  void            erase( const_iterator _pos )  { iterator it   = const_cast<iterator>( _pos ) - 1;
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
  short            lumaCoeffFinal     [MAX_NUM_ALF_TRANSPOSE_ID * MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
  short            lumaClippFinal     [MAX_NUM_ALF_TRANSPOSE_ID * MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
  short            chrmClippFinal     [MAX_NUM_ALF_ALTERNATIVES_CHROMA * MAX_NUM_ALF_CHROMA_COEFF];
  int              tLayer;
  bool             newFilterFlag      [MAX_NUM_CHANNEL_TYPE];
  
  AlfSliceParam()
  {
    memset( this, 0, sizeof( *this ) );
  }

  void reset()
  {
    std::memset( this, 0, sizeof( AlfSliceParam ) );
    numLumaFilters = 1;
    numAlternativesChroma = 1;
  }

  const AlfSliceParam& operator=( const AlfSliceParam& src )
  {
    std::memcpy( this, &src, sizeof( AlfSliceParam ) );
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
template<typename TList> static void move_to_end( typename TList::const_iterator it, TList& list )
{
#ifdef _DEBUG
  const auto* oldAddr = &list.front();
#endif

  list.splice( list.cend(), list, it );

  CHECKD( &list.back() != oldAddr, "moving failed" );
}


// ---------------------------------------------------------------------------
// c++11/14 workarounds
// ---------------------------------------------------------------------------

// move_wrapper:
//   adapter class, that moves an object instead of copying
//   (not needed with c++14, when we have generalized lambda capture)
//
//   Be careful, when using this class to not acciedentially create 'copies'
//   invalidating the state of the copied-from object.
#ifndef __cpp_init_captures
template<class T>
struct move_wrapper: public T
{
  // constuct from rvalue-ref
  explicit move_wrapper(T && rref): T(std::forward<T>(rref)) {}

  // the copy-constructor actually does a move of the contained object
  move_wrapper(move_wrapper & other): T( std::move(other) )  {}

  // normal move constructor and assignment
  move_wrapper(move_wrapper &&)             = default;
  move_wrapper & operator=(move_wrapper &&) = default;

  // disable default and const-copy construction
  move_wrapper() = delete;
  move_wrapper(const move_wrapper &) = delete;
  move_wrapper & operator=(const move_wrapper &) = delete;

};
#endif // !__cpp_init_captures

//! \}

#endif
