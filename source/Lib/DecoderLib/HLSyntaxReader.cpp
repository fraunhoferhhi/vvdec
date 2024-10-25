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

/** \file     HLSyntaxReader.cpp
 *  \brief    Reader for high level syntax
 */

//! \ingroup DecoderLib
//! \{

#include "HLSyntaxReader.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/Rom.h"
#include "CommonLib/dtrace_next.h"
#include "CommonLib/AdaptiveLoopFilter.h"

#include "ParameterSetManager.h"

#define CHECK_CONSTRAINT( cond, msg ) CHECK( cond, msg )

namespace vvdec
{

void AUDReader::parseAccessUnitDelimiter( InputBitstream* bs, uint32_t &picType )
{
  setBitstream( bs );

#if ENABLE_TRACING
  xTraceAccessUnitDelimiter();
#endif

  X_READ_FLAG( aud_irap_or_gdr_au_flag );
  (void)aud_irap_or_gdr_au_flag;

  X_READ_CODE_NO_RANGE( aud_pic_type, 3 );
  picType = aud_pic_type;

  xReadRbspTrailingBits();
}

void FDReader::parseFillerData( InputBitstream* bs, uint32_t &fdSize )
{
  setBitstream( bs );
#if ENABLE_TRACING
  DTRACE( g_trace_ctx, D_HEADER, "=========== Filler Data ===========\n" );
#endif
  fdSize = 0;
  while( m_pcBitstream->getNumBitsLeft() > 8 )
  {
    X_READ_CODE_NO_RANGE( fd_ff_byte, 8 );
    CHECK_WARN( fd_ff_byte != 0xff, "Invalid fillter data not '0xff'" );
    fdSize++;
  }
  xReadRbspTrailingBits();
}


// ====================================================================================================================
// Public member functions
// ====================================================================================================================

void HLSyntaxReader::copyRefPicList( const SPS* sps, const ReferencePictureList* source_rpl, ReferencePictureList* dest_rp )
{
  memcpy( dest_rp, source_rpl, sizeof( ReferencePictureList ) );

  if( !sps->getLongTermRefsPresent() )
  {
    dest_rp->setNumberOfLongtermPictures( 0 );
  }
}

void HLSyntaxReader::parseRefPicList( ReferencePictureList* rpl, int rplIdx, const SPS* sps )
{
  X_READ_UVLC_idx( num_ref_entries, "[ listIdx ][ rplsIdx ]", 0, MAX_NUM_REF_PICS );
  // The value of num_ref_entries[ listIdx ][ rplsIdx ] shall be in the range of 0 to MaxDpbSize + 13, inclusive,
  //  where MaxDpbSize is as specified in clause A.4.2.

  if( sps->getLongTermRefsPresent() && num_ref_entries > 0 && rplIdx != -1 )   // rplsIdx == -1 means it's called from parsePicOrSliceHeader
  {
    X_READ_FLAG_idx( ltrp_in_header_flag, "[ listIdx ][ rplsIdx ]" );
    rpl->setLtrpInSliceHeaderFlag( ltrp_in_header_flag );
  }
  else if( sps->getLongTermRefsPresent() )
  {
    // When sps_long_term_ref_pics_flag is equal to 1 and rplsIdx is equal to sps_num_ref_pic_lists[ listIdx ],
    //  the value of ltrp_in_header_flag[ listIdx ][ rplsIdx ] is inferred to be equal to 1.
    rpl->setLtrpInSliceHeaderFlag( 1 );
  }

  uint32_t numStrp = 0;
  uint32_t numLtrp = 0;
  uint32_t numIlrp = 0;

  rpl->setInterLayerPresentFlag( sps->getInterLayerPresentFlag() );

  int prevDelta = 0;
  for( unsigned ii = 0; ii < num_ref_entries; ii++ )
  {
    if( rpl->getInterLayerPresentFlag() )
    {
      X_READ_FLAG_idx( inter_layer_ref_pic_flag, "[ listIdx ][ rplsIdx ][ i ]" );
      if( inter_layer_ref_pic_flag )
      {
        X_READ_UVLC_idx( ilrp_idx, "[ listIdx ][ rplsIdx ][ i ]", 0, MAX_VPS_LAYERS );
        // The value of ilrp_idx[ listIdx ][ rplsIdx ][ i ] shall be in the range of 0 to NumDirectRefLayers[ GeneralLayerIdx[ nuh_layer_id ] ] − 1, inclusive.
        rpl->setRefPicIdentifier( ii, 0, true, true, ilrp_idx );
        numIlrp++;

        continue;
      }
    }

    // if( !inter_layer_ref_pic_flag )  // implicit due to previous `continue`
    {
      bool isLongTerm = false;
      if( sps->getLongTermRefsPresent() )
      {
        X_READ_FLAG_idx( st_ref_pic_flag, "[ listIdx ][ rplsIdx ][ i ]" );
        isLongTerm = !st_ref_pic_flag;
      }

      if( !isLongTerm )
      {
        X_READ_UVLC_idx( abs_delta_poc_st, "[ listIdx ][ rplsIdx ][ i ]", 0, ( 1 << 15 ) - 1 );
        int deltaPocSt = abs_delta_poc_st;
        if( ( !sps->getUseWP() && !sps->getUseWPBiPred() ) || ii == 0 )
        {
          deltaPocSt++;
        }

        if( deltaPocSt > 0 )
        {
          X_READ_FLAG_idx( strp_entry_sign_flag, "[ listIdx ][ rplsIdx ][ i ]" );
          if( strp_entry_sign_flag )
          {
            deltaPocSt = -deltaPocSt;
          }
        }

        deltaPocSt += prevDelta;
        prevDelta = deltaPocSt;
        rpl->setRefPicIdentifier( ii, deltaPocSt, false, false, 0 );
        numStrp++;
      }
      else
      {
        if( !rpl->getLtrpInSliceHeaderFlag() )
        {
          X_READ_CODE_NO_RANGE_idx( rpls_poc_lsb_lt, "[ listIdx ][ rplsIdx ][ j++ ]", sps->getBitsForPOC() );
          rpl->setRefPicIdentifier( ii, rpls_poc_lsb_lt, true, false, 0 );
        }
        else
        {
          rpl->setRefPicIdentifier( ii, 0, true, false, 0 );
        }
        numLtrp++;
      }
    }
  }
  rpl->setNumberOfShorttermPictures( numStrp );
  rpl->setNumberOfLongtermPictures( numLtrp );
  rpl->setNumberOfInterLayerPictures( numIlrp );
}

void HLSyntaxReader::parsePPS( PPS* pcPPS, const ParameterSetManager* parameterSetManager )
{
#if ENABLE_TRACING
  xTracePPSHeader();
#endif

  X_READ_CODE_NO_RANGE( pps_pic_parameter_set_id, 6 );
  pcPPS->setPPSId( pps_pic_parameter_set_id );

  X_READ_CODE( pps_seq_parameter_set_id, 4, 0, 15 );
  const SPS* sps = parameterSetManager->getSPS( pps_seq_parameter_set_id );
  CHECK( !sps, "SPS with id " << pps_seq_parameter_set_id << " missing." );
  pcPPS->setSPSId( pps_seq_parameter_set_id );

  const int SubWidthC  = 1 << getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, sps->getChromaFormatIdc() );
  const int SubHeightC = 1 << getChannelTypeScaleY( CHANNEL_TYPE_CHROMA, sps->getChromaFormatIdc() );

  X_READ_FLAG( pps_mixed_nalu_types_in_pic_flag );
  pcPPS->setMixedNaluTypesInPicFlag( pps_mixed_nalu_types_in_pic_flag == 1 );

  const int CtbSizeY   = sps->getCTUSize();
  const int MinCbSizeY = 1 << sps->getLog2MinCodingBlockSize();

  X_READ_UVLC( pps_pic_width_in_luma_samples, 1, sps->getMaxPicWidthInLumaSamples() );
  CHECK( pps_pic_width_in_luma_samples & std::max( 8 - 1, MinCbSizeY - 1 ), "pps_pic_width_in_luma_samples not a multiple of 8 or MinCbSizeY" );
  CHECK( !sps->getResChangeInClvsEnabledFlag() && pps_pic_width_in_luma_samples != sps->getMaxPicWidthInLumaSamples(),
         "When sps_res_change_in_clvs_allowed_flag equal to 0, the value of pps_pic_width_in_luma_samples shall be equal"
         " to sps_pic_width_max_in_luma_samples." )
  CHECK( sps->getUseWrapAround() && CtbSizeY / MinCbSizeY + 1 > pps_pic_width_in_luma_samples / MinCbSizeY - 1,
         "When sps_ref_wraparound_enabled_flag is equal to 1, the value of ( CtbSizeY / MinCbSizeY + 1 ) shall be less than or"
         " equal to the value of ( pps_pic_width_in_luma_samples / MinCbSizeY − 1 )." );
  pcPPS->setPicWidthInLumaSamples( pps_pic_width_in_luma_samples );

  X_READ_UVLC( pps_pic_height_in_luma_samples, 1, sps->getMaxPicHeightInLumaSamples() );
  CHECK( pps_pic_height_in_luma_samples & std::max( 8 - 1, MinCbSizeY - 1 ), "pps_pic_height_in_luma_samples not a multiple of 8 or MinCbSizeY" );
  CHECK( !sps->getResChangeInClvsEnabledFlag() && pps_pic_height_in_luma_samples != sps->getMaxPicHeightInLumaSamples(),
         "When sps_res_change_in_clvs_allowed_flag equal to 0, the value of pps_pic_height_in_luma_samples shall be equal"
         " to sps_pic_height_max_in_luma_samples." )
  pcPPS->setPicHeightInLumaSamples( pps_pic_height_in_luma_samples );

  const unsigned PicWidthInCtbsY     = ( pps_pic_width_in_luma_samples + CtbSizeY - 1 ) / CtbSizeY;      (void) PicWidthInCtbsY;
  const unsigned PicHeightInCtbsY    = ( pps_pic_height_in_luma_samples + CtbSizeY - 1 ) / CtbSizeY;     (void) PicHeightInCtbsY;
  const unsigned PicSizeInCtbsY      = PicWidthInCtbsY * PicHeightInCtbsY;                               (void) PicSizeInCtbsY;
  const unsigned PicWidthInMinCbsY   = pps_pic_width_in_luma_samples / MinCbSizeY;                       (void) PicWidthInMinCbsY;
  const unsigned PicHeightInMinCbsY  = pps_pic_height_in_luma_samples / MinCbSizeY;                      (void) PicHeightInMinCbsY;
  const unsigned PicSizeInMinCbsY    = PicWidthInMinCbsY * PicHeightInMinCbsY;                           (void) PicSizeInMinCbsY;
  const unsigned PicSizeInSamplesY   = pps_pic_width_in_luma_samples * pps_pic_height_in_luma_samples;   (void) PicSizeInSamplesY;
  const unsigned PicWidthInSamplesC  = pps_pic_width_in_luma_samples / SubWidthC;                        (void) PicWidthInSamplesC;
  const unsigned PicHeightInSamplesC = pps_pic_height_in_luma_samples / SubHeightC;                      (void) PicHeightInSamplesC;

  X_READ_FLAG( pps_conformance_window_flag );
  CHECK( pps_conformance_window_flag != 0 && pcPPS->getPicWidthInLumaSamples() == sps->getMaxPicWidthInLumaSamples()
           && pcPPS->getPicHeightInLumaSamples() == sps->getMaxPicHeightInLumaSamples(),
         "pps_conformance_window_flag shall be equal to 0" )
  pcPPS->setConformanceWindowPresentFlag( pps_conformance_window_flag );

  if( pps_conformance_window_flag != 0 )
  {
    X_READ_UVLC_NO_RANGE( pps_conf_win_left_offset );
    X_READ_UVLC_NO_RANGE( pps_conf_win_right_offset );
    X_READ_UVLC_NO_RANGE( pps_conf_win_top_offset );
    X_READ_UVLC_NO_RANGE( pps_conf_win_bottom_offset );

    CHECK( SubWidthC * ( pps_conf_win_left_offset + pps_conf_win_right_offset ) >= pps_pic_width_in_luma_samples,
           "pps_conf_win_left_offset + pps_conf_win_right_offset too large" );
    CHECK( SubHeightC * ( pps_conf_win_top_offset + pps_conf_win_bottom_offset ) >= pps_pic_height_in_luma_samples,
           "pps_conf_win_top_offset + pps_conf_win_bottom_offset too large" );

    Window& conf = pcPPS->getConformanceWindow();
    conf.setWindowLeftOffset( pps_conf_win_left_offset );
    conf.setWindowRightOffset( pps_conf_win_right_offset );
    conf.setWindowTopOffset( pps_conf_win_top_offset );
    conf.setWindowBottomOffset( pps_conf_win_bottom_offset );
  }

  X_READ_FLAG_CHECK( pps_scaling_window_explicit_signalling_flag,
                     !sps->getRprEnabledFlag() && pps_scaling_window_explicit_signalling_flag != 0,
                     "When sps_ref_pic_resampling_enabled_flag is equal to 0, the value of pps_scaling_window_explicit_signalling_flag shall be equal to 0" );

  if( pps_scaling_window_explicit_signalling_flag != 0 )
  {
    X_READ_SVLC_NO_RANGE( pps_scaling_win_left_offset );
    X_READ_SVLC_NO_RANGE( pps_scaling_win_right_offset );
    X_READ_SVLC_NO_RANGE( pps_scaling_win_top_offset );
    X_READ_SVLC_NO_RANGE( pps_scaling_win_bottom_offset );

    CHECK( SubWidthC * pps_scaling_win_left_offset < -(int) pps_pic_width_in_luma_samples * 15
             || SubWidthC * pps_scaling_win_left_offset >= (int) pps_pic_width_in_luma_samples,
           "pps_scaling_win_left_offset (" << pps_scaling_win_left_offset << ") out of bounds" );
    CHECK( SubWidthC * pps_scaling_win_right_offset < -(int) pps_pic_width_in_luma_samples * 15
             || SubWidthC * pps_scaling_win_right_offset >= (int) pps_pic_width_in_luma_samples,
           "pps_scaling_win_right_offset (" << pps_scaling_win_right_offset << ") out of bounds" );
    CHECK( SubHeightC * pps_scaling_win_top_offset < -(int) pps_pic_height_in_luma_samples * 15
             || SubHeightC * pps_scaling_win_top_offset >= (int) pps_pic_height_in_luma_samples,
           "pps_scaling_win_top_offset (" << pps_scaling_win_top_offset << ") out of bounds" );
    CHECK( SubHeightC * pps_scaling_win_bottom_offset < -(int) pps_pic_height_in_luma_samples * 15
             || SubHeightC * pps_scaling_win_bottom_offset >= (int) pps_pic_height_in_luma_samples,
           "pps_scaling_win_bottom_offset (" << pps_scaling_win_bottom_offset << ") out of bounds" );

    CHECK( SubWidthC * ( pps_scaling_win_left_offset + pps_scaling_win_right_offset ) < -(int) pps_pic_width_in_luma_samples * 15
             || SubWidthC * ( pps_scaling_win_left_offset + pps_scaling_win_right_offset ) >= (int) pps_pic_width_in_luma_samples,
           "SubWidthC * ( pps_scaling_win_left_offset + pps_scaling_win_right_offset ) shall be greater than or equal to"
           " -pps_pic_width_in_luma_samples * 15 and less than pps_pic_width_in_luma_samples."
           " (pps_scaling_win_left_offset + pps_scaling_win_right_offset = "
             << pps_scaling_win_left_offset + pps_scaling_win_right_offset << ")" );

    CHECK( SubHeightC * ( pps_scaling_win_top_offset + pps_scaling_win_bottom_offset ) < -(int) pps_pic_height_in_luma_samples * 15
             || SubHeightC * ( pps_scaling_win_top_offset + pps_scaling_win_bottom_offset ) >= (int) pps_pic_height_in_luma_samples,
           "SubHeightC * ( pps_scaling_win_top_offset + pps_scaling_win_bottom_offset ) shall be greater than or equal to"
           " -pps_pic_height_in_luma_samples * 15 and less than pps_pic_height_in_luma_samples."
           " (pps_scaling_win_top_offset + pps_scaling_win_bottom_offset = "
             << pps_scaling_win_top_offset + pps_scaling_win_bottom_offset << ")" )

#if 0
    // these checks need to be moved to the creation of ref pic lists
    //
    const int CurrPicScalWinWidthL  = pps_pic_width_in_luma_samples - SubWidthC * ( pps_scaling_win_right_offset + pps_scaling_win_left_offset );
    const int CurrPicScalWinHeightL = pps_pic_height_in_luma_samples - SubHeightC * ( pps_scaling_win_bottom_offset + pps_scaling_win_top_offset );
    CHECK( CurrPicScalWinWidthL * 2 < refPicScalWinWidthL,
           "Requirement of bitstream conformance: CurrPicScalWinWidthL * 2 is greater than or equal to refPicScalWinWidthL." );
    CHECK( CurrPicScalWinHeightL * 2 < refPicScalWinHeightL,
           "Requirement of bitstream conformance: CurrPicScalWinHeightL * 2 is greater than or equal to refPicScalWinHeightL." );
    CHECK( CurrPicScalWinWidthL > refPicScalWinWidthL * 8,
           "Requirement of bitstream conformance: CurrPicScalWinWidthL is less than or equal to refPicScalWinWidthL * 8." );
    CHECK( CurrPicScalWinHeightL > refPicScalWinHeightL * 8,
           "Requirement of bitstream conformance: CurrPicScalWinHeightL is less than or equal to refPicScalWinHeightL * 8." );
    CHECK( CurrPicScalWinWidthL * sps->getMaxPicWidthInLumaSamples() < refPicScalWinWidthL * ( pps_pic_width_in_luma_samples - std::max( 8, MinCbSizeY ) ),
           "Requirement of bitstream conformance: CurrPicScalWinWidthL * sps_pic_width_max_in_luma_samples is greater than or equal to"
           " refPicScalWinWidthL * ( pps_pic_width_in_luma_samples - Max( 8, MinCbSizeY ) )." );
    CHECK( CurrPicScalWinHeightL * sps->getMaxPicHeightInLumaSamples() < refPicScalWinHeightL * ( pps_pic_height_in_luma_samples - std::max( 8, MinCbSizeY ) ),
           "Requirement of bitstream conformance: CurrPicScalWinHeightL * sps_pic_height_max_in_luma_samples is greater than or equal to"
           " refPicScalWinHeightL * ( pps_pic_height_in_luma_samples - Max( 8, MinCbSizeY ) )." );
#endif

    Window& scalingWindow = pcPPS->getScalingWindow();
    scalingWindow.setWindowLeftOffset( pps_scaling_win_left_offset );
    scalingWindow.setWindowRightOffset( pps_scaling_win_right_offset );
    scalingWindow.setWindowTopOffset( pps_scaling_win_top_offset );
    scalingWindow.setWindowBottomOffset( pps_scaling_win_bottom_offset );
  }
  else
  {
    pcPPS->setScalingWindow( pcPPS->getConformanceWindow() );
  }

  X_READ_FLAG( pps_output_flag_present_flag );
  pcPPS->setOutputFlagPresentFlag( pps_output_flag_present_flag );

  X_READ_FLAG_CHECK( pps_no_pic_partition_flag,
                     ( sps->getNumSubPics() > 1 || pps_mixed_nalu_types_in_pic_flag == 1 ) && pps_no_pic_partition_flag != 0,
                     "When sps_num_subpics_minus1 is greater than 0 or pps_mixed_nalu_types_in_pic_flag is equal to 1, the value of pps_no_pic_partition_flag "
                     "shall be equal to 0." );
  pcPPS->setNoPicPartitionFlag( pps_no_pic_partition_flag );

  X_READ_FLAG( pps_subpic_id_mapping_present_flag );
  CHECK( ( sps->getSubPicIdMappingExplicitlySignalledFlag() == 0 || sps->getSubPicIdMappingPresentFlag() == 1 ) && pps_subpic_id_mapping_present_flag != 0,
         "If sps_subpic_id_mapping_explicitly_signalled_flag is 0 or sps_subpic_id_mapping_present_flag is equal to 1, the value of"
         " pps_subpic_id_mapping_present_flag shall be equal to 0." );
  CHECK( ( sps->getSubPicIdMappingExplicitlySignalledFlag() == 1 && sps->getSubPicIdMappingPresentFlag() == 0 ) && pps_subpic_id_mapping_present_flag != 1,
         "Otherwise (sps_subpic_id_mapping_explicitly_signalled_flag is equal to 1 and sps_subpic_id_mapping_present_flag is equal to 0), the value of"
         " pps_subpic_id_mapping_present_flag  shall be equal to 1." );
  pcPPS->setSubPicIdMappingPresentFlag( pps_subpic_id_mapping_present_flag );

  if( pps_subpic_id_mapping_present_flag )
  {
    if( !pps_no_pic_partition_flag )
    {
      X_READ_UVLC( pps_num_subpics_minus1, 0, MAX_NUM_SUB_PICS - 1 );
      CHECK( pps_num_subpics_minus1 != sps->getNumSubPics() - 1, "pps_num_subpics_minus1 shall be equal to sps_num_subpics_minus1" );
      pcPPS->setNumSubPics( pps_num_subpics_minus1 + 1 );
    }
    else
    {
      // When pps_no_pic_partition_flag is equal to 1, the value of pps_num_subpics_minus1 is inferred to be equal to 0.
      pcPPS->setNumSubPics( 1 );
    }

    X_READ_UVLC( pps_subpic_id_len_minus1, 0, 15 );
    CHECK( pps_subpic_id_len_minus1 != sps->getSubPicIdLen() - 1, "pps_subpic_id_len_minus1 shall be equal to sps_subpic_id_len_minus1" );
    CHECK( ( 1 << ( pps_subpic_id_len_minus1 + 1 ) ) < pcPPS->getNumSubPics(), "pps_subpic_id_len too short" );
    pcPPS->setSubPicIdLen( pps_subpic_id_len_minus1 + 1 );

    for( int picIdx = 0; picIdx < pcPPS->getNumSubPics(); picIdx++ )
    {
      X_READ_CODE_NO_RANGE_idx( pps_subpic_id, "[i]", pcPPS->getSubPicIdLen() );
      pcPPS->setSubPicId( picIdx, pps_subpic_id );
    }
  }
  else
  {
    for( int picIdx = 0; picIdx < MAX_NUM_SUB_PICS; picIdx++ )
    {
      pcPPS->setSubPicId( picIdx, sps->getSubPicIdMappingExplicitlySignalledFlag() ? sps->getSubPicId( picIdx ) : picIdx );
    }
  }
  for( int i = 0; i < pcPPS->getNumSubPics(); ++i )
  {
    for( int j = 0; j < i; ++j )
    {
      CHECK( pcPPS->getSubPicId( i ) == pcPPS->getSubPicId( j ),
                         "It is a requirement of bitstream conformance that, for any two different values of i and j in the range of"
                         " 0 to sps_num_subpics_minus1, inclusive, SubpicIdVal[ i ] shall not be equal to SubpicIdVal[ j ]." )
    }
  }

  pcPPS->resetTileSliceInfo();

  if( !pps_no_pic_partition_flag )
  {
    // CTU size - required to match size in SPS
    X_READ_CODE( pps_log2_ctu_size_minus5, 2, 0,2 );
    CHECK( pps_log2_ctu_size_minus5 != getLog2( sps->getCTUSize() ) - 5, "pps_log2_ctu_size_minus5 shall be equal to sps_log2_ctu_size_minus5" );
    pcPPS->setLog2CtuSize( pps_log2_ctu_size_minus5 + 5 );

    // number of explicit tile columns/rows

    X_READ_UVLC( pps_num_exp_tile_columns_minus1, 0, PicWidthInCtbsY - 1 );
    pcPPS->setNumExpTileColumns( pps_num_exp_tile_columns_minus1 + 1 );

    X_READ_UVLC( pps_num_exp_tile_rows_minus1, 0, PicHeightInCtbsY - 1 );
    pcPPS->setNumExpTileRows( pps_num_exp_tile_rows_minus1 + 1 );
    CHECK( pcPPS->getNumExpTileColumns() > MAX_TILE_COLS, "Number of explicit tile columns exceeds valid range" );

    // tile sizes
    for( int colIdx = 0; colIdx < pcPPS->getNumExpTileColumns(); colIdx++ )
    {
      X_READ_UVLC_idx( pps_tile_column_width_minus1, "[i]", 0, PicWidthInCtbsY - 1 );
      pcPPS->addTileColumnWidth( pps_tile_column_width_minus1 + 1 );
    }
    for( int rowIdx = 0; rowIdx < pcPPS->getNumExpTileRows(); rowIdx++ )
    {
      X_READ_UVLC_idx( pps_tile_row_height_minus1, "[i]", 0, PicHeightInCtbsY - 1 );
      pcPPS->addTileRowHeight( pps_tile_row_height_minus1 + 1 );
    }
    pcPPS->initTiles();

    // rectangular slice signalling
    if( pcPPS->getNumTiles() > 1 )
    {
      X_READ_FLAG( pps_loop_filter_across_tiles_enabled_flag );
      pcPPS->setLoopFilterAcrossTilesEnabledFlag( pps_loop_filter_across_tiles_enabled_flag );
      X_READ_FLAG( pps_rect_slice_flag );
      CHECK( ( sps->getSubPicInfoPresentFlag() || pps_mixed_nalu_types_in_pic_flag ) && pps_rect_slice_flag != 1,
             "When sps_subpic_info_present_flag is equal to 1 or pps_mixed_nalu_types_in_pic_flag is equal to 1,"
             " the value of pps_rect_slice_flag shall be equal to 1." );
      pcPPS->setRectSliceFlag( pps_rect_slice_flag );
    }
    else
    {
      pcPPS->setLoopFilterAcrossTilesEnabledFlag( false );
      pcPPS->setRectSliceFlag( true );
    }

    if( pcPPS->getRectSliceFlag() )
    {
      X_READ_FLAG( pps_single_slice_per_subpic_flag );
      pcPPS->setSingleSlicePerSubPicFlag( pps_single_slice_per_subpic_flag );
    }

    if( pcPPS->getRectSliceFlag() && !pcPPS->getSingleSlicePerSubPicFlag() )
    {
      X_READ_UVLC( pps_num_slices_in_pic_minus1, 0, MAX_SLICES - 1 /*  TODO: should be MaxSlicesPerAU -1  */ );
      pcPPS->setNumSlicesInPic( pps_num_slices_in_pic_minus1 + 1 );

      if( pps_num_slices_in_pic_minus1 > 1 )
      {
        X_READ_FLAG( pps_tile_idx_delta_present_flag );
        pcPPS->setTileIdxDeltaPresentFlag( pps_tile_idx_delta_present_flag );
      }
      pcPPS->initRectSlices();

      int32_t tileIdx = 0;
      // read rectangular slice parameters
      for( int sliceIdx = 0; sliceIdx < pcPPS->getNumSlicesInPic() - 1; sliceIdx++ )
      {
        pcPPS->setSliceTileIdx( sliceIdx, tileIdx );   // this is SliceTopLeftTileIdx[ i ]

        // complete tiles within a single slice
        if( tileIdx % pcPPS->getNumTileColumns() != pcPPS->getNumTileColumns() - 1 )
        {
          X_READ_UVLC_idx( pps_slice_width_in_tiles_minus1, "[i]", 0, pcPPS->getNumTileColumns() - 1 );
          pcPPS->setSliceWidthInTiles( sliceIdx, pps_slice_width_in_tiles_minus1 + 1 );
        }
        else
        {
          pcPPS->setSliceWidthInTiles( sliceIdx, 1 );
        }

        if( tileIdx / pcPPS->getNumTileColumns() != pcPPS->getNumTileRows() - 1
            && ( pcPPS->getTileIdxDeltaPresentFlag() || tileIdx % pcPPS->getNumTileColumns() == 0 ) )
        {
          X_READ_UVLC_idx( pps_slice_height_in_tiles_minus1, "[i]", 0, pcPPS->getNumTileRows() - 1 );
          pcPPS->setSliceHeightInTiles( sliceIdx, pps_slice_height_in_tiles_minus1 + 1 );
        }
        else
        {
          if( tileIdx / pcPPS->getNumTileColumns() == pcPPS->getNumTileRows() - 1 )
          {
            pcPPS->setSliceHeightInTiles( sliceIdx, 1 );
          }
          else
          {
            pcPPS->setSliceHeightInTiles( sliceIdx, pcPPS->getSliceHeightInTiles( sliceIdx - 1 ) );
          }
        }

        // multiple slices within a single tile special case
        if( pcPPS->getSliceWidthInTiles( sliceIdx ) == 1 && pcPPS->getSliceHeightInTiles( sliceIdx ) == 1 )
        {
          if( pcPPS->getTileRowHeight( tileIdx / pcPPS->getNumTileColumns() ) > 1 )
          {
            X_READ_UVLC_idx( pps_num_exp_slices_in_tile, "[i]", 0, pcPPS->getTileRowHeight( tileIdx / pcPPS->getNumTileColumns() ) - 1 );
            if( pps_num_exp_slices_in_tile == 0 )
            {
              pcPPS->setNumSlicesInTile( sliceIdx, 1 );
              pcPPS->setSliceHeightInCtu( sliceIdx, pcPPS->getTileRowHeight( tileIdx / pcPPS->getNumTileColumns() ) );
            }
            else   // ( pps_num_exp_slices_in_tile > 0 )
            {
              uint32_t remTileRowHeight         = pcPPS->getTileRowHeight( tileIdx / pcPPS->getNumTileColumns() );
              uint32_t lastExpSliceHeightInCtus = 0;

              int j = 0;
              for( ; j < pps_num_exp_slices_in_tile; j++ )
              {
                X_READ_UVLC_idx( pps_exp_slice_height_in_ctus_minus1, "[i][j]", 0, pcPPS->getTileRowHeight( tileIdx / pcPPS->getNumTileColumns() ) - 1 );
                pcPPS->setSliceHeightInCtu( sliceIdx + j, pps_exp_slice_height_in_ctus_minus1 + 1 );
                remTileRowHeight -= ( pps_exp_slice_height_in_ctus_minus1 + 1 );

                lastExpSliceHeightInCtus = pps_exp_slice_height_in_ctus_minus1 + 1;
              }

              uint32_t uniformSliceHeight = lastExpSliceHeightInCtus;
              while( remTileRowHeight >= uniformSliceHeight )
              {
                pcPPS->setSliceHeightInCtu( sliceIdx + j, uniformSliceHeight );
                remTileRowHeight -= uniformSliceHeight;
                j++;
              }
              if( remTileRowHeight > 0 )
              {
                pcPPS->setSliceHeightInCtu( sliceIdx + j, remTileRowHeight );
                j++;
              }
              for( int k = 0; k < j; k++ )
              {
                pcPPS->setNumSlicesInTile( sliceIdx + k, j );
                pcPPS->setSliceWidthInTiles( sliceIdx + k, 1 );
                pcPPS->setSliceHeightInTiles( sliceIdx + k, 1 );
                pcPPS->setSliceTileIdx( sliceIdx + k, tileIdx );
              }
              sliceIdx += ( j - 1 );
            }    // ( pps_num_exp_slices_in_tile > 0 )
          }
          else   // ( pps->getTileRowHeight( tileIdx / pps->getNumTileColumns() ) <= 1 )
          {
            pcPPS->setNumSlicesInTile( sliceIdx, 1 );
            pcPPS->setSliceHeightInCtu( sliceIdx, pcPPS->getTileRowHeight( tileIdx / pcPPS->getNumTileColumns() ) );
          }
        }   // ( pps->getSliceWidthInTiles(sliceIdx) == 1 && pps->getSliceHeightInTiles(sliceIdx) == 1 )

        // tile index offset to start of next slice
        if( sliceIdx < pps_num_slices_in_pic_minus1 )
        {
          if( pcPPS->getTileIdxDeltaPresentFlag() )
          {
            X_READ_SVLC_idx( pps_tile_idx_delta_val, "[i]", -(int) pcPPS->getNumTiles() + 1, (int) pcPPS->getNumTiles() - 1 );
            CHECK( pps_tile_idx_delta_val == 0, "When present, the value of pps_tile_idx_delta_val[ i ] shall not be equal to 0." );

            // TODO: When pps_rect_slice_flag is equal to 1, it is a requirement of bitstrream conformance that, for any two slices, with picture-
            //       level slice indices idxA and idxB, that belong to the same picture and different subpictures, when
            //       SubpicIdxForSlice[idxA] is less than SubpicIdxForSlice[idxB], the value of idxA shall be less than idxB.
            // CHECK( pps->getRectSliceFlag() && /* TODO */ )

            tileIdx += pps_tile_idx_delta_val;
            CHECK( tileIdx < 0 || tileIdx >= (int) pcPPS->getNumTiles(), "Invalid tile_idx_delta." );
          }
          else
          {
            tileIdx += pcPPS->getSliceWidthInTiles( sliceIdx );
            if( tileIdx % pcPPS->getNumTileColumns() == 0 )
            {
              tileIdx += ( pcPPS->getSliceHeightInTiles( sliceIdx ) - 1 ) * pcPPS->getNumTileColumns();
            }
          }
        }
        // TODO: hier?
        CHECK( tileIdx < 0 || tileIdx >= (int) pcPPS->getNumTiles(), "Invalid tile_idx_delta." );

      }   // for( int i = 0; i < pps->getNumSlicesInPic()-1; i++ )
      pcPPS->setSliceTileIdx( pcPPS->getNumSlicesInPic() - 1, tileIdx );

    }   // if( pps->getRectSliceFlag() && !pps->getSingleSlicePerSubPicFlag() )

    if( !pcPPS->getRectSliceFlag() || pcPPS->getSingleSlicePerSubPicFlag() || pcPPS->getNumSlicesInPic() > 1 )
    {
      X_READ_FLAG( pps_loop_filter_across_slices_enabled_flag );
      pcPPS->setLoopFilterAcrossSlicesEnabledFlag( pps_loop_filter_across_slices_enabled_flag );
    }
    else
    {
      pcPPS->setLoopFilterAcrossSlicesEnabledFlag( false );
    }
  }
  else   // if( !pps_no_pic_partition_flag )
  {
    // When pps_no_pic_partition_flag is equal to 1, the value of pps_single_slice_per_subpic_flag is inferred to be equal to 1.
    pcPPS->setSingleSlicePerSubPicFlag( 1 );
  }

  X_READ_FLAG( pps_cabac_init_present_flag );
  pcPPS->setCabacInitPresentFlag( pps_cabac_init_present_flag );

  {
    X_READ_UVLC_idx( pps_num_ref_idx_default_active_minus1, "[0]", 0, 14 );
    pcPPS->setNumRefIdxL0DefaultActive( pps_num_ref_idx_default_active_minus1 + 1 );
  }
  {
    X_READ_UVLC_idx( pps_num_ref_idx_default_active_minus1, "[1]", 0, 14 );
    pcPPS->setNumRefIdxL1DefaultActive( pps_num_ref_idx_default_active_minus1 + 1 );
  }

  X_READ_FLAG( pps_rpl1_idx_present_flag );
  pcPPS->setRpl1IdxPresentFlag( pps_rpl1_idx_present_flag );

  X_READ_FLAG_CHECK( pps_weighted_pred_flag,
                     sps->getUseWP() == 0 && pps_weighted_pred_flag != 0,
                     "sps_weighted_pred_flag is equal to 0, the value of pps_weighted_pred_flag shall be equal to 0." );
  pcPPS->setUseWP( pps_weighted_pred_flag );

  X_READ_FLAG_CHECK( pps_weighted_bipred_flag,
                     sps->getUseWPBiPred() == 0 && pps_weighted_bipred_flag != 0,
                     "When sps_weighted_bipred_flag is equal to 0, the value of pps_weighted_bipred_flag shall be equal to 0." );
  pcPPS->setWPBiPred( pps_weighted_bipred_flag );

  X_READ_FLAG_CHECK( pps_ref_wraparound_enabled_flag,
                     ( sps->getUseWrapAround() == 0 || CtbSizeY / MinCbSizeY + 1 > pps_pic_width_in_luma_samples / MinCbSizeY - 1 )
                       && pps_ref_wraparound_enabled_flag != 0,
                     "When sps_ref_wraparound_enabled_flag is equal to 0 or the value of CtbSizeY / MinCbSizeY + 1 is greater than"
                     " pps_pic_width_in_luma_samples / MinCbSizeY − 1, the value of pps_ref_wraparound_enabled_flag shall be equal to 0." );
  pcPPS->setUseWrapAround( pps_ref_wraparound_enabled_flag );

  if( pps_ref_wraparound_enabled_flag )
  {
    X_READ_UVLC( pps_pic_width_minus_wraparound_offset, 0, ( pps_pic_width_in_luma_samples / MinCbSizeY ) - ( CtbSizeY / MinCbSizeY ) - 2 );
    pcPPS->setPicWidthMinusWrapAroundOffset( pps_pic_width_minus_wraparound_offset );
  }

  X_READ_SVLC( pps_init_qp_minus26, -( 26 + sps->getQpBDOffset() ), 37 );
  pcPPS->setPicInitQPMinus26( pps_init_qp_minus26 );

  X_READ_FLAG( pps_cu_qp_delta_enabled_flag );
  pcPPS->setUseDQP( pps_cu_qp_delta_enabled_flag );

  X_READ_FLAG_CHECK( pps_chroma_tool_offsets_present_flag,
                     sps->getChromaFormatIdc() == 0 && pps_chroma_tool_offsets_present_flag != 0,
                     "When sps_chroma_format_idc is equal to 0, the value of pps_chroma_tool_offsets_present_flag shall be equal to 0." );
  pcPPS->setPPSChromaToolFlag( pps_chroma_tool_offsets_present_flag );

  if( pps_chroma_tool_offsets_present_flag )
  {
    X_READ_SVLC( pps_cb_qp_offset, -12, 12 );
    pcPPS->setQpOffset( COMPONENT_Cb, pps_cb_qp_offset );

    X_READ_SVLC( pps_cr_qp_offset, -12, 12 );
    pcPPS->setQpOffset( COMPONENT_Cr, pps_cr_qp_offset );

    X_READ_FLAG_CHECK( pps_joint_cbcr_qp_offset_present_flag,
                       ( sps->getChromaFormatIdc() == 0 || sps->getJointCbCrEnabledFlag() == 0 ) && pps_joint_cbcr_qp_offset_present_flag != 0,
                       "When sps_chroma_format_idc is equal to 0 or sps_joint_cbcr_enabled_flag is equal to 0, the value of"
                       " pps_joint_cbcr_qp_offset_present_flag shall be equal to 0." );
    pcPPS->setJointCbCrQpOffsetPresentFlag( pps_joint_cbcr_qp_offset_present_flag );

    if( pps_joint_cbcr_qp_offset_present_flag )
    {
      X_READ_SVLC( pps_joint_cbcr_qp_offset_value, -12, 12 );
      pcPPS->setQpOffset( JOINT_CbCr, pps_joint_cbcr_qp_offset_value );
    }
    else
    {
      pcPPS->setQpOffset( JOINT_CbCr, 0 );
    }

    X_READ_FLAG( pps_slice_chroma_qp_offsets_present_flag );
    pcPPS->setSliceChromaQpFlag( pps_slice_chroma_qp_offsets_present_flag );

    X_READ_FLAG( pps_cu_chroma_qp_offset_list_enabled_flag );
    if( !pps_cu_chroma_qp_offset_list_enabled_flag )
    {
      pcPPS->clearChromaQpOffsetList();
    }
    else
    {
      X_READ_UVLC( pps_chroma_qp_offset_list_len_minus1, 0, MAX_QP_OFFSET_LIST_SIZE - 1 );

      for( int cuChromaQpOffsetIdx = 0; cuChromaQpOffsetIdx <= pps_chroma_qp_offset_list_len_minus1; cuChromaQpOffsetIdx++ )
      {
        X_READ_SVLC_idx( pps_cb_qp_offset_list, "[i]", -12, 12 );
        X_READ_SVLC_idx( pps_cr_qp_offset_list, "[i]", -12, 12 );
        int jointCbCrOffset = 0;
        if( pps_joint_cbcr_qp_offset_present_flag )
        {
          X_READ_SVLC_idx( pps_joint_cbcr_qp_offset_list, "[i]", -12, 12 );
          jointCbCrOffset = pps_joint_cbcr_qp_offset_list;
        }
        // table uses +1 for index (see comment inside the function)
        pcPPS->setChromaQpOffsetListEntry( cuChromaQpOffsetIdx + 1, pps_cb_qp_offset_list, pps_cr_qp_offset_list, jointCbCrOffset );
      }
      CHECK( pcPPS->getChromaQpOffsetListLen() != pps_chroma_qp_offset_list_len_minus1 + 1, "Invalid chroma QP offset list length" );
    }
  }
  else
  {
    pcPPS->setQpOffset( COMPONENT_Cb, 0 );
    pcPPS->setQpOffset( COMPONENT_Cr, 0 );
    pcPPS->setJointCbCrQpOffsetPresentFlag( 0 );
    pcPPS->setSliceChromaQpFlag( 0 );
    pcPPS->clearChromaQpOffsetList();
  }

  X_READ_FLAG( pps_deblocking_filter_control_present_flag );
  pcPPS->setDeblockingFilterControlPresentFlag( pps_deblocking_filter_control_present_flag );

  if( pps_deblocking_filter_control_present_flag )
  {
    X_READ_FLAG( pps_deblocking_filter_override_enabled_flag );
    pcPPS->setDeblockingFilterOverrideEnabledFlag( pps_deblocking_filter_override_enabled_flag );

    X_READ_FLAG( pps_deblocking_filter_disabled_flag );
    pcPPS->setPPSDeblockingFilterDisabledFlag( pps_deblocking_filter_disabled_flag );

    if( !pps_no_pic_partition_flag && pps_deblocking_filter_override_enabled_flag )
    {
      X_READ_FLAG( pps_dbf_info_in_ph_flag );
      pcPPS->setDbfInfoInPhFlag( pps_dbf_info_in_ph_flag );
    }

    if( !pps_deblocking_filter_disabled_flag )
    {
      X_READ_SVLC( pps_luma_beta_offset_div2, -12, 12 );
      pcPPS->setDeblockingFilterBetaOffsetDiv2( pps_luma_beta_offset_div2 );

      X_READ_SVLC( pps_luma_tc_offset_div2, -12, 12 );
      pcPPS->setDeblockingFilterTcOffsetDiv2( pps_luma_tc_offset_div2 );

      if( pps_chroma_tool_offsets_present_flag )
      {
        X_READ_SVLC( pps_cb_beta_offset_div2, -12, 12 );
        pcPPS->setDeblockingFilterCbBetaOffsetDiv2( pps_cb_beta_offset_div2 );

        X_READ_SVLC( pps_cb_tc_offset_div2, -12, 12 );
        pcPPS->setDeblockingFilterCbTcOffsetDiv2( pps_cb_tc_offset_div2 );

        X_READ_SVLC( pps_cr_beta_offset_div2, -12, 12 );
        pcPPS->setDeblockingFilterCrBetaOffsetDiv2( pps_cr_beta_offset_div2 );

        X_READ_SVLC( pps_cr_tc_offset_div2, -12, 12 );
        pcPPS->setDeblockingFilterCrTcOffsetDiv2( pps_cr_tc_offset_div2 );
      }
      else
      {
        pcPPS->setDeblockingFilterCbBetaOffsetDiv2( pps_luma_beta_offset_div2 );
        pcPPS->setDeblockingFilterCbTcOffsetDiv2( pps_luma_tc_offset_div2 );
        pcPPS->setDeblockingFilterCrBetaOffsetDiv2( pps_luma_beta_offset_div2 );
        pcPPS->setDeblockingFilterCrTcOffsetDiv2( pps_luma_tc_offset_div2 );
      }
    }
  }

  if( !pps_no_pic_partition_flag )
  {
    X_READ_FLAG( pps_rpl_info_in_ph_flag );
    pcPPS->setRplInfoInPhFlag( pps_rpl_info_in_ph_flag );

    X_READ_FLAG( pps_sao_info_in_ph_flag );
    pcPPS->setSaoInfoInPhFlag( pps_sao_info_in_ph_flag );

    X_READ_FLAG( pps_alf_info_in_ph_flag );
    pcPPS->setAlfInfoInPhFlag( pps_alf_info_in_ph_flag );

    if( ( pps_weighted_pred_flag || pps_weighted_bipred_flag ) && pps_rpl_info_in_ph_flag )
    {
      X_READ_FLAG( pps_wp_info_in_ph_flag );
      pcPPS->setWpInfoInPhFlag( pps_wp_info_in_ph_flag );
    }

    X_READ_FLAG( pps_qp_delta_info_in_ph_flag );
    pcPPS->setQpDeltaInfoInPhFlag( pps_qp_delta_info_in_ph_flag );
  }

  X_READ_FLAG( pps_picture_header_extension_present_flag );
  pcPPS->setPictureHeaderExtensionPresentFlag( pps_picture_header_extension_present_flag );

  X_READ_FLAG( pps_slice_header_extension_present_flag );
  pcPPS->setSliceHeaderExtensionPresentFlag( pps_slice_header_extension_present_flag );

  X_READ_FLAG( pps_extension_flag );
  if( pps_extension_flag )
  {
    while( xMoreRbspData() )
    {
      X_READ_FLAG( pps_extension_data_flag );
      (void) pps_extension_data_flag;
    }
  }

  xReadRbspTrailingBits();

  // ================================
  //  PPS READING DONE
  // ================================

  if( pcPPS->getPicWidthInLumaSamples() == sps->getMaxPicWidthInLumaSamples() && pcPPS->getPicHeightInLumaSamples() == sps->getMaxPicHeightInLumaSamples() )
  {
    CHECK( pcPPS->getConformanceWindowPresentFlag(),
                       "When pps_pic_width_in_luma_samples is equal to sps_pic_width_max_in_luma_samples and "
                       "pps_pic_height_in_luma_samples is equal to sps_pic_height_max_in_luma_samples, the value of "
                       "pps_conformance_window_flag shall be equal to 0" );

    pcPPS->setConformanceWindow( sps->getConformanceWindow() );

    if( !pcPPS->getScalingWindow().getWindowEnabledFlag() )
    {
      pcPPS->setScalingWindow( pcPPS->getConformanceWindow() );
    }
  }

  pcPPS->finalizePPSPartitioning( sps );

  // set wraparound offset from PPS and SPS info
  int minCbSizeY = ( 1 << sps->getLog2MinCodingBlockSize() );
  CHECK( !sps->getUseWrapAround() && pcPPS->getUseWrapAround(),
         "When sps_ref_wraparound_enabled_flag is equal to 0, the value of pps_ref_wraparound_enabled_flag shall be equal to 0." );
  CHECK( sps->getCTUSize() / minCbSizeY + 1 > pcPPS->getPicWidthInLumaSamples() / minCbSizeY - 1 && pcPPS->getUseWrapAround(),
         "When the value of CtbSizeY / MinCbSizeY + 1 is greater than pic_width_in_luma_samples / MinCbSizeY - 1, the value of "
         "pps_ref_wraparound_enabled_flag shall be equal to 0." );
  if( pcPPS->getUseWrapAround() )
  {
    CHECK( pcPPS->getPicWidthMinusWrapAroundOffset() > pcPPS->getPicWidthInLumaSamples() / minCbSizeY - sps->getCTUSize() / minCbSizeY - 2,
           "pps_pic_width_minus_wraparound_ofsfet shall be less than or equal to pps_pic_width_in_luma_samples/MinCbSizeY - CtbSizeY/MinCbSizeY-2" );
    pcPPS->setWrapAroundOffset( minCbSizeY * ( pcPPS->getPicWidthInLumaSamples() / minCbSizeY - pcPPS->getPicWidthMinusWrapAroundOffset() ) );
  }
  else
  {
    pcPPS->setWrapAroundOffset( 0 );
  }

  pcPPS->pcv = std::make_unique<PreCalcValues>( *sps, *pcPPS );
}

bool HLSyntaxReader::parseAPS( APS* aps )
{
#if ENABLE_TRACING
  xTraceAPSHeader();
#endif

  X_READ_CODE_NO_RANGE( aps_params_type, 3 );
  // if not in supported range, this APS is ignored `switch() -> default`
  aps->setAPSType( aps_params_type );

  X_READ_CODE_NO_RANGE( adaptation_parameter_set_id, 5 );
  // range check happens later in the `switch( aps_params_type )`
  aps->setAPSId( adaptation_parameter_set_id );

  X_READ_FLAG( aps_chroma_present_flag );
  aps->chromaPresentFlag = aps_chroma_present_flag;

  switch( aps_params_type )
  {
  case ALF_APS:
    CHECK_READ_RANGE( adaptation_parameter_set_id, 0, 7, "adaptation_parameter_set_id for ALF_APS" );
    parseAlfAps( aps );
    break;
  case LMCS_APS:
    CHECK_READ_RANGE( adaptation_parameter_set_id, 0, 3, "adaptation_parameter_set_id for LMCS_APS," );
    parseLmcsAps( aps );
    break;
  case SCALING_LIST_APS:
    CHECK_READ_RANGE( adaptation_parameter_set_id, 0, 7, "adaptation_parameter_set_id for SCALING_APS" );
    parseScalingListAps( aps );
    break;
  default:
    WARN( "unknown APS type (" << aps_params_type << ")" );
    return false;
  }

  X_READ_FLAG( aps_extension_flag );
  if( aps_extension_flag )
  {
    while( xMoreRbspData() )
    {
      X_READ_FLAG( aps_extension_data_flag );
      (void) aps_extension_data_flag;
    }
  }
  xReadRbspTrailingBits();

  return true;
}

void HLSyntaxReader::parseAlfAps( APS* aps )
{
  AlfSliceParam& param = aps->getAlfAPSParam();
  param.reset();

  X_READ_FLAG( alf_luma_new_filter );   //  standard: alf_luma_filter_signal_flag
  param.newFilterFlag[CHANNEL_TYPE_LUMA] = alf_luma_new_filter;

  CcAlfFilterParam& ccAlfParam = aps->getCcAlfAPSParam();
  ccAlfParam.reset();
  if( aps->chromaPresentFlag )
  {
    X_READ_FLAG( alf_chroma_new_filter );   // standard: alf_chroma_filter_signal_flag
    param.newFilterFlag[CHANNEL_TYPE_CHROMA] = alf_chroma_new_filter;

    X_READ_FLAG( alf_cc_cb_filter_signal_flag );
    ccAlfParam.newCcAlfFilter[COMPONENT_Cb - 1] = alf_cc_cb_filter_signal_flag;

    X_READ_FLAG( alf_cc_cr_filter_signal_flag );
    ccAlfParam.newCcAlfFilter[COMPONENT_Cr - 1] = alf_cc_cr_filter_signal_flag;
  }
  CHECK( param.newFilterFlag[CHANNEL_TYPE_LUMA] == 0 && param.newFilterFlag[CHANNEL_TYPE_CHROMA] == 0
         && ccAlfParam.newCcAlfFilter[COMPONENT_Cb - 1] == 0 && ccAlfParam.newCcAlfFilter[COMPONENT_Cr - 1] == 0,
         "bitstream conformance error: one of alf_luma_filter_signal_flag, alf_chroma_filter_signal_flag, "
         "alf_cross_component_cb_filter_signal_flag, and alf_cross_component_cr_filter_signal_flag shall be nonzero" );

  // The variable NumAlfFilters specifying the number of different adaptive loop filters is set equal to 25.
  static constexpr int NumAlfFilters = MAX_NUM_ALF_CLASSES;

  if( alf_luma_new_filter )
  {
    X_READ_FLAG( alf_luma_clip );
    param.nonLinearFlagLuma = alf_luma_clip;

    X_READ_UVLC( alf_luma_num_filters_signalled_minus1, 0, NumAlfFilters - 1 );
    param.numLumaFilters = alf_luma_num_filters_signalled_minus1 + 1;

    if( alf_luma_num_filters_signalled_minus1 > 0 )
    {
      const int length = (int) ceil( log2( alf_luma_num_filters_signalled_minus1 + 1 ) );
      for( int filtIdx = 0; filtIdx < NumAlfFilters; filtIdx++ )
      {
        X_READ_CODE( alf_luma_coeff_delta_idx, length, 0, alf_luma_num_filters_signalled_minus1 );
        param.filterCoeffDeltaIdx[filtIdx] = alf_luma_coeff_delta_idx;
      }
    }

    alfFilterCoeffs( param, false, 0 );
  }

  if( param.newFilterFlag[CHANNEL_TYPE_CHROMA] )
  {
    X_READ_FLAG( alf_nonlinear_enable_flag_chroma );
    param.nonLinearFlagChroma = alf_nonlinear_enable_flag_chroma;

    X_READ_UVLC( alf_chroma_num_alts_minus1, 0, MAX_NUM_ALF_ALTERNATIVES_CHROMA - 1 );
    param.numAlternativesChroma = alf_chroma_num_alts_minus1 + 1;

    for( int altIdx = 0; altIdx <= alf_chroma_num_alts_minus1; ++altIdx )
    {
      alfFilterCoeffs( param, true, altIdx );
    }
  }

  for( int ccIdx = 0; ccIdx < 2; ccIdx++ )
  {
    if( ccAlfParam.newCcAlfFilter[ccIdx] )
    {
      uint32_t code;
      READ_UVLC( code, ccIdx == 0 ? "alf_cc_cb_filters_signalled_minus1" : "alf_cc_cr_filters_signalled_minus1" );
      CHECK_READ_RANGE( code, 0, 3, ( ccIdx == 0 ? "alf_cc_cb_filters_signalled_minus1" : "alf_cc_cr_filters_signalled_minus1" ) )

      ccAlfParam.ccAlfFilterCount[ccIdx] = code + 1;

      for( int filterIdx = 0; filterIdx < ccAlfParam.ccAlfFilterCount[ccIdx]; filterIdx++ )
      {
        ccAlfParam.ccAlfFilterIdxEnabled[ccIdx][filterIdx] = true;

        const int numCoeff = g_alfNumCoeff[CC_ALF];

        short* coeff = ccAlfParam.ccAlfCoeff[ccIdx][filterIdx];
        // Filter coefficients
        for( int i = 0; i < numCoeff - 1; i++ )
        {
          READ_CODE( 3, code, ccIdx == 0 ? "alf_cc_cb_mapped_coeff_abs" : "alf_cc_cr_mapped_coeff_abs" );
          if( code )
          {
            coeff[i] = 1 << ( code - 1 );

            READ_FLAG( code, ccIdx == 0 ? "alf_cc_cb_coeff_sign" : "alf_cc_cr_coeff_sign" );
            coeff[i] *= 1 - 2 * code;
          }
          else
          {
            coeff[i] = 0;
          }
        }

        DTRACE( g_trace_ctx, D_SYNTAX, "%s coeff filterIdx %d: ", ccIdx == 0 ? "Cb" : "Cr", filterIdx );
        for( int i = 0; i < numCoeff; i++ )
        {
          DTRACE( g_trace_ctx, D_SYNTAX, "%d ", coeff[i] );
        }
        DTRACE( g_trace_ctx, D_SYNTAX, "\n" );
      }
    }
  }
}

void HLSyntaxReader::parseLmcsAps( APS* aps )
{
  SliceReshapeInfo& info = aps->getReshaperAPSInfo();
  info.reset();

  X_READ_UVLC( lmcs_min_bin_idx, 0, 15 );
  info.reshaperModelMinBinIdx = lmcs_min_bin_idx;

  X_READ_UVLC( lmcs_delta_max_bin_idx, 0, 15 );
  info.reshaperModelMaxBinIdx = PIC_CODE_CW_BINS - 1 - lmcs_delta_max_bin_idx;
  CHECK( info.reshaperModelMaxBinIdx < lmcs_min_bin_idx, "The value of LmcsMaxBinIdx shall be greater than or equal to lmcs_min_bin_idx." );

  X_READ_UVLC( lmcs_delta_cw_prec_minus1, 0, 14 );
  info.maxNbitsNeededDeltaCW = lmcs_delta_cw_prec_minus1 + 1;

  CHECK( info.maxNbitsNeededDeltaCW == 0, "wrong" );

  for( uint32_t i = info.reshaperModelMinBinIdx; i <= info.reshaperModelMaxBinIdx; i++ )
  {
    X_READ_CODE_NO_RANGE_idx( lmcs_delta_abs_cw, "[ i ]", info.maxNbitsNeededDeltaCW );
    info.reshaperModelBinCWDelta[i] = lmcs_delta_abs_cw;
    if( lmcs_delta_abs_cw )
    {
      X_READ_FLAG_idx( lmcs_delta_sign_cw_flag, "[ i ]" );
      info.reshaperModelBinCWDelta[i] *= 1 - 2 * lmcs_delta_sign_cw_flag;
    }
#if 0
    // TODO: needs bitdepth from SPS
    int OrgCW = ( 1 << BitDepth ) / 16;
    int lmcsCW = OrgCW + info.reshaperModelBinCWDelta[i];
    CHECK_READ_RANGE( lmcsCW, OrgCW >> 3, ( OrgCW << 3 ) - 1, "lmcsCW[i]" );
    // TODO: also CHECK( sum(lmcsCW[0..15]) > ( 1 << BitDepth ) − 1 );
#endif
  }

  if( aps->chromaPresentFlag )
  {
    X_READ_CODE_NO_RANGE( lmcs_delta_abs_crs, 3 );
    info.chrResScalingOffset = lmcs_delta_abs_crs;
    if( lmcs_delta_abs_crs > 0 )
    {
      X_READ_FLAG( lmcs_delta_sign_crs_flag );
      info.chrResScalingOffset *= ( 1 - 2 * lmcs_delta_sign_crs_flag );
    }
  }
  // TODO:
  // It is a requirement of bitstream conformance that, when lmcsCW[ i ] is not equal to 0, ( lmcsCW[ i ] + lmcsDeltaCrs )
  //   shall be in the range of ( OrgCW >> 3 ) to ( ( OrgCW << 3 ) - 1 ), inclusive
}

void HLSyntaxReader::parseScalingListAps( APS* aps )
{
  ScalingList& info = aps->getScalingList();
  parseScalingList( &info, aps->chromaPresentFlag );
}

static const int SARFixedRatios[][2] =
{
    { 1,  1 },
    { 12, 11 },
    { 10, 11 },
    { 16, 11 },
    { 40, 33 },
    { 24, 11 },
    { 20, 11 },
    { 32, 11 },
    { 80, 33 },
    { 18, 11 },
    { 15, 11 },
    { 64, 33 },
    { 160, 99 },
    { 4, 3 },
    { 3, 2 },
    { 2, 1 },
};

void HLSyntaxReader::parseVUI( VUI* pcVUI, unsigned vuiPayloadSize )
{
#if ENABLE_TRACING
  DTRACE( g_trace_ctx, D_HEADER, "----------- vui_parameters -----------\n");
#endif
  InputBitstream *bs = getBitstream();
  auto substream = bs->extractSubstream( vuiPayloadSize * 8 );
  setBitstream( substream.get() );

  X_READ_FLAG( vui_general_progressive_source_flag );
  pcVUI->setProgressiveSourceFlag( vui_general_progressive_source_flag );

  X_READ_FLAG( vui_general_interlaced_source_flag );
  pcVUI->setInterlacedSourceFlag( vui_general_interlaced_source_flag );

  X_READ_FLAG( vui_non_packed_constraint_flag );
  pcVUI->setNonPackedFlag( vui_non_packed_constraint_flag );

  X_READ_FLAG( vui_non_projected_constraint_flag );
  pcVUI->setNonProjectedFlag( vui_non_projected_constraint_flag );

  X_READ_FLAG( vui_aspect_ratio_info_present_flag );
  pcVUI->setAspectRatioInfoPresentFlag( vui_aspect_ratio_info_present_flag );

  if( vui_aspect_ratio_info_present_flag )
  {
    X_READ_FLAG( vui_aspect_ratio_constant_flag );
    pcVUI->setAspectRatioConstantFlag( vui_aspect_ratio_constant_flag );

    X_READ_CODE_NO_RANGE( vui_aspect_ratio_idc, 8 );
    pcVUI->setAspectRatioIdc( vui_aspect_ratio_idc );

    if( pcVUI->getAspectRatioIdc() == 255 )
    {
      X_READ_CODE_NO_RANGE( vui_sar_width, 16 );
      pcVUI->setSarWidth( vui_sar_width );

      X_READ_CODE_NO_RANGE( vui_sar_height, 16 );
      pcVUI->setSarHeight( vui_sar_height );
    }
    else if ((size_t)pcVUI->getAspectRatioIdc() <= sizeof(SARFixedRatios) / sizeof(SARFixedRatios[0]))
    {
      pcVUI->setSarWidth( SARFixedRatios[pcVUI->getAspectRatioIdc() - 1][0] );
      pcVUI->setSarHeight( SARFixedRatios[pcVUI->getAspectRatioIdc() - 1][1] );
    }
  }

  X_READ_FLAG( vui_overscan_info_present_flag );
  pcVUI->setOverscanInfoPresentFlag( vui_overscan_info_present_flag );

  if( vui_overscan_info_present_flag )
  {
    X_READ_FLAG( vui_overscan_appropriate_flag );
    pcVUI->setOverscanAppropriateFlag( vui_overscan_appropriate_flag );
  }

  X_READ_FLAG( vui_colour_description_present_flag );
  pcVUI->setColourDescriptionPresentFlag( vui_colour_description_present_flag );

  if( vui_colour_description_present_flag )
  {
    X_READ_CODE_NO_RANGE( vui_colour_primaries, 8 );
    pcVUI->setColourPrimaries( vui_colour_primaries );

    X_READ_CODE_NO_RANGE( vui_transfer_characteristics, 8 );
    pcVUI->setTransferCharacteristics( vui_transfer_characteristics );

    X_READ_CODE_NO_RANGE( vui_matrix_coeffs, 8 );
    pcVUI->setMatrixCoefficients( vui_matrix_coeffs );

    X_READ_FLAG( vui_video_full_range_flag );
    pcVUI->setVideoFullRangeFlag( vui_video_full_range_flag );
  }

  X_READ_FLAG( vui_chroma_loc_info_present_flag );
  pcVUI->setChromaLocInfoPresentFlag( vui_chroma_loc_info_present_flag );

  if( vui_chroma_loc_info_present_flag )
  {
    if( vui_general_progressive_source_flag && !vui_general_interlaced_source_flag )
    {
      X_READ_UVLC( vui_chroma_sample_loc_type_frame, 0, 6 );
      pcVUI->setChromaSampleLocType( vui_chroma_sample_loc_type_frame );
    }
    else
    {
      X_READ_UVLC( vui_chroma_sample_loc_type_top_field, 0, 6 );
      pcVUI->setChromaSampleLocTypeTopField( vui_chroma_sample_loc_type_top_field );

      X_READ_UVLC( vui_chroma_sample_loc_type_bottom_field, 0, 6 );
      pcVUI->setChromaSampleLocTypeBottomField( vui_chroma_sample_loc_type_bottom_field );
    }
  }

  int payloadBitsRem = getBitstream()->getNumBitsLeft();
  if( payloadBitsRem )      //Corresponds to more_data_in_payload()
  {
    while( payloadBitsRem > 9 )    //payload_extension_present()
    {
      X_READ_FLAG( vui_reserved_payload_extension_data );
      (void) vui_reserved_payload_extension_data;
      payloadBitsRem--;
    }
    int finalBits = getBitstream()->peekBits( payloadBitsRem );
    int numFinalZeroBits = 0;
    int mask = 0xff;
    while( finalBits & (mask >> numFinalZeroBits) )
    {
      numFinalZeroBits++;
    }
    while( payloadBitsRem > 9-numFinalZeroBits )     //payload_extension_present()
    {
      X_READ_FLAG( vui_reserved_payload_extension_data );
      (void) vui_reserved_payload_extension_data;
      payloadBitsRem--;
    }
    X_READ_FLAG_CHECK( vui_payload_bit_equal_to_one, vui_payload_bit_equal_to_one != 1, "vui_payload_bit_equal_to_one not equal to 1" );
    (void)vui_payload_bit_equal_to_one;
    payloadBitsRem--;
    while( payloadBitsRem )
    {
      X_READ_FLAG_CHECK( vui_payload_bit_equal_to_zero, vui_payload_bit_equal_to_zero != 0, "vui_payload_bit_equal_to_zero not equal to 0" );
      (void)vui_payload_bit_equal_to_zero;
      payloadBitsRem--;
    }
  }
  setBitstream( bs );
}

void HLSyntaxReader::parseGeneralHrdParameters( GeneralHrdParams *hrd )
{
  X_READ_CODE_NO_RANGE( num_units_in_tick, 32 );
  CHECK( num_units_in_tick <= 0, "num_units_in_tick shall be greater than 0" );
  hrd->setNumUnitsInTick( num_units_in_tick );

  X_READ_CODE_NO_RANGE( time_scale, 32 );
  CHECK( time_scale <= 0, "The value of time_scale shall be greater than 0." );
  hrd->setTimeScale( time_scale );

  X_READ_FLAG( general_nal_hrd_params_present_flag );
  hrd->setGeneralNalHrdParamsPresentFlag( general_nal_hrd_params_present_flag );

  X_READ_FLAG( general_vcl_hrd_params_present_flag );
  hrd->setGeneralVclHrdParamsPresentFlag( general_vcl_hrd_params_present_flag );

  if( general_nal_hrd_params_present_flag || general_vcl_hrd_params_present_flag )
  {
    X_READ_FLAG( general_same_pic_timing_in_all_ols_flag );
    hrd->setGeneralSamePicTimingInAllOlsFlag( general_same_pic_timing_in_all_ols_flag );

    X_READ_FLAG( general_du_hrd_params_present_flag );
    hrd->setGeneralDuHrdParamsPresentFlag( general_du_hrd_params_present_flag );

    if( general_du_hrd_params_present_flag )
    {
      X_READ_CODE_NO_RANGE( tick_divisor_minus2, 8 );
      hrd->setTickDivisorMinus2( tick_divisor_minus2 );
    }

    X_READ_CODE_NO_RANGE( bit_rate_scale, 4 );
    hrd->setBitRateScale( bit_rate_scale );

    X_READ_CODE_NO_RANGE( cpb_size_scale, 4 );
    hrd->setCpbSizeScale( cpb_size_scale );

    if( general_du_hrd_params_present_flag )
    {
      X_READ_CODE_NO_RANGE( cpb_size_du_scale, 4 );
      hrd->setCpbSizeDuScale( cpb_size_du_scale );
    }

    X_READ_UVLC( hrd_cpb_cnt_minus1, 0, MAX_CPB_CNT - 1 );
    hrd->setHrdCpbCntMinus1( hrd_cpb_cnt_minus1 );
  }
}

void HLSyntaxReader::parseOlsHrdParameters( GeneralHrdParams * generalHrd, std::vector<OlsHrdParams>& olsHrd, uint32_t firstSubLayer, uint32_t maxNumSubLayersMinus1 )
{
  olsHrd.resize( maxNumSubLayersMinus1 + 1 );

  for( unsigned i = firstSubLayer; i <= maxNumSubLayersMinus1; i++ )
  {
    OlsHrdParams& hrd = olsHrd[i];

    X_READ_FLAG( fixed_pic_rate_general_flag );
    hrd.setFixedPicRateGeneralFlag( fixed_pic_rate_general_flag );

    // When fixed_pic_rate_general_flag[ i ] is equal to 1, the value of fixed_pic_rate_within_cvs_flag[ i ] is inferred to be equal to 1.
    hrd.setFixedPicRateWithinCvsFlag( fixed_pic_rate_general_flag );
    if( !fixed_pic_rate_general_flag )
    {
      X_READ_FLAG( fixed_pic_rate_within_cvs_flag );
      hrd.setFixedPicRateWithinCvsFlag( fixed_pic_rate_within_cvs_flag );
    }

    // TODO: It is a requirement of bitstream conformance that when general_nal_hrd_params_present_flag and
    //       general_vcl_hrd_params_present_flag are both equal to 0, there shall be at least one value of
    //       fixed_pic_rate_within_cvs_flag[ i ] equal to 1 for i in the range of 0 to MaxSubLayersVal − 1, inclusive.

    hrd.setLowDelayHrdFlag( false );   // Inferred to be 0 when not present

    if( hrd.getFixedPicRateWithinCvsFlag() )
    {
      X_READ_UVLC( elemental_duration_in_tc_minus1, 0, 2047 );
      hrd.setElementDurationInTcMinus1( elemental_duration_in_tc_minus1 );
    }
    else if( ( generalHrd->getGeneralNalHrdParamsPresentFlag() || generalHrd->getGeneralVclHrdParamsPresentFlag() )
             && generalHrd->getHrdCpbCntMinus1() == 0 )
    {
      X_READ_FLAG( low_delay_hrd_flag );
      hrd.setLowDelayHrdFlag( low_delay_hrd_flag );
    }

    using NalOrVcl = enum { NAL = 0, VCL = 1 };
    for( NalOrVcl nalOrVcl: { NAL, VCL } )
    {
      if( ( nalOrVcl == NAL && generalHrd->getGeneralNalHrdParamsPresentFlag() ) ||   //
          ( nalOrVcl == VCL && generalHrd->getGeneralVclHrdParamsPresentFlag() ) )
      {
        for( int j = 0; j <= generalHrd->getHrdCpbCntMinus1(); j++ )
        {
          CHECK( generalHrd->getHrdCpbCntMinus1() >= MAX_CPB_CNT, "hrd_cpb_cnt_minus1 out of bounds" );

          X_READ_UVLC_NO_RANGE( bit_rate_value_minus1 );
          CHECK( j > 0 && bit_rate_value_minus1 <= hrd.getBitRateValueMinus1( j - 1, nalOrVcl ),
                 "For any j greater than 0 and any particular value of i, bit_rate_value_minus1[ i ][ j ]"
                 " shall be greater than bit_rate_value_minus1[ i ][ j − 1 ]." );
          hrd.setBitRateValueMinus1( j, nalOrVcl, bit_rate_value_minus1 );

          X_READ_UVLC( cpb_size_value_minus1, 0, ( 1ull << 32 ) - 2 );
          CHECK( j > 0 && cpb_size_value_minus1 > hrd.getCpbSizeValueMinus1( j - 1, nalOrVcl ),
                 "For any j greater than 0 and any particular value of i, cpb_size_value_minus1[ i ][ j ]"
                 " shall be less than or equal to cpb_size_value_minus1[ i ][ j − 1 ]." );
          hrd.setCpbSizeValueMinus1( j, nalOrVcl, cpb_size_value_minus1 );

          if( generalHrd->getGeneralDuHrdParamsPresentFlag() )
          {
            X_READ_UVLC( cpb_size_du_value_minus1, 0, ( 1ull << 32 ) - 2 );
            CHECK( j > 0 && cpb_size_du_value_minus1 > hrd.getDuCpbSizeValueMinus1( j - 1, nalOrVcl ),
                   "For any j greater than 0 and any particular value of i, cpb_size_du_value_minus1[ i ][ j ]"
                   " shall be less than or equal to cpb_size_du_value_minus1[ i ][ j − 1 ]." )
            hrd.setDuCpbSizeValueMinus1( j, nalOrVcl, cpb_size_du_value_minus1 );

            X_READ_UVLC( bit_rate_du_value_minus1, 0, ( 1ull << 32 ) - 2 );
            CHECK( j > 0 && bit_rate_du_value_minus1 <= hrd.getDuBitRateValueMinus1( j - 1, nalOrVcl ),
                   "For any j greater than 0 and any particular value of i, bit_rate_du_value_minus1[ i ][ j ]"
                   " shall be greater than bit_rate_du_value_minus1[ i ][ j − 1 ]" )
            hrd.setDuBitRateValueMinus1( j, nalOrVcl, bit_rate_du_value_minus1 );
          }
          X_READ_FLAG( cbr_flag );
          hrd.setCbrFlag( j, nalOrVcl, cbr_flag );
        }
      }
    }
  }

  for( int i = 0; i < firstSubLayer; i++ )
  {
    OlsHrdParams* hrdHighestTLayer = &( olsHrd[maxNumSubLayersMinus1] );
    OlsHrdParams* hrdTemp = &( olsHrd[i] );
    bool tempFlag = hrdHighestTLayer->getFixedPicRateGeneralFlag();
    hrdTemp->setFixedPicRateGeneralFlag( tempFlag );
    tempFlag = hrdHighestTLayer->getFixedPicRateWithinCvsFlag();
    hrdTemp->setFixedPicRateWithinCvsFlag( tempFlag );
    uint32_t tempElementDurationInTcMinus1 = hrdHighestTLayer->getElementDurationInTcMinus1();
    hrdTemp->setElementDurationInTcMinus1( tempElementDurationInTcMinus1 );
    for( int nalOrVcl = 0; nalOrVcl < 2; nalOrVcl++ )
    {
      if( ( nalOrVcl == 0 && generalHrd->getGeneralNalHrdParamsPresentFlag() ) || ( nalOrVcl == 1 && generalHrd->getGeneralVclHrdParamsPresentFlag() ) )
      {
        for( int j = 0; j <= (generalHrd->getHrdCpbCntMinus1()); j++ )
        {
          uint32_t bitRate = hrdHighestTLayer->getBitRateValueMinus1( j, nalOrVcl );
          hrdTemp->setBitRateValueMinus1( j, nalOrVcl, bitRate );
          uint32_t cpbSize = hrdHighestTLayer->getCpbSizeValueMinus1( j, nalOrVcl );
          hrdTemp->setCpbSizeValueMinus1( j, nalOrVcl, cpbSize );
          if( generalHrd->getGeneralDuHrdParamsPresentFlag() )
          {
            uint32_t bitRateDu = hrdHighestTLayer->getDuBitRateValueMinus1( j, nalOrVcl );
            hrdTemp->setDuBitRateValueMinus1(j, nalOrVcl, bitRateDu);
            uint32_t cpbSizeDu = hrdHighestTLayer->getDuCpbSizeValueMinus1( j, nalOrVcl );
            hrdTemp->setDuCpbSizeValueMinus1( j, nalOrVcl, cpbSizeDu );
          }
          bool flag = hrdHighestTLayer->getCbrFlag( j, nalOrVcl );
          hrdTemp->setCbrFlag( j, nalOrVcl, flag );
        }
      }
    }
  }
}

void HLSyntaxReader::dpb_parameters( int maxSubLayersMinus1, bool subLayerInfoFlag, SPS *pcSPS )
{
  CHECK( maxSubLayersMinus1 >= MAX_TLAYER, "maxSubLayersMinus1 out of bounds" );
  for( int i = ( subLayerInfoFlag ? 0 : maxSubLayersMinus1 ); i <= maxSubLayersMinus1; i++ )
  {
    X_READ_UVLC_NO_RANGE_idx( dpb_max_dec_pic_buffering_minus1, "[i]" );
    CHECK( i > 0 && dpb_max_dec_pic_buffering_minus1 < pcSPS->getMaxDecPicBuffering( i - 1 ) - 1,
           "When i is greater than 0, dpb_max_dec_pic_buffering_minus1[ i ] shall be greater than or equal to dpb_max_dec_pic_buffering_minus1[ i − 1 ]." );
    pcSPS->setMaxDecPicBuffering( dpb_max_dec_pic_buffering_minus1 + 1, i );

    X_READ_UVLC_idx( dpb_max_num_reorder_pics, "[i]", 0, dpb_max_dec_pic_buffering_minus1 );
    CHECK( i > 0 && dpb_max_dec_pic_buffering_minus1 < pcSPS->getNumReorderPics( i - 1 ),
           "When i is greater than 0, dpb_max_num_reorder_pics[ i ] shall be greater than or equal to dpb_max_num_reorder_pics[ i − 1 ]." );
    pcSPS->setNumReorderPics( dpb_max_num_reorder_pics, i );

    X_READ_UVLC_idx( dpb_max_latency_increase_plus1, "[i]", 0, ( uint64_t( 1 ) << 32 ) - 2 );
    pcSPS->setMaxLatencyIncreasePlus1( dpb_max_latency_increase_plus1, i );
  }
}

void HLSyntaxReader::parseExtraPHBitsStruct( SPS *sps, int numBytes )
{
  std::vector<bool> presentFlags;
  presentFlags.resize ( 8 * numBytes );

  for( int i = 0; i < 8 * numBytes; i++ )
  {
    X_READ_FLAG_idx( sps_extra_ph_bit_present_flag, "[i]" );
    presentFlags[i] = sps_extra_ph_bit_present_flag;
  }

  sps->setExtraPHBitPresentFlags( std::move( presentFlags ) );
}

void HLSyntaxReader::parseExtraSHBitsStruct( SPS *sps, int numBytes )
{
  std::vector<bool> presentFlags;
  presentFlags.resize ( 8 * numBytes );

  for( int i = 0; i < 8 * numBytes; i++ )
  {
    X_READ_FLAG_idx( sps_extra_sh_bit_present_flag, "[i]" );
    presentFlags[i] = sps_extra_sh_bit_present_flag;
  }

  sps->setExtraSHBitPresentFlags( std::move( presentFlags ) );
}

void HLSyntaxReader::parseSPS( SPS* sps, const ParameterSetManager* parameterSetManager )
{
#if ENABLE_TRACING
  xTraceSPSHeader ();
#endif

  X_READ_CODE_NO_RANGE( sps_seq_parameter_set_id, 4 );
  sps->setSPSId( sps_seq_parameter_set_id );

  X_READ_CODE_NO_RANGE( sps_video_parameter_set_id, 4 );
  (void) sps_video_parameter_set_id;   // sps->setVPSId( sps_video_parameter_set_id ); // TODO: change to support VPS
  const VPS* vps = nullptr;
#if 0
  if( sps_video_parameter_set_id > 0 && parameterSetManager->getVPS( sps_video_parameter_set_id ) )
  {
    vps = parameterSetManager->getVPS( sps_video_parameter_set_id );
  }
#endif

  X_READ_CODE( sps_max_sublayers_minus1, 3, 0, 6 );
  CHECK( vps && sps_video_parameter_set_id > vps->getMaxSubLayers(),
         "If sps_video_parameter_set_id is greater than 0, the value of sps_max_sublayers_minus1 shall be in the range of 0 to vps_max_sublayers_minus1, "
         "inclusive." )
  sps->setMaxTLayers( sps_max_sublayers_minus1 + 1 );

  X_READ_CODE_NO_RANGE( sps_chroma_format_idc, 2 );
  // it is a requirement of bitstream conformance that the value of sps_chroma_format_idc shall
  // be less than or equal to the value of vps_ols_dpb_chroma_format[ i ].
  sps->setChromaFormatIdc( ChromaFormat( sps_chroma_format_idc ) );

  X_READ_CODE( sps_log2_ctu_size_minus5, 2, 0, 2 );
  sps->setCTUSize( 1 << ( sps_log2_ctu_size_minus5 + 5 ) );

  const int CtbLog2SizeY = sps_log2_ctu_size_minus5 + 5;
  const int CtbSizeY     = 1 << CtbLog2SizeY;
  sps->setMaxCUWidth( sps->getCTUSize() );
  sps->setMaxCUHeight( sps->getCTUSize() );

  X_READ_FLAG( sps_ptl_dpb_hrd_params_present_flag );
  CHECK( sps_video_parameter_set_id == 0 && !sps_ptl_dpb_hrd_params_present_flag,
                     "When sps_video_parameter_set_id is equal to 0, the value of sps_ptl_dpb_hrd_params_present_flag shall be equal to 1" );
  sps->setPtlDpbHrdParamsPresentFlag( sps_ptl_dpb_hrd_params_present_flag );

  if( sps_ptl_dpb_hrd_params_present_flag )
  {
    parseProfileTierLevel( sps->getProfileTierLevel(), true, sps->getMaxTLayers() - 1 );
  }

  const ProfileTierLevel* ptl = sps->getProfileTierLevel();
  const ConstraintInfo*   gci = ptl->getConstraintInfo();

  X_READ_FLAG( sps_gdr_enabled_flag );
  CHECK_CONSTRAINT( gci->getNoGdrConstraintFlag() && sps_gdr_enabled_flag,
                     "When gci_no_gdr_constraint_flag equal to 1 , the value of sps_gdr_enabled_flag shall be equal to 0" );
  sps->setGDREnabledFlag( sps_gdr_enabled_flag );

  X_READ_FLAG( sps_ref_pic_resampling_enabled_flag );
  CHECK_CONSTRAINT( gci->getNoRprConstraintFlag() && sps_ref_pic_resampling_enabled_flag,
                    "When gci_no_ref_pic_resampling_constraint_flag is equal to 1, sps_ref_pic_resampling_enabled_flag shall be equal to 0" );
  sps->setRprEnabledFlag( sps_ref_pic_resampling_enabled_flag );

  if( sps_ref_pic_resampling_enabled_flag )
  {
    X_READ_FLAG( sps_res_change_in_clvs_allowed_flag );
    CHECK_CONSTRAINT( gci->getNoResChangeInClvsConstraintFlag() && sps_res_change_in_clvs_allowed_flag,
                      "When no_res_change_in_clvs_constraint_flag is equal to 1, res_change_in_clvs_allowed_flag shall be equal to 0" );
    sps->setResChangeInClvsEnabledFlag( sps_res_change_in_clvs_allowed_flag );
  }

  X_READ_UVLC_NO_RANGE( sps_pic_width_max_in_luma_samples );
  sps->setMaxPicWidthInLumaSamples( sps_pic_width_max_in_luma_samples );

  X_READ_UVLC_NO_RANGE( sps_pic_height_max_in_luma_samples );
  sps->setMaxPicHeightInLumaSamples( sps_pic_height_max_in_luma_samples );

  const int SubWidthC  = 1 << getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, sps->getChromaFormatIdc() );
  const int SubHeightC = 1 << getChannelTypeScaleY( CHANNEL_TYPE_CHROMA, sps->getChromaFormatIdc() );

  X_READ_FLAG( sps_conformance_window_flag );
  sps->setConformanceWindowPresentFlag( sps_conformance_window_flag );

  if( sps_conformance_window_flag )
  {
    X_READ_UVLC_NO_RANGE( sps_conf_win_left_offset );
    X_READ_UVLC_NO_RANGE( sps_conf_win_right_offset );
    X_READ_UVLC_NO_RANGE( sps_conf_win_top_offset );
    X_READ_UVLC_NO_RANGE( sps_conf_win_bottom_offset );

    CHECK( SubWidthC * ( sps_conf_win_left_offset + sps_conf_win_right_offset ) > sps_pic_width_max_in_luma_samples,
           "The value of SubWidthC * ( sps_conf_win_left_offset + sps_conf_win_right_offset ) shall be less than sps_pic_width_max_in_luma_samples." );
    CHECK( SubHeightC * ( sps_conf_win_top_offset + sps_conf_win_bottom_offset ) > sps_pic_height_max_in_luma_samples,
           "The value of SubHeightC * ( sps_conf_win_top_offset + sps_conf_win_bottom_offset ) shall be less than sps_pic_height_max_in_luma_samples." );

    Window& conf = sps->getConformanceWindow();
    conf.setWindowLeftOffset( sps_conf_win_left_offset );
    conf.setWindowRightOffset( sps_conf_win_right_offset );
    conf.setWindowTopOffset( sps_conf_win_top_offset );
    conf.setWindowBottomOffset( sps_conf_win_bottom_offset );
  }
  X_READ_FLAG( sps_subpic_info_present_flag );
  CHECK( sps->getResChangeInClvsEnabledFlag() && sps_subpic_info_present_flag,
         "When sps_res_change_in_clvs_allowed_flag is equal to 1, the value of sps_subpic_info_present_flag shall be equal to 0." )
  CHECK_CONSTRAINT( gci->getNoSubpicInfoConstraintFlag() && sps_subpic_info_present_flag,
                    "When gci_no_subpic_info_constraint_flag is equal to 1, the value of subpic_info_present_flag shall be equal to 0" );
  sps->setSubPicInfoPresentFlag( sps_subpic_info_present_flag );


  if( sps_subpic_info_present_flag )
  {
    X_READ_UVLC_NO_RANGE( sps_num_subpics_minus1 );
    CHECK( sps_num_subpics_minus1 + 1 > ( ( sps_pic_width_max_in_luma_samples + CtbSizeY - 1 ) / CtbSizeY )
                                                      * ( ( sps_pic_height_max_in_luma_samples + CtbSizeY - 1 ) / CtbSizeY ),
                       "Invalid sps_num_subpics_minus1 value" );
    sps->setNumSubPics( sps_num_subpics_minus1 + 1 );

    if( sps_num_subpics_minus1 == 0 )
    {
      sps->setSubPicCtuTopLeftX( 0, 0 );
      sps->setSubPicCtuTopLeftY( 0, 0 );
      sps->setSubPicWidth( 0, ( sps->getMaxPicWidthInLumaSamples() + sps->getCTUSize() - 1 ) >> getLog2( sps->getCTUSize() ) );
      sps->setSubPicHeight( 0, ( sps->getMaxPicHeightInLumaSamples() + sps->getCTUSize() - 1 ) >> getLog2( sps->getCTUSize() ) );
      sps->setIndependentSubPicsFlag( 1 );
      sps->setSubPicSameSizeFlag( 0 );
      sps->setSubPicTreatedAsPicFlag( 0, 1 );
      sps->setLoopFilterAcrossSubpicEnabledFlag( 0, 0 );
    }
    else   // ( sps_num_subpics_minus1 > 0 )
    {
      X_READ_FLAG( sps_independent_subpics_flag );
      sps->setIndependentSubPicsFlag( sps_independent_subpics_flag );

      X_READ_FLAG( sps_subpic_same_size_flag );
      sps->setSubPicSameSizeFlag( sps_subpic_same_size_flag );

      const uint32_t tmpWidthVal = ( sps->getMaxPicWidthInLumaSamples() + sps->getCTUSize() - 1 ) / sps->getCTUSize();
      const uint32_t tmpHeightVal = ( sps->getMaxPicHeightInLumaSamples() + sps->getCTUSize() - 1 ) / sps->getCTUSize();

      const int ceilLog2tmpWidth  = (int) ceil( log2( tmpWidthVal ) );
      const int ceilLog2tmpHeight = (int) ceil( log2( tmpHeightVal ) );
      for( unsigned picIdx = 0; picIdx < sps->getNumSubPics(); picIdx++ )
      {
        if( !sps_subpic_same_size_flag || picIdx == 0 )
        {
          if( picIdx > 0 && sps_pic_width_max_in_luma_samples > CtbSizeY )
          {
            X_READ_CODE_NO_RANGE_idx( sps_subpic_ctu_top_left_x, "[ i ]", ceilLog2tmpWidth );
            sps->setSubPicCtuTopLeftX( picIdx, sps_subpic_ctu_top_left_x );
          }
          else
          {
            sps->setSubPicCtuTopLeftX( picIdx, 0 );
          }

          if( picIdx > 0 && sps_pic_height_max_in_luma_samples > CtbSizeY )
          {
            X_READ_CODE_NO_RANGE_idx( sps_subpic_ctu_top_left_y, "[ i ]", ceilLog2tmpHeight );
            sps->setSubPicCtuTopLeftY( picIdx, sps_subpic_ctu_top_left_y );
          }
          else
          {
            sps->setSubPicCtuTopLeftY( picIdx, 0 );
          }

          if( picIdx < sps_num_subpics_minus1 && sps_pic_width_max_in_luma_samples > CtbSizeY )
          {
            X_READ_CODE_NO_RANGE_idx( sps_subpic_width_minus1, "[ i ]", ceilLog2tmpWidth );
            sps->setSubPicWidth( picIdx, sps_subpic_width_minus1 + 1 );
          }
          else
          {
            // If sps_subpic_same_size_flag is equal to 0 or i is equal to 0, the value of sps_subpic_width_minus1[ i ] is inferred
            //   to be equal to tmpWidthVal - sps_subpic_ctu_top_left_x[ i ] - 1.
            sps->setSubPicWidth( picIdx, tmpWidthVal - sps->getSubPicCtuTopLeftX( picIdx ) );
          }

          if( picIdx < sps_num_subpics_minus1 && sps_pic_height_max_in_luma_samples > CtbSizeY )
          {
            X_READ_CODE_NO_RANGE_idx( sps_subpic_height_minus1, "[ i ]", ceilLog2tmpHeight );
            sps->setSubPicHeight( picIdx, sps_subpic_height_minus1 + 1 );
          }
          else
          {
            // If sps_subpic_same_size_flag is equal to 0 or i is equal to 0, the value of sps_subpic_height_minus1[ i ] is inferred
            //   to be equal to tmpHeightVal - sps_subpic_ctu_top_left_y[ i ] − 1.
            sps->setSubPicHeight( picIdx, tmpHeightVal - sps->getSubPicCtuTopLeftY( picIdx ) );
          }
        }
        else   // ( sps_subpic_same_size_flag && picIdx != 0 )
        {
          const int numSubpicCols = tmpWidthVal / sps->getSubPicWidth( 0 );

          // const int numSubpicCols = tmpWidthVal / ( sps->getSubPicWidth( 0 ) + 1 );
          CHECK( sps_subpic_same_size_flag && numSubpicCols * tmpHeightVal / sps->getSubPicHeight( 0 ) - 1 != sps_num_subpics_minus1,
                 "When sps_subpic_same_size_flag is equal to 1, the value of numSubpicCols * tmpHeightVal / ( sps_subpic_height_minus1[ 0 ] + 1 ) − 1"
                 " shall be equal to sps_num_subpics_minus1." )
          CHECK( sps_subpic_same_size_flag && tmpWidthVal % sps->getSubPicWidth( 0 ) != 0,
                 "When sps_subpic_same_size_flag is equal to 1, the value of tmpWidthVal % ( sps_subpic_width_minus1[ 0 ] + 1 ) shall"
                 " be equal to 0." );
          CHECK( sps_subpic_same_size_flag && tmpHeightVal % sps->getSubPicHeight( 0 ) != 0,
                 "When sps_subpic_same_size_flag is equal to 1, the value of tmpHeightVal % ( sps_subpic_height_minus1[ 0 ] + 1 ) shall"
                 " be equal to 0." )

          // Otherwise, the value of sps_subpic_ctu_top_left_x[ i ] is inferred to be equal to ( i % numSubpicCols ) * ( sps_subpic_width_minus1[ 0 ] + 1 ).
          // Otherwise, the value of sps_subpic_ctu_top_left_y[ i ] is inferred to be equal to ( i / numSubpicCols ) * ( sps_subpic_height_minus1[ 0 ] + 1 ).
          sps->setSubPicCtuTopLeftX( picIdx, ( picIdx % numSubpicCols ) * sps->getSubPicWidth( 0 ) );
          sps->setSubPicCtuTopLeftY( picIdx, ( picIdx / numSubpicCols ) * sps->getSubPicHeight( 0 ) );
          // Otherwise, the value of sps_subpic_width_minus1[ i ] is inferred to be equal to sps_subpic_width_minus1[ 0 ].
          // Otherwise, the value of sps_subpic_height_minus1[ i ] is inferred to be equal to sps_subpic_height_minus1[ 0 ].
          sps->setSubPicWidth( picIdx, sps->getSubPicWidth( 0 ) );
          sps->setSubPicHeight( picIdx, sps->getSubPicHeight( 0 ) );
        }

        // TODO: It is a requirement of bitstream conformance that the shapes of the subpictures shall be such that each subpicture, when
        //       decoded, shall have its entire left boundary and entire top boundary consisting of picture boundaries or consisting of
        //       boundaries of previously decoded subpictures.
        //       For each subpicture with subpicture index i in the range of 0 to sps_num_subpics_minus1, inclusive, it is a requirement
        //       of bitstream conformance that all of the following conditions are true:
        //       - The value of ( sps_subpic_ctu_top_left_x[ i ] * CtbSizeY ) shall be less than ( sps_pic_width_max_in_luma_samples - sps_conf_win_right_offset * SubWidthC ).
        //       - The value of ( ( sps_subpic_ctu_top_left_x[ i ] + sps_subpic_width_minus1[ i ] + 1 ) * CtbSizeY ) shall be greater than ( sps_conf_win_left_offset * SubWidthC ).
        //       - The value of ( sps_subpic_ctu_top_left_y[ i ] * CtbSizeY ) shall  be less than ( sps_pic_height_max_in_luma_samples - sps_conf_win_bottom_offset * SubHeightC ).
        //       - The value of ( ( sps_subpic_ctu_top_left_y[ i ] + sps_subpic_height_minus1[ i ] + 1 ) * CtbSizeY ) shall be greater than ( sps_conf_win_top_offset * SubHeightC ).

        Window& conf = sps->getConformanceWindow();
        CHECK( sps->getSubPicCtuTopLeftX( picIdx ) * CtbSizeY >= sps->getMaxPicWidthInLumaSamples() - conf.getWindowRightOffset() * SubWidthC,
               "The value of ( sps_subpic_ctu_top_left_x[ i ] * CtbSizeY )"
               " shall be less than ( sps_pic_width_max_in_luma_samples - sps_conf_win_right_offset * SubWidthC )." );
        CHECK( ( sps->getSubPicCtuTopLeftX( picIdx ) + sps->getSubPicWidth( picIdx ) ) * CtbSizeY <= conf.getWindowLeftOffset() * SubWidthC,
               "The value of ( ( sps_subpic_ctu_top_left_x[ i ] + sps_subpic_width_minus1[ i ] + 1 ) * CtbSizeY )"
               " shall be greater than ( sps_conf_win_left_offset * SubWidthC )." );
        CHECK( sps->getSubPicCtuTopLeftY( picIdx ) * CtbSizeY >= sps->getMaxPicHeightInLumaSamples() - conf.getWindowBottomOffset() * SubHeightC,
               "The value of ( sps_subpic_ctu_top_left_y[ i ] * CtbSizeY )"
               " shall  be less than ( sps_pic_height_max_in_luma_samples - sps_conf_win_bottom_offset * SubHeightC )." );
        CHECK( ( sps->getSubPicCtuTopLeftX( picIdx ) + sps->getSubPicHeight( picIdx ) ) * CtbSizeY <= conf.getWindowTopOffset() * SubHeightC,
               "The value of ( ( sps_subpic_ctu_top_left_y[ i ] + sps_subpic_height_minus1[ i ] + 1 ) * CtbSizeY )"
               " shall be greater than ( sps_conf_win_top_offset * SubHeightC )." );

        if( !sps_independent_subpics_flag )
        {
          X_READ_FLAG_idx( sps_subpic_treated_as_pic_flag, "[ i ]" );
          sps->setSubPicTreatedAsPicFlag( picIdx, sps_subpic_treated_as_pic_flag );

          X_READ_FLAG_idx( sps_loop_filter_across_subpic_enabled_flag, "[ i ]" );
          sps->setLoopFilterAcrossSubpicEnabledFlag( picIdx, sps_loop_filter_across_subpic_enabled_flag );
        }
        else
        {
          // should be set as default
            sps->setSubPicTreatedAsPicFlag( picIdx, 1 );
        }
      }   //      for( unsigned picIdx = 0; picIdx < sps->getNumSubPics(); picIdx++ )
    }


    X_READ_UVLC( sps_subpic_id_len_minus1, 0, 15 );
    CHECK( 1 << ( sps_subpic_id_len_minus1 + 1 ) < sps_num_subpics_minus1 + 1,
           "The value of 1 << ( sps_subpic_id_len_minus1 + 1 ) shall be greater than or equal to sps_num_subpics_minus1 + 1" );
    sps->setSubPicIdLen( sps_subpic_id_len_minus1 + 1 );

    X_READ_FLAG( sps_subpic_id_mapping_explicitly_signalled_flag );
    sps->setSubPicIdMappingExplicitlySignalledFlag( sps_subpic_id_mapping_explicitly_signalled_flag );

    if( sps_subpic_id_mapping_explicitly_signalled_flag )
    {
      X_READ_FLAG( sps_subpic_id_mapping_present_flag );
      sps->setSubPicIdMappingPresentFlag( sps_subpic_id_mapping_present_flag );

      if( sps_subpic_id_mapping_present_flag )
      {
        for( int picIdx = 0; picIdx <= sps_num_subpics_minus1; picIdx++ )
        {
          X_READ_CODE_NO_RANGE_idx( sps_subpic_id, "[ i ]", sps->getSubPicIdLen() );
          sps->setSubPicId( picIdx, sps_subpic_id );
        }
      }
    }
  }   // sps_subpic_info_present_flag
  else  // ( !sps_subpic_info_present_flag )
  {
    sps->setSubPicIdMappingExplicitlySignalledFlag( 0 );
    sps->setNumSubPics( 1 );
    sps->setSubPicCtuTopLeftX( 0, 0 );
    sps->setSubPicCtuTopLeftY( 0, 0 );
    sps->setSubPicWidth( 0, ( sps->getMaxPicWidthInLumaSamples() + sps->getCTUSize() - 1 ) >> getLog2( sps->getCTUSize() ) );
    sps->setSubPicHeight( 0, ( sps->getMaxPicHeightInLumaSamples() + sps->getCTUSize() - 1 ) >> getLog2( sps->getCTUSize() ) );
    sps->setSubPicTreatedAsPicFlag( 0, true );
    sps->setLoopFilterAcrossSubpicEnabledFlag( 0, false );
  }

  if( !sps->getSubPicIdMappingExplicitlySignalledFlag() || !sps->getSubPicIdMappingPresentFlag() )
  {
    for( int picIdx = 0; picIdx < sps->getNumSubPics(); picIdx++ )
    {
      sps->setSubPicId( picIdx, picIdx );
    }
  }

  X_READ_UVLC( sps_bitdepth_minus8, 0, 8 );
  const Profile::Name profile = sps->getProfileTierLevel()->getProfileIdc();
  if( profile != Profile::NONE )
  {
    CHECK( sps_bitdepth_minus8 + 8 > ProfileFeatures::getProfileFeatures( profile )->maxBitDepth,
                       "sps_bitdepth_minus8 exceeds range supported by signalled profile" );
  }
  sps->setBitDepth( 8 + sps_bitdepth_minus8 );
  sps->setQpBDOffset( 6 * sps_bitdepth_minus8 );
  const int BitDepth = 8 + sps_bitdepth_minus8;
  const int QpBdOffset = 6 * sps_bitdepth_minus8;

  X_READ_FLAG( sps_entropy_coding_sync_enabled_flag );
  sps->setEntropyCodingSyncEnabledFlag( sps_entropy_coding_sync_enabled_flag );

  X_READ_FLAG( sps_entry_point_offsets_present_flag );
  sps->setEntryPointsPresentFlag( sps_entry_point_offsets_present_flag );

  X_READ_CODE( sps_log2_max_pic_order_cnt_lsb_minus4, 4, 0, 12 );
  sps->setBitsForPOC( sps_log2_max_pic_order_cnt_lsb_minus4 + 4 );

  X_READ_FLAG( sps_poc_msb_cycle_flag );
  sps->setPocMsbFlag( sps_poc_msb_cycle_flag );

  if( sps_poc_msb_cycle_flag )
  {
    X_READ_UVLC( sps_poc_msb_cycle_len_minus1, 0, 32 - sps_log2_max_pic_order_cnt_lsb_minus4 - 5 );
    sps->setPocMsbLen( sps_poc_msb_cycle_len_minus1 + 1 );
  }

  // extra bits are for future extensions, we will read, but ignore them,
  // unless a meaning is specified in the spec
  X_READ_CODE( sps_num_extra_ph_bytes, 2, 0, 2 );
  sps->setNumExtraPHBitsBytes( sps_num_extra_ph_bytes );
  if( sps_num_extra_ph_bytes )
  {
    parseExtraPHBitsStruct( sps, sps_num_extra_ph_bytes );
  }

  X_READ_CODE( sps_num_extra_sh_bytes, 2, 0, 2 );
  sps->setNumExtraSHBitsBytes( sps_num_extra_sh_bytes );
  if( sps_num_extra_sh_bytes )
  {
    parseExtraSHBitsStruct( sps, sps_num_extra_sh_bytes );
  }

  if( sps_ptl_dpb_hrd_params_present_flag )
  {
    if( sps_max_sublayers_minus1 > 0 )
    {
      X_READ_FLAG( sps_sublayer_dpb_params_flag );
      sps->setSubLayerDpbParamsFlag( sps_sublayer_dpb_params_flag );
    }
    dpb_parameters( sps_max_sublayers_minus1, sps->getSubLayerDpbParamsFlag(), sps );
  }

  X_READ_UVLC( sps_log2_min_luma_coding_block_size_minus2, 0, std::min( 4u, sps_log2_ctu_size_minus5 + 3 ) );
  sps->setLog2MinCodingBlockSize( sps_log2_min_luma_coding_block_size_minus2 + 2 );

  const int MinCbLog2SizeY = sps_log2_min_luma_coding_block_size_minus2 + 2;
  const int MinCbSizeY     = 1 << MinCbLog2SizeY;
  const int VSize          = std::min( 64, CtbSizeY );
  CHECK( MinCbSizeY > VSize, "The value of MinCbSizeY shall be less than or equal to VSize." )
  CHECK( MinCbLog2SizeY > CtbLog2SizeY, "Invalid log2_min_luma_coding_block_size_minus2 signalled" );

  // postponed checks for sps_pic_{width,height}_max_in_luma_samples,
  CHECK( sps_pic_width_max_in_luma_samples == 0 || sps_pic_width_max_in_luma_samples & ( std::max( 8, MinCbSizeY ) - 1 ),
         "sps_pic_width_max_in_luma_samples shall not be equal to 0 and shall be an integer multiple of Max( 8, MinCbSizeY )" );
  CHECK( sps_pic_height_max_in_luma_samples == 0 || sps_pic_height_max_in_luma_samples & ( std::max( 8, MinCbSizeY ) - 1 ),
         "sps_pic_height_max_in_luma_samples shall not be equal to 0 and shall be an integer multiple of Max( 8, MinCbSizeY )" );

  const int minCuSize = 1 << sps->getLog2MinCodingBlockSize();
  CHECK( ( sps->getMaxPicWidthInLumaSamples() % ( std::max( 8, minCuSize ) ) ) != 0, "Coded frame width must be a multiple of Max(8, the minimum unit size)" );
  CHECK( ( sps->getMaxPicHeightInLumaSamples() % ( std::max( 8, minCuSize ) ) ) != 0, "Coded frame height must be a multiple of Max(8, the minimum unit size)" );

  X_READ_FLAG( sps_partition_constraints_override_enabled_flag );
  sps->setSplitConsOverrideEnabledFlag( sps_partition_constraints_override_enabled_flag );

  X_READ_UVLC( sps_log2_diff_min_qt_min_cb_intra_slice_luma, 0, std::min( 6, CtbLog2SizeY ) - MinCbLog2SizeY );
  const unsigned MinQtLog2SizeIntraY = sps_log2_diff_min_qt_min_cb_intra_slice_luma + MinCbLog2SizeY;

  X_READ_UVLC( sps_max_mtt_hierarchy_depth_intra_slice_luma, 0, 2 * ( CtbLog2SizeY - MinCbLog2SizeY ) );

  PartitionConstraints minQT     = { 1u << MinQtLog2SizeIntraY, 0, 0 };
  PartitionConstraints maxBTD    = { sps_max_mtt_hierarchy_depth_intra_slice_luma, 0, 0 };
  PartitionConstraints maxTTSize = { 1u << MinQtLog2SizeIntraY, 0, 0 };
  PartitionConstraints maxBTSize = { 1u << MinQtLog2SizeIntraY, 0, 0 };

  int spsLog2DiffMinQtMinCbIntraSliceLuma = 0;
  if( sps_max_mtt_hierarchy_depth_intra_slice_luma != 0 )
  {
    X_READ_UVLC( sps_log2_diff_max_bt_min_qt_intra_slice_luma, 0, CtbLog2SizeY - MinQtLog2SizeIntraY );
    maxBTSize[0] <<= sps_log2_diff_max_bt_min_qt_intra_slice_luma;
    spsLog2DiffMinQtMinCbIntraSliceLuma = sps_log2_diff_max_bt_min_qt_intra_slice_luma;

    X_READ_UVLC( sps_log2_diff_max_tt_min_qt_intra_slice_luma, 0, std::min( 6, CtbLog2SizeY ) - MinQtLog2SizeIntraY );
    maxTTSize[0] <<= sps_log2_diff_max_tt_min_qt_intra_slice_luma;
  }
  CHECK( maxTTSize[0] > 64, "The value of sps_log2_diff_max_tt_min_qt_intra_slice_luma shall be in the range of 0 to min(6,CtbLog2SizeY) - MinQtLog2SizeIntraY" );

  if( sps_chroma_format_idc != CHROMA_400 )
  {
    X_READ_FLAG( sps_qtbtt_dual_tree_intra_flag );
    sps->setUseDualITree( sps_qtbtt_dual_tree_intra_flag );

    (void)spsLog2DiffMinQtMinCbIntraSliceLuma;
    // this breaks the testset (TREE_C_HHI_3.bit) although specified in the spec
    // CHECK( spsLog2DiffMinQtMinCbIntraSliceLuma > std::min( 6, CtbLog2SizeY ) - MinQtLog2SizeIntraY && sps_qtbtt_dual_tree_intra_flag,
    //        "When sps_log2_diff_max_bt_min_qt_intra_slice_luma is greater than Min( 6, CtbLog2SizeY ) - MinQtLog2SizeIntraY, the value of "
    //        "sps_qtbtt_dual_tree_intra_flag shall be equal to 0." )
  }


  if( sps->getUseDualITree() )
  {
    X_READ_UVLC( sps_log2_diff_min_qt_min_cb_intra_slice_chroma, 0, std::min( 6, CtbLog2SizeY ) - MinCbLog2SizeY );
    const int MinQtLog2SizeIntraC = sps_log2_diff_min_qt_min_cb_intra_slice_chroma + MinCbLog2SizeY;

    X_READ_UVLC( sps_max_mtt_hierarchy_depth_intra_slice_chroma, 0, 2 * ( CtbLog2SizeY - MinCbLog2SizeY ) );
    maxBTD[2] = sps_max_mtt_hierarchy_depth_intra_slice_chroma;

    minQT[2] = 1 << MinQtLog2SizeIntraC;
    maxTTSize[2] = maxBTSize[2] = minQT[2];
    if( sps_max_mtt_hierarchy_depth_intra_slice_chroma != 0 )
    {
      X_READ_UVLC( sps_log2_diff_max_bt_min_qt_intra_slice_chroma, 0, std::min( 6, CtbLog2SizeY ) - MinQtLog2SizeIntraC );
      maxBTSize[2] <<= sps_log2_diff_max_bt_min_qt_intra_slice_chroma;

      X_READ_UVLC( sps_log2_diff_max_tt_min_qt_intra_slice_chroma, 0, std::min( 6, CtbLog2SizeY ) - MinQtLog2SizeIntraC );
      maxTTSize[2] <<= sps_log2_diff_max_tt_min_qt_intra_slice_chroma;

      CHECK( maxTTSize[2] > 64, "The value of sps_log2_diff_max_tt_min_qt_intra_slice_chroma shall be in the range of 0 to min(6,CtbLog2SizeY) - MinQtLog2SizeIntraChroma" );
      CHECK( maxBTSize[2] > 64, "The value of sps_log2_diff_max_bt_min_qt_intra_slice_chroma shall be in the range of 0 to min(6,CtbLog2SizeY) - MinQtLog2SizeIntraChroma" );
    }
  }
  // minQT[2] = 1 << MinQtLog2SizeIntraC; // THIS WAS MISSING? -> only read for dual tree

  X_READ_UVLC( sps_log2_diff_min_qt_min_cb_inter_slice, 0, std::min( 6, CtbLog2SizeY ) - MinCbLog2SizeY );
  const int MinQtLog2SizeInterY = sps_log2_diff_min_qt_min_cb_inter_slice + MinCbLog2SizeY;

  X_READ_UVLC( sps_max_mtt_hierarchy_depth_inter_slice, 0, 2 * ( CtbLog2SizeY - MinCbLog2SizeY ) );
  maxBTD[1] = sps_max_mtt_hierarchy_depth_inter_slice;

  minQT[1] = 1 << MinQtLog2SizeInterY;
  maxTTSize[1] = maxBTSize[1] = minQT[1];
  if( sps_max_mtt_hierarchy_depth_inter_slice != 0 )
  {
    X_READ_UVLC( sps_log2_diff_max_bt_min_qt_inter_slice, 0, CtbLog2SizeY - MinQtLog2SizeInterY );
    maxBTSize[1] <<= sps_log2_diff_max_bt_min_qt_inter_slice;

    X_READ_UVLC( sps_log2_diff_max_tt_min_qt_inter_slice, 0, std::min( 6, CtbLog2SizeY ) - MinQtLog2SizeInterY );
    maxTTSize[1] <<= sps_log2_diff_max_tt_min_qt_inter_slice;
  }

  sps->setMinQTSizes( minQT );
  sps->setMaxMTTHierarchyDepths( maxBTD );
  sps->setMaxBTSizes( maxBTSize );
  sps->setMaxTTSizes( maxTTSize );

  if( CtbSizeY > 32 )
  {
    X_READ_FLAG( sps_max_luma_transform_size_64_flag );
    sps->setLog2MaxTbSize( 5 + sps_max_luma_transform_size_64_flag );
  }
  else
  {
    sps->setLog2MaxTbSize( 5 );
  }

  X_READ_FLAG( sps_transform_skip_enabled_flag );
  sps->setTransformSkipEnabledFlag( sps_transform_skip_enabled_flag );

  if( sps_transform_skip_enabled_flag )
  {
    X_READ_UVLC( sps_log2_transform_skip_max_size_minus2, 0, 3 );
    sps->setLog2MaxTransformSkipBlockSize( sps_log2_transform_skip_max_size_minus2 + 2 );

    X_READ_FLAG( sps_bdpcm_enabled_flag );
    sps->setBDPCMEnabledFlag( sps_bdpcm_enabled_flag );
  }

  X_READ_FLAG( sps_mts_enabled_flag );
  sps->setUseMTS( sps_mts_enabled_flag );

  if( sps_mts_enabled_flag )
  {
    X_READ_FLAG( sps_explicit_mts_intra_enabled_flag );
    sps->setUseIntraMTS( sps_explicit_mts_intra_enabled_flag );

    X_READ_FLAG( sps_explicit_mts_inter_enabled_flag );
    sps->setUseInterMTS( sps_explicit_mts_inter_enabled_flag );
  }

  X_READ_FLAG( sps_lfnst_enabled_flag );
  sps->setUseLFNST( sps_lfnst_enabled_flag );

  if( sps_chroma_format_idc != CHROMA_400 )
  {
    X_READ_FLAG( sps_joint_cbcr_enabled_flag );
    sps->setJointCbCrEnabledFlag( sps_joint_cbcr_enabled_flag );

    X_READ_FLAG( sps_same_qp_table_for_chroma_flag );
    ChromaQpMappingTableParams chromaQpMappingTableParams;
    chromaQpMappingTableParams.setSameCQPTableForAllChromaFlag( sps_same_qp_table_for_chroma_flag );

    const int numQpTables = sps_same_qp_table_for_chroma_flag ? 1 : ( sps_joint_cbcr_enabled_flag ? 3 : 2 );
    chromaQpMappingTableParams.setNumQpTables( numQpTables );

    for( int i = 0; i < numQpTables; i++ )
    {
      X_READ_SVLC( sps_qp_table_start_minus26, -26 - QpBdOffset, 36 );
      chromaQpMappingTableParams.setQpTableStartMinus26( i, sps_qp_table_start_minus26 );

      X_READ_UVLC( sps_num_points_in_qp_table_minus1, 0, 36u - sps_qp_table_start_minus26 );
      chromaQpMappingTableParams.setNumPtsInCQPTableMinus1( i, sps_num_points_in_qp_table_minus1 );

      std::vector<int> deltaQpInValMinus1( sps_num_points_in_qp_table_minus1 + 1 );
      std::vector<int> deltaQpOutVal( sps_num_points_in_qp_table_minus1 + 1 );
      for( unsigned j = 0; j <= sps_num_points_in_qp_table_minus1; j++ )
      {
        X_READ_UVLC_NO_RANGE( sps_delta_qp_in_val_minus1 );   // checked later in ChromaQpMappingTable::deriveChromaQPMappingTables()
        deltaQpInValMinus1[j] = sps_delta_qp_in_val_minus1;

        X_READ_UVLC_NO_RANGE( sps_delta_qp_diff_val );   // checked later in ChromaQpMappingTable::deriveChromaQPMappingTables()
        deltaQpOutVal[j] = sps_delta_qp_diff_val ^ deltaQpInValMinus1[j];
      }
      chromaQpMappingTableParams.setDeltaQpInValMinus1( i, deltaQpInValMinus1 );
      chromaQpMappingTableParams.setDeltaQpOutVal( i, deltaQpOutVal );
    }
    chromaQpMappingTableParams.m_qpBdOffset = sps->getQpBDOffset();
    sps->setChromaQpMappingTableFromParams( std::move( chromaQpMappingTableParams ) );
    sps->deriveChromaQPMappingTables();
  }


  X_READ_FLAG( sps_sao_enabled_flag );
  sps->setUseSAO( sps_sao_enabled_flag );

  X_READ_FLAG( sps_alf_enabled_flag );
  sps->setUseALF( sps_alf_enabled_flag );

  if( sps_alf_enabled_flag && sps_chroma_format_idc != CHROMA_400 )
  {
    X_READ_FLAG( sps_ccalf_enabled_flag );
    sps->setUseCCALF( sps_ccalf_enabled_flag );
  }
  else
  {
    sps->setUseCCALF( false );
  }

  X_READ_FLAG( sps_lmcs_enable_flag );
  sps->setUseReshaper( sps_lmcs_enable_flag );

  X_READ_FLAG( sps_weighted_pred_flag );
  sps->setUseWP( sps_weighted_pred_flag );

  X_READ_FLAG( sps_weighted_bipred_flag );
  sps->setUseWPBiPred( sps_weighted_bipred_flag );

  X_READ_FLAG( sps_long_term_ref_pics_flag );
  sps->setLongTermRefsPresent( sps_long_term_ref_pics_flag );

  if( sps_video_parameter_set_id > 0 )
  {
    X_READ_FLAG( sps_inter_layer_prediction_enabled_flag );
    sps->setInterLayerPresentFlag( sps_inter_layer_prediction_enabled_flag );
    // CHECK( vps->getIndependentLayerFlag( vps->getGeneralLayerIdx( nuh_layer_id ) ) == 1 && sps_inter_layer_prediction_enabled_flag != 0,
    //        "When vps_independent_layer_flag[ GeneralLayerIdx[ nuh_layer_id ] ] is equal to 1,"
    //        " the value of sps_inter_layer_prediction_enabled_flag shall be equal to 0." )
  }

  X_READ_FLAG( sps_idr_rpl_present_flag );
  CHECK_CONSTRAINT( gci->getNoIdrRplConstraintFlag() && sps_idr_rpl_present_flag,
                    "When gci_no_idr_rpl_constraint_flag equal to 1, the value of sps_idr_rpl_present_flag shall be equal to 0" );
  sps->setIDRRefParamListPresent( sps_idr_rpl_present_flag );

  X_READ_FLAG( sps_rpl1_same_as_rpl0_flag );
  sps->setRPL1CopyFromRPL0Flag( sps_rpl1_same_as_rpl0_flag );

  for( unsigned i = 0; i < ( sps_rpl1_same_as_rpl0_flag ? 1 : 2 ); i++ )
  {
    X_READ_UVLC_idx( sps_num_ref_pic_lists, "[i]", 0, 64 );

    RPLList& rplList = sps->createRPLList( i, sps_num_ref_pic_lists );
    for( unsigned j = 0; j < sps_num_ref_pic_lists; j++ )
    {
      parseRefPicList( &rplList[j], j, sps );
    }
  }
  if( sps_rpl1_same_as_rpl0_flag )
  {
    const unsigned numberOfRPL   = sps->getNumRPL( 0 );
    const RPLList& rplListSource = sps->getRPLList( 0 );
    RPLList&       rplListDest   = sps->createRPLList( 1, numberOfRPL );
    for( unsigned j = 0; j < numberOfRPL; j++ )
    {
      copyRefPicList( sps, &rplListSource[j], &rplListDest[j] );
    }
  }


  X_READ_FLAG( sps_ref_wraparound_enabled_flag );
  sps->setUseWrapAround( sps_ref_wraparound_enabled_flag );
  for( int i = 0; i < sps->getNumSubPics(); ++i )
  {
    CHECK( sps->getSubPicTreatedAsPicFlag( i ) == 1 && sps->getSubPicWidth( i ) != ( sps_pic_width_max_in_luma_samples + CtbSizeY - 1 ) >> CtbLog2SizeY
             && sps_ref_wraparound_enabled_flag != 0,
           "It is a requirement of bitstream conformance that, when there is one or more values of i in the range of 0 to sps_num_subpics_minus1, inclusive,"
           " for which sps_subpic_treated_as_pic_flag[ i ] is equal to 1 and sps_subpic_width_minus1[ i ] plus 1 is not equal to"
           " ( sps_pic_width_max_in_luma_samples + CtbSizeY− 1 ) >> CtbLog2SizeY ), the value of sps_ref_wraparound_enabled_flag shall be equal to 0." );
  }

  X_READ_FLAG( sps_temporal_mvp_enabled_flag );
  sps->setSPSTemporalMVPEnabledFlag( sps_temporal_mvp_enabled_flag );

  if( sps_temporal_mvp_enabled_flag )
  {
    X_READ_FLAG( sps_sbtmvp_enabled_flag );
    sps->setSBTMVPEnabledFlag( sps_sbtmvp_enabled_flag );
  }

  X_READ_FLAG( sps_amvr_enabled_flag );
  sps->setAMVREnabledFlag( sps_amvr_enabled_flag );

  X_READ_FLAG( sps_bdof_enabled_flag );
  sps->setUseBIO( sps_bdof_enabled_flag );

  if( sps_bdof_enabled_flag )
  {
    X_READ_FLAG( sps_bdof_control_present_in_ph_flag );
    sps->setBdofControlPresentInPhFlag( sps_bdof_control_present_in_ph_flag );
  }

  X_READ_FLAG( sps_smvd_enabled_flag );
  sps->setUseSMVD( sps_smvd_enabled_flag );

  X_READ_FLAG( sps_dmvr_enabled_flag );
  sps->setUseDMVR( sps_dmvr_enabled_flag );

  if( sps_dmvr_enabled_flag )
  {
    X_READ_FLAG( sps_dmvr_control_present_in_ph_flag );
    sps->setDmvrControlPresentInPhFlag( sps_dmvr_control_present_in_ph_flag );
  }

  X_READ_FLAG( sps_mmvd_enabled_flag );
  sps->setUseMMVD( sps_mmvd_enabled_flag );

  if( sps->getUseMMVD() )
  {
    X_READ_FLAG( sps_mmvd_fullpel_only_flag );
    sps->setFpelMmvdEnabledFlag( sps_mmvd_fullpel_only_flag );
  }

  X_READ_UVLC( sps_six_minus_max_num_merge_cand, 0, 5 );
  const unsigned MaxNumMergeCand = MRG_MAX_NUM_CANDS - sps_six_minus_max_num_merge_cand;
  sps->setMaxNumMergeCand( MaxNumMergeCand );

  X_READ_FLAG( sps_sbt_enabled_flag );
  sps->setUseSBT( sps_sbt_enabled_flag );

  X_READ_FLAG( sps_affine_enabled_flag );
  sps->setUseAffine( sps_affine_enabled_flag );

  if( sps_affine_enabled_flag )
  {
    X_READ_UVLC( sps_five_minus_max_num_subblock_merge_cand, 0, 5 - sps->getSBTMVPEnabledFlag() );
    sps->setMaxNumAffineMergeCand( AFFINE_MRG_MAX_NUM_CANDS - sps_five_minus_max_num_subblock_merge_cand );

    X_READ_FLAG( sps_6param_affine_enabled_flag );
    sps->setUseAffineType( sps_6param_affine_enabled_flag );

    if( sps_amvr_enabled_flag )
    {
      X_READ_FLAG( sps_affine_amvr_enabled_flag );
      sps->setAffineAmvrEnabledFlag( sps_affine_amvr_enabled_flag );
    }

    X_READ_FLAG( sps_affine_prof_enabled_flag );
    sps->setUsePROF( sps_affine_prof_enabled_flag );

    if( sps_affine_prof_enabled_flag )
    {
      X_READ_FLAG( sps_prof_control_present_in_ph_flag );
      sps->setProfControlPresentInPhFlag( sps_prof_control_present_in_ph_flag );
    }
  }

  X_READ_FLAG( sps_bcw_enabled_flag );
  sps->setUseBcw( sps_bcw_enabled_flag );

  X_READ_FLAG( sps_ciip_enabled_flag );
  sps->setUseCiip( sps_ciip_enabled_flag );

  if( MaxNumMergeCand >= 2 )
  {
    X_READ_FLAG( sps_gpm_enabled_flag );
    sps->setUseGeo( sps_gpm_enabled_flag );

    if( sps_gpm_enabled_flag && MaxNumMergeCand >= 3 )
    {
      X_READ_UVLC( sps_max_num_merge_cand_minus_max_num_gpm_cand, 0, MaxNumMergeCand - 2 );
      sps->setMaxNumGeoCand( MaxNumMergeCand - sps_max_num_merge_cand_minus_max_num_gpm_cand );
    }
    else if( sps_gpm_enabled_flag )
    {
      sps->setMaxNumGeoCand( 2 );
    }
  }

  X_READ_UVLC( sps_log2_parallel_merge_level_minus2, 0, CtbLog2SizeY - 2 );
  sps->setLog2ParallelMergeLevelMinus2( sps_log2_parallel_merge_level_minus2 );

  X_READ_FLAG( sps_isp_enabled_flag );
  sps->setUseISP( sps_isp_enabled_flag );

  X_READ_FLAG( sps_mrl_enabled_flag );
  sps->setUseMRL( sps_mrl_enabled_flag );

  X_READ_FLAG( sps_mip_enabled_flag );
  sps->setUseMIP( sps_mip_enabled_flag );

  if( sps_chroma_format_idc != CHROMA_400 )
  {
    X_READ_FLAG( sps_cclm_enabled_flag );
    sps->setUseLMChroma( sps_cclm_enabled_flag );
  }

  if( sps_chroma_format_idc == CHROMA_420 )
  {
    X_READ_FLAG( sps_chroma_horizontal_collocated_flag );
    sps->setHorCollocatedChromaFlag( sps_chroma_horizontal_collocated_flag );

    X_READ_FLAG( sps_chroma_vertical_collocated_flag );
    sps->setVerCollocatedChromaFlag( sps_chroma_vertical_collocated_flag );
  }

  X_READ_FLAG( sps_palette_enabled_flag );
  CHECK( sps_palette_enabled_flag, "palette mode is not yet supported" );

  if( sps_chroma_format_idc == CHROMA_444 && sps->getLog2MaxTbSize() != 6 )
  {
    X_READ_FLAG( sps_act_enabled_flag );
    sps->setUseColorTrans( sps_act_enabled_flag );
  }

  if( sps_transform_skip_enabled_flag || sps_palette_enabled_flag )
  {
    X_READ_UVLC( sps_internal_bit_depth_minus_input_bit_depth, 0, 8 );   // Why is this called sps_min_qp_prime_ts in the standard?
    sps->setInternalMinusInputBitDepth( sps_internal_bit_depth_minus_input_bit_depth );
  }

  X_READ_FLAG( sps_ibc_enabled_flag );
  sps->setIBCFlag( sps_ibc_enabled_flag );

  if( sps_ibc_enabled_flag )
  {
    X_READ_UVLC( sps_six_minus_max_num_ibc_merge_cand, 0, 5 );
    sps->setMaxNumIBCMergeCand( IBC_MRG_MAX_NUM_CANDS - sps_six_minus_max_num_ibc_merge_cand );
  }

#  if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
  X_READ_FLAG( sps_ladf_enabled_flag );
  sps->setLadfEnabled( sps_ladf_enabled_flag );

  if( sps->getLadfEnabled() )
  {
    X_READ_CODE( sps_num_ladf_intervals_minus2, 2, 0, 3 );
    sps->setLadfNumIntervals( sps_num_ladf_intervals_minus2 + 2 );

    X_READ_SVLC( sps_ladf_lowest_interval_qp_offset, -63, 63 );
    sps->setLadfQpOffset( sps_ladf_lowest_interval_qp_offset, 0 );

    for( unsigned i = 0; i < sps_num_ladf_intervals_minus2 + 1; i++ )
    {
      X_READ_SVLC_idx( sps_ladf_qp_offset, "[i]", -63, 63 );
      sps->setLadfQpOffset( sps_ladf_qp_offset, i + 1 );

      X_READ_UVLC_idx( sps_ladf_delta_threshold_minus1, "[i]", 0, ( 1 << BitDepth ) - 3 );
      sps->setLadfIntervalLowerBound( sps->getLadfIntervalLowerBound( i ) + sps_ladf_delta_threshold_minus1 + 1, i + 1 );
    }
  }
#endif

  X_READ_FLAG( sps_explicit_scaling_list_enabled_flag );
  CHECK_CONSTRAINT( gci->getNoExplicitScaleListConstraintFlag() && sps_explicit_scaling_list_enabled_flag,
                    "When gci_no_explicit_scaling_list_constraint_flag is equal to 1, sps_explicit_scaling_list_enabled_flag shall be equal to 0" );
  sps->setScalingListFlag( sps_explicit_scaling_list_enabled_flag );

  if( sps_lfnst_enabled_flag && sps_explicit_scaling_list_enabled_flag )
  {
    X_READ_FLAG( sps_scaling_matrix_for_lfnst_disabled_flag );
    sps->setDisableScalingMatrixForLfnstBlks( sps_scaling_matrix_for_lfnst_disabled_flag );
  }

  if( sps->getUseColorTrans() && sps_explicit_scaling_list_enabled_flag )
  {
    X_READ_FLAG( sps_scaling_matrix_for_alternative_colour_space_disabled_flag );
    sps->setScalingMatrixForAlternativeColourSpaceDisabledFlag( sps_scaling_matrix_for_alternative_colour_space_disabled_flag );

    if( sps_scaling_matrix_for_alternative_colour_space_disabled_flag )
    {
      X_READ_FLAG( sps_scaling_matrix_designated_colour_space_flag );
      sps->setScalingMatrixDesignatedColourSpaceFlag( sps_scaling_matrix_designated_colour_space_flag );
    }
  }

  X_READ_FLAG( sps_dep_quant_enabled_flag );
  sps->setDepQuantEnabledFlag( sps_dep_quant_enabled_flag );

  X_READ_FLAG( sps_sign_data_hiding_enabled_flag );
  sps->setSignDataHidingEnabledFlag( sps_sign_data_hiding_enabled_flag );

  X_READ_FLAG( sps_virtual_boundaries_enabled_flag );
  CHECK_CONSTRAINT( gci->getNoVirtualBoundaryConstraintFlag() && sps_virtual_boundaries_enabled_flag,
                    "When gci_no_virtual_boundaries_constraint_flag is equal to 1, sps_virtual_boundaries_enabled_flag shall be equal to 0" );
  sps->setVirtualBoundariesEnabledFlag( sps_virtual_boundaries_enabled_flag );

  if( sps_virtual_boundaries_enabled_flag )
  {
    X_READ_FLAG( sps_virtual_boundaries_present_flag );
    sps->setVirtualBoundariesPresentFlag( sps_virtual_boundaries_present_flag );

    if( sps_virtual_boundaries_present_flag )
    {
      X_READ_UVLC( sps_num_ver_virtual_boundaries, 0, sps_pic_width_max_in_luma_samples <= 8 ? 0 : 3 );
      sps->setNumVerVirtualBoundaries( sps_num_ver_virtual_boundaries );

      for( unsigned i = 0; i < sps_num_ver_virtual_boundaries; i++ )
      {
        X_READ_UVLC_idx( sps_virtual_boundary_pos_x_minus1, "[i]", 0, ( sps_pic_width_max_in_luma_samples + 7 ) / 8 - 2 );
        sps->setVirtualBoundariesPosX( ( sps_virtual_boundary_pos_x_minus1 + 1 ) << 3, i );
      }

      X_READ_UVLC_idx( sps_num_hor_virtual_boundaries, "[i]", 0, sps_pic_height_max_in_luma_samples <= 8 ? 0 : 3  );
      sps->setNumHorVirtualBoundaries( sps_num_hor_virtual_boundaries );

      for( unsigned i = 0; i <sps_num_hor_virtual_boundaries; i++ )
      {
        X_READ_UVLC_idx( sps_virtual_boundary_pos_y_minus1, "[i]", 0, ( sps_pic_height_max_in_luma_samples + 7 ) / 8 - 2 );
        sps->setVirtualBoundariesPosY( (sps_virtual_boundary_pos_y_minus1 + 1) << 3, i );
      }
    }
  }

  if( sps_ptl_dpb_hrd_params_present_flag )
  {
    X_READ_FLAG( sps_timing_hrd_params_present_flag );
    sps->setGeneralHrdParametersPresentFlag( sps_timing_hrd_params_present_flag );

    if( sps_timing_hrd_params_present_flag )
    {
      parseGeneralHrdParameters( sps->getGeneralHrdParameters() );
      if( sps_max_sublayers_minus1 > 0 )
      {
        X_READ_FLAG( sps_sublayer_cpb_params_present_flag );
        sps->setSubLayerParametersPresentFlag( sps_sublayer_cpb_params_present_flag );
      }

      uint32_t firstSubLayer = sps->getSubLayerParametersPresentFlag() ? 0 : sps_max_sublayers_minus1;
      parseOlsHrdParameters( sps->getGeneralHrdParameters(), sps->getOlsHrdParameters(), firstSubLayer, sps_max_sublayers_minus1 );
    }
  }

  X_READ_FLAG( sps_field_seq_flag );
  sps->setFieldSeqFlag( sps_field_seq_flag );

  CHECK_CONSTRAINT( sps->getProfileTierLevel()->getFrameOnlyConstraintFlag() && sps_field_seq_flag,
                     "When ptl_frame_only_constraint_flag equal to 1 , the value of sps_field_seq_flag shall be equal to 0" );

  X_READ_FLAG( sps_vui_parameters_present_flag );
  sps->setVuiParametersPresentFlag( sps_vui_parameters_present_flag );

  if( sps_vui_parameters_present_flag )
  {
    X_READ_UVLC( sps_vui_payload_size_minus1, 0, 1023 );
    sps->setVuiPayloadSize( sps_vui_payload_size_minus1 + 1 );

    while( !isByteAligned() )
    {
      X_READ_FLAG( sps_vui_alignment_zero_bit );
      CHECK( sps_vui_alignment_zero_bit, "sps_vui_alignment_zero_bit not equal to 0" );
    }

    parseVUI( sps->getVuiParameters(), sps->getVuiPayloadSize() );
  }

  X_READ_FLAG( sps_extension_present_flag );
  if( sps_extension_present_flag )
  {
    while( xMoreRbspData() )
    {
      X_READ_FLAG( sps_extension_data_flag );
      (void)sps_extension_data_flag;
    }
  }

  xReadRbspTrailingBits();
}

void HLSyntaxReader::parseDCI( DCI* dci )
{
#if ENABLE_TRACING
  xTraceDCIHeader();
#endif
  X_READ_CODE_NO_RANGE( dci_reserved_zero_4bits, 4 );
  (void) dci_reserved_zero_4bits;

  X_READ_CODE( dci_num_ptls_minus1, 4, 0, 15 );
  CHECK_WARN( dci_num_ptls_minus1 == 15, "reserved dci_num_ptls_minus1==15 used" );
  const int numPTLs = dci_num_ptls_minus1 + 1;

  std::vector<ProfileTierLevel> ptls( numPTLs );
  for( int i = 0; i < numPTLs; i++ )
  {
    parseProfileTierLevel( &ptls[i], true, 0 );
  }
  dci->setProfileTierLevel( std::move( ptls ) );

  X_READ_FLAG(dci_extension_flag );
  if( dci_extension_flag )
  {
    while( xMoreRbspData() )
    {
      X_READ_FLAG( dci_extension_data_flag );
      (void) dci_extension_data_flag;
    }
  }
  xReadRbspTrailingBits();
}

#if 0
// We don't parse the VPS, because the needed bounds checks in parseVPS() are not yet implemented, and we don't process it anyways
void HLSyntaxReader::parseVPS( VPS* pcVPS )
{
#if ENABLE_TRACING
  xTraceVPSHeader();
#endif
  uint32_t  uiCode;

  //CHECK( true, "needs to be adjusted, e.g. sublayer and independent layer stuff -> see VTM-9.0" );

  READ_CODE( 4, uiCode, "vps_video_parameter_set_id" );                      pcVPS->setVPSId( uiCode );
  CHECK( uiCode == 0, "vps_video_parameter_set_id equal to zero is reserved and shall not be used in a bitstream" );

  READ_CODE( 6, uiCode, "vps_max_layers_minus1" );                           pcVPS->setMaxLayers( uiCode + 1 );
  CHECK( uiCode + 1 > MAX_VPS_LAYERS, "Signalled number of layers larger than MAX_VPS_LAYERS." );

  if( pcVPS->getMaxLayers() - 1 == 0 )
  {
    pcVPS->setEachLayerIsAnOlsFlag( 1 );
  }
  READ_CODE( 3, uiCode, "vps_max_sublayers_minus1" );                        pcVPS->setMaxSubLayers( uiCode + 1 );
  CHECK( uiCode + 1 > MAX_VPS_SUBLAYERS, "Signalled number of sublayers larger than MAX_VPS_SUBLAYERS." );

  if( pcVPS->getMaxLayers() > 1 && pcVPS->getMaxSubLayers() > 1 )
  {
    READ_FLAG( uiCode, "vps_default_ptl_dpb_hrd_max_tid_flag" );             pcVPS->setAllLayersSameNumSublayersFlag( uiCode );
  }
  else
  {
    pcVPS->setAllLayersSameNumSublayersFlag( 1 );
  }
  if( pcVPS->getMaxLayers() > 1 )
  {
    READ_FLAG( uiCode, "vps_all_independent_layers_flag" );                  pcVPS->setAllIndependentLayersFlag( uiCode );
    if( pcVPS->getAllIndependentLayersFlag() == 0 )
    {
      pcVPS->setEachLayerIsAnOlsFlag( 0 );
    }
  }
  for( uint32_t i = 0; i < pcVPS->getMaxLayers(); i++ )
  {
    READ_CODE( 6, uiCode, "vps_layer_id[i]" );                             pcVPS->setLayerId( i, uiCode );
    pcVPS->setGeneralLayerIdx( uiCode, i );

    if( i > 0 && !pcVPS->getAllIndependentLayersFlag() )
    {
      READ_FLAG( uiCode, "vps_independent_layer_flag[i]" );                pcVPS->setIndependentLayerFlag( i, uiCode );
      if( !pcVPS->getIndependentLayerFlag( i ) )
      {
        READ_FLAG( uiCode, "vps_max_tid_ref_present_flag[i]" );
        bool vpsMaxTidRefPresentFlag = ( uiCode == 1 );

        uint16_t sumUiCode = 0;
        for( int j = 0, k = 0; j < i; j++ )
        {
          READ_FLAG( uiCode, "vps_direct_ref_layer_flag[ i ][ j ]" );        pcVPS->setDirectRefLayerFlag( i, j, uiCode );
          if( uiCode )
          {
            pcVPS->setInterLayerRefIdc( i, j, k );
            pcVPS->setDirectRefLayerIdx( i, k++, j );
            sumUiCode++;
            if( vpsMaxTidRefPresentFlag )
            {
              READ_CODE( 3, uiCode, "vps_max_tid_il_ref_pics_plus1[i][ j ]" );
//              pcVPS->setMaxTidIlRefPicsPlus1( i, uiCode );
            }
          }
        }
        CHECK(sumUiCode == 0, "There has to be at least one value of j such that the value of vps_direct_dependency_flag[i][ j ] is equal to 1,when vps_independent_layer_flag[i] is equal to 0 " );
//        if( uiCode )
//        {
//          READ_CODE( 3, uiCode, "max_tid_il_ref_pics_plus1[i]" ); pcVPS->setMaxTidIlRefPicsPlus1( i, uiCode );
//        }
//        else
//        {
//          pcVPS->setMaxTidIlRefPicsPlus1( i, 7) ;
//        }
      }
    }
  }

  if( pcVPS->getMaxLayers() > 1 )
  {
    if( pcVPS->getAllIndependentLayersFlag() )
    {
      READ_FLAG( uiCode, "vps_each_layer_is_an_ols_flag" );                  pcVPS->setEachLayerIsAnOlsFlag( uiCode );
      if( pcVPS->getEachLayerIsAnOlsFlag() == 0 )
      {
        pcVPS->setOlsModeIdc( 2 );
      }
    }
    if( !pcVPS->getEachLayerIsAnOlsFlag() )
    {
      if( !pcVPS->getAllIndependentLayersFlag() )
      {
        READ_CODE( 2, uiCode, "vps_ols_mode_idc" );                          pcVPS->setOlsModeIdc( uiCode );
        CHECK( uiCode > MAX_VPS_OLS_MODE_IDC, "ols_mode_idc shall be in the rage of 0 to 2" );
      }
      if( pcVPS->getOlsModeIdc() == 2 )
      {
        READ_CODE( 8, uiCode, "vps_num_output_layer_sets_minus2" );          pcVPS->setNumOutputLayerSets( uiCode + 2 );
        for( uint32_t i = 1; i <= pcVPS->getNumOutputLayerSets() - 1; i++ )
        {
          for( uint32_t j = 0; j < pcVPS->getMaxLayers(); j++ )
          {
            READ_FLAG( uiCode, "vps_ols_output_layer_flag[i][ j ]" );      pcVPS->setOlsOutputLayerFlag( i, j, uiCode );
          }
        }
      }
    }
    READ_CODE( 8, uiCode, "vps_num_ptls_minus1" );                           pcVPS->setNumPtls( uiCode + 1 );
  }
  else
  {
    pcVPS->setNumPtls( 1 );
  }

  pcVPS->deriveOutputLayerSets();
  CHECK( uiCode >= pcVPS->getTotalNumOLSs(), "The value of vps_num_ptls_minus1 shall be less than TotalNumOlss" );

  std::vector<bool> isPTLReferred( pcVPS->getNumPtls(), false );
  for( int i = 0; i < pcVPS->getNumPtls(); i++ )
  {
    if( i > 0 )
    {
      READ_FLAG( uiCode, "pt_present_flag[i]" );                           pcVPS->setPtPresentFlag( i, uiCode );
    }
    else
    {
       pcVPS->setPtPresentFlag( 0, 1 );
    }

    if( !pcVPS->getAllLayersSameNumSublayersFlag() )
    {
      READ_CODE( 3, uiCode, "ptl_max_tid[i]" );                            pcVPS->setPtlMaxTemporalId( i, uiCode );
    }
    else
    {
      pcVPS->setPtlMaxTemporalId( i, pcVPS->getMaxSubLayers() - 1 );
    }
  }
  int cnt = 0;
  while( m_pcBitstream->getNumBitsUntilByteAligned() )
  {
    READ_FLAG( uiCode, "vps_ptl_reserved_zero_bit" );
    CHECK( uiCode!=0, "Alignment bit is not '0'" );
    cnt++;
  }
  CHECK( cnt >= 8, "Read more than '8' alignment bits" );
  {
    std::vector<ProfileTierLevel> ptls( pcVPS->getNumPtls() );
    for( int i = 0; i < pcVPS->getNumPtls(); i++ )
    {
      parseProfileTierLevel( &ptls[i], pcVPS->getPtPresentFlag( i ), pcVPS->getPtlMaxTemporalId( i ) );
    }
    pcVPS->setProfileTierLevel( std::move( ptls ) );
  }
  for( int i = 0; i < pcVPS->getTotalNumOLSs(); i++ )
  {
    if( pcVPS->getNumPtls() > 1 && pcVPS->getNumPtls() != pcVPS->getTotalNumOLSs() )
    {
      READ_CODE( 8, uiCode, "vps_ols_ptl_idx[i]" );                        pcVPS->setOlsPtlIdx( i, uiCode );
    }
    else if( pcVPS->getNumPtls() == pcVPS->getTotalNumOLSs() )
    {
      pcVPS->setOlsPtlIdx( i, i );
    }
    else
    {
      pcVPS->setOlsPtlIdx( i, 0 );
    }
    isPTLReferred[pcVPS->getOlsPtlIdx( i )] = true;
  }
  for( int i = 0; i < pcVPS->getNumPtls(); i++ )
  {
    CHECK( !isPTLReferred[i],"Each profile_tier_level( ) syntax structure in the VPS shall be referred to by at least one value of vps_ols_ptl_idx[i] for i in the range of 0 to TotalNumOlss ? 1, inclusive" );
  }

  if( !pcVPS->getEachLayerIsAnOlsFlag() )
  {
    READ_UVLC( uiCode, "vps_num_dpb_params_minus1" );                        pcVPS->m_numDpbParams = uiCode + 1;

    CHECK( pcVPS->m_numDpbParams > pcVPS->getNumMultiLayeredOlss(), "The value of vps_num_dpb_params_minus1 shall be in the range of 0 to NumMultiLayerOlss - 1, inclusive" );
    std::vector<bool> isDPBParamReferred( pcVPS->m_numDpbParams, false );

    if( pcVPS->m_numDpbParams > 0 && pcVPS->getMaxSubLayers() > 1 )
    {
      READ_FLAG( uiCode, "vps_sublayer_dpb_params_present_flag" );           pcVPS->m_sublayerDpbParamsPresentFlag = uiCode;
    }

    pcVPS->m_dpbParameters.resize( pcVPS->m_numDpbParams );

    for( int i = 0; i < pcVPS->m_numDpbParams; i++ )
    {
      if( !pcVPS->getAllLayersSameNumSublayersFlag() )
      {
        READ_CODE( 3, uiCode, "vps_dpb_max_tid[i]" );
        pcVPS->m_dpbMaxTemporalId.push_back( uiCode );
        CHECK( uiCode > ( pcVPS->getMaxSubLayers() - 1 ), "The value of vps_dpb_max_tid[i] shall be in the range of 0 to vps_max_sublayers_minus1, inclusive." )
      }
      else
      {
        pcVPS->m_dpbMaxTemporalId.push_back( pcVPS->getMaxSubLayers() - 1 );
      }

      for( int j = ( pcVPS->m_sublayerDpbParamsPresentFlag ? 0 : pcVPS->m_dpbMaxTemporalId[i] ); j <= pcVPS->m_dpbMaxTemporalId[i]; j++ )
      {
        READ_UVLC( uiCode, "dpb_max_dec_pic_buffering_minus1[i]" );        pcVPS->m_dpbParameters[i].m_maxDecPicBuffering[j] = uiCode + 1;
        READ_UVLC( uiCode, "dpb_max_num_reorder_pics[i]" );                pcVPS->m_dpbParameters[i].m_numReorderPics[j] = uiCode;
        READ_UVLC( uiCode, "dpb_max_latency_increase_plus1[i]" );          pcVPS->m_dpbParameters[i].m_maxLatencyIncreasePlus1[j] = uiCode;
      }

      for( int j = ( pcVPS->m_sublayerDpbParamsPresentFlag ? pcVPS->m_dpbMaxTemporalId[i] : 0 ); j < pcVPS->m_dpbMaxTemporalId[i]; j++ )
      {
        // When max_dec_pic_buffering_minus1[i] is not present for i in the range of 0 to maxSubLayersMinus1 - 1, inclusive, due to subLayerInfoFlag being equal to 0, it is inferred to be equal to max_dec_pic_buffering_minus1[ maxSubLayersMinus1 ].
        pcVPS->m_dpbParameters[i].m_maxDecPicBuffering[j] = pcVPS->m_dpbParameters[i].m_maxDecPicBuffering[pcVPS->m_dpbMaxTemporalId[i]];

        // When max_num_reorder_pics[i] is not present for i in the range of 0 to maxSubLayersMinus1 - 1, inclusive, due to subLayerInfoFlag being equal to 0, it is inferred to be equal to max_num_reorder_pics[ maxSubLayersMinus1 ].
        pcVPS->m_dpbParameters[i].m_numReorderPics[j] = pcVPS->m_dpbParameters[i].m_numReorderPics[pcVPS->m_dpbMaxTemporalId[i]];

        // When max_latency_increase_plus1[i] is not present for i in the range of 0 to maxSubLayersMinus1 - 1, inclusive, due to subLayerInfoFlag being equal to 0, it is inferred to be equal to max_latency_increase_plus1[ maxSubLayersMinus1 ].
        pcVPS->m_dpbParameters[i].m_maxLatencyIncreasePlus1[j] = pcVPS->m_dpbParameters[i].m_maxLatencyIncreasePlus1[pcVPS->m_dpbMaxTemporalId[i]];
      }
    }

    for( int i = 0, j=0; i < pcVPS->getTotalNumOLSs(); i++ )
    {
      if( pcVPS->m_numLayersInOls[i] > 1 )
      {
        READ_UVLC( uiCode, "vps_ols_dpb_pic_width[i]" );                   pcVPS->setOlsDpbPicWidth( i, uiCode );
        READ_UVLC( uiCode, "vps_ols_dpb_pic_height[i]" );                  pcVPS->setOlsDpbPicHeight( i, uiCode );
        READ_CODE( 2, uiCode, "vps_ols_dpb_chroma_format[i]" );            pcVPS->setOlsDpbChromaFormatIdc( i, uiCode );
        READ_UVLC( uiCode, "vps_ols_dpb_bitdepth_minus8[i]" );             pcVPS->setOlsDpbBitDepthMinus8( i, uiCode );
        const Profile::Name profile = pcVPS->getProfileTierLevel( pcVPS->getOlsPtlIdx( i ) ).getProfileIdc();
        if( profile != Profile::NONE )
        {
          CHECK( uiCode + 8 > ProfileFeatures::getProfileFeatures( profile )->maxBitDepth,
                 "vps_ols_dpb_bitdepth_minus8[ i ] exceeds range supported by signalled profile" );
        }
        if( ( pcVPS->m_numDpbParams > 1 ) && ( pcVPS->m_numDpbParams != pcVPS->m_numMultiLayeredOlss) )
        {
          READ_UVLC( uiCode, "vps_ols_dpb_params_idx[i]" );                pcVPS->setOlsDpbParamsIdx( i, uiCode );
        }
        else if( pcVPS->m_numDpbParams == 1 )
        {
          pcVPS->setOlsDpbParamsIdx( i, 0 );
        }
        else
        {
          pcVPS->setOlsDpbParamsIdx( i, j );
        }
        j += 1;
        isDPBParamReferred[pcVPS->getOlsDpbParamsIdx( i )] = true;
      }
    }
    for( int i = 0; i < pcVPS->m_numDpbParams; i++ )
    {
      CHECK( !isDPBParamReferred[i], "Each dpb_parameters() syntax structure in the VPS shall be referred to by at least one value of vps_ols_dpb_params_idx[i] for i in the range of 0 to NumMultiLayerOlss - 1, inclusive" );
    }
  }

  if( !pcVPS->getEachLayerIsAnOlsFlag() )
  {
    READ_FLAG( uiCode, "vps_general_hrd_params_present_flag" );              pcVPS->setVPSGeneralHrdParamsPresentFlag( uiCode );
  }
  if( pcVPS->getVPSGeneralHrdParamsPresentFlag() )
  {
    parseGeneralHrdParameters( pcVPS->getGeneralHrdParameters() );
    if( ( pcVPS->getMaxSubLayers() - 1 ) > 0 )
    {
      READ_FLAG( uiCode, "vps_sublayer_cpb_params_present_flag" );           pcVPS->setVPSSublayerCpbParamsPresentFlag( uiCode );
    }
    else
    {
      pcVPS->setVPSSublayerCpbParamsPresentFlag( 0 );
    }

    READ_UVLC( uiCode, "vps_num_ols_hrd_params_minus1" );                    pcVPS->setNumOlsHrdParamsMinus1( uiCode );
    CHECK( uiCode >= pcVPS->getNumMultiLayeredOlss(), "The value of vps_num_ols_hrd_params_minus1 shall be in the range of 0 to NumMultiLayerOlss - 1, inclusive" );
    std::vector<bool> isHRDParamReferred( uiCode + 1, false );
    pcVPS->m_olsHrdParams.clear();
    pcVPS->m_olsHrdParams.resize( pcVPS->getNumOlsHrdParamsMinus1() + 1, std::vector<OlsHrdParams>( pcVPS->getMaxSubLayers() ) );
    for( int i = 0; i <= pcVPS->getNumOlsHrdParamsMinus1(); i++ )
    {
      if( !pcVPS->getAllLayersSameNumSublayersFlag() )
      {
        READ_CODE( 3, uiCode, "vps_hrd_max_tid[i]" );                      pcVPS->setHrdMaxTid(i, uiCode );
        CHECK( uiCode > ( pcVPS->getMaxSubLayers() - 1 ), "The value of vps_hrd_max_tid[i] shall be in the range of 0 to vps_max_sublayers_minus1, inclusive." );
      }
      else
      {
        pcVPS->setHrdMaxTid( i, pcVPS->getMaxSubLayers() - 1 );
      }
      uint32_t firstSublayer = pcVPS->getVPSSublayerCpbParamsPresentFlag() ? 0 : pcVPS->getHrdMaxTid( i );
      parseOlsHrdParameters( pcVPS->getGeneralHrdParameters(), pcVPS->getOlsHrdParameters( i ), firstSublayer, pcVPS->getHrdMaxTid( i ) );
    }
    for( int i = pcVPS->getNumOlsHrdParamsMinus1() + 1; i < pcVPS->getTotalNumOLSs(); i++ )
    {
      pcVPS->setHrdMaxTid( i, pcVPS->getMaxSubLayers() - 1 );
    }
    for( int i = 0; i < pcVPS->m_numMultiLayeredOlss; i++ )
    {
      if( ( ( pcVPS->getNumOlsHrdParamsMinus1() + 1 ) != pcVPS->m_numMultiLayeredOlss ) && ( pcVPS->getNumOlsHrdParamsMinus1() > 0 ) )
      {
        READ_UVLC( uiCode, "vps_ols_hrd_idx[i]" );                         pcVPS->setOlsHrdIdx( i, uiCode );
        CHECK( uiCode > pcVPS->getNumOlsHrdParamsMinus1(), "The value of ols_hrd_idx[[i] shall be in the range of 0 to num_ols_hrd_params_minus1, inclusive." );
      }
      else if( pcVPS->getNumOlsHrdParamsMinus1() == 0 )
      {
        pcVPS->setOlsHrdIdx( i, 0 );
      }
      else
      {
        pcVPS->setOlsHrdIdx( i, i );
      }
      isHRDParamReferred[pcVPS->getOlsHrdIdx( i )] = true;
    }
    for( int i = 0; i <= pcVPS->getNumOlsHrdParamsMinus1(); i++ )
    {
      CHECK( !isHRDParamReferred[i], "Each ols_hrd_parameters( ) syntax structure in the VPS shall be referred to by at least one value of vps_ols_hrd_idx[i] for i in the range of 1 to NumMultiLayerOlss - 1, inclusive");
    }
  }
  else
  {
    for( int i = 0; i < pcVPS->getTotalNumOLSs(); i++ )
    {
      pcVPS->setHrdMaxTid( i, pcVPS->getMaxSubLayers() - 1 );
    }
  }


  READ_FLAG( uiCode, "vps_extension_flag" );
  if( uiCode )
  {
    while( xMoreRbspData() )
    {
      READ_FLAG( uiCode, "vps_extension_data_flag" );
    }
  }
  pcVPS->checkVPS();
  xReadRbspTrailingBits();
}
#endif

void HLSyntaxReader::parsePictureHeader( PicHeader* picHeader, const ParameterSetManager *parameterSetManager, bool readRbspTrailingBits )
{
#if ENABLE_TRACING
  xTracePictureHeader();
#endif

  X_READ_FLAG( ph_gdr_or_irap_pic_flag );
  picHeader->setGdrOrIrapPicFlag( ph_gdr_or_irap_pic_flag );

  X_READ_FLAG( ph_non_ref_pic_flag );
  picHeader->setNonReferencePictureFlag( ph_non_ref_pic_flag );

  if( picHeader->getGdrOrIrapPicFlag() )
  {
    X_READ_FLAG( ph_gdr_pic_flag );
    picHeader->setGdrPicFlag( ph_gdr_pic_flag );
  }

  X_READ_FLAG( ph_inter_slice_allowed_flag );
  picHeader->setPicInterSliceAllowedFlag( ph_inter_slice_allowed_flag );

  if( ph_inter_slice_allowed_flag )
  {
    X_READ_FLAG( ph_intra_slice_allowed_flag );
    picHeader->setPicIntraSliceAllowedFlag( ph_intra_slice_allowed_flag );
  }
  CHECK( picHeader->getPicInterSliceAllowedFlag() == 0 && picHeader->getPicIntraSliceAllowedFlag() == 0,
                     "Invalid picture without intra or inter slice" );

  // parameter sets
  X_READ_UVLC( ph_pic_parameter_set_id, 0, 63 );
  picHeader->setPPSId( ph_pic_parameter_set_id );

  const PPS* pps = parameterSetManager->getPPS( picHeader->getPPSId() );
  CHECK( !pps, "Invalid PPS" );

  picHeader->setSPSId( pps->getSPSId() );
  const SPS* sps = parameterSetManager->getSPS( picHeader->getSPSId() );
  CHECK( !sps, "Invalid SPS" );

  const unsigned CtbSizeY          = sps->getCTUSize();
  const unsigned CtbLog2SizeY      = getLog2( CtbSizeY );
  const unsigned MinCbLog2SizeY    = sps->getLog2MinCodingBlockSize();
  const unsigned MaxPicOrderCntLsb = 1 << sps->getBitsForPOC();

  X_READ_CODE_NO_RANGE( ph_pic_order_cnt_lsb, sps->getBitsForPOC() );
  picHeader->setPocLsb( ph_pic_order_cnt_lsb );

  if( picHeader->getGdrPicFlag() )
  {
    X_READ_UVLC( ph_recovery_poc_cnt, 0, MaxPicOrderCntLsb );
    picHeader->setRecoveryPocCnt( ph_recovery_poc_cnt );
  }
  else
  {
    picHeader->setRecoveryPocCnt( -1 );
  }

  const std::vector<bool>& phExtraBitsPresent = sps->getExtraPHBitPresentFlags();
  for( int i = 0; i < sps->getNumExtraPHBitsBytes() * 8; i++ )
  {
    // extra bits are ignored (when present)
    if( phExtraBitsPresent[i] )
    {
      X_READ_FLAG_idx( ph_extra_bit, "[i]" );
      (void) ph_extra_bit;
    }
  }

  if( sps->getPocMsbFlag() )
  {
    X_READ_FLAG( ph_poc_msb_cycle_present_flag );
    picHeader->setPocMsbPresentFlag( ph_poc_msb_cycle_present_flag );

    if( ph_poc_msb_cycle_present_flag )
    {
      X_READ_CODE_NO_RANGE( ph_poc_msb_cycle_val, sps->getPocMsbLen() );
      picHeader->setPocMsbVal( ph_poc_msb_cycle_val );
    }
  }

  // alf enable flags and aps IDs
  if( sps->getUseALF() && pps->getAlfInfoInPhFlag() )
  {
    X_READ_FLAG( ph_alf_enabled_flag );
    picHeader->setAlfEnabledFlag( COMPONENT_Y, ph_alf_enabled_flag );

    if( ph_alf_enabled_flag )
    {
      X_READ_CODE( ph_num_alf_aps_ids_luma, 3, 0, MAX_NUM_ALF_APS_IDS );
      picHeader->setNumAlfAps( ph_num_alf_aps_ids_luma );

      // auto* alfAPSs = parameterSetManager->getAlfAPSs();

      AlfApsIdVec apsId( ph_num_alf_aps_ids_luma, -1 );
      for( int i = 0; i < ph_num_alf_aps_ids_luma; i++ )
      {
        X_READ_CODE_NO_RANGE_idx( ph_alf_aps_id_luma, "[i]", 3 );
        apsId[i] = ph_alf_aps_id_luma;

        // auto* alfAPS = parameterSetManager->getAPS( ALF_APS, ph_alf_aps_id_luma );
        // CHECK( !alfAPS, "ALF APS with ph_alf_aps_id_luma == " << ph_alf_aps_id_luma << " not available.");
        // CHECK( alfAPS->getAlfAPSParam().newFilterFlag[CHANNEL_TYPE_LUMA] != 1,
        //        "The value of alf_luma_filter_signal_flag of the APS NAL unit having aps_params_type equal to ALF_APS "
        //        " and aps_adaptation_parameter_set_id equal to ph_alf_aps_id_luma[ i ] shall be equal to 1." );
        // /*CHECK( parameterSetManager->getAPS( ALF_APS, ph_alf_aps_id_luma )->getTemporalId() > todo: don't know curr temp ID
        //        "The TemporalId of the APS NAL unit having aps_params_type equal to ALF_APS and aps_adaptation_parameter_set_id"
        //        " equal to ph_alf_aps_id_luma[ i ] shall be less than or equal to the TemporalId of the current picture." );*/
        // CHECK( sps->getChromaFormatIdc() == 0 && alfAPS->chromaPresentFlag != 0,
        //        "When sps_chroma_format_idc is equal to 0, the value of aps_chroma_present_flag of the APS NAL unit having aps_params_type"
        //        " equal to ALF_APS and aps_adaptation_parameter_set_id equal to ph_alf_aps_id_luma[ i ] shall be equal to 0." );
        // CHECK( sps->getUseCCALF() == 0
        //          && ( alfAPS->getCcAlfAPSParam().newCcAlfFilter[COMPONENT_Cb - 1] != 0 || alfAPS->getCcAlfAPSParam().newCcAlfFilter[COMPONENT_Cr - 1] != 0 ),
        //        "When sps_ccalf_enabled_flag is equal to 0, the values of alf_cc_cb_filter_signal_flag and alf_cc_cr_filter_signal_flag of the APS NAL unit "
        //        " having aps_params_type equal to ALF_APS and aps_adaptation_parameter_set_id equal to ph_alf_aps_id_luma[ i ] shall be equal to 0." );
      }
      picHeader->setAlfAPSIds( apsId );

      if( sps->getChromaFormatIdc() != CHROMA_400 )
      {
        X_READ_FLAG( ph_alf_cb_enabled_flag );
        picHeader->setAlfEnabledFlag( COMPONENT_Cb, ph_alf_cb_enabled_flag );

        X_READ_FLAG( ph_alf_cr_enabled_flag );
        picHeader->setAlfEnabledFlag( COMPONENT_Cr, ph_alf_cr_enabled_flag );
      }

      if( picHeader->getAlfEnabledFlag( COMPONENT_Cb ) || picHeader->getAlfEnabledFlag( COMPONENT_Cr ) )
      {
        X_READ_CODE_NO_RANGE( ph_alf_aps_id_chroma, 3 );
        picHeader->setAlfApsIdChroma( ph_alf_aps_id_chroma );

        // auto* alfAPS = parameterSetManager->getAPS( ALF_APS, ph_alf_aps_id_chroma );
        // CHECK( !alfAPS, "ALF APS with ph_alf_aps_id_chroma == " << ph_alf_aps_id_chroma << " not available.");

        // CHECK( alfAPS->getCcAlfAPSParam().newCcAlfFilter[CHANNEL_TYPE_CHROMA] != 1,
        //        "The value of alf_chroma_filter_signal_flag of the APS NAL unit having aps_params_type equal to ALF_APS and"
        //        " aps_adaptation_parameter_set_id equal to ph_alf_aps_id_chroma shall be equal to 1." );
        // // The TemporalId of the APS NAL unit having aps_params_type equal to ALF_APS and
        // // aps_adaptation_parameter_set_id equal to ph_alf_aps_id_chroma shall be less than or equal to the TemporalId of the
        // // current picture.

        // CHECK( sps->getUseCCALF() == 0
        //          && ( alfAPS->getCcAlfAPSParam().newCcAlfFilter[COMPONENT_Cb - 1] != 0 || alfAPS->getCcAlfAPSParam().newCcAlfFilter[COMPONENT_Cr - 1] != 0 ),
        //        "When sps_ccalf_enabled_flag is equal to 0, the values of alf_cc_cb_filter_signal_flag and alf_cc_cr_filter_signal_flag of the APS NAL unit "
        //        " having aps_params_type equal to ALF_APS and aps_adaptation_parameter_set_id equal to ph_alf_aps_id_chroma shall be equal to 0." );
      }

      if( sps->getUseCCALF() )
      {
        X_READ_FLAG( ph_alf_cc_cb_enabled_flag );
        picHeader->setCcAlfEnabledFlag( COMPONENT_Cb, ph_alf_cc_cb_enabled_flag );

        if( ph_alf_cc_cb_enabled_flag )
        {
          X_READ_CODE_NO_RANGE( ph_alf_cc_cb_aps_id, 3 );
          picHeader->setCcAlfCbApsId( ph_alf_cc_cb_aps_id );

          // auto* alfAPS = parameterSetManager->getAPS( ALF_APS, ph_alf_cc_cb_aps_id );
          // CHECK( !alfAPS, "ALF APS with ph_alf_cc_cb_aps_id == " << ph_alf_cc_cb_aps_id << " not available.");
          // CHECK( alfAPS->getCcAlfAPSParam().newCcAlfFilter[COMPONENT_Cb - 1] != 1,
          //        "The value of alf_cc_cb_filter_signal_flag of the APS NAL unit having aps_params_type equal to ALF_APS and"
          //        " aps_adaptation_parameter_set_id equal to ph_alf_cc_cb_aps_id shall be equal to 1." );
          // // The TemporalId of the APS NAL unit having aps_params_type equal to ALF_APS and
          // // aps_adaptation_parameter_set_id equal to ph_alf_cc_cb_aps_id shall be less than or equal to the TemporalId of the
          // // current picture.
        }

        X_READ_FLAG( ph_alf_cc_cr_enabled_flag );
        picHeader->setCcAlfEnabledFlag( COMPONENT_Cr, ph_alf_cc_cr_enabled_flag );

        if( ph_alf_cc_cr_enabled_flag )
        {
          X_READ_CODE_NO_RANGE( ph_alf_cc_cr_aps_id, 3 );
          picHeader->setCcAlfCrApsId( ph_alf_cc_cr_aps_id );

          // auto* alfAPS = parameterSetManager->getAPS( ALF_APS, ph_alf_cc_cr_aps_id );
          // CHECK( !alfAPS, "ALF APS with ph_alf_cc_cr_aps_id == " << ph_alf_cc_cr_aps_id << " not available." );
          // CHECK( alfAPS->getCcAlfAPSParam().newCcAlfFilter[COMPONENT_Cr - 1] != 1,
          //        "The value of alf_cc_cr_filter_signal_flag of the APS NAL unit having aps_params_type equal to ALF_APS and"
          //        " aps_adaptation_parameter_set_id equal to ph_alf_cc_cr_aps_id shall be equal to 1." );
          // // The TemporalId of the APS NAL unit having aps_params_type equal to ALF_APS and
          // // aps_adaptation_parameter_set_id equal to ph_alf_cc_cr_aps_id shall be less than or equal to the TemporalId of the
          // // current picture.
        }
      }
    }   // if( ph_alf_enabled_flag )
  }     // if( sps->getUseALF() && pps->getAlfInfoInPhFlag() )

  // luma mapping / chroma scaling controls
  if( sps->getUseReshaper() )
  {
    X_READ_FLAG( ph_lmcs_enabled_flag );
    picHeader->setLmcsEnabledFlag( ph_lmcs_enabled_flag );

    if( ph_lmcs_enabled_flag )
    {
      X_READ_CODE_NO_RANGE( ph_lmcs_aps_id, 2 );
      picHeader->setLmcsAPSId( ph_lmcs_aps_id );

      if( sps->getChromaFormatIdc() != CHROMA_400 )
      {
        X_READ_FLAG( ph_chroma_residual_scale_flag );
        picHeader->setLmcsChromaResidualScaleFlag( ph_chroma_residual_scale_flag );
      }
    }
  }

  // quantization scaling lists
  if( sps->getScalingListFlag() )
  {
    X_READ_FLAG( ph_explicit_scaling_list_enabled_flag );
    picHeader->setExplicitScalingListEnabledFlag( ph_explicit_scaling_list_enabled_flag );

    if( ph_explicit_scaling_list_enabled_flag )
    {
      X_READ_CODE_NO_RANGE( ph_scaling_list_aps_id, 3 );
      picHeader->setScalingListAPSId( ph_scaling_list_aps_id );
    }
  }

  // virtual boundaries
  if( sps->getVirtualBoundariesEnabledFlag() && !sps->getVirtualBoundariesPresentFlag() )
  {
    X_READ_FLAG( ph_virtual_boundaries_present_flag );
    picHeader->setVirtualBoundariesPresentFlag( ph_virtual_boundaries_present_flag );

    if( ph_virtual_boundaries_present_flag )
    {
      X_READ_UVLC( ph_num_ver_virtual_boundaries, 0, pps->getPicWidthInLumaSamples() <= 8 ? 0 : 3 );
      picHeader->setNumVerVirtualBoundaries( ph_num_ver_virtual_boundaries );

      for( unsigned i = 0, prevBoundPos = 0; i < ph_num_ver_virtual_boundaries; i++ )
      {
        X_READ_UVLC_idx( ph_virtual_boundary_pos_x_minus1, "[ i ]", 0, ( pps->getPicWidthInLumaSamples() + 7 ) / 8 - 2 );

        const unsigned virtBoundPosX = ( ph_virtual_boundary_pos_x_minus1 + 1 ) << 3;
        CHECK( i > 0 && virtBoundPosX < prevBoundPos + CtbSizeY,
               "The distance between any two vertical virtual boundaries shall be greater than or equal to CtbSizeY luma samples." );
        picHeader->setVirtualBoundariesPosX( virtBoundPosX, i );

        prevBoundPos = virtBoundPosX;
      }

      X_READ_UVLC( ph_num_hor_virtual_boundaries, 0, pps->getPicHeightInLumaSamples() <= 8 ? 0 : 3 );
      picHeader->setNumHorVirtualBoundaries( ph_num_hor_virtual_boundaries );

      for( unsigned i = 0, prevBoundPos = 0; i < ph_num_hor_virtual_boundaries; i++ )
      {
        X_READ_UVLC_idx( ph_virtual_boundary_pos_y_minus1, "[ i ]", 0, ( pps->getPicHeightInLumaSamples() + 7 ) / 8 - 2 );

        const unsigned virtBoundPosY = ( ph_virtual_boundary_pos_y_minus1 + 1 ) << 3;
        CHECK( i > 0 && virtBoundPosY < prevBoundPos + CtbSizeY,
               "The distance between any two horizontal virtual boundaries shall be greater than or equal to CtbSizeY luma samples." );
        picHeader->setVirtualBoundariesPosY( virtBoundPosY, i );

        prevBoundPos = virtBoundPosY;
      }
      CHECK( ph_num_ver_virtual_boundaries + ph_num_hor_virtual_boundaries <= 0,
             "When sps_virtual_boundaries_enabled_flag is equal to 1 and ph_virtual_boundaries_present_flag is equal to 1, the sum"
             " of ph_num_ver_virtual_boundaries and ph_num_hor_virtual_boundaries shall be greater than 0." );
    }
  }
  else if( sps->getVirtualBoundariesPresentFlag() )
  {
    picHeader->setVirtualBoundariesPresentFlag( true );
    picHeader->setNumVerVirtualBoundaries( sps->getNumVerVirtualBoundaries() );
    picHeader->setNumHorVirtualBoundaries( sps->getNumHorVirtualBoundaries() );

    for( unsigned i = 0, prevBoundPosX = 0, prevBoundPosY = 0; i < 3; i++ )
    {
      picHeader->setVirtualBoundariesPosX( sps->getVirtualBoundariesPosX( i ), i );
      picHeader->setVirtualBoundariesPosY( sps->getVirtualBoundariesPosY( i ), i );

      CHECK( i > 0 && i < picHeader->getNumVerVirtualBoundaries() && picHeader->getVirtualBoundariesPosX( i ) < prevBoundPosX + CtbSizeY,
             "The distance between any two vertical virtual boundaries shall be greater than or equal to CtbSizeY luma samples." );
      CHECK( i > 0 && i < picHeader->getNumHorVirtualBoundaries() && picHeader->getVirtualBoundariesPosY( i ) < prevBoundPosY + CtbSizeY,
             "The distance between any two horizontal virtual boundaries shall be greater than or equal to CtbSizeY luma samples." );

      prevBoundPosX = picHeader->getVirtualBoundariesPosX( i );
      prevBoundPosY = picHeader->getVirtualBoundariesPosY( i );
    }
  }

  // picture output flag
  if( pps->getOutputFlagPresentFlag() && !picHeader->getNonReferencePictureFlag() )
  {
    X_READ_FLAG( ph_pic_output_flag );
    picHeader->setPicOutputFlag( ph_pic_output_flag);
  }

  // reference picture lists
  if( pps->getRplInfoInPhFlag() )
  {
    parsePicOrSliceHeaderRPL( picHeader, sps, pps );
  }

  // partitioning constraint overrides
  if( sps->getSplitConsOverrideEnabledFlag() )
  {
    X_READ_FLAG( ph_partition_constraints_override_flag );
    picHeader->setSplitConsOverrideFlag( ph_partition_constraints_override_flag );
  }

  // inherit constraint values from SPS
  PartitionConstraints minQT     = sps->getMinQTSizes();
  PartitionConstraints maxBTD    = sps->getMaxMTTHierarchyDepths();
  PartitionConstraints maxBTSize = sps->getMaxBTSizes();
  PartitionConstraints maxTTSize = sps->getMaxTTSizes();

  if( picHeader->getPicIntraSliceAllowedFlag() )
  {
    unsigned MinQtLog2SizeIntraY = getLog2( minQT[0] );
    if( picHeader->getSplitConsOverrideFlag() )
    {
      X_READ_UVLC( ph_log2_diff_min_qt_min_cb_intra_slice_luma, 0, std::min( 6u, CtbLog2SizeY ) - MinCbLog2SizeY );
      MinQtLog2SizeIntraY = ph_log2_diff_min_qt_min_cb_intra_slice_luma + MinCbLog2SizeY;
      minQT[0] = 1 << MinQtLog2SizeIntraY;

      X_READ_UVLC( ph_max_mtt_hierarchy_depth_intra_slice_luma, 0, 2 * ( CtbLog2SizeY - MinCbLog2SizeY ) );
      maxBTD[0] = ph_max_mtt_hierarchy_depth_intra_slice_luma;
      // TODO: When not present, the value of ph_max_mtt_hierarchy_depth_intra_slice_luma is inferred to be equal to sps_max_mtt_hierarchy_depth_intra_slice_luma.

      maxTTSize[0] = maxBTSize[0] = minQT[0];
      if( maxBTD[0] != 0 )
      {
        X_READ_UVLC( ph_log2_diff_max_bt_min_qt_intra_slice_luma,
                     0, ( sps->getUseDualITree() ? std::min( 6u, CtbLog2SizeY ) : CtbLog2SizeY ) - MinQtLog2SizeIntraY );
        maxBTSize[0] <<= ph_log2_diff_max_bt_min_qt_intra_slice_luma;
        // TODO: When not present, the value of ph_log2_diff_max_bt_min_qt_intra_slice_luma is inferred to be equal to
        //       sps_log2_diff_max_bt_min_qt_intra_slice_luma.

        X_READ_UVLC( ph_log2_diff_max_tt_min_qt_intra_slice_luma, 0, std::min( 6u, CtbLog2SizeY ) - MinQtLog2SizeIntraY );
        maxTTSize[0] <<= ph_log2_diff_max_tt_min_qt_intra_slice_luma;
        // TODO: When not present, the value of ph_log2_diff_max_tt_min_qt_intra_slice_luma is inferred to be equal to
        //       sps_log2_diff_max_tt_min_qt_intra_slice_luma.
      }
      if( sps->getUseDualITree() )
      {
        X_READ_UVLC( ph_log2_diff_min_qt_min_cb_intra_slice_chroma, 0, std::min( 6u, CtbLog2SizeY ) - MinCbLog2SizeY );
        unsigned MinQtLog2SizeIntraC = ph_log2_diff_min_qt_min_cb_intra_slice_chroma + MinCbLog2SizeY;
        minQT[2] = 1 << MinQtLog2SizeIntraC;
        // TODO: When not present, the value of ph_log2_diff_min_qt_min_cb_intra_slice_chroma is inferred to be equal to
        //       sps_log2_diff_min_qt_min_cb_intra_slice_chroma.

        X_READ_UVLC( ph_max_mtt_hierarchy_depth_intra_slice_chroma, 0, 2 * ( CtbLog2SizeY - MinCbLog2SizeY ) );
        maxBTD[2]    = ph_max_mtt_hierarchy_depth_intra_slice_chroma;
        // TODO: When not present, the value of ph_max_mtt_hierarchy_depth_intra_slice_chroma is inferred to be equal to
        //       sps_max_mtt_hierarchy_depth_intra_slice_chroma.

        maxTTSize[2] = maxBTSize[2] = minQT[2];
        if( ph_max_mtt_hierarchy_depth_intra_slice_chroma )
        {
          X_READ_UVLC( ph_log2_diff_max_bt_min_qt_intra_slice_chroma, 0, std::min( 6u, CtbLog2SizeY ) - MinQtLog2SizeIntraC );
          maxBTSize[2] <<= ph_log2_diff_max_bt_min_qt_intra_slice_chroma;
          // TODO: When not present, the value of ph_log2_diff_max_bt_min_qt_intra_slice_chroma is inferred to be equal to
          //       sps_log2_diff_max_bt_min_qt_intra_slice_chroma.

          X_READ_UVLC( ph_log2_diff_max_tt_min_qt_intra_slice_chroma, 0, std::min( 6u, CtbLog2SizeY ) - MinQtLog2SizeIntraC );
          maxTTSize[2] <<= ph_log2_diff_max_tt_min_qt_intra_slice_chroma;
          // TODO: When not present, the value of ph_log2_diff_max_tt_min_qt_intra_slice_chroma is inferred to be equal to
          //       sps_log2_diff_max_tt_min_qt_intra_slice_chroma.
        }
      }   // if( sps->getUseDualITree() )
    }     // if( picHeader->getSplitConsOverrideFlag() )

    // delta quantization and chrom and chroma offset
    if( pps->getUseDQP() )
    {
      X_READ_UVLC( ph_cu_qp_delta_subdiv_intra_slice,
                   0, 2 * ( CtbLog2SizeY - MinQtLog2SizeIntraY + maxBTD[0] /*ph_max_mtt_hierarchy_depth_intra_slice_luma*/ ) );
      picHeader->setCuQpDeltaSubdivIntra( ph_cu_qp_delta_subdiv_intra_slice );
    }

    if( pps->getCuChromaQpOffsetEnabledFlag() )
    {
      X_READ_UVLC( ph_cu_chroma_qp_offset_subdiv_intra_slice,
                   0, 2 * ( CtbLog2SizeY - MinQtLog2SizeIntraY + maxBTD[0] /*ph_max_mtt_hierarchy_depth_intra_slice_luma*/ ) );
      picHeader->setCuChromaQpOffsetSubdivIntra( ph_cu_chroma_qp_offset_subdiv_intra_slice );
    }
  }   // if( picHeader->getPicIntraSliceAllowedFlag() )

  if( ph_inter_slice_allowed_flag )
  {
    unsigned MinQtLog2SizeInterY = MinCbLog2SizeY;
    if( picHeader->getSplitConsOverrideFlag() )
    {
      X_READ_UVLC( ph_log2_diff_min_qt_min_cb_inter_slice, 0, std::min( 6u, CtbLog2SizeY ) - MinCbLog2SizeY );
      MinQtLog2SizeInterY += ph_log2_diff_min_qt_min_cb_inter_slice;
      minQT[1] = 1 << MinQtLog2SizeInterY;

      X_READ_UVLC( ph_max_mtt_hierarchy_depth_inter_slice, 0, 2 * ( CtbLog2SizeY - MinCbLog2SizeY ) );
      maxBTD[1] = ph_max_mtt_hierarchy_depth_inter_slice;

      maxTTSize[1] = maxBTSize[1] = minQT[1];
      if( maxBTD[1] != 0 )
      {
        X_READ_UVLC( ph_log2_diff_max_bt_min_qt_inter_slice, 0, CtbLog2SizeY - MinQtLog2SizeInterY );
        maxBTSize[1] <<= ph_log2_diff_max_bt_min_qt_inter_slice;

        X_READ_UVLC( ph_log2_diff_max_tt_min_qt_inter_slice, 0, std::min( 6u, CtbLog2SizeY ) - MinQtLog2SizeInterY );
        maxTTSize[1] <<= ph_log2_diff_max_tt_min_qt_inter_slice;
      }
    }

    // delta quantization and chrom and chroma offset
    if( pps->getUseDQP() )
    {
      X_READ_UVLC( ph_cu_qp_delta_subdiv_inter_slice, 0, 2 * ( CtbLog2SizeY - MinQtLog2SizeInterY + maxBTD[1] /*ph_max_mtt_hierarchy_depth_inter_slice*/ ) );
      picHeader->setCuQpDeltaSubdivInter( ph_cu_qp_delta_subdiv_inter_slice );
    }

    if( pps->getCuChromaQpOffsetEnabledFlag() )
    {
      X_READ_UVLC( ph_cu_chroma_qp_offset_subdiv_inter_slice,
                   0, 2 * ( CtbLog2SizeY - MinQtLog2SizeInterY + maxBTD[1] /*ph_max_mtt_hierarchy_depth_inter_slice*/ ) );
      picHeader->setCuChromaQpOffsetSubdivInter( ph_cu_chroma_qp_offset_subdiv_inter_slice );
    }

    // temporal motion vector prediction
    if( sps->getSPSTemporalMVPEnabledFlag() )
    {
      X_READ_FLAG( ph_temporal_mvp_enabled_flag );
      picHeader->setEnableTMVPFlag( ph_temporal_mvp_enabled_flag );

      if( ph_temporal_mvp_enabled_flag && pps->getRplInfoInPhFlag() )
      {
        if( picHeader->getRPL( REF_PIC_LIST_1 )->getNumRefEntries() > 0 )
        {
          X_READ_FLAG( ph_collocated_from_l0_flag );
          picHeader->setPicColFromL0Flag( ph_collocated_from_l0_flag );
        }
        else   // ( num_ref_entries[ 1 ][ RplsIdx[ 1 ] ] == 0 )
        {
          // When ph_temporal_mvp_enabled_flag and pps_rpl_info_in_ph_flag are both equal to 1 and num_ref_entries[ 1 ][ RplsIdx[ 1 ] ] is equal to 0, the value
          // of ph_collocated_from_l0_flag is inferred to be equal to 1.
          picHeader->setPicColFromL0Flag( 1 );
        }

        if( ( picHeader->getPicColFromL0Flag() && picHeader->getRPL( REF_PIC_LIST_0 )->getNumRefEntries() > 1 )
            || ( !picHeader->getPicColFromL0Flag() && picHeader->getRPL( REF_PIC_LIST_1 )->getNumRefEntries() > 1 ) )
        {
          X_READ_UVLC_NO_RANGE( ph_collocated_ref_idx );
          if( picHeader->getPicColFromL0Flag() )
          {
            CHECK( ph_collocated_ref_idx > picHeader->getRPL( REF_PIC_LIST_0 )->getNumRefEntries() - 1,
                   "When ph_collocated_from_l0_flag is equal to 1, ph_collocated_ref_idx refers to an entry in RPL 0, and the value of ph_collocated_ref_idx"
                   " shall be in the range of 0 to num_ref_entries[ 0 ][ RplsIdx[ 0 ] ] − 1, inclusive." )
          }
          else
          {
            CHECK( ph_collocated_ref_idx > picHeader->getRPL( REF_PIC_LIST_1 )->getNumRefEntries() - 1,
                   "When ph_collocated_from_l0_flag is equal to 0, ph_collocated_ref_idx refers to an entry in RPL 1, and the value of ph_collocated_ref_idx"
                   " shall be in the range of 0 to num_ref_entries[ 1 ][ RplsIdx[ 1 ] ] − 1, inclusive." );
          }
          picHeader->setColRefIdx( ph_collocated_ref_idx );
        }
      }
    }

    // merge candidate list size
    // subblock merge candidate list size
    if( sps->getUseAffine() )
    {
      picHeader->setMaxNumAffineMergeCand( sps->getMaxNumAffineMergeCand() );
    }
    else
    {
      picHeader->setMaxNumAffineMergeCand( sps->getSBTMVPEnabledFlag() && picHeader->getEnableTMVPFlag() );
    }

    // full-pel MMVD flag
    if( sps->getFpelMmvdEnabledFlag() )
    {
      X_READ_FLAG( ph_fpel_mmvd_enabled_flag );
      picHeader->setDisFracMMVD( ph_fpel_mmvd_enabled_flag );
    }

    bool presenceFlag = 0;
    if( !pps->getRplInfoInPhFlag() )   // This condition is intentionally not merged into the next, to avoid possible interpretation of RplsIdx[ i ] not having
    {                                  // a specified value.
      presenceFlag = 1;
    }
    else if( picHeader->getRPL( REF_PIC_LIST_1 )->getNumRefEntries() > 0 )
    {
      presenceFlag = 1;
    }

    // If sps_bdof_control_present_in_ph_flag is equal to 0, the value of ph_bdof_disabled_flag is inferred to be equal to 1 - sps_bdof_enabled_flag
    // Otherwise (sps_bdof_control_present_in_ph_flag is equal to 1), the value of ph_bdof_disabled_flag is inferred to be  equal to 1.
    picHeader->setDisBdofFlag( sps->getBdofControlPresentInPhFlag() == 0 ? 1 - sps->getUseBIO() : 1 );

    // If sps_dmvr_control_present_in_ph_flag is equal to 0, the value of ph_dmvr_disabled_flag is inferred to be equal to 1 - sps_dmvr_enabled_flag.
    // Otherwise (sps_dmvr_control_present_in_ph_flag is equal to 1), the value of ph_dmvr_disabled_flag is inferred to be equal to 1.
    picHeader->setDisDmvrFlag( sps->getDmvrControlPresentInPhFlag() == 0 ? 1 - sps->getUseDMVR() : 1 );

    if( presenceFlag )
    {
      // mvd L1 zero flag
      X_READ_FLAG( ph_mvd_l1_zero_flag );
      picHeader->setMvdL1ZeroFlag( ph_mvd_l1_zero_flag );

      // picture level BDOF disable flags
      if( sps->getBdofControlPresentInPhFlag() )
      {
        X_READ_FLAG( ph_bdof_disabled_flag );
        picHeader->setDisBdofFlag( ph_bdof_disabled_flag );
      }

      // picture level DMVR disable flags
      if( sps->getDmvrControlPresentInPhFlag() )
      {
        X_READ_FLAG( ph_dmvr_disabled_flag );
        picHeader->setDisDmvrFlag( ph_dmvr_disabled_flag );
      }
    }

    // picture level PROF disable flags
    if( sps->getProfControlPresentInPhFlag() )
    {
      X_READ_FLAG( ph_prof_disabled_flag );
      picHeader->setDisProfFlag( ph_prof_disabled_flag );
    }
    else
    {
      // If sps_affine_prof_enabled_flag is equal to 1, the value of ph_prof_disabled_flag is inferred to be equal to 0.
      // Otherwise (sps_affine_prof_enabled_flag is equal to 0), the value of ph_prof_disabled_flag is inferred to be equal to 1.

      picHeader->setDisProfFlag( !sps->getUsePROF() );
    }

    if( ( pps->getUseWP() || pps->getWPBiPred() ) && pps->getWpInfoInPhFlag() )
    {
      parsePredWeightTable( picHeader, sps, pps );
    }
  }   // if( ph_inter_slice_allowed_flag )

  picHeader->setMinQTSizes( minQT );
  picHeader->setMaxMTTHierarchyDepths( maxBTD );
  picHeader->setMaxBTSizes( maxBTSize );
  picHeader->setMaxTTSizes( maxTTSize );

  // ibc merge candidate list size
  if( pps->getQpDeltaInfoInPhFlag() )
  {
   X_READ_SVLC_NO_RANGE( ph_qp_delta );
   int SliceQpY = 26 + pps->getPicInitQPMinus26() + ph_qp_delta;
   CHECK( SliceQpY < -sps->getQpBDOffset() || SliceQpY > 63, "The value of SliceQpY shall be in the range of −QpBdOffset to +63, inclusive." );
   picHeader->setQpDelta( ph_qp_delta );
  }

  // joint Cb/Cr sign flag
  if( sps->getJointCbCrEnabledFlag() )
  {
    X_READ_FLAG( ph_joint_cbcr_sign_flag );
    picHeader->setJointCbCrSignFlag( ph_joint_cbcr_sign_flag );
  }

  // sao enable flags
  if( sps->getUseSAO() && pps->getSaoInfoInPhFlag() )
  {
    X_READ_FLAG( ph_sao_luma_enabled_flag );
    picHeader->setSaoEnabledFlag( CHANNEL_TYPE_LUMA, ph_sao_luma_enabled_flag );

    if( sps->getChromaFormatIdc() != CHROMA_400 )
    {
      X_READ_FLAG( ph_sao_chroma_enabled_flag );
      picHeader->setSaoEnabledFlag( CHANNEL_TYPE_CHROMA, ph_sao_chroma_enabled_flag );
    }
  }

  // deblocking filter controls

  if( pps->getDbfInfoInPhFlag() )
  {
    X_READ_FLAG( ph_deblocking_params_present_flag );
    picHeader->setDeblockingFilterOverrideFlag( ph_deblocking_params_present_flag );
  }

  if( pps->getPPSDeblockingFilterDisabledFlag() == 1 && picHeader->getDeblockingFilterOverrideFlag() == 1 )
  {
    // If pps_deblocking_filter_disabled_flag and ph_deblocking_params_present_flag are both equal to 1, the value of ph_deblocking_filter_disabled_flag
    // is inferred to be equal to 0.
    picHeader->setDeblockingFilterDisable( false );
  }
  else // ( pps->getPPSDeblockingFilterDisabledFlag() == 0 || picHeader->getDeblockingFilterOverrideFlag() == 0 )
  {
    // Otherwise (pps_deblocking_filter_disabled_flag or ph_deblocking_params_present_flag is equal to 0), the value of ph_deblocking_filter_disabled_flag is
    // inferred to be equal to pps_deblocking_filter_disabled_flag.
    picHeader->setDeblockingFilterDisable( pps->getPPSDeblockingFilterDisabledFlag() );
  }

  // When not present, the values of ph_luma_beta_offset_div2 and ph_luma_tc_offset_div2 are inferred to be equal to pps_luma_beta_offset_div2 and
  // pps_luma_tc_offset_div2, respectively.
  picHeader->setDeblockingFilterBetaOffsetDiv2( pps->getDeblockingFilterBetaOffsetDiv2() );
  picHeader->setDeblockingFilterTcOffsetDiv2( pps->getDeblockingFilterTcOffsetDiv2() );

  if( picHeader->getDeblockingFilterOverrideFlag() /*ph_deblocking_params_present_flag*/ )
  {
    if( !pps->getPPSDeblockingFilterDisabledFlag() )
    {
      X_READ_FLAG( ph_deblocking_filter_disabled_flag );
      picHeader->setDeblockingFilterDisable( ph_deblocking_filter_disabled_flag );
    }

    if( !picHeader->getDeblockingFilterDisable() )
    {
      X_READ_SVLC( ph_luma_beta_offset_div2, -12, 12 );
      picHeader->setDeblockingFilterBetaOffsetDiv2( ph_luma_beta_offset_div2 );

      X_READ_SVLC( ph_luma_tc_offset_div2, -12, 12 );
      picHeader->setDeblockingFilterTcOffsetDiv2( ph_luma_tc_offset_div2 );
    }
  }

  if( picHeader->getDeblockingFilterOverrideFlag() /*ph_deblocking_params_present_flag*/
      && !picHeader->getDeblockingFilterDisable() && pps->getPPSChromaToolFlag() )
  {
    X_READ_SVLC( ph_cb_beta_offset_div2, -12, 12 );
    picHeader->setDeblockingFilterCbBetaOffsetDiv2( ph_cb_beta_offset_div2 );

    X_READ_SVLC( ph_cb_tc_offset_div2, -12, 12 );
    picHeader->setDeblockingFilterCbTcOffsetDiv2( ph_cb_tc_offset_div2 );

    X_READ_SVLC( ph_cr_beta_offset_div2, -12, 12 );
    picHeader->setDeblockingFilterCrBetaOffsetDiv2( ph_cr_beta_offset_div2 );

    X_READ_SVLC( ph_cr_tc_offset_div2, -12, 12 );
    picHeader->setDeblockingFilterCrTcOffsetDiv2( ph_cr_tc_offset_div2 );
  }
  else
  {
    // When not present, the values of ph_cb_beta_offset_div2 and ph_cb_tc_offset_div2 are inferred as follows:
    // If pps_chroma_tool_offsets_present_flag is equal to 1, the values of ph_cb_beta_offset_div2 and ph_cb_tc_offset_div2 are inferred to be equal to
    // pps_cb_beta_offset_div2 and pps_cb_tc_offset_div2, respectively.
    // Otherwise (pps_chroma_tool_offsets_present_flag is equal to 0), the values of ph_cb_beta_offset_div2 and ph_cb_tc_offset_div2 are inferred to be equal
    // to ph_luma_beta_offset_div2 and ph_luma_tc_offset_div2, respectively.
    picHeader->setDeblockingFilterCbBetaOffsetDiv2( pps->getPPSChromaToolFlag() ? pps->getDeblockingFilterCbBetaOffsetDiv2()
                                                                                : picHeader->getDeblockingFilterBetaOffsetDiv2() );
    picHeader->setDeblockingFilterCbTcOffsetDiv2( pps->getPPSChromaToolFlag() ? pps->getDeblockingFilterCbTcOffsetDiv2()
                                                                              : picHeader->getDeblockingFilterTcOffsetDiv2() );

    // When not present, the values of ph_cr_beta_offset_div2 and ph_cr_tc_offset_div2 are inferred as follows:
    // If pps_chroma_tool_offsets_present_flag is equal to 1, the values of ph_cr_beta_offset_div2 and ph_cr_tc_offset_div2 are inferred to be equal to
    // pps_cr_beta_offset_div2 and pps_cr_tc_offset_div2, respectively.
    // Otherwise (pps_chroma_tool_offsets_present_flag is equal to 0), the values of ph_cr_beta_offset_div2 and ph_cr_tc_offset_div2 are inferred to be equal
    // to ph_luma_beta_offset_div2 and ph_luma_tc_offset_div2, respectively.
    picHeader->setDeblockingFilterCrBetaOffsetDiv2( pps->getPPSChromaToolFlag() ? pps->getDeblockingFilterCrBetaOffsetDiv2()
                                                                                : picHeader->getDeblockingFilterBetaOffsetDiv2() );
    picHeader->setDeblockingFilterCrTcOffsetDiv2( pps->getPPSChromaToolFlag() ? pps->getDeblockingFilterCrTcOffsetDiv2()
                                                                              : picHeader->getDeblockingFilterTcOffsetDiv2() );
  }

  // picture header extension
  if( pps->getPictureHeaderExtensionPresentFlag() )
  {
    X_READ_UVLC( ph_extension_length, 0, 256 );
    for( unsigned i = 0; i < ph_extension_length; i++ )
    {
      X_READ_CODE_NO_RANGE_idx( ph_extension_data_byte, "[i]", 8 );
      (void) ph_extension_data_byte;
    }
  }

  if( readRbspTrailingBits )
  {
    xReadRbspTrailingBits();
  }
  picHeader->setValid();
}

void HLSyntaxReader::checkAlfNaluTidAndPicTid( const Slice* pcSlice, const PicHeader* picHeader, const ParameterSetManager *parameterSetManager )
{
  const SPS* sps = parameterSetManager->getSPS(picHeader->getSPSId());
  const PPS* pps = parameterSetManager->getPPS(picHeader->getPPSId());

  int  curPicTid = pcSlice->getTLayer();
  const APS* aps = nullptr;

  if( sps->getUseALF() && pps->getAlfInfoInPhFlag() && picHeader->getAlfEnabledFlag( COMPONENT_Y ) )
  {
    const auto& apsIds = picHeader->getAlfAPSIds();
    //luma
    for( int i = 0; i < picHeader->getNumAlfAps(); i++ )
    {
      aps = parameterSetManager->getAPS( apsIds[i], ALF_APS );
      CHECK( aps->getTemporalId() > curPicTid, "The TemporalId of the APS NAL unit having aps_params_type equal to ALF_APS and adaptation_parameter_set_id equal to ph_alf_aps_id_luma[i] shall be less than or equal to the TemporalId of the picture associated with the PH." );
      if( pcSlice->getNalUnitLayerId() != aps->getLayerId() )
      {
        CHECK( aps->getLayerId() > pcSlice->getNalUnitLayerId(), "Layer Id of APS cannot be greater than layer Id of VCL NAL unit the refer to it" );
        CHECK( sps->getVPSId() == 0, "VPSId of the referred SPS cannot be 0 when layer Id of APS and layer Id of current slice are different" );
        const VPS* vps = parameterSetManager->getVPS( sps->getVPSId() );
        for( int i = 0; i < vps->getNumOutputLayerSets(); i++ )
        {
          bool isCurrLayerInOls = false;
          bool isRefLayerInOls = false;
          for( int j = vps->getNumLayersInOls(i) - 1; j >= 0; j-- )
          {
            if( vps->getLayerIdInOls(i, j) == pcSlice->getNalUnitLayerId() )
            {
              isCurrLayerInOls = true;
            }
            if( vps->getLayerIdInOls(i, j) == aps->getLayerId() )
            {
              isRefLayerInOls = true;
            }
          }
          CHECK( isCurrLayerInOls && !isRefLayerInOls, "When VCL NAl unit in layer A refers to APS in layer B, all OLS that contains layer A shall also contains layer B" );
        }
      }
    }
    //chroma
    if( picHeader->getAlfEnabledFlag(COMPONENT_Cb) || picHeader->getAlfEnabledFlag( COMPONENT_Cr ) )
    {
      int chromaAlfApsId = picHeader->getAlfApsIdChroma();
      aps = parameterSetManager->getAPS( chromaAlfApsId, ALF_APS );
      CHECK( aps->getTemporalId() > curPicTid, "The TemporalId of the APS NAL unit having aps_params_type equal to ALF_APS and adaptation_parameter_set_id equal to ph_alf_aps_id_chroma shall be less than or equal to the TemporalId of the picture associated with the PH.") ;
      if( pcSlice->getNalUnitLayerId() != aps->getLayerId() )
      {
        CHECK( aps->getLayerId() > pcSlice->getNalUnitLayerId(), "Layer Id of APS cannot be greater than layer Id of VCL NAL unit the refer to it" );
        CHECK( sps->getVPSId() == 0, "VPSId of the referred SPS cannot be 0 when layer Id of APS and layer Id of current slice are different" );
        const VPS* vps = parameterSetManager->getVPS( sps->getVPSId() );
        for( int i = 0; i < vps->getNumOutputLayerSets(); i++ )
        {
          bool isCurrLayerInOls = false;
          bool isRefLayerInOls = false;
          for( int j = vps->getNumLayersInOls(i) - 1; j >= 0; j-- )
          {
            if( vps->getLayerIdInOls( i, j ) == pcSlice->getNalUnitLayerId() )
            {
              isCurrLayerInOls = true;
            }
            if( vps->getLayerIdInOls(i, j) == aps->getLayerId() )
            {
              isRefLayerInOls = true;
            }
          }
          CHECK( isCurrLayerInOls && !isRefLayerInOls, "When VCL NAl unit in layer A refers to APS in layer B, all OLS that contains layer A shall also contains layer B" );
        }
      }
    }
  }
}

void HLSyntaxReader::parseSliceHeader( Slice*                      pcSlice,
                                       std::shared_ptr<PicHeader>& picHeader,
                                       const ParameterSetManager*  parameterSetManager,
                                       const int                   prevTid0POC,
                                       bool&                       firstSliceInPic )
{
#if ENABLE_TRACING
  xTraceSliceHeader();
#endif

  X_READ_FLAG( sh_picture_header_in_slice_header_flag );
  pcSlice->setPictureHeaderInSliceHeader( sh_picture_header_in_slice_header_flag );

  if( sh_picture_header_in_slice_header_flag )
  {
    picHeader.reset( new PicHeader );
    parsePictureHeader( picHeader.get(), parameterSetManager, false );
  }
  CHECK( !picHeader, "Picture Header not allocated" );   // should always be allocated, even if it is not valid
  CHECK( !picHeader->isValid(), "Picture Header missing" );

  checkAlfNaluTidAndPicTid( pcSlice, picHeader.get(), parameterSetManager );

  const PPS* pps = parameterSetManager->getPPS( picHeader->getPPSId() );
  CHECK( pps == 0, "Invalid PPS" );
  const SPS* sps = parameterSetManager->getSPS( pps->getSPSId() );
  CHECK( sps == 0, "Invalid SPS" );

  auto gci = sps->getProfileTierLevel()->getConstraintInfo();
  CHECK_CONSTRAINT( gci->getPicHeaderInSliceHeaderConstraintFlag() && !sh_picture_header_in_slice_header_flag,
                    "PH shall be present in SH, when pic_header_in_slice_header_constraint_flag is equal to 1" );

  if( sh_picture_header_in_slice_header_flag )
  {
    CHECK( pps->getRplInfoInPhFlag() == 1, "When sh_picture_header_in_slice_header_flag is equal to 1, rpl_info_in_ph_flag shall be equal to 0" );
    CHECK( pps->getDbfInfoInPhFlag() == 1, "When sh_picture_header_in_slice_header_flag is equal to 1, dbf_info_in_ph_flag shall be equal to 0" );
    CHECK( pps->getSaoInfoInPhFlag() == 1, "When sh_picture_header_in_slice_header_flag is equal to 1, sao_info_in_ph_flag shall be equal to 0" );
    CHECK( pps->getAlfInfoInPhFlag() == 1, "When sh_picture_header_in_slice_header_flag is equal to 1, alf_info_in_ph_flag shall be equal to 0" );
    CHECK( pps->getWpInfoInPhFlag() == 1, "When sh_picture_header_in_slice_header_flag is equal to 1, wp_info_in_ph_flag shall be equal to 0" );
    CHECK( pps->getQpDeltaInfoInPhFlag() == 1,
                       "When sh_picture_header_in_slice_header_flag is equal to 1, qp_delta_info_in_ph_flag shall be equal to 0" );
    CHECK( sps->getSubPicInfoPresentFlag() == 1,
                       "When sps_subpic_info_present_flag is equal to 1, the value of sh_picture_header_in_slice_header_flag shall be equal to 0" );
  }
  CHECK( sps->getSubPicInfoPresentFlag() == 1 && sps->getVirtualBoundariesEnabledFlag() == 1 && sps->getVirtualBoundariesPresentFlag() == 0,
                     "when sps_subpic_info_present_flag is equal to 1 and sps_virtual_boundaries_enabled_flag is equal to 1, "
                     "sps_virtual_boundaries_present_flag shall be equal 1" );


  const bool bChroma = sps->getChromaFormatIdc() != CHROMA_400;

  // picture order count
  const int iPOClsb    = picHeader->getPocLsb();
  const int iMaxPOClsb = 1 << sps->getBitsForPOC();
  if( pcSlice->getIdrPicFlag() )
  {
    int iPOCmsb;
    if( picHeader->getPocMsbPresentFlag() )
    {
      iPOCmsb = picHeader->getPocMsbVal() * iMaxPOClsb;
    }
    else
    {
      iPOCmsb = 0;
    }
    pcSlice->setPOC( iPOCmsb + iPOClsb );
  }
  else
  {
    const int iPrevPOC    = prevTid0POC;
    const int iPrevPOClsb = iPrevPOC & ( iMaxPOClsb - 1 );
    const int iPrevPOCmsb = iPrevPOC - iPrevPOClsb;
    int       iPOCmsb;
    if( picHeader->getPocMsbPresentFlag() )
    {
      iPOCmsb = picHeader->getPocMsbVal() * iMaxPOClsb;
    }
    else
    {
      if( ( iPOClsb < iPrevPOClsb ) && ( ( iPrevPOClsb - iPOClsb ) >= ( iMaxPOClsb / 2 ) ) )
      {
        iPOCmsb = iPrevPOCmsb + iMaxPOClsb;
      }
      else if( ( iPOClsb > iPrevPOClsb ) && ( ( iPOClsb - iPrevPOClsb ) > ( iMaxPOClsb / 2 ) ) )
      {
        iPOCmsb = iPrevPOCmsb - iMaxPOClsb;
      }
      else
      {
        iPOCmsb = iPrevPOCmsb;
      }
    }
    pcSlice->setPOC( iPOCmsb + iPOClsb );
  }

  if( sps->getSubPicInfoPresentFlag() )
  {
    X_READ_CODE_NO_RANGE( sh_subpic_id, sps->getSubPicIdLen() );
    pcSlice->setSliceSubPicId( sh_subpic_id );
  }

  const unsigned NumTilesInPic = pps->getNumTiles();

  uint32_t sliceAddr = 0;
  if( !pps->getRectSliceFlag() )   // raster scan slices
  {
    if( NumTilesInPic > 1 )
    {
      // slice address is the raster scan tile index of first tile in slice
      const int bitsSliceAddress = (int) std::ceil( std::log2( NumTilesInPic ) );
      X_READ_CODE( sh_slice_address, bitsSliceAddress, 0, NumTilesInPic - 1 );
      sliceAddr = sh_slice_address;
    }
  }
  else   // rectangular slices
  {
    // slice address is the index of the slice within the current sub-picture
    const uint32_t currSubPicIdx         = pps->getSubPicIdxFromSubPicId( pcSlice->getSliceSubPicId() );
    const SubPic&  currSubPic            = pps->getSubPic( currSubPicIdx );
    const unsigned NumSlicesInCurrSubpic = currSubPic.getNumSlicesInSubPic();
    if( NumSlicesInCurrSubpic > 1 )
    {
      const int bitsSliceAddress = (int) std::ceil( std::log2( NumSlicesInCurrSubpic ) );
      X_READ_CODE( sh_slice_address, bitsSliceAddress, 0, NumSlicesInCurrSubpic - 1 );
      sliceAddr = sh_slice_address;
    }
  }

  const std::vector<bool>& shExtraBitsPresent = sps->getExtraSHBitPresentFlags();
  for( int i = 0; i < sps->getNumExtraSHBitsBytes() * 8; i++ )
  {
    // extra bits are ignored (when present)
    if( shExtraBitsPresent[i] )
    {
      X_READ_FLAG_idx( sh_extra_bit, "[i]" );
      (void) sh_extra_bit;
    }
  }

  uint32_t numTilesInSlice = 1;
  if( !pps->getRectSliceFlag() && (int) NumTilesInPic - (int) sliceAddr > 1 )
  {
    X_READ_UVLC( sh_num_tiles_in_slice_minus1, 0, NumTilesInPic - 1 );
    CHECK_CONSTRAINT( gci->getOneSlicePerPicConstraintFlag() && sh_num_tiles_in_slice_minus1 != NumTilesInPic - 1,
                      "When rect_slice_flag is equal to 0 and one_slice_per_pic_constraint_flag equal to 1, the value of num_tiles_in_slice_minus1 present in"
                      " each slice header shall be equal to NumTilesInPic - 1" );
    numTilesInSlice = sh_num_tiles_in_slice_minus1 + 1;
  }

  if( !pps->getRectSliceFlag() )
  {
    CHECK( sliceAddr >= pps->getNumTiles(), "Invalid slice address" );

    pcSlice->resetSliceMap();
    pcSlice->setSliceID( sliceAddr );

    // NumCtusInCurrSlice = 0;
    for( uint32_t tileIdx = sliceAddr; tileIdx < sliceAddr + numTilesInSlice; tileIdx++ )
    {
      uint32_t tileX = tileIdx % pps->getNumTileColumns();
      uint32_t tileY = tileIdx / pps->getNumTileColumns();
      CHECK( tileY >= pps->getNumTileRows(), "Number of tiles in slice exceeds the remaining number of tiles in picture" );

      pcSlice->addCtusToSlice( pps->getTileColumnBd( tileX ), pps->getTileColumnBd( tileX + 1 ),
                               pps->getTileRowBd( tileY ), pps->getTileRowBd( tileY + 1 ), pps->getPicWidthInCtu() );
    }
  }
  else
  {
    uint32_t       picLevelSliceIdx = sliceAddr;
    const uint32_t currSubPicIdx    = pps->getSubPicIdxFromSubPicId( pcSlice->getSliceSubPicId() );
    for( int subpic = 0; subpic < currSubPicIdx; subpic++ )
    {
      picLevelSliceIdx += pps->getSubPic( subpic ).getNumSlicesInSubPic();
    }
    pcSlice->setSliceMap( pps->getSliceMap( picLevelSliceIdx ) );
    pcSlice->setSliceID( picLevelSliceIdx );
  }

  if( firstSliceInPic != ( pcSlice->getCtuAddrInSlice( 0 ) == 0 ) )
  {
    // exit early, because we need to start again with some fields copied from previous slice
    firstSliceInPic = false;
    return;
  }

  if( picHeader->getPicInterSliceAllowedFlag() )
  {
    X_READ_UVLC( sh_slice_type, 0, 2 );
    pcSlice->setSliceType( (SliceType) sh_slice_type );
  }
  else
  {
    pcSlice->setSliceType( I_SLICE );
  }

  CHECK( !picHeader->getPicIntraSliceAllowedFlag() && pcSlice->getSliceType() != B_SLICE && pcSlice->getSliceType() != P_SLICE,
                     "When ph_intra_slice_allowed_flag is equal to 0, the value of sh_slice_type shall be equal to 0 or 1." );

  if( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
      || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP
      || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA
      || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR )
  {
    X_READ_FLAG( sh_no_output_of_prior_pics_flag );
    pcSlice->setNoOutputOfPriorPicsFlag( sh_no_output_of_prior_pics_flag );
  }

  // inherit values from picture header
  //   set default values in case slice overrides are disabled
  pcSlice->inheritFromPicHeader( picHeader.get(), pps, sps );

  if( sps->getUseALF() && !pps->getAlfInfoInPhFlag() )
  {
    X_READ_FLAG( sh_alf_enabled_flag );
    pcSlice->setAlfEnabledFlag( COMPONENT_Y, sh_alf_enabled_flag );

    if( sh_alf_enabled_flag )
    {
      X_READ_CODE_NO_RANGE( sh_num_alf_aps_ids_luma, 3 );
      pcSlice->setNumAlfAps( sh_num_alf_aps_ids_luma );

      AlfApsIdVec apsId( sh_num_alf_aps_ids_luma, -1 );
      for( int i = 0; i < sh_num_alf_aps_ids_luma; i++ )
      {
        X_READ_CODE_NO_RANGE_idx( sh_alf_aps_id_luma, "[i]", 3 );
        apsId[i] = sh_alf_aps_id_luma;

        const APS* APStoCheckLuma = parameterSetManager->getAPS( apsId[i], ALF_APS );
        CHECK( APStoCheckLuma == nullptr, "referenced APS not found" );
        CHECK( APStoCheckLuma->getAlfAPSParam().newFilterFlag[CHANNEL_TYPE_LUMA] != 1,
                           "The value of alf_luma_filter_signal_flag of the APS NAL unit having aps_params_type equal to ALF_APS and"
                           " aps_adaptation_parameter_set_id equal to sh_alf_aps_id_luma[ i ] shall be equal to 1." );
      }
      pcSlice->setAlfApsIdsLuma( apsId );

      if( bChroma )
      {
        X_READ_FLAG( sh_alf_cb_enabled_flag );
        pcSlice->setAlfEnabledFlag( COMPONENT_Cb, sh_alf_cb_enabled_flag );

        X_READ_FLAG( sh_alf_cr_enabled_flag );
        pcSlice->setAlfEnabledFlag( COMPONENT_Cr, sh_alf_cr_enabled_flag );
      }

      if( pcSlice->getAlfEnabledFlag( COMPONENT_Cb ) || pcSlice->getAlfEnabledFlag( COMPONENT_Cr ) )
      {
        X_READ_CODE_NO_RANGE( sh_alf_aps_id_chroma, 3 );
        pcSlice->setAlfApsIdChroma( sh_alf_aps_id_chroma );

        const APS* APStoCheckChroma = parameterSetManager->getAPS( sh_alf_aps_id_chroma, ALF_APS );
        CHECK( APStoCheckChroma == nullptr, "referenced APS not found" );
        CHECK( APStoCheckChroma->getAlfAPSParam().newFilterFlag[CHANNEL_TYPE_CHROMA] != 1,
                           "The value of alf_chroma_filter_signal_flag of the APS NAL unit having aps_params_type equal to ALF_APS and"
                           " aps_adaptation_parameter_set_id equal to sh_alf_aps_id_chroma shall be equal to 1." );
      }

      if( sps->getUseCCALF() )
      {
        X_READ_FLAG( sh_alf_cc_cb_enabled_flag );
        pcSlice->setCcAlfCbEnabledFlag( sh_alf_cc_cb_enabled_flag );

        if( sh_alf_cc_cb_enabled_flag )
        {
          X_READ_CODE_NO_RANGE( sh_alf_cc_cb_aps_id, 3 );
          pcSlice->setCcAlfCbApsId( sh_alf_cc_cb_aps_id );

          const APS* APStoCheckCcCb = parameterSetManager->getAPS( sh_alf_cc_cb_aps_id, ALF_APS );
          CHECK( !APStoCheckCcCb, "referenced APS not found" );
          CHECK( APStoCheckCcCb->getCcAlfAPSParam().newCcAlfFilter[0] != 1,
                             "The value of alf_cc_cb_filter_signal_flag of the APS NAL unit having aps_params_type equal to ALF_APS and "
                             "aps_adaptation_parameter_set_id equal to sh_alf_cc_cb_aps_id shall be equal to 1." );
        }

        X_READ_FLAG( sh_alf_cc_cr_enabled_flag );
        pcSlice->setCcAlfCrEnabledFlag( sh_alf_cc_cr_enabled_flag );

        if( sh_alf_cc_cr_enabled_flag )
        {
          X_READ_CODE_NO_RANGE( sh_alf_cc_cr_aps_id, 3 );
          pcSlice->setCcAlfCrApsId( sh_alf_cc_cr_aps_id );

          const APS* APStoCheckCcCr = parameterSetManager->getAPS( sh_alf_cc_cr_aps_id, ALF_APS );
          CHECK( !APStoCheckCcCr, "referenced APS not found" );
          CHECK( APStoCheckCcCr->getCcAlfAPSParam().newCcAlfFilter[1] != 1,
                             "The value of alf_cc_cr_filter_signal_flag of the APS NAL unit having aps_params_type equal to ALF_APS and "
                             "aps_adaptation_parameter_set_id equal to sh_alf_cc_cr_aps_id shall be equal to 1." );
        }
      }
    }
  }

  if( picHeader->getLmcsEnabledFlag() && !sh_picture_header_in_slice_header_flag )
  {
    X_READ_FLAG( sh_lmcs_used_flag );
    pcSlice->setLmcsEnabledFlag( sh_lmcs_used_flag );
  }

  if( picHeader->getExplicitScalingListEnabledFlag() && !sh_picture_header_in_slice_header_flag )
  {
    X_READ_FLAG( sh_explicit_scaling_list_used_flag );
    pcSlice->setExplicitScalingListUsed( sh_explicit_scaling_list_used_flag );
  }

  if( pps->getRplInfoInPhFlag() )
  {
    // inheritFromPicHeader() already called
  }
  else if( pcSlice->getIdrPicFlag() && !sps->getIDRRefParamListPresent() )
  {
    pcSlice->clearRPL( REF_PIC_LIST_0 );
    pcSlice->clearRPL( REF_PIC_LIST_1 );
  }
  else
  {
    parsePicOrSliceHeaderRPL( pcSlice, sps, pps );
  }

  bool     numRefIdxActiveOverrideFlag = true;
  unsigned numRefIdxActiveMinus1[2]    = { 0, 0 };
  if( ( !pcSlice->isIntra() && pcSlice->getRPL( REF_PIC_LIST_0 )->getNumRefEntries() > 1 ) ||   //
      ( pcSlice->isInterB() && pcSlice->getRPL( REF_PIC_LIST_1 )->getNumRefEntries() > 1 ) )
  {
    X_READ_FLAG( sh_num_ref_idx_active_override_flag );
    numRefIdxActiveOverrideFlag = sh_num_ref_idx_active_override_flag;

    if( sh_num_ref_idx_active_override_flag )
    {
      for( int i = 0; i < ( pcSlice->isInterB() ? 2 : 1 ); ++i )
      {
        if( pcSlice->getRPL( (RefPicList) i )->getNumRefEntries() > 1 )
        {
          X_READ_UVLC_idx( sh_num_ref_idx_active_minus1, "[ i ]", 0, 14 );
          numRefIdxActiveMinus1[i] = sh_num_ref_idx_active_minus1;
        }
      }
    }
  }

  for( auto i: { REF_PIC_LIST_0, REF_PIC_LIST_1 } )
  {
    if( pcSlice->isInterB() || ( pcSlice->isInterP() && i == REF_PIC_LIST_0 ) )
    {
      if( numRefIdxActiveOverrideFlag )
      {
        pcSlice->setNumRefIdx( i, numRefIdxActiveMinus1[i] + 1 );
      }
      else
      {
        const int pps_num_ref_idx_default_active_minus1 = ( i == 0 ? pps->getNumRefIdxL0DefaultActive() : pps->getNumRefIdxL1DefaultActive() ) - 1;
        if( pcSlice->getRPL( i )->getNumRefEntries() >= pps_num_ref_idx_default_active_minus1 + 1 )
        {
          pcSlice->setNumRefIdx( i, pps_num_ref_idx_default_active_minus1 + 1 );
        }
        else
        {
          pcSlice->setNumRefIdx( i, pcSlice->getRPL( i )->getNumRefEntries() );
        }
      }
    }
    else /* sh_slice_type == I || ( sh_slice_type == P && i == 1 ) */
    {
      pcSlice->setNumRefIdx( i, 0 );
    }
  }

  if( pcSlice->isInterP() || pcSlice->isInterB() )
  {
    CHECK( pcSlice->getNumRefIdx( REF_PIC_LIST_0 ) == 0, "Number of active entries in RPL0 of P or B picture shall be greater than 0" );
    if( pcSlice->isInterB() )
    {
      CHECK( pcSlice->getNumRefIdx( REF_PIC_LIST_1 ) == 0, "Number of active entries in RPL1 of B picture shall be greater than 0" );
    }
  }

  if( !pcSlice->isIntra() )
  {
    if( pps->getCabacInitPresentFlag() )
    {
      X_READ_FLAG( sh_cabac_init_flag );
      pcSlice->setCabacInitFlag( sh_cabac_init_flag );
    }

    if( picHeader->getEnableTMVPFlag() && !pps->getRplInfoInPhFlag() )
    {
      if( pcSlice->isInterB() )
      {
        X_READ_FLAG( sh_collocated_from_l0_flag );
        pcSlice->setColFromL0Flag( sh_collocated_from_l0_flag );
      }

      if( ( pcSlice->getColFromL0Flag() && pcSlice->getNumRefIdx( REF_PIC_LIST_0 ) > 1 )
          || ( !pcSlice->getColFromL0Flag() && pcSlice->getNumRefIdx( REF_PIC_LIST_1 ) > 1 ) )
      {
        X_READ_UVLC( sh_collocated_ref_idx, 0, pcSlice->getNumRefIdx( pcSlice->getColFromL0Flag() ? REF_PIC_LIST_0 : REF_PIC_LIST_1 ) - 1u );

        pcSlice->setColRefIdx( sh_collocated_ref_idx );
      }
    }

    if( !pps->getWpInfoInPhFlag() &&
        ( ( pps->getUseWP() && pcSlice->isInterP() ) || ( pps->getWPBiPred() && pcSlice->isInterB() ) ) )
    {
      parsePredWeightTable( pcSlice, sps, pps, { pcSlice->getNumRefIdx( REF_PIC_LIST_0 ), pcSlice->getNumRefIdx( REF_PIC_LIST_1 ) } );
    }

    if( pps->getWpInfoInPhFlag() )
    {
      CHECK( pps->getUseWP() && pcSlice->isInterP() && pcSlice->getNumRefIdx( REF_PIC_LIST_0 ) > picHeader->getNumL0Weights(),
             "When pps_wp_info_in_ph_flag is equal to 1, pps_weighted_pred_flag is equal to 1, and sh_slice_type is equal to P,"
             " NumRefIdxActive[ 0 ] shall be less than or equal to the value of NumWeightsL0." );
      CHECK( pps->getWPBiPred() && pcSlice->isInterB() && pcSlice->getNumRefIdx( REF_PIC_LIST_0 ) > picHeader->getNumL0Weights(),
             "When pps_wp_info_in_ph_flag is equal to 1, pps_weighted_bipred_flag is equal to 1, and sh_slice_type is equal to B,"
             " NumRefIdxActive[ 0 ] shall be less than or equal to the value of NumWeightsL0." );
      CHECK( pps->getWPBiPred() && pcSlice->isInterB() && pcSlice->getNumRefIdx( REF_PIC_LIST_1 ) > picHeader->getNumL1Weights(),
             "When pps_wp_info_in_ph_flag is equal to 1, pps_weighted_bipred_flag is equal to 1, and sh_slice_type is equal to B, "
             " the value of NumRefIdxActive[ 1 ] shall be less than or equal to the value of NumWeightsL1." );
    }
  }

  if( !pps->getQpDeltaInfoInPhFlag() )
  {
    X_READ_SVLC_NO_RANGE( sh_qp_delta );

    int SliceQpY = 26 + pps->getPicInitQPMinus26() + sh_qp_delta;
    CHECK_READ_RANGE( SliceQpY, -sps->getQpBDOffset(), MAX_QP, SliceQpY );

    pcSlice->setSliceQp( SliceQpY );
  }

  if( pps->getSliceChromaQpFlag() )
  {
    X_READ_SVLC( sh_cb_qp_offset, -12, +12 );
    CHECK_READ_RANGE( sh_cb_qp_offset + pps->getQpOffset( COMPONENT_Cb ), -12, +12, "pps_cb_qp_offset + sh_cb_qp_offset" );
    pcSlice->setSliceChromaQpDelta( COMPONENT_Cb, sh_cb_qp_offset );

    X_READ_SVLC( sh_cr_qp_offset, -12, +12 );
    CHECK_READ_RANGE( sh_cr_qp_offset + pps->getQpOffset( COMPONENT_Cr ), -12, +12, "pps_cr_qp_offset + sh_cr_qp_offset" );
    pcSlice->setSliceChromaQpDelta( COMPONENT_Cr, sh_cr_qp_offset );

    if( sps->getJointCbCrEnabledFlag() )
    {
      X_READ_SVLC( sh_joint_cbcr_qp_offset, -12, +12 );
      CHECK_READ_RANGE( sh_joint_cbcr_qp_offset + pps->getQpOffset( JOINT_CbCr ), -12, +12, "pps_joint_cbcr_qp_offset_value + sh_joint_cbcr_qp_offset" );
      pcSlice->setSliceChromaQpDelta( JOINT_CbCr, sh_joint_cbcr_qp_offset );
    }
  }
  if( pps->getCuChromaQpOffsetEnabledFlag() )
  {
    X_READ_FLAG( sh_cu_chroma_qp_offset_enabled_flag );
    pcSlice->setUseChromaQpAdj( sh_cu_chroma_qp_offset_enabled_flag );
  }

  if( sps->getUseSAO() && !pps->getSaoInfoInPhFlag() )
  {
    X_READ_FLAG( sh_sao_luma_used_flag );
    pcSlice->setSaoEnabledFlag( CHANNEL_TYPE_LUMA, sh_sao_luma_used_flag );

    if( bChroma )
    {
      X_READ_FLAG( sh_sao_chroma_used_flag );
      pcSlice->setSaoEnabledFlag( CHANNEL_TYPE_CHROMA, sh_sao_chroma_used_flag );
    }
  }

  if( pps->getDeblockingFilterOverrideEnabledFlag() && !pps->getDbfInfoInPhFlag() )
  {
    X_READ_FLAG( sh_deblocking_params_present_flag );
    pcSlice->setDeblockingFilterOverrideFlag( sh_deblocking_params_present_flag );
  }
  // If pps_deblocking_filter_disabled_flag and sh_deblocking_params_present_flag are both equal to 1, the value of sh_deblocking_filter_disabled_flag is
  // inferred to be equal to 0.
  // Otherwise (pps_deblocking_filter_disabled_flag or sh_deblocking_params_present_flag is equal to 0), the value of sh_deblocking_filter_disabled_flag is
  // inferred to be equal to ph_deblocking_filter_disabled_flag.
  pcSlice->setDeblockingFilterDisable( pps->getPPSDeblockingFilterDisabledFlag() && pcSlice->getDeblockingFilterOverrideFlag()
                                       ? 0 : picHeader->getDeblockingFilterDisable() );

  if( pcSlice->getDeblockingFilterOverrideFlag() )
  {
    if( !pps->getPPSDeblockingFilterDisabledFlag() )
    {
      X_READ_FLAG( sh_deblocking_filter_disabled_flag );
      pcSlice->setDeblockingFilterDisable( sh_deblocking_filter_disabled_flag );
    }

    if( !pcSlice->getDeblockingFilterDisable() )
    {
      X_READ_SVLC( sh_luma_beta_offset_div2, -12, 12 );
      pcSlice->setDeblockingFilterBetaOffsetDiv2( sh_luma_beta_offset_div2 );
      X_READ_SVLC( sh_luma_tc_offset_div2, -12, 12 );
      pcSlice->setDeblockingFilterTcOffsetDiv2( sh_luma_tc_offset_div2 );
    }
  }

  if( pcSlice->getDeblockingFilterOverrideFlag() && !pcSlice->getDeblockingFilterDisable() && pps->getPPSChromaToolFlag() )
  {
    X_READ_SVLC( sh_cb_beta_offset_div2, -12, 12 );
    pcSlice->setDeblockingFilterCbBetaOffsetDiv2( sh_cb_beta_offset_div2 );

    X_READ_SVLC( sh_cb_tc_offset_div2, -12, 12 );
    pcSlice->setDeblockingFilterCbTcOffsetDiv2( sh_cb_tc_offset_div2 );

    X_READ_SVLC( sh_cr_beta_offset_div2, -12, 12 );
    pcSlice->setDeblockingFilterCrBetaOffsetDiv2( sh_cr_beta_offset_div2 );

    X_READ_SVLC( sh_cr_tc_offset_div2, -12, 12 );
    pcSlice->setDeblockingFilterCrTcOffsetDiv2( sh_cr_tc_offset_div2 );
  }
  else
  {
    if( pps->getPPSChromaToolFlag() )
    {
      // If pps_chroma_tool_offsets_present_flag is equal to 1, the values of sh_cb_beta_offset_div2 and sh_cb_tc_offset_div2 are inferred to be equal to
      // ph_cb_beta_offset_div2 and ph_cb_tc_offset_div2, respectively.
      // If pps_chroma_tool_offsets_present_flag is equal to 1, the values of sh_cr_beta_offset_div2 and sh_cr_tc_offset_div2 are inferred to be equal to
      // ph_cr_beta_offset_div2 and ph_cr_tc_offset_div2, respectively.

      pcSlice->setDeblockingFilterCbBetaOffsetDiv2( picHeader->getDeblockingFilterCbBetaOffsetDiv2() );
      pcSlice->setDeblockingFilterCbTcOffsetDiv2  ( picHeader->getDeblockingFilterCbTcOffsetDiv2()   );
      pcSlice->setDeblockingFilterCrBetaOffsetDiv2( picHeader->getDeblockingFilterCrBetaOffsetDiv2() );
      pcSlice->setDeblockingFilterCrTcOffsetDiv2  ( picHeader->getDeblockingFilterCrTcOffsetDiv2()   );
    }
    else
    {
      // Otherwise (pps_chroma_tool_offsets_present_flag is equal to 0), the values of sh_cb_beta_offset_div2 and sh_cb_tc_offset_div2 are inferred to be equal to
      // sh_luma_beta_offset_div2 and sh_luma_tc_offset_div2, respectively.
      // Otherwise( pps_chroma_tool_offsets_present_flag is equal to 0 ), the values of sh_cr_beta_offset_div2 and sh_cr_tc_offset_div2 are inferred to be equal
      // to sh_luma_beta_offset_div2 and sh_luma_tc_offset_div2, respectively.

      pcSlice->setDeblockingFilterCbBetaOffsetDiv2( pcSlice->getDeblockingFilterBetaOffsetDiv2() );
      pcSlice->setDeblockingFilterCbTcOffsetDiv2  ( pcSlice->getDeblockingFilterTcOffsetDiv2()   );
      pcSlice->setDeblockingFilterCrBetaOffsetDiv2( pcSlice->getDeblockingFilterBetaOffsetDiv2() );
      pcSlice->setDeblockingFilterCrTcOffsetDiv2  ( pcSlice->getDeblockingFilterTcOffsetDiv2()   );
    }
  }

  // dependent quantization
  if( sps->getDepQuantEnabledFlag() )
  {
    X_READ_FLAG( sh_dep_quant_used_flag );
    pcSlice->setDepQuantEnabledFlag( sh_dep_quant_used_flag );
  }

  // sign data hiding
  if( sps->getSignDataHidingEnabledFlag() && !pcSlice->getDepQuantEnabledFlag() )
  {
    X_READ_FLAG( sh_sign_data_hiding_used_flag );
    pcSlice->setSignDataHidingEnabledFlag( sh_sign_data_hiding_used_flag );
  }

  // signal TS residual coding disabled flag
  if( sps->getTransformSkipEnabledFlag() && !pcSlice->getDepQuantEnabledFlag() && !pcSlice->getSignDataHidingEnabledFlag() )
  {
    X_READ_FLAG( sh_ts_residual_coding_disabled_flag );
    pcSlice->setTSResidualCodingDisabledFlag( sh_ts_residual_coding_disabled_flag );
  }

  if( pps->getSliceHeaderExtensionPresentFlag() )
  {
    X_READ_UVLC( sh_slice_header_extension_length, 0, 256 );
    for( int i = 0; i < sh_slice_header_extension_length; i++ )
    {
      X_READ_CODE_NO_RANGE_idx( sh_slice_header_extension_data_byte, "[ i ]", 8 );
      (void) sh_slice_header_extension_data_byte;
    }
  }

  std::vector<uint32_t> entryPointOffset;
  pcSlice->setNumEntryPoints( sps, pps );
  if( pcSlice->getNumEntryPoints() > 0 )
  {
    entryPointOffset.resize( pcSlice->getNumEntryPoints() );

    X_READ_UVLC( sh_entry_offset_len_minus1, 0, 31 );
    for( uint32_t idx = 0; idx < pcSlice->getNumEntryPoints(); idx++ )
    {
      X_READ_CODE_NO_RANGE_idx( sh_entry_point_offset_minus1, "[i]", sh_entry_offset_len_minus1 + 1 );
      entryPointOffset[idx] = sh_entry_point_offset_minus1 + 1;
    }
  }

#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::IncrementStatisticEP(STATS__BYTE_ALIGNMENT_BITS,m_pcBitstream->readByteAlignment(),0);
#else
  m_pcBitstream->readByteAlignment();
#endif


  if( pcSlice->getFirstCtuRsAddrInSlice() == 0 )
  {
    pcSlice->setDefaultClpRng( *sps );
  }

  pcSlice->clearSubstreamSizes();

  if( pcSlice->getNumEntryPoints() > 0 )
  {
    int endOfSliceHeaderLocation = m_pcBitstream->getByteLocation();

    // Adjust endOfSliceHeaderLocation to account for emulation prevention bytes in the slice segment header
    for( uint32_t curByteIdx = 0; curByteIdx < m_pcBitstream->numEmulationPreventionBytesRead(); curByteIdx++ )
    {
      if( m_pcBitstream->getEmulationPreventionByteLocation( curByteIdx ) < endOfSliceHeaderLocation )
      {
        endOfSliceHeaderLocation++;
      }
    }

    uint32_t prevEntryPoint = 0;
    for( uint32_t idx = 0; idx < entryPointOffset.size(); idx++ )
    {
      const uint32_t currEntryPoint = prevEntryPoint + entryPointOffset[idx];

      int emulationPreventionByteCount = 0;
      for( uint32_t curByteIdx = 0; curByteIdx < m_pcBitstream->numEmulationPreventionBytesRead(); curByteIdx++ )
      {
        if( m_pcBitstream->getEmulationPreventionByteLocation( curByteIdx ) >= prevEntryPoint + endOfSliceHeaderLocation
            && m_pcBitstream->getEmulationPreventionByteLocation( curByteIdx ) < currEntryPoint + endOfSliceHeaderLocation )
        {
          emulationPreventionByteCount++;
        }
      }

      entryPointOffset[idx] -= emulationPreventionByteCount;

      prevEntryPoint = currEntryPoint;
    }

    pcSlice->setSubstreamSizes( std::move( entryPointOffset ) );
  }
}

template<typename HeaderT>
void HLSyntaxReader::parsePicOrSliceHeaderRPL( HeaderT* header, const SPS* sps, const PPS* pps )
{
  bool rplSpsFlag[2] = { false, false };

  // List0 and List1
  for( RefPicList listIdx: { REF_PIC_LIST_0, REF_PIC_LIST_1 } )
  {
    const unsigned sps_num_ref_pic_lists_i   = sps->getNumRPL( listIdx );
    const bool     pps_rpl1_idx_present_flag = pps->getRpl1IdxPresentFlag();

    if( sps_num_ref_pic_lists_i > 0 && ( listIdx == 0 || ( listIdx == 1 && pps_rpl1_idx_present_flag ) ) )
    {
      X_READ_FLAG_idx( ref_pic_list_sps_flag, "[i]" );   // rpl_sps_flag[i] in the standard
      rplSpsFlag[listIdx] = ref_pic_list_sps_flag;
    }
    else if( sps_num_ref_pic_lists_i == 0 )
    {
      rplSpsFlag[listIdx] = false;
    }
    else if( sps_num_ref_pic_lists_i > 0 && !pps_rpl1_idx_present_flag && listIdx == REF_PIC_LIST_1 )
    {
      rplSpsFlag[listIdx] = rplSpsFlag[0];
    }

    if( rplSpsFlag[listIdx] )
    {
      // When rpl_sps_flag[ i ] is equal to 1 and sps_num_ref_pic_lists[ i ] is equal to 1, the value of rpl_idx[ i ] is inferred to be
      // equal to 0. When rpl_sps_flag[ 1 ] is equal to 1, pps_rpl1_idx_present_flag is equal to 0, and sps_num_ref_pic_lists[ 1 ]
      // is greater than 1, the value of rpl_idx[ 1 ] is inferred to be equal to rpl_idx[ 0 ].
      int rpl_idx_i = 0;
      if( rplSpsFlag[listIdx] && sps_num_ref_pic_lists_i == 1 )
      {
        rpl_idx_i = 0;
      }
      else if( listIdx == REF_PIC_LIST_1 && rplSpsFlag[1] && !pps_rpl1_idx_present_flag && sps->getNumRPL( REF_PIC_LIST_1 ) > 1 )
      {
        rpl_idx_i = header->getRPLIdx( REF_PIC_LIST_0 );
      }

      if( sps_num_ref_pic_lists_i > 1 && ( listIdx == REF_PIC_LIST_0 || ( listIdx == REF_PIC_LIST_1 && pps_rpl1_idx_present_flag ) ) )
      {
        int numBits = std::ceil( std::log2( sps_num_ref_pic_lists_i ) );
        X_READ_CODE_idx( ref_pic_list_idx, "[ listIdx ]", numBits, 0, sps_num_ref_pic_lists_i - 1 );   // rpl_idx[i] in the standard
        rpl_idx_i = ref_pic_list_idx;
      }

      CHECK( rpl_idx_i < 0 || rpl_idx_i > (int) sps_num_ref_pic_lists_i - 1,
             "The value of rpl_idx[ i ] shall be in the range of 0 to sps_num_ref_pic_lists[ i ] - 1, inclusive." );

      header->setRPL( listIdx, sps->getRPLList( listIdx )[rpl_idx_i] );
      header->setRPLIdx( listIdx, rpl_idx_i );
    }
    else
    {
      header->clearRPL( listIdx );
      parseRefPicList( header->getRPL( listIdx ), -1, sps );   // ref_pic_list_struct( i, sps_num_ref_pic_lists[ i ] )
      header->setRPLIdx( listIdx, -1 );
    }

    if( std::is_same<HeaderT, PicHeader>::value )   // The contained CHECK is only valid, when we are in a PicHeader.
    {
      // The reinterpret_cast<> is a no-op, but it's needed for compilation. It's never executed in the case, when HeaderT is not a PicHeader due to the
      // surrounding if-condition
      CHECK( pps->getRplInfoInPhFlag() && reinterpret_cast<PicHeader*>( header )->getPicInterSliceAllowedFlag()
               && header->getRPL( REF_PIC_LIST_0 )->getNumRefEntries() <= 0,
             "When pps_rpl_info_in_ph_flag is equal to 1 and ph_inter_slice_allowed_flag is equal to 1, the value of"
             " num_ref_entries[ 0 ][ RplsIdx[ 0 ] ] shall be greater than 0." )
    }

    // Deal POC Msb cycle signalling for LTRP
    auto* rpl = header->getRPL( listIdx );
    for( int j = 0; j < rpl->getNumRefEntries(); ++j )
    {
      if( !rpl->isRefPicLongterm( j ) )
      {
        continue;
      }

      if( rpl->getLtrpInSliceHeaderFlag() )
      {
        X_READ_CODE_NO_RANGE_idx( poc_lsb_lt, "[i][j]", sps->getBitsForPOC() );
        rpl->setRefPicIdentifier( j, poc_lsb_lt, true, false, 0 );
      }

      X_READ_FLAG_idx( delta_poc_msb_cycle_present_flag, "[i][j]" );
      rpl->setDeltaPocMSBPresentFlag( j, delta_poc_msb_cycle_present_flag );

      if( delta_poc_msb_cycle_present_flag )
      {
        X_READ_UVLC( delta_poc_msb_cycle_lt, 0, 1 << ( 32 - sps->getBitsForPOC() ) );
        rpl->setDeltaPocMSBCycleLT( j, delta_poc_msb_cycle_lt );
      }
    }
  }
}

void HLSyntaxReader::parseConstraintInfo( ConstraintInfo *cinfo )
{
  uint32_t symbol;
  READ_FLAG( symbol, "gci_present_flag" );                                   cinfo->setGciPresentFlag( symbol ? true : false );
  if( cinfo->getGciPresentFlag() )
  {
    /* general */
    READ_FLAG( symbol, "gci_intra_only_constraint_flag" );                   cinfo->setIntraOnlyConstraintFlag( symbol ? true : false );
    READ_FLAG( symbol, "gci_all_layers_independent_constraint_flag" );       cinfo->setAllLayersIndependentConstraintFlag( symbol ? true : false );
    READ_FLAG( symbol, "gci_one_au_only_constraint_flag" );                  cinfo->setOnePictureOnlyConstraintFlag( symbol ? true : false );

    /* picture format */
    READ_CODE( 4, symbol, "gci_sixteen_minus_max_bitdepth_constraint_idc" ); cinfo->setMaxBitDepthConstraintIdc( symbol>8 ? 16 : ( 16 - symbol ) );
    CHECK(symbol>8, "gci_sixteen_minus_max_bitdepth_constraint_idc shall be in the range 0 to 8, inclusive");
    READ_CODE( 2, symbol, "gci_three_minus_max_chroma_format_constraint_idc" );
    cinfo->setMaxChromaFormatConstraintIdc( (ChromaFormat)( 3 - symbol ) );

    /* NAL unit type related */
    READ_FLAG( symbol, "gci_no_mixed_nalu_types_in_pic_constraint_flag" );   cinfo->setNoMixedNaluTypesInPicConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_trail_constraint_flag" );                     cinfo->setNoTrailConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_stsa_constraint_flag" );                      cinfo->setNoStsaConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_rasl_constraint_flag" );                      cinfo->setNoRaslConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_radl_constraint_flag" );                      cinfo->setNoRadlConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_idr_constraint_flag" );                       cinfo->setNoIdrConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_cra_constraint_flag" );                       cinfo->setNoCraConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_gdr_constraint_flag" );                       cinfo->setNoGdrConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_aps_constraint_flag" );                       cinfo->setNoApsConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_idr_rpl_constraint_flag" );                   cinfo->setNoIdrRplConstraintFlag( symbol > 0 ? true : false );

    /* tile, slice, subpicture partitioning */
    READ_FLAG( symbol, "gci_one_tile_per_pic_constraint_flag" );             cinfo->setOneTilePerPicConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_pic_header_in_slice_header_constraint_flag" );   cinfo->setPicHeaderInSliceHeaderConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_one_slice_per_pic_constraint_flag" );            cinfo->setOneSlicePerPicConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_rectangular_slice_constraint_flag" );         cinfo->setNoRectSliceConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_one_slice_per_subpic_constraint_flag" );         cinfo->setOneSlicePerSubpicConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_subpic_info_constraint_flag" );               cinfo->setNoSubpicInfoConstraintFlag( symbol > 0 ? true : false );

    /* CTU and block partitioning */
    READ_CODE( 2, symbol, "gci_three_minus_max_log2_ctu_size_constraint_idc");   cinfo->setMaxLog2CtuSizeConstraintIdc( ( ( 3 - symbol ) + 5 ) );
    READ_FLAG( symbol, "gci_no_partition_constraints_override_constraint_flag"); cinfo->setNoPartitionConstraintsOverrideConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_mtt_constraint_flag");                            cinfo->setNoMttConstraintFlag( symbol > 0 ? true : false);
    READ_FLAG( symbol, "gci_no_qtbtt_dual_tree_intra_constraint_flag");          cinfo->setNoQtbttDualTreeIntraConstraintFlag( symbol > 0 ? true : false );

    /* intra */
    READ_FLAG( symbol, "gci_no_palette_constraint_flag" );                   cinfo->setNoPaletteConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_ibc_constraint_flag" );                       cinfo->setNoIbcConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_isp_constraint_flag" );                       cinfo->setNoIspConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_mrl_constraint_flag" );                       cinfo->setNoMrlConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_mip_constraint_flag" );                       cinfo->setNoMipConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_cclm_constraint_flag" );                      cinfo->setNoCclmConstraintFlag( symbol > 0 ? true : false );

    /* inter */
    READ_FLAG( symbol, "gci_no_ref_pic_resampling_constraint_flag" );        cinfo->setNoRprConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_res_change_in_clvs_constraint_flag" );        cinfo->setNoResChangeInClvsConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_weighted_prediction_constraint_flag" );       cinfo->setNoWeightedPredictionConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_ref_wraparound_constraint_flag" );            cinfo->setNoRefWraparoundConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_temporal_mvp_constraint_flag" );              cinfo->setNoTemporalMvpConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_sbtmvp_constraint_flag" );                    cinfo->setNoSbtmvpConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_amvr_constraint_flag" );                      cinfo->setNoAmvrConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_bdof_constraint_flag" );                      cinfo->setNoBdofConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_smvd_constraint_flag" );                      cinfo->setNoSmvdConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_dmvr_constraint_flag" );                      cinfo->setNoDmvrConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_mmvd_constraint_flag" );                      cinfo->setNoMmvdConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_affine_motion_constraint_flag" );             cinfo->setNoAffineMotionConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_prof_constraint_flag" );                      cinfo->setNoProfConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_bcw_constraint_flag" );                       cinfo->setNoBcwConstraintFlag( symbol > 0 ? true : false  );
    READ_FLAG( symbol, "gci_no_ciip_constraint_flag" );                      cinfo->setNoCiipConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_gpm_constraint_flag" );                       cinfo->setNoGeoConstraintFlag( symbol > 0 ? true : false );

    /* transform, quantization, residual */
    READ_FLAG( symbol, "gci_no_luma_transform_size_64_constraint_flag" );    cinfo->setNoLumaTransformSize64ConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_transform_skip_constraint_flag" );            cinfo->setNoTransformSkipConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_bdpcm_constraint_flag" );                     cinfo->setNoBDPCMConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_mts_constraint_flag" );                       cinfo->setNoMtsConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_lfnst_constraint_flag" );                     cinfo->setNoLfnstConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_joint_cbcr_constraint_flag" );                cinfo->setNoJointCbCrConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_sbt_constraint_flag" );                       cinfo->setNoSbtConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_act_constraint_flag" );                       cinfo->setNoActConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_explicit_scaling_list_constraint_flag" );     cinfo->setNoExplicitScaleListConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_dep_quant_constraint_flag" );                 cinfo->setNoDepQuantConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_sign_data_hiding_constraint_flag" );          cinfo->setNoSignDataHidingConstraintFlag( symbol > 0 ? true : false) ;
    READ_FLAG( symbol, "gci_no_cu_qp_delta_constraint_flag" );               cinfo->setNoQpDeltaConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_chroma_qp_offset_constraint_flag" );          cinfo->setNoChromaQpOffsetConstraintFlag( symbol > 0 ? true : false );

    /* loop filter */
    READ_FLAG( symbol, "gci_no_sao_constraint_flag" );                       cinfo->setNoSaoConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_alf_constraint_flag" );                       cinfo->setNoAlfConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_ccalf_constraint_flag" );                     cinfo->setNoCCAlfConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_lmcs_constraint_flag" );                      cinfo->setNoLmcsConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_ladf_constraint_flag" );                      cinfo->setNoLadfConstraintFlag( symbol > 0 ? true : false );
    READ_FLAG( symbol, "gci_no_virtual_boundaries_constraint_flag" );        cinfo->setNoVirtualBoundaryConstraintFlag( symbol > 0 ? true : false );

    READ_CODE( 8, symbol, "gci_num_reserved_bits" );
    uint32_t const numReservedBits = symbol;
    for (int i = 0; i < numReservedBits; i++)
    {
      READ_FLAG( symbol, "gci_reserved_zero_bit" );                          CHECK( symbol != 0, "gci_reserved_zero_bit not equal to zero" );
    }
  }
  while( !isByteAligned() )
  {
    READ_FLAG( symbol, "gci_alignment_zero_bit" );                           CHECK( symbol != 0, "gci_alignment_zero_bit not equal to zero" );
  }
}


void HLSyntaxReader::parseProfileTierLevel( ProfileTierLevel *ptl, bool profileTierPresentFlag, int maxNumSubLayersMinus1 )
{
  if( profileTierPresentFlag )
  {
    X_READ_CODE_NO_RANGE( general_profile_idc, 7 );
    ptl->setProfileIdc( Profile::Name( general_profile_idc ) );

    X_READ_FLAG( general_tier_flag );
    ptl->setTierFlag( general_tier_flag ? Tier::HIGH : Tier::MAIN );
  }

  X_READ_CODE_NO_RANGE( general_level_idc, 8 );
  ptl->setLevelIdc( vvdecLevel( general_level_idc ) );

  X_READ_FLAG( ptl_frame_only_constraint_flag );
  ptl->setFrameOnlyConstraintFlag( ptl_frame_only_constraint_flag );

  X_READ_FLAG( ptl_multilayer_enabled_flag );
  ptl->setMultiLayerEnabledFlag( ptl_multilayer_enabled_flag );

  CHECK( ( ptl->getProfileIdc() == Profile::MAIN_10 || ptl->getProfileIdc() == Profile::MAIN_10_444
        || ptl->getProfileIdc() == Profile::MAIN_10_STILL_PICTURE
        || ptl->getProfileIdc() == Profile::MAIN_10_444_STILL_PICTURE )
          && ptl_multilayer_enabled_flag,
        "ptl_multilayer_enabled_flag shall be equal to 0 for non-multilayer profiles");

  CHECK_UNSUPPORTED( ptl->getProfileIdc() == Profile::MULTILAYER_MAIN_10 || ptl->getProfileIdc() == Profile::MULTILAYER_MAIN_10_STILL_PICTURE ||
         ptl->getProfileIdc() == Profile::MULTILAYER_MAIN_10_444 || ptl->getProfileIdc() == Profile::MULTILAYER_MAIN_10_444_STILL_PICTURE,
         "Multilayer profiles not yet supported" );

  if( ptl->getProfileIdc() == Profile::MAIN_10_444 || ptl->getProfileIdc() == Profile::MAIN_10_444_STILL_PICTURE )
  {
    msg( WARNING, "Warning: MAIN_10_444 and MAIN_10_444_STILL_PICTURE is still experimental.\n" );
  }

  if( profileTierPresentFlag )
  {
    parseConstraintInfo( ptl->getConstraintInfo() );
  }

  for( int i = maxNumSubLayersMinus1 - 1; i >= 0; i-- )
  {
    X_READ_FLAG_idx( sub_layer_level_present_flag, "[i]" );
    ptl->setSubLayerLevelPresentFlag( i, sub_layer_level_present_flag );
  }

  while( !isByteAligned() )
  {
    X_READ_FLAG( ptl_reserved_zero_bit );
    CHECK_WARN( ptl_reserved_zero_bit != 0, "ptl_reserved_zero_bit not equal to zero" );
  }

  ptl->setSubLayerLevelIdc( maxNumSubLayersMinus1, ptl->getLevelIdc() );
  for( int i = maxNumSubLayersMinus1 - 1; i >= 0; i-- )
  {
    if( ptl->getSubLayerLevelPresentFlag( i ) )
    {
      X_READ_CODE_NO_RANGE_idx( sub_layer_level_idc, "[i]", 8 );
      ptl->setSubLayerLevelIdc( i, vvdecLevel( sub_layer_level_idc ) );
    }
    else
    {
      ptl->setSubLayerLevelIdc( i, ptl->getSubLayerLevelIdc( i + 1 ) );
    }
  }

  if( profileTierPresentFlag )
  {
    X_READ_CODE_NO_RANGE( ptl_num_sub_profiles, 8 );
    ptl->setNumSubProfile( ptl_num_sub_profiles );

    for( int i = 0; i < ptl_num_sub_profiles; i++ )
    {
      X_READ_CODE_NO_RANGE_idx( general_sub_profile_idc, "[i]", 32 );
      ptl->setSubProfileIdc( i, general_sub_profile_idc );
    }
  }
}


// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

//! parse explicit wp tables
template<typename HeaderT>
void HLSyntaxReader::parsePredWeightTable( HeaderT* sh_or_ph, const SPS* sps, const PPS* pps, std::array<int, 2> numRefIdxActive )
{
  const bool bChroma                  = sps->getChromaFormatIdc() != CHROMA_400;
  const bool pps_wp_info_in_ph_flag   = pps->getWpInfoInPhFlag();
  const bool pps_weighted_bipred_flag = pps->getWPBiPred();

  uint32_t log2WeightDenomChroma = 0;
  uint32_t sumWeightFlags        = 0;

  X_READ_UVLC( luma_log2_weight_denom, 0, 7 );
  if( bChroma )
  {
    X_READ_SVLC_NO_RANGE( delta_chroma_log2_weight_denom );
    CHECK_READ_RANGE( luma_log2_weight_denom + delta_chroma_log2_weight_denom, 0, 7, "luma_log2_weight_denom + delta_chroma_log2_weight_denom" )

    log2WeightDenomChroma = luma_log2_weight_denom + delta_chroma_log2_weight_denom;
  }

  for( auto& l: { REF_PIC_LIST_0, REF_PIC_LIST_1 } )
  {
    const int num_ref_entries = sh_or_ph->getRPL( l )->getNumRefEntries();

    unsigned numWeights = numRefIdxActive[l];
    if( l == REF_PIC_LIST_0 )
    {
      // derive NumWeightsL0
      if( pps_wp_info_in_ph_flag )
      {
        X_READ_UVLC( num_l0_weights, 1, std::min<unsigned>( 15, num_ref_entries ) );
        numWeights = num_l0_weights;

        CHECK( ( !std::is_same<HeaderT, PicHeader>::value ),
               "Implementation error: parsePredWeightTable() should not be called with a Slice, when pps_wp_info_in_ph_flag is true." );
        if( std::is_same<HeaderT, PicHeader>::value )
        {
          // We know HeaderT is a PicHeader, so we use reinterpret_cast here just so it compiles for the HeaderT==Slice case.
          reinterpret_cast<PicHeader*>( sh_or_ph )->setNumL0Weights( num_l0_weights );
        }
      }
    }
    else if( l == REF_PIC_LIST_1 )
    {
      // The variable NumWeightsL1 is derived as follows (semantics section):
      if( !pps_weighted_bipred_flag || ( pps_wp_info_in_ph_flag && num_ref_entries == 0 ) )
      {
        numWeights = 0;
      }
      else if( pps_weighted_bipred_flag )   // condition from semantics section
      {
        // What should numWeights be set, when pps_weighted_bipred_flag && !pps_wp_info_in_ph_flag && num_ref_entries > 0 ?
        // According to the syntax section num_l1_weights isn't read, but the semantics says, we should set NumWeightsL1 = num_l1_weights.
        // It seems to work, when we leave numWeights set to NumRefIdxActive[ 1 ].
        //CHECK( pps_weighted_bipred_flag && !pps_wp_info_in_ph_flag && num_ref_entries > 0,
        //       "don't know how to set NumWeightsL1: it's missing from the spec" );

        if( pps_weighted_bipred_flag && pps_wp_info_in_ph_flag && num_ref_entries > 0 )   // condition from syntax section
        {
          X_READ_UVLC( num_l1_weights, 1, std::min<unsigned>( 15, num_ref_entries ) );
          numWeights = num_l1_weights;

          CHECK( ( !std::is_same<HeaderT, PicHeader>::value ),
                 "Implementation error: parsePredWeightTable() should not be called with a Slice, when pps_wp_info_in_ph_flag is true." );
          if( std::is_same<HeaderT, PicHeader>::value )
          {
            // We know HeaderT is a PicHeader, so we use reinterpret_cast here just so it compiles for the HeaderT==Slice case.
            reinterpret_cast<PicHeader*>( sh_or_ph )->setNumL1Weights( num_l1_weights );
          }
        }
      }
    }

    for( int i = 0; i < numWeights; i++ )
    {
      WPScalingParam* wp;
      sh_or_ph->getWpScaling( l, i, wp );

      X_READ_FLAG_idx( luma_weight_lX_flag, "[i]" );
      wp[COMPONENT_Y].bPresentFlag      = luma_weight_lX_flag;
      wp[COMPONENT_Y].uiLog2WeightDenom = luma_log2_weight_denom;

      sumWeightFlags += luma_weight_lX_flag;
    }

    if( bChroma )
    {
      for( int i = 0; i < numWeights; i++ )
      {
        WPScalingParam* wp;
        sh_or_ph->getWpScaling( l, i, wp );

        X_READ_FLAG_idx( chroma_weight_lX_flag, "[i]" );
        wp[COMPONENT_Cb].bPresentFlag      = wp[COMPONENT_Cr].bPresentFlag      = chroma_weight_lX_flag;
        wp[COMPONENT_Cb].uiLog2WeightDenom = wp[COMPONENT_Cr].uiLog2WeightDenom = log2WeightDenomChroma;

        sumWeightFlags += 2 * chroma_weight_lX_flag;
      }
    }

    for( int i = 0; i < numWeights; i++ )
    {
      WPScalingParam* wp;
      sh_or_ph->getWpScaling( l, i, wp );

      wp[COMPONENT_Y].iWeight = ( 1 << wp[COMPONENT_Y].uiLog2WeightDenom );
      wp[COMPONENT_Y].iOffset = 0;

      if( wp[COMPONENT_Y].bPresentFlag /*luma_weight_l0_flag[i]*/ )
      {
        X_READ_SVLC_idx( delta_luma_weight_lX, "[i]", -128, 127 );
        wp[COMPONENT_Y].iWeight += delta_luma_weight_lX;

        X_READ_SVLC_idx( luma_offset_lX, "[i]", -128, 127 );
        wp[COMPONENT_Y].iOffset = luma_offset_lX;
      }

      for( int j = COMPONENT_Cb; j < MAX_NUM_COMPONENT; j++ )
      {
        wp[j].iWeight = ( 1 << wp[j].uiLog2WeightDenom );
        wp[j].iOffset = 0;
        if( wp[j].bPresentFlag /*chroma_weight_l0_flag[i]*/ )
        {
          X_READ_SVLC_idx( delta_chroma_weight_lX, "[i][j]", -128, 127 );
          wp[j].iWeight += delta_chroma_weight_lX;

          X_READ_SVLC_idx( delta_chroma_offset_lX, "[i][j]", -4 * 128, 4 * 127 );
          wp[j].iOffset = Clip3( -128, 127, 128 + delta_chroma_offset_lX - ( ( 128 * wp[j].iWeight ) >> wp[j].uiLog2WeightDenom ) );
        }
      }
    }

    for( int iRefIdx = numWeights; iRefIdx < MAX_NUM_REF; iRefIdx++ )
    {
      WPScalingParam* wp;
      sh_or_ph->getWpScaling( l, iRefIdx, wp );

      wp[0].bPresentFlag = false;
      wp[1].bPresentFlag = false;
      wp[2].bPresentFlag = false;
    }
  }

  CHECK( sumWeightFlags > 24,
         "It is a requirement of bitstream conformance that, when sh_slice_type is equal to P, sumWeightL0Flags shall be less than"
         " or equal to 24 and when sh_slice_type is equal to B, the sum of sumWeightL0Flags and sumWeightL1Flags shall be less"
         " than or equal to 24." );
}

/** decode quantization matrix
* \param scalingList quantization matrix information
*/
void HLSyntaxReader::parseScalingList( ScalingList *scalingList, bool aps_chromaPresentFlag )
{
  scalingList->reset();

  for( int id = 0; id < 28; id++ )
  {
    if( aps_chromaPresentFlag || scalingList->isLumaScalingList( id ) )
    {
      X_READ_FLAG( scaling_list_copy_mode_flag );

      bool scalingListPredModeFlag = false;
      if( !scaling_list_copy_mode_flag )
      {
        X_READ_FLAG( scaling_list_pred_mode_flag );
        scalingListPredModeFlag = scaling_list_pred_mode_flag;
      }

      int scalingListPredIdDelta = 0;
      if( ( scaling_list_copy_mode_flag || scalingListPredModeFlag )
          && id != SCALING_LIST_1D_START_2x2 && id != SCALING_LIST_1D_START_4x4
          && id != SCALING_LIST_1D_START_8x8 )   // Copy Mode
      {
        const unsigned maxIdDelta = id < 2 ? id : ( id < 8 ? id - 2 : id - 8 );
        X_READ_UVLC_idx( scaling_list_pred_id_delta, "[id]", 0, maxIdDelta );

        scalingListPredIdDelta = scaling_list_pred_id_delta;
      }

      decodeScalingList( scalingList, id, scalingListPredIdDelta, scaling_list_copy_mode_flag, scalingListPredModeFlag );
    }   // ( aps_chromaPresentFlag || scalingList->isLumaScalingList( id ) )
  }

  return;
}

void HLSyntaxReader::decodeScalingList( ScalingList* scalingList, uint32_t id, uint32_t scalingListPredIdDelta, bool scalingListCopyModeFlag , bool scalingListPredModeFlag)
{
  const unsigned matrixSize = ScalingList::matrixSize( id );

  const int refId = id - scalingListPredIdDelta;
  CHECK( refId<0, "refId < 0 doesn't make sense" );

  int   scalingMatrixDcPred = 0;
  auto& scalingMatrixPred   = scalingList->getScalingListVec( id );   // we use the same storage for scalingMatrixPred and scalingMatrixRec
  {
    // Derive scalingMatrixPred and scalingMatrixDcPred

    const size_t scalingMatrixPred_size = matrixSize * matrixSize;
    CHECK( scalingMatrixPred_size != scalingList->getScalingListVec( id ).size(), "wrong scalingMatrixPred/Rec[] size" )

    if( scalingListCopyModeFlag == 0 && scalingListPredModeFlag == 0 )
    {
      scalingMatrixPred.assign( scalingMatrixPred_size, 8 );
      scalingMatrixDcPred = 8;
    }
    else if( scalingListPredIdDelta == 0 )
    {
      scalingMatrixPred.assign( scalingMatrixPred_size, 16 );
      scalingMatrixDcPred = 16;
    }
    else
    {
      CHECK( ( scalingListCopyModeFlag == 0 && scalingListPredModeFlag == 0 ) || scalingListPredIdDelta <= 0,
             "wrong condition: Otherwise (either scaling_list_copy_mode_flag[ id ] or scaling_list_pred_mode_flag[ id ] is equal to 1 and"
             " scaling_list_pred_id_delta[ id ] is greater than 0)," );

      CHECK( scalingListCopyModeFlag == 0 && ( scalingListPredModeFlag == 0 || scalingListPredIdDelta <= 0 ),
             "wrong condition: Otherwise (either scaling_list_copy_mode_flag[ id ] or scaling_list_pred_mode_flag[ id ] is equal to 1 and"
             " scaling_list_pred_id_delta[ id ] is greater than 0)," );

      // scalingMatrixPred is set equal to ScalingMatrixRec[ refId ]
      scalingMatrixPred = scalingList->getScalingListVec( refId );

#define MINUS_14 -( 14 * 0 )   // TODO: where did the -14 from the standard go ? when it is there, it breaks decoding.

      scalingMatrixDcPred = refId > 13 ? scalingList->getScalingListDC( refId MINUS_14 ) : scalingList->getScalingListVec( id )[0];
    }
  }

  if( scalingListCopyModeFlag )
  {
    // When not present, the value of scaling_list_dc_coef[id − 14] is inferred to be equal to 0.
    if( id >= 14 )
    {
      scalingList->setScalingListDC( id MINUS_14, scalingMatrixDcPred );
    }
    return;
  }

  const auto& DiagScanOrder = g_scanOrder[SCAN_UNGROUPED];

  int nextCoef = 0;
  if( id > 13 )
  {
    X_READ_SVLC_idx( scaling_list_dc_coef, "[id-14]", -128, 127 );
    nextCoef += scaling_list_dc_coef;

    scalingList->setScalingListDC( id MINUS_14, ( scalingMatrixDcPred + scaling_list_dc_coef ) & 255 );
    CHECK( scalingList->getScalingListDC( id MINUS_14 ) <= 0, "The value of ScalingMatrixDcRec[" << id MINUS_14 << "] shall be greater than 0." )
  }

  for( unsigned i = 0; i < matrixSize * matrixSize; i++ )
  {
    const int numBits = g_sizeIdxInfo.idxFrom( matrixSize );
    const int x       = DiagScanOrder[3][3][i] & ( ( 1 << numBits ) - 1 );
    const int y       = DiagScanOrder[3][3][i] >> numBits;

    if( !( id > 25 && x >= 4 && y >= 4 ) )
    {
      X_READ_SVLC_idx( scaling_list_delta_coef, "[id][i]", -128, 127 );
      nextCoef += scaling_list_delta_coef;
    }

    int pos = DiagScanOrder[getLog2( matrixSize )][getLog2( matrixSize )][i];

    auto& scalingMatrixRec = scalingList->getScalingListVec( id );
    scalingMatrixRec[pos]  = ( scalingMatrixPred[pos] + nextCoef ) & 255;
    CHECK( scalingMatrixRec[pos] <= 0, "The value of ScalingMatrixRec[" << id << " ][x][y] shall be greater than 0." )
  }
}
bool HLSyntaxReader::xMoreRbspData()
{
  int bitsLeft = m_pcBitstream->getNumBitsLeft();

  // if there are more than 8 bits, it cannot be rbsp_trailing_bits
  if( bitsLeft > 8 )
  {
    return true;
  }

  uint8_t lastByte = m_pcBitstream->peekBits( bitsLeft );
  int cnt = bitsLeft;

  // remove trailing bits equal to zero
  while( ( cnt > 0 ) && ( ( lastByte & 1 ) == 0 ) )
  {
    lastByte >>= 1;
    cnt--;
  }
  // remove bit equal to one
  cnt--;

  // we should not have a negative number of bits
  CHECK( cnt<0, "Negative number of bits") ;

  // we have more data, if cnt is not zero
  return ( cnt>0 );
}


void HLSyntaxReader::alfFilterCoeffs( AlfSliceParam& alfSliceParam, const bool isChroma, const int altIdx )
{
  const bool isLuma = !isChroma;

  // derive maxGolombIdx
  const int numCoeff   = g_alfNumCoeff[isLuma];
  const int numFilters = isLuma ? alfSliceParam.numLumaFilters : 1;
  short*    coeff      = isLuma ? alfSliceParam.lumaCoeff      : alfSliceParam.chromaCoeff + altIdx * MAX_NUM_ALF_CHROMA_COEFF;
  short*    clipp      = isLuma ? alfSliceParam.lumaClipp      : alfSliceParam.chromaClipp + altIdx * MAX_NUM_ALF_CHROMA_COEFF;

  // Filter coefficients
  for( int sfIdx = 0; sfIdx < numFilters; ++sfIdx )
  {
    for( int j = 0; j < numCoeff - 1; j++ )
    {
      uint32_t code;
      READ_UVLC( code, isLuma ? "alf_luma_coeff_abs" : "alf_chroma_coeff_abs" );
      CHECK_READ_RANGE( code, 0, 128, ( isLuma ? "alf_luma_coeff_abs[sfIdx][j]" : "alf_chroma_coeff_abs[sfIdx][j]" ) );
      coeff[sfIdx * MAX_NUM_ALF_LUMA_COEFF + j] = code;

      if( code )
      {
        READ_FLAG( code, isLuma ? "alf_luma_coeff_sign" : "alf_chroma_coeff_sign" );

        const int sign = code ? -1 : 1;
        coeff[sfIdx * MAX_NUM_ALF_LUMA_COEFF + j] *= sign;

        CHECK_READ_RANGE( coeff[sfIdx * MAX_NUM_ALF_LUMA_COEFF + j],
                          -( 1 << 7 ), ( 1 << 7 ) - 1,
                          ( isLuma ? "AlfLumaCoeff[sfIdx * MAX_NUM_ALF_LUMA_COEFF + j]" : "AlfChromaCoeff[sfIdx * MAX_NUM_ALF_LUMA_COEFF + j]" ) );
      }
    }

    const int factor = 1 << ( AdaptiveLoopFilter::m_NUM_BITS - 1 );
    coeff[sfIdx * MAX_NUM_ALF_LUMA_COEFF + numCoeff - 1] = factor;
  }

  // Clipping values coding
  bool alfClipFlag = isLuma ? alfSliceParam.nonLinearFlagLuma : alfSliceParam.nonLinearFlagChroma;
  if( alfClipFlag )
  {
    // Filter coefficients
    for( int sfIdx = 0; sfIdx < numFilters; ++sfIdx )
    {
      for( int j = 0; j < numCoeff - 1; j++ )
      {
        uint32_t code;
        READ_CODE( 2, code, isLuma ? "alf_luma_clip_idx" : "alf_chroma_clip_idx" );
        clipp[sfIdx * MAX_NUM_ALF_LUMA_COEFF + j] = code;
      }
    }
  }
}

}   // namespace vvdec
