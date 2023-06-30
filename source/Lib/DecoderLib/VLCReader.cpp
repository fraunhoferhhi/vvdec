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

/** \file     VLCWReader.cpp
 *  \brief    Reader for high level syntax
 */

//! \ingroup DecoderLib
//! \{

#include "VLCReader.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/dtrace_next.h"
#include "CommonLib/AdaptiveLoopFilter.h"
#include "CommonLib/ParameterSetManager.h"

namespace vvdec
{

#if ENABLE_TRACING

void VLCReader::xReadCodeTr( uint32_t length, uint32_t& rValue, const char *pSymbolName )
{
  xReadCode( length, rValue );
  if( length < 10 )
  {
    DTRACE( g_trace_ctx, D_HEADER, "%-50s u(%d)  : %u\n", pSymbolName, length, rValue );
  }
  else
  {
    DTRACE( g_trace_ctx, D_HEADER, "%-50s u(%d) : %u\n", pSymbolName, length, rValue );
  }
}

void VLCReader::xReadUvlcTr( uint32_t& rValue, const char *pSymbolName )
{
  xReadUvlc( rValue );
  DTRACE( g_trace_ctx, D_HEADER, "%-50s ue(v) : %u\n", pSymbolName, rValue );
}

void VLCReader::xReadSvlcTr( int& rValue, const char *pSymbolName )
{
  xReadSvlc( rValue );
  DTRACE( g_trace_ctx, D_HEADER, "%-50s se(v) : %d\n", pSymbolName, rValue );
}

void VLCReader::xReadFlagTr( uint32_t& rValue, const char *pSymbolName )
{
  xReadFlag( rValue );
  DTRACE( g_trace_ctx, D_HEADER, "%-50s u(1)  : %d\n", pSymbolName, rValue );
}

void xTraceFillerData()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Filler Data ===========\n");
}

#endif

#if ENABLE_TRACING
void VLCReader::xReadSCode( uint32_t length, int& value, const char *pSymbolName )
#else
void VLCReader::xReadSCode( uint32_t length, int& value )
#endif
{
  uint32_t val;
  CHECK_RECOVERABLE( length == 0 || length > 31, "wrong" );
  m_pcBitstream->read( length, val );
  value = length >= 32 ? int( val ) : ( ( -int( val & ( uint32_t( 1 ) << ( length - 1 ) ) ) ) | int( val ) );

#if ENABLE_TRACING
  if( length < 10 )
  {
    DTRACE( g_trace_ctx, D_HEADER, "%-50s i(%d)  : %d\n", pSymbolName, length, value );
  }
  else
  {
    DTRACE( g_trace_ctx, D_HEADER, "%-50s i(%d) : %d\n", pSymbolName, length, value );
  }
#endif
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================
void VLCReader::xReadCode( uint32_t uiLength, uint32_t& ruiCode )
{
  CHECK_RECOVERABLE( uiLength == 0, "Reading a code of lenght '0'" );
  m_pcBitstream->read( uiLength, ruiCode );
}

void VLCReader::xReadUvlc( uint32_t& ruiVal )
{
  uint32_t uiVal = 0;
  uint32_t uiCode = 0;
  uint32_t uiLength;
  m_pcBitstream->read( 1, uiCode );

  if( 0 == uiCode )
  {
    uiLength = 0;

    while( ! ( uiCode & 1 ) )
    {
      m_pcBitstream->read( 1, uiCode );
      uiLength++;
    }

    m_pcBitstream->read( uiLength, uiVal );

    uiVal += ( 1 << uiLength ) - 1;
  }

  ruiVal = uiVal;
}

void VLCReader::xReadSvlc( int& riVal )
{
  uint32_t uiBits = 0;
  m_pcBitstream->read( 1, uiBits );
  if( 0 == uiBits )
  {
    uint32_t uiLength = 0;

    while( ! ( uiBits & 1 ) )
    {
      m_pcBitstream->read( 1, uiBits );
      uiLength++;
    }

    m_pcBitstream->read( uiLength, uiBits );

    uiBits += ( 1 << uiLength );
    riVal = ( uiBits & 1 ) ? -(int)( uiBits>>1 ) : (int)( uiBits>>1 );
  }
  else
  {
    riVal = 0;
  }
}

void VLCReader::xReadFlag( uint32_t& ruiCode )
{
  m_pcBitstream->read( 1, ruiCode );
}

void VLCReader::xReadRbspTrailingBits()
{
  uint32_t bit;
  READ_FLAG( bit, "rbsp_stop_one_bit" );
  CHECK_RECOVERABLE( bit!=1, "Trailing bit not '1'" );
  int cnt = 0;
  while( m_pcBitstream->getNumBitsUntilByteAligned() )
  {
    READ_FLAG( bit, "rbsp_alignment_zero_bit" );
    CHECK_RECOVERABLE( bit!=0, "Alignment bit is not '0'" );
    cnt++;
  }
  CHECK_RECOVERABLE( cnt >= 8, "Read more than '8' trailing bits" );
}

void AUDReader::parseAccessUnitDelimiter( InputBitstream* bs, uint32_t &picType )
{
  setBitstream( bs );

#if ENABLE_TRACING
  xTraceAccessUnitDelimiter();
#endif

  uint32_t audIrapOrGdrAuFlag;
  READ_FLAG( audIrapOrGdrAuFlag, "aud_irap_or_gdr_au_flag" ); //just a parsing fix, but not used TODO: check m_audIrapOrGdrAuFlag in VTM-10.0 reference software
  READ_CODE( 3, picType, "aud_pic_type" );
  xReadRbspTrailingBits();
}

void FDReader::parseFillerData( InputBitstream* bs, uint32_t &fdSize )
{
  setBitstream( bs );
#if ENABLE_TRACING
  xTraceFillerData();
#endif
  uint32_t ffByte;
  fdSize = 0;
  while( m_pcBitstream->getNumBitsLeft() > 8 )
  {
    READ_CODE( 8, ffByte, "fd_ff_byte" );
    CHECK_RECOVERABLE( ffByte != 0xff, "Invalid fillter data not '0xff'" );
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
  uint32_t code;
  READ_UVLC( code, "num_ref_entries[ listIdx ][ rplsIdx ]" );
  uint32_t numRefPic = code;
  uint32_t numStrp = 0;
  uint32_t numLtrp = 0;
  uint32_t numIlrp = 0;

  if( sps->getLongTermRefsPresent() && numRefPic > 0 && rplIdx != -1 )
  {
    READ_FLAG( code, "ltrp_in_header_flag[ listIdx ][ rplsIdx ]" );
    rpl->setLtrpInSliceHeaderFlag( code );
  }
  else if( sps->getLongTermRefsPresent() )
  {
    rpl->setLtrpInSliceHeaderFlag( 1 );
  }
  int prevDelta = MAX_INT;
  int deltaValue = 0;
  bool firstSTRP = true;
  rpl->setInterLayerPresentFlag( sps->getInterLayerPresentFlag() );
  for( int ii = 0; ii < numRefPic; ii++ )
  {
    uint32_t isInterLayerRefPic = 0;

    if( rpl->getInterLayerPresentFlag() )
    {
      READ_FLAG( isInterLayerRefPic, "inter_layer_ref_pic_flag[ listIdx ][ rplsIdx ][ i ]" );

      if( isInterLayerRefPic )
      {
        READ_UVLC( code, "ilrp_idx[ listIdx ][ rplsIdx ][ i ]" );
        rpl->setRefPicIdentifier( ii, 0, true, true, code );
        numIlrp++;
      }
    }

    if( !isInterLayerRefPic )
    {
      bool isLongTerm = false;
      if( sps->getLongTermRefsPresent() )
      {
        READ_FLAG( code, "st_ref_pic_flag[ listIdx ][ rplsIdx ][ i ]" );
        isLongTerm = ( code == 1 ) ? false : true;
      }

      if( !isLongTerm )
      {
        READ_UVLC( code, "abs_delta_poc_st[ listIdx ][ rplsIdx ][ i ]" );
        if( ( !sps->getUseWP() && !sps->getUseWPBiPred() ) || ii == 0 )
        {
          code++;
        }
        int readValue = code;
        if( readValue > 0 )
        {
          READ_FLAG( code, "strp_entry_sign_flag[ listIdx ][ rplsIdx ][ i ]" );
          if( code )
          {
            readValue = -readValue;
          }
        }
        if( firstSTRP )
        {
          firstSTRP = false;
          prevDelta = deltaValue = readValue;
        }
        else
        {
          deltaValue = prevDelta + readValue;
          prevDelta = deltaValue;
        }
        rpl->setRefPicIdentifier( ii, deltaValue, isLongTerm, false, 0 );
        numStrp++;
      }
      else
      {
        if( !rpl->getLtrpInSliceHeaderFlag() )
        {
          READ_CODE( sps->getBitsForPOC(), code, "rpls_poc_lsb_lt[listIdx][rplsIdx][i]" );
        }
        rpl->setRefPicIdentifier( ii, code, isLongTerm, false, 0 );
        numLtrp++;
      }
    }
  }
  rpl->setNumberOfShorttermPictures( numStrp );
  rpl->setNumberOfLongtermPictures( numLtrp );
  rpl->setNumberOfInterLayerPictures( numIlrp );
}

void HLSyntaxReader::parsePPS( PPS* pcPPS, const ParameterSetManager *parameterSetManager )
{
#if ENABLE_TRACING
  xTracePPSHeader ();
#endif
  uint32_t uiCode;

  int iCode;
  READ_CODE( 6, uiCode, "pps_pic_parameter_set_id" );                        pcPPS->setPPSId( uiCode );
  CHECK_RECOVERABLE( uiCode > 63, "PPS id exceeds boundary (63)" );

  READ_CODE( 4, uiCode, "pps_seq_parameter_set_id" );                        pcPPS->setSPSId( uiCode );
  const SPS* pcSPS = parameterSetManager->getSPS( pcPPS->getSPSId() );
  CHECK_RECOVERABLE( !pcSPS, "SPS with id " << pcPPS->getSPSId() << " missing." );

  READ_FLAG( uiCode, "pps_mixed_nalu_types_in_pic_flag" );                   pcPPS->setMixedNaluTypesInPicFlag( uiCode == 1 );

  READ_UVLC( uiCode, "pps_pic_width_in_luma_samples" );                      pcPPS->setPicWidthInLumaSamples( uiCode );
  READ_UVLC( uiCode, "pps_pic_height_in_luma_samples" );                     pcPPS->setPicHeightInLumaSamples( uiCode );

  READ_FLAG(uiCode, "pps_conformance_window_flag");                          pcPPS->setConformanceWindowPresentFlag( uiCode );
  if( uiCode != 0 )
  {
    Window &conf = pcPPS->getConformanceWindow();
    READ_UVLC( uiCode, "pps_conf_win_left_offset" );                        conf.setWindowLeftOffset( uiCode );
    READ_UVLC( uiCode, "pps_conf_win_right_offset" );                       conf.setWindowRightOffset( uiCode );
    READ_UVLC( uiCode, "pps_conf_win_top_offset" );                         conf.setWindowTopOffset( uiCode );
    READ_UVLC( uiCode, "pps_conf_win_bottom_offset" );                      conf.setWindowBottomOffset( uiCode );
  }

  READ_FLAG( uiCode, "pps_scaling_window_explicit_signalling_flag" );
  if( uiCode != 0 )
  {
    Window &scalingWindow = pcPPS->getScalingWindow();
    READ_SVLC( iCode, "pps_scaling_win_left_offset" );                      scalingWindow.setWindowLeftOffset( iCode );
    READ_SVLC( iCode, "pps_scaling_win_right_offset" );                     scalingWindow.setWindowRightOffset( iCode );
    READ_SVLC( iCode, "pps_scaling_win_top_offset" );                       scalingWindow.setWindowTopOffset( iCode );
    READ_SVLC( iCode, "pps_scaling_win_bottom_offset" );                    scalingWindow.setWindowBottomOffset( iCode );
  }
  else
  {
    pcPPS->setScalingWindow( pcPPS->getConformanceWindow() );
  }

  READ_FLAG( uiCode, "pps_output_flag_present_flag" );                      pcPPS->setOutputFlagPresentFlag( uiCode==1 );

  READ_FLAG( uiCode, "pps_no_pic_partition_flag");                          pcPPS->setNoPicPartitionFlag( uiCode == 1 );
  READ_FLAG( uiCode, "pps_subpic_id_mapping_present_flag" );                pcPPS->setSubPicIdMappingInPpsFlag( uiCode != 0 );
  if( pcPPS->getSubPicIdMappingInPpsFlag() )
  {
    if( !pcPPS->getNoPicPartitionFlag() )
    {
      READ_UVLC( uiCode, "pps_num_subpics_minus1" );                        pcPPS->setNumSubPics( uiCode + 1 );
    }
    else
    {
      pcPPS->setNumSubPics( 1 );
    }
    CHECK_RECOVERABLE( uiCode > MAX_NUM_SUB_PICS-1,  "Number of sub-pictures exceeds limit" );

    READ_UVLC( uiCode, "pps_subpic_id_len_minus1" );                        pcPPS->setSubPicIdLen( uiCode + 1 );
    CHECK_RECOVERABLE( uiCode > 15, "Invalid pps_subpic_id_len_minus1 signalled");

    CHECK_RECOVERABLE( ( 1 << pcPPS->getSubPicIdLen() ) < pcPPS->getNumSubPics(), "pps_subpic_id_len exceeds valid range" );
    for( int picIdx = 0; picIdx < pcPPS->getNumSubPics( ); picIdx++ )
    {
      READ_CODE( pcPPS->getSubPicIdLen(), uiCode, "pps_subpic_id[i]" );   pcPPS->setSubPicId( picIdx, uiCode );
    }
  }
  else
  {
    for( int picIdx = 0; picIdx < MAX_NUM_SUB_PICS; picIdx++ )
    {
      pcPPS->setSubPicId( picIdx, picIdx );
    }
  }


  if( !pcPPS->getNoPicPartitionFlag() )
  {
    int colIdx, rowIdx;
    pcPPS->resetTileSliceInfo(); //TODO: tbd

    // CTU size - required to match size in SPS
    READ_CODE( 2, uiCode, "pps_log2_ctu_size_minus5");                      pcPPS->setLog2CtuSize( uiCode + 5 );
    CHECK_RECOVERABLE( uiCode > 2, "pps_log2_ctu_size_minus5 must be less than or equal to 2" );

    // number of explicit tile columns/rows
    READ_UVLC( uiCode, "pps_num_exp_tile_columns_minus1" );                 pcPPS->setNumExpTileColumns( uiCode + 1 );
    READ_UVLC( uiCode, "pps_num_exp_tile_rows_minus1" );                    pcPPS->setNumExpTileRows( uiCode + 1 );
    CHECK_RECOVERABLE( pcPPS->getNumExpTileColumns() > MAX_TILE_COLS, "Number of explicit tile columns exceeds valid range" );

    // tile sizes
    for( colIdx = 0; colIdx < pcPPS->getNumExpTileColumns(); colIdx++ )
    {
      READ_UVLC( uiCode, "pps_tile_column_width_minus1[i]" );             pcPPS->addTileColumnWidth( uiCode + 1 );
      CHECK_RECOVERABLE( uiCode > ( pcPPS->getPicWidthInCtu() - 1 ), "The value of pps_tile_column_width_minus1[i] shall be in the range of 0 to PicWidthInCtbY-1, inclusive" );
    }
    for( rowIdx = 0; rowIdx < pcPPS->getNumExpTileRows(); rowIdx++ )
    {
      READ_UVLC( uiCode, "pps_tile_row_height_minus1[i]" );               pcPPS->addTileRowHeight( uiCode + 1 );
      CHECK_RECOVERABLE( uiCode > ( pcPPS->getPicHeightInCtu() - 1 ), "The value of pps_tile_row_height_minus shall be in the range of 0 to PicHeightInCtbY-1, inclusive" );
    }
    pcPPS->initTiles(); //TODO: tbd

    // rectangular slice signalling
    if( pcPPS->getNumTiles() > 1 )
    {
      READ_CODE( 1, uiCode, "pps_loop_filter_across_tiles_enabled_flag" );  pcPPS->setLoopFilterAcrossTilesEnabledFlag( uiCode == 1 );
      READ_CODE( 1, uiCode, "pps_rect_slice_flag" );                        pcPPS->setRectSliceFlag( uiCode == 1 ) ;
    }
    else
    {
      pcPPS->setLoopFilterAcrossTilesEnabledFlag( false );
      pcPPS->setRectSliceFlag( true );
    }
    if( pcPPS->getRectSliceFlag()  )
    {
      READ_FLAG( uiCode, "pps_single_slice_per_subpic_flag" );              pcPPS->setSingleSlicePerSubPicFlag( uiCode == 1 );
    }
    if( pcPPS->getRectSliceFlag() && !pcPPS->getSingleSlicePerSubPicFlag() )
    {
      int32_t tileIdx = 0;

      READ_UVLC( uiCode, "pps_num_slices_in_pic_minus1" );                  pcPPS->setNumSlicesInPic( uiCode + 1 );
      CHECK_RECOVERABLE( pcPPS->getNumSlicesInPic() > MAX_SLICES, "Number of slices in picture exceeds valid range");

      if( ( pcPPS->getNumSlicesInPic() - 1 ) > 1 )
      {
        READ_CODE( 1, uiCode, "pps_tile_idx_delta_present_flag" );          pcPPS->setTileIdxDeltaPresentFlag( uiCode == 1 ) ;
      }
      else
      {
        pcPPS->setTileIdxDeltaPresentFlag( 0 );
      }
      pcPPS->initRectSlices(); //TODO: tbd

      // read rectangular slice parameters
      for( int i = 0; i < pcPPS->getNumSlicesInPic()-1; i++ )
      {
        pcPPS->setSliceTileIdx( i, tileIdx );

        // complete tiles within a single slice
        if( ( tileIdx % pcPPS->getNumTileColumns() ) != pcPPS->getNumTileColumns() - 1 )
        {
          READ_UVLC( uiCode, "pps_slice_width_in_tiles_minus1[i]" );      pcPPS->setSliceWidthInTiles ( i, uiCode + 1 );
        }
        else
        {
          pcPPS->setSliceWidthInTiles( i, 1 );
        }

        if( tileIdx / pcPPS->getNumTileColumns() != pcPPS->getNumTileRows() - 1  &&
         ( pcPPS->getTileIdxDeltaPresentFlag() || tileIdx % pcPPS->getNumTileColumns() == 0 ) )
        {
          READ_UVLC( uiCode, "pps_slice_height_in_tiles_minus1[i]" );     pcPPS->setSliceHeightInTiles( i, uiCode + 1 );
        }
        else
        {
          if( ( tileIdx / pcPPS->getNumTileColumns() ) == pcPPS->getNumTileRows() - 1 )
          {
            pcPPS->setSliceHeightInTiles( i, 1 );
          }
          else
          {
            pcPPS->setSliceHeightInTiles( i, pcPPS->getSliceHeightInTiles( i - 1 ) );
          }
        }

        // multiple slices within a single tile special case
        if( pcPPS->getSliceWidthInTiles(i) == 1 && pcPPS->getSliceHeightInTiles(i) == 1 )
        {
          if( pcPPS->getTileRowHeight(tileIdx / pcPPS->getNumTileColumns()) > 1 )
          {
            READ_UVLC(uiCode, "pps_num_exp_slices_in_tile[i]");
            if( uiCode == 0 )
            {
              pcPPS->setNumSlicesInTile( i, 1 );
              pcPPS->setSliceHeightInCtu( i, pcPPS->getTileRowHeight( tileIdx / pcPPS->getNumTileColumns() ) );
            }
            else
            {
              uint32_t numExpSliceInTile = uiCode;
              uint32_t remTileRowHeight  = pcPPS->getTileRowHeight( tileIdx / pcPPS->getNumTileColumns() );
              int j = 0;
              for( ; j < numExpSliceInTile; j++ )
              {
                READ_UVLC( uiCode, "pps_exp_slice_height_in_ctus_minus1[i]" );
                pcPPS->setSliceHeightInCtu( i + j, uiCode + 1 );
                remTileRowHeight -= ( uiCode + 1 );
              }

              uint32_t uniformSliceHeight = uiCode + 1;
              while( remTileRowHeight >= uniformSliceHeight )
              {
                pcPPS->setSliceHeightInCtu( i + j, uniformSliceHeight );
                remTileRowHeight -= uniformSliceHeight;
                j++;
              }
              if( remTileRowHeight > 0 )
              {
                pcPPS->setSliceHeightInCtu( i + j, remTileRowHeight );
                j++;
              }
              for( int k = 0; k < j; k++ )
              {
                pcPPS->setNumSlicesInTile( i + k, j );
                pcPPS->setSliceWidthInTiles( i + k, 1 );
                pcPPS->setSliceHeightInTiles( i + k, 1 );
                pcPPS->setSliceTileIdx( i + k, tileIdx );
              }
              i += (j - 1);
            }
          }
          else
          {
            pcPPS->setNumSlicesInTile( i, 1 );
            pcPPS->setSliceHeightInCtu( i, pcPPS->getTileRowHeight( tileIdx / pcPPS->getNumTileColumns() ) );
          }
        }
        // tile index offset to start of next slice
        if( i < pcPPS->getNumSlicesInPic()-1 )
        {
          if( pcPPS->getTileIdxDeltaPresentFlag() )
          {
            int32_t  tileIdxDelta;
            READ_SVLC( tileIdxDelta, "pps_tile_idx_delta_val[i]" );
            tileIdx += tileIdxDelta;
            CHECK_RECOVERABLE( tileIdx < 0 || tileIdx >= pcPPS->getNumTiles(), "Invalid tile_idx_delta." );
          }
          else
          {
            tileIdx += pcPPS->getSliceWidthInTiles( i );
            if( tileIdx % pcPPS->getNumTileColumns() == 0)
            {
              tileIdx += (pcPPS->getSliceHeightInTiles( i ) - 1) * pcPPS->getNumTileColumns();
            }
          }
        }
      }
      pcPPS->setSliceTileIdx(pcPPS->getNumSlicesInPic()-1, tileIdx );

      // initialize mapping between rectangular slices and CTUs
//      pcPPS->initRectSliceMap(); //TODO: tbd
    }

    if (pcPPS->getRectSliceFlag() == 0 || pcPPS->getSingleSlicePerSubPicFlag() || pcPPS->getNumSlicesInPic() > 1)
    {
      READ_CODE( 1, uiCode, "pps_loop_filter_across_slices_enabled_flag");  pcPPS->setLoopFilterAcrossSlicesEnabledFlag( uiCode == 1 );
    }
    else
    {
      pcPPS->setLoopFilterAcrossSlicesEnabledFlag( false );
    }
  }
  else
  {
    pcPPS->setSingleSlicePerSubPicFlag(1);
  }
  READ_FLAG( uiCode, "pps_cabac_init_present_flag" );                       pcPPS->setCabacInitPresentFlag( uiCode ? true : false );

  READ_UVLC( uiCode, "pps_num_ref_idx_default_active_minus1[0]" );          pcPPS->setNumRefIdxL0DefaultActive( uiCode + 1 );
  CHECK_RECOVERABLE( uiCode > 14, "Invalid code read" );

  READ_UVLC( uiCode, "pps_num_ref_idx_default_active_minus1[1]" );          pcPPS->setNumRefIdxL1DefaultActive( uiCode + 1 );
  CHECK_RECOVERABLE( uiCode > 14, "Invalid code read" );

  READ_FLAG( uiCode, "pps_rpl1_idx_present_flag" );                         pcPPS->setRpl1IdxPresentFlag( uiCode );
  READ_FLAG( uiCode, "pps_weighted_pred_flag" );                            pcPPS->setUseWP( uiCode == 1 );
  READ_FLAG( uiCode, "pps_weighted_bipred_flag" );                          pcPPS->setWPBiPred( uiCode == 1 );
  READ_FLAG( uiCode, "pps_ref_wraparound_enabled_flag" );                   pcPPS->setUseWrapAround( uiCode ? true : false );
  if( pcPPS->getUseWrapAround() )
  {
    READ_UVLC( uiCode, "pps_pic_width_minus_wraparound_offset" );           pcPPS->setPicWidthMinusWrapAroundOffset( uiCode );
  }
  else
  {
    pcPPS->setPicWidthMinusWrapAroundOffset( 0 );
  }

  READ_SVLC( iCode, "pps_init_qp_minus26" );                                pcPPS->setPicInitQPMinus26( iCode );

  READ_FLAG( uiCode, "pps_cu_qp_delta_enabled_flag" );                      pcPPS->setUseDQP( uiCode ? true : false );
  READ_FLAG( uiCode, "pps_chroma_tool_offsets_present_flag" );              pcPPS->setPPSChromaToolFlag( uiCode ? true : false );
  if( pcPPS->getPPSChromaToolFlag() )
  {
    READ_SVLC( iCode, "pps_cb_qp_offset" );                                 pcPPS->setQpOffset( COMPONENT_Cb, iCode );
    CHECK_RECOVERABLE( pcPPS->getQpOffset( COMPONENT_Cb ) < -12, "Invalid Cb QP offset" );
    CHECK_RECOVERABLE( pcPPS->getQpOffset( COMPONENT_Cb ) > 12, "Invalid Cb QP offset" );

    READ_SVLC( iCode, "pps_cr_qp_offset" );                                 pcPPS->setQpOffset( COMPONENT_Cr, iCode );
    CHECK_RECOVERABLE( pcPPS->getQpOffset( COMPONENT_Cr ) < -12, "Invalid Cr QP offset" );
    CHECK_RECOVERABLE( pcPPS->getQpOffset( COMPONENT_Cr ) > 12, "Invalid Cr QP offset" );

    READ_FLAG( uiCode, "pps_joint_cbcr_qp_offset_present_flag" );           pcPPS->setJointCbCrQpOffsetPresentFlag( uiCode ? true : false );

    if( pcPPS->getJointCbCrQpOffsetPresentFlag() )
    {
      READ_SVLC( iCode, "pps_joint_cbcr_qp_offset_value" );
    }
    else
    {
      iCode = 0;
    }
    pcPPS->setQpOffset( JOINT_CbCr, iCode );

    CHECK_RECOVERABLE( pcPPS->getQpOffset( JOINT_CbCr ) < -12, "Invalid CbCr QP offset" );
    CHECK_RECOVERABLE( pcPPS->getQpOffset( JOINT_CbCr ) > 12, "Invalid CbCr QP offset" );

    CHECK_RECOVERABLE( MAX_NUM_COMPONENT > 3, "Invalid maximal number of components" );

    READ_FLAG( uiCode, "pps_slice_chroma_qp_offsets_present_flag" );        pcPPS->setSliceChromaQpFlag( uiCode ? true : false );

    READ_FLAG( uiCode, "pps_cu_chroma_qp_offset_list_enabled_flag" );
    if( uiCode == 0 )
    {
      pcPPS->clearChromaQpOffsetList();
    }
    else
    {
      uint32_t tableSizeMinus1 = 0;
      READ_UVLC( tableSizeMinus1, "pps_chroma_qp_offset_list_len_minus1" );
      CHECK_RECOVERABLE( tableSizeMinus1 >= MAX_QP_OFFSET_LIST_SIZE, "Table size exceeds maximum" );

      for( int cuChromaQpOffsetIdx = 0; cuChromaQpOffsetIdx <= (tableSizeMinus1); cuChromaQpOffsetIdx++ )
      {
        int cbOffset;
        int crOffset;
        int jointCbCrOffset;
        READ_SVLC( cbOffset, "pps_cb_qp_offset_list[i] ");
        CHECK_RECOVERABLE( cbOffset < -12 || cbOffset > 12, "Invalid chroma QP offset" );
        READ_SVLC( crOffset, "pps_cr_qp_offset_list[i]" );
        CHECK_RECOVERABLE( crOffset < -12 || crOffset > 12, "Invalid chroma QP offset" );
        if( pcPPS->getJointCbCrQpOffsetPresentFlag() )
        {
          READ_SVLC( jointCbCrOffset, "pps_joint_cbcr_qp_offset_list[i]" );
        }
        else
        {
          jointCbCrOffset = 0;
        }
        CHECK_RECOVERABLE( jointCbCrOffset < -12 || jointCbCrOffset > 12, "Invalid chroma QP offset" );
        // table uses +1 for index (see comment inside the function)
        pcPPS->setChromaQpOffsetListEntry( cuChromaQpOffsetIdx + 1, cbOffset, crOffset, jointCbCrOffset );
      }
      CHECK_RECOVERABLE( pcPPS->getChromaQpOffsetListLen() != tableSizeMinus1 + 1, "Invalid chroma QP offset list length" );
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
  READ_FLAG( uiCode, "pps_deblocking_filter_control_present_flag" );        pcPPS->setDeblockingFilterControlPresentFlag( uiCode ? true : false );
  if( pcPPS->getDeblockingFilterControlPresentFlag() )
  {
    READ_FLAG( uiCode, "pps_deblocking_filter_override_enabled_flag" );     pcPPS->setDeblockingFilterOverrideEnabledFlag( uiCode ? true : false );
    READ_FLAG( uiCode, "pps_deblocking_filter_disabled_flag" );             pcPPS->setPPSDeblockingFilterDisabledFlag( uiCode ? true : false );
    if( !pcPPS->getNoPicPartitionFlag() && pcPPS->getDeblockingFilterOverrideEnabledFlag() )
    {
      READ_FLAG( uiCode, "pps_dbf_info_in_ph_flag" );                       pcPPS->setDbfInfoInPhFlag( uiCode ? true : false );
    }
    else
    {
      pcPPS->setDbfInfoInPhFlag( false );
    }
    if( !pcPPS->getPPSDeblockingFilterDisabledFlag() )
    {
      READ_SVLC( iCode, "pps_luma_beta_offset_div2" );                      pcPPS->setDeblockingFilterBetaOffsetDiv2( iCode );
      CHECK_RECOVERABLE( pcPPS->getDeblockingFilterBetaOffsetDiv2() < -12 || pcPPS->getDeblockingFilterBetaOffsetDiv2() > 12,
             "Invalid deblocking filter configuration" );

      READ_SVLC( iCode, "pps_luma tc_offset_div2" );                        pcPPS->setDeblockingFilterTcOffsetDiv2( iCode );
      CHECK_RECOVERABLE( pcPPS->getDeblockingFilterTcOffsetDiv2() < -12 || pcPPS->getDeblockingFilterTcOffsetDiv2() > 12,
             "Invalid deblocking filter configuration" );

      if( pcPPS->getPPSChromaToolFlag() )
      {
        READ_SVLC( iCode, "pps_cb_beta_offset_div2" );                      pcPPS->setDeblockingFilterCbBetaOffsetDiv2( iCode );
        CHECK_RECOVERABLE( pcPPS->getDeblockingFilterCbBetaOffsetDiv2() < -12 || pcPPS->getDeblockingFilterCbBetaOffsetDiv2() > 12,
               "Invalid deblocking filter configuration" );

        READ_SVLC( iCode, "pps_cb_tc_offset_div2" );                        pcPPS->setDeblockingFilterCbTcOffsetDiv2( iCode );
        CHECK_RECOVERABLE( pcPPS->getDeblockingFilterCbTcOffsetDiv2() < -12 || pcPPS->getDeblockingFilterCbTcOffsetDiv2() > 12,
               "Invalid deblocking filter configuration" );

        READ_SVLC( iCode, "pps_cr_beta_offset_div2" );                      pcPPS->setDeblockingFilterCrBetaOffsetDiv2( iCode );
        CHECK_RECOVERABLE( pcPPS->getDeblockingFilterCrBetaOffsetDiv2() < -12 || pcPPS->getDeblockingFilterCrBetaOffsetDiv2() > 12,
               "Invalid deblocking filter configuration" );

        READ_SVLC( iCode, "pps_cr_tc_offset_div2" );                        pcPPS->setDeblockingFilterCrTcOffsetDiv2( iCode );
        CHECK_RECOVERABLE( pcPPS->getDeblockingFilterCrTcOffsetDiv2() < -12 || pcPPS->getDeblockingFilterCrTcOffsetDiv2() > 12,
               "Invalid deblocking filter configuration" );
      }
      else
      {
        pcPPS->setDeblockingFilterCbBetaOffsetDiv2 ( pcPPS->getDeblockingFilterBetaOffsetDiv2() );
        pcPPS->setDeblockingFilterCbTcOffsetDiv2   ( pcPPS->getDeblockingFilterTcOffsetDiv2()   );
        pcPPS->setDeblockingFilterCrBetaOffsetDiv2 ( pcPPS->getDeblockingFilterBetaOffsetDiv2() );
        pcPPS->setDeblockingFilterCrTcOffsetDiv2   ( pcPPS->getDeblockingFilterTcOffsetDiv2()   );
      }
    }
  }
  else
  {
    pcPPS->setDeblockingFilterOverrideEnabledFlag( false );
    pcPPS->setDbfInfoInPhFlag( false );
  }

  if( !pcPPS->getNoPicPartitionFlag() )
  {
    READ_FLAG( uiCode, "pps_rpl_info_in_ph_flag" );                         pcPPS->setRplInfoInPhFlag( uiCode ? true : false );
    READ_FLAG( uiCode, "pps_sao_info_in_ph_flag" );                         pcPPS->setSaoInfoInPhFlag( uiCode ? true : false );
    READ_FLAG( uiCode, "pps_alf_info_in_ph_flag" );                         pcPPS->setAlfInfoInPhFlag( uiCode ? true : false );
    if( (pcPPS->getUseWP() || pcPPS->getWPBiPred()) && pcPPS->getRplInfoInPhFlag() )
    {
      READ_FLAG( uiCode, "pps_wp_info_in_ph_flag" );                        pcPPS->setWpInfoInPhFlag( uiCode ? true : false );
    }
    else
    {
      pcPPS->setWpInfoInPhFlag( false );
    }
    READ_FLAG( uiCode, "pps_qp_delta_info_in_ph_flag" );                    pcPPS->setQpDeltaInfoInPhFlag( uiCode ? true : false );
  }
  else
  {
    pcPPS->setRplInfoInPhFlag( false );
    pcPPS->setSaoInfoInPhFlag( false );
    pcPPS->setAlfInfoInPhFlag( false );
    pcPPS->setWpInfoInPhFlag( false );
    pcPPS->setQpDeltaInfoInPhFlag( false );
  }


  READ_FLAG( uiCode, "pps_picture_header_extension_present_flag" );         pcPPS->setPictureHeaderExtensionPresentFlag( uiCode );
  READ_FLAG( uiCode, "pps_slice_header_extension_present_flag" );           pcPPS->setSliceHeaderExtensionPresentFlag( uiCode );

  READ_FLAG( uiCode, "pps_extension_flag" );
  if( uiCode )
  {
    while( xMoreRbspData() )
    {
      READ_FLAG( uiCode, "pps_extension_data_flag" );
    }
  }

  xReadRbspTrailingBits();

  if( pcPPS->getPicWidthInLumaSamples() == pcSPS->getMaxPicWidthInLumaSamples() && pcPPS->getPicHeightInLumaSamples() == pcSPS->getMaxPicHeightInLumaSamples() )
  {
    CHECK_RECOVERABLE( pcPPS->getConformanceWindowPresentFlag(),
      "When pps_pic_width_in_luma_samples is equal to sps_pic_width_max_in_luma_samples and "
      "pps_pic_height_in_luma_samples is equal to sps_pic_height_max_in_luma_samples, the value of "
      "pps_conformance_window_flag shall be equal to 0" );

    pcPPS->setConformanceWindow( pcSPS->getConformanceWindow() );

    if( !pcPPS->getScalingWindow().getWindowEnabledFlag() )
    {
      pcPPS->setScalingWindow( pcPPS->getConformanceWindow() );
    }
  }

  pcPPS->finalizePPSPartitioning( pcSPS );

  // set wraparound offset from PPS and SPS info
  int minCbSizeY = ( 1 << pcSPS->getLog2MinCodingBlockSize() );
  CHECK_RECOVERABLE( !pcSPS->getUseWrapAround() && pcPPS->getUseWrapAround(), "When sps_ref_wraparound_enabled_flag is equal to 0, the value of pps_ref_wraparound_enabled_flag shall be equal to 0." );
  CHECK_RECOVERABLE( ( ( ( pcSPS->getCTUSize() / minCbSizeY ) + 1 ) > ( ( pcPPS->getPicWidthInLumaSamples() / minCbSizeY ) - 1 ) ) && pcPPS->getUseWrapAround(), "When the value of CtbSizeY / MinCbSizeY + 1 is greater than pic_width_in_luma_samples / MinCbSizeY - 1, the value of pps_ref_wraparound_enabled_flag shall be equal to 0." );
  if( pcPPS->getUseWrapAround() )
  {
    CHECK_RECOVERABLE( ( pcPPS->getPicWidthMinusWrapAroundOffset() > ( pcPPS->getPicWidthInLumaSamples() / minCbSizeY - pcSPS->getCTUSize() / minCbSizeY - 2 ) ), "pps_pic_width_minus_wraparound_ofsfet shall be less than or equal to pps_pic_width_in_luma_samples/MinCbSizeY - CtbSizeY/MinCbSizeY-2" );
    pcPPS->setWrapAroundOffset( minCbSizeY * ( pcPPS->getPicWidthInLumaSamples() / minCbSizeY - pcPPS->getPicWidthMinusWrapAroundOffset() ) );
  }
  else
  {
    pcPPS->setWrapAroundOffset( 0 );
  }

  pcPPS->pcv = std::make_unique<PreCalcValues>( *pcSPS, *pcPPS );
}

void HLSyntaxReader::parseAPS( APS* aps )
{
#if ENABLE_TRACING
  xTraceAPSHeader();
#endif

  uint32_t  code;
  READ_CODE( 3, code, "aps_params_type" );                                  aps->setAPSType( code );
  READ_CODE( 5, code, "adaptation_parameter_set_id" );                      aps->setAPSId( code );
  READ_FLAG( code, "aps_chroma_present_flag" );                             aps->chromaPresentFlag = code;

  const int apsType = aps->getAPSType();

  if( apsType == ALF_APS )
  {
    parseAlfAps( aps );
  }
  else if( apsType == LMCS_APS )
  {
    parseLmcsAps( aps );
  }
  else if( apsType == SCALING_LIST_APS )
  {
    parseScalingListAps( aps );
  }

  READ_FLAG( code, "aps_extension_flag" );
  if( code )
  {
    while( xMoreRbspData() )
    {
      READ_FLAG( code, "aps_extension_data_flag" );
    }
  }
  xReadRbspTrailingBits();
}

void HLSyntaxReader::parseAlfAps( APS* aps )
{
  uint32_t  code;

  AlfSliceParam& param = aps->getAlfAPSParam();
  param.reset();
  param.enabledFlag[COMPONENT_Y] = param.enabledFlag[COMPONENT_Cb] = param.enabledFlag[COMPONENT_Cr] = true;
  READ_FLAG( code, "alf_luma_new_filter" );                                 param.newFilterFlag[CHANNEL_TYPE_LUMA] = code;

  if( aps->chromaPresentFlag )
  {
    READ_FLAG( code, "alf_chroma_new_filter" );                             param.newFilterFlag[CHANNEL_TYPE_CHROMA] = code;
  }
  else
  {
    param.newFilterFlag[CHANNEL_TYPE_CHROMA] = 0;
  }

  CcAlfFilterParam ccAlfParam = aps->getCcAlfAPSParam();
  if( aps->chromaPresentFlag )
  {
    READ_FLAG( code, "alf_cc_cb_filter_signal_flag" );                      ccAlfParam.newCcAlfFilter[COMPONENT_Cb - 1] = code;
  }
  else
  {
    ccAlfParam.newCcAlfFilter[COMPONENT_Cb - 1] = 0;
  }
  if( aps->chromaPresentFlag )
  {
    READ_FLAG( code, "alf_cc_cr_filter_signal_flag" );                      ccAlfParam.newCcAlfFilter[COMPONENT_Cr - 1] = code;
  }
  else
  {
    ccAlfParam.newCcAlfFilter[COMPONENT_Cr - 1] = 0;
  }
  CHECK_RECOVERABLE( param.newFilterFlag[CHANNEL_TYPE_LUMA] == 0 && param.newFilterFlag[CHANNEL_TYPE_CHROMA] == 0 && ccAlfParam.newCcAlfFilter[COMPONENT_Cb - 1] == 0
           && ccAlfParam.newCcAlfFilter[COMPONENT_Cr - 1] == 0,
         "bitstream conformance error: one of alf_luma_filter_signal_flag, alf_chroma_filter_signal_flag, "
         "alf_cross_component_cb_filter_signal_flag, and alf_cross_component_cr_filter_signal_flag shall be nonzero" );

  if( param.newFilterFlag[CHANNEL_TYPE_LUMA] )
  {
    READ_FLAG( code, "alf_luma_clip" );                                     param.nonLinearFlagLuma = code ? true : false;
    READ_UVLC( code, "alf_luma_num_filters_signalled_minus1" );             param.numLumaFilters = code + 1;
    if( param.numLumaFilters > 1 )
    {
      const int length = (int)ceil( log2( param.numLumaFilters ) );
      for( int i = 0; i < MAX_NUM_ALF_CLASSES; i++ )
      {
        READ_CODE( length, code, "alf_luma_coeff_delta_idx" );              param.filterCoeffDeltaIdx[i] = code;
      }
    }
    else
    {
      memset( param.filterCoeffDeltaIdx, 0, sizeof( param.filterCoeffDeltaIdx ) );
    }
    alfFilter( param, false, 0 );
  }
  if( param.newFilterFlag[CHANNEL_TYPE_CHROMA] )
  {
    READ_FLAG( code, "alf_nonlinear_enable_flag_chroma" );                  param.nonLinearFlagChroma = code ? true : false;

    if( MAX_NUM_ALF_ALTERNATIVES_CHROMA > 1 )
    {
      READ_UVLC( code, "alf_chroma_num_alts_minus1" );
    }
    else
    {
      code = 0;
    }
    param.numAlternativesChroma = code + 1;

    for( int altIdx=0; altIdx < param.numAlternativesChroma; ++altIdx )
    {
      alfFilter( param, true, altIdx );
    }
  }

  for( int ccIdx = 0; ccIdx < 2; ccIdx++ )
  {
    if( ccAlfParam.newCcAlfFilter[ccIdx] )
    {
      if( MAX_NUM_CC_ALF_FILTERS > 1 )
      {
        READ_UVLC( code, ccIdx == 0 ? "alf_cc_cb_filters_signalled_minus1" : "alf_cc_cr_filters_signalled_minus1" );
      }
      else
      {
        code = 0;
      }
      ccAlfParam.ccAlfFilterCount[ccIdx] = code + 1;

      for( int filterIdx = 0; filterIdx < ccAlfParam.ccAlfFilterCount[ccIdx]; filterIdx++ )
      {
        ccAlfParam.ccAlfFilterIdxEnabled[ccIdx][filterIdx] = true;
        const int numCoeff = g_alfNumCoeff[CC_ALF];

        short *coeff = ccAlfParam.ccAlfCoeff[ccIdx][filterIdx];
        // Filter coefficients
        for( int i = 0; i < numCoeff - 1; i++ )
        {
          READ_CODE( CCALF_BITS_PER_COEFF_LEVEL, code, ccIdx == 0 ? "alf_cc_cb_mapped_coeff_abs" : "alf_cc_cr_mapped_coeff_abs" );
          if( code == 0 )
          {
            coeff[i] = 0;
          }
          else
          {
            coeff[i] = 1 << (code - 1);
            READ_FLAG( code, ccIdx == 0 ? "alf_cc_cb_coeff_sign" : "alf_cc_cr_coeff_sign" );
            coeff[i] *= 1 - 2 * code;
          }
        }

        DTRACE( g_trace_ctx, D_SYNTAX, "%s coeff filterIdx %d: ", ccIdx == 0 ? "Cb" : "Cr", filterIdx );
        for( int i = 0; i < numCoeff; i++ )
        {
          DTRACE( g_trace_ctx, D_SYNTAX, "%d ", coeff[i] );
        }
        DTRACE( g_trace_ctx, D_SYNTAX, "\n" );
      }

      for( int filterIdx = ccAlfParam.ccAlfFilterCount[ccIdx]; filterIdx < MAX_NUM_CC_ALF_FILTERS; filterIdx++ )
      {
        ccAlfParam.ccAlfFilterIdxEnabled[ccIdx][filterIdx] = false;
      }
    }
  }
  aps->setCcAlfAPSParam( ccAlfParam );
}

void HLSyntaxReader::parseLmcsAps( APS* aps )
{
  uint32_t  code;

  SliceReshapeInfo& info = aps->getReshaperAPSInfo();
  memset( info.reshaperModelBinCWDelta, 0, PIC_CODE_CW_BINS * sizeof(int) );
  READ_UVLC( code, "lmcs_min_bin_idx" );                                    info.reshaperModelMinBinIdx = code;
  READ_UVLC( code, "lmcs_delta_max_bin_idx" );                              info.reshaperModelMaxBinIdx = PIC_CODE_CW_BINS - 1 - code;
  READ_UVLC( code, "lmcs_delta_cw_prec_minus1" );                           info.maxNbitsNeededDeltaCW = code + 1;
  CHECK_RECOVERABLE( info.maxNbitsNeededDeltaCW == 0, "wrong" );

  for( uint32_t i = info.reshaperModelMinBinIdx; i <= info.reshaperModelMaxBinIdx; i++ )
  {
    READ_CODE( info.maxNbitsNeededDeltaCW, code, "lmcs_delta_abs_cw[ i ]" );
    int absCW = code;
    if( absCW > 0 )
    {
      READ_CODE( 1, code, "lmcs_delta_sign_cw_flag[ i ]" );
    }
    int signCW = code;
    info.reshaperModelBinCWDelta[i] = ( 1 - 2 * signCW ) * absCW;
  }
  if( aps->chromaPresentFlag )
  {
    READ_CODE( 3, code, "lmcs_delta_abs_crs" );
  }
  int absCW = aps->chromaPresentFlag ? code : 0;
  if( absCW > 0 )
  {
    READ_CODE( 1, code, "lmcs_delta_sign_crs_flag" );
  }
  int signCW = code;
  info.chrResScalingOffset = ( 1 - 2 * signCW ) * absCW;

  aps->setReshaperAPSInfo( info );
}

void HLSyntaxReader::parseScalingListAps( APS* aps )
{
  ScalingList& info = aps->getScalingList();
  parseScalingList( &info, aps->chromaPresentFlag );
}

void  HLSyntaxReader::parseVUI( VUI* pcVUI, SPS *pcSPS )
{
#if ENABLE_TRACING
  DTRACE( g_trace_ctx, D_HEADER, "----------- vui_parameters -----------\n");
#endif
  unsigned vuiPayloadSize = pcSPS->getVuiPayloadSize();
  InputBitstream *bs = getBitstream();
  setBitstream( bs->extractSubstream( vuiPayloadSize * 8 ) );

  uint32_t  symbol;

  READ_FLAG( symbol, "vui_general_progressive_source_flag" );                pcVUI->setProgressiveSourceFlag( symbol ? true : false );
  READ_FLAG( symbol, "vui_general_interlaced_source_flag" );                 pcVUI->setInterlacedSourceFlag( symbol ? true : false );
  READ_FLAG( symbol, "vui_non_packed_constraint_flag" );                     pcVUI->setNonPackedFlag( symbol ? true : false );
  READ_FLAG( symbol, "vui_non_projected_constraint_flag" );                  pcVUI->setNonProjectedFlag( symbol ? true : false );
  READ_FLAG( symbol, "vui_aspect_ratio_info_present_flag" );                 pcVUI->setAspectRatioInfoPresentFlag( symbol );
  if( pcVUI->getAspectRatioInfoPresentFlag() )
  {
    READ_FLAG( symbol, "vui_aspect_ratio_constant_flag" );                   pcVUI->setAspectRatioConstantFlag( symbol );
    READ_CODE( 8, symbol, "vui_aspect_ratio_idc" );                          pcVUI->setAspectRatioIdc( symbol );
    if( pcVUI->getAspectRatioIdc() == 255 )
    {
      READ_CODE( 16, symbol, "vui_sar_width" );                              pcVUI->setSarWidth( symbol );
      READ_CODE( 16, symbol, "vui_sar_height" );                             pcVUI->setSarHeight( symbol );
    }
  }

  READ_FLAG( symbol, "vui_overscan_info_present_flag" );                     pcVUI->setOverscanInfoPresentFlag( symbol );
  if( pcVUI->getOverscanInfoPresentFlag() )
  {
    READ_FLAG( symbol, "vui_overscan_appropriate_flag" );                    pcVUI->setOverscanAppropriateFlag( symbol );
  }

  READ_FLAG( symbol, "vui_colour_description_present_flag" );                pcVUI->setColourDescriptionPresentFlag( symbol );
  if( pcVUI->getColourDescriptionPresentFlag() )
  {
    READ_CODE (8, symbol, "vui_colour_primaries" );                          pcVUI->setColourPrimaries( symbol );
    READ_CODE (8, symbol, "vui_transfer_characteristics" );                  pcVUI->setTransferCharacteristics( symbol );
    READ_CODE( 8, symbol, "vui_matrix_coeffs" );                             pcVUI->setMatrixCoefficients( symbol );
    READ_FLAG( symbol, "vui_video_full_range_flag" );                        pcVUI->setVideoFullRangeFlag( symbol);
  }

  READ_FLAG( symbol, "vui_chroma_loc_info_present_flag" );                   pcVUI->setChromaLocInfoPresentFlag( symbol );
  if( pcVUI->getChromaLocInfoPresentFlag() )
  {
    if( pcVUI->getProgressiveSourceFlag() && !pcVUI->getInterlacedSourceFlag() )
    {
      READ_UVLC( symbol, "vui_chroma_sample_loc_type" );                     pcVUI->setChromaSampleLocType( symbol );
    }
    else
    {
      READ_UVLC( symbol, "vui_chroma_sample_loc_type_top_field" );           pcVUI->setChromaSampleLocTypeTopField( symbol );
      READ_UVLC( symbol, "vui_chroma_sample_loc_type_bottom_field" );        pcVUI->setChromaSampleLocTypeBottomField( symbol );
    }
  }

  int payloadBitsRem = getBitstream()->getNumBitsLeft();
  if( payloadBitsRem )      //Corresponds to more_data_in_payload()
  {
    while( payloadBitsRem > 9 )    //payload_extension_present()
    {
      READ_CODE( 1, symbol, "vui_reserved_payload_extension_data" );
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
      READ_CODE( 1, symbol, "vui_reserved_payload_extension_data" );
      payloadBitsRem--;
    }
    READ_FLAG( symbol, "vui_payload_bit_equal_to_one" );
    CHECK_RECOVERABLE( symbol != 1, "vui_payload_bit_equal_to_one not equal to 1" );
    payloadBitsRem--;
    while( payloadBitsRem )
    {
      READ_FLAG( symbol, "vui_payload_bit_equal_to_zero" );
      CHECK_RECOVERABLE( symbol != 0, "vui_payload_bit_equal_to_zero not equal to 0" );
      payloadBitsRem--;
    }
  }
  delete getBitstream();
  setBitstream(bs);
}

void HLSyntaxReader::parseGeneralHrdParameters( GeneralHrdParams *hrd )
{
  uint32_t  symbol;
  READ_CODE( 32, symbol, "num_units_in_tick" );                              hrd->setNumUnitsInTick( symbol );
  READ_CODE( 32, symbol, "time_scale" );                                     hrd->setTimeScale( symbol );
  READ_FLAG( symbol, "general_nal_hrd_parameters_present_flag" );            hrd->setGeneralNalHrdParametersPresentFlag( symbol == 1 ? true : false );
  READ_FLAG( symbol, "general_vcl_hrd_parameters_present_flag" );            hrd->setGeneralVclHrdParametersPresentFlag( symbol == 1 ? true : false );

  if( hrd->getGeneralNalHrdParametersPresentFlag() || hrd->getGeneralVclHrdParametersPresentFlag() )
  {
    READ_FLAG( symbol, "general_same_pic_timing_in_all_ols_flag" );          hrd->setGeneralSamePicTimingInAllOlsFlag( symbol == 1 ? true : false );
    READ_FLAG( symbol, "general_decoding_unit_hrd_params_present_flag" );    hrd->setGeneralDecodingUnitHrdParamsPresentFlag( symbol == 1 ? true : false );
    if( hrd->getGeneralDecodingUnitHrdParamsPresentFlag() )
    {
      READ_CODE( 8, symbol, "tick_divisor_minus2" );                         hrd->setTickDivisorMinus2( symbol );
    }
    READ_CODE( 4, symbol, "bit_rate_scale" );                                hrd->setBitRateScale( symbol );
    READ_CODE( 4, symbol, "cpb_size_scale" );                                hrd->setCpbSizeScale( symbol );
    if( hrd->getGeneralDecodingUnitHrdParamsPresentFlag() )
    {
      READ_CODE( 4, symbol, "cpb_size_du_scale" );                           hrd->setCpbSizeDuScale( symbol );
    }
    READ_UVLC( symbol, "hrd_cpb_cnt_minus1" );                               hrd->setHrdCpbCntMinus1( symbol );
    CHECK_RECOVERABLE( symbol > 31, "The value of hrd_cpb_cnt_minus1 shall be in the range of 0 to 31, inclusive" );
  }
}

void HLSyntaxReader::parseOlsHrdParameters( GeneralHrdParams * generalHrd, OlsHrdParams *olsHrd, uint32_t firstSubLayer, uint32_t maxNumSubLayersMinus1 )
{
  uint32_t  symbol;

  for( int i = firstSubLayer; i <= maxNumSubLayersMinus1; i ++ )
  {
    OlsHrdParams *hrd = &(olsHrd[i]);
    READ_FLAG( symbol, "fixed_pic_rate_general_flag" );                 hrd->setFixedPicRateGeneralFlag( symbol == 1 ? true : false );
    if( !hrd->getFixedPicRateGeneralFlag() )
    {
      READ_FLAG( symbol, "fixed_pic_rate_within_cvs_flag" );            hrd->setFixedPicRateWithinCvsFlag( symbol == 1 ? true : false );
    }
    else
    {
      hrd->setFixedPicRateWithinCvsFlag( true );
    }

    hrd->setLowDelayHrdFlag( false ); // Inferred to be 0 when not present

    if( hrd->getFixedPicRateWithinCvsFlag() )
    {
      READ_UVLC( symbol, "elemental_duration_in_tc_minus1" );           hrd->setElementDurationInTcMinus1( symbol );
    }
    else if( generalHrd->getHrdCpbCntMinus1() == 0 )
    {
      READ_FLAG(symbol, "low_delay_hrd_flag");                          hrd->setLowDelayHrdFlag( symbol == 1 ? true : false );
    }

    for( int nalOrVcl = 0; nalOrVcl < 2; nalOrVcl ++ )
    {
      if( ( nalOrVcl == 0 && generalHrd->getGeneralNalHrdParametersPresentFlag() ) || ( nalOrVcl == 1 && generalHrd->getGeneralVclHrdParametersPresentFlag() ) )
      {
        for( int j = 0; j <= generalHrd->getHrdCpbCntMinus1(); j++ )
        {
          READ_UVLC( symbol, "bit_rate_value_minus1");             hrd->setBitRateValueMinus1( j, nalOrVcl, symbol );
          READ_UVLC( symbol, "cpb_size_value_minus1");             hrd->setCpbSizeValueMinus1( j, nalOrVcl, symbol );
          if( generalHrd->getGeneralDecodingUnitHrdParamsPresentFlag() )
          {
            READ_UVLC( symbol, "cpb_size_du_value_minus1" );       hrd->setDuCpbSizeValueMinus1( j, nalOrVcl, symbol );
            READ_UVLC( symbol, "bit_rate_du_value_minus1" );       hrd->setDuBitRateValueMinus1( j, nalOrVcl, symbol );
          }
          READ_FLAG( symbol, "cbr_flag" );                         hrd->setCbrFlag( j, nalOrVcl, symbol == 1 ? true : false );
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
      if( ( nalOrVcl == 0 && generalHrd->getGeneralNalHrdParametersPresentFlag() ) || ( nalOrVcl == 1 && generalHrd->getGeneralVclHrdParametersPresentFlag() ) )
      {
        for( int j = 0; j <= (generalHrd->getHrdCpbCntMinus1()); j++ )
        {
          uint32_t bitRate = hrdHighestTLayer->getBitRateValueMinus1( j, nalOrVcl );
          hrdTemp->setBitRateValueMinus1( j, nalOrVcl, bitRate );
          uint32_t cpbSize = hrdHighestTLayer->getCpbSizeValueMinus1( j, nalOrVcl );
          hrdTemp->setCpbSizeValueMinus1( j, nalOrVcl, cpbSize );
          if( generalHrd->getGeneralDecodingUnitHrdParamsPresentFlag() )
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
  uint32_t code;
  for( int i = ( subLayerInfoFlag ? 0 : maxSubLayersMinus1 ); i <= maxSubLayersMinus1; i++ )
  {
    READ_UVLC( code, "dpb_max_dec_pic_buffering_minus1[i]" );
    pcSPS->setMaxDecPicBuffering( code + 1, i );
    READ_UVLC( code, "dpb_max_num_reorder_pics[i]" );
    pcSPS->setNumReorderPics( code, i );
    CHECK_RECOVERABLE( pcSPS->getNumReorderPics( i ) >= pcSPS->getMaxDecPicBuffering( i ),
           "The value of dpb_max_num_reorder_pics[ i ] shall be in the range of 0 to dpb_max_dec_pic_buffering_minus1[ i ], inclusive" );
    READ_UVLC( code, "dpb_max_latency_increase_plus1[i]" );
    pcSPS->setMaxLatencyIncreasePlus1( code, i );
  }
}

void HLSyntaxReader::parseExtraPHBitsStruct( SPS *sps, int numBytes )
{
  uint32_t symbol;
  std::vector<bool> presentFlags;
  presentFlags.resize ( 8 * numBytes );

  for( int i = 0; i < 8 * numBytes; i++ )
  {
    READ_FLAG( symbol, "sps_extra_ph_bit_present_flag[i]" );
    presentFlags[i] = symbol;
  }

  sps->setExtraPHBitPresentFlags( presentFlags );
}

void HLSyntaxReader::parseExtraSHBitsStruct( SPS *sps, int numBytes )
{
  uint32_t symbol;
  std::vector<bool> presentFlags;
  presentFlags.resize ( 8 * numBytes );

  for( int i = 0; i < 8 * numBytes; i++ )
  {
    READ_FLAG( symbol, "sps_extra_sh_bit_present_flag[i]" );
    presentFlags[i] = symbol;
  }

  sps->setExtraSHBitPresentFlags( presentFlags );
}

void HLSyntaxReader::parseSPS( SPS* pcSPS, const ParameterSetManager *parameterSetManager )
{
#if ENABLE_TRACING
  xTraceSPSHeader ();
#endif

  uint32_t uiCode = 0;

  READ_CODE( 4, uiCode, "sps_seq_parameter_set_id" );                        pcSPS->setSPSId( uiCode );
  READ_CODE( 4, uiCode, "sps_video_parameter_set_id" );                      int vpsId = uiCode; //pcSPS->setVPSId( vpsId ); // TODO: change to support VPS
  READ_CODE( 3, uiCode, "sps_max_sub_layers_minus1" );                       pcSPS->setMaxTLayers( uiCode + 1 );
  CHECK_RECOVERABLE( uiCode > 6, "Invalid maximum number of T-layer signalled" );
  READ_CODE( 2, uiCode, "sps_chroma_format_idc" );                           pcSPS->setChromaFormatIdc( ChromaFormat( uiCode ) );

  READ_CODE( 2, uiCode, "sps_log2_ctu_size_minus5"  );                       pcSPS->setCTUSize( 1 << ( uiCode + 5 ) );
  CHECK_RECOVERABLE( uiCode > 2, "sps_log2_ctu_size_minus5 must be less than or equal to 2" );
  unsigned ctbLog2SizeY = uiCode + 5;
  pcSPS->setMaxCUWidth( pcSPS->getCTUSize() );
  pcSPS->setMaxCUHeight( pcSPS->getCTUSize() );
  READ_FLAG( uiCode, "sps_ptl_dpb_hrd_params_present_flag" );                pcSPS->setPtlDpbHrdParamsPresentFlag( uiCode );

  if( !vpsId )
  {
    CHECK_RECOVERABLE( !pcSPS->getPtlDpbHrdParamsPresentFlag(), "When sps_video_parameter_set_id is equal to 0, the value of sps_ptl_dpb_hrd_params_present_flag shall be equal to 1" );
  }

  if( pcSPS->getPtlDpbHrdParamsPresentFlag() )
  {
    parseProfileTierLevel( pcSPS->getProfileTierLevel(), true, pcSPS->getMaxTLayers() - 1 );
  }

  READ_FLAG( uiCode, "sps_gdr_enabled_flag" );                               pcSPS->setGDREnabledFlag( uiCode );
  if( pcSPS->getProfileTierLevel()->getConstraintInfo()->getNoGdrConstraintFlag() )
  {
    CHECK_RECOVERABLE( uiCode != 0, "When gci_no_gdr_constraint_flag equal to 1 , the value of sps_gdr_enabled_flag shall be equal to 0" );
  }

  READ_FLAG( uiCode, "sps_ref_pic_resampling_enabled_flag" );                pcSPS->setRprEnabledFlag( uiCode );
  if( pcSPS->getProfileTierLevel()->getConstraintInfo()->getNoRprConstraintFlag() )
  {
    CHECK_RECOVERABLE( uiCode != 0, "When gci_no_ref_pic_resampling_constraint_flag is equal to 1, sps_ref_pic_resampling_enabled_flag shall be equal to 0" );
  }
  if( uiCode )
  {
    READ_FLAG( uiCode, "sps_res_change_in_clvs_allowed_flag" );              pcSPS->setResChangeInClvsEnabledFlag( uiCode );
  }
  else
  {
    pcSPS->setResChangeInClvsEnabledFlag( 0 );
  }

  if( pcSPS->getProfileTierLevel()->getConstraintInfo()->getNoResChangeInClvsConstraintFlag() )
  {
    CHECK_RECOVERABLE( uiCode != 0, "When no_res_change_in_clvs_constraint_flag is equal to 1, res_change_in_clvs_allowed_flag shall be equal to 0" );
  }

  READ_UVLC( uiCode, "sps_pic_width_max_in_luma_samples" );                  pcSPS->setMaxPicWidthInLumaSamples( uiCode );
  READ_UVLC( uiCode, "sps_pic_height_max_in_luma_samples" );                 pcSPS->setMaxPicHeightInLumaSamples( uiCode );

  READ_FLAG( uiCode, "sps_conformance_window_flag" );                        pcSPS->setConformanceWindowPresentFlag( uiCode );
  if( uiCode != 0 )
  {
    Window& conf = pcSPS->getConformanceWindow();
    READ_UVLC( uiCode, "sps_conf_win_left_offset" );                         conf.setWindowLeftOffset( uiCode );
    READ_UVLC( uiCode, "sps_conf_win_right_offset" );                        conf.setWindowRightOffset( uiCode );
    READ_UVLC( uiCode, "sps_conf_win_top_offset" );                          conf.setWindowTopOffset( uiCode );
    READ_UVLC( uiCode, "sps_conf_win_bottom_offset" );                       conf.setWindowBottomOffset( uiCode );
  }

  READ_FLAG( uiCode, "sps_subpic_info_present_flag" );                       pcSPS->setSubPicInfoPresentFlag( uiCode );

  if( pcSPS->getProfileTierLevel()->getConstraintInfo()->getNoSubpicInfoConstraintFlag() )
  {
    CHECK_RECOVERABLE( uiCode != 0, "When gci_no_subpic_info_constraint_flag is equal to 1, the value of subpic_info_present_flag shall be equal to 0" );
  }

  if( pcSPS->getSubPicInfoPresentFlag() )
  {
    READ_UVLC( uiCode, "sps_num_subpics_minus1" );                           pcSPS->setNumSubPics( uiCode + 1 );
    CHECK_RECOVERABLE( uiCode > ( ( pcSPS->getMaxPicWidthInLumaSamples()  + pcSPS->getCTUSize() - 1 ) / ( pcSPS->getCTUSize() ) ) *
                    ( ( pcSPS->getMaxPicHeightInLumaSamples() + pcSPS->getCTUSize() - 1 ) / ( pcSPS->getCTUSize() ) ) - 1,
           "Invalid sps_num_subpics_minus1 value" );
    if( pcSPS->getNumSubPics() == 1 )
    {
      pcSPS->setSubPicCtuTopLeftX( 0, 0 );
      pcSPS->setSubPicCtuTopLeftY( 0, 0 );
      pcSPS->setSubPicWidth( 0, ( pcSPS->getMaxPicWidthInLumaSamples() + pcSPS->getCTUSize() - 1 ) >> getLog2( pcSPS->getCTUSize() ) );
      pcSPS->setSubPicHeight( 0, ( pcSPS->getMaxPicHeightInLumaSamples() + pcSPS->getCTUSize() - 1 ) >> getLog2( pcSPS->getCTUSize() ) );
      pcSPS->setIndependentSubPicsFlag( 1 );
      pcSPS->setSubPicSameSizeFlag( 0 );
      pcSPS->setSubPicTreatedAsPicFlag( 0, 1 );
      pcSPS->setLoopFilterAcrossSubpicEnabledFlag( 0, 0 );
    }
    else
    {
      READ_FLAG( uiCode, "sps_independent_subpics_flag" );                   pcSPS->setIndependentSubPicsFlag( uiCode != 0 );
      READ_FLAG( uiCode, "sps_subpic_same_size_flag" );                      pcSPS->setSubPicSameSizeFlag( uiCode );
      uint32_t tmpWidthVal = ( pcSPS->getMaxPicWidthInLumaSamples() + pcSPS->getCTUSize() - 1 ) / pcSPS->getCTUSize();
      uint32_t tmpHeightVal = ( pcSPS->getMaxPicHeightInLumaSamples() + pcSPS->getCTUSize() - 1 ) / pcSPS->getCTUSize();
      uint32_t numSubpicCols = 1;
      for( int picIdx = 0; picIdx < pcSPS->getNumSubPics(); picIdx++ )
      {
        if( !pcSPS->getSubPicSameSizeFlag() || picIdx == 0 )
        {
          if( ( picIdx > 0 ) && ( pcSPS->getMaxPicWidthInLumaSamples() > pcSPS->getCTUSize() ) )
          {
            READ_CODE( (int)ceil( log2( tmpWidthVal ) ), uiCode, "sps_subpic_ctu_top_left_x[i]" );
            pcSPS->setSubPicCtuTopLeftX( picIdx, uiCode );
          }
          else
          {
            pcSPS->setSubPicCtuTopLeftX( picIdx, 0 );
          }
          if( ( picIdx > 0 ) && ( pcSPS->getMaxPicHeightInLumaSamples() > pcSPS->getCTUSize() ) )
          {
            READ_CODE( (int)ceil( log2( tmpHeightVal ) ), uiCode, "sps_subpic_ctu_top_left_y[i]" );
            pcSPS->setSubPicCtuTopLeftY( picIdx, uiCode );
          }
          else
          {
            pcSPS->setSubPicCtuTopLeftY( picIdx, 0 );
          }
          if( picIdx <pcSPS->getNumSubPics() - 1 && pcSPS->getMaxPicWidthInLumaSamples() > pcSPS->getCTUSize() )
          {
            READ_CODE( (int)ceil( log2( tmpWidthVal ) ), uiCode, "sps_subpic_width_minus1[i]" );
            pcSPS->setSubPicWidth( picIdx, uiCode + 1 );
          }
          else
          {
            pcSPS->setSubPicWidth( picIdx, tmpWidthVal - pcSPS->getSubPicCtuTopLeftX( picIdx ) );
          }
          if( picIdx <pcSPS->getNumSubPics() - 1 && pcSPS->getMaxPicHeightInLumaSamples() > pcSPS->getCTUSize() )
          {
            READ_CODE( (int)ceil( log2( tmpHeightVal ) ), uiCode, "sps_subpic_height_minus1[i]" );
            pcSPS->setSubPicHeight(picIdx, uiCode + 1);
          }
          else
          {
            pcSPS->setSubPicHeight( picIdx, tmpHeightVal - pcSPS->getSubPicCtuTopLeftY(picIdx) );
          }
          if( pcSPS->getSubPicSameSizeFlag() )
          {
            numSubpicCols = tmpWidthVal / pcSPS->getSubPicWidth( 0 );
            CHECK_RECOVERABLE( !( tmpWidthVal % pcSPS->getSubPicWidth(0) == 0 ), "subpic_width_minus1[0] is invalid." );
            CHECK_RECOVERABLE( !( tmpHeightVal % pcSPS->getSubPicHeight(0) == 0 ), "subpic_height_minus1[0] is invalid." );
            CHECK_RECOVERABLE( !( numSubpicCols * ( tmpHeightVal / pcSPS->getSubPicHeight(0) ) == pcSPS->getNumSubPics() ), "when sps_subpic_same_size_flag is equal to, sps_num_subpics_minus1 is invalid" );
          }
        }
        else
        {
          pcSPS->setSubPicCtuTopLeftX( picIdx, ( picIdx % numSubpicCols ) * pcSPS->getSubPicWidth( 0 ) );
          pcSPS->setSubPicCtuTopLeftY( picIdx, ( picIdx / numSubpicCols ) * pcSPS->getSubPicHeight( 0 ) );
          pcSPS->setSubPicWidth( picIdx, pcSPS->getSubPicWidth( 0 ) );
          pcSPS->setSubPicHeight( picIdx, pcSPS->getSubPicHeight( 0 ) );
        }
        if( !pcSPS->getIndependentSubPicsFlag() )
        {
          READ_FLAG( uiCode, "sps_subpic_treated_as_pic_flag[i]" );
          pcSPS->setSubPicTreatedAsPicFlag( picIdx, uiCode );
          READ_FLAG( uiCode, "sps_loop_filter_across_subpic_enabled_flag[i]" );
          pcSPS->setLoopFilterAcrossSubpicEnabledFlag( picIdx, uiCode );
        }
      }
    }

    READ_UVLC( uiCode, "sps_subpic_id_len_minus1" );                         pcSPS->setSubPicIdLen( uiCode + 1 );
    CHECK_RECOVERABLE( uiCode > 15, "Invalid sps_subpic_id_len_minus1 value" );
    CHECK_RECOVERABLE( ( 1 << ( uiCode + 1 ) ) < pcSPS->getNumSubPics(), "Invalid sps_subpic_id_len_minus1 value" );
    READ_FLAG( uiCode, "sps_subpic_id_mapping_explicitly_signalled_flag" );  pcSPS->setSubPicIdMappingExplicitlySignalledFlag( uiCode != 0 );
    if( pcSPS->getSubPicIdMappingExplicitlySignalledFlag() )
    {
      READ_FLAG( uiCode, "sps_subpic_id_mapping_present_flag" );             pcSPS->setSubPicIdMappingInSpsFlag( uiCode != 0 );
      if( pcSPS->getSubPicIdMappingInSpsFlag() )
      {
        for( int picIdx = 0; picIdx < pcSPS->getNumSubPics(); picIdx++ )
        {
          READ_CODE( pcSPS->getSubPicIdLen(), uiCode, "sps_subpic_id[ i ]" );
          pcSPS->setSubPicId( picIdx, uiCode );
        }
      }
    }
  }
  else
  {
    pcSPS->setSubPicIdMappingExplicitlySignalledFlag( 0 );
    pcSPS->setNumSubPics( 1 );
    pcSPS->setSubPicCtuTopLeftX( 0, 0 );
    pcSPS->setSubPicCtuTopLeftY( 0, 0 );
    pcSPS->setSubPicWidth( 0, ( pcSPS->getMaxPicWidthInLumaSamples() + pcSPS->getCTUSize() - 1 ) >> getLog2( pcSPS->getCTUSize() ) );
    pcSPS->setSubPicHeight( 0, ( pcSPS->getMaxPicHeightInLumaSamples() + pcSPS->getCTUSize() - 1 ) >> getLog2( pcSPS->getCTUSize() ) );
    pcSPS->setSubPicTreatedAsPicFlag( 0, false );
    pcSPS->setLoopFilterAcrossSubpicEnabledFlag( 0, true );
  }

  if( !pcSPS->getSubPicIdMappingExplicitlySignalledFlag() || !pcSPS->getSubPicIdMappingInSpsFlag() )
  {
    for( int picIdx = 0; picIdx < pcSPS->getNumSubPics(); picIdx++ )
    {
      pcSPS->setSubPicId( picIdx, picIdx );
    }
  }


  READ_UVLC( uiCode, "sps_bitdepth_minus8" );
  CHECK_RECOVERABLE( uiCode > 8, "Invalid bit depth signalled" );
  const Profile::Name profile = pcSPS->getProfileTierLevel()->getProfileIdc();
  if( profile != Profile::NONE )
  {
    CHECK_RECOVERABLE( uiCode + 8 > ProfileFeatures::getProfileFeatures( profile )->maxBitDepth, "sps_bitdepth_minus8 exceeds range supported by signalled profile" );
  }
  pcSPS->setBitDepth( CHANNEL_TYPE_LUMA, 8 + uiCode );
  pcSPS->setBitDepth( CHANNEL_TYPE_CHROMA, 8 + uiCode);
  pcSPS->setQpBDOffset( CHANNEL_TYPE_LUMA, (int) (6*uiCode) );
  pcSPS->setQpBDOffset( CHANNEL_TYPE_CHROMA, (int) (6*uiCode) );

  READ_FLAG( uiCode, "sps_entropy_coding_sync_enabled_flag" );               pcSPS->setEntropyCodingSyncEnabledFlag( uiCode == 1 );
  READ_FLAG( uiCode, "sps_entry_point_offsets_present_flag" );               pcSPS->setEntryPointsPresentFlag( uiCode == 1 );
  READ_CODE( 4, uiCode, "sps_log2_max_pic_order_cnt_lsb_minus4" );           pcSPS->setBitsForPOC( 4 + uiCode );
  CHECK_RECOVERABLE( uiCode > 12, "log2_max_pic_order_cnt_lsb_minus4 shall be in the range of 0 to 12" );

  READ_FLAG( uiCode, "sps_poc_msb_cycle_flag" );                             pcSPS->setPocMsbFlag( uiCode ? true : false );
  if( pcSPS->getPocMsbFlag() )
  {
    READ_UVLC( uiCode, "sps_poc_msb_cycle_len_minus1" );                     pcSPS->setPocMsbLen( 1 + uiCode );
    CHECK_RECOVERABLE( uiCode > ( 32 - ( pcSPS->getBitsForPOC() - 4 ) - 5 ), "The value of poc_msb_len_minus1 shall be in the range of 0 to 32 - log2_max_pic_order_cnt_lsb_minus4 - 5, inclusive" );
  }

  // extra bits are for future extensions, we will read, but ignore them,
  // unless a meaning is specified in the spec
  READ_CODE( 2, uiCode, "sps_num_extra_ph_bytes" );                          pcSPS->setNumExtraPHBitsBytes( uiCode );
  parseExtraPHBitsStruct( pcSPS, uiCode );
  READ_CODE( 2, uiCode, "sps_num_extra_sh_bytes");                           pcSPS->setNumExtraSHBitsBytes( uiCode );
  parseExtraSHBitsStruct( pcSPS, uiCode );

  if( pcSPS->getPtlDpbHrdParamsPresentFlag() )
  {
    if( pcSPS->getMaxTLayers() - 1 > 0 )
    {
      READ_FLAG( uiCode, "sps_sublayer_dpb_params_flag" );                   pcSPS->setSubLayerDpbParamsFlag( uiCode ? true : false );
    }
    dpb_parameters( pcSPS->getMaxTLayers() - 1, pcSPS->getSubLayerDpbParamsFlag(), pcSPS );
  }
  unsigned  minQT[3]  = { 0, 0, 0 };
  unsigned  maxBTD[3] = { 0, 0, 0 };

  unsigned  maxBTSize[3] = { 0, 0, 0 };
  unsigned  maxTTSize[3] = { 0, 0, 0 };
  READ_UVLC( uiCode, "sps_log2_min_luma_coding_block_size_minus2" );
  int log2MinCUSize = uiCode + 2;
  pcSPS->setLog2MinCodingBlockSize( log2MinCUSize );
  CHECK_RECOVERABLE( uiCode > ctbLog2SizeY - 2, "Invalid log2_min_luma_coding_block_size_minus2 signalled" );
  CHECK_RECOVERABLE( log2MinCUSize > std::min( 6, (int)(ctbLog2SizeY) ), "log2_min_luma_coding_block_size_minus2 shall be in the range of 0 to min (4, log2_ctu_size - 2)" );

  const int minCuSize = 1 << pcSPS->getLog2MinCodingBlockSize();
  CHECK_RECOVERABLE( ( pcSPS->getMaxPicWidthInLumaSamples() % ( std::max( 8, minCuSize ) ) ) != 0, "Coded frame width must be a multiple of Max(8, the minimum unit size)" );
  CHECK_RECOVERABLE( ( pcSPS->getMaxPicHeightInLumaSamples() % ( std::max( 8, minCuSize ) ) ) != 0, "Coded frame height must be a multiple of Max(8, the minimum unit size)" );

  READ_FLAG( uiCode, "sps_partition_constraints_override_enabled_flag");     pcSPS->setSplitConsOverrideEnabledFlag( uiCode );
  READ_UVLC( uiCode, "sps_log2_diff_min_qt_min_cb_intra_slice_luma");        minQT[0] = 1 << ( uiCode + pcSPS->getLog2MinCodingBlockSize() );
  CHECK_RECOVERABLE( minQT[0] > 64, "The value of sps_log2_diff_min_qt_min_cb_intra_slice_luma shall be in the range of 0 to min(6,CtbLog2SizeY) - MinCbLog2Size" );
  CHECK_RECOVERABLE( minQT[0] > ( 1<<ctbLog2SizeY ), "The value of sps_log2_diff_min_qt_min_cb_intra_slice_luma shall be in the range of 0 to min(6,CtbLog2SizeY) - MinCbLog2Size" );
  READ_UVLC( uiCode, "sps_max_mtt_hierarchy_depth_intra_slice_luma");        maxBTD[0] = uiCode;
  CHECK_RECOVERABLE( uiCode > 2 * ( ctbLog2SizeY - log2MinCUSize ), "sps_max_mtt_hierarchy_depth_intra_slice_luma shall be in the range 0 to 2*(ctbLog2SizeY - log2MinCUSize)" );

  maxTTSize[0] = maxBTSize[0] = minQT[0];
  if( maxBTD[0] != 0 )
  {
    READ_UVLC( uiCode, "sps_log2_diff_max_bt_min_qt_intra_slice_luma" );     maxBTSize[0] <<= uiCode;
    READ_UVLC( uiCode, "sps_log2_diff_max_tt_min_qt_intra_slice_luma" );     maxTTSize[0] <<= uiCode;
  }

  if( pcSPS->getChromaFormatIdc() != CHROMA_400 )
  {
    READ_FLAG( uiCode, "sps_qtbtt_dual_tree_intra_flag" );                   pcSPS->setUseDualITree( uiCode );
  }
  else
  {
    pcSPS->setUseDualITree( 0 );
  }
  if( pcSPS->getUseDualITree() )
  {
    READ_UVLC( uiCode, "sps_log2_diff_min_qt_min_cb_intra_slice_chroma" );   minQT[2] = 1 << ( uiCode + pcSPS->getLog2MinCodingBlockSize() );
    READ_UVLC( uiCode, "sps_max_mtt_hierarchy_depth_intra_slice_chroma" );   maxBTD[2] = uiCode;
    CHECK_RECOVERABLE( uiCode > 2 * ( ctbLog2SizeY - log2MinCUSize ), "sps_max_mtt_hierarchy_depth_intra_slice_chroma shall be in the range 0 to 2*(ctbLog2SizeY - log2MinCUSize)" );
    maxTTSize[2] = maxBTSize[2] = minQT[2];
    if( maxBTD[2] != 0 )
    {
      READ_UVLC( uiCode, "sps_log2_diff_max_bt_min_qt_intra_slice_chroma" ); maxBTSize[2] <<= uiCode;
      READ_UVLC( uiCode, "sps_log2_diff_max_tt_min_qt_intra_slice_chroma" ); maxTTSize[2] <<= uiCode;
      CHECK_RECOVERABLE( maxTTSize[2] > 64, "The value of sps_log2_diff_max_tt_min_qt_intra_slice_chroma shall be in the range of 0 to min(6,CtbLog2SizeY) - MinQtLog2SizeIntraChroma" );
      CHECK_RECOVERABLE( maxBTSize[2] > 64, "The value of sps_log2_diff_max_bt_min_qt_intra_slice_chroma shall be in the range of 0 to min(6,CtbLog2SizeY) - MinQtLog2SizeIntraChroma" );
    }
  }

  READ_UVLC( uiCode, "sps_log2_diff_min_qt_min_cb_inter_slice" );            minQT[1] = 1 << ( uiCode + pcSPS->getLog2MinCodingBlockSize() );
  CHECK_RECOVERABLE( maxTTSize[0] > 64, "The value of sps_log2_diff_max_tt_min_qt_intra_slice_luma shall be in the range of 0 to min(6,CtbLog2SizeY) - MinQtLog2SizeIntraY" );
  READ_UVLC( uiCode, "sps_max_mtt_hierarchy_depth_inter_slice");             maxBTD[1] = uiCode;
  CHECK_RECOVERABLE( uiCode > 2 * ( ctbLog2SizeY - log2MinCUSize ), "sps_max_mtt_hierarchy_depth_inter_slice shall be in the range 0 to 2*(ctbLog2SizeY - log2MinCUSize)" );
  maxTTSize[1] = maxBTSize[1] = minQT[1];
  if( maxBTD[1] != 0 )
  {
    READ_UVLC( uiCode, "sps_log2_diff_max_bt_min_qt_inter_slice" );          maxBTSize[1] <<= uiCode;
    READ_UVLC( uiCode, "sps_log2_diff_max_tt_min_qt_inter_slice" );          maxTTSize[1] <<= uiCode;
    CHECK_RECOVERABLE( maxTTSize[1] > 64, "The value of sps_log2_diff_max_tt_min_qt_inter_slice shall be in the range of 0 to min(6,CtbLog2SizeY) - MinQtLog2SizeInterY" );
  }

  pcSPS->setMinQTSizes( minQT );
  pcSPS->setMaxBTDepth( maxBTD[1], maxBTD[0], maxBTD[2] );
  pcSPS->setMaxBTSize( maxBTSize[1], maxBTSize[0], maxBTSize[2] );
  pcSPS->setMaxTTSize( maxTTSize[1], maxTTSize[0], maxTTSize[2] );

  if( pcSPS->getCTUSize() > 32 )
  {
    READ_FLAG( uiCode, "sps_max_luma_transform_size_64_flag" );              pcSPS->setLog2MaxTbSize( ( uiCode ? 1 : 0 ) + 5 );
  }
  else
  {
    pcSPS->setLog2MaxTbSize( 5 );
  }

  READ_FLAG( uiCode, "sps_transform_skip_enabled_flag" );                    pcSPS->setTransformSkipEnabledFlag( uiCode ? true : false );
  if( pcSPS->getTransformSkipEnabledFlag() )
  {
    READ_UVLC( uiCode, "sps_log2_transform_skip_max_size_minus2" );          pcSPS->setLog2MaxTransformSkipBlockSize( uiCode + 2 );
    READ_FLAG( uiCode, "sps_bdpcm_enabled_flag" );                           pcSPS->setBDPCMEnabledFlag( uiCode ? true : false );
  }
  READ_FLAG( uiCode, "sps_mts_enabled_flag" );                               pcSPS->setUseMTS( uiCode != 0 );
  if( pcSPS->getUseMTS() )
  {
    READ_FLAG( uiCode, "sps_explicit_mts_intra_enabled_flag" );              pcSPS->setUseIntraMTS( uiCode != 0 );
    READ_FLAG( uiCode, "sps_explicit_mts_inter_enabled_flag" );              pcSPS->setUseInterMTS( uiCode != 0 );
  }
  READ_FLAG( uiCode, "sps_lfnst_enabled_flag" );                             pcSPS->setUseLFNST( uiCode != 0 );

  if( pcSPS->getChromaFormatIdc() != CHROMA_400 )
  {
    READ_FLAG( uiCode, "sps_joint_cbcr_enabled_flag" );                      pcSPS->setJointCbCrEnabledFlag( uiCode ? true : false );
    ChromaQpMappingTableParams chromaQpMappingTableParams;
    READ_FLAG( uiCode, "sps_same_qp_table_for_chroma_flag" );                chromaQpMappingTableParams.setSameCQPTableForAllChromaFlag( uiCode );
    int numQpTables = chromaQpMappingTableParams.getSameCQPTableForAllChromaFlag() ? 1 : ( pcSPS->getJointCbCrEnabledFlag() ? 3 : 2 );
    chromaQpMappingTableParams.setNumQpTables( numQpTables );
    for( int i = 0; i < numQpTables; i++ )
    {
      int32_t qpTableStart = 0;
      READ_SVLC( qpTableStart, "sps_qp_table_starts_minus26" );              chromaQpMappingTableParams.setQpTableStartMinus26( i, qpTableStart );
      CHECK_RECOVERABLE( qpTableStart < -26 - pcSPS->getQpBDOffset( CHANNEL_TYPE_LUMA ) || qpTableStart > 36, "The value of sps_qp_table_start_minus26[ i ] shall be in the range of -26 - QpBdOffset to 36 inclusive" );
      READ_UVLC( uiCode, "sps_num_points_in_qp_table_minus1" );              chromaQpMappingTableParams.setNumPtsInCQPTableMinus1( i, uiCode );
      CHECK_RECOVERABLE( uiCode > 36 - qpTableStart, "The value of sps_num_points_in_qp_table_minus1[ i ] shall be in the range of 0 to 36 - sps_qp_table_start_minus26[ i ], inclusive" );
      std::vector<int> deltaQpInValMinus1( chromaQpMappingTableParams.getNumPtsInCQPTableMinus1( i ) + 1 );
      std::vector<int> deltaQpOutVal( chromaQpMappingTableParams.getNumPtsInCQPTableMinus1( i ) + 1 );
      for( int j = 0; j <= chromaQpMappingTableParams.getNumPtsInCQPTableMinus1( i ); j++ )
      {
        READ_UVLC( uiCode, "sps_delta_qp_in_val_minus1" );                   deltaQpInValMinus1[j] = uiCode;
        READ_UVLC( uiCode, "sps_delta_qp_diff_val" );                        deltaQpOutVal[j] = uiCode ^ deltaQpInValMinus1[j];
      }
      chromaQpMappingTableParams.setDeltaQpInValMinus1( i, deltaQpInValMinus1 );
      chromaQpMappingTableParams.setDeltaQpOutVal( i, deltaQpOutVal );
    }
    pcSPS->setChromaQpMappingTableFromParams( chromaQpMappingTableParams, pcSPS->getQpBDOffset( CHANNEL_TYPE_CHROMA ) );
    pcSPS->derivedChromaQPMappingTables();
  }

  READ_FLAG( uiCode, "sps_sao_enabled_flag" );                               pcSPS->setUseSAO ( uiCode );
  READ_FLAG( uiCode, "sps_alf_enabled_flag" );                               pcSPS->setUseALF( uiCode );
  if( pcSPS->getUseALF() && pcSPS->getChromaFormatIdc() != CHROMA_400 )
  {
    READ_FLAG( uiCode, "sps_ccalf_enabled_flag" );                           pcSPS->setUseCCALF( uiCode );
  }
  else
  {
    pcSPS->setUseCCALF( false );
  }

  READ_FLAG( uiCode, "sps_lmcs_enable_flag" );                               pcSPS->setUseReshaper( uiCode == 1 );

  READ_FLAG( uiCode, "sps_weighted_pred_flag" );                             pcSPS->setUseWP( uiCode ? true : false );
  READ_FLAG( uiCode, "sps_weighted_bipred_flag" );                           pcSPS->setUseWPBiPred( uiCode ? true : false );

  READ_FLAG( uiCode, "sps_long_term_ref_pics_flag" );                        pcSPS->setLongTermRefsPresent( uiCode );
  if( vpsId > 0 )
  {
    READ_FLAG( uiCode, "sps_inter_layer_ref_pics_present_flag" );            pcSPS->setInterLayerPresentFlag( uiCode );
  }
  else
  {
    pcSPS->setInterLayerPresentFlag( 0 );
  }
  READ_FLAG( uiCode, "sps_idr_rpl_present_flag" );                           pcSPS->setIDRRefParamListPresent( (bool) uiCode );
  if( pcSPS->getProfileTierLevel()->getConstraintInfo()->getNoIdrRplConstraintFlag() )
  {
    CHECK_RECOVERABLE( uiCode != 0, "When gci_no_idr_rpl_constraint_flag equal to 1 , the value of sps_idr_rpl_present_flag shall be equal to 0" );
  }

  READ_FLAG( uiCode, "sps_rpl1_same_as_rpl0_flag" );                         pcSPS->setRPL1CopyFromRPL0Flag( uiCode );

  //Read candidate for List0
  READ_UVLC( uiCode, "sps_num_ref_pic_lists[0]" );
  uint32_t numberOfRPL = uiCode;
  RPLList& rplList = pcSPS->createRPLList( 0, numberOfRPL );
  for( uint32_t ii = 0; ii < numberOfRPL; ii++ )
  {
    parseRefPicList( &rplList[ii], ii, pcSPS );
  }

  //Read candidate for List1
  if( !pcSPS->getRPL1CopyFromRPL0Flag() )
  {
    READ_UVLC( uiCode, "sps_num_ref_pic_lists[1]" );
    numberOfRPL = uiCode;
    RPLList& rplList = pcSPS->createRPLList( 1, numberOfRPL );
    for( uint32_t ii = 0; ii < numberOfRPL; ii++ )
    {
      parseRefPicList( &rplList[ii], ii, pcSPS );
    }
  }
  else
  {
    numberOfRPL = ( uint32_t ) pcSPS->getNumRPL( 0 );
    const RPLList& rplListSource = pcSPS->getRPLList( 0 );
    RPLList&       rplListDest   = pcSPS->createRPLList( 1, numberOfRPL );
    for( uint32_t ii = 0; ii < numberOfRPL; ii++ )
    {
      copyRefPicList( pcSPS, &rplListSource[ii], &rplListDest[ii] );
    }
  }

  {
    READ_FLAG( uiCode, "sps_ref_wraparound_enabled_flag" );                  pcSPS->setUseWrapAround( uiCode ? true : false );
  }

  READ_FLAG( uiCode, "sps_temporal_mvp_enabled_flag" );                      pcSPS->setSPSTemporalMVPEnabledFlag( uiCode );

  if( pcSPS->getSPSTemporalMVPEnabledFlag() )
  {
    READ_FLAG( uiCode, "sps_sbtmvp_enabled_flag" );                          pcSPS->setSBTMVPEnabledFlag( uiCode != 0 );
  }
  else
  {
    pcSPS->setSBTMVPEnabledFlag( false );
  }

  READ_FLAG( uiCode, "sps_amvr_enabled_flag" );                              pcSPS->setAMVREnabledFlag ( uiCode != 0 );

  READ_FLAG( uiCode, "sps_bdof_enabled_flag" );                              pcSPS->setUseBIO ( uiCode != 0 );
  if( pcSPS->getUseBIO() )
  {
    READ_FLAG( uiCode, "sps_bdof_control_present_in_ph_flag" );              pcSPS->setBdofControlPresentFlag( uiCode != 0 );
  }
  READ_FLAG( uiCode, "sps_smvd_enabled_flag" );                              pcSPS->setUseSMVD( uiCode != 0 );
  READ_FLAG( uiCode, "sps_dmvr_enabled_flag" );                              pcSPS->setUseDMVR( uiCode != 0 );
  if( pcSPS->getUseDMVR() )
  {
    READ_FLAG( uiCode, "sps_dmvr_control_present_in_ph_flag" );              pcSPS->setDmvrControlPresentFlag( uiCode != 0 );
  }
  READ_FLAG( uiCode, "sps_mmvd_enabled_flag" );                              pcSPS->setUseMMVD( uiCode != 0 );
  if( pcSPS->getUseMMVD() )
  {
    READ_FLAG( uiCode, "sps_mmvd_fullpel_only_flag" );                       pcSPS->setFpelMmvdEnabledFlag( uiCode != 0 );
  }
  else
  {
    pcSPS->setFpelMmvdEnabledFlag( false );
  }

  READ_UVLC( uiCode, "sps_six_minus_max_num_merge_cand" );                   pcSPS->setMaxNumMergeCand( MRG_MAX_NUM_CANDS - uiCode );
  CHECK_RECOVERABLE( MRG_MAX_NUM_CANDS <= uiCode, "Incorrrect max number of merge candidates!" );
  READ_FLAG( uiCode, "sps_sbt_enabled_flag" );                               pcSPS->setUseSBT( uiCode != 0 );
  READ_FLAG( uiCode, "sps_affine_enabled_flag" );                            pcSPS->setUseAffine( uiCode != 0 );
  if( pcSPS->getUseAffine() )
  {
    READ_UVLC( uiCode, "sps_five_minus_max_num_subblock_merge_cand" );       pcSPS->setMaxNumAffineMergeCand( AFFINE_MRG_MAX_NUM_CANDS - uiCode );
    CHECK_RECOVERABLE( AFFINE_MRG_MAX_NUM_CANDS < uiCode,
           "The value of sps_five_minus_max_num_subblock_merge_cand shall be in the "
           "range of 0 to 5 - sps_sbtmvp_enabled_flag" );
    READ_FLAG( uiCode, "sps_affine_type_flag" );                             pcSPS->setUseAffineType( uiCode != 0 );
    if( pcSPS->getAMVREnabledFlag() )
    {
      READ_FLAG( uiCode, "sps_affine_amvr_enabled_flag" );                   pcSPS->setAffineAmvrEnabledFlag( uiCode != 0 );
    }
    READ_FLAG( uiCode, "sps_affine_prof_enabled_flag" );                     pcSPS->setUsePROF( uiCode != 0 );
    if( pcSPS->getUsePROF() )
    {
      READ_FLAG( uiCode, "sps_prof_control_present_in_ph_flag" );            pcSPS->setProfControlPresentFlag( uiCode != 0 );
    }
    else
    {
      pcSPS->setProfControlPresentFlag( false );
    }
  }

  READ_FLAG( uiCode, "sps_bcw_enabled_flag" );                               pcSPS->setUseBcw( uiCode != 0 );

  READ_FLAG( uiCode, "sps_ciip_enabled_flag" );                              pcSPS->setUseCiip( uiCode != 0 );
  if( pcSPS->getMaxNumMergeCand() >= 2 )
  {
    READ_FLAG( uiCode, "sps_gpm_enabled_flag" );                             pcSPS->setUseGeo( uiCode != 0 );
    if( pcSPS->getUseGeo() && pcSPS->getMaxNumMergeCand() >= 3 )
    {
      READ_UVLC( uiCode, "sps_max_num_merge_cand_minus_max_num_gpm_cand" );  pcSPS->setMaxNumGeoCand( (uint32_t)( pcSPS->getMaxNumMergeCand() - uiCode ) );
      CHECK_RECOVERABLE( pcSPS->getMaxNumMergeCand() - 2 < uiCode,
             "sps_max_num_merge_cand_minus_max_num_gpm_cand must not be "
             "greater than the number of merge candidates minus 2" );
    }
    else if( pcSPS->getUseGeo() )
    {
      pcSPS->setMaxNumGeoCand( 2 );
    }
  }
  else
  {
    pcSPS->setUseGeo( 0 );
    pcSPS->setMaxNumGeoCand( 0 );
  }

  READ_UVLC( uiCode, "sps_log2_parallel_merge_level_minus2" );               pcSPS->setLog2ParallelMergeLevelMinus2( uiCode );
  CHECK_RECOVERABLE( uiCode + 2 > ctbLog2SizeY, "The value of sps_log2_parallel_merge_level_minus2 shall be in the range of 0 to ctbLog2SizeY - 2" );

  READ_FLAG( uiCode, "sps_isp_enabled_flag" );                               pcSPS->setUseISP( uiCode != 0 );
  READ_FLAG( uiCode, "sps_mrl_enabled_flag" );                               pcSPS->setUseMRL( uiCode != 0 );
  READ_FLAG( uiCode, "sps_mip_enabled_flag" );                               pcSPS->setUseMIP( uiCode != 0 );

  if( pcSPS->getChromaFormatIdc() != CHROMA_400 )
  {
    READ_FLAG( uiCode, "sps_cclm_enabled_flag" );                            pcSPS->setUseLMChroma( uiCode != 0 );
  }
  else
  {
    pcSPS->setUseLMChroma( 0 );
  }
  if( pcSPS->getChromaFormatIdc() == CHROMA_420 )
  {
    READ_FLAG( uiCode, "sps_chroma_horizontal_collocated_flag" );            pcSPS->setHorCollocatedChromaFlag( uiCode != 0 );
    READ_FLAG( uiCode, "sps_chroma_vertical_collocated_flag" );              pcSPS->setVerCollocatedChromaFlag( uiCode != 0 );
  }

  READ_FLAG( uiCode, "sps_palette_enabled_flag" );
  CHECK_RECOVERABLE( uiCode != 0, "palette mode is not yet supported" );

  if( pcSPS->getChromaFormatIdc() == CHROMA_444 && pcSPS->getLog2MaxTbSize() != 6 )
  {
    READ_FLAG( uiCode, "sps_act_enabled_flag" );                             pcSPS->setUseColorTrans( uiCode != 0 );
  }
  else
  {
    pcSPS->setUseColorTrans( false );
  }

//  if( pcSPS->getTransformSkipEnabledFlag() || pcSPS->getPLTMode() ) //palette mode not in here
  if( pcSPS->getTransformSkipEnabledFlag() )
  {
    READ_UVLC( uiCode, "sps_internal_bit_depth_minus_input_bit_depth" );
    pcSPS->setInternalMinusInputBitDepth( CHANNEL_TYPE_LUMA, uiCode );
    CHECK_RECOVERABLE( uiCode > 8, "Invalid sps_internal_bit_depth_minus_input_bit_depth signalled" );
    pcSPS->setInternalMinusInputBitDepth( CHANNEL_TYPE_CHROMA, uiCode );
  }
  READ_FLAG( uiCode, "sps_ibc_enabled_flag" );                               pcSPS->setIBCFlag( uiCode );
  if( pcSPS->getIBCFlag() )
  {
    READ_UVLC( uiCode, "sps_six_minus_max_num_ibc_merge_cand" );             pcSPS->setMaxNumIBCMergeCand( IBC_MRG_MAX_NUM_CANDS - uiCode );
    CHECK_RECOVERABLE( IBC_MRG_MAX_NUM_CANDS <= uiCode, "Incorrect max number of IBC merge candidates!" );
  }
  else
  {
    pcSPS->setMaxNumIBCMergeCand( 0 );
  }

#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
  READ_FLAG( uiCode, "sps_ladf_enabled_flag" );                              pcSPS->setLadfEnabled( uiCode != 0 );
  if( pcSPS->getLadfEnabled() )
  {
    int signedSymbol = 0;
    READ_CODE( 2, uiCode, "sps_num_ladf_intervals_minus2" );                 pcSPS->setLadfNumIntervals( uiCode + 2 );
    READ_SVLC( signedSymbol, "sps_ladf_lowest_interval_qp_offset" );         pcSPS->setLadfQpOffset( signedSymbol, 0 );
    for( int k = 1; k < pcSPS->getLadfNumIntervals(); k++ )
    {
      READ_SVLC( signedSymbol, "sps_ladf_qp_offset[i]" );                  pcSPS->setLadfQpOffset( signedSymbol, k );
      READ_UVLC( uiCode, "sps_ladf_delta_threshold_minus1[i]" );           pcSPS->setLadfIntervalLowerBound( uiCode + pcSPS->getLadfIntervalLowerBound( k - 1 ) + 1, k );
    }
  }
#endif

  READ_FLAG( uiCode, "sps_explicit_scaling_list_enabled_flag" );             pcSPS->setScalingListFlag ( uiCode );

  if( pcSPS->getProfileTierLevel()->getConstraintInfo()->getNoExplicitScaleListConstraintFlag() )
  {
    CHECK_RECOVERABLE( uiCode != 0, "When gci_no_explicit_scaling_list_constraint_flag is equal to 1, sps_explicit_scaling_list_enabled_flag shall be equal to 0" );
  }

  if( pcSPS->getUseLFNST() && pcSPS->getScalingListFlag() )
  {
    READ_FLAG( uiCode, "sps_scaling_matrix_for_lfnst_disabled_flag" );       pcSPS->setDisableScalingMatrixForLfnstBlks( uiCode ? true : false );
  }

  if( pcSPS->getUseColorTrans() && pcSPS->getScalingListFlag() )
  {
    READ_FLAG( uiCode, "sps_scaling_matrix_for_alternative_colour_space_disabled_flag");
    pcSPS->setScalingMatrixForAlternativeColourSpaceDisabledFlag( uiCode );
  }
  if( pcSPS->getScalingMatrixForAlternativeColourSpaceDisabledFlag() )
  {
    READ_FLAG(uiCode, "sps_scaling_matrix_designated_colour_space_flag");    pcSPS->setScalingMatrixDesignatedColourSpaceFlag( uiCode );
  }
  READ_FLAG( uiCode, "sps_dep_quant_enabled_flag" );                         pcSPS->setDepQuantEnabledFlag( uiCode );

  READ_FLAG( uiCode, "sps_sign_data_hiding_enabled_flag" );                pcSPS->setSignDataHidingEnabledFlag( uiCode );

  READ_FLAG( uiCode, "sps_virtual_boundaries_enabled_flag" );                pcSPS->setVirtualBoundariesEnabledFlag( uiCode != 0 );
  if( pcSPS->getProfileTierLevel()->getConstraintInfo()->getNoVirtualBoundaryConstraintFlag() )
  {
    CHECK_RECOVERABLE( uiCode != 0, "When gci_no_virtual_boundaries_constraint_flag is equal to 1, sps_virtual_boundaries_enabled_flag shall be equal to 0" );
  }

  if( pcSPS->getVirtualBoundariesEnabledFlag() )
  {
    READ_FLAG( uiCode, "sps_loop_filter_across_virtual_boundaries_present_flag" );              pcSPS->setVirtualBoundariesPresentFlag( uiCode != 0 );
    if( pcSPS->getVirtualBoundariesPresentFlag() )
    {
      READ_UVLC( uiCode, "sps_num_ver_virtual_boundaries" );                pcSPS->setNumVerVirtualBoundaries( uiCode );
      if( pcSPS->getMaxPicWidthInLumaSamples() <= 8 )
      {
        CHECK_RECOVERABLE( pcSPS->getNumVerVirtualBoundaries() != 0,
               "SPS: When picture width is less than or equal to 8, the number of vertical virtual boundaries shall be equal to 0" );
      }
      else
      {
        CHECK_RECOVERABLE( pcSPS->getNumVerVirtualBoundaries() > 3,
               "SPS: The number of vertical virtual boundaries shall be in the range of 0 to 3" );
      }
      for( unsigned i = 0; i < pcSPS->getNumVerVirtualBoundaries(); i++ )
      {
        READ_UVLC( uiCode, "sps_virtual_boundary_pos_x_minus1[i]" );        pcSPS->setVirtualBoundariesPosX( (uiCode + 1) << 3, i );
        CHECK_RECOVERABLE( uiCode > ( ( ( pcSPS->getMaxPicWidthInLumaSamples() + 7 ) >> 3 ) - 2 ),
               "The value of sps_virtual_boundary_pos_x_minus1[ i ] shall be in the range of 0 to Ceil( sps_pic_width_max_in_luma_samples / 8 ) - 2, inclusive." );
      }
      READ_UVLC( uiCode, "sps_num_hor_virtual_boundaries" );                pcSPS->setNumHorVirtualBoundaries( uiCode );
      if( pcSPS->getMaxPicHeightInLumaSamples() <= 8 )
      {
        CHECK_RECOVERABLE( pcSPS->getNumHorVirtualBoundaries() != 0,
               "SPS: When picture height is less than or equal to 8, the number of horizontal virtual boundaries shall be equal to 0" );
      }
      else
      {
        CHECK_RECOVERABLE( pcSPS->getNumHorVirtualBoundaries() > 3,
               "SPS: The number of horizontal virtual boundaries shall be in the range of 0 to 3" );
      }
      for( unsigned i = 0; i < pcSPS->getNumHorVirtualBoundaries(); i++ )
      {
        READ_UVLC( uiCode, "sps_virtual_boundary_pos_y_minus1[i]" );        pcSPS->setVirtualBoundariesPosY( (uiCode + 1) << 3, i );
        CHECK_RECOVERABLE( uiCode > ( ( ( pcSPS->getMaxPicHeightInLumaSamples() + 7 ) >> 3 ) - 2 ),
               "The value of sps_virtual_boundary_pos_y_minus1[ i ] shall be in the range of 0 to Ceil( sps_pic_height_max_in_luma_samples / 8 ) - 2, inclusive." );
      }
    }
    else
    {
      pcSPS->setNumVerVirtualBoundaries( 0 );
      pcSPS->setNumHorVirtualBoundaries( 0 );
    }
  }
  else
  {
    pcSPS->setVirtualBoundariesPresentFlag( false );
  }

  if( pcSPS->getPtlDpbHrdParamsPresentFlag() )
  {
    READ_FLAG( uiCode, "sps_timing_hrd_params_present_flag" );               pcSPS->setGeneralHrdParametersPresentFlag( uiCode );
    if( pcSPS->getGeneralHrdParametersPresentFlag() )
    {
      parseGeneralHrdParameters( pcSPS->getGeneralHrdParameters() );
      if( ( pcSPS->getMaxTLayers() - 1 ) > 0 )
      {
        READ_FLAG( uiCode, "sps_sublayer_cpb_params_present_flag" );         pcSPS->setSubLayerParametersPresentFlag( uiCode );
      }
      else if( ( pcSPS->getMaxTLayers() - 1 ) == 0 )
      {
        pcSPS->setSubLayerParametersPresentFlag( 0 );
      }

      uint32_t firstSubLayer = pcSPS->getSubLayerParametersPresentFlag() ? 0 : ( pcSPS->getMaxTLayers() - 1 );
      parseOlsHrdParameters( pcSPS->getGeneralHrdParameters(), pcSPS->getOlsHrdParameters(), firstSubLayer, pcSPS->getMaxTLayers() - 1 );
    }
  }

  READ_FLAG( uiCode, "sps_field_seq_flag");                                  pcSPS->setFieldSeqFlag( uiCode );
  CHECK_RECOVERABLE( pcSPS->getProfileTierLevel()->getFrameOnlyConstraintFlag() && uiCode, "When ptl_frame_only_constraint_flag equal to 1 , the value of sps_field_seq_flag shall be equal to 0" );

  READ_FLAG( uiCode, "sps_vui_parameters_present_flag" );                    pcSPS->setVuiParametersPresentFlag( uiCode );

  if( pcSPS->getVuiParametersPresentFlag() )
  {
    READ_UVLC( uiCode, "sps_vui_payload_size_minus1" );                      pcSPS->setVuiPayloadSize( uiCode+1 );
    while( !isByteAligned() )
    {
      READ_FLAG( uiCode, "sps_vui_alignment_zero_bit" );
      CHECK_RECOVERABLE( uiCode != 0, "sps_vui_alignment_zero_bit not equal to 0" );
    }
    parseVUI( pcSPS->getVuiParameters(), pcSPS );
  }

  READ_FLAG( uiCode, "sps_extension_present_flag" );
  if( uiCode )
  {
    while( xMoreRbspData() )
    {
      READ_FLAG( uiCode, "sps_extension_data_flag" );
    }
  }

  xReadRbspTrailingBits();
}

void HLSyntaxReader::parseDCI( DCI* dci )
{
#if ENABLE_TRACING
  xTraceDCIHeader();
#endif
  uint32_t  symbol;

  READ_CODE( 4, symbol, "dci_reserved_zero_4bits" );

  uint32_t numPTLs;
  READ_CODE( 4, numPTLs, "dci_num_ptls_minus1" );
  numPTLs += 1;

  std::vector<ProfileTierLevel> ptls;
  ptls.resize(numPTLs);
  for( int i = 0; i < numPTLs; i++ )
  {
    parseProfileTierLevel( &ptls[i], true, 0 );
  }
  dci->setProfileTierLevel( ptls );

  READ_FLAG( symbol, "dci_extension_flag" );
  if( symbol )
  {
    while( xMoreRbspData() )
    {
      READ_FLAG( symbol, "dci_extension_data_flag" );
    }
  }
  xReadRbspTrailingBits();
}
void HLSyntaxReader::parseVPS( VPS* pcVPS )
{
#if ENABLE_TRACING
  xTraceVPSHeader();
#endif
  uint32_t  uiCode;

  //CHECK_RECOVERABLE( true, "needs to be adjusted, e.g. sublayer and independent layer stuff -> see VTM-9.0" );

  READ_CODE( 4, uiCode, "vps_video_parameter_set_id" );                      pcVPS->setVPSId( uiCode );
  CHECK_RECOVERABLE( uiCode == 0, "vps_video_parameter_set_id equal to zero is reserved and shall not be used in a bitstream" );

  READ_CODE( 6, uiCode, "vps_max_layers_minus1" );                           pcVPS->setMaxLayers( uiCode + 1 );
  CHECK_RECOVERABLE( uiCode + 1 > MAX_VPS_LAYERS, "Signalled number of layers larger than MAX_VPS_LAYERS." );

  if( pcVPS->getMaxLayers() - 1 == 0 )
  {
    pcVPS->setEachLayerIsAnOlsFlag( 1 );
  }
  READ_CODE( 3, uiCode, "vps_max_sublayers_minus1" );                        pcVPS->setMaxSubLayers( uiCode + 1 );
  CHECK_RECOVERABLE( uiCode + 1 > MAX_VPS_SUBLAYERS, "Signalled number of sublayers larger than MAX_VPS_SUBLAYERS." );

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
        CHECK_RECOVERABLE(sumUiCode == 0, "There has to be at least one value of j such that the value of vps_direct_dependency_flag[i][ j ] is equal to 1,when vps_independent_layer_flag[i] is equal to 0 " );
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
        CHECK_RECOVERABLE( uiCode > MAX_VPS_OLS_MODE_IDC, "ols_mode_idc shall be in the rage of 0 to 2" );
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
  CHECK_RECOVERABLE( uiCode >= pcVPS->getTotalNumOLSs(), "The value of vps_num_ptls_minus1 shall be less than TotalNumOlss" );

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
    CHECK_RECOVERABLE( uiCode!=0, "Alignment bit is not '0'" );
    cnt++;
  }
  CHECK_RECOVERABLE( cnt >= 8, "Read more than '8' alignment bits" );
  std::vector<ProfileTierLevel> ptls;
  ptls.resize( pcVPS->getNumPtls() );
  for( int i = 0; i < pcVPS->getNumPtls(); i++ )
  {
    parseProfileTierLevel( &ptls[i], pcVPS->getPtPresentFlag( i ), pcVPS->getPtlMaxTemporalId( i ) );
  }
  pcVPS->setProfileTierLevel( ptls );
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
    CHECK_RECOVERABLE( !isPTLReferred[i],"Each profile_tier_level( ) syntax structure in the VPS shall be referred to by at least one value of vps_ols_ptl_idx[i] for i in the range of 0 to TotalNumOlss ? 1, inclusive" );
  }

  if( !pcVPS->getEachLayerIsAnOlsFlag() )
  {
    READ_UVLC( uiCode, "vps_num_dpb_params_minus1" );                        pcVPS->m_numDpbParams = uiCode + 1;

    CHECK_RECOVERABLE( pcVPS->m_numDpbParams > pcVPS->getNumMultiLayeredOlss(), "The value of vps_num_dpb_params_minus1 shall be in the range of 0 to NumMultiLayerOlss - 1, inclusive" );
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
        CHECK_RECOVERABLE( uiCode > ( pcVPS->getMaxSubLayers() - 1 ), "The value of vps_dpb_max_tid[i] shall be in the range of 0 to vps_max_sublayers_minus1, inclusive." )
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
          CHECK_RECOVERABLE( uiCode + 8 > ProfileFeatures::getProfileFeatures( profile )->maxBitDepth,
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
      CHECK_RECOVERABLE( !isDPBParamReferred[i], "Each dpb_parameters() syntax structure in the VPS shall be referred to by at least one value of vps_ols_dpb_params_idx[i] for i in the range of 0 to NumMultiLayerOlss - 1, inclusive" );
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
    CHECK_RECOVERABLE( uiCode >= pcVPS->getNumMultiLayeredOlss(), "The value of vps_num_ols_hrd_params_minus1 shall be in the range of 0 to NumMultiLayerOlss - 1, inclusive" );
    std::vector<bool> isHRDParamReferred( uiCode + 1, false );
    pcVPS->m_olsHrdParams.clear();
    pcVPS->m_olsHrdParams.resize( pcVPS->getNumOlsHrdParamsMinus1(), std::vector<OlsHrdParams>( pcVPS->getMaxSubLayers() ) );
    for( int i = 0; i <= pcVPS->getNumOlsHrdParamsMinus1(); i++ )
    {
      if( !pcVPS->getAllLayersSameNumSublayersFlag() )
      {
        READ_CODE( 3, uiCode, "vps_hrd_max_tid[i]" );                      pcVPS->setHrdMaxTid(i, uiCode );
        CHECK_RECOVERABLE( uiCode > ( pcVPS->getMaxSubLayers() - 1 ), "The value of vps_hrd_max_tid[i] shall be in the range of 0 to vps_max_sublayers_minus1, inclusive." );
      }
      else
      {
        pcVPS->setHrdMaxTid( i, pcVPS->getMaxSubLayers() - 1 );
      }
      uint32_t firstSublayer = pcVPS->getVPSSublayerCpbParamsPresentFlag() ? 0 : pcVPS->getHrdMaxTid( i );
      parseOlsHrdParameters( pcVPS->getGeneralHrdParameters(),pcVPS->getOlsHrdParameters( i ), firstSublayer, pcVPS->getHrdMaxTid( i ) );
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
        CHECK_RECOVERABLE( uiCode > pcVPS->getNumOlsHrdParamsMinus1(), "The value of ols_hrd_idx[[i] shall be in the range of 0 to num_ols_hrd_params_minus1, inclusive." );
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
      CHECK_RECOVERABLE( !isHRDParamReferred[i], "Each ols_hrd_parameters( ) syntax structure in the VPS shall be referred to by at least one value of vps_ols_hrd_idx[i] for i in the range of 1 to NumMultiLayerOlss - 1, inclusive");
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

void HLSyntaxReader::parsePictureHeader( PicHeader* picHeader, const ParameterSetManager *parameterSetManager, bool readRbspTrailingBits )
{
  uint32_t uiCode = 0;
  int      iCode  = 0;

#if ENABLE_TRACING
  xTracePictureHeader();
#endif

  READ_FLAG( uiCode, "ph_gdr_or_irap_pic_flag" );                            picHeader->setGdrOrIrapPicFlag( uiCode != 0 );
  READ_FLAG( uiCode, "ph_non_ref_pic_flag" );                                picHeader->setNonReferencePictureFlag( uiCode != 0 );
  if( picHeader->getGdrOrIrapPicFlag() )
  {
    READ_FLAG( uiCode, "ph_gdr_pic_flag" );                                  picHeader->setGdrPicFlag( uiCode != 0 );
  }
  else
  {
    picHeader->setGdrPicFlag( false );
  }
  READ_FLAG( uiCode, "ph_inter_slice_allowed_flag" );                        picHeader->setPicInterSliceAllowedFlag( uiCode != 0 );
  if( picHeader->getPicInterSliceAllowedFlag() )
  {
    READ_FLAG( uiCode, "ph_intra_slice_allowed_flag" );                      picHeader->setPicIntraSliceAllowedFlag( uiCode != 0 );
  }
  else
  {
    picHeader->setPicIntraSliceAllowedFlag( true );
  }
  CHECK_RECOVERABLE( picHeader->getPicInterSliceAllowedFlag() == 0 && picHeader->getPicIntraSliceAllowedFlag() == 0, "Invalid picture without intra or inter slice" );
  // parameter sets
  READ_UVLC( uiCode, "ph_pic_parameter_set_id" );                            picHeader->setPPSId( uiCode );
  const PPS* pps = parameterSetManager->getPPS( picHeader->getPPSId() );
  CHECK_RECOVERABLE( pps == 0, "Invalid PPS" );
  picHeader->setSPSId( pps->getSPSId() );
  const SPS* sps = parameterSetManager->getSPS( picHeader->getSPSId() );
  CHECK_RECOVERABLE( sps == 0, "Invalid SPS" );
  READ_CODE( sps->getBitsForPOC(), uiCode, "ph_pic_order_cnt_lsb" );         picHeader->setPocLsb( uiCode );
  if( picHeader->getGdrPicFlag() )
  {
    READ_UVLC( uiCode, "ph_recovery_poc_cnt" );                              picHeader->setRecoveryPocCnt( uiCode );
  }
  else
  {
    picHeader->setRecoveryPocCnt( -1 );
  }

  std::vector<bool> phExtraBitsPresent = sps->getExtraPHBitPresentFlags();
  for( int i=0; i< sps->getNumExtraPHBitsBytes() * 8; i++ )
  {
    // extra bits are ignored (when present)
    if( phExtraBitsPresent[i] )
    {
      READ_FLAG( uiCode, "ph_extra_bit[i]" );
    }
  }

  if( sps->getPocMsbFlag() )
  {
    READ_FLAG( uiCode, "ph_poc_msb_cycle_present_flag" );                    picHeader->setPocMsbPresentFlag( uiCode != 0 );
    if( picHeader->getPocMsbPresentFlag() )
    {
      READ_CODE( sps->getPocMsbLen(), uiCode, "ph_poc_msb_cycle_val" );      picHeader->setPocMsbVal( uiCode );
    }
  }

  // alf enable flags and aps IDs
  picHeader->setCcAlfEnabledFlag( COMPONENT_Cb, false );
  picHeader->setCcAlfEnabledFlag( COMPONENT_Cr, false );
  if( sps->getUseALF() )
  {
    if( pps->getAlfInfoInPhFlag() )
    {
      READ_FLAG( uiCode, "ph_alf_enabled_flag" );                            picHeader->setAlfEnabledFlag( COMPONENT_Y, uiCode );

      int alfCbEnabledFlag = 0;
      int alfCrEnabledFlag = 0;
      if( uiCode )
      {
        READ_CODE( 3, uiCode, "ph_num_alf_aps_ids_luma" );
        int numAps = uiCode;
        picHeader->setNumAlfAps( numAps );

        std::vector<int> apsId( numAps, -1 );
        for( int i = 0; i < numAps; i++ )
        {
          READ_CODE( 3, uiCode, "ph_alf_aps_id_luma[i]" );
          apsId[i] = uiCode;
        }
        picHeader->setAlfAPSIds( std::move( apsId ) );

        if( sps->getChromaFormatIdc() != CHROMA_400 )
        {
          READ_CODE( 1, uiCode, "ph_alf_cb_enabled_flag" );   alfCbEnabledFlag = uiCode;
          READ_CODE( 1, uiCode, "ph_alf_cr_enabled_flag" );   alfCrEnabledFlag = uiCode;
        }
        else
        {
          alfCbEnabledFlag = 0;
          alfCrEnabledFlag = 0;
        }
        if( alfCbEnabledFlag || alfCrEnabledFlag )
        {
          READ_CODE( 3, uiCode, "ph_alf_aps_id_chroma" );                    picHeader->setAlfApsIdChroma( uiCode );
        }
        if( sps->getUseCCALF() )
        {
          READ_FLAG( uiCode, "ph_cc_alf_cb_enabled_flag" );                  picHeader->setCcAlfEnabledFlag( COMPONENT_Cb, uiCode != 0 );
          picHeader->setCcAlfCbApsId( -1 );
          if( picHeader->getCcAlfEnabledFlag( COMPONENT_Cb ) )
          {
            // parse APS ID
            READ_CODE( 3, uiCode, "ph_cc_alf_cb_aps_id" );                   picHeader->setCcAlfCbApsId( uiCode );
           }
          // Cr
          READ_FLAG( uiCode, "ph_cc_alf_cr_enabled_flag" );                  picHeader->setCcAlfEnabledFlag( COMPONENT_Cr, uiCode != 0 );
          picHeader->setCcAlfCrApsId( -1 );
          if( picHeader->getCcAlfEnabledFlag( COMPONENT_Cr ) )
          {
            // parse APS ID
            READ_CODE( 3, uiCode, "ph_cc_alf_cr_aps_id" );                   picHeader->setCcAlfCrApsId( uiCode );
          }
        }
      }
      else
      {
        picHeader->setNumAlfAps( 0 );
      }
      picHeader->setAlfEnabledFlag( COMPONENT_Cb, alfCbEnabledFlag );
      picHeader->setAlfEnabledFlag( COMPONENT_Cr, alfCrEnabledFlag );
    }
    else
    {
      picHeader->setAlfEnabledFlag( COMPONENT_Y, true );
      picHeader->setAlfEnabledFlag( COMPONENT_Cb, true );
      picHeader->setAlfEnabledFlag( COMPONENT_Cr, true );
    }
  }
  else
  {
    picHeader->setAlfEnabledFlag( COMPONENT_Y, false );
    picHeader->setAlfEnabledFlag( COMPONENT_Cb, false );
    picHeader->setAlfEnabledFlag( COMPONENT_Cr, false );
  }

  // luma mapping / chroma scaling controls
  if( sps->getUseReshaper() )
  {
    READ_FLAG( uiCode, "ph_lmcs_enabled_flag" );                             picHeader->setLmcsEnabledFlag( uiCode != 0 );

    if( picHeader->getLmcsEnabledFlag() )
    {
      READ_CODE( 2, uiCode, "ph_lmcs_aps_id" );                              picHeader->setLmcsAPSId( uiCode );

      if( sps->getChromaFormatIdc() != CHROMA_400 )
      {
        READ_FLAG( uiCode, "ph_chroma_residual_scale_flag" );                picHeader->setLmcsChromaResidualScaleFlag( uiCode != 0 );
      }
      else
      {
        picHeader->setLmcsChromaResidualScaleFlag( false );
      }
    }
  }
  else
  {
    picHeader->setLmcsEnabledFlag( false );
    picHeader->setLmcsChromaResidualScaleFlag( false );
  }
  // quantization scaling lists
  if( sps->getScalingListFlag() )
  {
    READ_FLAG( uiCode, "ph_explicit_scaling_list_enabled_flag" );            picHeader->setExplicitScalingListEnabledFlag( uiCode );
    if( picHeader->getExplicitScalingListEnabledFlag() )
    {
      READ_CODE( 3, uiCode, "ph_scaling_list_aps_id" );                      picHeader->setScalingListAPSId( uiCode );
    }
  }
  else
  {
    picHeader->setExplicitScalingListEnabledFlag( false );
  }

  // virtual boundaries
  if( sps->getVirtualBoundariesEnabledFlag() && !sps->getVirtualBoundariesPresentFlag() )
  {
    READ_FLAG( uiCode, "ph_virtual_boundaries_present_flag" );               picHeader->setVirtualBoundariesPresentFlag( uiCode != 0 );
    if( picHeader->getVirtualBoundariesPresentFlag() )
    {
      READ_UVLC( uiCode, "ph_num_ver_virtual_boundaries" );               picHeader->setNumVerVirtualBoundaries( uiCode );
      if( pps->getPicWidthInLumaSamples() <= 8 )
      {
        CHECK_RECOVERABLE( picHeader->getNumVerVirtualBoundaries() != 0, "PH: When picture width is less than or equal to 8, the number of vertical virtual boundaries shall be equal to 0" );
      }
      else
      {
        CHECK_RECOVERABLE( picHeader->getNumVerVirtualBoundaries() > 3, "PH: The number of vertical virtual boundaries shall be in the range of 0 to 3" );
      }
      for( unsigned i = 0; i < picHeader->getNumVerVirtualBoundaries(); i++ )
      {
        READ_UVLC( uiCode, "ph_virtual_boundary_pos_x_minus1[ i ]" );        picHeader->setVirtualBoundariesPosX( (uiCode + 1) << 3, i );
        CHECK_RECOVERABLE( uiCode > ( ( ( pps->getPicWidthInLumaSamples() + 7 ) >> 3 ) - 2 ),
               "The value of ph_virtual_boundary_pos_x_minus1[ i ] shall be in the range of 0 to Ceil( pps_pic_width_in_luma_samples / 8 ) - 2, inclusive." );
      }
      READ_UVLC( uiCode, "ph_num_hor_virtual_boundaries" );               picHeader->setNumHorVirtualBoundaries( uiCode );
      if( pps->getPicHeightInLumaSamples() <= 8 )
      {
        CHECK_RECOVERABLE( picHeader->getNumHorVirtualBoundaries() != 0, "PH: When picture width is less than or equal to 8, the number of horizontal virtual boundaries shall be equal to 0" );
      }
      else
      {
        CHECK_RECOVERABLE( picHeader->getNumHorVirtualBoundaries() > 3, "PH: The number of horizontal virtual boundaries shall be in the range of 0 to 3" );
      }
      for( unsigned i = 0; i < picHeader->getNumHorVirtualBoundaries(); i++ )
      {
        READ_UVLC( uiCode, "ph_virtual_boundary_pos_y_minus1[ i ]" );        picHeader->setVirtualBoundariesPosY( (uiCode + 1) << 3, i );
        CHECK_RECOVERABLE( uiCode > ( ( ( pps->getPicHeightInLumaSamples() + 7 ) >> 3 ) - 2 ),
               "The value of ph_virtual_boundary_pos_y_minus1[ i ] shall be in the range of 0 to Ceil( pps_pic_height_in_luma_samples / 8 ) - 2, inclusive." );
      }
    }
    else
    {
      picHeader->setNumVerVirtualBoundaries( 0 );
      picHeader->setNumHorVirtualBoundaries( 0 );
    }
  }
  else
  {
    picHeader->setVirtualBoundariesPresentFlag( sps->getVirtualBoundariesPresentFlag() );
    if( picHeader->getVirtualBoundariesPresentFlag() )
    {
      picHeader->setNumVerVirtualBoundaries( sps->getNumVerVirtualBoundaries() );
      picHeader->setNumHorVirtualBoundaries( sps->getNumHorVirtualBoundaries() );
      for( unsigned i = 0; i < 3; i++ )
      {
        picHeader->setVirtualBoundariesPosX( sps->getVirtualBoundariesPosX( i ), i );
        picHeader->setVirtualBoundariesPosY( sps->getVirtualBoundariesPosY( i ), i );
      }
    }
  }

  // picture output flag
  if( pps->getOutputFlagPresentFlag() && !picHeader->getNonReferencePictureFlag() )
  {
    READ_FLAG( uiCode, "ph_pic_output_flag" );                               picHeader->setPicOutputFlag( uiCode != 0 );
  }
  else
  {
    picHeader->setPicOutputFlag( true );
  }

  // reference picture lists
  if( pps->getRplInfoInPhFlag() )
  {
    parsePicOrSliceHeaderRPL( picHeader, sps, pps );
  }

  // partitioning constraint overrides
  if( sps->getSplitConsOverrideEnabledFlag() )
  {
    READ_FLAG( uiCode, "ph_partition_constraints_override_flag" );           picHeader->setSplitConsOverrideFlag( uiCode != 0 );
  }
  else
  {
    picHeader->setSplitConsOverrideFlag( 0 );
  }
  // Q0781, two-flags
  unsigned minQT[3]     = { 0, 0, 0 };
  unsigned maxBTD[3]    = { 0, 0, 0 };
  unsigned maxBTSize[3] = { 0, 0, 0 };
  unsigned maxTTSize[3] = { 0, 0, 0 };

  if( picHeader->getPicIntraSliceAllowedFlag() )
  {
    if( picHeader->getSplitConsOverrideFlag() )
    {

      READ_UVLC( uiCode, "ph_log2_diff_min_qt_min_cb_intra_slice_luma" );
      unsigned minQtLog2SizeIntraY = uiCode + sps->getLog2MinCodingBlockSize();
      minQT[0] = 1 << minQtLog2SizeIntraY;
      CHECK_RECOVERABLE( minQT[0] > 64, "The value of ph_log2_diff_min_qt_min_cb_intra_slice_luma shall be in the range of 0 to min(6,CtbLog2SizeY) - MinCbLog2Size" );
      READ_UVLC( uiCode, "ph_max_mtt_hierarchy_depth_intra_slice_luma" );      maxBTD[0] = uiCode;

      maxTTSize[0] = maxBTSize[0] = minQT[0];
      if( maxBTD[0] != 0 )
      {
        READ_UVLC( uiCode, "ph_log2_diff_max_bt_min_qt_intra_slice_luma" );    maxBTSize[0] <<= uiCode;
        READ_UVLC( uiCode, "ph_log2_diff_max_tt_min_qt_intra_slice_luma" );    maxTTSize[0] <<= uiCode;
      }
      if( sps->getUseDualITree() )
      {
        READ_UVLC( uiCode, "ph_log2_diff_min_qt_min_cb_intra_slice_chroma" );  minQT[2] = 1 << (uiCode + sps->getLog2MinCodingBlockSize());
        CHECK_RECOVERABLE( minQT[2] > 64, "The value of ph_log2_diff_min_qt_min_cb_intra_slice_chroma shall be in the range of 0 to min(6,CtbLog2SizeY) - MinCbLog2Size" );
        READ_UVLC( uiCode, "ph_max_mtt_hierarchy_depth_intra_slice_chroma" );  maxBTD[2] = uiCode;
        maxTTSize[2] = maxBTSize[2] = minQT[2];
        if( maxBTD[2] != 0 )
        {
          READ_UVLC( uiCode, "ph_log2_diff_max_bt_min_qt_intra_slice_chroma" ); maxBTSize[2] <<= uiCode;
          READ_UVLC( uiCode, "ph_log2_diff_max_tt_min_qt_intra_slice_chroma" ); maxTTSize[2] <<= uiCode;
          CHECK_RECOVERABLE( maxBTSize[2] > 64, "The value of ph_log2_diff_max_bt_min_qt_intra_slice_chroma shall be in the range of 0 to min(6,CtbLog2SizeY) - MinQtLog2SizeIntraChroma" );
          CHECK_RECOVERABLE( maxTTSize[2] > 64, "The value of ph_log2_diff_max_tt_min_qt_intra_slice_chroma shall be in the range of 0 to min(6,CtbLog2SizeY) - MinQtLog2SizeIntraChroma" );
        }
      }
    }
  }


  if( picHeader->getPicIntraSliceAllowedFlag() )
  {
    // delta quantization and chrom and chroma offset
    if( pps->getUseDQP() )
    {
      READ_UVLC( uiCode, "ph_cu_qp_delta_subdiv_intra_slice" );              picHeader->setCuQpDeltaSubdivIntra( uiCode );
    }
    else
    {
      picHeader->setCuQpDeltaSubdivIntra( 0 );
    }
    if( pps->getCuChromaQpOffsetEnabledFlag() )
    {
      READ_UVLC( uiCode, "ph_cu_chroma_qp_offset_subdiv_intra_slice" );      picHeader->setCuChromaQpOffsetSubdivIntra( uiCode );
    }
    else
    {
      picHeader->setCuChromaQpOffsetSubdivIntra( 0 );
    }
  }

  if( picHeader->getPicInterSliceAllowedFlag() )
  {
    if( picHeader->getSplitConsOverrideFlag() )
    {
      READ_UVLC( uiCode, "ph_log2_diff_min_qt_min_cb_inter_slice" );
      unsigned minQtLog2SizeInterY = uiCode + sps->getLog2MinCodingBlockSize();
      minQT[1] = 1 << minQtLog2SizeInterY;
      READ_UVLC( uiCode, "ph_max_mtt_hierarchy_depth_inter_slice" );         maxBTD[1] = uiCode;

      maxTTSize[1] = maxBTSize[1] = minQT[1];
      if( maxBTD[1] != 0 )
      {
        READ_UVLC( uiCode, "ph_log2_diff_max_bt_min_qt_inter_slice" );       maxBTSize[1] <<= uiCode;
        READ_UVLC( uiCode, "ph_log2_diff_max_tt_min_qt_inter_slice" );       maxTTSize[1] <<= uiCode;
      }
    }
    // delta quantization and chrom and chroma offset
    if( pps->getUseDQP() )
    {
      READ_UVLC( uiCode, "ph_cu_qp_delta_subdiv_inter_slice" );              picHeader->setCuQpDeltaSubdivInter( uiCode );
    }
    else
    {
      picHeader->setCuQpDeltaSubdivInter( 0 );
    }
    if( pps->getCuChromaQpOffsetEnabledFlag() )
    {
      READ_UVLC( uiCode, "ph_cu_chroma_qp_offset_subdiv_inter_slice" );      picHeader->setCuChromaQpOffsetSubdivInter( uiCode );
    }
    else
    {
      picHeader->setCuChromaQpOffsetSubdivInter( 0 );
    }

    // temporal motion vector prediction
    if( sps->getSPSTemporalMVPEnabledFlag() )
    {
      READ_FLAG( uiCode, "ph_temporal_mvp_enabled_flag" );                   picHeader->setEnableTMVPFlag( uiCode != 0 );
    }
    else
    {
      picHeader->setEnableTMVPFlag( false) ;
    }

    if( picHeader->getEnableTMVPFlag() && pps->getRplInfoInPhFlag() )
    {
      if( picHeader->getRPL(REF_PIC_LIST_1)->getNumRefEntries() > 0 )
      {
        READ_CODE( 1, uiCode, "ph_collocated_from_l0_flag" );                picHeader->setPicColFromL0Flag( uiCode );
      }
      else
      {
        picHeader->setPicColFromL0Flag(1);
      }
      if( ( picHeader->getPicColFromL0Flag() == 1 && picHeader->getRPL( REF_PIC_LIST_0 )->getNumRefEntries() > 1 ) ||
          ( picHeader->getPicColFromL0Flag() == 0 && picHeader->getRPL( REF_PIC_LIST_1 )->getNumRefEntries() > 1 ) )
      {
        READ_UVLC( uiCode, "ph_collocated_ref_idx" );                        picHeader->setColRefIdx( uiCode );
      }
      else
      {
        picHeader->setColRefIdx( 0 );
      }
    }
    else
    {
      picHeader->setPicColFromL0Flag( 0 );
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
      READ_FLAG( uiCode, "ph_fpel_mmvd_enabled_flag" );                      picHeader->setDisFracMMVD( uiCode != 0 );
    }
    else
    {
      picHeader->setDisFracMMVD( false );
    }

    // mvd L1 zero flag
    if( !pps->getRplInfoInPhFlag() || picHeader->getRPL( REF_PIC_LIST_1 )->getNumRefEntries() > 0 )
    {
      READ_FLAG( uiCode, "ph_mvd_l1_zero_flag" );                            picHeader->setMvdL1ZeroFlag( uiCode != 0 );
    }
    else
    {
      picHeader->setMvdL1ZeroFlag( true );
    }

    // picture level BDOF disable flags
    if( sps->getBdofControlPresentFlag() && ( !pps->getRplInfoInPhFlag() || picHeader->getRPL( REF_PIC_LIST_1 )->getNumRefEntries() > 0 ) )
    {
      READ_FLAG( uiCode, "ph_bdof_disabled_flag" );                          picHeader->setDisBdofFlag( uiCode != 0 );
    }
    else
    {
      if( sps->getBdofControlPresentFlag() == 0 )
      {
        picHeader->setDisBdofFlag( 1 - (int)( sps->getUseBIO() ) );
      }
      else
      {
        picHeader->setDisBdofFlag( 1 );
      }
    }

    // picture level DMVR disable flags
    if( sps->getDmvrControlPresentFlag() && ( !pps->getRplInfoInPhFlag() || picHeader->getRPL( REF_PIC_LIST_1 )->getNumRefEntries() > 0 ) )
    {
      READ_FLAG( uiCode, "ph_dmvr_disabled_flag" );                          picHeader->setDisDmvrFlag( uiCode != 0 );
    }
    else
    {
      if( sps->getDmvrControlPresentFlag() == 0 )
      {
        picHeader->setDisDmvrFlag( 1 - (int)( sps->getUseDMVR() ) );
      }
      else
      {
        picHeader->setDisDmvrFlag( 1 );
      }
    }

    // picture level PROF disable flags
    if( sps->getProfControlPresentFlag() )
    {
      READ_FLAG( uiCode, "ph_prof_disabled_flag" );                          picHeader->setDisProfFlag( uiCode != 0 );
    }
    else
    {
      picHeader->setDisProfFlag(0);
    }

    if( ( pps->getUseWP() || pps->getWPBiPred() ) && pps->getWpInfoInPhFlag() )
    {
      parsePredWeightTable( picHeader, sps );
    }
  } // if( picHeader->getPicInterSliceAllowedFlag() )

  // inherit constraint values from SPS
  if( !sps->getSplitConsOverrideEnabledFlag() || !picHeader->getSplitConsOverrideFlag() )
  {
    picHeader->setMinQTSizes( sps->getMinQTSizes() );
    picHeader->setMaxMTTHierarchyDepths( sps->getMaxBTSizes() );
    picHeader->setMaxBTSizes( sps->getMaxBTSizes() );
    picHeader->setMaxTTSizes( sps->getMaxTTSizes() );
  }
  else
  {
    picHeader->setMinQTSizes( minQT );
    picHeader->setMaxMTTHierarchyDepths( maxBTD );
    picHeader->setMaxBTSizes( maxBTSize );
    picHeader->setMaxTTSizes( maxTTSize );
  }

  // ibc merge candidate list size
  if( pps->getQpDeltaInfoInPhFlag() )
  {
    int iCode = 0;
    READ_SVLC( iCode, "ph_qp_delta" );                                       picHeader->setQpDelta( iCode );
  }

  // joint Cb/Cr sign flag
  if( sps->getJointCbCrEnabledFlag() )
  {
    READ_FLAG( uiCode, "ph_joint_cbcr_sign_flag" );                          picHeader->setJointCbCrSignFlag( uiCode != 0 );
  }
  else
  {
    picHeader->setJointCbCrSignFlag( false );
  }

  // sao enable flags
  if( sps->getUseSAO() )
  {
    if( pps->getSaoInfoInPhFlag() )
    {
      READ_FLAG( uiCode, "ph_sao_luma_enabled_flag" );                       picHeader->setSaoEnabledFlag( CHANNEL_TYPE_LUMA, uiCode != 0 );

      if( sps->getChromaFormatIdc() != CHROMA_400 )
      {
        READ_FLAG( uiCode, "ph_sao_chroma_enabled_flag" );                   picHeader->setSaoEnabledFlag( CHANNEL_TYPE_CHROMA, uiCode != 0 );
      }
    }
    else
    {
      picHeader->setSaoEnabledFlag( CHANNEL_TYPE_LUMA, true) ;
      picHeader->setSaoEnabledFlag( CHANNEL_TYPE_CHROMA, sps->getChromaFormatIdc() != CHROMA_400 );
    }
  }
  else
  {
    picHeader->setSaoEnabledFlag( CHANNEL_TYPE_LUMA,   false );
    picHeader->setSaoEnabledFlag( CHANNEL_TYPE_CHROMA, false );
  }

  // deblocking filter controls
  if( pps->getDeblockingFilterControlPresentFlag() )
  {
    if( pps->getDeblockingFilterOverrideEnabledFlag() )
    {
      if( pps->getDbfInfoInPhFlag() )
      {
        READ_FLAG ( uiCode, "ph_deblocking_params_present_flag" );           picHeader->setDeblockingFilterOverrideFlag( uiCode != 0 );
      }
      else
      {
        picHeader->setDeblockingFilterOverrideFlag( false );
      }
    }
    else
    {
      picHeader->setDeblockingFilterOverrideFlag( false );
    }

    if( picHeader->getDeblockingFilterOverrideFlag() )
    {
      if( !pps->getPPSDeblockingFilterDisabledFlag() )
      {
        READ_FLAG( uiCode, "ph_deblocking_filter_disabled_flag" );           picHeader->setDeblockingFilterDisable( uiCode != 0 );
      }
      else
      {
        picHeader->setDeblockingFilterDisable( false );
      }
      if( !picHeader->getDeblockingFilterDisable() )
      {
        READ_SVLC( iCode, "ph_luma_beta_offset_div2" );                      picHeader->setDeblockingFilterBetaOffsetDiv2( iCode );
        CHECK_RECOVERABLE( picHeader->getDeblockingFilterBetaOffsetDiv2() < -12 || picHeader->getDeblockingFilterBetaOffsetDiv2() > 12,
               "Invalid deblocking filter configuration" );

        READ_SVLC( iCode, "ph_luma_tc_offset_div2" );                        picHeader->setDeblockingFilterTcOffsetDiv2( iCode );
        CHECK_RECOVERABLE( picHeader->getDeblockingFilterTcOffsetDiv2() < -12 || picHeader->getDeblockingFilterTcOffsetDiv2() > 12,
               "Invalid deblocking filter configuration" );

        if( pps->getPPSChromaToolFlag() )
        {
          READ_SVLC( iCode, "ph_cb_beta_offset_div2" );                      picHeader->setDeblockingFilterCbBetaOffsetDiv2( iCode );
          CHECK_RECOVERABLE( picHeader->getDeblockingFilterCbBetaOffsetDiv2() < -12 || picHeader->getDeblockingFilterCbBetaOffsetDiv2() > 12,
                 "Invalid deblocking filter configuration" );

          READ_SVLC( iCode, "ph_cb_tc_offset_div2" );                        picHeader->setDeblockingFilterCbTcOffsetDiv2( iCode );
          CHECK_RECOVERABLE( picHeader->getDeblockingFilterCbTcOffsetDiv2() < -12 || picHeader->getDeblockingFilterCbTcOffsetDiv2() > 12,
                 "Invalid deblocking filter configuration" );

          READ_SVLC( iCode, "ph_cr_beta_offset_div2" );                      picHeader->setDeblockingFilterCrBetaOffsetDiv2( iCode );
          CHECK_RECOVERABLE( picHeader->getDeblockingFilterCrBetaOffsetDiv2() < -12 || picHeader->getDeblockingFilterCrBetaOffsetDiv2() > 12,
                 "Invalid deblocking filter configuration" );

          READ_SVLC( iCode, "ph_cr_tc_offset_div2" );                        picHeader->setDeblockingFilterCrTcOffsetDiv2( iCode );
          CHECK_RECOVERABLE( picHeader->getDeblockingFilterCrTcOffsetDiv2() < -12 || picHeader->getDeblockingFilterCrTcOffsetDiv2() > 12,
                 "Invalid deblocking filter configuration" );
        }
        else
        {
          picHeader->setDeblockingFilterCbBetaOffsetDiv2 ( picHeader->getDeblockingFilterBetaOffsetDiv2() );
          picHeader->setDeblockingFilterCbTcOffsetDiv2   ( picHeader->getDeblockingFilterTcOffsetDiv2()   );
          picHeader->setDeblockingFilterCrBetaOffsetDiv2 ( picHeader->getDeblockingFilterBetaOffsetDiv2() );
          picHeader->setDeblockingFilterCrTcOffsetDiv2   ( picHeader->getDeblockingFilterTcOffsetDiv2()   );
        }
      }
    }
    else
    {
      picHeader->setDeblockingFilterDisable       ( pps->getPPSDeblockingFilterDisabledFlag() );
      picHeader->setDeblockingFilterBetaOffsetDiv2( pps->getDeblockingFilterBetaOffsetDiv2() );
      picHeader->setDeblockingFilterTcOffsetDiv2  ( pps->getDeblockingFilterTcOffsetDiv2() );
      picHeader->setDeblockingFilterCbBetaOffsetDiv2( pps->getDeblockingFilterCbBetaOffsetDiv2() );
      picHeader->setDeblockingFilterCbTcOffsetDiv2  ( pps->getDeblockingFilterCbTcOffsetDiv2() );
      picHeader->setDeblockingFilterCrBetaOffsetDiv2( pps->getDeblockingFilterCrBetaOffsetDiv2() );
      picHeader->setDeblockingFilterCrTcOffsetDiv2  ( pps->getDeblockingFilterCrTcOffsetDiv2() );
    }
  }
  else
  {
    picHeader->setDeblockingFilterDisable       ( false );
    picHeader->setDeblockingFilterBetaOffsetDiv2( 0 );
    picHeader->setDeblockingFilterTcOffsetDiv2  ( 0 );
    picHeader->setDeblockingFilterCbBetaOffsetDiv2( 0 );
    picHeader->setDeblockingFilterCbTcOffsetDiv2  ( 0 );
    picHeader->setDeblockingFilterCrBetaOffsetDiv2( 0 );
    picHeader->setDeblockingFilterCrTcOffsetDiv2( 0 );
  }


  // picture header extension
  if( pps->getPictureHeaderExtensionPresentFlag() )
  {
    READ_UVLC( uiCode, "ph_extension_length" );
    for( int i = 0; i < uiCode; i++ )
    {
      uint32_t ignore_;
      READ_CODE( 8, ignore_, "ph_extension_data_byte[i]" );
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
    const std::vector<int>& apsIds = picHeader->getAlfAPSIds();
    //luma
    for( int i = 0; i < picHeader->getNumAlfAps(); i++ )
    {
      aps = parameterSetManager->getAPS( apsIds[i], ALF_APS );
      CHECK_RECOVERABLE( aps->getTemporalId() > curPicTid, "The TemporalId of the APS NAL unit having aps_params_type equal to ALF_APS and adaptation_parameter_set_id equal to ph_alf_aps_id_luma[i] shall be less than or equal to the TemporalId of the picture associated with the PH." );
      if( pcSlice->getNalUnitLayerId() != aps->getLayerId() )
      {
        CHECK_RECOVERABLE( aps->getLayerId() > pcSlice->getNalUnitLayerId(), "Layer Id of APS cannot be greater than layer Id of VCL NAL unit the refer to it" );
        CHECK_RECOVERABLE( pcSlice->getSPS()->getVPSId() == 0, "VPSId of the referred SPS cannot be 0 when layer Id of APS and layer Id of current slice are different" );
        for( int i = 0; i < pcSlice->getVPS()->getNumOutputLayerSets(); i++ )
        {
          bool isCurrLayerInOls = false;
          bool isRefLayerInOls = false;
          for( int j = pcSlice->getVPS()->getNumLayersInOls(i) - 1; j >= 0; j-- )
          {
            if( pcSlice->getVPS()->getLayerIdInOls(i, j) == pcSlice->getNalUnitLayerId() )
            {
              isCurrLayerInOls = true;
            }
            if( pcSlice->getVPS()->getLayerIdInOls(i, j) == aps->getLayerId() )
            {
              isRefLayerInOls = true;
            }
          }
          CHECK_RECOVERABLE( isCurrLayerInOls && !isRefLayerInOls, "When VCL NAl unit in layer A refers to APS in layer B, all OLS that contains layer A shall also contains layer B" );
        }
      }
    }
    //chroma
    if( picHeader->getAlfEnabledFlag(COMPONENT_Cb) || picHeader->getAlfEnabledFlag( COMPONENT_Cr ) )
    {
      int chromaAlfApsId = picHeader->getAlfApsIdChroma();
      aps = parameterSetManager->getAPS( chromaAlfApsId, ALF_APS );
      CHECK_RECOVERABLE( aps->getTemporalId() > curPicTid, "The TemporalId of the APS NAL unit having aps_params_type equal to ALF_APS and adaptation_parameter_set_id equal to ph_alf_aps_id_chroma shall be less than or equal to the TemporalId of the picture associated with the PH.") ;
      if( pcSlice->getNalUnitLayerId() != aps->getLayerId() )
      {
        CHECK_RECOVERABLE( aps->getLayerId() > pcSlice->getNalUnitLayerId(), "Layer Id of APS cannot be greater than layer Id of VCL NAL unit the refer to it" );
        CHECK_RECOVERABLE( pcSlice->getSPS()->getVPSId() == 0, "VPSId of the referred SPS cannot be 0 when layer Id of APS and layer Id of current slice are different" );
        for( int i = 0; i < pcSlice->getVPS()->getNumOutputLayerSets(); i++ )
        {
          bool isCurrLayerInOls = false;
          bool isRefLayerInOls = false;
          for( int j = pcSlice->getVPS()->getNumLayersInOls(i) - 1; j >= 0; j-- )
          {
            if( pcSlice->getVPS()->getLayerIdInOls(i, j) == pcSlice->getNalUnitLayerId() )
            {
              isCurrLayerInOls = true;
            }
            if( pcSlice->getVPS()->getLayerIdInOls(i, j) == aps->getLayerId() )
            {
              isRefLayerInOls = true;
            }
          }
          CHECK_RECOVERABLE( isCurrLayerInOls && !isRefLayerInOls, "When VCL NAl unit in layer A refers to APS in layer B, all OLS that contains layer A shall also contains layer B" );
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
  uint32_t uiCode = 0;
  int      iCode  = 0;

#if ENABLE_TRACING
  xTraceSliceHeader();
#endif

  READ_FLAG( uiCode, "sh_picture_header_in_slice_header_flag" );
  if( uiCode )
  {
    pcSlice->setPictureHeaderInSliceHeader( true );
    picHeader.reset( new PicHeader );
    parsePictureHeader( picHeader.get(), parameterSetManager, false );
  }
  CHECK            ( !picHeader,            "Picture Header not allocated" );   // should always be allocated, even if it is not valid
  CHECK_RECOVERABLE( !picHeader->isValid(), "Picture Header missing" );

  checkAlfNaluTidAndPicTid( pcSlice, picHeader.get(), parameterSetManager );

  const PPS* pps = parameterSetManager->getPPS( picHeader->getPPSId() );
  CHECK_RECOVERABLE( pps==0, "Invalid PPS" );
  const SPS* sps = parameterSetManager->getSPS( pps->getSPSId() );
  CHECK_RECOVERABLE( sps==0, "Invalid SPS" );

  if( sps->getProfileTierLevel()->getConstraintInfo()->getPicHeaderInSliceHeaderConstraintFlag() )
  {
    CHECK_RECOVERABLE( pcSlice->getPictureHeaderInSliceHeader() == false, "PH shall be present in SH, when pic_header_in_slice_header_constraint_flag is equal to 1" );
  }
  if( pcSlice->getPictureHeaderInSliceHeader() )
  {
    CHECK_RECOVERABLE( pps->getRplInfoInPhFlag() == 1, "When sh_picture_header_in_slice_header_flag is equal to 1, rpl_info_in_ph_flag shall be equal to 0" );
    CHECK_RECOVERABLE( pps->getDbfInfoInPhFlag() == 1, "When sh_picture_header_in_slice_header_flag is equal to 1, dbf_info_in_ph_flag shall be equal to 0" );
    CHECK_RECOVERABLE( pps->getSaoInfoInPhFlag() == 1, "When sh_picture_header_in_slice_header_flag is equal to 1, sao_info_in_ph_flag shall be equal to 0" );
    CHECK_RECOVERABLE( pps->getAlfInfoInPhFlag() == 1, "When sh_picture_header_in_slice_header_flag is equal to 1, alf_info_in_ph_flag shall be equal to 0" );
    CHECK_RECOVERABLE( pps->getWpInfoInPhFlag() == 1, "When sh_picture_header_in_slice_header_flag is equal to 1, wp_info_in_ph_flag shall be equal to 0" );
    CHECK_RECOVERABLE( pps->getQpDeltaInfoInPhFlag() == 1, "When sh_picture_header_in_slice_header_flag is equal to 1, qp_delta_info_in_ph_flag shall be equal to 0" );
    CHECK_RECOVERABLE( sps->getSubPicInfoPresentFlag() == 1, "When sps_subpic_info_present_flag is equal to 1, the value of sh_picture_header_in_slice_header_flag shall be equal to 0" );
  }
  CHECK_RECOVERABLE( sps->getSubPicInfoPresentFlag() == 1 && sps->getVirtualBoundariesEnabledFlag() == 1 && sps->getVirtualBoundariesPresentFlag() == 0,
        "when sps_subpic_info_present_flag is equal to 1 and sps_virtual_boundaries_enabled_flag is equal to 1, sps_virtual_boundaries_present_flag shall be equal 1" );

  const ChromaFormat chFmt        = sps->getChromaFormatIdc();
  const uint32_t     numValidComp = getNumberValidComponents( chFmt );
  const bool         bChroma      = ( chFmt != CHROMA_400 );

  // picture order count
  uiCode         = picHeader->getPocLsb();
  int iPOClsb    = uiCode;
  int iMaxPOClsb = 1 << sps->getBitsForPOC();
  int iPOCmsb;
  if( pcSlice->getIdrPicFlag() )
  {
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
    int iPrevPOC    = prevTid0POC;
    int iPrevPOClsb = iPrevPOC & ( iMaxPOClsb - 1 );
    int iPrevPOCmsb = iPrevPOC - iPrevPOClsb;
    if( picHeader->getPocMsbPresentFlag() )
    {
      iPOCmsb = picHeader->getPocMsbVal() * iMaxPOClsb;
    }
    else
    {
      if( ( iPOClsb  <  iPrevPOClsb ) && ( ( iPrevPOClsb - iPOClsb ) >= ( iMaxPOClsb / 2 ) ) )
      {
        iPOCmsb = iPrevPOCmsb + iMaxPOClsb;
      }
      else if( ( iPOClsb  >  iPrevPOClsb ) && ( ( iPOClsb - iPrevPOClsb ) > ( iMaxPOClsb / 2 ) ) )
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
    uint32_t bitsSubPicId;
    bitsSubPicId = sps->getSubPicIdLen();
    READ_CODE( bitsSubPicId, uiCode, "sh_subpic_id" );
    pcSlice->setSliceSubPicId( uiCode );
  }
  else
  {
    pcSlice->setSliceSubPicId( 0 );
  }

  // raster scan slices
  uint32_t sliceAddr = 0;
  if( pps->getRectSliceFlag() == 0 )
  {
    // slice address is the raster scan tile index of first tile in slice
    if( pps->getNumTiles() > 1 )
    {
      int bitsSliceAddress = (int)ceil( log2( pps->getNumTiles() ) );
      READ_CODE( bitsSliceAddress, uiCode, "sh_slice_address" );
      sliceAddr = uiCode;
    }
  }
  // rectangular slices
  else
  {
    // slice address is the index of the slice within the current sub-picture
    uint32_t      currSubPicIdx = pps->getSubPicIdxFromSubPicId( pcSlice->getSliceSubPicId() );
    const SubPic& currSubPic    = pps->getSubPic( currSubPicIdx );
    if( currSubPic.getNumSlicesInSubPic() > 1 )
    {
      int bitsSliceAddress = (int)ceil( log2( currSubPic.getNumSlicesInSubPic() ) );
      READ_CODE( bitsSliceAddress, uiCode, "sh_slice_address" );
      sliceAddr = uiCode;
    }
    uint32_t picLevelSliceIdx = sliceAddr;
    for( int subpic = 0; subpic < currSubPicIdx; subpic++ )
    {
      picLevelSliceIdx += pps->getSubPic( subpic ).getNumSlicesInSubPic();
    }
    pcSlice->setSliceMap( pps->getSliceMap( picLevelSliceIdx ) );
    pcSlice->setSliceID( picLevelSliceIdx );
  }

  std::vector<bool> shExtraBitsPresent = sps->getExtraSHBitPresentFlags();
  for( int i=0; i< sps->getNumExtraSHBitsBytes() * 8; i++ )
  {
    // extra bits are ignored (when present)
    if( shExtraBitsPresent[i] )
    {
      READ_FLAG( uiCode, "sh_extra_bit[i]" );
    }
  }

  if( pps->getRectSliceFlag() == 0 )
  {
    uint32_t numTilesInSlice = 1;
    if( pps->getNumTiles() > 1 )
    {
      if( ( (int)pps->getNumTiles() - (int)sliceAddr ) > 1)
      {
        READ_UVLC( uiCode, "sh_num_tiles_in_slice_minus1" );
        numTilesInSlice = uiCode + 1;
      }
      if( !pps->getRectSliceFlag() && sps->getProfileTierLevel()->getConstraintInfo()->getOneSlicePerPicConstraintFlag() )
      {
        CHECK_RECOVERABLE( pps->getNumTiles() != uiCode + 1, "When rect_slice_flag is equal to 0 and one_slice_per_pic_constraint_flag equal to 1, the value of num_tiles_in_slice_minus1 present in each slice header shall be equal to NumTilesInPic - 1" );
      }
    }
    CHECK_RECOVERABLE( sliceAddr >= pps->getNumTiles(), "Invalid slice address" );
    pcSlice->resetSliceMap();
    pcSlice->setSliceID( sliceAddr );

    for( uint32_t tileIdx = sliceAddr; tileIdx < sliceAddr + numTilesInSlice; tileIdx++ )
    {
      uint32_t tileX = tileIdx % pps->getNumTileColumns();
      uint32_t tileY = tileIdx / pps->getNumTileColumns();
      CHECK_RECOVERABLE( tileY >= pps->getNumTileRows(), "Number of tiles in slice exceeds the remaining number of tiles in picture" );

      pcSlice->addCtusToSlice( pps->getTileColumnBd( tileX ), pps->getTileColumnBd( tileX + 1 ),
                               pps->getTileRowBd( tileY ), pps->getTileRowBd( tileY + 1 ), pps->getPicWidthInCtu() );
    }
  }

  if( firstSliceInPic != ( pcSlice->getCtuAddrInSlice( 0 ) == 0 ) )
  {
    // exit early, because we need to start again with some fields copied from previous slice
    firstSliceInPic = false;
    return;
  }

  if( picHeader->getPicInterSliceAllowedFlag() )
  {
    READ_UVLC( uiCode, "sh_slice_type" );
    pcSlice->setSliceType( (SliceType)uiCode );
  }
  else
  {
    pcSlice->setSliceType( I_SLICE );
  }
  if( !picHeader->getPicIntraSliceAllowedFlag() )
  {
    CHECK_RECOVERABLE( pcSlice->getSliceType() == I_SLICE, "when pic_intra_slice_allowed_flag = 0, no I_Slice is allowed" );
  }
  if( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_GDR )
  {
    READ_FLAG(uiCode, "sh_no_output_of_prior_pics_flag" );
    pcSlice->setNoOutputOfPriorPicsFlag( uiCode != 0 );
  }
  // inherit values from picture header
  //   set default values in case slice overrides are disabled
  pcSlice->inheritFromPicHeader( picHeader.get(), pps, sps );

  if( sps->getUseALF() && !pps->getAlfInfoInPhFlag() )
  {
    READ_FLAG( uiCode, "sh_alf_enabled_flag" );
    pcSlice->setAlfEnabledFlag( COMPONENT_Y, uiCode );
    int alfCbEnabledFlag = 0;
    int alfCrEnabledFlag = 0;

    if( uiCode )
    {
      READ_CODE( 3, uiCode, "sh_num_alf_aps_ids_luma" );
      int numAps = uiCode;
      pcSlice->setNumAlfAps( numAps );
      std::vector<int> apsId( numAps, -1) ;
      for( int i = 0; i < numAps; i++ )
      {
        READ_CODE( 3, uiCode, "sh_alf_aps_id_luma[i]" );
        apsId[i] = uiCode;
        const APS* APStoCheckLuma = parameterSetManager->getAPS( apsId[i], ALF_APS );
        CHECK_RECOVERABLE( APStoCheckLuma == nullptr, "referenced APS not found" );
        CHECK_RECOVERABLE( APStoCheckLuma->getAlfAPSParam().newFilterFlag[CHANNEL_TYPE_LUMA] != 1, "bitstream conformance error, alf_luma_filter_signal_flag shall be equal to 1" );
      }

      pcSlice->setAlfApsIdLuma( apsId );

      if( bChroma )
      {
        READ_CODE( 1, uiCode, "sh_alf_cb_enabled_flag" );  alfCbEnabledFlag = uiCode;
        READ_CODE( 1, uiCode, "sh_alf_cr_enabled_flag" );  alfCrEnabledFlag = uiCode;
      }
      else
      {
        alfCbEnabledFlag = 0;
        alfCrEnabledFlag = 0;
      }
      if( alfCbEnabledFlag || alfCrEnabledFlag )
      {
        READ_CODE( 3, uiCode, "sh_alf_aps_id_chroma" );
        pcSlice->setAlfApsIdChroma( uiCode );
        const APS* APStoCheckChroma = parameterSetManager->getAPS( uiCode, ALF_APS );
        CHECK_RECOVERABLE( APStoCheckChroma == nullptr, "referenced APS not found" );
        CHECK_RECOVERABLE( APStoCheckChroma->getAlfAPSParam().newFilterFlag[CHANNEL_TYPE_CHROMA] != 1, "bitstream conformance error, alf_chroma_filter_signal_flag shall be equal to 1" );
      }
    }
    else
    {
      pcSlice->setNumAlfAps( 0 );
    }
    pcSlice->setAlfEnabledFlag( COMPONENT_Cb, alfCbEnabledFlag );
    pcSlice->setAlfEnabledFlag( COMPONENT_Cr, alfCrEnabledFlag );

    if( sps->getUseCCALF() && pcSlice->getAlfEnabledFlag( COMPONENT_Y ) )
    {
      READ_FLAG( uiCode, "sh_alf_cc_cb_enabled_flag" );
      pcSlice->setCcAlfCbEnabledFlag( uiCode );
      pcSlice->setCcAlfCbApsId( -1 );
      if( uiCode )
      {
        // parse APS ID
        READ_CODE( 3, uiCode, "sh_cc_alf_cb_aps_id" );
        pcSlice->setCcAlfCbApsId( uiCode );
      }
      // Cr
      READ_FLAG(uiCode, "sh_alf_cc_cr_enabled_flag" );
      pcSlice->setCcAlfCrEnabledFlag( uiCode );
      pcSlice->setCcAlfCrApsId(-1);
      if( uiCode )
      {
        // parse APS ID
        READ_CODE( 3, uiCode, "sh_cc_alf_cr_aps_id" );
        pcSlice->setCcAlfCrApsId( uiCode );
      }
    }
    else
    {
      pcSlice->setCcAlfCbEnabledFlag( 0 );
      pcSlice->setCcAlfCrEnabledFlag( 0 );
      pcSlice->setCcAlfCbApsId( -1 );
      pcSlice->setCcAlfCrApsId( -1 );
    }
  }
  if( picHeader->getLmcsEnabledFlag() && !pcSlice->getPictureHeaderInSliceHeader() )
  {
    READ_FLAG( uiCode, "sh_lmcs_used_flag" );
    pcSlice->setLmcsEnabledFlag( uiCode );
  }
  else
  {
    pcSlice->setLmcsEnabledFlag( pcSlice->getPictureHeaderInSliceHeader() ? picHeader->getLmcsEnabledFlag() : false );
  }
  if( picHeader->getExplicitScalingListEnabledFlag() && !pcSlice->getPictureHeaderInSliceHeader() )
  {
    READ_FLAG( uiCode, "sh_explicit_scaling_list_used_flag" );
    pcSlice->setExplicitScalingListUsed( uiCode );
  }
  else
  {
    pcSlice->setExplicitScalingListUsed( pcSlice->getPictureHeaderInSliceHeader() ? picHeader->getExplicitScalingListEnabledFlag() : false );
  }

  if( pps->getRplInfoInPhFlag() )
  {
    pcSlice->setRPL( REF_PIC_LIST_0, *picHeader->getRPL( REF_PIC_LIST_0 ) );
    pcSlice->setRPL( REF_PIC_LIST_1, *picHeader->getRPL( REF_PIC_LIST_1 ) );
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

  if( !pps->getRplInfoInPhFlag() && pcSlice->getIdrPicFlag() && !(sps->getIDRRefParamListPresent()) )
  {
    pcSlice->setNumRefIdx(REF_PIC_LIST_0, 0);
    pcSlice->setNumRefIdx(REF_PIC_LIST_1, 0);
  }

  if( ( !pcSlice->isIntra() && pcSlice->getRPL( REF_PIC_LIST_0 )->getNumRefEntries() > 1 ) ||
      ( pcSlice->isInterB() && pcSlice->getRPL( REF_PIC_LIST_1 )->getNumRefEntries() > 1 ) )
  {
    READ_FLAG( uiCode, "sh_num_ref_idx_active_override_flag" );
    if( uiCode )
    {
      if( pcSlice->getRPL( REF_PIC_LIST_0 )->getNumRefEntries() > 1 )
      {
        READ_UVLC( uiCode, "sh_num_ref_idx_active_minus1[ 0 ]" );
      }
      else
      {
        uiCode = 0;
      }
      pcSlice->setNumRefIdx( REF_PIC_LIST_0, uiCode + 1 );
      if( pcSlice->isInterB() )
      {
        if( pcSlice->getRPL( REF_PIC_LIST_1 )->getNumRefEntries() > 1 )
        {
          READ_UVLC( uiCode, "sh_num_ref_idx_active_minus1[ 1 ]" );
        }
        else
        {
          uiCode = 0;
        }
        pcSlice->setNumRefIdx( REF_PIC_LIST_1, uiCode + 1 );
      }
      else
      {
        pcSlice->setNumRefIdx( REF_PIC_LIST_1, 0 );
      }
    }
    else
    {
      if( pcSlice->getRPL( REF_PIC_LIST_0 )->getNumRefEntries() >= pps->getNumRefIdxL0DefaultActive() )
      {
        pcSlice->setNumRefIdx( REF_PIC_LIST_0, pps->getNumRefIdxL0DefaultActive() );
      }
      else
      {
        pcSlice->setNumRefIdx( REF_PIC_LIST_0, pcSlice->getRPL( REF_PIC_LIST_0 )->getNumRefEntries() );
      }

      if( pcSlice->isInterB() )
      {
        if( pcSlice->getRPL( REF_PIC_LIST_1 )->getNumRefEntries() >= pps->getNumRefIdxL1DefaultActive() )
        {
          pcSlice->setNumRefIdx( REF_PIC_LIST_1, pps->getNumRefIdxL1DefaultActive() );
        }
        else
        {
          pcSlice->setNumRefIdx( REF_PIC_LIST_1, pcSlice->getRPL( REF_PIC_LIST_1 )->getNumRefEntries() );
        }
      }
      else
      {
        pcSlice->setNumRefIdx( REF_PIC_LIST_1, 0 );
      }
    }
  }
  else
  {
    if( !pcSlice->isIntra() )
    {
      pcSlice->setNumRefIdx( REF_PIC_LIST_0, pcSlice->getRPL( REF_PIC_LIST_0 )->getNumRefEntries() );
    }
    if( pcSlice->isInterB() )
    {
      pcSlice->setNumRefIdx( REF_PIC_LIST_1, pcSlice->getRPL( REF_PIC_LIST_1 )->getNumRefEntries() );
    }
  }

  if( pcSlice->isInterP() || pcSlice->isInterB() )
  {
    CHECK_RECOVERABLE( pcSlice->getNumRefIdx(REF_PIC_LIST_0) == 0, "Number of active entries in RPL0 of P or B picture shall be greater than 0" );
    if( pcSlice->isInterB() )
    {
      CHECK_RECOVERABLE( pcSlice->getNumRefIdx(REF_PIC_LIST_1) == 0, "Number of active entries in RPL1 of B picture shall be greater than 0" );
    }
  }

  pcSlice->setCabacInitFlag( false ); // default
  if( pps->getCabacInitPresentFlag() && !pcSlice->isIntra() )
  {
    READ_FLAG( uiCode, "sh_cabac_init_flag" );
    pcSlice->setCabacInitFlag( uiCode ? true : false );
  }

  if( picHeader->getEnableTMVPFlag() )
  {
    if( pcSlice->getSliceType() == P_SLICE )
    {
      pcSlice->setColFromL0Flag( true );
    }
    else if( !pps->getRplInfoInPhFlag() && pcSlice->getSliceType() == B_SLICE )
    {
      READ_FLAG( uiCode, "sh_collocated_from_l0_flag" );
      pcSlice->setColFromL0Flag( uiCode );
    }
    else
    {
      pcSlice->setColFromL0Flag( picHeader->getPicColFromL0Flag() );
    }

    if( !pps->getRplInfoInPhFlag() )
    {
      if( pcSlice->getSliceType() != I_SLICE &&
          ( ( pcSlice->getColFromL0Flag() == 1 && pcSlice->getNumRefIdx( REF_PIC_LIST_0 ) > 1 ) ||
            ( pcSlice->getColFromL0Flag() == 0 && pcSlice->getNumRefIdx( REF_PIC_LIST_1 ) > 1 ) ) )
      {
        READ_UVLC( uiCode, "sh_collocated_ref_idx" );
        pcSlice->setColRefIdx( uiCode );
      }
      else
      {
        pcSlice->setColRefIdx( 0 );
      }
    }
    else
    {
      pcSlice->setColRefIdx( picHeader->getColRefIdx() );
    }
  }
  if( ( pps->getUseWP() && pcSlice->getSliceType() == P_SLICE ) || ( pps->getWPBiPred() && pcSlice->getSliceType() == B_SLICE ) )
  {
    if( pps->getWpInfoInPhFlag() )
    {
      CHECK_RECOVERABLE( pcSlice->getNumRefIdx(REF_PIC_LIST_0) > picHeader->getNumL0Weights(), "ERROR: Number of active reference picture L0 is greater than the number of weighted prediction signalled in Picture Header" );
      CHECK_RECOVERABLE( pcSlice->getNumRefIdx(REF_PIC_LIST_1) > picHeader->getNumL1Weights(), "ERROR: Number of active reference picture L1 is greater than the number of weighted prediction signalled in Picture Header" );
      pcSlice->setWpScaling( picHeader->getWpScalingAll() );
    }
    else
    {
      parsePredWeightTable( pcSlice, sps );
    }
    pcSlice->initWpScaling( sps );
  }
  else
  {
    WPScalingParam *wp;
    for( int iNumRef = 0 ; iNumRef < ( ( pcSlice->getSliceType() == B_SLICE ) ? 2 : 1 ); iNumRef++ )
    {
      RefPicList  eRefPicList = ( iNumRef ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
      for( int iRefIdx=0; iRefIdx<pcSlice->getNumRefIdx( eRefPicList ); iRefIdx++ )
      {
        pcSlice->getWpScaling( eRefPicList, iRefIdx, wp );
        wp[0].bPresentFlag = false;
        wp[1].bPresentFlag = false;
        wp[2].bPresentFlag = false;
      }
    }
  }

  int qpDelta = 0;
  if( pps->getQpDeltaInfoInPhFlag() )
  {
    qpDelta = picHeader->getQpDelta();
  }
  else
  {
    READ_SVLC( iCode, "sh_qp_delta" );
    qpDelta = iCode;
  }
  pcSlice->setSliceQp( 26 + pps->getPicInitQPMinus26() + qpDelta );
  pcSlice->setSliceQpBase( pcSlice->getSliceQp() );

  CHECK_RECOVERABLE( pcSlice->getSliceQp() < -sps->getQpBDOffset( CHANNEL_TYPE_LUMA ), "Invalid slice QP delta" );
  CHECK_RECOVERABLE( pcSlice->getSliceQp() > MAX_QP, "Invalid slice QP" );

  if( pps->getSliceChromaQpFlag() )
  {
    if( numValidComp > COMPONENT_Cb )
    {
      READ_SVLC( iCode, "sh_cb_qp_offset" );
      pcSlice->setSliceChromaQpDelta( COMPONENT_Cb, iCode );
      CHECK_RECOVERABLE( pcSlice->getSliceChromaQpDelta( COMPONENT_Cb ) < -12, "Invalid chroma QP offset" );
      CHECK_RECOVERABLE( pcSlice->getSliceChromaQpDelta( COMPONENT_Cb ) > 12, "Invalid chroma QP offset" );
      CHECK_RECOVERABLE( ( pps->getQpOffset( COMPONENT_Cb ) + pcSlice->getSliceChromaQpDelta( COMPONENT_Cb ) ) < -12, "Invalid chroma QP offset" );
      CHECK_RECOVERABLE( ( pps->getQpOffset( COMPONENT_Cb ) + pcSlice->getSliceChromaQpDelta( COMPONENT_Cb ) ) > 12, "Invalid chroma QP offset" );
    }

    if( numValidComp > COMPONENT_Cr )
    {
      READ_SVLC( iCode, "sh_cr_qp_offset" );
      pcSlice->setSliceChromaQpDelta( COMPONENT_Cr, iCode );
      CHECK_RECOVERABLE( pcSlice->getSliceChromaQpDelta( COMPONENT_Cr ) < -12, "Invalid chroma QP offset" );
      CHECK_RECOVERABLE( pcSlice->getSliceChromaQpDelta( COMPONENT_Cr ) > 12, "Invalid chroma QP offset" );
      CHECK_RECOVERABLE( ( pps->getQpOffset( COMPONENT_Cr ) + pcSlice->getSliceChromaQpDelta( COMPONENT_Cr ) ) < -12, "Invalid chroma QP offset" );
      CHECK_RECOVERABLE( ( pps->getQpOffset( COMPONENT_Cr ) + pcSlice->getSliceChromaQpDelta( COMPONENT_Cr ) ) > 12, "Invalid chroma QP offset" );
      if( sps->getJointCbCrEnabledFlag() )
      {
        READ_SVLC( iCode, "sh_joint_cbcr_qp_offset" );
        pcSlice->setSliceChromaQpDelta( JOINT_CbCr, iCode );
        CHECK_RECOVERABLE( pcSlice->getSliceChromaQpDelta( JOINT_CbCr ) < -12, "Invalid chroma QP offset" );
        CHECK_RECOVERABLE( pcSlice->getSliceChromaQpDelta( JOINT_CbCr ) > 12, "Invalid chroma QP offset" );
        CHECK_RECOVERABLE( ( pps->getQpOffset( JOINT_CbCr ) + pcSlice->getSliceChromaQpDelta( JOINT_CbCr ) ) < -12, "Invalid chroma QP offset" );
        CHECK_RECOVERABLE( ( pps->getQpOffset( JOINT_CbCr ) + pcSlice->getSliceChromaQpDelta( JOINT_CbCr ) ) > 12, "Invalid chroma QP offset" );
      }
    }
  }

  if( pps->getCuChromaQpOffsetEnabledFlag() )
  {
    READ_FLAG( uiCode, "sh_cu_chroma_qp_offset_enabled_flag");
    pcSlice->setUseChromaQpAdj( uiCode != 0 );
  }
  else
  {
    pcSlice->setUseChromaQpAdj( false );
  }

  if( sps->getUseSAO() && !pps->getSaoInfoInPhFlag() )
  {
    READ_FLAG( uiCode, "sh_sao_luma_used_flag" );
    pcSlice->setSaoEnabledFlag( CHANNEL_TYPE_LUMA, (bool)uiCode );

    if( bChroma )
    {
      READ_FLAG( uiCode, "sh_sao_chroma_used_flag" );
      pcSlice->setSaoEnabledFlag( CHANNEL_TYPE_CHROMA, (bool)uiCode );
    }
  }

  if( pps->getDeblockingFilterControlPresentFlag() )
  {
    if( pps->getDeblockingFilterOverrideEnabledFlag() && !pps->getDbfInfoInPhFlag() )
    {
      READ_FLAG ( uiCode, "sh_deblocking_params_present_flag" );
      pcSlice->setDeblockingFilterOverrideFlag( uiCode ? true : false );
    }
    else
    {
      pcSlice->setDeblockingFilterOverrideFlag( 0 );
    }
    if(pcSlice->getDeblockingFilterOverrideFlag())
    {
      if (!pps->getPPSDeblockingFilterDisabledFlag())
      {
        READ_FLAG ( uiCode, "sh_deblocking_filter_disabled_flag" );
        pcSlice->setDeblockingFilterDisable( uiCode ? 1 : 0 );
      }
      else
      {
        pcSlice->setDeblockingFilterDisable( false );
      }
      if( !pcSlice->getDeblockingFilterDisable() )
      {
        READ_SVLC( iCode, "sh_luma_beta_offset_div2" );
        pcSlice->setDeblockingFilterBetaOffsetDiv2( iCode );
        CHECK_RECOVERABLE( pcSlice->getDeblockingFilterBetaOffsetDiv2() < -12 || pcSlice->getDeblockingFilterBetaOffsetDiv2() > 12,
               "Invalid deblocking filter configuration" );
        READ_SVLC( iCode, "sh_luma_tc_offset_div2" );
        pcSlice->setDeblockingFilterTcOffsetDiv2( iCode );
        CHECK_RECOVERABLE( pcSlice->getDeblockingFilterTcOffsetDiv2() < -12 || pcSlice->getDeblockingFilterTcOffsetDiv2() > 12, "Invalid deblocking filter configuration" );

        if( pps->getPPSChromaToolFlag() )
        {
          READ_SVLC( iCode, "sh_cb_beta_offset_div2" );
          pcSlice->setDeblockingFilterCbBetaOffsetDiv2( iCode );
          CHECK_RECOVERABLE( pcSlice->getDeblockingFilterCbBetaOffsetDiv2() < -12 || pcSlice->getDeblockingFilterCbBetaOffsetDiv2() > 12,
                 "Invalid deblocking filter configuration" );
          READ_SVLC( iCode, "sh_cb_tc_offset_div2" );
          pcSlice->setDeblockingFilterCbTcOffsetDiv2( iCode );
          CHECK_RECOVERABLE( pcSlice->getDeblockingFilterCbTcOffsetDiv2() < -12 || pcSlice->getDeblockingFilterCbTcOffsetDiv2() > 12,
                 "Invalid deblocking filter configuration" );

          READ_SVLC( iCode, "sh_cr_beta_offset_div2" );
          pcSlice->setDeblockingFilterCrBetaOffsetDiv2( iCode );
          CHECK_RECOVERABLE( pcSlice->getDeblockingFilterCrBetaOffsetDiv2() < -12 || pcSlice->getDeblockingFilterCrBetaOffsetDiv2() > 12,
                 "Invalid deblocking filter configuration" );
          READ_SVLC( iCode, "sh_cr_tc_offset_div2" );
          pcSlice->setDeblockingFilterCrTcOffsetDiv2( iCode );
          CHECK_RECOVERABLE( pcSlice->getDeblockingFilterCrTcOffsetDiv2() < -12 || pcSlice->getDeblockingFilterCrTcOffsetDiv2() > 12,
                 "Invalid deblocking filter configuration" );
        }
        else
        {
          pcSlice->setDeblockingFilterCbBetaOffsetDiv2 ( pcSlice->getDeblockingFilterBetaOffsetDiv2() );
          pcSlice->setDeblockingFilterCbTcOffsetDiv2   ( pcSlice->getDeblockingFilterTcOffsetDiv2()   );
          pcSlice->setDeblockingFilterCrBetaOffsetDiv2 ( pcSlice->getDeblockingFilterBetaOffsetDiv2() );
          pcSlice->setDeblockingFilterCrTcOffsetDiv2   ( pcSlice->getDeblockingFilterTcOffsetDiv2()   );
        }
      }
    }
    else
    {
      pcSlice->setDeblockingFilterDisable         ( picHeader->getDeblockingFilterDisable() );
      pcSlice->setDeblockingFilterBetaOffsetDiv2  ( picHeader->getDeblockingFilterBetaOffsetDiv2() );
      pcSlice->setDeblockingFilterTcOffsetDiv2    ( picHeader->getDeblockingFilterTcOffsetDiv2() );
      pcSlice->setDeblockingFilterCbBetaOffsetDiv2( picHeader->getDeblockingFilterCbBetaOffsetDiv2() );
      pcSlice->setDeblockingFilterCbTcOffsetDiv2  ( picHeader->getDeblockingFilterCbTcOffsetDiv2() );
      pcSlice->setDeblockingFilterCrBetaOffsetDiv2( picHeader->getDeblockingFilterCrBetaOffsetDiv2() );
      pcSlice->setDeblockingFilterCrTcOffsetDiv2  ( picHeader->getDeblockingFilterCrTcOffsetDiv2() );
    }
  }
  else
  {
    pcSlice->setDeblockingFilterDisable       ( false );
    pcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
    pcSlice->setDeblockingFilterTcOffsetDiv2  ( 0 );
    pcSlice->setDeblockingFilterCbBetaOffsetDiv2( 0 );
    pcSlice->setDeblockingFilterCbTcOffsetDiv2  ( 0 );
    pcSlice->setDeblockingFilterCrBetaOffsetDiv2( 0 );
    pcSlice->setDeblockingFilterCrTcOffsetDiv2  ( 0 );
  }

  // dependent quantization
  if( sps->getDepQuantEnabledFlag() )
  {
    READ_FLAG( uiCode, "sh_dep_quant_used_flag" );
    pcSlice->setDepQuantEnabledFlag( uiCode != 0 );
  }
  else
  {
    pcSlice->setDepQuantEnabledFlag( false );
  }

  // sign data hiding
  if( sps->getSignDataHidingEnabledFlag() && !pcSlice->getDepQuantEnabledFlag() )
  {
    READ_FLAG( uiCode, "sh_sign_data_hiding_used_flag" );
    pcSlice->setSignDataHidingEnabledFlag( uiCode != 0 );
  }
  else
  {
    pcSlice->setSignDataHidingEnabledFlag( false );
  }

  // signal TS residual coding disabled flag
  if( sps->getTransformSkipEnabledFlag() && !pcSlice->getDepQuantEnabledFlag() && !pcSlice->getSignDataHidingEnabledFlag() )
  {
    READ_FLAG( uiCode, "sh_ts_residual_coding_disabled_flag" );
    pcSlice->setTSResidualCodingDisabledFlag( uiCode != 0 );
  }
  else
  {
    pcSlice->setTSResidualCodingDisabledFlag( false );
  }

  if( pcSlice->getFirstCtuRsAddrInSlice() == 0 )
  {
    pcSlice->setDefaultClpRng( *sps );
  }

  if( pps->getSliceHeaderExtensionPresentFlag() )
  {
    READ_UVLC( uiCode, "sh_slice_header_extension_length ");
    for( int i = 0; i < uiCode; i++ )
    {
      uint32_t ignore_;
      READ_CODE( 8, ignore_, "sh_slice_header_extension_data_byte[ i ]" );
    }
  }

  std::vector<uint32_t> entryPointOffset;
  pcSlice->setNumEntryPoints( sps, pps );
  if( pcSlice->getNumEntryPoints() > 0 )
  {
    uint32_t offsetLenMinus1;
    READ_UVLC( offsetLenMinus1, "sh_offset_len_minus1" );
    entryPointOffset.resize( pcSlice->getNumEntryPoints() );
    for( uint32_t idx = 0; idx < pcSlice->getNumEntryPoints(); idx++ )
    {
      READ_CODE( offsetLenMinus1 + 1, uiCode, "sh_entry_point_offset_minus1[i]" );
      entryPointOffset[idx] = uiCode + 1;
    }
  }

#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::IncrementStatisticEP(STATS__BYTE_ALIGNMENT_BITS,m_pcBitstream->readByteAlignment(),0);
#else
  m_pcBitstream->readByteAlignment();
#endif

  pcSlice->clearSubstreamSizes();

  if( pcSlice->getNumEntryPoints() > 0 )
  {
    int endOfSliceHeaderLocation = m_pcBitstream->getByteLocation();

    // Adjust endOfSliceHeaderLocation to account for emulation prevention bytes in the slice segment header
    for( uint32_t curByteIdx  = 0; curByteIdx<m_pcBitstream->numEmulationPreventionBytesRead(); curByteIdx++ )
    {
      if( m_pcBitstream->getEmulationPreventionByteLocation( curByteIdx ) < endOfSliceHeaderLocation )
      {
        endOfSliceHeaderLocation++;
      }
    }

    int  curEntryPointOffset     = 0;
    int  prevEntryPointOffset    = 0;
    for( uint32_t idx = 0; idx < entryPointOffset.size(); idx++ )
    {
      curEntryPointOffset += entryPointOffset[ idx ];

      int emulationPreventionByteCount = 0;
      for( uint32_t curByteIdx  = 0; curByteIdx<m_pcBitstream->numEmulationPreventionBytesRead(); curByteIdx++ )
      {
        if( m_pcBitstream->getEmulationPreventionByteLocation( curByteIdx ) >= ( prevEntryPointOffset + endOfSliceHeaderLocation ) &&
            m_pcBitstream->getEmulationPreventionByteLocation( curByteIdx ) <  ( curEntryPointOffset  + endOfSliceHeaderLocation ) )
        {
          emulationPreventionByteCount++;
        }
      }

      entryPointOffset[ idx ] -= emulationPreventionByteCount;
      prevEntryPointOffset = curEntryPointOffset;
      pcSlice->addSubstreamSize(entryPointOffset [ idx ] );
    }
  }
  return;
}

template<typename HeaderT>
void HLSyntaxReader::parsePicOrSliceHeaderRPL( HeaderT* header, const SPS* sps, const PPS* pps )
{
  uint32_t  uiCode;

  // List0 and List1
  for( int iListIdx = 0; iListIdx < 2; iListIdx++ )
  {
    const RefPicList listIdx = static_cast<RefPicList>( iListIdx );

    bool readExplicitRPL = false;

    // copy L1 index from L0 index
    if( listIdx == REF_PIC_LIST_1 && !pps->getRpl1IdxPresentFlag() )
    {
      const int rpl0idx = header->getRPLIdx( REF_PIC_LIST_0 );
      header->setRPLIdx( REF_PIC_LIST_1, rpl0idx );
      readExplicitRPL = ( rpl0idx == -1 );
    }
    // RPL in picture header or SPS
    else if( sps->getNumRPL( listIdx ) == 0 )
    {
      readExplicitRPL = true;
    }
    else
    {
      READ_FLAG( uiCode, "ref_pic_list_sps_flag[ listidx ]" );
      readExplicitRPL = !uiCode;
    }

    // explicitly carried in this PH or SH
    if( readExplicitRPL )
    {
      header->clearRPL( listIdx );
      parseRefPicList( header->getRPL( listIdx ), -1, sps );
      header->setRPLIdx( listIdx, -1 );
    }
    // use list from SPS
    else
    {
      if( listIdx == REF_PIC_LIST_1 && !pps->getRpl1IdxPresentFlag() )
      {
        header->setRPL( listIdx, sps->getRPLList( listIdx )[header->getRPLIdx( listIdx )] );
      }
      else if( sps->getNumRPL( listIdx ) > 1 )
      {
        int numBits = (int)ceil( log2( sps->getNumRPL( listIdx ) ) );
        READ_CODE( numBits, uiCode, "ref_pic_list_idx[ listIdx ]" );
        header->setRPLIdx( listIdx, uiCode );
        header->setRPL( listIdx, sps->getRPLList( listIdx )[uiCode] );
      }
      else
      {
        header->setRPLIdx( listIdx, 0 );
        header->setRPL( listIdx, sps->getRPLList( listIdx )[0] );
      }
    }


    // Deal POC Msb cycle signalling for LTRP
    auto* rpl = header->getRPL( listIdx );
    for( int i = 0; i < rpl->getNumberOfLongtermPictures() +rpl->getNumberOfShorttermPictures(); i++ )
    {
      rpl->setDeltaPocMSBPresentFlag( i, false );
      rpl->setDeltaPocMSBCycleLT( i, 0 );
    }
    if( rpl->getNumberOfLongtermPictures() )
    {
      for( int i = 0; i < rpl->getNumberOfLongtermPictures() + rpl->getNumberOfShorttermPictures(); i++ )
      {
        if( rpl->isRefPicLongterm( i ) )
        {
          if( rpl->getLtrpInSliceHeaderFlag() )
          {
            READ_CODE( sps->getBitsForPOC(), uiCode, "poc_lsb_lt[i][j]" );
            rpl->setRefPicIdentifier( i, uiCode, true, false, 0 );
          }
          READ_FLAG( uiCode, "delta_poc_msb_present_flag[i][j]" );
          rpl->setDeltaPocMSBPresentFlag( i, uiCode ? true : false );
          if( uiCode )
          {
            READ_UVLC( uiCode, "delta_poc_msb_cycle_lt[i][j]" );
            rpl->setDeltaPocMSBCycleLT( i, uiCode );
          }
        }
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
    CHECK_RECOVERABLE(symbol>8, "gci_sixteen_minus_max_bitdepth_constraint_idc shall be in the range 0 to 8, inclusive");
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
      READ_FLAG( symbol, "gci_reserved_zero_bit" );                          CHECK_RECOVERABLE( symbol != 0, "gci_reserved_zero_bit not equal to zero" );
    }
  }
  while( !isByteAligned() )
  {
    READ_FLAG( symbol, "gci_alignment_zero_bit" );                           CHECK_RECOVERABLE( symbol != 0, "gci_alignment_zero_bit not equal to zero" );
  }
}


void HLSyntaxReader::parseProfileTierLevel( ProfileTierLevel *ptl, bool profileTierPresentFlag, int maxNumSubLayersMinus1 )
{
  uint32_t symbol;
  if( profileTierPresentFlag )
  {
    READ_CODE( 7, symbol, "general_profile_idc" );                           ptl->setProfileIdc( Profile::Name( symbol ) );
    READ_FLAG( symbol, "general_tier_flag" );                                ptl->setTierFlag( symbol ? Tier::HIGH : Tier::MAIN );
  }

  READ_CODE( 8, symbol, "general_level_idc" );                               ptl->setLevelIdc( vvdecLevel( symbol ) );

  READ_FLAG( symbol, "ptl_frame_only_constraint_flag" );                     ptl->setFrameOnlyConstraintFlag( symbol );
  READ_FLAG( symbol, "ptl_multilayer_enabled_flag" );                        ptl->setMultiLayerEnabledFlag( symbol );
  CHECK_RECOVERABLE( ( ptl->getProfileIdc() == Profile::MAIN_10 || ptl->getProfileIdc() == Profile::MAIN_10_444
        || ptl->getProfileIdc() == Profile::MAIN_10_STILL_PICTURE
        || ptl->getProfileIdc() == Profile::MAIN_10_444_STILL_PICTURE )
          && symbol,
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
    READ_FLAG( symbol, "sub_layer_level_present_flag[i]" );              ptl->setSubLayerLevelPresentFlag( i, symbol );
  }

  while( !isByteAligned() )
  {
    READ_FLAG( symbol, "ptl_reserved_zero_bit" );
    CHECK_RECOVERABLE( symbol != 0, "ptl_reserved_zero_bit not equal to zero" );
  }

  for( int i = maxNumSubLayersMinus1 - 1; i >= 0; i-- )
  {
    if( ptl->getSubLayerLevelPresentFlag( i ) )
    {
      READ_CODE( 8, symbol, "sub_layer_level_idc[i]" );                    ptl->setSubLayerLevelIdc( i, vvdecLevel( symbol ) );
    }
  }

  ptl->setSubLayerLevelIdc( maxNumSubLayersMinus1, ptl->getLevelIdc() );
  for( int i = maxNumSubLayersMinus1 - 1; i >= 0; i-- )
  {
    if( !ptl->getSubLayerLevelPresentFlag( i ) )
    {
      ptl->setSubLayerLevelIdc( i, ptl->getSubLayerLevelIdc( i + 1 ) );
    }
  }

  if( profileTierPresentFlag )
  {
    READ_CODE( 8, symbol, "ptl_num_sub_profiles" );
    uint8_t numSubProfiles = symbol;
    ptl->setNumSubProfile( numSubProfiles );
    for( int i = 0; i < numSubProfiles; i++ )
    {
      READ_CODE( 32, symbol, "general_sub_profile_idc[i]" );
      ptl->setSubProfileIdc( i, symbol );
    }
  }
}


// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

//! parse explicit wp tables
void HLSyntaxReader::parsePredWeightTable( Slice* pcSlice, const SPS *sps )
{
  WPScalingParam* wp;
  const ChromaFormat chFmt        = sps->getChromaFormatIdc();
  const int          numValidComp = int( getNumberValidComponents( chFmt ) );
  const bool         bChroma      = ( chFmt != CHROMA_400 );
  const SliceType    eSliceType   = pcSlice->getSliceType();
  const int          iNbRef       = ( eSliceType == B_SLICE ) ? 2 : 1;
  uint32_t           uiLog2WeightDenomLuma = 0, uiLog2WeightDenomChroma = 0;
  uint32_t           uiTotalSignalledWeightFlags = 0;

  int iDeltaDenom;
  // decode delta_luma_log2_weight_denom :
  READ_UVLC( uiLog2WeightDenomLuma, "luma_log2_weight_denom" );
  CHECK_RECOVERABLE( uiLog2WeightDenomLuma > 7, "Invalid code" );
  if( bChroma )
  {
    READ_SVLC( iDeltaDenom, "delta_chroma_log2_weight_denom" );
    CHECK_RECOVERABLE( ( iDeltaDenom + (int)uiLog2WeightDenomLuma ) < 0, "Invalid code" );
    CHECK_RECOVERABLE( ( iDeltaDenom + (int)uiLog2WeightDenomLuma ) > 7, "Invalid code" );
    uiLog2WeightDenomChroma = (uint32_t)( iDeltaDenom + uiLog2WeightDenomLuma );
  }

  for( int iNumRef=0 ; iNumRef < iNbRef ; iNumRef++ ) // loop over l0 and l1 syntax elements
  {
    RefPicList  eRefPicList = ( iNumRef ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
    for ( int iRefIdx=0 ; iRefIdx<pcSlice->getNumRefIdx( eRefPicList ) ; iRefIdx++ )
    {
      pcSlice->getWpScaling( eRefPicList, iRefIdx, wp );

      wp[COMPONENT_Y].uiLog2WeightDenom = uiLog2WeightDenomLuma;
      for( int j = 1; j < numValidComp; j++ )
      {
        wp[j].uiLog2WeightDenom = uiLog2WeightDenomChroma;
      }

      uint32_t  uiCode;
      READ_FLAG( uiCode, iNumRef == 0 ? "luma_weight_l0_flag[i]" : "luma_weight_l1_flag[i]" );
      wp[COMPONENT_Y].bPresentFlag = ( uiCode == 1 );
      uiTotalSignalledWeightFlags += wp[COMPONENT_Y].bPresentFlag;
    }
    if( bChroma )
    {
      uint32_t  uiCode;
      for( int iRefIdx=0 ; iRefIdx < pcSlice->getNumRefIdx( eRefPicList ) ; iRefIdx++ )
      {
        pcSlice->getWpScaling( eRefPicList, iRefIdx, wp );
        READ_FLAG( uiCode, iNumRef == 0 ? "chroma_weight_l0_flag[i]" : "chroma_weight_l1_flag[i]" );
        for( int j = 1; j < numValidComp; j++ )
        {
          wp[j].bPresentFlag = ( uiCode == 1 );
        }
        uiTotalSignalledWeightFlags += 2 * wp[COMPONENT_Cb].bPresentFlag;
      }
    }
    for( int iRefIdx=0 ; iRefIdx<pcSlice->getNumRefIdx( eRefPicList ) ; iRefIdx++ )
    {
      pcSlice->getWpScaling( eRefPicList, iRefIdx, wp );
      if ( wp[COMPONENT_Y].bPresentFlag )
      {
        int iDeltaWeight;
        READ_SVLC( iDeltaWeight, iNumRef == 0 ? "delta_luma_weight_l0[i]" : "delta_luma_weight_l1[i]" );
        CHECK_RECOVERABLE( iDeltaWeight < -128, "Invalid code" );
        CHECK_RECOVERABLE( iDeltaWeight >  127, "Invalid code" );
        wp[COMPONENT_Y].iWeight = ( iDeltaWeight + ( 1 << wp[COMPONENT_Y].uiLog2WeightDenom ) );
        READ_SVLC( wp[COMPONENT_Y].iOffset, iNumRef == 0? "luma_offset_l0[i]" : "luma_offset_l1[i]" );
        const int range = /* sps->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag() ? ( 1 << sps->getBitDepth(CHANNEL_TYPE_LUMA ) ) / 2 :*/ 128;
        if( wp[0].iOffset < -range ) { THROW( "Offset out of range" ); }
        if( wp[0].iOffset >= range ) { THROW( "Offset out of range" ); }
      }
      else
      {
        wp[COMPONENT_Y].iWeight = ( 1 << wp[COMPONENT_Y].uiLog2WeightDenom );
        wp[COMPONENT_Y].iOffset = 0;
      }
      if( bChroma )
      {
        if( wp[COMPONENT_Cb].bPresentFlag )
        {
          int range = /*sps->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag() ? ( 1 << sps->getBitDepth( CHANNEL_TYPE_CHROMA ) ) / 2 :*/ 128 ;
          for( int j = 1 ; j < numValidComp ; j++ )
          {
            int iDeltaWeight;
            READ_SVLC( iDeltaWeight, iNumRef == 0 ? "delta_chroma_weight_l0[i]" : "delta_chroma_weight_l1[i]" );
            CHECK_RECOVERABLE( iDeltaWeight < -128, "Invalid code" );
            CHECK_RECOVERABLE( iDeltaWeight >  127, "Invalid code" );
            wp[j].iWeight = (iDeltaWeight + (1<<wp[j].uiLog2WeightDenom));

            int iDeltaChroma;
            READ_SVLC( iDeltaChroma, iNumRef == 0 ? "delta_chroma_offset_l0[i]" : "delta_chroma_offset_l1[i]" );
            CHECK_RECOVERABLE( iDeltaChroma <  -4*range, "Invalid code" );
            CHECK_RECOVERABLE( iDeltaChroma >=  4*range, "Invalid code" );
            int pred = ( range - ( ( range * wp[j].iWeight ) >> ( wp[j].uiLog2WeightDenom ) ) );
            wp[j].iOffset = Clip3( -range, range - 1, iDeltaChroma + pred );
          }
        }
        else
        {
          for( int j = 1 ; j < numValidComp ; j++ )
          {
            wp[j].iWeight = ( 1 << wp[j].uiLog2WeightDenom );
            wp[j].iOffset = 0;
          }
        }
      }
    }

    for( int iRefIdx = pcSlice->getNumRefIdx( eRefPicList ); iRefIdx < MAX_NUM_REF; iRefIdx++ )
    {
      pcSlice->getWpScaling( eRefPicList, iRefIdx, wp );

      wp[0].bPresentFlag = false;
      wp[1].bPresentFlag = false;
      wp[2].bPresentFlag = false;
    }
  }
  CHECK_RECOVERABLE( uiTotalSignalledWeightFlags > 24, "Too many weight flag signalled" );
}

void HLSyntaxReader::parsePredWeightTable( PicHeader *picHeader, const SPS *sps )
{
  WPScalingParam *   wp;
  const ChromaFormat chFmt                     = sps->getChromaFormatIdc();
  const int          numValidComp              = int( getNumberValidComponents( chFmt ) );
  const bool         chroma                    = ( chFmt != CHROMA_400 );
  uint32_t           log2WeightDenomLuma       = 0;
  uint32_t           log2WeightDenomChroma     = 0;
  uint32_t           totalSignalledWeightFlags = 0;

  int deltaDenom;
  READ_UVLC( log2WeightDenomLuma, "luma_log2_weight_denom" );
  CHECK_RECOVERABLE( log2WeightDenomLuma > 7, "Invalid code" );
  if( chroma )
  {
    READ_SVLC( deltaDenom, "delta_chroma_log2_weight_denom" );
    CHECK_RECOVERABLE( ( deltaDenom + (int) log2WeightDenomLuma ) < 0, "Invalid code" );
    CHECK_RECOVERABLE( ( deltaDenom + (int) log2WeightDenomLuma ) > 7, "Invalid code" );
    log2WeightDenomChroma = (uint32_t)( deltaDenom + log2WeightDenomLuma );
  }

  uint32_t numLxWeights;
  READ_UVLC( numLxWeights, "num_l0_weights" );
  picHeader->setNumL0Weights( numLxWeights );
  picHeader->setNumL1Weights( 0 );

  bool moreSyntaxToBeParsed = true;
  for( int numRef = 0; numRef < NUM_REF_PIC_LIST_01 && moreSyntaxToBeParsed; numRef++ )
  {
    RefPicList refPicList = ( numRef ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
    for( int refIdx = 0; refIdx < numLxWeights; refIdx++ )
    {
      picHeader->getWpScaling( refPicList, refIdx, wp );

      wp[COMPONENT_Y].uiLog2WeightDenom = log2WeightDenomLuma;
      for (int j = 1; j < numValidComp; j++)
      {
        wp[j].uiLog2WeightDenom = log2WeightDenomChroma;
      }

      uint32_t uiCode;
      READ_FLAG(uiCode, numRef == 0 ? "luma_weight_l0_flag[i]" : "luma_weight_l1_flag[i]");
      wp[COMPONENT_Y].bPresentFlag = ( uiCode == 1 );
      totalSignalledWeightFlags += wp[COMPONENT_Y].bPresentFlag;
    }
    if( chroma )
    {
      uint32_t uiCode;
      for( int refIdx = 0; refIdx < numLxWeights; refIdx++ )
      {
        picHeader->getWpScaling( refPicList, refIdx, wp );
        READ_FLAG( uiCode, numRef == 0 ? "chroma_weight_l0_flag[i]" : "chroma_weight_l1_flag[i]" );
        for( int j = 1; j < numValidComp; j++ )
        {
          wp[j].bPresentFlag = ( uiCode == 1 );
        }
        totalSignalledWeightFlags += 2 * wp[COMPONENT_Cb].bPresentFlag;
      }
    }
    else
    {
      for( int refIdx = 0; refIdx < MAX_NUM_REF; refIdx++ )
      {
        picHeader->getWpScaling( refPicList, refIdx, wp );
        wp[1].bPresentFlag = false;
        wp[2].bPresentFlag = false;
      }
    }
    for( int refIdx = 0; refIdx < numLxWeights; refIdx++ )
    {
      picHeader->getWpScaling( refPicList, refIdx, wp );
      if( wp[COMPONENT_Y].bPresentFlag )
      {
        int deltaWeight;
        READ_SVLC( deltaWeight, numRef == 0 ? "delta_luma_weight_l0[i]" : "delta_luma_weight_l1[i]" );
        wp[COMPONENT_Y].iWeight = ( deltaWeight + ( 1 << wp[COMPONENT_Y].uiLog2WeightDenom ) );
        READ_SVLC( wp[COMPONENT_Y].iOffset, numRef == 0 ? "luma_offset_l0[i]" : "luma_offset_l1[i]" );
        const int range = /*sps->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag() ? ( 1 << sps->getBitDepth( CHANNEL_TYPE_LUMA ) ) / 2 :*/ 128;
        if( wp[0].iOffset < -range )
        {
          THROW( "Offset out of range" );
        }
        if( wp[0].iOffset >= range )
        {
          THROW( "Offset out of range" );
        }
      }
      else
      {
        wp[COMPONENT_Y].iWeight = ( 1 << wp[COMPONENT_Y].uiLog2WeightDenom );
        wp[COMPONENT_Y].iOffset = 0;
      }
      if( chroma )
      {
        if( wp[COMPONENT_Cb].bPresentFlag )
        {
          int range = /*sps->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag() ? ( 1 << sps->getBitDepth( CHANNEL_TYPE_CHROMA ) ) / 2 :*/ 128;
          for( int j = 1; j < numValidComp; j++ )
          {
            int deltaWeight;
            READ_SVLC( deltaWeight, numRef == 0 ? "delta_chroma_weight_l0[i]" : "delta_chroma_weight_l1[i]" );
            wp[j].iWeight = ( deltaWeight + (1 << wp[j].uiLog2WeightDenom ) );

            int deltaChroma;
            READ_SVLC( deltaChroma, numRef == 0 ? "delta_chroma_offset_l0[i]" : "delta_chroma_offset_l1[i]");
            int pred      = ( range - ( ( range * wp[j].iWeight ) >> ( wp[j].uiLog2WeightDenom ) ) );
            wp[j].iOffset = Clip3( -range, range - 1, ( deltaChroma + pred ) );
          }
        }
        else
        {
          for( int j = 1; j < numValidComp; j++ )
          {
            wp[j].iWeight = ( 1 << wp[j].uiLog2WeightDenom );
            wp[j].iOffset = 0;
          }
        }
      }
    }

    for( int refIdx = numLxWeights; refIdx < MAX_NUM_REF; refIdx++ )
    {
      picHeader->getWpScaling( refPicList, refIdx, wp );

      wp[0].bPresentFlag = false;
      wp[1].bPresentFlag = false;
      wp[2].bPresentFlag = false;
    }

    if( numRef == 0 )
    {
      if( picHeader->getRPL( REF_PIC_LIST_1 )->getNumRefEntries() > 0 )
      {
        READ_UVLC( numLxWeights, "num_l1_weights" );
      }
      else
      {
        numLxWeights = 0;
      }
      moreSyntaxToBeParsed = ( numLxWeights == 0 ) ? false : true;
      picHeader->setNumL1Weights( numLxWeights );
    }
  }
  CHECK_RECOVERABLE( totalSignalledWeightFlags > 24, "Too many weight flag signalled" );
}

/** decode quantization matrix
* \param scalingList quantization matrix information
*/
void HLSyntaxReader::parseScalingList( ScalingList *scalingList, bool aps_chromaPrsentFlag )
{
  uint32_t  code;
  bool scalingListCopyModeFlag;
  scalingList->setChromaScalingListPresentFlag( aps_chromaPrsentFlag );
  for( int scalingListId = 0; scalingListId < 28; scalingListId++ )
  {
    if( aps_chromaPrsentFlag || scalingList->isLumaScalingList( scalingListId ) )
    {
      READ_FLAG( code, "scaling_list_copy_mode_flag" );
      scalingListCopyModeFlag = ( code ) ? true : false;
      scalingList->setScalingListCopyModeFlag( scalingListId, scalingListCopyModeFlag );

      scalingList->setScalingListPreditorModeFlag( scalingListId, false );
      if( !scalingListCopyModeFlag )
      {
        READ_FLAG( code, "scaling_list_pred_mode_flag" );
        scalingList->setScalingListPreditorModeFlag( scalingListId, code );
      }

      if( ( scalingListCopyModeFlag || scalingList->getScalingListPreditorModeFlag( scalingListId ) ) && scalingListId !=  SCALING_LIST_1D_START_2x2 && scalingListId != SCALING_LIST_1D_START_4x4 && scalingListId != SCALING_LIST_1D_START_8x8 ) //Copy Mode
      {
        READ_UVLC( code, "scaling_list_pred_id_delta" );
        scalingList->setRefMatrixId( scalingListId, (uint32_t)( (int)( scalingListId ) - ( code ) ) );
      }
      else if( scalingListCopyModeFlag || scalingList->getScalingListPreditorModeFlag( scalingListId ) )
      {
        scalingList->setRefMatrixId( scalingListId, (uint32_t)( (int)( scalingListId ) ) );
      }
      if( scalingListCopyModeFlag )//copy
      {
        if( scalingListId >= SCALING_LIST_1D_START_16x16 )
        {
          scalingList->setScalingListDC( scalingListId,
                                         ( ( scalingListId == scalingList->getRefMatrixId( scalingListId ) ) ? 16
                                           : ( scalingList->getRefMatrixId( scalingListId ) < SCALING_LIST_1D_START_16x16 )
                                             ? scalingList->getScalingListAddress( scalingList->getRefMatrixId( scalingListId ) )[0]
                                             : scalingList->getScalingListDC( scalingList->getRefMatrixId( scalingListId ) ) ) );
        }
        scalingList->processRefMatrix( scalingListId, scalingList->getRefMatrixId( scalingListId ) );
      }
      else
      {
        decodeScalingList( scalingList, scalingListId, scalingList->getScalingListPreditorModeFlag( scalingListId ) );
      }
    }
    else
    {
      scalingListCopyModeFlag = true;
      scalingList->setScalingListCopyModeFlag( scalingListId, scalingListCopyModeFlag );
      scalingList->setRefMatrixId(scalingListId, (uint32_t)( (int)( scalingListId ) ) );
      if (scalingListId >= SCALING_LIST_1D_START_16x16)
      {
        scalingList->setScalingListDC( scalingListId, 16 );
      }
      scalingList->processRefMatrix( scalingListId, scalingList->getRefMatrixId( scalingListId ) );
    }
  }

  return;
}

/** decode DPCM
* \param scalingList  quantization matrix information
* \param sizeId size index
* \param listId list index
*/
void HLSyntaxReader::decodeScalingList( ScalingList *scalingList, uint32_t scalingListId, bool isPredictor )
{
  int matrixSize = ( scalingListId < SCALING_LIST_1D_START_4x4 ) ? 2 : ( scalingListId < SCALING_LIST_1D_START_8x8 ) ? 4 : 8;
  int i, coefNum = matrixSize * matrixSize;
  int data;
  int scalingListDcCoefMinus8 = 0;
  int nextCoef = ( isPredictor ) ? 0 : SCALING_LIST_START_VALUE;
  const uint16_t *scan = g_scanOrder[SCAN_UNGROUPED][g_sizeIdxInfo.idxFrom( matrixSize )][g_sizeIdxInfo.idxFrom( matrixSize )];
  int *dst = scalingList->getScalingListAddress( scalingListId );

  int PredListId = scalingList->getRefMatrixId( scalingListId );
  CHECK_RECOVERABLE( isPredictor && PredListId > scalingListId, "Scaling List error predictor!" );
  const int *srcPred = isPredictor ? ( ( scalingListId == PredListId ) ? scalingList->getScalingListDefaultAddress( scalingListId )
                                                                       : scalingList->getScalingListAddress( PredListId ) )
                                   : NULL;
  int predCoef = 0;

  if( scalingListId >= SCALING_LIST_1D_START_16x16 )
  {
    READ_SVLC( scalingListDcCoefMinus8, "scaling_list_dc_coef" );
    nextCoef += scalingListDcCoefMinus8;
    if( isPredictor )
    {
      predCoef = ( PredListId >= SCALING_LIST_1D_START_16x16 ) ? scalingList->getScalingListDC( PredListId ) : srcPred[0];
    }
    scalingList->setScalingListDC( scalingListId, ( nextCoef + predCoef + 256 ) % 256 );
  }

  for( i = 0; i < coefNum; i++ )
  {
    int blkPos = scan[i];
    int posX = blkPos & ( ( 1 << g_sizeIdxInfo.idxFrom( matrixSize ) ) - 1 );
    int posY = blkPos >> g_sizeIdxInfo.idxFrom( matrixSize );

    if( scalingListId >= SCALING_LIST_1D_START_64x64 && posX >= 4 && posY >= 4 )
    {
      dst[scan[i]] = 0;
      continue;
    }
    READ_SVLC( data, "scaling_list_delta_coef[i]" );
    nextCoef += data;
    predCoef = ( isPredictor ) ? srcPred[scan[i]] : 0;
    dst[scan[i]] = (nextCoef + predCoef + 256) % 256;
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
  CHECK_RECOVERABLE( cnt<0, "Negative number of bits") ;

  // we have more data, if cnt is not zero
  return ( cnt>0 );
}


void HLSyntaxReader::alfFilter( AlfSliceParam& alfSliceParam, const bool isChroma, const int altIdx )
{
  const bool isLuma = !isChroma;
  uint32_t code;

  // derive maxGolombIdx
  const int numCoeff = g_alfNumCoeff[isChroma ? 0 : 1];
//  READ_UVLC( code, "min_golomb_order" );
  const int numFilters = isChroma ? 1 : alfSliceParam.numLumaFilters;
  short* coeff = isChroma ? alfSliceParam.chromaCoeff + altIdx*MAX_NUM_ALF_CHROMA_COEFF : alfSliceParam.lumaCoeff;
  short* clipp = isChroma ? alfSliceParam.chromaClipp + altIdx*MAX_NUM_ALF_CHROMA_COEFF : alfSliceParam.lumaClipp;

  // Filter coefficients
  for( int ind = 0; ind < numFilters; ++ind )
  {
    for( int i = 0; i < numCoeff - 1; i++ )
    {
      READ_UVLC( code, isLuma ? "alf_luma_coeff_abs" : "alf_chroma_coeff_abs" );
      coeff[ ind * MAX_NUM_ALF_LUMA_COEFF + i ] = code;
      if( coeff[ ind * MAX_NUM_ALF_LUMA_COEFF + i ] != 0 )
      {
        READ_FLAG( code, isLuma ? "alf_luma_coeff_sign" : "alf_chroma_coeff_sign" );
        coeff[ ind * MAX_NUM_ALF_LUMA_COEFF + i ] = ( code ) ? -coeff[ ind * MAX_NUM_ALF_LUMA_COEFF + i ] : coeff[ ind * MAX_NUM_ALF_LUMA_COEFF + i ];
       }
    }

    if( isLuma )
    {
      alfSliceParam.lumaCoeffSummed = false;
    }

    const int factor = 1 << ( AdaptiveLoopFilter::m_NUM_BITS - 1 );
    coeff[ind * MAX_NUM_ALF_LUMA_COEFF + numCoeff - 1] = factor;
  }

  // Clipping values coding
  bool nonLinFlag = isChroma ? alfSliceParam.nonLinearFlagChroma : alfSliceParam.nonLinearFlagLuma;
  if( nonLinFlag )
  {
    // Filter coefficients
    for( int ind = 0; ind < numFilters; ++ind )
    {
      for( int i = 0; i < numCoeff - 1; i++ )
      {
        READ_CODE( 2, code, isLuma ? "alf_luma_clip_idx" : "alf_chroma_clip_idx" );
        clipp[ind * MAX_NUM_ALF_LUMA_COEFF + i] = code;
      }
    }
  }
  else
  {
    for( int ind = 0; ind < numFilters; ++ind )
    {
      std::fill_n( clipp + ind * MAX_NUM_ALF_LUMA_COEFF, numCoeff, 0 );
    }
  }
}

}
