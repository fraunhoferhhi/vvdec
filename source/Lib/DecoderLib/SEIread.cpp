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

Copyright (c) 2018-2021, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. 
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

/**
 \file     SEIread.cpp
 \brief    reading funtionality for SEI messages
 */

#include "CommonLib/CommonDef.h"
#include "CommonLib/BitStream.h"
#include "CommonLib/Slice.h"
#include "VLCReader.h"
#include "SEIread.h"
#include "CommonLib/Picture.h"
#include "CommonLib/dtrace_next.h"
#include <iomanip>

#include "vvdec/sei.h"


//! \ingroup DecoderLib
//! \{

#if ENABLE_TRACING
void  xTraceSEIHeader()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== SEI message ===========\n");
}

void  xTraceSEIMessageType(SEIPayloadType payloadType)
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== %s SEI message ===========\n", SEI_internal::getSEIMessageString( payloadType ) );
}
#endif

void SEIReader::sei_read_scode(std::ostream *pOS, uint32_t length, int& code, const char *pSymbolName)
{
  READ_SCODE(length, code, pSymbolName);
  if (pOS)
  {
    (*pOS) << "  " << std::setw(55) << pSymbolName << ": " << code << "\n";
  }
}

void SEIReader::sei_read_code(std::ostream *pOS, uint32_t uiLength, uint32_t& ruiCode, const char *pSymbolName)
{
  READ_CODE(uiLength, ruiCode, pSymbolName);
  if (pOS)
  {
    (*pOS) << "  " << std::setw(55) << pSymbolName << ": " << ruiCode << "\n";
  }
}

void SEIReader::sei_read_uvlc(std::ostream *pOS, uint32_t& ruiCode, const char *pSymbolName)
{
  READ_UVLC(ruiCode, pSymbolName);
  if (pOS)
  {
    (*pOS) << "  " << std::setw(55) << pSymbolName << ": " << ruiCode << "\n";
  }
}

void SEIReader::sei_read_svlc(std::ostream *pOS, int& ruiCode, const char *pSymbolName)
{
  READ_SVLC(ruiCode, pSymbolName);
  if (pOS)
  {
    (*pOS) << "  " << std::setw(55) << pSymbolName << ": " << ruiCode << "\n";
  }
}

void SEIReader::sei_read_flag(std::ostream *pOS, uint32_t& ruiCode, const char *pSymbolName)
{
  READ_FLAG(ruiCode, pSymbolName);
  if (pOS)
  {
    (*pOS) << "  " << std::setw(55) << pSymbolName << ": " << (ruiCode?1:0) << "\n";
  }
}

static inline void output_sei_message_header(vvdec_sei_message_t* sei, std::ostream *pDecodedMessageOutputStream, uint32_t payloadSize)
{
  if (pDecodedMessageOutputStream)
  {
    std::string seiMessageHdr(SEI_internal::getSEIMessageString(sei->payloadType)); seiMessageHdr+=" SEI message";
    (*pDecodedMessageOutputStream) << std::setfill('-') << std::setw(seiMessageHdr.size()) << "-" << std::setfill(' ') << "\n" << seiMessageHdr << " (" << payloadSize << " bytes)"<< "\n";
  }
}

#undef READ_CODE
#undef READ_SCODE
#undef READ_SVLC
#undef READ_UVLC
#undef READ_FLAG


/**
 * unmarshal a single SEI message from bitstream bs
 */
void SEIReader::parseSEImessage(InputBitstream* bs, seiMessages& seiList,
                                const NalUnitType nalUnitType, const uint32_t nuh_layer_id, const uint32_t temporalId,
                                const VPS *vps, const SPS *sps, HRD &hrd, std::ostream *pDecodedMessageOutputStream )
{
#if JVET_S0178_GENERAL_SEI_CHECK
  seiMessages seiListInCurNalu;
#endif
  setBitstream(bs);
  CHECK(m_pcBitstream->getNumBitsUntilByteAligned(), "Bitstream not aligned");

  do
  {
    xReadSEImessage( seiList, nalUnitType, nuh_layer_id, temporalId, vps, sps, hrd, pDecodedMessageOutputStream);
#if JVET_S0178_GENERAL_SEI_CHECK
    if( !seiList.empty() ){  seiListInCurNalu.push_back(seiList.back()); }
#endif
    /* SEI messages are an integer number of bytes, something has failed
    * in the parsing if bitstream not byte-aligned */
    CHECK(m_pcBitstream->getNumBitsUntilByteAligned(), "Bitstream not aligned");
  }
  while (m_pcBitstream->getNumBitsLeft() > 8);

#if JVET_S0178_GENERAL_SEI_CHECK
  seiMessages fillerData = SEI_internal::getSeisByType(seiListInCurNalu, VVDEC_FILLER_PAYLOAD);
  CHECK(fillerData.size() > 0 && fillerData.size() != seiListInCurNalu.size(), "When an SEI NAL unit contains an SEI message with payloadType equal to filler payload, the SEI NAL unit shall not contain any other SEI message with payloadType not equal to filler payload");
#endif

  xReadRbspTrailingBits();
}

void SEIReader::xReadSEImessage( seiMessages& seiList, const NalUnitType nalUnitType, const uint32_t nuh_layer_id,
                                const uint32_t temporalId, const VPS *vps, const SPS *sps, HRD &hrd, std::ostream *pDecodedMessageOutputStream)
{
#if ENABLE_TRACING
  xTraceSEIHeader();
#endif
  int payloadType = 0;
  uint32_t val = 0;

  do
  {
    sei_read_code(NULL, 8, val, "payload_type");
    payloadType += val;
  } while (val==0xFF);

  uint32_t payloadSize = 0;
  do
  {
    sei_read_code(NULL, 8, val, "payload_size");
    payloadSize += val;
  } while (val==0xFF);

#if ENABLE_TRACING
  xTraceSEIMessageType((SEIPayloadType)payloadType);
#endif

  /* extract the payload for this single SEI message.
   * This allows greater safety in erroneous parsing of an SEI message
   * from affecting subsequent messages.
   * After parsing the payload, bs needs to be restored as the primary
   * bitstream.
   */
  InputBitstream *bs = getBitstream();
  setBitstream(bs->extractSubstream(payloadSize * 8));

  const vvdec_sei_buffering_period_t *bp = NULL;
  const vvdec_sei_picture_timing_t   *pt = NULL;

  vvdec_sei_message_t *s =   SEI_internal::allocSEI( (SEIPayloadType)payloadType );

  if(nalUnitType == NAL_UNIT_PREFIX_SEI)
  {
    switch (payloadType)
    {
    case VVDEC_USER_DATA_REGISTERED_ITU_T_T35:
          if( 0 <= SEI_internal::allocSEIPayload( s ) ){ CHECK( s->payload == NULL, "allocation error in vvdec_sei_user_data_registered_t" );}
          xParseSEIUserDataRegistered( s, payloadSize, pDecodedMessageOutputStream);
          break;
    case VVDEC_USER_DATA_UNREGISTERED:
          if( 0 <= SEI_internal::allocSEIPayload( s ) ){ CHECK( s->payload == NULL, "allocation error in vvdec_sei_user_data_unregistered_t" ); }
          xParseSEIuserDataUnregistered(s, payloadSize, pDecodedMessageOutputStream);
      break;
    case VVDEC_DECODING_UNIT_INFO:
      bp = hrd.getBufferingPeriodSEI();
      if (!bp)
      {
        msg( WARNING, "Warning: Found Decoding unit information SEI message, but no active buffering period is available. Ignoring.");
      }
      else
      {
        if( 0 <= SEI_internal::allocSEIPayload( s ) ) { CHECK( s->payload == NULL, "allocation error in vvdec_sei_decoding_unit_info_t" ); }
        xParseSEIDecodingUnitInfo( s, payloadSize, *bp, temporalId, pDecodedMessageOutputStream);
      }
      break;
    case VVDEC_BUFFERING_PERIOD:
      if( 0 <= SEI_internal::allocSEIPayload( s ) ){ CHECK( s->payload == NULL, "allocation error in vvdec_sei_buffering_period_t" );}
      xParseSEIBufferingPeriod(s, payloadSize, pDecodedMessageOutputStream);
      if( s )
      {
        vvdec_sei_buffering_period_t* bufferingPeriod = reinterpret_cast<vvdec_sei_buffering_period_t *>(s->payload);
        hrd.setBufferingPeriodSEI(bufferingPeriod);
      }

      break;
    case VVDEC_PICTURE_TIMING:
      {
        bp = hrd.getBufferingPeriodSEI();
        if (!bp)
        {
          msg( WARNING, "Warning: Found Picture timing SEI message, but no active buffering period is available. Ignoring.");
        }
        else
        {
          if( 0 <= SEI_internal::allocSEIPayload( s ) ) { CHECK( s->payload == NULL, "allocation error in vvdec_sei_picture_timing_t" ); }
          xParseSEIPictureTiming(s, payloadSize, temporalId, *bp, pDecodedMessageOutputStream);
#if JVET_Q0818_PT_SEI
          if( s )
          {
            vvdec_sei_picture_timing_t* picTiming= reinterpret_cast<vvdec_sei_picture_timing_t *>(s->payload);
            hrd.setPictureTimingSEI( picTiming );
          }
#endif
        }
      }
      break;
    case VVDEC_SCALABLE_NESTING:
      if( 0 <= SEI_internal::allocSEIPayload( s ) ) { CHECK( s->payload == NULL, "allocation error in vvdec_sei_scalable_nesting_t" ); }
      xParseSEIScalableNesting(s, nalUnitType, nuh_layer_id, payloadSize, vps, sps, pDecodedMessageOutputStream);
      break;
    case VVDEC_FRAME_FIELD_INFO:
      if( 0 <= SEI_internal::allocSEIPayload( s ) ) { CHECK( s->payload == NULL, "allocation error in vvdec_sei_frame_field_info_t" ); }
      xParseSEIFrameFieldinfo( s, *pt, payloadSize, pDecodedMessageOutputStream);
      break;
    case VVDEC_DEPENDENT_RAP_INDICATION:
      if( 0 <= SEI_internal::allocSEIPayload( s ) ){ CHECK( s->payload == NULL, "allocation error in vvdec_sei_dependent_rap_indication_t" ); }
      xParseSEIDependentRAPIndication( s, payloadSize, pDecodedMessageOutputStream);
      break;
    case VVDEC_FRAME_PACKING:
      if( 0 <= SEI_internal::allocSEIPayload( s ) ) { CHECK( s->payload == NULL, "allocation error in vvdec_sei_frame_packing_t" ); }
      xParseSEIFramePacking( s, payloadSize, pDecodedMessageOutputStream);
      break;
    case VVDEC_PARAMETER_SETS_INCLUSION_INDICATION:
        if( 0 <= SEI_internal::allocSEIPayload( s ) ) { CHECK( s->payload == NULL, "allocation error in vvdec_sei_parameter_sets_inclusion_indication_t" ); }
        xParseSEIParameterSetsInclusionIndication( s, payloadSize, pDecodedMessageOutputStream);
      break;
    case VVDEC_MASTERING_DISPLAY_COLOUR_VOLUME:
        if( 0 <= SEI_internal::allocSEIPayload( s ) ){ CHECK( s->payload == NULL, "allocation error in vvdec_sei_mastering_display_colour_volume_t" ); }
        xParseSEIMasteringDisplayColourVolume( s, payloadSize, pDecodedMessageOutputStream);
      break;
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
    case VVDEC_ALTERNATIVE_TRANSFER_CHARACTERISTICS:
        if( 0 <= SEI_internal::allocSEIPayload( s ) ) { CHECK( s->payload == NULL, "allocation error in vvdec_sei_alternative_transfer_characteristics_t" );}
        xParseSEIAlternativeTransferCharacteristics( s, payloadSize, pDecodedMessageOutputStream);
      break;
#endif
    case VVDEC_EQUIRECTANGULAR_PROJECTION:
      if( 0 <= SEI_internal::allocSEIPayload( s ) ) { CHECK( s->payload == NULL, "allocation error in vvdec_sei_equirectangular_projection_t" ); }
      xParseSEIEquirectangularProjection( s, payloadSize, pDecodedMessageOutputStream);
      break;
    case VVDEC_SPHERE_ROTATION:
      if( 0 <= SEI_internal::allocSEIPayload( s ) ) { CHECK( s->payload == NULL, "allocation error in vvdec_sei_sphere_rotation_t" ); }
      xParseSEISphereRotation( s, payloadSize, pDecodedMessageOutputStream);
      break;
    case VVDEC_OMNI_VIEWPORT:
      if( 0 <= SEI_internal::allocSEIPayload( s ) ) { CHECK( s->payload == NULL, "allocation error in vvdec_sei_omni_viewport_t" ); }
      xParseSEIOmniViewport( s, payloadSize, pDecodedMessageOutputStream);
      break;
    case VVDEC_REGION_WISE_PACKING:
      if( 0 <= SEI_internal::allocSEIPayload( s ) ) { CHECK( s->payload == NULL, "allocation error in vvdec_sei_region_wise_packing_t" ); }
      xParseSEIRegionWisePacking( s, payloadSize, pDecodedMessageOutputStream);
      break;
    case VVDEC_GENERALIZED_CUBEMAP_PROJECTION:
      if( 0 <= SEI_internal::allocSEIPayload( s ) ) { CHECK( s->payload == NULL, "allocation error in vvdec_sei_generalized_cubemap_projection_t" );}
      xParseSEIGeneralizedCubemapProjection( s, payloadSize, pDecodedMessageOutputStream);
      break;
    case VVDEC_SUBPICTURE_LEVEL_INFO:
      if( 0 <= SEI_internal::allocSEIPayload( s ) ) { CHECK( s->payload == NULL, "allocation error in vvdec_sei_subpicture_level_info_t" );}
      xParseSEISubpictureLevelInfo(s, payloadSize, pDecodedMessageOutputStream);
      break;
    case VVDEC_SAMPLE_ASPECT_RATIO_INFO:
      if( 0 <= SEI_internal::allocSEIPayload( s ) ) { CHECK( s->payload == NULL, "allocation error in vvdec_sei_sample_aspect_ratio_info_t" ); }
      xParseSEISampleAspectRatioInfo( s, payloadSize, pDecodedMessageOutputStream);
      break;
    case VVDEC_FILM_GRAIN_CHARACTERISTICS:
      if( 0 <= SEI_internal::allocSEIPayload( s ) ) { CHECK( s->payload == NULL, "allocation error in vvdec_sei_film_grain_characteristics_t" ); }
      xParseSEIFilmGrainCharacteristics(s, payloadSize, pDecodedMessageOutputStream);
      break;
    case VVDEC_CONTENT_LIGHT_LEVEL_INFO:
      if( 0 <= SEI_internal::allocSEIPayload( s ) ) { CHECK( s->payload == NULL, "allocation error in vvdec_sei_content_light_level_info_t" ); }
      xParseSEIContentLightLevelInfo( s, payloadSize, pDecodedMessageOutputStream);
      break;
    case VVDEC_AMBIENT_VIEWING_ENVIRONMENT:
        if( 0 <= SEI_internal::allocSEIPayload( s ) ) { CHECK( s->payload == NULL, "allocation error in vvdec_sei_ambient_viewing_environment_t" ); }
        xParseSEIAmbientViewingEnvironment( s, payloadSize, pDecodedMessageOutputStream);
      break;
    case VVDEC_CONTENT_COLOUR_VOLUME:
      if( 0 <= SEI_internal::allocSEIPayload( s ) ) { CHECK( s->payload == NULL, "allocation error in vvdec_sei_content_colour_volume_t" ); }
      xParseSEIContentColourVolume( s, payloadSize, pDecodedMessageOutputStream);
      break;
    default:
      for (uint32_t i = 0; i < payloadSize; i++)
      {
        uint32_t seiByte;
        sei_read_code (NULL, 8, seiByte, "unknown prefix SEI payload byte");
      }
      msg( WARNING, "Unknown prefix SEI message (payloadType = %d) was found!\n", payloadType);
      if (pDecodedMessageOutputStream)
      {
        (*pDecodedMessageOutputStream) << "Unknown prefix SEI message (payloadType = " << payloadType << ") was found!\n";
      }
      break;
    }
  }
  else
  {
    switch (payloadType)
    {
      case VVDEC_USER_DATA_UNREGISTERED:
        if( 0 <= SEI_internal::allocSEIPayload( s ) ){ CHECK( s->payload == NULL, "allocation error in vvdec_sei_user_data_unregistered_t" ); }
        xParseSEIuserDataUnregistered(s, payloadSize, pDecodedMessageOutputStream);
        break;
      case  VVDEC_DECODED_PICTURE_HASH:
        if( 0 <= SEI_internal::allocSEIPayload( s ) ){ CHECK( s->payload == NULL, "allocation error in vvdec_sei_decoded_picture_hash_t" ); }
        xParseSEIDecodedPictureHash(s, payloadSize, pDecodedMessageOutputStream);
        break;
#if JVET_R0294_SUBPIC_HASH
      case VVDEC_SCALABLE_NESTING:
        if( 0 <= SEI_internal::allocSEIPayload( s ) ) { CHECK( s->payload == NULL, "allocation error in vvdec_sei_scalable_nesting_t" ); }
        xParseSEIScalableNesting(s, nalUnitType, nuh_layer_id, payloadSize, vps, sps, pDecodedMessageOutputStream);
        break;
#endif
      default:
        for (uint32_t i = 0; i < payloadSize; i++)
        {
          uint32_t seiByte;
          sei_read_code( NULL, 8, seiByte, "unknown suffix SEI payload byte");
        }
        msg( WARNING, "Unknown suffix SEI message (payloadType = %d) was found!\n", payloadType);
        if (pDecodedMessageOutputStream)
        {
          (*pDecodedMessageOutputStream) << "Unknown suffix SEI message (payloadType = " << payloadType << ") was found!\n";
        }
        break;
    }
  }

  if( s != NULL && s->size )
  {
    seiList.push_back(s);
  }

  /* By definition the underlying bitstream terminates in a byte-aligned manner.
   * 1. Extract all bar the last MIN(bitsremaining,nine) bits as reserved_payload_extension_data
   * 2. Examine the final 8 bits to determine the payload_bit_equal_to_one marker
   * 3. Extract the remainingreserved_payload_extension_data bits.
   *
   * If there are fewer than 9 bits available, extract them.
   */
  int payloadBitsRemaining = getBitstream()->getNumBitsLeft();
  if (payloadBitsRemaining) /* more_data_in_payload() */
  {
    for (; payloadBitsRemaining > 9; payloadBitsRemaining--)
    {
      uint32_t reservedPayloadExtensionData;
      sei_read_code ( pDecodedMessageOutputStream, 1, reservedPayloadExtensionData, "reserved_payload_extension_data");
    }

    /* 2 */
    int finalBits = getBitstream()->peekBits(payloadBitsRemaining);
    int finalPayloadBits = 0;
    for (int mask = 0xff; finalBits & (mask >> finalPayloadBits); finalPayloadBits++)
    {
      continue;
    }

    /* 3 */
    for (; payloadBitsRemaining > 9 - finalPayloadBits; payloadBitsRemaining--)
    {
      uint32_t reservedPayloadExtensionData;
      sei_read_flag ( 0, reservedPayloadExtensionData, "reserved_payload_extension_data");
    }

    uint32_t dummy;
    sei_read_flag( 0, dummy, "payload_bit_equal_to_one"); payloadBitsRemaining--;
    while (payloadBitsRemaining)
    {
      sei_read_flag( 0, dummy, "payload_bit_equal_to_zero"); payloadBitsRemaining--;
    }
  }

  /* restore primary bitstream for sei_message */
  delete getBitstream();
  setBitstream(bs);
}



/**
 * parse bitstream bs and unpack a decoded picture hash SEI message
 * of payloadSize bytes into sei.
 */
void SEIReader::xParseSEIDecodedPictureHash(vvdec_sei_message_t* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t bytesRead = 0;
  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);

  CHECK ( !s->size, "vvdec_sei_decoded_picture_hash_t no payload" );
  vvdec_sei_decoded_picture_hash_t* sei =(vvdec_sei_decoded_picture_hash_t*)s->payload;
  ::memset(sei, 0, sizeof(vvdec_sei_decoded_picture_hash_t));

  uint32_t val;
  sei_read_code( pDecodedMessageOutputStream, 8, val, "dph_sei_hash_type" );
  sei->method = static_cast<HashType>(val); bytesRead++;
  sei_read_code( pDecodedMessageOutputStream, 1, val, "dph_sei_single_component_flag");
  sei->singleCompFlag = val;
  sei_read_code( pDecodedMessageOutputStream, 7, val, "dph_sei_reserved_zero_7bits");
  bytesRead++;
  uint32_t expectedSize = ( sei->singleCompFlag ? 1 : 3 ) * (sei->method == 0 ? 16 : (sei->method == 1 ? 2 : 4));
  CHECK ((payloadSize - bytesRead) != expectedSize, "The size of the decoded picture hash does not match the expected size.");

  const char *traceString="\0";
  switch (sei->method)
  {
    case VVDEC_HASHTYPE_MD5: traceString="picture_md5"; break;
    case VVDEC_HASHTYPE_CRC: traceString="picture_crc"; break;
    case VVDEC_HASHTYPE_CHECKSUM: traceString="picture_checksum"; break;
    default: THROW("Unknown hash type"); break;
  }

  if (pDecodedMessageOutputStream)
  {
    (*pDecodedMessageOutputStream) << "  " << std::setw(55) << traceString << ": " << std::hex << std::setfill('0');
  }

  for(int i=0;bytesRead < payloadSize; bytesRead++,i++)
  {
    sei_read_code( NULL, 8, val, traceString);
    sei->digest[i] = (uint8_t)val;
    sei->digist_length++;

    if (pDecodedMessageOutputStream)
    {
      (*pDecodedMessageOutputStream) << std::setw(2) << val;
    }
  }

  if (pDecodedMessageOutputStream)
  {
    (*pDecodedMessageOutputStream) << std::dec << std::setfill(' ') << "\n";
  }
}

void SEIReader::xParseSEIScalableNesting(vvdec_sei_message_t* s, const NalUnitType nalUnitType, const uint32_t nuhLayerId, uint32_t payloadSize, const VPS *vps, const SPS *sps, std::ostream *decodedMessageOutputStream)
{
  uint32_t symbol;
  output_sei_message_header(s, decodedMessageOutputStream, payloadSize);

  CHECK ( !s->size, "vvdec_sei_scalable_nesting_t no payload" );
  vvdec_sei_scalable_nesting_t* sei =(vvdec_sei_scalable_nesting_t*)s->payload;
  ::memset(sei, 0, sizeof(vvdec_sei_scalable_nesting_t));

  sei_read_flag(decodedMessageOutputStream, symbol, "sn_ols_flag"); sei->m_snOlsFlag = symbol;
  sei_read_flag(decodedMessageOutputStream, symbol, "sn_subpic_flag"); sei->m_snSubpicFlag = symbol;
  if (sei->m_snOlsFlag)
  {
    sei_read_uvlc(decodedMessageOutputStream, symbol, "sn_num_olss_minus1"); sei->m_snNumOlss = symbol+1;
    for (uint32_t i = 0; i < sei->m_snNumOlss; i++)
    {
      sei_read_uvlc(decodedMessageOutputStream, symbol, "sn_ols_idx_delta_minus1[i]"); sei->m_snOlsIdxDelta[i] = symbol+1;
    }
    for (uint32_t i = 0; i < sei->m_snNumOlss; i++)
    {
      if (i == 0)
      {
        sei->m_snOlsIdx[i] = sei->m_snOlsIdxDelta[i]-1;
      }
      else
      {
        sei->m_snOlsIdx[i] = sei->m_snOlsIdxDelta[i]-1 + sei->m_snOlsIdxDelta[i - 1];
      }
    }
    if (vps && vps->getVPSId() != 0)
    {
      uint32_t lowestLayerId = MAX_UINT;
      for (uint32_t olsIdxForSEI = 0; olsIdxForSEI < sei->m_snNumOlss; olsIdxForSEI++)
      {
        int olsIdx = sei->m_snOlsIdx[olsIdxForSEI];
        for (int layerIdx = 0; layerIdx < vps->getNumLayersInOls(olsIdx); layerIdx++)
        {
          if (lowestLayerId > vps->getLayerIdInOls(olsIdx, layerIdx))
          {
            lowestLayerId = vps->getLayerIdInOls(olsIdx, layerIdx);
          }
        }
      }
      CHECK(lowestLayerId!= nuhLayerId, "nuh_layer_id is not equal to the lowest layer among Olss that the scalable SEI applies");
    }
  }
  else
  {
    sei_read_flag(decodedMessageOutputStream, symbol, "sn_all_layers_flag"); sei->m_snAllLayersFlag = symbol;
    if (!sei->m_snAllLayersFlag)
    {
      sei_read_uvlc(decodedMessageOutputStream, symbol, "sn_num_layers_minus1"); sei->m_snNumLayers = symbol+1;
      for (uint32_t i = 0; i < sei->m_snNumLayers; i++)
      {
        sei_read_code(decodedMessageOutputStream, 6, symbol, "sn_layer_id[i]"); sei->m_snLayerId[i] = symbol;
      }
    }
  }
  if (sei->m_snSubpicFlag)
  {
    sei_read_uvlc(decodedMessageOutputStream, symbol, "sn_num_subpics_minus1"); sei->m_snNumSubpics = symbol + 1;
    sei_read_uvlc(decodedMessageOutputStream, symbol, "sn_subpic_id_len_minus1"); sei->m_snSubpicIdLen = symbol + 1;
    for (uint32_t i = 0; i < sei->m_snNumSubpics; i++)
    {
      sei_read_code(decodedMessageOutputStream, sei->m_snSubpicIdLen, symbol, "sn_subpic_id[i]"); sei->m_snSubpicId[i] = symbol;
    }
  }

  sei_read_uvlc(decodedMessageOutputStream, symbol, "sn_num_seis_minus1"); sei->m_snNumSEIs = symbol + 1;
  CHECK (sei->m_snNumSEIs > 64, "The value of sn_num_seis_minus1 shall be in the range of 0 to 63");

  // byte alignment
  while (m_pcBitstream->getNumBitsRead() % 8 != 0)
  {
    sei_read_flag(decodedMessageOutputStream, symbol, "sn_zero_bit");
  }

  // read nested SEI messages
  for (uint32_t i=0; i<sei->m_snNumSEIs; i++)
  {
    seiMessages tmpSeiList;
    xReadSEImessage(tmpSeiList, nalUnitType, nuhLayerId, 0, vps, sps, m_nestedHrd, decodedMessageOutputStream);

    if (tmpSeiList.front()->payloadType == VVDEC_BUFFERING_PERIOD)
    {
      vvdec_sei_buffering_period_t *bp = (vvdec_sei_buffering_period_t*) tmpSeiList.front();
      m_nestedHrd.setBufferingPeriodSEI(bp);
    }
    sei->m_nestedSEIs[i] = tmpSeiList.front();
    tmpSeiList.clear();
  }

  xCheckScalableNestingConstraints(sei, nalUnitType, vps);

  if (decodedMessageOutputStream)
  {
    (*decodedMessageOutputStream) << "End of scalable nesting SEI message\n";
  }
}

void SEIReader::xCheckScalableNestingConstraints(const vvdec_sei_scalable_nesting_t* sei, const NalUnitType nalUnitType, const VPS* vps)
{
#if JVET_S0177_SCALABLE_NESTING_SEI
  const std::vector<int> vclAssociatedSeiList { 3, 19, 45, 129, 137, 144, 145, 147, 148, 149, 150, 153, 154, 155, 156, 168, 204 };

  bool containBPorPTorDUIorSLI = false;
  bool containNoBPorPTorDUIorSLI = false;

  for (auto nestedsei : sei->m_nestedSEIs)
  {
    CHECK(nestedsei->payloadType == VVDEC_FILLER_PAYLOAD || nestedsei->payloadType == VVDEC_SCALABLE_NESTING, "An SEI message that has payloadType equal to filler payload or scalable nesting shall not be contained in a scalable nesting SEI message");

    CHECK(nestedsei->payloadType != VVDEC_FILLER_PAYLOAD && nestedsei->payloadType != VVDEC_DECODED_PICTURE_HASH && nalUnitType != NAL_UNIT_PREFIX_SEI, "When a scalable nesting SEI message contains an SEI message that has payloadType not equal to filler payload or decoded picture hash, the SEI NAL unit containing the scalable nesting SEI message shall have nal_unit_type equal to PREFIX_SEI_NUT");

    CHECK(nestedsei->payloadType == VVDEC_DECODED_PICTURE_HASH && nalUnitType != NAL_UNIT_SUFFIX_SEI, "When a scalable nesting SEI message contains an SEI message that has payloadType equal to decoded picture hash, the SEI NAL unit containing the scalable nesting SEI message shall have nal_unit_type equal to SUFFIX_SEI_NUT");

    CHECK(nestedsei->payloadType == VVDEC_DECODED_PICTURE_HASH && !sei->m_snSubpicFlag, "When the scalable nesting SEI message contains an SEI message that has payloadType equal to decoded picture hash, the value of sn_subpic_flag shall be equal to 1");

    CHECK(nestedsei->payloadType == VVDEC_SUBPICTURE_LEVEL_INFO && sei->m_snSubpicFlag, "When the scalable nesting SEI message contains an SEI message that has payloadType equal to SLI, the value of sn_subpic_flag shall be equal to 0");

#if JVET_S0178_GENERAL_SEI_CHECK
    CHECK(vps->getGeneralHrdParameters()->getGeneralSamePicTimingInAllOlsFlag() && nestedsei->payloadType == VVDEC_PICTURE_TIMING, "When general_same_pic_timing_in_all_ols_flag is equal to 1, there shall be no SEI NAL unit that contain a scalable-nested SEI message with payloadType equal to PT");
#endif

    for (int i = 0; i < (int)vclAssociatedSeiList.size(); i++)
    {
      CHECK(nestedsei->payloadType == vclAssociatedSeiList[i] && sei->m_snOlsFlag, "When the scalable nesting SEI message contains an SEI message that has payloadType equal to a value in vclAssociatedSeiList, the value of sn_ols_flag shall be equal to 0");
    }

    if (nestedsei->payloadType == VVDEC_BUFFERING_PERIOD || nestedsei->payloadType == VVDEC_PICTURE_TIMING || nestedsei->payloadType == VVDEC_DECODING_UNIT_INFO || nestedsei->payloadType == VVDEC_SUBPICTURE_LEVEL_INFO)
    {
      containBPorPTorDUIorSLI = true;
      CHECK(!sei->m_snOlsFlag, "When the scalable nesting SEI message contains an SEI message that has payloadType equal to BP, PT, or DUI, or SLI, the value of sn_ols_flag shall be equal to 1");
    }
    if (!(nestedsei->payloadType == VVDEC_BUFFERING_PERIOD || nestedsei->payloadType == VVDEC_PICTURE_TIMING || nestedsei->payloadType == VVDEC_DECODING_UNIT_INFO || nestedsei->payloadType == VVDEC_SUBPICTURE_LEVEL_INFO))
    {
      containNoBPorPTorDUIorSLI = true;
    }
  }
  CHECK(containBPorPTorDUIorSLI && containNoBPorPTorDUIorSLI, "When a scalable nesting SEI message contains a BP, PT, DUI, or SLI SEI message, the scalable nesting SEI message shall not contain any other SEI message with payloadType not equal to BP, PT, DUI, or SLI");
#else
  bool containBPorPTorDUI   = false;
  bool containNoBPorPTorDUI = false;
  for (auto nestedsei : sei->m_nestedSEIs)
  {
#if JVET_S0178_GENERAL_SEI_CHECK
    CHECK(vps->getGeneralHrdParameters()->getGeneralSamePicTimingInAllOlsFlag() && nestedsei->payloadType() == PICTURE_TIMING, "When general_same_pic_timing_in_all_ols_flag is equal to 1, there shall be no SEI NAL unit that contain a scalable-nested SEI message with payloadType equal to PT");
#endif
    if (nestedsei->payloadType() == VVDEC_BUFFERING_PERIOD || nestedsei->payloadType() == VVDEC_PICTURE_TIMING || nestedsei->payloadType() == VVDEC_DECODING_UNIT_INFO)
    {
      containBPorPTorDUI = true;
  }
    if (!(nestedsei->payloadType() == VVDEC_BUFFERING_PERIOD || nestedsei->payloadType() == VVDEC_PICTURE_TIMING || nestedsei->payloadType() == VVDEC_DECODING_UNIT_INFO))
    {
      containNoBPorPTorDUI = true;
    }
  }
  CHECK(containBPorPTorDUI && containNoBPorPTorDUI, "Scalable Nesting SEI cannot contain timing-related SEI and none-timing-related SEIs at the same time");
#endif
}

void SEIReader::xParseSEIDecodingUnitInfo(vvdec_sei_message_t* s, uint32_t payloadSize, const vvdec_sei_buffering_period_t& bp, const uint32_t temporalId, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;

  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);

  CHECK ( !s->size, "vvdec_sei_decoding_unit_info_t no payload" );
  vvdec_sei_decoding_unit_info_t* sei =(vvdec_sei_decoding_unit_info_t*)s->payload;
  ::memset(sei, 0, sizeof(vvdec_sei_decoding_unit_info_t));

  sei_read_uvlc( pDecodedMessageOutputStream, val, "decoding_unit_idx");
  sei->m_decodingUnitIdx = val;

  if(!bp.m_decodingUnitCpbParamsInPicTimingSeiFlag)
  {
    for (int i = temporalId; i <= bp.m_bpMaxSubLayers - 1; i++)
    {
      if (i < (bp.m_bpMaxSubLayers - 1))
      {
        sei_read_flag( pDecodedMessageOutputStream, val, "dui_sub_layer_delays_present_flag[i]" );
        sei->m_duiSubLayerDelaysPresentFlag[i] = val;
      }
      else
      {
        sei->m_duiSubLayerDelaysPresentFlag[i] = 1;
      }
      if( sei->m_duiSubLayerDelaysPresentFlag[i] )
      {
        sei_read_code( pDecodedMessageOutputStream, bp.m_duCpbRemovalDelayIncrementLength, val, "du_spt_cpb_removal_delay_increment[i]");
        sei->m_duSptCpbRemovalDelayIncrement[i] = val;
      }
      else
      {
        sei->m_duSptCpbRemovalDelayIncrement[i] = 0;
      }
    }
  }
  else
  {
    for( int i = temporalId; i < bp.m_bpMaxSubLayers - 1; i ++ )
    {
      sei->m_duSptCpbRemovalDelayIncrement[i] = 0;
    }
  }
  if (bp.m_decodingUnitDpbDuParamsInPicTimingSeiFlag)
  {
    sei_read_flag( pDecodedMessageOutputStream, val, "dpb_output_du_delay_present_flag"); sei->m_dpbOutputDuDelayPresentFlag = (val != 0);
  }
  else
  {
    sei->m_dpbOutputDuDelayPresentFlag = false;
  }
  if(sei->m_dpbOutputDuDelayPresentFlag)
  {
    sei_read_code( pDecodedMessageOutputStream, bp.m_dpbOutputDelayDuLength, val, "pic_spt_dpb_output_du_delay");
    sei->m_picSptDpbOutputDuDelay = val;
  }
}

void SEIReader::xParseSEIBufferingPeriod(vvdec_sei_message_t* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  int i, nalOrVcl;
  uint32_t code;

  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);

  CHECK ( !s->size, "vvdec_sei_buffering_period_t no payload" );
  vvdec_sei_buffering_period_t* sei =(vvdec_sei_buffering_period_t*)s->payload;
  ::memset(sei, 0, sizeof(vvdec_sei_buffering_period_t));

  sei_read_flag( pDecodedMessageOutputStream, code, "bp_nal_hrd_parameters_present_flag" );               sei->m_bpNalCpbParamsPresentFlag = code;
  sei_read_flag( pDecodedMessageOutputStream, code, "bp_vcl_hrd_parameters_present_flag" );               sei->m_bpVclCpbParamsPresentFlag = code;

  sei_read_code( pDecodedMessageOutputStream, 5, code, "initial_cpb_removal_delay_length_minus1" );     sei->m_initialCpbRemovalDelayLength = code + 1;
  sei_read_code( pDecodedMessageOutputStream, 5, code, "cpb_removal_delay_length_minus1" );             sei->m_cpbRemovalDelayLength        = code + 1;
  sei_read_code( pDecodedMessageOutputStream, 5, code, "dpb_output_delay_length_minus1" );              sei->m_dpbOutputDelayLength         = code + 1;
#if !JVET_S0185_PROPOSAL2_SEI_CLEANUP
  sei_read_flag( pDecodedMessageOutputStream, code, "alt_cpb_params_present_flag");                     sei->m_altCpbParamsPresentFlag      = code;
#endif
  sei_read_flag( pDecodedMessageOutputStream, code, "bp_decoding_unit_hrd_params_present_flag" );       sei->m_bpDecodingUnitHrdParamsPresentFlag = code;
  if( sei->m_bpDecodingUnitHrdParamsPresentFlag )
  {
    sei_read_code( pDecodedMessageOutputStream, 5, code, "du_cpb_removal_delay_increment_length_minus1" );  sei->m_duCpbRemovalDelayIncrementLength = code + 1;
    sei_read_code( pDecodedMessageOutputStream, 5, code, "dpb_output_delay_du_length_minus1" );             sei->m_dpbOutputDelayDuLength = code + 1;
    sei_read_flag( pDecodedMessageOutputStream, code, "decoding_unit_cpb_params_in_pic_timing_sei_flag" );  sei->m_decodingUnitCpbParamsInPicTimingSeiFlag = code;
    sei_read_flag(pDecodedMessageOutputStream, code, "decoding_unit_dpb_du_params_in_pic_timing_sei_flag");  sei->m_decodingUnitDpbDuParamsInPicTimingSeiFlag = code;
  }
  else
  {
    sei->m_duCpbRemovalDelayIncrementLength = 24;
    sei->m_dpbOutputDelayDuLength = 24;
    sei->m_decodingUnitDpbDuParamsInPicTimingSeiFlag = false;
  }

#if JVET_S0248_HRD_CLEANUP
  CHECK(sei->m_altCpbParamsPresentFlag && sei->m_bpDecodingUnitHrdParamsPresentFlag,"When bp_alt_cpb_params_present_flag is equal to 1, the value of bp_du_hrd_params_present_flag shall be equal to 0");
#endif

  sei_read_flag( pDecodedMessageOutputStream, code, "concatenation_flag");
  sei->m_concatenationFlag = code;
  sei_read_flag ( pDecodedMessageOutputStream, code, "additional_concatenation_info_present_flag");
  sei->m_additionalConcatenationInfoPresentFlag = code;
  if (sei->m_additionalConcatenationInfoPresentFlag)
  {
    sei_read_code( pDecodedMessageOutputStream, sei->m_initialCpbRemovalDelayLength, code, "max_initial_removal_delay_for_concatenation" );
    sei->m_maxInitialRemovalDelayForConcatenation = code;
  }

  sei_read_code( pDecodedMessageOutputStream, ( sei->m_cpbRemovalDelayLength ), code, "au_cpb_removal_delay_delta_minus1" );
  sei->m_auCpbRemovalDelayDelta = code + 1;
#if JVET_S0181_PROPOSAL2_BUFFERING_PERIOD_CLEANUP
  sei_read_code(pDecodedMessageOutputStream, 3, code, "bp_max_sub_layers_minus1");
  sei->m_bpMaxSubLayers = code + 1;
  if (sei->m_bpMaxSubLayers - 1 > 0)
  {
    sei_read_flag(pDecodedMessageOutputStream, code, "cpb_removal_delay_deltas_present_flag");
    sei->m_cpbRemovalDelayDeltasPresentFlag = code;
  }
  else
  {
    sei->m_cpbRemovalDelayDeltasPresentFlag = false;
  }
#else
  sei_read_flag( pDecodedMessageOutputStream, code, "cpb_removal_delay_deltas_present_flag" );               sei->m_cpbRemovalDelayDeltasPresentFlag = code;
#endif
  if (sei->m_cpbRemovalDelayDeltasPresentFlag)
  {
    sei_read_uvlc( pDecodedMessageOutputStream, code, "num_cpb_removal_delay_deltas_minus1" );               sei->m_numCpbRemovalDelayDeltas = code + 1;
    for( i = 0; i < sei->m_numCpbRemovalDelayDeltas; i ++ )
    {
      sei_read_code( pDecodedMessageOutputStream, ( sei->m_cpbRemovalDelayLength ), code, "cpb_removal_delay_delta[i]" );
      sei->m_cpbRemovalDelayDelta[ i ] = code;
    }
  }
#if !JVET_S0181_PROPOSAL2_BUFFERING_PERIOD_CLEANUP
  sei_read_code( pDecodedMessageOutputStream, 3, code, "bp_max_sub_layers_minus1" );     sei->m_bpMaxSubLayers = code + 1;
#endif
  sei_read_uvlc( pDecodedMessageOutputStream, code, "bp_cpb_cnt_minus1" ); sei->m_bpCpbCnt = code + 1;
#if JVET_S0181_PROPOSAL1
  if (sei->m_bpMaxSubLayers - 1 > 0)
  {
    sei_read_flag(pDecodedMessageOutputStream, code, "bp_sublayer_initial_cpb_removal_delay_present_flag");
    sei->m_sublayerInitialCpbRemovalDelayPresentFlag = code;
  }
  else
  {
    sei->m_sublayerInitialCpbRemovalDelayPresentFlag = false;
  }
#else
  sei_read_flag(pDecodedMessageOutputStream, code, "sublayer_initial_cpb_removal_delay_present_flag");
  sei->m_sublayerInitialCpbRemovalDelayPresentFlag = code;
#endif
  for (i = (sei->m_sublayerInitialCpbRemovalDelayPresentFlag ? 0 : sei->m_bpMaxSubLayers - 1); i < sei->m_bpMaxSubLayers; i++)
  {
    for( nalOrVcl = 0; nalOrVcl < 2; nalOrVcl ++ )
    {
      if( ( ( nalOrVcl == 0 ) && ( sei->m_bpNalCpbParamsPresentFlag ) ) ||
         ( ( nalOrVcl == 1 ) && ( sei->m_bpVclCpbParamsPresentFlag ) ) )
      {
        for( int j = 0; j < ( sei->m_bpCpbCnt ); j ++ )
        {
          sei_read_code( pDecodedMessageOutputStream, sei->m_initialCpbRemovalDelayLength, code, nalOrVcl ? "vcl_initial_cpb_removal_delay[i][j]" : "nal_initial_cpb_removal_delay[i][j]" );
          sei->m_initialCpbRemovalDelay[i][j][nalOrVcl] = code;
          sei_read_code( pDecodedMessageOutputStream, sei->m_initialCpbRemovalDelayLength, code, nalOrVcl ? "vcl_initial_cpb_removal_offset[i][j]" : "nal_initial_cpb_removal_offset[i][j]" );
          sei->m_initialCpbRemovalDelay[i][j][nalOrVcl] = code;
        }
      }
    }
  }
#if JVET_S0064_SEI_BUFFERING_PERIOD_CLEANUP
  if (sei->m_bpMaxSubLayers-1 > 0)
  {
    sei_read_flag(pDecodedMessageOutputStream, code, "bp_sublayer_dpb_output_offsets_present_flag");
    sei->m_sublayerDpbOutputOffsetsPresentFlag = code;
  }
  else
  {
    sei->m_sublayerDpbOutputOffsetsPresentFlag = false;
  }
#else
  sei_read_flag( pDecodedMessageOutputStream, code, "sublayer_dpb_output_offsets_present_flag" );
  sei->m_sublayerDpbOutputOffsetsPresentFlag = code;
#endif
  if(sei->m_sublayerDpbOutputOffsetsPresentFlag)
  {
    for(int i = 0; i < sei->m_bpMaxSubLayers - 1; i++)
    {
      sei_read_uvlc( pDecodedMessageOutputStream, code, "dpb_output_tid_offset[i]" );
      sei->m_dpbOutputTidOffset[i] = code;
    }
    sei->m_dpbOutputTidOffset[sei->m_bpMaxSubLayers-1] = 0;
  }
#if JVET_S0185_PROPOSAL2_SEI_CLEANUP
  sei_read_flag(pDecodedMessageOutputStream, code, "bp_alt_cpb_params_present_flag");
  sei->m_altCpbParamsPresentFlag = code;
#endif
  if (sei->m_altCpbParamsPresentFlag)
  {
    sei_read_flag(pDecodedMessageOutputStream, code, "use_alt_cpb_params_flag"); sei->m_useAltCpbParamsFlag = code;
  }

}

void SEIReader::xParseSEIPictureTiming(vvdec_sei_message_t* s, uint32_t payloadSize, const uint32_t temporalId, const vvdec_sei_buffering_period_t& bp, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);

  CHECK ( !s->size, "vvdec_sei_picture_timing_t no payload" );
  vvdec_sei_picture_timing_t* sei =(vvdec_sei_picture_timing_t*)s->payload;
  ::memset(sei, 0, sizeof(vvdec_sei_picture_timing_t));

  uint32_t symbol;
#if JVET_S0185_PROPOSAl1_PICTURE_TIMING_CLEANUP
  sei_read_code( pDecodedMessageOutputStream, bp.m_cpbRemovalDelayLength, symbol, "pt_cpb_removal_delay_minus1[bp_max_sub_layers_minus1]" );
  sei->m_auCpbRemovalDelay[bp.m_bpMaxSubLayers - 1] = symbol + 1;
  for (int i = temporalId; i < bp.m_bpMaxSubLayers - 1; i++)
  {
    sei_read_flag(pDecodedMessageOutputStream, symbol, "pt_sub_layer_delays_present_flag[i]");
    sei->m_ptSubLayerDelaysPresentFlag[i] = (symbol == 1);
    if (sei->m_ptSubLayerDelaysPresentFlag[i])
    {
      if (bp.m_cpbRemovalDelayDeltasPresentFlag)
      {
        sei_read_flag(pDecodedMessageOutputStream, symbol, "pt_cpb_removal_delay_delta_enabled_flag[i]");
        sei->m_cpbRemovalDelayDeltaEnabledFlag[i] = (symbol == 1);
      }
      else
      {
        sei->m_cpbRemovalDelayDeltaEnabledFlag[i] = false;
      }
      if (sei->m_cpbRemovalDelayDeltaEnabledFlag[i])
      {
        if ((bp.m_numCpbRemovalDelayDeltas - 1) > 0)
        {
          sei_read_code(pDecodedMessageOutputStream, (int)ceil(log2(bp.m_numCpbRemovalDelayDeltas)), symbol, "pt_cpb_removal_delay_delta_idx[i]");
          sei->m_cpbRemovalDelayDeltaIdx[i] = symbol;
        }
        else
        {
          sei->m_cpbRemovalDelayDeltaIdx[i] = 0;
        }
      }
      else
      {
        sei_read_code(pDecodedMessageOutputStream, bp.m_cpbRemovalDelayLength, symbol, "pt_cpb_removal_delay_minus1[i]");
        sei->m_auCpbRemovalDelay[i] = symbol + 1;
      }
    }
  }
  sei_read_code(pDecodedMessageOutputStream, bp.m_dpbOutputDelayLength, symbol, "pt_dpb_output_delay");
  sei->m_picDpbOutputDelay = symbol;
#else
  sei_read_code( pDecodedMessageOutputStream, bp.m_cpbRemovalDelayLength, symbol, "cpb_removal_delay_minus1[bp_max_sub_layers_minus1]" );
  sei->m_auCpbRemovalDelay[bp.m_bpMaxSubLayers - 1] = symbol + 1;
#endif

  if( bp.m_altCpbParamsPresentFlag )
  {
    sei_read_flag( pDecodedMessageOutputStream, symbol, "cpb_alt_timing_info_present_flag" ); sei->m_cpbAltTimingInfoPresentFlag = symbol;
    if( sei->m_cpbAltTimingInfoPresentFlag )
    {
      if (bp.m_bpNalCpbParamsPresentFlag)
      {
        for (int i = (bp.m_sublayerInitialCpbRemovalDelayPresentFlag ? 0 : bp.m_bpMaxSubLayers - 1);
             i <= bp.m_bpMaxSubLayers - 1; ++i)
        {
          for (int j = 0; j < bp.m_bpCpbCnt; j++)
          {
            sei_read_code(pDecodedMessageOutputStream, bp.m_initialCpbRemovalDelayLength, symbol,
                          "nal_cpb_alt_initial_cpb_removal_delay_delta[ i ][ j ]");
            sei->m_nalCpbAltInitialRemovalDelayDelta[i][j] = symbol;
            sei_read_code(pDecodedMessageOutputStream, bp.m_initialCpbRemovalDelayLength, symbol,
                          "nal_cpb_alt_initial_cpb_removal_offset_delta[ i ][ j ]");
            sei->m_nalCpbAltInitialRemovalOffsetDelta[i][j] = symbol;
          }
          sei_read_code(pDecodedMessageOutputStream, bp.m_initialCpbRemovalDelayLength, sei->m_nalCpbDelayOffset[i],
                        "nal_cpb_delay_offset[ i ]");
          sei_read_code(pDecodedMessageOutputStream, bp.m_initialCpbRemovalDelayLength, sei->m_nalDpbDelayOffset[i],
                        "nal_dpb_delay_offset[ i ]");
        }
      }

      if (bp.m_bpVclCpbParamsPresentFlag)
      {
        for (int i = (bp.m_sublayerInitialCpbRemovalDelayPresentFlag ? 0 : bp.m_bpMaxSubLayers - 1);
             i <= bp.m_bpMaxSubLayers - 1; ++i)
        {
          for (int j = 0; j < bp.m_bpCpbCnt; j++)
          {
            sei_read_code(pDecodedMessageOutputStream, bp.m_initialCpbRemovalDelayLength, symbol,
                          "vcl_cpb_alt_initial_cpb_removal_delay_delta[ i ][ j ]");
            sei->m_vclCpbAltInitialRemovalDelayDelta[i][j] = symbol;
            sei_read_code(pDecodedMessageOutputStream, bp.m_initialCpbRemovalDelayLength, symbol,
                          "vcl_cpb_alt_initial_cpb_removal_offset_delta[ i ][ j ]");
            sei->m_vclCpbAltInitialRemovalOffsetDelta[i][j] = symbol;
          }
          sei_read_code(pDecodedMessageOutputStream, bp.m_initialCpbRemovalDelayLength, sei->m_vclCpbDelayOffset[i],
                        "vcl_cpb_delay_offset[ i ]");
          sei_read_code(pDecodedMessageOutputStream, bp.m_initialCpbRemovalDelayLength, sei->m_vclDpbDelayOffset[i],
                        "vcl_dpb_delay_offset[ i ]");
        }
      }
    }
  }
  else
  {
    sei->m_cpbAltTimingInfoPresentFlag = false;
  }

#if !JVET_S0185_PROPOSAl1_PICTURE_TIMING_CLEANUP
  for( int i = temporalId; i < bp.m_bpMaxSubLayers - 1; i ++ )
  {
    sei_read_flag( pDecodedMessageOutputStream,    symbol, "pt_sub_layer_delays_present_flag[i]" );    sei->m_ptSubLayerDelaysPresentFlag[i] = (symbol == 1);
    if( sei->m_ptSubLayerDelaysPresentFlag[ i ] )
    {
      if( bp.m_cpbRemovalDelayDeltasPresentFlag )
      {
        sei_read_flag( pDecodedMessageOutputStream,    symbol, "cpb_removal_delay_delta_enabled_flag[i]" );         
        sei->m_cpbRemovalDelayDeltaEnabledFlag[i]        = (symbol == 1);
      }
      else
      {
        sei->m_cpbRemovalDelayDeltaEnabledFlag[i] = false;
      }
      if( sei->m_cpbRemovalDelayDeltaEnabledFlag[ i ] )
      {
        if( ( bp.m_numCpbRemovalDelayDeltas - 1) > 0 )
        {
          sei_read_code( pDecodedMessageOutputStream, (int)ceil(log2(bp.m_numCpbRemovalDelayDeltas)), symbol, "cpb_removal_delay_delta_idx[i]" );
          sei->m_cpbRemovalDelayDeltaIdx[ i ] = symbol;
        }
        else
        {
          sei->m_cpbRemovalDelayDeltaIdx[i] = 0;
        }
      }
      else
      {
        sei_read_code( pDecodedMessageOutputStream, bp.m_cpbRemovalDelayLength, symbol, "cpb_removal_delay_minus1[i]" );
        sei->m_auCpbRemovalDelay[ i ] = symbol + 1;
      }
    }
  }
  sei_read_code( pDecodedMessageOutputStream, bp.m_dpbOutputDelayLength,  symbol, "dpb_output_delay" );
  sei->m_picDpbOutputDelay = symbol;
#endif
  if ( bp.m_bpDecodingUnitHrdParamsPresentFlag && bp.m_decodingUnitDpbDuParamsInPicTimingSeiFlag )
  {
    sei_read_code( pDecodedMessageOutputStream, bp.m_dpbOutputDelayDuLength, symbol, "pic_dpb_output_du_delay" );
    sei->m_picDpbOutputDuDelay = symbol;
  }
  if( bp.m_bpDecodingUnitHrdParamsPresentFlag && bp.m_decodingUnitCpbParamsInPicTimingSeiFlag )
  {
    sei_read_uvlc( pDecodedMessageOutputStream, symbol, "num_decoding_units_minus1" );
    sei->m_numDecodingUnits = symbol+1;

    if (sei->m_numDecodingUnits > 1)
    {
    sei_read_flag( pDecodedMessageOutputStream, symbol, "du_common_cpb_removal_delay_flag" );
    sei->m_duCommonCpbRemovalDelayFlag = symbol;
    if( sei->m_duCommonCpbRemovalDelayFlag )
    {
      for( int i = temporalId; i < bp.m_bpMaxSubLayers - 1; i ++ )
      {
        if( sei->m_ptSubLayerDelaysPresentFlag[i] )
        {
          sei_read_code( pDecodedMessageOutputStream, bp.m_duCpbRemovalDelayIncrementLength, symbol, "du_common_cpb_removal_delay_increment_minus1[i]" );
          sei->m_duCommonCpbRemovalDelay[i] = symbol+1;
        }
      }
    }
    for( uint32_t i = 0; i < sei->m_numDecodingUnits; i ++ )
    {
      sei_read_uvlc( pDecodedMessageOutputStream, symbol, "num_nalus_in_du_minus1[i]" );
      sei->m_numNalusInDu[i] = symbol+1;
      if( !sei->m_duCommonCpbRemovalDelayFlag && i < (sei->m_numDecodingUnits-1) )
      {
        for( int j = temporalId; j < bp.m_bpMaxSubLayers - 1; j ++ )
        {
          if( sei->m_ptSubLayerDelaysPresentFlag[j] )
          {
            sei_read_code( pDecodedMessageOutputStream, bp.m_duCpbRemovalDelayIncrementLength, symbol, "du_cpb_removal_delay_increment_minus1[i][j]" );
            sei->m_duCpbRemovalDelay[i * bp.m_bpMaxSubLayers + j] = symbol+1;
          }
        }
      }
    }
    }
    else
    {
      sei->m_duCommonCpbRemovalDelayFlag = 0;
    }
  }
#if JVET_Q0818_PT_SEI
  sei_read_code( pDecodedMessageOutputStream, 8, symbol, "pt_display_elemental_periods_minus1" );
  sei->m_ptDisplayElementalPeriods = symbol+1;
#endif
}

void SEIReader::xParseSEIFrameFieldinfo(vvdec_sei_message_t* s, const vvdec_sei_picture_timing_t& pt, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t symbol;
  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);

  CHECK ( !s->size, "vvdec_sei_frame_field_info_t no payload" );
  vvdec_sei_frame_field_info_t* sei =(vvdec_sei_frame_field_info_t*)s->payload;
  ::memset(sei, 0, sizeof(vvdec_sei_frame_field_info_t));

  sei_read_flag( pDecodedMessageOutputStream, symbol,      "field_pic_flag" );
  sei->m_fieldPicFlag= symbol;
  if (sei->m_fieldPicFlag)
  {
    sei_read_flag( pDecodedMessageOutputStream, symbol,    "bottom_field_flag" );
    sei->m_bottomFieldFlag = symbol;
    sei_read_flag( pDecodedMessageOutputStream, symbol,    "pairing_indicated_flag" );
    sei->m_pairingIndicatedFlag = symbol;
    if (sei->m_pairingIndicatedFlag)
    {
      sei_read_flag( pDecodedMessageOutputStream, symbol,  "paired_with_next_field_flag" );
      sei->m_pairedWithNextFieldFlag = symbol;
    }
  }
  else
  {
    sei_read_flag( pDecodedMessageOutputStream, symbol,    "display_fields_from_frame_flag" );
    sei->m_displayFieldsFromFrameFlag = symbol;
    if (sei->m_displayFieldsFromFrameFlag)
    {
      sei_read_flag( pDecodedMessageOutputStream, symbol,  "display_fields_from_frame_flag" );
      sei->m_topFieldFirstFlag = symbol;
    }
    sei_read_code( pDecodedMessageOutputStream, 8, symbol, "ffi_display_elemental_periods_minus1" );
    sei->m_displayElementalPeriods = symbol+1;
    if( pt.m_ptDisplayElementalPeriods != sei->m_displayElementalPeriods )
      msg( WARNING, "Warning: display_elemental_periods_minus1 is different in picture timing and frame field information SEI messages!");
  }
  sei_read_code( pDecodedMessageOutputStream, 2, symbol,   "source_scan_type" );
  sei->m_sourceScanType = symbol;
  sei_read_flag( pDecodedMessageOutputStream, symbol,      "duplicate_flag" );
  sei->m_duplicateFlag = symbol;
}

void SEIReader::xParseSEIDependentRAPIndication( vvdec_sei_message_t* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream )
{
  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);
}

void SEIReader::xParseSEIFramePacking(vvdec_sei_message_t* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;

  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);

  CHECK ( !s->size, "vvdec_sei_frame_packing_t no payload" );
  vvdec_sei_frame_packing_t* sei =(vvdec_sei_frame_packing_t*)s->payload;
  ::memset(sei, 0, sizeof(vvdec_sei_frame_packing_t));

  sei_read_uvlc( pDecodedMessageOutputStream, val, "frame_packing_arrangement_id" );                 sei->m_arrangementId = val;
  sei_read_flag( pDecodedMessageOutputStream, val, "frame_packing_arrangement_cancel_flag" );        sei->m_arrangementCancelFlag = val;

  if( !sei->m_arrangementCancelFlag )
  {
    sei_read_code( pDecodedMessageOutputStream, 7, val, "frame_packing_arrangement_type" );          sei->m_arrangementType = val;
    CHECK( ( sei->m_arrangementType <= 2 ) || ( sei->m_arrangementType >= 6 ), "Invalid arrangement type" );

    sei_read_flag( pDecodedMessageOutputStream, val, "quincunx_sampling_flag" );                     sei->m_quincunxSamplingFlag = val;

    sei_read_code( pDecodedMessageOutputStream, 6, val, "content_interpretation_type" );             sei->m_contentInterpretationType = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "spatial_flipping_flag" );                      sei->m_spatialFlippingFlag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "frame0_flipped_flag" );                        sei->m_frame0FlippedFlag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "field_views_flag" );                           sei->m_fieldViewsFlag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "current_frame_is_frame0_flag" );               sei->m_currentFrameIsFrame0Flag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "frame0_self_contained_flag" );                 sei->m_frame0SelfContainedFlag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "frame1_self_contained_flag" );                 sei->m_frame1SelfContainedFlag = val;

    if ( sei->m_quincunxSamplingFlag == 0 && sei->m_arrangementType != 5)
    {
      sei_read_code( pDecodedMessageOutputStream, 4, val, "frame0_grid_position_x" );                sei->m_frame0GridPositionX = val;
      sei_read_code( pDecodedMessageOutputStream, 4, val, "frame0_grid_position_y" );                sei->m_frame0GridPositionY = val;
      sei_read_code( pDecodedMessageOutputStream, 4, val, "frame1_grid_position_x" );                sei->m_frame1GridPositionX = val;
      sei_read_code( pDecodedMessageOutputStream, 4, val, "frame1_grid_position_y" );                sei->m_frame1GridPositionY = val;
    }

    sei_read_code( pDecodedMessageOutputStream, 8, val, "frame_packing_arrangement_reserved_byte" ); sei->m_arrangementReservedByte = val;
    sei_read_flag( pDecodedMessageOutputStream, val,  "frame_packing_arrangement_persistence_flag" );sei->m_arrangementPersistenceFlag = (val != 0);
  }
  sei_read_flag( pDecodedMessageOutputStream, val, "upsampled_aspect_ratio_flag" );                  sei->m_upsampledAspectRatio = val;
}

void SEIReader::xParseSEIParameterSetsInclusionIndication(vvdec_sei_message_t* s, uint32_t payloadSize, std::ostream* pDecodedMessageOutputStream)
{
  uint32_t val;

  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);

  CHECK ( !s->size, "vvdec_sei_parameter_sets_inclusion_indication_t no payload" );
  vvdec_sei_parameter_sets_inclusion_indication_t* sei =(vvdec_sei_parameter_sets_inclusion_indication_t*)s->payload;

  sei_read_flag( pDecodedMessageOutputStream, val, "self_contained_clvs_flag" ); sei->m_selfContainedClvsFlag = val;
}

void SEIReader::xParseSEIMasteringDisplayColourVolume( vvdec_sei_message_t* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t code;

  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);
  
  CHECK ( !s->size, "vvdec_sei_mastering_display_colour_volume_t no payload" );
  vvdec_sei_mastering_display_colour_volume_t* sei =(vvdec_sei_mastering_display_colour_volume_t*)s->payload;
  ::memset(s, 0, sizeof(vvdec_sei_mastering_display_colour_volume_t));

  sei_read_code( pDecodedMessageOutputStream, 16, code, "display_primaries_x[0]" ); sei->primaries[0][0] = code;
  sei_read_code( pDecodedMessageOutputStream, 16, code, "display_primaries_y[0]" ); sei->primaries[0][1] = code;

  sei_read_code( pDecodedMessageOutputStream, 16, code, "display_primaries_x[1]" ); sei->primaries[1][0] = code;
  sei_read_code( pDecodedMessageOutputStream, 16, code, "display_primaries_y[1]" ); sei->primaries[1][1] = code;

  sei_read_code( pDecodedMessageOutputStream, 16, code, "display_primaries_x[2]" ); sei->primaries[2][0] = code;
  sei_read_code( pDecodedMessageOutputStream, 16, code, "display_primaries_y[2]" ); sei->primaries[2][1] = code;

  sei_read_code( pDecodedMessageOutputStream, 16, code, "white_point_x" ); sei->whitePoint[0] = code;
  sei_read_code( pDecodedMessageOutputStream, 16, code, "white_point_y" ); sei->whitePoint[1] = code;

  sei_read_code( pDecodedMessageOutputStream, 32, code, "max_display_mastering_luminance" ); sei->maxLuminance = code;
  sei_read_code( pDecodedMessageOutputStream, 32, code, "min_display_mastering_luminance" ); sei->minLuminance = code;
}

#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
void SEIReader::xParseSEIAlternativeTransferCharacteristics(vvdec_sei_message_t* s, uint32_t payloadSize, std::ostream* pDecodedMessageOutputStream)
{
  uint32_t code;
  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);
  
  CHECK ( !s->size, "vvdec_sei_alternative_transfer_characteristics_t no payload" );
  vvdec_sei_alternative_transfer_characteristics_t* sei =(vvdec_sei_alternative_transfer_characteristics_t*)s->payload;

  sei_read_code(pDecodedMessageOutputStream, 8, code, "preferred_transfer_characteristics"); sei->preferred_transfer_characteristics = code;
}
#endif
void SEIReader::xParseSEIUserDataRegistered(vvdec_sei_message_t* sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t code;

  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  vvdec_sei_user_data_registered_t t;

  CHECK(payloadSize == 0, "wrong" );
  sei_read_code(pDecodedMessageOutputStream, 8, code, "itu_t_t35_country_code"); payloadSize--;
  if (code == 255)
  {
    CHECK(payloadSize == 0, "wrong" );
    sei_read_code(pDecodedMessageOutputStream, 8, code, "itu_t_t35_country_code_extension_byte"); payloadSize--;
    code += 255;
  }
  t.ituCountryCode = code;
  t.userDataLength = payloadSize;

  if( !payloadSize )
  {
    t.userData = 0;
    ::memcpy(sei->payload, &t, sizeof(vvdec_sei_user_data_registered_t));
    return;
  }

  if( sei->payload )
      free( sei->payload );

  SEI_internal::allocSEIPayload( sei, sizeof( vvdec_sei_user_data_registered_t ) + sizeof( uint8_t )*(t.userDataLength + 1) );
  CHECK( sei->payload == NULL, "allocation error in vvdec_sei_user_data_registered_t" );

  vvdec_sei_user_data_registered_t* target = (vvdec_sei_user_data_registered_t*)sei->payload;
  target->ituCountryCode = t.ituCountryCode;
  target->userDataLength = t.userDataLength;
  target->userData = (uint8_t*)((uint8_t*)sei->payload + sizeof( vvdec_sei_user_data_registered_t ));

  for (uint32_t i = 0; i < target->userDataLength; i++)
  {
    sei_read_code(NULL, 8, code, "itu_t_t35_payload_byte");
    target->userData[i] = code;
  }

  if (pDecodedMessageOutputStream)
  {
    (*pDecodedMessageOutputStream) << "  itu_t_t35 payload size: " << target->userDataLength << "\n";
  }
}

/**
 * parse bitstream bs and unpack a user_data_unregistered SEI message
 * of payloasSize bytes into sei.
 */
void SEIReader::xParseSEIuserDataUnregistered(vvdec_sei_message_t* sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  CHECK(payloadSize < 16, "Payload too small");
  uint32_t val;

  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  vvdec_sei_user_data_unregistered_t t;

  for (uint32_t i = 0; i < 16; i++)
  {
    sei_read_code( pDecodedMessageOutputStream, 8, val, "uuid_iso_iec_11578"); t.uuid_iso_iec_11578[i] = val;
  }

  t.userDataLength = payloadSize - 16;
  if (!t.userDataLength)
  {
    t.userData = 0;
    ::memcpy(sei->payload, &t, sizeof(vvdec_sei_user_data_unregistered_t));
    return;
  }

  if( sei->payload )
      free( sei->payload );

  SEI_internal::allocSEIPayload( sei,  sizeof( vvdec_sei_user_data_unregistered_t ) + sizeof( uint8_t )*(t.userDataLength + 1) );
  CHECK( sei->payload == NULL, "allocation error in vvdec_sei_user_data_unregistered_t" );

  vvdec_sei_user_data_unregistered_t* target = (vvdec_sei_user_data_unregistered_t*)sei->payload;
  target->userDataLength = t.userDataLength;
  target->userData = (uint8_t*)((uint8_t*)sei->payload + sizeof( vvdec_sei_user_data_unregistered_t ));

  for (uint32_t i = 0; i < target->userDataLength; i++)
  {
    sei_read_code(NULL, 8, val, "user_data_payload_byte");
    target->userData[i] = val;
  }

  if (pDecodedMessageOutputStream)
  {
    (*pDecodedMessageOutputStream) << "  User data payload size: " << target->userDataLength << "\n";
  }
}

void SEIReader::xParseSEIFilmGrainCharacteristics(vvdec_sei_message_t* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t code;

  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);

  CHECK ( !s->size, "vvdec_sei_film_grain_characteristics_t no payload" );
  vvdec_sei_film_grain_characteristics_t* sei =(vvdec_sei_film_grain_characteristics_t*)s->payload;
  ::memset(sei, 0, sizeof(vvdec_sei_film_grain_characteristics_t));

  sei_read_flag(pDecodedMessageOutputStream, code, "film_grain_characteristics_cancel_flag");     sei->m_filmGrainCharacteristicsCancelFlag = code != 0;
  if (!sei->m_filmGrainCharacteristicsCancelFlag)
  {
    sei_read_code(pDecodedMessageOutputStream, 2, code, "film_grain_model_id");                   sei->m_filmGrainModelId = code;
    sei_read_flag(pDecodedMessageOutputStream, code, "separate_colour_description_present_flag"); sei->m_separateColourDescriptionPresentFlag = code != 0;
    if (sei->m_separateColourDescriptionPresentFlag)
    {
      sei_read_code(pDecodedMessageOutputStream, 3, code, "film_grain_bit_depth_luma_minus8");    sei->m_filmGrainBitDepthLuma = code+8;
      sei_read_code(pDecodedMessageOutputStream, 3, code, "film_grain_bit_depth_chroma_minus8");  sei->m_filmGrainBitDepthChroma = code+8;
      sei_read_flag(pDecodedMessageOutputStream, code, "film_grain_full_range_flag");             sei->m_filmGrainFullRangeFlag = code != 0;
      sei_read_code(pDecodedMessageOutputStream, 8, code, "film_grain_colour_primaries");         sei->m_filmGrainColourPrimaries = code;
      sei_read_code(pDecodedMessageOutputStream, 8, code, "film_grain_transfer_characteristics"); sei->m_filmGrainTransferCharacteristics = code;
      sei_read_code(pDecodedMessageOutputStream, 8, code, "film_grain_matrix_coeffs");            sei->m_filmGrainMatrixCoeffs = code;
    }
    sei_read_code(pDecodedMessageOutputStream, 2, code, "blending_mode_id");                      sei->m_blendingModeId = code;
    sei_read_code(pDecodedMessageOutputStream, 4, code, "log2_scale_factor");                     sei->m_log2ScaleFactor = code;
    for (int c = 0; c<3; c++)
    {
      sei_read_flag(pDecodedMessageOutputStream, code, "comp_model_present_flag[c]");             sei->m_compModel[c].presentFlag = code != 0;
    }
    for (int c = 0; c<3; c++)
    {
      vvdec_CompModel_t &cm = sei->m_compModel[c];
      if (cm.presentFlag)
      {
        uint32_t numIntensityIntervals;
        sei_read_code(pDecodedMessageOutputStream, 8, code, "num_intensity_intervals_minus1[c]"); numIntensityIntervals = code + 1;
        sei_read_code(pDecodedMessageOutputStream, 3, code, "num_model_values_minus1[c]");        cm.numModelValues = code + 1;

        CHECK ( numIntensityIntervals > 255, "num_intensity_intervals_minus1[c] out of range" );
        CHECK ( cm.numModelValues > 5, "num_model_values_minus1[c] out of range" );

        for (uint32_t interval = 0; interval < numIntensityIntervals; interval++)
        {
          vvdec_CompModelIntensityValues_t &cmiv = cm.intensityValues[interval];
          sei_read_code(pDecodedMessageOutputStream, 8, code, "intensity_interval_lower_bound[c][i]"); cmiv.intensityIntervalLowerBound = code;
          sei_read_code(pDecodedMessageOutputStream, 8, code, "intensity_interval_upper_bound[c][i]"); cmiv.intensityIntervalUpperBound = code;
          for (uint32_t j = 0; j<cm.numModelValues; j++)
          {
            sei_read_svlc(pDecodedMessageOutputStream, cmiv.compModelValue[j], "comp_model_value[c][i]");
          }
        }
      }
    } // for c
    sei_read_flag(pDecodedMessageOutputStream, code, "film_grain_characteristics_persistence_flag"); sei->m_filmGrainCharacteristicsPersistenceFlag = code != 0;
  } // cancel flag
}

void SEIReader::xParseSEIContentLightLevelInfo(vvdec_sei_message_t* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t code;

  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);

  CHECK ( !s->size, "vvdec_sei_content_light_level_info_t no payload" );
  vvdec_sei_content_light_level_info_t* sei =(vvdec_sei_content_light_level_info_t*)s->payload;
  ::memset(sei, 0, sizeof(vvdec_sei_content_light_level_info_t));

  sei_read_code(pDecodedMessageOutputStream, 16, code, "max_content_light_level");     sei->m_maxContentLightLevel    = code;
  sei_read_code(pDecodedMessageOutputStream, 16, code, "max_pic_average_light_level"); sei->m_maxPicAverageLightLevel = code;
}

void SEIReader::xParseSEIAmbientViewingEnvironment(vvdec_sei_message_t* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t code;

  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);

  CHECK ( !s->size, "vvdec_sei_ambient_viewing_environment_t no payload" );
  vvdec_sei_ambient_viewing_environment_t* sei =(vvdec_sei_ambient_viewing_environment_t*)s->payload;
  ::memset(sei, 0, sizeof(vvdec_sei_ambient_viewing_environment_t));

  sei_read_code(pDecodedMessageOutputStream, 32, code, "ambient_illuminance"); sei->m_ambientIlluminance = code;
  sei_read_code(pDecodedMessageOutputStream, 16, code, "ambient_light_x");     sei->m_ambientLightX = (uint16_t)code;
  sei_read_code(pDecodedMessageOutputStream, 16, code, "ambient_light_y");     sei->m_ambientLightY = (uint16_t)code;
}

void SEIReader::xParseSEIContentColourVolume(vvdec_sei_message_t* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
  {
  int i;
  uint32_t val;

  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);

  CHECK ( !s->size, "vvdec_sei_content_colour_volume_t no payload" );
  vvdec_sei_content_colour_volume_t* sei =(vvdec_sei_content_colour_volume_t*)s->payload;
  ::memset(sei, 0, sizeof(vvdec_sei_content_colour_volume_t));

  sei_read_flag(pDecodedMessageOutputStream, val, "ccv_cancel_flag");          sei->m_ccvCancelFlag = val;
  if (!sei->m_ccvCancelFlag)
    {
    int iVal;
    sei_read_flag(pDecodedMessageOutputStream, val, "ccv_persistence_flag");                   sei->m_ccvPersistenceFlag = val;
    sei_read_flag(pDecodedMessageOutputStream, val, "ccv_primaries_present_flag");             sei->m_ccvPrimariesPresentFlag = val;
    sei_read_flag(pDecodedMessageOutputStream, val, "ccv_min_luminance_value_present_flag");   sei->m_ccvMinLuminanceValuePresentFlag = val;
    sei_read_flag(pDecodedMessageOutputStream, val, "ccv_max_luminance_value_present_flag");   sei->m_ccvMaxLuminanceValuePresentFlag = val;
    sei_read_flag(pDecodedMessageOutputStream, val, "ccv_avg_luminance_value_present_flag");   sei->m_ccvAvgLuminanceValuePresentFlag = val;

    if (sei->m_ccvPrimariesPresentFlag)
    {
      for (i = 0; i < MAX_NUM_COMPONENT; i++)
    {
        sei_read_scode(pDecodedMessageOutputStream, 32, iVal, "ccv_primaries_x[i]");          sei->m_ccvPrimariesX[i] = iVal;
        sei_read_scode(pDecodedMessageOutputStream, 32, iVal, "ccv_primaries_y[i]");          sei->m_ccvPrimariesY[i] = iVal;
    }
  }
    if (sei->m_ccvMinLuminanceValuePresentFlag)
  {
      sei_read_code(pDecodedMessageOutputStream, 32, val, "ccv_min_luminance_value");   sei->m_ccvMinLuminanceValue = val;
    }
    if (sei->m_ccvMaxLuminanceValuePresentFlag)
    {
      sei_read_code(pDecodedMessageOutputStream, 32, val, "ccv_max_luminance_value");   sei->m_ccvMaxLuminanceValue = val;
    }
    if (sei->m_ccvAvgLuminanceValuePresentFlag)
    {
      sei_read_code(pDecodedMessageOutputStream, 32, val, "ccv_avg_luminance_value");   sei->m_ccvAvgLuminanceValue = val;
    }
  }
}
void SEIReader::xParseSEIEquirectangularProjection(vvdec_sei_message_t* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;

  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);

  CHECK ( !s->size, "vvdec_sei_content_light_level_info_t no payload" );
  vvdec_sei_equirectangular_projection_t* sei =(vvdec_sei_equirectangular_projection_t*)s->payload;
  ::memset(sei, 0, sizeof(vvdec_sei_equirectangular_projection_t));

  sei_read_flag( pDecodedMessageOutputStream, val,       "erp_cancel_flag" );              sei->m_erpCancelFlag = val;
  if( !sei->m_erpCancelFlag )
  {
    sei_read_flag( pDecodedMessageOutputStream, val,      "erp_persistence_flag"    );     sei->m_erpPersistenceFlag   = val;
    sei_read_flag( pDecodedMessageOutputStream, val,      "erp_guard_band_flag"     );     sei->m_erpGuardBandFlag     = val;
    sei_read_code( pDecodedMessageOutputStream, 2, val,   "erp_reserved_zero_2bits" );
    if ( sei->m_erpGuardBandFlag == 1)
    {
      sei_read_code( pDecodedMessageOutputStream, 3, val,     "erp_guard_band_type"       );   sei->m_erpGuardBandType  = val;
      sei_read_code( pDecodedMessageOutputStream, 8, val,     "erp_left_guard_band_width" );   sei->m_erpLeftGuardBandWidth = val;
      sei_read_code( pDecodedMessageOutputStream, 8, val,     "erp_right_guard_band_width");   sei->m_erpRightGuardBandWidth = val;
    }
  }
}

void SEIReader::xParseSEISphereRotation(vvdec_sei_message_t* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;
  int  sval;

  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);

  CHECK ( !s->size, "vvdec_sei_sphere_rotation_t no payload" );
  vvdec_sei_sphere_rotation_t* sei =(vvdec_sei_sphere_rotation_t*)s->payload;
  ::memset(sei, 0, sizeof(vvdec_sei_sphere_rotation_t));

  sei_read_flag( pDecodedMessageOutputStream, val,       "sphere_rotation_cancel_flag" );              sei->m_sphereRotationCancelFlag = val;
  if( !sei->m_sphereRotationCancelFlag )
    {
    sei_read_flag ( pDecodedMessageOutputStream,      val,   "sphere_rotation_persistence_flag"    );     sei->m_sphereRotationPersistenceFlag = val;
    sei_read_code ( pDecodedMessageOutputStream, 6,   val,   "sphere_rotation_reserved_zero_6bits" );
    sei_read_scode( pDecodedMessageOutputStream, 32, sval,   "sphere_rotation_yaw"                 );     sei->m_sphereRotationYaw = sval;
    sei_read_scode( pDecodedMessageOutputStream, 32, sval,   "sphere_rotation_pitch"               );     sei->m_sphereRotationPitch = sval;
    sei_read_scode( pDecodedMessageOutputStream, 32, sval,   "sphere_rotation_roll"                );     sei->m_sphereRotationRoll = sval;
  }
}

void SEIReader::xParseSEIOmniViewport(vvdec_sei_message_t* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t code;
  int  scode;

  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);

  CHECK ( !s->size, "vvdec_sei_omni_viewport_t no payload" );
  vvdec_sei_omni_viewport_t* sei =(vvdec_sei_omni_viewport_t*)s->payload;
  ::memset(sei, 0, sizeof(vvdec_sei_omni_viewport_t));

  sei_read_code( pDecodedMessageOutputStream, 10, code, "omni_viewport_id"          ); sei->m_omniViewportId         = code;
  sei_read_flag( pDecodedMessageOutputStream,     code, "omni_viewport_cancel_flag" ); sei->m_omniViewportCancelFlag = code;

  if (!sei->m_omniViewportCancelFlag)
  {
    uint32_t numRegions;
    sei_read_flag( pDecodedMessageOutputStream,    code,       "omni_viewport_persistence_flag" ); sei->m_omniViewportPersistenceFlag = code;
    sei_read_code( pDecodedMessageOutputStream, 4, numRegions, "omni_viewport_cnt_minus1"       ); numRegions++;
    for(uint32_t region=0; region<numRegions; region++)
    {
      vvdec_OmniViewport &viewport = sei->m_omniViewportRegions[region];
      sei_read_scode( pDecodedMessageOutputStream, 32, scode, "omni_viewport_azimuth_centre"   );   viewport.azimuthCentre   = scode;
      sei_read_scode( pDecodedMessageOutputStream, 32, scode, "omni_viewport_elevation_centre" );   viewport.elevationCentre = scode;
      sei_read_scode( pDecodedMessageOutputStream, 32, scode, "omni_viewport_tilt_centre"      );   viewport.tiltCentre      = code;
      sei_read_code( pDecodedMessageOutputStream,  32, code, "omni_viewport_hor_range"         );   viewport.horRange        = code;
      sei_read_code( pDecodedMessageOutputStream,  32, code, "omni_viewport_ver_range"         );   viewport.verRange        = code;
    }
  }
  else
  {
    sei->m_omniViewportPersistenceFlag=false;
  }
}

void SEIReader::xParseSEIRegionWisePacking(vvdec_sei_message_t* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;
  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);

  CHECK ( !s->size, "vvdec_sei_region_wise_packing_t no payload" );
  vvdec_sei_region_wise_packing_t* sei =(vvdec_sei_region_wise_packing_t*)s->payload;
  ::memset(sei, 0, sizeof(vvdec_sei_region_wise_packing_t));

  if (!sei->m_rwpCancelFlag)
    {
    sei_read_flag( pDecodedMessageOutputStream,           val,    "rwp_persistence_flag" );                 sei->m_rwpPersistenceFlag = val;
    sei_read_flag( pDecodedMessageOutputStream,           val,    "constituent_picture_matching_flag" );    sei->m_constituentPictureMatchingFlag = val;
    sei_read_code( pDecodedMessageOutputStream,       5,  val,    "rwp_reserved_zero_5bits" );
    sei_read_code( pDecodedMessageOutputStream,       8,  val,    "num_packed_regions" );                   sei->m_numPackedRegions    = val;
    sei_read_code( pDecodedMessageOutputStream,       32, val,    "proj_picture_width" );                   sei->m_projPictureWidth    = val;
    sei_read_code( pDecodedMessageOutputStream,       32, val,    "proj_picture_height" );                  sei->m_projPictureHeight   = val;
    sei_read_code( pDecodedMessageOutputStream,       16, val,    "packed_picture_width" );                 sei->m_packedPictureWidth  = val;
    sei_read_code( pDecodedMessageOutputStream,       16, val,    "packed_picture_height" );                sei->m_packedPictureHeight = val;

    for( int i=0; i < sei->m_numPackedRegions; i++ )
    {
      sei_read_code( pDecodedMessageOutputStream,     4,  val,    "rwp_reserved_zero_4bits" );
      sei_read_code( pDecodedMessageOutputStream,     3,  val,    "rwp_tTransform_type" );                  sei->m_rwpTransformType[i]   = val;
      sei_read_flag( pDecodedMessageOutputStream,         val,    "rwp_guard_band_flag" );                  sei->m_rwpGuardBandFlag[i]   = val;
      sei_read_code( pDecodedMessageOutputStream,     32, val,    "proj_region_width" );                    sei->m_projRegionWidth[i]    = val;
      sei_read_code( pDecodedMessageOutputStream,     32, val,    "proj_region_height" );                   sei->m_projRegionHeight[i]   = val;
      sei_read_code( pDecodedMessageOutputStream,     32, val,    "rwp_proj_regionTop" );                   sei->m_rwpProjRegionTop[i]   = val;
      sei_read_code( pDecodedMessageOutputStream,     32, val,    "proj_region_left" );                     sei->m_projRegionLeft[i]     = val;
      sei_read_code( pDecodedMessageOutputStream,     16, val,    "packed_region_width" );                  sei->m_packedRegionWidth[i]  = val;
      sei_read_code( pDecodedMessageOutputStream,     16, val,    "packed_region_height" );                 sei->m_packedRegionHeight[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     16, val,    "packed_region_top" );                    sei->m_packedRegionTop[i]    = val;
      sei_read_code( pDecodedMessageOutputStream,     16, val,    "packed_region_left" );                   sei->m_packedRegionLeft[i]   = val;
      if( sei->m_rwpGuardBandFlag[i] )
        {
        sei_read_code( pDecodedMessageOutputStream,   8,  val,    "rwp_left_guard_band_width" );            sei->m_rwpLeftGuardBandWidth[i]     = val;
        sei_read_code( pDecodedMessageOutputStream,   8,  val,    "rwp_right_guard_band_width" );           sei->m_rwpRightGuardBandWidth[i]    = val;
        sei_read_code( pDecodedMessageOutputStream,   8,  val,    "rwp_top_guard_band_height" );            sei->m_rwpTopGuardBandHeight[i]     = val;
        sei_read_code( pDecodedMessageOutputStream,   8,  val,    "rwp_bottom_guard_band_height" );         sei->m_rwpBottomGuardBandHeight[i]  = val;
        sei_read_flag( pDecodedMessageOutputStream,       val,    "rwp_guard_band_not_used_forPred_flag" ); sei->m_rwpGuardBandNotUsedForPredFlag[i] = val;
        for( int j=0; j < 4; j++ )
          {
          sei_read_code( pDecodedMessageOutputStream, 3,  val,     "rwp_guard_band_type" ); sei->m_rwpGuardBandType[i*4 + j] = val;
        }
        sei_read_code( pDecodedMessageOutputStream,   3,  val,    "rwp_guard_band_reserved_zero_3bits" );
      }
    }
  }
}

void SEIReader::xParseSEIGeneralizedCubemapProjection(vvdec_sei_message_t* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;

  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);
  
  CHECK ( !s->size, "vvdec_sei_generalized_cubemap_projection_t no payload" );
  vvdec_sei_generalized_cubemap_projection_t* sei =(vvdec_sei_generalized_cubemap_projection_t*)s->payload;
  ::memset(sei, 0, sizeof(vvdec_sei_generalized_cubemap_projection_t));

  sei_read_flag( pDecodedMessageOutputStream,          val,    "gcmp_cancel_flag" );                      sei->m_gcmpCancelFlag = val;
  if (!sei->m_gcmpCancelFlag)
{
    sei_read_flag( pDecodedMessageOutputStream,        val,    "gcmp_persistence_flag"    );              sei->m_gcmpPersistenceFlag = val;
    sei_read_code( pDecodedMessageOutputStream,     3, val,    "gcmp_packing_type" );                     sei->m_gcmpPackingType = val;
    sei_read_code( pDecodedMessageOutputStream,     2, val,    "gcmp_mapping_function_type"     );        sei->m_gcmpMappingFunctionType = val;

    int numFace = sei->m_gcmpPackingType == 4 || sei->m_gcmpPackingType == 5 ? 5 : 6;

    for (int i = 0; i < numFace; i++)
    {
      sei_read_code( pDecodedMessageOutputStream,   3, val,    "gcmp_face_index" );                       sei->m_gcmpFaceIndex[i] = val;
      sei_read_code( pDecodedMessageOutputStream,   2, val,    "gcmp_face_rotation" );                    sei->m_gcmpFaceRotation[i] = val;
      if (sei->m_gcmpMappingFunctionType == 2)
      {
        sei_read_code( pDecodedMessageOutputStream, 7, val,    "gcmp_function_coeff_u" );                 sei->m_gcmpFunctionCoeffU[i] = val;
        sei_read_flag( pDecodedMessageOutputStream,    val,    "gcmp_function_u_affected_by_v_flag"    ); sei->m_gcmpFunctionUAffectedByVFlag[i] = val;
        sei_read_code( pDecodedMessageOutputStream, 7, val,    "gcmp_function_coeff_v" );                 sei->m_gcmpFunctionCoeffV[i] = val;
        sei_read_flag( pDecodedMessageOutputStream,    val,    "gcmp_function_v_affected_by_u_flag"    ); sei->m_gcmpFunctionVAffectedByUFlag[i] = val;
      }
    }
    sei_read_flag( pDecodedMessageOutputStream,        val,    "gcmp_guard_band_flag" );                  sei->m_gcmpGuardBandFlag = val;
    if (sei->m_gcmpGuardBandFlag)
{
      sei_read_code( pDecodedMessageOutputStream,   3, val,    "gcmp_guard_band_type" );                   sei->m_gcmpGuardBandType = val;
      sei_read_flag( pDecodedMessageOutputStream,      val,    "gcmp_guard_band_boundary_exterior_flag" ); sei->m_gcmpGuardBandBoundaryExteriorFlag = val;
      sei_read_code( pDecodedMessageOutputStream,   4, val,    "gcmp_guard_band_samples_minus1" );         sei->m_gcmpGuardBandSamples = val+1;
}
}
}

void SEIReader::xParseSEISubpictureLevelInfo(vvdec_sei_message_t* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);
  CHECK ( !s->size, "vvdec_sei_subpicture_level_info_t no payload" );
  vvdec_sei_subpicture_level_info_t* sei =(vvdec_sei_subpicture_level_info_t*)s->payload;
  ::memset(sei, 0, sizeof(vvdec_sei_subpicture_level_info_t));

  uint32_t val;
  sei_read_code( pDecodedMessageOutputStream,   3,  val,    "sli_num_ref_levels_minus1" );            sei->m_numRefLevels  = val + 1;
  sei_read_flag( pDecodedMessageOutputStream,       val,    "sli_cbr_constraint_flag" );              sei->m_cbrConstraintFlag = val;
  sei_read_flag( pDecodedMessageOutputStream,       val,    "sli_explicit_fraction_present_flag" );   sei->m_explicitFractionPresentFlag = val;
  if (sei->m_explicitFractionPresentFlag)
  {
    sei_read_uvlc(pDecodedMessageOutputStream,      val,    "sli_num_subpics_minus1");             sei->m_numSubpics = val + 1;
#if JVET_S0176_SLI_SEI
    sei_read_code(pDecodedMessageOutputStream,  3,  val,    "sli_max_sublayers_minus1"  );            sei->m_sliMaxSublayers = val + 1;
    sei_read_flag(pDecodedMessageOutputStream,      val,    "sli_sublayer_info_present_flag");        sei->m_sliSublayerInfoPresentFlag = val;
#endif
    while (!isByteAligned())
    {
      sei_read_flag( pDecodedMessageOutputStream,   val,    "sli_alignment_zero_bit" );           CHECK (val != 0, "sli_alignment_zero_bit not equal to zero" );
    }
  }

#if JVET_S0176_SLI_SEI
  // sei parameters initialization
  for (int i = 0; i < sei->m_numRefLevels; i++)
  {
    for (int k = 0; k < sei->m_sliMaxSublayers; k++)
    {
      sei->m_refLevelIdc[i][k] = Level::VVDEC_LEVEL15_5;
    }
  }
  if (sei->m_explicitFractionPresentFlag)
  {
    for (int i = 0; i < sei->m_numRefLevels; i++)
    {
      for (int j = 0; j < sei->m_numSubpics; j++)
      {
        for (int k = 0; k < sei->m_sliMaxSublayers; k++)
        {
          sei->m_refLevelFraction[i][j][k] = 0;
        }
      }
    }
  }

  // parsing
  for (int k = sei->m_sliSublayerInfoPresentFlag ? 0 : sei->m_sliMaxSublayers - 1; k < sei->m_sliMaxSublayers; k++)
  {
    for (int i = 0; i < sei->m_numRefLevels; i++)
    {
#if JVET_S0098_SLI_FRACTION
      sei_read_code(pDecodedMessageOutputStream, 8, val, "sli_non_subpic_layers_fraction[i][k]");    sei->m_nonSubpicLayersFraction[i][k] = (Level) val;
#endif
      sei_read_code(pDecodedMessageOutputStream, 8, val, "sli_ref_level_idc[i][k]");                 sei->m_refLevelIdc[i][k] = (Level) val;

      if (sei->m_explicitFractionPresentFlag)
      {
        for (int j = 0; j < sei->m_numSubpics; j++)
        {
          sei_read_code(pDecodedMessageOutputStream, 8, val, "sli_ref_level_fraction_minus1[i][j][k]");  sei->m_refLevelFraction[i][j][k] = val;
        }
      }
    }
  }

  // update the inference of m_refLevelIdc[][] and m_refLevelFraction[][][]
  if (!sei->m_sliSublayerInfoPresentFlag)
  {
    for (int k = sei->m_sliMaxSublayers - 2; k >= 0; k--)
    {
      for (int i = 0; i < sei->m_numRefLevels; i++)
      {
#if JVET_S0098_SLI_FRACTION
        sei->m_nonSubpicLayersFraction[i][k] = sei->m_nonSubpicLayersFraction[i][sei->m_sliMaxSublayers - 1];
#endif
        sei->m_refLevelIdc[i][k] = sei->m_refLevelIdc[i][sei->m_sliMaxSublayers - 1];
        if (sei->m_explicitFractionPresentFlag)
        {
          for (int j = 0; j < sei->m_numSubpics; j++)
          {
            sei->m_refLevelFraction[i][j][k] = sei->m_refLevelFraction[i][j][sei->m_sliMaxSublayers - 1];
          }
        }
      }
    }
  }
#else
  if (sei->m_explicitFractionPresentFlag)
  {
    sei->m_refLevelFraction.resize(sei->m_numRefLevels);
  }
  for( int i = 0; i  <  sei->m_numRefLevels; i++ )
  {
    sei_read_code( pDecodedMessageOutputStream,   8,  val,    "sli_ref_level_idc[i]" );         sei->m_refLevelIdc[i]  = (Level::Name) val;
    if( sei->m_explicitFractionPresentFlag )
    {
      for( int j = 0; j  <  sei->m_numSubpics; j++ )
      {
        sei_read_code( pDecodedMessageOutputStream,   8,  val,    "sli_ref_level_fraction_minus1[i][j]" );  sei->m_refLevelFraction[i][j]= val;
      }
    }
  }
#endif
}

void SEIReader::xParseSEISampleAspectRatioInfo(vvdec_sei_message_t* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;

  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);

  CHECK ( !s->size, "vvdec_sei_sample_aspect_ratio_info_t no payload" );
  vvdec_sei_sample_aspect_ratio_info_t* sei =(vvdec_sei_sample_aspect_ratio_info_t*)s->payload;
  ::memset(sei, 0, sizeof(vvdec_sei_sample_aspect_ratio_info_t));

  sei_read_flag( pDecodedMessageOutputStream,           val,    "sari_cancel_flag" );                      sei->m_sariCancelFlag = val;
  if (!sei->m_sariCancelFlag)
  {
    sei_read_flag( pDecodedMessageOutputStream,         val,    "sari_persistence_flag" );                 sei->m_sariPersistenceFlag = val;
    sei_read_code( pDecodedMessageOutputStream,     8,  val,    "sari_aspect_ratio_idc" );                 sei->m_sariAspectRatioIdc = val;
    if (sei->m_sariAspectRatioIdc == 255)
    {
      sei_read_code( pDecodedMessageOutputStream,  16,  val,    "sari_sar_width" );                        sei->m_sariSarWidth = val;
      sei_read_code( pDecodedMessageOutputStream,  16,  val,    "sari_sar_height" );                       sei->m_sariSarHeight = val;
    }
  }
}

#if JVET_S0257_DUMP_360SEI_MESSAGE
void SeiCfgFileDump::write360SeiDump (std::string decoded360MessageFileName, SEIMessages& seis, const SPS* sps)
{
  if (m_360SEIMessageDumped)
  {
    return;
  }

  SEIMessages equirectangularProjectionSEIs = getSeisByType(seis, EQUIRECTANGULAR_PROJECTION);
  if (!equirectangularProjectionSEIs.empty())
  {
    SEIEquirectangularProjection* sei = (SEIEquirectangularProjection*)equirectangularProjectionSEIs.front();
    xDumpSEIEquirectangularProjection(*sei, sps, decoded360MessageFileName);
    m_360SEIMessageDumped = true;
  }
  else
  {
    SEIMessages generalizedCubemapProjectionSEIs = getSeisByType(seis, GENERALIZED_CUBEMAP_PROJECTION);
    if (!generalizedCubemapProjectionSEIs.empty())
    {
      SEIGeneralizedCubemapProjection* sei = (SEIGeneralizedCubemapProjection*)generalizedCubemapProjectionSEIs.front();
      xDumpSEIGeneralizedCubemapProjection(*sei, sps, decoded360MessageFileName);
      m_360SEIMessageDumped = true; 
    }
  }
}

void SeiCfgFileDump::xDumpSEIEquirectangularProjection     (SEIEquirectangularProjection &sei, const SPS* sps, std::string decoded360MessageFileName)
{
  if (!decoded360MessageFileName.empty())
  {
    FILE *fp = fopen(decoded360MessageFileName.c_str(), "w");
    if (fp)
    {
      int chromaFormatTable[4] = {400, 420, 422, 444};
      fprintf(fp, "InputBitDepth                 : %d    # Input bitdepth\n", sps->getBitDepth(CHANNEL_TYPE_LUMA));
      fprintf(fp, "InputChromaFormat             : %d    # Ratio of luminance to chrominance samples\n", chromaFormatTable[sps->getChromaFormatIdc()]);
      fprintf(fp, "SourceWidth                   : %d    # Input  frame width\n", sps->getMaxPicWidthInLumaSamples());
      fprintf(fp, "SourceHeight                  : %d    # Input  frame height\n\n", sps->getMaxPicHeightInLumaSamples());

      fprintf(fp, "InputGeometryType             : 0     # 0: equirectangular; 1: cubemap; 2: equalarea; this should be in the cfg of per sequence.\n");
      if (sei.m_erpGuardBandFlag == 1)
      {
        fprintf(fp, "InputPERP                     : 1     # 0: original ERP input; 1: padded ERP input\n");
        fprintf(fp, "CodingPERP                    : 0     # 0: coding with original ERP size; 1: coding with padded ERP\n");
      }
      fclose(fp);
      m_360SEIMessageDumped = true;
    }
    else
    {
      msg( ERROR, "File %s could not be opened.\n", decoded360MessageFileName.c_str() );
    }
  }
}
void SeiCfgFileDump::xDumpSEIGeneralizedCubemapProjection  (SEIGeneralizedCubemapProjection &sei, const SPS* sps, std::string decoded360MessageFileName)
{
  if (!sei.m_gcmpCancelFlag)
  {
    int numFace = sei.m_gcmpPackingType == 4 || sei.m_gcmpPackingType == 5 ? 5 : 6;
    int packingTypeTable[6][2] = {{6, 1}, {3, 2}, {2, 3}, {1, 6}, {1, 5}, {5, 1}};
    int rotationTable[4] = {0, 90, 180, 270};
    std::string packingTypeStr = "";
    std::string gcmpsettingsStr = "";
    std::ostringstream oss;

    packingTypeStr += "SourceFPStructure                 : " + std::to_string(packingTypeTable[sei.m_gcmpPackingType][0]) + " " + std::to_string(packingTypeTable[sei.m_gcmpPackingType][1]);
    gcmpsettingsStr += "InputGCMPSettings                 : ";

    for (int i = 0; i < numFace; i++)
    {
      int rotation = rotationTable[sei.m_gcmpFaceRotation[i]];
      if (sei.m_gcmpFaceIndex[i] == 1)
      {
        rotation = (rotation + 270) % 360 + 360;
      }
      else if (sei.m_gcmpFaceIndex[i] == 2)
      {
        rotation = (rotation + 180) % 360 + 360;
      }
      else
      {
        rotation += 360;
      }
      if (i % packingTypeTable[sei.m_gcmpPackingType][1] == 0)
      {
        packingTypeStr += "   ";
      }
      packingTypeStr += std::to_string(sei.m_gcmpFaceIndex[i]) + " " + std::to_string(rotation) + " ";

      if (sei.m_gcmpMappingFunctionType == 2)
      {
        double a = ((int)sei.m_gcmpFunctionCoeffU[i] + 1) / 128.0;
        double b = ((int)sei.m_gcmpFunctionCoeffV[i] + 1) / 128.0;
        oss.str("");
        oss<<a;
        std::string a_str = oss.str();
        oss.str("");
        oss<<b;
        std::string b_str = oss.str();
        gcmpsettingsStr += a_str + " " + std::to_string(sei.m_gcmpFunctionUAffectedByVFlag[i]) + " " + b_str + " " + std::to_string(sei.m_gcmpFunctionVAffectedByUFlag[i]) + "   ";
      }
    }
    if (!decoded360MessageFileName.empty())
    {
      FILE *fp = fopen(decoded360MessageFileName.c_str(), "w");
      if (fp)
      {
        int chromaFormatTable[4] = {400, 420, 422, 444};
        fprintf(fp, "InputBitDepth                 : %d    # Input bitdepth\n", sps->getBitDepth(CHANNEL_TYPE_LUMA));
        fprintf(fp, "InputChromaFormat             : %d    # Ratio of luminance to chrominance samples\n", chromaFormatTable[sps->getChromaFormatIdc()]);
        fprintf(fp, "SourceWidth                   : %d    # Input  frame width\n", sps->getMaxPicWidthInLumaSamples());
        fprintf(fp, "SourceHeight                  : %d    # Input  frame height\n\n", sps->getMaxPicHeightInLumaSamples());

        fprintf(fp, "InputGeometryType             : 15    # 0: equirectangular; 1: cubemap; 2: equalarea; this should be in the cfg of per sequence.\n");

        packingTypeStr += " # frame packing order: numRows numCols Row0Idx0 ROT Row0Idx1 ROT ... Row1...";
        gcmpsettingsStr += " # mapping function parameters for each face: u coefficient, u affected by v flag, v coefficient, v affected by u flag";
        fprintf(fp, "%s\n", packingTypeStr.c_str());
        fprintf(fp, "InputGCMPMappingType              : %d                                    # 0: CMP; 1: EAC; 2: parameterized CMP\n", (int)sei.m_gcmpMappingFunctionType);
        if ((int)sei.m_gcmpMappingFunctionType == 2)
          fprintf(fp, "%s\n", gcmpsettingsStr.c_str());
        fprintf(fp, "InputGCMPPaddingFlag              : %d                                   # 0: input without guard bands; 1: input with guard bands\n", sei.m_gcmpGuardBandFlag);
        if (sei.m_gcmpGuardBandFlag)
        {
          fprintf(fp, "InputGCMPPaddingType              : %d                                   # 0: unspecified(repetitive padding is used); 1: repetitive padding; 2: copy from neighboring face; 3: geometry padding\n", (int)sei.m_gcmpGuardBandType);
          fprintf(fp, "InputGCMPPaddingExteriorFlag      : %d                                   # 0: guard bands only on discontinuous edges; 1: guard bands on both discontinuous edges and frame boundaries\n", sei.m_gcmpGuardBandBoundaryExteriorFlag);
          fprintf(fp, "InputGCMPPaddingSize              : %d                                   # guard band size for input GCMP\n", (int)sei.m_gcmpGuardBandSamplesMinus1 + 1);
        }
        fclose(fp);
        m_360SEIMessageDumped = true;
      }
      else
      {
        msg( ERROR, "File %s could not be opened.\n", decoded360MessageFileName.c_str() );
      }
    }
  }
}

#endif

//! \}
