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

namespace vvdec
{

#if ENABLE_TRACING
void  xTraceSEIHeader()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== SEI message ===========\n");
}

void  xTraceSEIMessageType(vvdecSEIPayloadType payloadType)
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

static inline void output_sei_message_header(vvdecSEI* sei, std::ostream *pDecodedMessageOutputStream, uint32_t payloadSize)
{
  if (pDecodedMessageOutputStream)
  {
    std::string seiMessageHdr(SEI_internal::getSEIMessageString(sei->payloadType)); seiMessageHdr+=" SEI message";
    (*pDecodedMessageOutputStream) << std::setfill('-') << std::setw((int)seiMessageHdr.size()) << "-" << std::setfill(' ') << "\n" << seiMessageHdr << " (" << payloadSize << " bytes)"<< "\n";
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
  setBitstream(bs);
  CHECK(m_pcBitstream->getNumBitsUntilByteAligned(), "Bitstream not aligned");

  seiMessages seiListInCurNalu;
  do
  {
    xReadSEImessage( seiList, nalUnitType, nuh_layer_id, temporalId, vps, sps, hrd, pDecodedMessageOutputStream);
    if( !seiList.empty() ){  seiListInCurNalu.push_back(seiList.back()); }
    /* SEI messages are an integer number of bytes, something has failed
    * in the parsing if bitstream not byte-aligned */
    CHECK(m_pcBitstream->getNumBitsUntilByteAligned(), "Bitstream not aligned");
  }
  while (m_pcBitstream->getNumBitsLeft() > 8);

  seiMessages fillerData = SEI_internal::getSeisByType(seiListInCurNalu, VVDEC_FILLER_PAYLOAD);
  CHECK(fillerData.size() > 0 && fillerData.size() != seiListInCurNalu.size(), "When an SEI NAL unit contains an SEI message with payloadType equal to filler payload, the SEI NAL unit shall not contain any other SEI message with payloadType not equal to filler payload");

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
  xTraceSEIMessageType((vvdecSEIPayloadType)payloadType);
#endif

  /* extract the payload for this single SEI message.
   * This allows greater safety in erroneous parsing of an SEI message
   * from affecting subsequent messages.
   * After parsing the payload, bs needs to be restored as the primary
   * bitstream.
   */
  InputBitstream *bs = getBitstream();
  auto substream = bs->extractSubstream( payloadSize * 8 );
  setBitstream( substream.get() );

  const vvdecSEIBufferingPeriod *bp = NULL;

  vvdecSEI *s = NULL;
  vvdecSEIPayloadType type = (vvdecSEIPayloadType)payloadType;

  if(nalUnitType == NAL_UNIT_PREFIX_SEI)
  {
    switch (payloadType)
    {
    case VVDEC_USER_DATA_REGISTERED_ITU_T_T35:
          s = SEI_internal::allocSEI( type ) ;
          xParseSEIUserDataRegistered( s, payloadSize, pDecodedMessageOutputStream);
          break;
    case VVDEC_USER_DATA_UNREGISTERED:
          s = SEI_internal::allocSEI( type ) ;
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
        s = SEI_internal::allocSEI( type ) ;
        xParseSEIDecodingUnitInfo( s, payloadSize, *bp, temporalId, pDecodedMessageOutputStream);
      }
      break;
    case VVDEC_BUFFERING_PERIOD:
        s = SEI_internal::allocSEI( type ) ;
        xParseSEIBufferingPeriod(s, payloadSize, pDecodedMessageOutputStream);
        if( s )
        {
          vvdecSEIBufferingPeriod* bufferingPeriod = reinterpret_cast<vvdecSEIBufferingPeriod *>(s->payload);
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
          s = SEI_internal::allocSEI( type ) ;
          xParseSEIPictureTiming(s, payloadSize, temporalId, *bp, pDecodedMessageOutputStream);
          if( s )
          {
            vvdecSEIPictureTiming* picTiming= reinterpret_cast<vvdecSEIPictureTiming *>(s->payload);
            hrd.setPictureTimingSEI( picTiming );
          }
        }
      }
      break;
    case VVDEC_SCALABLE_NESTING:
        s = SEI_internal::allocSEI( type ) ;
        xParseSEIScalableNesting(s, nalUnitType, nuh_layer_id, payloadSize, vps, sps, pDecodedMessageOutputStream);
      break;
    case VVDEC_FRAME_FIELD_INFO:
        s = SEI_internal::allocSEI( type ) ;
        xParseSEIFrameFieldinfo( s, payloadSize, pDecodedMessageOutputStream);
      break;
    case VVDEC_DEPENDENT_RAP_INDICATION:
        s = SEI_internal::allocSEI( type ) ;
        xParseSEIDependentRAPIndication( s, payloadSize, pDecodedMessageOutputStream);
      break;
    case VVDEC_FRAME_PACKING:
        s = SEI_internal::allocSEI( type ) ;
        xParseSEIFramePacking( s, payloadSize, pDecodedMessageOutputStream);
      break;
    case VVDEC_PARAMETER_SETS_INCLUSION_INDICATION:
        s = SEI_internal::allocSEI( type ) ;
        xParseSEIParameterSetsInclusionIndication( s, payloadSize, pDecodedMessageOutputStream);
      break;
    case VVDEC_MASTERING_DISPLAY_COLOUR_VOLUME:
        s = SEI_internal::allocSEI( type ) ;
        xParseSEIMasteringDisplayColourVolume( s, payloadSize, pDecodedMessageOutputStream);
      break;
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
    case VVDEC_ALTERNATIVE_TRANSFER_CHARACTERISTICS:
        s = SEI_internal::allocSEI( type ) ;
        xParseSEIAlternativeTransferCharacteristics( s, payloadSize, pDecodedMessageOutputStream);
      break;
#endif
    case VVDEC_EQUIRECTANGULAR_PROJECTION:
        s = SEI_internal::allocSEI( type ) ;
        xParseSEIEquirectangularProjection( s, payloadSize, pDecodedMessageOutputStream);
      break;
    case VVDEC_SPHERE_ROTATION:
        s = SEI_internal::allocSEI( type ) ;
        xParseSEISphereRotation( s, payloadSize, pDecodedMessageOutputStream);
      break;
    case VVDEC_OMNI_VIEWPORT:
        s = SEI_internal::allocSEI( type ) ;
        xParseSEIOmniViewport( s, payloadSize, pDecodedMessageOutputStream);
      break;
    case VVDEC_REGION_WISE_PACKING:
        s = SEI_internal::allocSEI( type ) ;
        xParseSEIRegionWisePacking( s, payloadSize, pDecodedMessageOutputStream);
      break;
    case VVDEC_GENERALIZED_CUBEMAP_PROJECTION:
        s = SEI_internal::allocSEI( type ) ;
        xParseSEIGeneralizedCubemapProjection( s, payloadSize, pDecodedMessageOutputStream);
      break;
    case VVDEC_SUBPICTURE_LEVEL_INFO:
        s = SEI_internal::allocSEI( type ) ;
        xParseSEISubpictureLevelInfo(s, payloadSize, pDecodedMessageOutputStream);
      break;
    case VVDEC_SAMPLE_ASPECT_RATIO_INFO:
        s = SEI_internal::allocSEI( type ) ;
        xParseSEISampleAspectRatioInfo( s, payloadSize, pDecodedMessageOutputStream);
      break;
    case VVDEC_FILM_GRAIN_CHARACTERISTICS:
        s = SEI_internal::allocSEI( type ) ;
        xParseSEIFilmGrainCharacteristics(s, payloadSize, pDecodedMessageOutputStream);
      break;
    case VVDEC_CONTENT_LIGHT_LEVEL_INFO:
        s = SEI_internal::allocSEI( type ) ;
        xParseSEIContentLightLevelInfo( s, payloadSize, pDecodedMessageOutputStream);
      break;
    case VVDEC_AMBIENT_VIEWING_ENVIRONMENT:
        s = SEI_internal::allocSEI( type ) ;
        xParseSEIAmbientViewingEnvironment( s, payloadSize, pDecodedMessageOutputStream);
      break;
    case VVDEC_CONTENT_COLOUR_VOLUME:
        s = SEI_internal::allocSEI( type ) ;
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
        s = SEI_internal::allocSEI( type ) ;
        xParseSEIuserDataUnregistered(s, payloadSize, pDecodedMessageOutputStream);
        break;
      case  VVDEC_DECODED_PICTURE_HASH:
        s = SEI_internal::allocSEI( type ) ;
        xParseSEIDecodedPictureHash(s, payloadSize, pDecodedMessageOutputStream);
        break;
      case VVDEC_SCALABLE_NESTING:
        s = SEI_internal::allocSEI( type ) ;
        xParseSEIScalableNesting(s, nalUnitType, nuh_layer_id, payloadSize, vps, sps, pDecodedMessageOutputStream);
        break;
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

  if( s )
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
  setBitstream( bs );
}



/**
 * parse bitstream bs and unpack a decoded picture hash SEI message
 * of payloadSize bytes into sei.
 */
void SEIReader::xParseSEIDecodedPictureHash(vvdecSEI* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t bytesRead = 0;
  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);

  CHECK_FATAL( !s || s->payload == NULL, "allocation error in vvdecSEIDecodedPictureHash" );

  vvdecSEIDecodedPictureHash* sei =(vvdecSEIDecodedPictureHash*)s->payload;
  ::memset(sei, 0, sizeof(vvdecSEIDecodedPictureHash));

  uint32_t val;
  sei_read_code( pDecodedMessageOutputStream, 8, val, "dph_sei_hash_type" );
  sei->method = static_cast<vvdecHashType>(val); bytesRead++;
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
    default: THROW_RECOVERABLE("Unknown hash type"); break;
  }

  if (pDecodedMessageOutputStream)
  {
    (*pDecodedMessageOutputStream) << "  " << std::setw(55) << traceString << ": " << std::hex << std::setfill('0');
  }

  CHECK( payloadSize-bytesRead > 48, "payload size of digest must be <= 48 in vvdecSEIDecodedPictureHash" );
  for(int i=0;bytesRead < payloadSize; bytesRead++,i++)
  {
    sei_read_code( NULL, 8, val, traceString);
    sei->digest[i] = (uint8_t)val;
    sei->digest_length++;

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

void SEIReader::xParseSEIScalableNesting(vvdecSEI* s, const NalUnitType nalUnitType, const uint32_t nuhLayerId, uint32_t payloadSize, const VPS *vps, const SPS *sps, std::ostream *decodedMessageOutputStream)
{
  uint32_t symbol;
  output_sei_message_header(s, decodedMessageOutputStream, payloadSize);

  CHECK_FATAL( !s || s->payload == NULL, "allocation error in vvdecSEIScalableNesting" );
  vvdecSEIScalableNesting* sei =(vvdecSEIScalableNesting*)s->payload;
  ::memset(sei, 0, sizeof(vvdecSEIScalableNesting));

  sei_read_flag(decodedMessageOutputStream, symbol, "sn_ols_flag");    sei->snOlsFlag = symbol;
  sei_read_flag(decodedMessageOutputStream, symbol, "sn_subpic_flag"); sei->snSubpicFlag = symbol;
  if (sei->snOlsFlag)
  {
    sei_read_uvlc(decodedMessageOutputStream, symbol, "sn_nuolss_minus1"); sei->snNumOlss = symbol+1;
    CHECK( sei->snNumOlss > 64, "sn_nuolss_minus1 must be < 64 in vvdecSEIScalableNesting" );

    for (uint32_t i = 0; i < sei->snNumOlss; i++)
    {
      sei_read_uvlc(decodedMessageOutputStream, symbol, "sn_ols_idx_delta_minus1[i]"); sei->snOlsIdxDelta[i] = symbol+1;
    }
    for (uint32_t i = 0; i < sei->snNumOlss; i++)
    {
      if (i == 0)
      {
        sei->snOlsIdx[i] = sei->snOlsIdxDelta[i]-1;
      }
      else
      {
        sei->snOlsIdx[i] = sei->snOlsIdxDelta[i]-1 + sei->snOlsIdxDelta[i - 1];
      }
    }
    if (vps && vps->getVPSId() != 0)
    {
      uint32_t lowestLayerId = MAX_UINT;
      for (uint32_t olsIdxForSEI = 0; olsIdxForSEI < sei->snNumOlss; olsIdxForSEI++)
      {
        int olsIdx = sei->snOlsIdx[olsIdxForSEI];
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
    sei_read_flag(decodedMessageOutputStream, symbol, "sn_all_layers_flag"); sei->snAllLayersFlag = symbol;
    if (!sei->snAllLayersFlag)
    {
      sei_read_uvlc(decodedMessageOutputStream, symbol, "sn_nulayers_minus1"); sei->snNumLayers = symbol+1;
      sei->snLayerId[0] = nuhLayerId;
      for (uint32_t i = 1; i < sei->snNumLayers; i++)
      {
        sei_read_code(decodedMessageOutputStream, 6, symbol, "sn_layer_id[i]"); sei->snLayerId[i] = symbol;
      }
    }
  }
  if (sei->snSubpicFlag)
  {
    sei_read_uvlc(decodedMessageOutputStream, symbol, "sn_nusubpics_minus1"); sei->snNumSubpics = symbol + 1;
    sei_read_uvlc(decodedMessageOutputStream, symbol, "sn_subpic_id_len_minus1"); sei->snSubpicIdLen = symbol + 1;
    for (uint32_t i = 0; i < sei->snNumSubpics; i++)
    {
      sei_read_code(decodedMessageOutputStream, sei->snSubpicIdLen, symbol, "sn_subpic_id[i]"); sei->snSubpicId[i] = symbol;
    }
  }

  sei_read_uvlc(decodedMessageOutputStream, symbol, "sn_nuseis_minus1"); sei->snNumSEIs = symbol + 1;
  CHECK (sei->snNumSEIs > 64, "The value of sn_nuseis_minus1 shall be in the range of 0 to 63");

  // byte alignment
  while( m_pcBitstream->getNumBitsUntilByteAligned() )
  {
    sei_read_flag(decodedMessageOutputStream, symbol, "sn_zero_bit");
  }

  // read nested SEI messages
  for (uint32_t i=0; i<sei->snNumSEIs; i++)
  {
    seiMessages tmpSeiList;
    xReadSEImessage(tmpSeiList, nalUnitType, nuhLayerId, 0, vps, sps, m_nestedHrd, decodedMessageOutputStream);
    CHECK( tmpSeiList.empty(), "read empty nested sei list." );

    if (tmpSeiList.front()->payloadType == VVDEC_BUFFERING_PERIOD)
    {
      vvdecSEIBufferingPeriod *bp = (vvdecSEIBufferingPeriod*) tmpSeiList.front();
      m_nestedHrd.setBufferingPeriodSEI(bp);
    }
    sei->nestedSEIs[i] = tmpSeiList.front();
  }

  xCheckScalableNestingConstraints(sei, nalUnitType, vps);

  if (decodedMessageOutputStream)
  {
    (*decodedMessageOutputStream) << "End of scalable nesting SEI message\n";
  }
}

void SEIReader::xCheckScalableNestingConstraints(const vvdecSEIScalableNesting* sei, const NalUnitType nalUnitType, const VPS* vps)
{
  const std::vector<int> vclAssociatedSeiList { 3, 19, 45, 129, 137, 144, 145, 147, 148, 149, 150, 153, 154, 155, 156, 168, 204 };

  bool containBPorPTorDUIorSLI = false;
  bool containNoBPorPTorDUIorSLI = false;

  for (auto* nestedsei : sei->nestedSEIs)
  {
    if( !nestedsei )
    {
      continue;
    }
    CHECK(nestedsei->payloadType == VVDEC_FILLER_PAYLOAD || nestedsei->payloadType == VVDEC_SCALABLE_NESTING, "An SEI message that has payloadType equal to filler payload or scalable nesting shall not be contained in a scalable nesting SEI message");

    CHECK(nestedsei->payloadType != VVDEC_FILLER_PAYLOAD && nestedsei->payloadType != VVDEC_DECODED_PICTURE_HASH && nalUnitType != NAL_UNIT_PREFIX_SEI, "When a scalable nesting SEI message contains an SEI message that has payloadType not equal to filler payload or decoded picture hash, the SEI NAL unit containing the scalable nesting SEI message shall have nal_unit_type equal to PREFIX_SEI_NUT");

    CHECK(nestedsei->payloadType == VVDEC_DECODED_PICTURE_HASH && nalUnitType != NAL_UNIT_SUFFIX_SEI, "When a scalable nesting SEI message contains an SEI message that has payloadType equal to decoded picture hash, the SEI NAL unit containing the scalable nesting SEI message shall have nal_unit_type equal to SUFFIX_SEI_NUT");

    CHECK(nestedsei->payloadType == VVDEC_DECODED_PICTURE_HASH && !sei->snSubpicFlag, "When the scalable nesting SEI message contains an SEI message that has payloadType equal to decoded picture hash, the value of sn_subpic_flag shall be equal to 1");

    CHECK(nestedsei->payloadType == VVDEC_SUBPICTURE_LEVEL_INFO && sei->snSubpicFlag, "When the scalable nesting SEI message contains an SEI message that has payloadType equal to SLI, the value of sn_subpic_flag shall be equal to 0");

    if( vps )
    {
      CHECK( vps->getGeneralHrdParameters()->getGeneralSamePicTimingInAllOlsFlag() && nestedsei->payloadType == VVDEC_PICTURE_TIMING,
             "When general_same_pic_timing_in_all_ols_flag is equal to 1, there shall be no SEI NAL unit that contain a scalable-nested SEI message with payloadType equal to PT" );
    }

    for (int i = 0; i < (int)vclAssociatedSeiList.size(); i++)
    {
      CHECK(nestedsei->payloadType == vclAssociatedSeiList[i] && sei->snOlsFlag, "When the scalable nesting SEI message contains an SEI message that has payloadType equal to a value in vclAssociatedSeiList, the value of sn_ols_flag shall be equal to 0");
    }

    if (nestedsei->payloadType == VVDEC_BUFFERING_PERIOD || nestedsei->payloadType == VVDEC_PICTURE_TIMING || nestedsei->payloadType == VVDEC_DECODING_UNIT_INFO || nestedsei->payloadType == VVDEC_SUBPICTURE_LEVEL_INFO)
    {
      containBPorPTorDUIorSLI = true;
      CHECK(!sei->snOlsFlag, "When the scalable nesting SEI message contains an SEI message that has payloadType equal to BP, PT, or DUI, or SLI, the value of sn_ols_flag shall be equal to 1");
    }
    if (!(nestedsei->payloadType == VVDEC_BUFFERING_PERIOD || nestedsei->payloadType == VVDEC_PICTURE_TIMING || nestedsei->payloadType == VVDEC_DECODING_UNIT_INFO || nestedsei->payloadType == VVDEC_SUBPICTURE_LEVEL_INFO))
    {
      containNoBPorPTorDUIorSLI = true;
    }
  }
  CHECK(containBPorPTorDUIorSLI && containNoBPorPTorDUIorSLI, "When a scalable nesting SEI message contains a BP, PT, DUI, or SLI SEI message, the scalable nesting SEI message shall not contain any other SEI message with payloadType not equal to BP, PT, DUI, or SLI");
}

void SEIReader::xParseSEIDecodingUnitInfo(vvdecSEI* s, uint32_t payloadSize, const vvdecSEIBufferingPeriod& bp, const uint32_t temporalId, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;

  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);
  CHECK_FATAL( !s || s->payload == NULL, "allocation error in vvdecSEIDecodingUnitInfo" );
  vvdecSEIDecodingUnitInfo* sei =(vvdecSEIDecodingUnitInfo*)s->payload;
  ::memset(sei, 0, sizeof(vvdecSEIDecodingUnitInfo));

  sei_read_uvlc( pDecodedMessageOutputStream, val, "decoding_unit_idx");
  sei->decodingUnitIdx = val;

  if(!bp.decodingUnitCpbParamsInPicTimingSeiFlag)
  {
    for (int i = temporalId; i <= bp.bpMaxSubLayers - 1; i++)
    {
      if (i < (bp.bpMaxSubLayers - 1))
      {
        sei_read_flag( pDecodedMessageOutputStream, val, "dui_sub_layer_delays_present_flag[i]" );
        sei->duiSubLayerDelaysPresentFlag[i] = val;
      }
      else
      {
        sei->duiSubLayerDelaysPresentFlag[i] = 1;
      }
      if( sei->duiSubLayerDelaysPresentFlag[i] )
      {
        sei_read_code( pDecodedMessageOutputStream, bp.duCpbRemovalDelayIncrementLength, val, "du_spt_cpb_removal_delay_increment[i]");
        sei->duSptCpbRemovalDelayIncrement[i] = val;
      }
      else
      {
        sei->duSptCpbRemovalDelayIncrement[i] = 0;
      }
    }
  }
  else
  {
    for( int i = temporalId; i < bp.bpMaxSubLayers - 1; i ++ )
    {
      sei->duSptCpbRemovalDelayIncrement[i] = 0;
    }
  }
  if (bp.decodingUnitDpbDuParamsInPicTimingSeiFlag)
  {
    sei_read_flag( pDecodedMessageOutputStream, val, "dpb_output_du_delay_present_flag"); sei->dpbOutputDuDelayPresentFlag = (val != 0);
  }
  else
  {
    sei->dpbOutputDuDelayPresentFlag = false;
  }
  if(sei->dpbOutputDuDelayPresentFlag)
  {
    sei_read_code( pDecodedMessageOutputStream, bp.dpbOutputDelayDuLength, val, "pic_spt_dpb_output_du_delay");
    sei->picSptDpbOutputDuDelay = val;
  }
}

void SEIReader::xParseSEIBufferingPeriod(vvdecSEI* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  int i, nalOrVcl;
  uint32_t code;

  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);
  CHECK_FATAL( !s || s->payload == NULL, "allocation error in vvdecSEIBufferingPeriod" );
  vvdecSEIBufferingPeriod* sei =(vvdecSEIBufferingPeriod*)s->payload;
  ::memset(sei, 0, sizeof(vvdecSEIBufferingPeriod));

  sei_read_flag( pDecodedMessageOutputStream, code, "bp_nal_hrd_parameters_present_flag" );             sei->bpNalCpbParamsPresentFlag = code;
  sei_read_flag( pDecodedMessageOutputStream, code, "bp_vcl_hrd_parameters_present_flag" );             sei->bpVclCpbParamsPresentFlag = code;

  sei_read_code( pDecodedMessageOutputStream, 5, code, "initial_cpb_removal_delay_length_minus1" );     sei->initialCpbRemovalDelayLength = code + 1;
  sei_read_code( pDecodedMessageOutputStream, 5, code, "cpb_removal_delay_length_minus1" );             sei->cpbRemovalDelayLength        = code + 1;
  sei_read_code( pDecodedMessageOutputStream, 5, code, "dpb_output_delay_length_minus1" );              sei->dpbOutputDelayLength         = code + 1;
  sei_read_flag( pDecodedMessageOutputStream, code, "bp_decoding_unit_hrd_params_present_flag" );       sei->bpDecodingUnitHrdParamsPresentFlag = code;
  if( sei->bpDecodingUnitHrdParamsPresentFlag )
  {
    sei_read_code( pDecodedMessageOutputStream, 5, code, "du_cpb_removal_delay_increment_length_minus1" );  sei->duCpbRemovalDelayIncrementLength = code + 1;
    sei_read_code( pDecodedMessageOutputStream, 5, code, "dpb_output_delay_du_length_minus1" );             sei->dpbOutputDelayDuLength = code + 1;
    sei_read_flag( pDecodedMessageOutputStream, code, "decoding_unit_cpb_params_in_pic_timing_sei_flag" );  sei->decodingUnitCpbParamsInPicTimingSeiFlag = code;
    sei_read_flag(pDecodedMessageOutputStream, code, "decoding_unit_dpb_du_params_in_pic_timing_sei_flag"); sei->decodingUnitDpbDuParamsInPicTimingSeiFlag = code;
  }
  else
  {
    sei->duCpbRemovalDelayIncrementLength = 24;
    sei->dpbOutputDelayDuLength = 24;
    sei->decodingUnitDpbDuParamsInPicTimingSeiFlag = false;
  }

  CHECK(sei->altCpbParamsPresentFlag && sei->bpDecodingUnitHrdParamsPresentFlag,"When bp_alt_cpb_params_present_flag is equal to 1, the value of bp_du_hrd_params_present_flag shall be equal to 0");

  sei_read_flag( pDecodedMessageOutputStream, code, "concatenation_flag");
  sei->concatenationFlag = code;
  sei_read_flag ( pDecodedMessageOutputStream, code, "additional_concatenation_info_present_flag");
  sei->additionalConcatenationInfoPresentFlag = code;
  if (sei->additionalConcatenationInfoPresentFlag)
  {
    sei_read_code( pDecodedMessageOutputStream, sei->initialCpbRemovalDelayLength, code, "max_initial_removal_delay_for_concatenation" );
    sei->maxInitialRemovalDelayForConcatenation = code;
  }

  sei_read_code( pDecodedMessageOutputStream, ( sei->cpbRemovalDelayLength ), code, "au_cpb_removal_delay_delta_minus1" );
  sei->auCpbRemovalDelayDelta = code + 1;

  sei_read_code(pDecodedMessageOutputStream, 3, code, "bp_max_sub_layers_minus1");
  sei->bpMaxSubLayers = code + 1;
  if (sei->bpMaxSubLayers - 1 > 0)
  {
    sei_read_flag(pDecodedMessageOutputStream, code, "cpb_removal_delay_deltas_present_flag");
    sei->cpbRemovalDelayDeltasPresentFlag = code;
  }
  else
  {
    sei->cpbRemovalDelayDeltasPresentFlag = false;
  }
  if (sei->cpbRemovalDelayDeltasPresentFlag)
  {
    sei_read_uvlc( pDecodedMessageOutputStream, code, "nucpb_removal_delay_deltas_minus1" );               sei->numCpbRemovalDelayDeltas = code + 1;
    CHECK( sei->numCpbRemovalDelayDeltas > 14, "nucpb_removal_delay_deltas_minus1 must be <= 13 in vvdecSEIBufferingPeriod" );

    for( i = 0; i < sei->numCpbRemovalDelayDeltas; i ++ )
    {
      sei_read_code( pDecodedMessageOutputStream, ( sei->cpbRemovalDelayLength ), code, "cpb_removal_delay_delta[i]" );
      sei->cpbRemovalDelayDelta[ i ] = code;
    }
  }
  sei_read_uvlc( pDecodedMessageOutputStream, code, "bp_cpb_cnt_minus1" ); sei->bpCpbCnt = code + 1;
  if (sei->bpMaxSubLayers - 1 > 0)
  {
    sei_read_flag(pDecodedMessageOutputStream, code, "bp_sublayer_initial_cpb_removal_delay_present_flag");
    sei->sublayerInitialCpbRemovalDelayPresentFlag = code;
  }
  else
  {
    sei->sublayerInitialCpbRemovalDelayPresentFlag = false;
  }
  for (i = (sei->sublayerInitialCpbRemovalDelayPresentFlag ? 0 : sei->bpMaxSubLayers - 1); i < sei->bpMaxSubLayers; i++)
  {
    for( nalOrVcl = 0; nalOrVcl < 2; nalOrVcl ++ )
    {
      if( ( ( nalOrVcl == 0 ) && ( sei->bpNalCpbParamsPresentFlag ) ) ||
         ( ( nalOrVcl == 1 ) && ( sei->bpVclCpbParamsPresentFlag ) ) )
      {
        for( int j = 0; j < ( sei->bpCpbCnt ); j ++ )
        {
          sei_read_code( pDecodedMessageOutputStream, sei->initialCpbRemovalDelayLength, code, nalOrVcl ? "vcl_initial_cpb_removal_delay[i][j]" : "nal_initial_cpb_removal_delay[i][j]" );
          sei->initialCpbRemovalDelay[i][j][nalOrVcl] = code;
          sei_read_code( pDecodedMessageOutputStream, sei->initialCpbRemovalDelayLength, code, nalOrVcl ? "vcl_initial_cpb_removal_offset[i][j]" : "nal_initial_cpb_removal_offset[i][j]" );
          sei->initialCpbRemovalDelay[i][j][nalOrVcl] = code;
        }
      }
    }
  }
  if (sei->bpMaxSubLayers-1 > 0)
  {
    sei_read_flag(pDecodedMessageOutputStream, code, "bp_sublayer_dpb_output_offsets_present_flag");
    sei->sublayerDpbOutputOffsetsPresentFlag = code;
  }
  else
  {
    sei->sublayerDpbOutputOffsetsPresentFlag = false;
  }
  if(sei->sublayerDpbOutputOffsetsPresentFlag)
  {
    for(int i = 0; i < sei->bpMaxSubLayers - 1; i++)
    {
      sei_read_uvlc( pDecodedMessageOutputStream, code, "dpb_output_tid_offset[i]" );
      sei->dpbOutputTidOffset[i] = code;
    }
    sei->dpbOutputTidOffset[sei->bpMaxSubLayers-1] = 0;
  }
  sei_read_flag(pDecodedMessageOutputStream, code, "bp_alt_cpb_params_present_flag");
  sei->altCpbParamsPresentFlag = code;
  if (sei->altCpbParamsPresentFlag)
  {
    sei_read_flag(pDecodedMessageOutputStream, code, "use_alt_cpb_params_flag"); sei->useAltCpbParamsFlag = code;
  }

}

void SEIReader::xParseSEIPictureTiming(vvdecSEI* s, uint32_t payloadSize, const uint32_t temporalId, const vvdecSEIBufferingPeriod& bp, std::ostream *pDecodedMessageOutputStream)
{
  CHECK_FATAL( !s || s->payload == NULL, "allocation error in vvdecSEIPictureTiming" );

  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);

  vvdecSEIPictureTiming* sei =(vvdecSEIPictureTiming*)s->payload;
  ::memset(sei, 0, sizeof(vvdecSEIPictureTiming));

  uint32_t symbol;
  sei_read_code( pDecodedMessageOutputStream, bp.cpbRemovalDelayLength, symbol, "pt_cpb_removal_delay_minus1[bp_max_sub_layers_minus1]" );
  sei->auCpbRemovalDelay[bp.bpMaxSubLayers - 1] = symbol + 1;
  for (int i = temporalId; i < bp.bpMaxSubLayers - 1; i++)
  {
    sei_read_flag(pDecodedMessageOutputStream, symbol, "pt_sub_layer_delays_present_flag[i]");
    sei->ptSubLayerDelaysPresentFlag[i] = (symbol == 1);
    if (sei->ptSubLayerDelaysPresentFlag[i])
    {
      if (bp.cpbRemovalDelayDeltasPresentFlag)
      {
        sei_read_flag(pDecodedMessageOutputStream, symbol, "pt_cpb_removal_delay_delta_enabled_flag[i]");
        sei->cpbRemovalDelayDeltaEnabledFlag[i] = (symbol == 1);
      }
      else
      {
        sei->cpbRemovalDelayDeltaEnabledFlag[i] = false;
      }
      if (sei->cpbRemovalDelayDeltaEnabledFlag[i])
      {
        if ((bp.numCpbRemovalDelayDeltas - 1) > 0)
        {
          sei_read_code(pDecodedMessageOutputStream, (int)ceil(log2(bp.numCpbRemovalDelayDeltas)), symbol, "pt_cpb_removal_delay_delta_idx[i]");
          sei->cpbRemovalDelayDeltaIdx[i] = symbol;
        }
        else
        {
          sei->cpbRemovalDelayDeltaIdx[i] = 0;
        }
      }
      else
      {
        sei_read_code(pDecodedMessageOutputStream, bp.cpbRemovalDelayLength, symbol, "pt_cpb_removal_delay_minus1[i]");
        sei->auCpbRemovalDelay[i] = symbol + 1;
      }
    }
  }
  sei_read_code(pDecodedMessageOutputStream, bp.dpbOutputDelayLength, symbol, "pt_dpb_output_delay");
  sei->picDpbOutputDelay = symbol;

  if( bp.altCpbParamsPresentFlag )
  {
    sei_read_flag( pDecodedMessageOutputStream, symbol, "cpb_alt_timing_info_present_flag" ); sei->cpbAltTimingInfoPresentFlag = symbol;
    if( sei->cpbAltTimingInfoPresentFlag )
    {
      if (bp.bpNalCpbParamsPresentFlag)
      {
        for (int i = (bp.sublayerInitialCpbRemovalDelayPresentFlag ? 0 : bp.bpMaxSubLayers - 1);
             i <= bp.bpMaxSubLayers - 1; ++i)
        {
          for (int j = 0; j < bp.bpCpbCnt; j++)
          {
            sei_read_code(pDecodedMessageOutputStream, bp.initialCpbRemovalDelayLength, symbol,
                          "nal_cpb_alt_initial_cpb_removal_delay_delta[ i ][ j ]");
            sei->nalCpbAltInitialRemovalDelayDelta[i][j] = symbol;
            sei_read_code(pDecodedMessageOutputStream, bp.initialCpbRemovalDelayLength, symbol,
                          "nal_cpb_alt_initial_cpb_removal_offset_delta[ i ][ j ]");
            sei->nalCpbAltInitialRemovalOffsetDelta[i][j] = symbol;
          }
          sei_read_code(pDecodedMessageOutputStream, bp.initialCpbRemovalDelayLength, sei->nalCpbDelayOffset[i],
                        "nal_cpb_delay_offset[ i ]");
          sei_read_code(pDecodedMessageOutputStream, bp.initialCpbRemovalDelayLength, sei->nalDpbDelayOffset[i],
                        "nal_dpb_delay_offset[ i ]");
        }
      }

      if (bp.bpVclCpbParamsPresentFlag)
      {
        for (int i = (bp.sublayerInitialCpbRemovalDelayPresentFlag ? 0 : bp.bpMaxSubLayers - 1);
             i <= bp.bpMaxSubLayers - 1; ++i)
        {
          for (int j = 0; j < bp.bpCpbCnt; j++)
          {
            sei_read_code(pDecodedMessageOutputStream, bp.initialCpbRemovalDelayLength, symbol,
                          "vcl_cpb_alt_initial_cpb_removal_delay_delta[ i ][ j ]");
            sei->vclCpbAltInitialRemovalDelayDelta[i][j] = symbol;
            sei_read_code(pDecodedMessageOutputStream, bp.initialCpbRemovalDelayLength, symbol,
                          "vcl_cpb_alt_initial_cpb_removal_offset_delta[ i ][ j ]");
            sei->vclCpbAltInitialRemovalOffsetDelta[i][j] = symbol;
          }
          sei_read_code(pDecodedMessageOutputStream, bp.initialCpbRemovalDelayLength, sei->vclCpbDelayOffset[i],
                        "vcl_cpb_delay_offset[ i ]");
          sei_read_code(pDecodedMessageOutputStream, bp.initialCpbRemovalDelayLength, sei->vclDpbDelayOffset[i],
                        "vcl_dpb_delay_offset[ i ]");
        }
      }
    }
  }
  else
  {
    sei->cpbAltTimingInfoPresentFlag = false;
  }

  if ( bp.bpDecodingUnitHrdParamsPresentFlag && bp.decodingUnitDpbDuParamsInPicTimingSeiFlag )
  {
    sei_read_code( pDecodedMessageOutputStream, bp.dpbOutputDelayDuLength, symbol, "pic_dpb_output_du_delay" );
    sei->picDpbOutputDuDelay = symbol;
  }
  if( bp.bpDecodingUnitHrdParamsPresentFlag && bp.decodingUnitCpbParamsInPicTimingSeiFlag )
  {
    sei_read_uvlc( pDecodedMessageOutputStream, symbol, "nudecoding_units_minus1" );
    sei->numDecodingUnits = symbol+1;

    if (sei->numDecodingUnits > 1)
    {
    sei_read_flag( pDecodedMessageOutputStream, symbol, "du_common_cpb_removal_delay_flag" );
    sei->duCommonCpbRemovalDelayFlag = symbol;
    if( sei->duCommonCpbRemovalDelayFlag )
    {
      for( int i = temporalId; i < bp.bpMaxSubLayers - 1; i ++ )
      {
        if( sei->ptSubLayerDelaysPresentFlag[i] )
        {
          sei_read_code( pDecodedMessageOutputStream, bp.duCpbRemovalDelayIncrementLength, symbol, "du_common_cpb_removal_delay_increment_minus1[i]" );
          sei->duCommonCpbRemovalDelay[i] = symbol+1;
        }
      }
    }
    for( uint32_t i = 0; i < sei->numDecodingUnits; i ++ )
    {
      sei_read_uvlc( pDecodedMessageOutputStream, symbol, "nunalus_in_du_minus1[i]" );
      sei->numNalusInDu[i] = symbol+1;
      if( !sei->duCommonCpbRemovalDelayFlag && i < (sei->numDecodingUnits-1) )
      {
        for( int j = temporalId; j < bp.bpMaxSubLayers - 1; j ++ )
        {
          if( sei->ptSubLayerDelaysPresentFlag[j] )
          {
            sei_read_code( pDecodedMessageOutputStream, bp.duCpbRemovalDelayIncrementLength, symbol, "du_cpb_removal_delay_increment_minus1[i][j]" );
            sei->duCpbRemovalDelay[i * bp.bpMaxSubLayers + j] = symbol+1;
          }
        }
      }
    }
    }
    else
    {
      sei->duCommonCpbRemovalDelayFlag = 0;
    }
  }
  sei_read_code( pDecodedMessageOutputStream, 8, symbol, "pt_display_elemental_periods_minus1" );
  sei->ptDisplayElementalPeriods = symbol+1;
}

void SEIReader::xParseSEIFrameFieldinfo(vvdecSEI* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  CHECK_FATAL( !s || s->payload == NULL, "allocation error in vvdecSEIFrameFieldInfo" );

  uint32_t symbol;
  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);

  vvdecSEIFrameFieldInfo* sei =(vvdecSEIFrameFieldInfo*)s->payload;
  ::memset(sei, 0, sizeof(vvdecSEIFrameFieldInfo));

  sei_read_flag( pDecodedMessageOutputStream, symbol,      "field_pic_flag" );
  sei->fieldPicFlag= symbol;
  if (sei->fieldPicFlag)
  {
    sei_read_flag( pDecodedMessageOutputStream, symbol,    "bottofield_flag" );
    sei->bottomFieldFlag = symbol;
    sei_read_flag( pDecodedMessageOutputStream, symbol,    "pairing_indicated_flag" );
    sei->pairingIndicatedFlag = symbol;
    if (sei->pairingIndicatedFlag)
    {
      sei_read_flag( pDecodedMessageOutputStream, symbol,  "paired_with_next_field_flag" );
      sei->pairedWithNextFieldFlag = symbol;
    }
  }
  else
  {
    sei_read_flag( pDecodedMessageOutputStream, symbol,    "display_fields_froframe_flag" );
    sei->displayFieldsFromFrameFlag = symbol;
    if (sei->displayFieldsFromFrameFlag)
    {
      sei_read_flag( pDecodedMessageOutputStream, symbol,  "display_fields_froframe_flag" );
      sei->topFieldFirstFlag = symbol;
    }
    sei_read_code( pDecodedMessageOutputStream, 8, symbol, "ffi_display_elemental_periods_minus1" );
    sei->displayElementalPeriods = symbol+1;
  }
  sei_read_code( pDecodedMessageOutputStream, 2, symbol,   "source_scan_type" );
  sei->sourceScanType = symbol;
  sei_read_flag( pDecodedMessageOutputStream, symbol,      "duplicate_flag" );
  sei->duplicateFlag = symbol;
}

void SEIReader::xParseSEIDependentRAPIndication( vvdecSEI* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream )
{
  CHECK_FATAL( !s || s->payload == NULL, "allocation error in vvdecSeidependent_rap_indication_t" );

  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);
}

void SEIReader::xParseSEIFramePacking(vvdecSEI* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;
  CHECK_FATAL( !s || s->payload == NULL, "allocation error in vvdecSEIFramePacking" );

  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);

  vvdecSEIFramePacking* sei =(vvdecSEIFramePacking*)s->payload;
  ::memset(sei, 0, sizeof(vvdecSEIFramePacking));

  sei_read_uvlc( pDecodedMessageOutputStream, val, "frame_packing_arrangement_id" );                 sei->arrangementId = val;
  sei_read_flag( pDecodedMessageOutputStream, val, "frame_packing_arrangement_cancel_flag" );        sei->arrangementCancelFlag = val;

  if( !sei->arrangementCancelFlag )
  {
    sei_read_code( pDecodedMessageOutputStream, 7, val, "frame_packing_arrangement_type" );          sei->arrangementType = val;
    CHECK( ( sei->arrangementType <= 2 ) || ( sei->arrangementType >= 6 ), "Invalid arrangement type" );

    sei_read_flag( pDecodedMessageOutputStream, val, "quincunx_sampling_flag" );                     sei->quincunxSamplingFlag = val;

    sei_read_code( pDecodedMessageOutputStream, 6, val, "content_interpretation_type" );             sei->contentInterpretationType = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "spatial_flipping_flag" );                      sei->spatialFlippingFlag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "frame0_flipped_flag" );                        sei->frame0FlippedFlag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "field_views_flag" );                           sei->fieldViewsFlag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "current_frame_is_frame0_flag" );               sei->currentFrameIsFrame0Flag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "frame0_self_contained_flag" );                 sei->frame0SelfContainedFlag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "frame1_self_contained_flag" );                 sei->frame1SelfContainedFlag = val;

    if ( sei->quincunxSamplingFlag == 0 && sei->arrangementType != 5)
    {
      sei_read_code( pDecodedMessageOutputStream, 4, val, "frame0_grid_position_x" );                sei->frame0GridPositionX = val;
      sei_read_code( pDecodedMessageOutputStream, 4, val, "frame0_grid_position_y" );                sei->frame0GridPositionY = val;
      sei_read_code( pDecodedMessageOutputStream, 4, val, "frame1_grid_position_x" );                sei->frame1GridPositionX = val;
      sei_read_code( pDecodedMessageOutputStream, 4, val, "frame1_grid_position_y" );                sei->frame1GridPositionY = val;
    }

    sei_read_code( pDecodedMessageOutputStream, 8, val, "frame_packing_arrangement_reserved_byte" ); sei->arrangementReservedByte = val;
    sei_read_flag( pDecodedMessageOutputStream, val,  "frame_packing_arrangement_persistence_flag" );sei->arrangementPersistenceFlag = (val != 0);
  }
  sei_read_flag( pDecodedMessageOutputStream, val, "upsampled_aspect_ratio_flag" );                  sei->upsampledAspectRatio = val;
}

void SEIReader::xParseSEIParameterSetsInclusionIndication(vvdecSEI* s, uint32_t payloadSize, std::ostream* pDecodedMessageOutputStream)
{
  uint32_t val;
  CHECK_FATAL( !s || s->payload == NULL, "allocation error in vvdecSEIParameterSetsInclusionIndication" );

  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);

  vvdecSEIParameterSetsInclusionIndication* sei =(vvdecSEIParameterSetsInclusionIndication*)s->payload;

  sei_read_flag( pDecodedMessageOutputStream, val, "self_contained_clvs_flag" ); sei->selfContainedClvsFlag = val;
}

void SEIReader::xParseSEIMasteringDisplayColourVolume( vvdecSEI* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t code;
  CHECK_FATAL( !s || s->payload == NULL, "allocation error in vvdecSEIMasteringDisplayColourVolume" );

  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);
  
  vvdecSEIMasteringDisplayColourVolume* sei =(vvdecSEIMasteringDisplayColourVolume*)s->payload;
  ::memset(sei, 0, sizeof(vvdecSEIMasteringDisplayColourVolume));

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
void SEIReader::xParseSEIAlternativeTransferCharacteristics(vvdecSEI* s, uint32_t payloadSize, std::ostream* pDecodedMessageOutputStream)
{
  uint32_t code;
  CHECK_FATAL( !s || s->payload == NULL, "allocation error in vvdecSEIAlternativeTransferCharacteristics" );

  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);
  
  vvdecSEIAlternativeTransferCharacteristics* sei =(vvdecSEIAlternativeTransferCharacteristics*)s->payload;

  sei_read_code(pDecodedMessageOutputStream, 8, code, "preferred_transfer_characteristics"); sei->preferred_transfer_characteristics = code;
}
#endif
void SEIReader::xParseSEIUserDataRegistered(vvdecSEI* sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t code;
  CHECK_FATAL( !sei || sei->payload == NULL, "allocation error in vvdecSEIUserDataRegistered" );

  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  vvdecSEIUserDataRegistered t;

  CHECK(payloadSize == 0, "no payload" );
  sei_read_code(pDecodedMessageOutputStream, 8, code, "itu_t_t35_country_code"); payloadSize--;
  if (code == 255)
  {
    CHECK(payloadSize == 0, "no payload" );
    sei_read_code(pDecodedMessageOutputStream, 8, code, "itu_t_t35_country_code_extension_byte"); payloadSize--;
    code += 255;
  }
  t.ituCountryCode = code;
  t.userDataLength = payloadSize;

  if( !payloadSize )
  {
    t.userData = 0;
    ::memcpy(sei->payload, &t, sizeof(vvdecSEIUserDataRegistered));
    return;
  }

  if( sei->payload )
      free( sei->payload );

  SEI_internal::allocSEIPayload( sei, sizeof( vvdecSEIUserDataRegistered ) + sizeof( uint8_t )*(t.userDataLength + 1) );
  CHECK_FATAL( sei->payload == NULL, "allocation error in vvdecSEIUserDataRegistered" );

  vvdecSEIUserDataRegistered* target = (vvdecSEIUserDataRegistered*)sei->payload;
  target->ituCountryCode = t.ituCountryCode;
  target->userDataLength = t.userDataLength;
  target->userData = (uint8_t*)((uint8_t*)sei->payload + sizeof( vvdecSEIUserDataRegistered ));

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
void SEIReader::xParseSEIuserDataUnregistered(vvdecSEI* sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  CHECK(payloadSize < 16, "Payload too small");
  CHECK_FATAL( !sei || sei->payload == NULL, "allocation error in vvdecSEIUserDataUnregistered" );

  uint32_t val;

  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  vvdecSEIUserDataUnregistered t;

  for (uint32_t i = 0; i < 16; i++)
  {
    sei_read_code( pDecodedMessageOutputStream, 8, val, "uuid_iso_iec_11578"); t.uuid_iso_iec_11578[i] = val;
  }

  t.userDataLength = payloadSize - 16;
  if (!t.userDataLength)
  {
    t.userData = 0;
    ::memcpy(sei->payload, &t, sizeof(vvdecSEIUserDataUnregistered));
    return;
  }

  if( sei->payload )
      free( sei->payload );

  SEI_internal::allocSEIPayload( sei,  sizeof( vvdecSEIUserDataUnregistered ) + sizeof( uint8_t )*(t.userDataLength + 1) );
  CHECK_FATAL( sei->payload == NULL, "allocation error in vvdecSEIUserDataUnregistered" );

  vvdecSEIUserDataUnregistered* target = (vvdecSEIUserDataUnregistered*)sei->payload;
  target->userDataLength = t.userDataLength;
  target->userData = (uint8_t*)((uint8_t*)sei->payload + sizeof( vvdecSEIUserDataUnregistered ));

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

void SEIReader::xParseSEIFilmGrainCharacteristics(vvdecSEI* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t code;
  CHECK_FATAL( !s || s->payload == NULL, "allocation error in vvdecSEIFilmGrainCharacteristics" );

  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);

  vvdecSEIFilmGrainCharacteristics* sei =(vvdecSEIFilmGrainCharacteristics*)s->payload;
  ::memset(sei, 0, sizeof(vvdecSEIFilmGrainCharacteristics));

  sei_read_flag(pDecodedMessageOutputStream, code, "filgrain_characteristics_cancel_flag");     sei->filmGrainCharacteristicsCancelFlag = code != 0;
  if (!sei->filmGrainCharacteristicsCancelFlag)
  {
    sei_read_code(pDecodedMessageOutputStream, 2, code, "filgrain_model_id");                   sei->filmGrainModelId = code;
    sei_read_flag(pDecodedMessageOutputStream, code, "separate_colour_description_present_flag"); sei->separateColourDescriptionPresentFlag = code != 0;
    if (sei->separateColourDescriptionPresentFlag)
    {
      sei_read_code(pDecodedMessageOutputStream, 3, code, "filgrain_bit_depth_luma_minus8");    sei->filmGrainBitDepthLuma = code+8;
      sei_read_code(pDecodedMessageOutputStream, 3, code, "filgrain_bit_depth_chroma_minus8");  sei->filmGrainBitDepthChroma = code+8;
      sei_read_flag(pDecodedMessageOutputStream, code, "filgrain_full_range_flag");             sei->filmGrainFullRangeFlag = code != 0;
      sei_read_code(pDecodedMessageOutputStream, 8, code, "filgrain_colour_primaries");         sei->filmGrainColourPrimaries = code;
      sei_read_code(pDecodedMessageOutputStream, 8, code, "filgrain_transfer_characteristics"); sei->filmGrainTransferCharacteristics = code;
      sei_read_code(pDecodedMessageOutputStream, 8, code, "filgrain_matrix_coeffs");            sei->filmGrainMatrixCoeffs = code;
    }
    sei_read_code(pDecodedMessageOutputStream, 2, code, "blending_mode_id");                      sei->blendingModeId = code;
    sei_read_code(pDecodedMessageOutputStream, 4, code, "log2_scale_factor");                     sei->log2ScaleFactor = code;
    for (int c = 0; c<3; c++)
    {
      sei_read_flag(pDecodedMessageOutputStream, code, "comp_model_present_flag[c]");             sei->compModel[c].presentFlag = code != 0;
    }
    for (int c = 0; c<3; c++)
    {
      vvdecCompModel &cm = sei->compModel[c];
      if (cm.presentFlag)
      {
        sei_read_code(pDecodedMessageOutputStream, 8, code, "nuintensity_intervals_minus1[c]"); cm.numIntensityIntervals = code + 1;
        sei_read_code(pDecodedMessageOutputStream, 3, code, "numodel_values_minus1[c]");        cm.numModelValues = code + 1;

        CHECK ( cm.numIntensityIntervals > 256, "nuintensity_intervals_minus1[c] out of range" );
        CHECK ( cm.numModelValues > 6, "numodel_values_minus1[c] out of range" );

        for (uint32_t interval = 0; interval < cm.numIntensityIntervals; interval++)
        {
          vvdecCompModelIntensityValues &cmiv = cm.intensityValues[interval];
          sei_read_code(pDecodedMessageOutputStream, 8, code, "intensity_interval_lower_bound[c][i]"); cmiv.intensityIntervalLowerBound = code;
          sei_read_code(pDecodedMessageOutputStream, 8, code, "intensity_interval_upper_bound[c][i]"); cmiv.intensityIntervalUpperBound = code;
          for (uint32_t j = 0; j<cm.numModelValues; j++)
          {
            sei_read_svlc(pDecodedMessageOutputStream, cmiv.compModelValue[j], "comp_model_value[c][i]");
          }
        }
      }
    } // for c
    sei_read_flag(pDecodedMessageOutputStream, code, "filgrain_characteristics_persistence_flag"); sei->filmGrainCharacteristicsPersistenceFlag = code != 0;
  } // cancel flag
}

void SEIReader::xParseSEIContentLightLevelInfo(vvdecSEI* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t code;
  CHECK_FATAL( !s || s->payload == NULL, "allocation error in vvdecSEIContentLightLevelInfo" );

  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);

  vvdecSEIContentLightLevelInfo* sei =(vvdecSEIContentLightLevelInfo*)s->payload;
  ::memset(sei, 0, sizeof(vvdecSEIContentLightLevelInfo));

  sei_read_code(pDecodedMessageOutputStream, 16, code, "max_content_light_level");     sei->maxContentLightLevel    = code;
  sei_read_code(pDecodedMessageOutputStream, 16, code, "max_pic_average_light_level"); sei->maxPicAverageLightLevel = code;
}

void SEIReader::xParseSEIAmbientViewingEnvironment(vvdecSEI* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t code;
  CHECK_FATAL( !s || s->payload == NULL, "allocation error in vvdecSEIAmbientViewingEnvironment" );

  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);

  vvdecSEIAmbientViewingEnvironment* sei =(vvdecSEIAmbientViewingEnvironment*)s->payload;
  ::memset(sei, 0, sizeof(vvdecSEIAmbientViewingEnvironment));

  sei_read_code(pDecodedMessageOutputStream, 32, code, "ambient_illuminance"); sei->ambientIlluminance = code;
  sei_read_code(pDecodedMessageOutputStream, 16, code, "ambient_light_x");     sei->ambientLightX = (uint16_t)code;
  sei_read_code(pDecodedMessageOutputStream, 16, code, "ambient_light_y");     sei->ambientLightY = (uint16_t)code;
}

void SEIReader::xParseSEIContentColourVolume(vvdecSEI* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
  {
  int i;
  uint32_t val;
  CHECK_FATAL( !s || s->payload == NULL, "allocation error in vvdecSEIContentColourVolume" );

  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);

  vvdecSEIContentColourVolume* sei =(vvdecSEIContentColourVolume*)s->payload;
  ::memset(sei, 0, sizeof(vvdecSEIContentColourVolume));

  sei_read_flag(pDecodedMessageOutputStream, val, "ccv_cancel_flag");          sei->ccvCancelFlag = val;
  if (!sei->ccvCancelFlag)
    {
    int iVal;
    sei_read_flag(pDecodedMessageOutputStream, val, "ccv_persistence_flag");                   sei->ccvPersistenceFlag = val;
    sei_read_flag(pDecodedMessageOutputStream, val, "ccv_primaries_present_flag");             sei->ccvPrimariesPresentFlag = val;
    sei_read_flag(pDecodedMessageOutputStream, val, "ccv_min_luminance_value_present_flag");   sei->ccvMinLuminanceValuePresentFlag = val;
    sei_read_flag(pDecodedMessageOutputStream, val, "ccv_max_luminance_value_present_flag");   sei->ccvMaxLuminanceValuePresentFlag = val;
    sei_read_flag(pDecodedMessageOutputStream, val, "ccv_avg_luminance_value_present_flag");   sei->ccvAvgLuminanceValuePresentFlag = val;

    if (sei->ccvPrimariesPresentFlag)
    {
      for (i = 0; i < MAX_NUM_COMPONENT; i++)
    {
        sei_read_scode(pDecodedMessageOutputStream, 32, iVal, "ccv_primaries_x[i]");          sei->ccvPrimariesX[i] = iVal;
        sei_read_scode(pDecodedMessageOutputStream, 32, iVal, "ccv_primaries_y[i]");          sei->ccvPrimariesY[i] = iVal;
    }
  }
    if (sei->ccvMinLuminanceValuePresentFlag)
  {
      sei_read_code(pDecodedMessageOutputStream, 32, val, "ccv_min_luminance_value");   sei->ccvMinLuminanceValue = val;
    }
    if (sei->ccvMaxLuminanceValuePresentFlag)
    {
      sei_read_code(pDecodedMessageOutputStream, 32, val, "ccv_max_luminance_value");   sei->ccvMaxLuminanceValue = val;
    }
    if (sei->ccvAvgLuminanceValuePresentFlag)
    {
      sei_read_code(pDecodedMessageOutputStream, 32, val, "ccv_avg_luminance_value");   sei->ccvAvgLuminanceValue = val;
    }
  }
}
void SEIReader::xParseSEIEquirectangularProjection(vvdecSEI* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;
  CHECK_FATAL( !s || s->payload == NULL, "allocation error in vvdecSEIEquirectangularProjection" );

  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);

  vvdecSEIEquirectangularProjection* sei =(vvdecSEIEquirectangularProjection*)s->payload;
  ::memset(sei, 0, sizeof(vvdecSEIEquirectangularProjection));

  sei_read_flag( pDecodedMessageOutputStream, val,       "erp_cancel_flag" );              sei->erpCancelFlag = val;
  if( !sei->erpCancelFlag )
  {
    sei_read_flag( pDecodedMessageOutputStream, val,      "erp_persistence_flag"    );     sei->erpPersistenceFlag   = val;
    sei_read_flag( pDecodedMessageOutputStream, val,      "erp_guard_band_flag"     );     sei->erpGuardBandFlag     = val;
    sei_read_code( pDecodedMessageOutputStream, 2, val,   "erp_reserved_zero_2bits" );
    if ( sei->erpGuardBandFlag == 1)
    {
      sei_read_code( pDecodedMessageOutputStream, 3, val,     "erp_guard_band_type"       );   sei->erpGuardBandType  = val;
      sei_read_code( pDecodedMessageOutputStream, 8, val,     "erp_left_guard_band_width" );   sei->erpLeftGuardBandWidth = val;
      sei_read_code( pDecodedMessageOutputStream, 8, val,     "erp_right_guard_band_width");   sei->erpRightGuardBandWidth = val;
    }
  }
}

void SEIReader::xParseSEISphereRotation(vvdecSEI* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;
  int  sval;
  CHECK_FATAL( !s || s->payload == NULL, "allocation error in vvdecSEISphereRotation" );
  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);

  vvdecSEISphereRotation* sei =(vvdecSEISphereRotation*)s->payload;
  ::memset(sei, 0, sizeof(vvdecSEISphereRotation));

  sei_read_flag( pDecodedMessageOutputStream, val,       "sphere_rotation_cancel_flag" );              sei->sphereRotationCancelFlag = val;
  if( !sei->sphereRotationCancelFlag )
    {
    sei_read_flag ( pDecodedMessageOutputStream,      val,   "sphere_rotation_persistence_flag"    );     sei->sphereRotationPersistenceFlag = val;
    sei_read_code ( pDecodedMessageOutputStream, 6,   val,   "sphere_rotation_reserved_zero_6bits" );
    sei_read_scode( pDecodedMessageOutputStream, 32, sval,   "sphere_rotation_yaw"                 );     sei->sphereRotationYaw = sval;
    sei_read_scode( pDecodedMessageOutputStream, 32, sval,   "sphere_rotation_pitch"               );     sei->sphereRotationPitch = sval;
    sei_read_scode( pDecodedMessageOutputStream, 32, sval,   "sphere_rotation_roll"                );     sei->sphereRotationRoll = sval;
  }
}

void SEIReader::xParseSEIOmniViewport(vvdecSEI* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t code;
  int  scode;
  CHECK_FATAL( !s || s->payload == NULL, "allocation error in vvdecSEIOmniViewport" );
  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);

  vvdecSEIOmniViewport* sei =(vvdecSEIOmniViewport*)s->payload;
  ::memset(sei, 0, sizeof(vvdecSEIOmniViewport));

  sei_read_code( pDecodedMessageOutputStream, 10, code, "omni_viewport_id"          ); sei->omniViewportId         = code;
  sei_read_flag( pDecodedMessageOutputStream,     code, "omni_viewport_cancel_flag" ); sei->omniViewportCancelFlag = code;

  if (!sei->omniViewportCancelFlag)
  {
    sei_read_flag( pDecodedMessageOutputStream,    code, "omni_viewport_persistence_flag" );  sei->omniViewportPersistenceFlag = code;
    sei_read_code( pDecodedMessageOutputStream, 4, code, "omni_viewport_cnt_minus1"       );  sei->omniViewportCnt =  code+1;
    CHECK( sei->omniViewportCnt > 16, "omni_viewport_cnt_minus1 must be < 16 in vvdecSEIDecodedPictureHash" );

    for(uint32_t region=0; region < sei->omniViewportCnt; region++)
    {
      vvdecOmniViewportRegion &viewport = sei->omniViewportRegions[region];
      sei_read_scode( pDecodedMessageOutputStream, 32, scode, "omni_viewport_azimuth_centre"   );   viewport.azimuthCentre   = scode;
      sei_read_scode( pDecodedMessageOutputStream, 32, scode, "omni_viewport_elevation_centre" );   viewport.elevationCentre = scode;
      sei_read_scode( pDecodedMessageOutputStream, 32, scode, "omni_viewport_tilt_centre"      );   viewport.tiltCentre      = code;
      sei_read_code( pDecodedMessageOutputStream,  32, code , "omni_viewport_hor_range"        );   viewport.horRange        = code;
      sei_read_code( pDecodedMessageOutputStream,  32, code , "omni_viewport_ver_range"        );   viewport.verRange        = code;
    }
  }
  else
  {
    sei->omniViewportPersistenceFlag=false;
  }
}

void SEIReader::xParseSEIRegionWisePacking(vvdecSEI* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;
  CHECK_FATAL( !s || s->payload == NULL, "allocation error in vvdecSEIRegionWisePacking" );
  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);

  vvdecSEIRegionWisePacking* sei =(vvdecSEIRegionWisePacking*)s->payload;
  ::memset(sei, 0, sizeof(vvdecSEIRegionWisePacking));

  if (!sei->rwpCancelFlag)
    {
    sei_read_flag( pDecodedMessageOutputStream,           val,    "rwp_persistence_flag" );                 sei->rwpPersistenceFlag = val;
    sei_read_flag( pDecodedMessageOutputStream,           val,    "constituent_picture_matching_flag" );    sei->constituentPictureMatchingFlag = val;
    sei_read_code( pDecodedMessageOutputStream,       5,  val,    "rwp_reserved_zero_5bits" );
    sei_read_code( pDecodedMessageOutputStream,       8,  val,    "nupacked_regions" );                   sei->numPackedRegions    = val;
    sei_read_code( pDecodedMessageOutputStream,       32, val,    "proj_picture_width" );                   sei->projPictureWidth    = val;
    sei_read_code( pDecodedMessageOutputStream,       32, val,    "proj_picture_height" );                  sei->projPictureHeight   = val;
    sei_read_code( pDecodedMessageOutputStream,       16, val,    "packed_picture_width" );                 sei->packedPictureWidth  = val;
    sei_read_code( pDecodedMessageOutputStream,       16, val,    "packed_picture_height" );                sei->packedPictureHeight = val;

    for( int i=0; i < sei->numPackedRegions; i++ )
    {
      sei_read_code( pDecodedMessageOutputStream,     4,  val,    "rwp_reserved_zero_4bits" );
      sei_read_code( pDecodedMessageOutputStream,     3,  val,    "rwp_tTransfortype" );                  sei->rwpTransformType[i]   = val;
      sei_read_flag( pDecodedMessageOutputStream,         val,    "rwp_guard_band_flag" );                  sei->rwpGuardBandFlag[i]   = val;
      sei_read_code( pDecodedMessageOutputStream,     32, val,    "proj_region_width" );                    sei->projRegionWidth[i]    = val;
      sei_read_code( pDecodedMessageOutputStream,     32, val,    "proj_region_height" );                   sei->projRegionHeight[i]   = val;
      sei_read_code( pDecodedMessageOutputStream,     32, val,    "rwp_proj_regionTop" );                   sei->rwpProjRegionTop[i]   = val;
      sei_read_code( pDecodedMessageOutputStream,     32, val,    "proj_region_left" );                     sei->projRegionLeft[i]     = val;
      sei_read_code( pDecodedMessageOutputStream,     16, val,    "packed_region_width" );                  sei->packedRegionWidth[i]  = val;
      sei_read_code( pDecodedMessageOutputStream,     16, val,    "packed_region_height" );                 sei->packedRegionHeight[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     16, val,    "packed_region_top" );                    sei->packedRegionTop[i]    = val;
      sei_read_code( pDecodedMessageOutputStream,     16, val,    "packed_region_left" );                   sei->packedRegionLeft[i]   = val;
      if( sei->rwpGuardBandFlag[i] )
        {
        sei_read_code( pDecodedMessageOutputStream,   8,  val,    "rwp_left_guard_band_width" );            sei->rwpLeftGuardBandWidth[i]     = val;
        sei_read_code( pDecodedMessageOutputStream,   8,  val,    "rwp_right_guard_band_width" );           sei->rwpRightGuardBandWidth[i]    = val;
        sei_read_code( pDecodedMessageOutputStream,   8,  val,    "rwp_top_guard_band_height" );            sei->rwpTopGuardBandHeight[i]     = val;
        sei_read_code( pDecodedMessageOutputStream,   8,  val,    "rwp_bottoguard_band_height" );         sei->rwpBottomGuardBandHeight[i]  = val;
        sei_read_flag( pDecodedMessageOutputStream,       val,    "rwp_guard_band_not_used_forPred_flag" ); sei->rwpGuardBandNotUsedForPredFlag[i] = val;
        for( int j=0; j < 4; j++ )
          {
          sei_read_code( pDecodedMessageOutputStream, 3,  val,     "rwp_guard_band_type" ); sei->rwpGuardBandType[i*4 + j] = val;
        }
        sei_read_code( pDecodedMessageOutputStream,   3,  val,    "rwp_guard_band_reserved_zero_3bits" );
      }
    }
  }
}

void SEIReader::xParseSEIGeneralizedCubemapProjection(vvdecSEI* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;

  CHECK_FATAL( !s || s->payload == NULL, "allocation error in vvdecSEIGeneralizedCubemapProjection" );
  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);
  
  vvdecSEIGeneralizedCubemapProjection* sei =(vvdecSEIGeneralizedCubemapProjection*)s->payload;
  ::memset(sei, 0, sizeof(vvdecSEIGeneralizedCubemapProjection));

  sei_read_flag( pDecodedMessageOutputStream,          val,    "gcmp_cancel_flag" );                      sei->gcmpCancelFlag = val;
  if (!sei->gcmpCancelFlag)
  {
    sei_read_flag( pDecodedMessageOutputStream,        val,    "gcmp_persistence_flag"    );              sei->gcmpPersistenceFlag = val;
    sei_read_code( pDecodedMessageOutputStream,     3, val,    "gcmp_packing_type" );                     sei->gcmpPackingType = val;
    sei_read_code( pDecodedMessageOutputStream,     2, val,    "gcmp_mapping_function_type"     );        sei->gcmpMappingFunctionType = val;

    int numFace = sei->gcmpPackingType == 4 || sei->gcmpPackingType == 5 ? 5 : 6;

    for (int i = 0; i < numFace; i++)
    {
      sei_read_code( pDecodedMessageOutputStream,   3, val,    "gcmp_face_index" );                       sei->gcmpFaceIndex[i] = val;
      sei_read_code( pDecodedMessageOutputStream,   2, val,    "gcmp_face_rotation" );                    sei->gcmpFaceRotation[i] = val;
      if (sei->gcmpMappingFunctionType == 2)
      {
        sei_read_code( pDecodedMessageOutputStream, 7, val,    "gcmp_function_coeff_u" );                 sei->gcmpFunctionCoeffU[i] = val;
        sei_read_flag( pDecodedMessageOutputStream,    val,    "gcmp_function_u_affected_by_v_flag"    ); sei->gcmpFunctionUAffectedByVFlag[i] = val;
        sei_read_code( pDecodedMessageOutputStream, 7, val,    "gcmp_function_coeff_v" );                 sei->gcmpFunctionCoeffV[i] = val;
        sei_read_flag( pDecodedMessageOutputStream,    val,    "gcmp_function_v_affected_by_u_flag"    ); sei->gcmpFunctionVAffectedByUFlag[i] = val;
      }
    }
    sei_read_flag( pDecodedMessageOutputStream,        val,    "gcmp_guard_band_flag" );                  sei->gcmpGuardBandFlag = val;
    if (sei->gcmpGuardBandFlag)
    {
      sei_read_code( pDecodedMessageOutputStream,   3, val,    "gcmp_guard_band_type" );                   sei->gcmpGuardBandType = val;
      sei_read_flag( pDecodedMessageOutputStream,      val,    "gcmp_guard_band_boundary_exterior_flag" ); sei->gcmpGuardBandBoundaryExteriorFlag = val;
      sei_read_code( pDecodedMessageOutputStream,   4, val,    "gcmp_guard_band_samples_minus1" );         sei->gcmpGuardBandSamples = val+1;
    }
  }
}

void SEIReader::xParseSEISubpictureLevelInfo(vvdecSEI* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  CHECK_FATAL( !s || s->payload == NULL, "allocation error in vvdecSEISubpictureLevelInfo" );
  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);
  vvdecSEISubpictureLevelInfo* sei =(vvdecSEISubpictureLevelInfo*)s->payload;
  ::memset(sei, 0, sizeof(vvdecSEISubpictureLevelInfo));

  uint32_t val;
  sei_read_code( pDecodedMessageOutputStream,   3,  val,    "sli_nuref_levels_minus1" );              sei->numRefLevels  = val + 1;
  CHECK( sei->numRefLevels > 6, "sli_nuref_levels_minus1 must be < 6 in vvdecSEISubpictureLevelInfo" );

  sei_read_flag( pDecodedMessageOutputStream,       val,    "sli_cbr_constraint_flag" );              sei->cbrConstraintFlag = val;
  sei_read_flag( pDecodedMessageOutputStream,       val,    "sli_explicit_fraction_present_flag" );   sei->explicitFractionPresentFlag = val;
  if (sei->explicitFractionPresentFlag)
  {
    sei_read_uvlc(pDecodedMessageOutputStream,      val,    "sli_nusubpics_minus1");                  sei->numSubpics = val + 1;
    CHECK( sei->numSubpics > 64, "sli_nusubpics_minus1 must be < 64 in vvdecSEISubpictureLevelInfo" );
    sei_read_code(pDecodedMessageOutputStream,  3,  val,    "sli_max_sublayers_minus1"  );            sei->sliMaxSublayers = val + 1;
    CHECK( sei->sliMaxSublayers > 6, "sli_max_sublayers_minus1 must be < 6 in vvdecSEISubpictureLevelInfo" );

    sei_read_flag(pDecodedMessageOutputStream,      val,    "sli_sublayer_info_present_flag");        sei->sliSublayerInfoPresentFlag = val;
    while (!isByteAligned())
    {
      sei_read_flag( pDecodedMessageOutputStream,   val,    "sli_alignment_zero_bit" );           CHECK (val != 0, "sli_alignment_zero_bit not equal to zero" );
    }
  }

  // sei parameters initialization
  for (int i = 0; i < sei->numRefLevels; i++)
  {
    for (int k = 0; k < sei->sliMaxSublayers; k++)
    {
      sei->refLevelIdc[i][k] = vvdecLevel::VVDEC_LEVEL15_5;
    }
  }
  if (sei->explicitFractionPresentFlag)
  {
    for (int i = 0; i < sei->numRefLevels; i++)
    {
      for (int j = 0; j < sei->numSubpics; j++)
      {
        for (int k = 0; k < sei->sliMaxSublayers; k++)
        {
          sei->refLevelFraction[i][j][k] = 0;
        }
      }
    }
  }

  // parsing
  for (int k = sei->sliSublayerInfoPresentFlag ? 0 : sei->sliMaxSublayers - 1; k < sei->sliMaxSublayers; k++)
  {
    for (int i = 0; i < sei->numRefLevels; i++)
    {
      sei_read_code(pDecodedMessageOutputStream, 8, val, "sli_non_subpic_layers_fraction[i][k]");    sei->nonSubpicLayersFraction[i][k] = (vvdecLevel) val;
      sei_read_code(pDecodedMessageOutputStream, 8, val, "sli_ref_level_idc[i][k]");                 sei->refLevelIdc[i][k] = (vvdecLevel) val;

      if (sei->explicitFractionPresentFlag)
      {
        for (int j = 0; j < sei->numSubpics; j++)
        {
          sei_read_code(pDecodedMessageOutputStream, 8, val, "sli_ref_level_fraction_minus1[i][j][k]");  sei->refLevelFraction[i][j][k] = val;
        }
      }
    }
  }

  // update the inference of refLevelIdc[][] and refLevelFraction[][][]
  if (!sei->sliSublayerInfoPresentFlag)
  {
    for (int k = sei->sliMaxSublayers - 2; k >= 0; k--)
    {
      for (int i = 0; i < sei->numRefLevels; i++)
      {
        sei->nonSubpicLayersFraction[i][k] = sei->nonSubpicLayersFraction[i][sei->sliMaxSublayers - 1];
        sei->refLevelIdc            [i][k] = sei->refLevelIdc            [i][sei->sliMaxSublayers - 1];
        if (sei->explicitFractionPresentFlag)
        {
          for (int j = 0; j < sei->numSubpics; j++)
          {
            sei->refLevelFraction[i][j][k] = sei->refLevelFraction[i][j][sei->sliMaxSublayers - 1];
          }
        }
      }
    }
  }
}

void SEIReader::xParseSEISampleAspectRatioInfo(vvdecSEI* s, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;
  CHECK_FATAL( !s || s->payload == NULL, "allocation error in vvdecSEISampleAspectRatioInfo" );
  output_sei_message_header(s, pDecodedMessageOutputStream, payloadSize);

  vvdecSEISampleAspectRatioInfo* sei =(vvdecSEISampleAspectRatioInfo*)s->payload;
  ::memset(sei, 0, sizeof(vvdecSEISampleAspectRatioInfo));

  sei_read_flag( pDecodedMessageOutputStream,           val,    "sari_cancel_flag" );                      sei->sariCancelFlag = val;
  if (!sei->sariCancelFlag)
  {
    sei_read_flag( pDecodedMessageOutputStream,         val,    "sari_persistence_flag" );                 sei->sariPersistenceFlag = val;
    sei_read_code( pDecodedMessageOutputStream,     8,  val,    "sari_aspect_ratio_idc" );                 sei->sariAspectRatioIdc = val;
    if (sei->sariAspectRatioIdc == 255)
    {
      sei_read_code( pDecodedMessageOutputStream,  16,  val,    "sari_sar_width" );                        sei->sariSarWidth = val;
      sei_read_code( pDecodedMessageOutputStream,  16,  val,    "sari_sar_height" );                       sei->sariSarHeight = val;
    }
  }
}

#if JVET_S0257_DUMP_360SEI_MESSAGE
void SeiCfgFileDump::write360SeiDump (std::string decoded360MessageFileName, SEIMessages& seis, const SPS* sps)
{
  if (360SEIMessageDumped)
  {
    return;
  }

  SEIMessages equirectangularProjectionSEIs = getSeisByType(seis, EQUIRECTANGULAR_PROJECTION);
  if (!equirectangularProjectionSEIs.empty())
  {
    SEIEquirectangularProjection* sei = (SEIEquirectangularProjection*)equirectangularProjectionSEIs.front();
    xDumpSEIEquirectangularProjection(*sei, sps, decoded360MessageFileName);
    360SEIMessageDumped = true;
  }
  else
  {
    SEIMessages generalizedCubemapProjectionSEIs = getSeisByType(seis, GENERALIZED_CUBEMAP_PROJECTION);
    if (!generalizedCubemapProjectionSEIs.empty())
    {
      SEIGeneralizedCubemapProjection* sei = (SEIGeneralizedCubemapProjection*)generalizedCubemapProjectionSEIs.front();
      xDumpSEIGeneralizedCubemapProjection(*sei, sps, decoded360MessageFileName);
      360SEIMessageDumped = true;
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
      if (sei.erpGuardBandFlag == 1)
      {
        fprintf(fp, "InputPERP                     : 1     # 0: original ERP input; 1: padded ERP input\n");
        fprintf(fp, "CodingPERP                    : 0     # 0: coding with original ERP size; 1: coding with padded ERP\n");
      }
      fclose(fp);
      360SEIMessageDumped = true;
    }
    else
    {
      msg( ERROR, "File %s could not be opened.\n", decoded360MessageFileName.c_str() );
    }
  }
}
void SeiCfgFileDump::xDumpSEIGeneralizedCubemapProjection  (SEIGeneralizedCubemapProjection &sei, const SPS* sps, std::string decoded360MessageFileName)
{
  if (!sei.gcmpCancelFlag)
  {
    int numFace = sei.gcmpPackingType == 4 || sei.gcmpPackingType == 5 ? 5 : 6;
    int packingTypeTable[6][2] = {{6, 1}, {3, 2}, {2, 3}, {1, 6}, {1, 5}, {5, 1}};
    int rotationTable[4] = {0, 90, 180, 270};
    std::string packingTypeStr = "";
    std::string gcmpsettingsStr = "";
    std::ostringstream oss;

    packingTypeStr += "SourceFPStructure                 : " + std::to_string(packingTypeTable[sei.gcmpPackingType][0]) + " " + std::to_string(packingTypeTable[sei.gcmpPackingType][1]);
    gcmpsettingsStr += "InputGCMPSettings                 : ";

    for (int i = 0; i < numFace; i++)
    {
      int rotation = rotationTable[sei.gcmpFaceRotation[i]];
      if (sei.gcmpFaceIndex[i] == 1)
      {
        rotation = (rotation + 270) % 360 + 360;
      }
      else if (sei.gcmpFaceIndex[i] == 2)
      {
        rotation = (rotation + 180) % 360 + 360;
      }
      else
      {
        rotation += 360;
      }
      if (i % packingTypeTable[sei.gcmpPackingType][1] == 0)
      {
        packingTypeStr += "   ";
      }
      packingTypeStr += std::to_string(sei.gcmpFaceIndex[i]) + " " + std::to_string(rotation) + " ";

      if (sei.gcmpMappingFunctionType == 2)
      {
        double a = ((int)sei.gcmpFunctionCoeffU[i] + 1) / 128.0;
        double b = ((int)sei.gcmpFunctionCoeffV[i] + 1) / 128.0;
        oss.str("");
        oss<<a;
        std::string a_str = oss.str();
        oss.str("");
        oss<<b;
        std::string b_str = oss.str();
        gcmpsettingsStr += a_str + " " + std::to_string(sei.gcmpFunctionUAffectedByVFlag[i]) + " " + b_str + " " + std::to_string(sei.gcmpFunctionVAffectedByUFlag[i]) + "   ";
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
        fprintf(fp, "InputGCMPMappingType              : %d                                    # 0: CMP; 1: EAC; 2: parameterized CMP\n", (int)sei.gcmpMappingFunctionType);
        if ((int)sei.gcmpMappingFunctionType == 2)
          fprintf(fp, "%s\n", gcmpsettingsStr.c_str());
        fprintf(fp, "InputGCMPPaddingFlag              : %d                                   # 0: input without guard bands; 1: input with guard bands\n", sei.gcmpGuardBandFlag);
        if (sei.gcmpGuardBandFlag)
        {
          fprintf(fp, "InputGCMPPaddingType              : %d                                   # 0: unspecified(repetitive padding is used); 1: repetitive padding; 2: copy from neighboring face; 3: geometry padding\n", (int)sei.gcmpGuardBandType);
          fprintf(fp, "InputGCMPPaddingExteriorFlag      : %d                                   # 0: guard bands only on discontinuous edges; 1: guard bands on both discontinuous edges and frame boundaries\n", sei.gcmpGuardBandBoundaryExteriorFlag);
          fprintf(fp, "InputGCMPPaddingSize              : %d                                   # guard band size for input GCMP\n", (int)sei.gcmpGuardBandSamplesMinus1 + 1);
        }
        fclose(fp);
        360SEIMessageDumped = true;
      }
      else
      {
        msg( ERROR, "File %s could not be opened.\n", decoded360MessageFileName.c_str() );
      }
    }
  }
}

#endif

}
