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
 \file     SEIread.h
 \brief    reading funtionality for SEI messages
 */

#ifndef __SEIREAD__
#define __SEIREAD__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

//! \ingroup DecoderLib
//! \{

#include "CommonLib/TypeDef.h"
#include "CommonLib/Slice.h"
#include "CommonLib/SEI_internal.h"
#include "VLCReader.h"

class InputBitstream;

class SEIReader: public VLCReader
{
public:
  SEIReader() {};
  virtual ~SEIReader() {};
  void parseSEImessage(InputBitstream* bs, seiMessages& seis, std::list<vvdec_sei_message_t*>& seiList,
                       const NalUnitType nalUnitType, const uint32_t nuh_layer_id, const uint32_t temporalId,
                       const VPS *vps, const SPS *sps, HRD &hrd, std::ostream *pDecodedMessageOutputStream);

protected:
  void xReadSEImessage                        (seiMessages& seis, std::list<vvdec_sei_message_t*>& seiList, const NalUnitType nalUnitType, const uint32_t nuh_layer_id, const uint32_t temporalId, const VPS *vps, const SPS *sps, HRD &hrd, std::ostream *pDecodedMessageOutputStream);
  void xParseSEIuserDataUnregistered          (vvdec_sei_message_t* sei,           uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIDecodingUnitInfo              (vvdec_sei_decoding_unit_info_t* sei,              uint32_t payloadSize, const seiBufferingPeriod& bp, const uint32_t temporalId, std::ostream *pDecodedMessageOutputStream);
  void xParseSEIDecodedPictureHash            (seiDecodedPictureHash& sei,            uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIBufferingPeriod               (seiBufferingPeriod& sei,               uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream);
  void xParseSEIPictureTiming                 (seiPictureTiming& sei,                 uint32_t payloadSize, const uint32_t temporalId, const seiBufferingPeriod& bp, std::ostream *pDecodedMessageOutputStream);
  void xParseSEIScalableNesting               (seiScalableNesting& sei, const NalUnitType nalUnitType, const uint32_t nuhLayerId, uint32_t payloadSize, const VPS *vps, const SPS *sps, std::ostream *decodedMessageOutputStream);
  void xCheckScalableNestingConstraints       (const seiScalableNesting& sei, const NalUnitType nalUnitType, const VPS* vps);
  void xParseSEIFrameFieldinfo                (vvdec_sei_frame_field_info_t* sei, const seiPictureTiming& pt, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream);
  void xParseSEIDependentRAPIndication        (seiDependentRAPIndication& sei,        uint32_t payLoadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIFramePacking                  (vvdec_sei_frame_packing_t* sei,                  uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIParameterSetsInclusionIndication(vvdec_sei_parameter_sets_inclusion_indication_t* sei, uint32_t payloadSize,                std::ostream* pDecodedMessageOutputStream);
  void xParseSEIMasteringDisplayColourVolume  ( vvdec_sei_mastering_display_colour_volume_t* s, uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
  void xParseSEIAlternativeTransferCharacteristics(vvdec_sei_alternative_transfer_characteristics_t* sei,              uint32_t payLoadSize,                     std::ostream *pDecodedMessageOutputStream);
#endif
  void xParseSEIEquirectangularProjection     (vvdec_sei_equirectangular_projection_t *sei,     uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEISphereRotation                (vvdec_sei_sphere_rotation_t *sei,                uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIOmniViewport                  (vvdec_sei_omni_viewport_t* sei,                  uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIRegionWisePacking             (vvdec_sei_region_wise_packing_t* sei,             uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIGeneralizedCubemapProjection  (vvdec_sei_message_t* sei,  uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEISubpictureLevelInfo           (seiSubpicureLevelInfo& sei,            uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEISampleAspectRatioInfo         (vvdec_sei_sample_aspect_ratio_info_t* sei,         uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIUserDataRegistered            (vvdec_sei_message_t* sei,            uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIFilmGrainCharacteristics      (seiFilmGrainCharacteristics& sei,      uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIContentLightLevelInfo         (vvdec_sei_message_t* sei,         uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIAmbientViewingEnvironment     (vvdec_sei_ambient_viewing_environment_t* sei,     uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIContentColourVolume           (vvdec_sei_content_colour_volume_t* sei,      uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);

  void sei_read_scode(std::ostream *pOS, uint32_t length, int& code, const char *pSymbolName);
  void sei_read_code(std::ostream *pOS, uint32_t uiLength, uint32_t& ruiCode, const char *pSymbolName);
  void sei_read_uvlc(std::ostream *pOS,                uint32_t& ruiCode, const char *pSymbolName);
  void sei_read_svlc(std::ostream *pOS,                int&  ruiCode, const char *pSymbolName);
  void sei_read_flag(std::ostream *pOS,                uint32_t& ruiCode, const char *pSymbolName);

private:

  void* xSeiAllocPayload( int size );

protected:
  HRD m_nestedHrd;
};

#if JVET_S0257_DUMP_360SEI_MESSAGE
class SeiCfgFileDump
{
public:
  SeiCfgFileDump()
  : m_360SEIMessageDumped(false)
  {};
  virtual ~SeiCfgFileDump() {};

  void write360SeiDump (std::string decoded360MessageFileName, SEIMessages& seis, const SPS* sps);

protected:
  void xDumpSEIEquirectangularProjection     (SEIEquirectangularProjection &sei, const SPS* sps, std::string decoded360MessageFileName);
  void xDumpSEIGeneralizedCubemapProjection  (SEIGeneralizedCubemapProjection &sei, const SPS* sps, std::string decoded360MessageFileName);

  bool m_360SEIMessageDumped;

};


#endif

//! \}

#endif
