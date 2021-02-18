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

#include "vvdec/sei.h"
#include "CommonLib/TypeDef.h"
#include "CommonLib/Slice.h"

class InputBitstream;

class SEIReader: public VLCReader
{
public:
  SEIReader() {};
  virtual ~SEIReader() {};
  void parseSEImessage(InputBitstream* bs, vvdec::seiMessages& seis, const NalUnitType nalUnitType, const uint32_t nuh_layer_id, const uint32_t temporalId,const VPS *vps, const SPS *sps, HRD &hrd, std::ostream *pDecodedMessageOutputStream);

protected:
  void xReadSEImessage                        (vvdec::seiMessages& seis, const NalUnitType nalUnitType, const uint32_t nuh_layer_id, const uint32_t temporalId, const VPS *vps, const SPS *sps, HRD &hrd, std::ostream *pDecodedMessageOutputStream);
  void xParseSEIuserDataUnregistered          (vvdec::seiUserDataUnregistered &sei,          uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIDecodingUnitInfo              (vvdec::seiDecodingUnitInfo& sei,              uint32_t payloadSize, const vvdec::seiBufferingPeriod& bp, const uint32_t temporalId, std::ostream *pDecodedMessageOutputStream);
  void xParseSEIDecodedPictureHash            (vvdec::seiDecodedPictureHash& sei,            uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIBufferingPeriod               (vvdec::seiBufferingPeriod& sei,               uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream);
  void xParseSEIPictureTiming                 (vvdec::seiPictureTiming& sei,                 uint32_t payloadSize, const uint32_t temporalId, const vvdec::seiBufferingPeriod& bp, std::ostream *pDecodedMessageOutputStream);
  void xParseSEIScalableNesting               (vvdec::seiScalableNesting& sei, const NalUnitType nalUnitType, const uint32_t nuhLayerId, uint32_t payloadSize, const VPS *vps, const SPS *sps, std::ostream *decodedMessageOutputStream);
  void xCheckScalableNestingConstraints       (const vvdec::seiScalableNesting& sei, const NalUnitType nalUnitType, const VPS* vps);
  void xParseSEIFrameFieldinfo                (vvdec::seiFrameFieldInfo& sei, const vvdec::seiPictureTiming& pt, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream);
  void xParseSEIDependentRAPIndication        (vvdec::seiDependentRAPIndication& sei,        uint32_t payLoadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIFramePacking                  (vvdec::seiFramePacking& sei,                  uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIParameterSetsInclusionIndication(vvdec::seiParameterSetsInclusionIndication& sei, uint32_t payloadSize,                std::ostream* pDecodedMessageOutputStream);
  void xParseSEIMasteringDisplayColourVolume  (vvdec::seiMasteringDisplayColourVolume& sei,  uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
  void xParseSEIAlternativeTransferCharacteristics(vvdec::seiAlternativeTransferCharacteristics& sei,              uint32_t payLoadSize,                     std::ostream *pDecodedMessageOutputStream);
#endif
  void xParseSEIEquirectangularProjection     (vvdec::seiEquirectangularProjection &sei,     uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEISphereRotation                (vvdec::seiSphereRotation &sei,                uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIOmniViewport                  (vvdec::seiOmniViewport& sei,                  uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIRegionWisePacking             (vvdec::seiRegionWisePacking& sei,             uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIGeneralizedCubemapProjection  (vvdec::seiGeneralizedCubemapProjection &sei,  uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEISubpictureLevelInfo           (vvdec::seiSubpicureLevelInfo& sei,              uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEISampleAspectRatioInfo         (vvdec::seiSampleAspectRatioInfo& sei,         uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIUserDataRegistered            (vvdec::seiUserDataRegistered& sei,            uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIFilmGrainCharacteristics      (vvdec::seiFilmGrainCharacteristics& sei,      uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIContentLightLevelInfo         (vvdec::seiContentLightLevelInfo& sei,         uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIAmbientViewingEnvironment     (vvdec::seiAmbientViewingEnvironment& sei,     uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIContentColourVolume           (vvdec::seiContentColourVolume& sei,           uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);

  void sei_read_scode(std::ostream *pOS, uint32_t length, int& code, const char *pSymbolName);
  void sei_read_code(std::ostream *pOS, uint32_t uiLength, uint32_t& ruiCode, const char *pSymbolName);
  void sei_read_uvlc(std::ostream *pOS,                uint32_t& ruiCode, const char *pSymbolName);
  void sei_read_svlc(std::ostream *pOS,                int&  ruiCode, const char *pSymbolName);
  void sei_read_flag(std::ostream *pOS,                uint32_t& ruiCode, const char *pSymbolName);
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
