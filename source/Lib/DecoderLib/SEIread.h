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
 \file     SEIread.h
 \brief    reading funtionality for SEI messages
 */

#pragma once

#include "CommonLib/CommonDef.h"

#include "CommonLib/Slice.h"
#include "CommonLib/SEI_internal.h"
#include "VLCReader.h"

namespace vvdec
{

class InputBitstream;

class SEIReader: public VLCReader
{
public:
  SEIReader() {};
  virtual ~SEIReader() {};
  void parseSEImessage(InputBitstream* bs, seiMessages& seiList,
                       const NalUnitType nalUnitType, const uint32_t nuh_layer_id, const uint32_t temporalId,
                       const VPS *vps, const SPS *sps, HRD &hrd, std::ostream *pDecodedMessageOutputStream);

protected:
  void xReadSEImessage                        (seiMessages& seiList, const NalUnitType nalUnitType, const uint32_t nuh_layer_id, const uint32_t temporalId, const VPS *vps, const SPS *sps, HRD &hrd, std::ostream *pDecodedMessageOutputStream);
  void xParseSEIuserDataUnregistered          (vvdecSEI* s,    uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIDecodingUnitInfo              (vvdecSEI* s,    uint32_t payloadSize, const vvdecSEIBufferingPeriod& bp, const uint32_t temporalId, std::ostream *pDecodedMessageOutputStream);
  void xParseSEIDecodedPictureHash            (vvdecSEI* s,    uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIBufferingPeriod               (vvdecSEI* s,    uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream);
  void xParseSEIPictureTiming                 (vvdecSEI* s,    uint32_t payloadSize, const uint32_t temporalId, const vvdecSEIBufferingPeriod& bp, std::ostream *pDecodedMessageOutputStream);
  void xParseSEIScalableNesting               (vvdecSEI* s, const NalUnitType nalUnitType, const uint32_t nuhLayerId, uint32_t payloadSize, const VPS *vps, const SPS *sps, std::ostream *decodedMessageOutputStream);
  void xCheckScalableNestingConstraints       (const vvdecSEIScalableNesting* sei, const NalUnitType nalUnitType, const VPS* vps);
  void xParseSEIFrameFieldinfo                (vvdecSEI* s,    uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream);
  void xParseSEIDependentRAPIndication        (vvdecSEI* s,    uint32_t payLoadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIFramePacking                  (vvdecSEI* s,    uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIParameterSetsInclusionIndication(vvdecSEI* s,  uint32_t payloadSize,                     std::ostream* pDecodedMessageOutputStream);
  void xParseSEIMasteringDisplayColourVolume  ( vvdecSEI* s,   uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
  void xParseSEIAlternativeTransferCharacteristics(vvdecSEI* s, uint32_t payLoadSize,                     std::ostream *pDecodedMessageOutputStream);
#endif
  void xParseSEIEquirectangularProjection     (vvdecSEI *s,    uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEISphereRotation                (vvdecSEI *s,    uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIOmniViewport                  (vvdecSEI* s,    uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIRegionWisePacking             (vvdecSEI* s,    uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIGeneralizedCubemapProjection  (vvdecSEI* s,    uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEISubpictureLevelInfo           (vvdecSEI* s,    uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEISampleAspectRatioInfo         (vvdecSEI* s,    uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIUserDataRegistered            (vvdecSEI* s,    uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIFilmGrainCharacteristics      (vvdecSEI* s,    uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIContentLightLevelInfo         (vvdecSEI* s,    uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIAmbientViewingEnvironment     (vvdecSEI* s,    uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIContentColourVolume           (vvdecSEI* s,    uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);

  void sei_read_scode(std::ostream *pOS, uint32_t length,   int& code,         const char *pSymbolName);
  void sei_read_code (std::ostream *pOS, uint32_t uiLength, uint32_t& ruiCode, const char *pSymbolName);
  void sei_read_uvlc (std::ostream *pOS, uint32_t& ruiCode,                    const char *pSymbolName);
  void sei_read_svlc (std::ostream *pOS, int&  ruiCode,                        const char *pSymbolName);
  void sei_read_flag (std::ostream *pOS, uint32_t& ruiCode,                    const char *pSymbolName);

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

}
