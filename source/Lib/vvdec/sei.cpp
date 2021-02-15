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

Copyright (c) 2018-2020, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. 
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
#include "vvdec/sei.h"

namespace vvdec {

const char *sei::getSEIMessageString(sei::PayloadType payloadType)
{
  switch (payloadType)
  {
    case sei::BUFFERING_PERIOD:                     return "Buffering period";
    case sei::PICTURE_TIMING:                       return "Picture timing";
    case sei::FILLER_PAYLOAD:                       return "Filler payload";                       // not currently decoded
    case sei::USER_DATA_REGISTERED_ITU_T_T35:       return "User data registered";                 // not currently decoded
    case sei::USER_DATA_UNREGISTERED:               return "User data unregistered";
    case sei::FILM_GRAIN_CHARACTERISTICS:           return "Film grain characteristics";           // not currently decoded
    case sei::FRAME_PACKING:                        return "Frame packing arrangement";
    case sei::PARAMETER_SETS_INCLUSION_INDICATION:  return "Parameter sets inclusion indication";
    case sei::DECODING_UNIT_INFO:                   return "Decoding unit information";
    case sei::SCALABLE_NESTING:                     return "Scalable nesting";
    case sei::DECODED_PICTURE_HASH:                 return "Decoded picture hash";
    case sei::DEPENDENT_RAP_INDICATION:             return "Dependent RAP indication";
    case sei::MASTERING_DISPLAY_COLOUR_VOLUME:      return "Mastering display colour volume";
    case sei::ALTERNATIVE_TRANSFER_CHARACTERISTICS: return "Alternative transfer characteristics";
    case sei::CONTENT_LIGHT_LEVEL_INFO:             return "Content light level information";
    case sei::AMBIENT_VIEWING_ENVIRONMENT:          return "Ambient viewing environment";
    case sei::CONTENT_COLOUR_VOLUME:                return "Content colour volume";
    case sei::EQUIRECTANGULAR_PROJECTION:           return "Equirectangular projection";
    case sei::SPHERE_ROTATION:                      return "Sphere rotation";
    case sei::REGION_WISE_PACKING:                  return "Region wise packing information";
    case sei::OMNI_VIEWPORT:                        return "Omni viewport";
    case sei::GENERALIZED_CUBEMAP_PROJECTION:       return "Generalized cubemap projection";
    case sei::SAMPLE_ASPECT_RATIO_INFO:             return "Sample aspect ratio information";
    case sei::SUBPICTURE_LEVEL_INFO:                return "Subpicture level information";
    default:                                        return "Unknown";
  }
}

std::list<sei*> sei::getSeisByType(const std::list<sei*> &seiList, sei::PayloadType seiType)
{
  std::list<sei*> result;

  for (std::list<sei*>::const_iterator it=seiList.begin(); it!=seiList.end(); it++)
  {
    if ((*it)->payloadType() == seiType)
    {
      result.push_back(*it);
    }
  }
  return result;
}

std::list<sei*> sei::extractSeisByType(std::list<sei*> &seiList, sei::PayloadType seiType)
{
  std::list<sei*> result;
  std::list<sei*>::iterator it=seiList.begin();
  while ( it!=seiList.end() )
  {
    if ((*it)->payloadType() == seiType)
    {
      result.push_back(*it);
      it = seiList.erase(it);
    }
    else
    {
      it++;
    }
  }
  return result;
}


void sei::deleteSEIs (std::list<sei*> &seiList)
{
  for (std::list<sei*>::iterator it=seiList.begin(); it!=seiList.end(); it++)
  {
    delete (*it);
  }
  seiList.clear();
}


void seiBufferingPeriod::copyTo (seiBufferingPeriod& target) const
{
  target.m_bpNalCpbParamsPresentFlag = m_bpNalCpbParamsPresentFlag;
  target.m_bpVclCpbParamsPresentFlag = m_bpVclCpbParamsPresentFlag;
  target.m_initialCpbRemovalDelayLength = m_initialCpbRemovalDelayLength;
  target.m_cpbRemovalDelayLength = m_cpbRemovalDelayLength;
  target.m_dpbOutputDelayLength = m_dpbOutputDelayLength;
  target.m_duCpbRemovalDelayIncrementLength = m_duCpbRemovalDelayIncrementLength;
  target.m_dpbOutputDelayDuLength = m_dpbOutputDelayDuLength;
  target.m_concatenationFlag = m_concatenationFlag;
  target.m_auCpbRemovalDelayDelta = m_auCpbRemovalDelayDelta;
  target.m_cpbRemovalDelayDeltasPresentFlag =  m_cpbRemovalDelayDeltasPresentFlag;
  target.m_numCpbRemovalDelayDeltas = m_numCpbRemovalDelayDeltas;
  target.m_bpMaxSubLayers = m_bpMaxSubLayers;
  ::memcpy(target.m_initialCpbRemovalDelay, m_initialCpbRemovalDelay, sizeof(m_initialCpbRemovalDelay));
  ::memcpy(target.m_initialCpbRemovalOffset, m_initialCpbRemovalOffset, sizeof(m_initialCpbRemovalOffset));
  ::memcpy(target.m_cpbRemovalDelayDelta, m_cpbRemovalDelayDelta, sizeof(m_cpbRemovalDelayDelta));
  target.m_bpCpbCnt = m_bpCpbCnt;
  target.m_bpDecodingUnitHrdParamsPresentFlag = m_bpDecodingUnitHrdParamsPresentFlag;
  target.m_decodingUnitCpbParamsInPicTimingSeiFlag = m_decodingUnitCpbParamsInPicTimingSeiFlag;
  target.m_sublayerInitialCpbRemovalDelayPresentFlag = m_sublayerInitialCpbRemovalDelayPresentFlag;
  target.m_concatenationFlag = m_concatenationFlag;
  target.m_maxInitialRemovalDelayForConcatenation = m_maxInitialRemovalDelayForConcatenation;
  target.m_sublayerDpbOutputOffsetsPresentFlag = m_sublayerDpbOutputOffsetsPresentFlag;
  ::memcpy(target.m_dpbOutputTidOffset, m_dpbOutputTidOffset, sizeof(m_dpbOutputTidOffset));
  target.m_altCpbParamsPresentFlag = m_altCpbParamsPresentFlag;
}

void seiPictureTiming::copyTo (seiPictureTiming& target) const
{
  ::memcpy(target.m_auCpbRemovalDelay, m_auCpbRemovalDelay, sizeof(m_auCpbRemovalDelay));
  ::memcpy(target.m_ptSubLayerDelaysPresentFlag, m_ptSubLayerDelaysPresentFlag, sizeof(m_ptSubLayerDelaysPresentFlag));
  ::memcpy(target.m_duCommonCpbRemovalDelayMinus1, m_duCommonCpbRemovalDelayMinus1, sizeof(m_duCommonCpbRemovalDelayMinus1));
  ::memcpy(target.m_cpbRemovalDelayDeltaEnabledFlag, m_cpbRemovalDelayDeltaEnabledFlag, sizeof(m_cpbRemovalDelayDeltaEnabledFlag));
  ::memcpy(target.m_cpbRemovalDelayDeltaIdx, m_cpbRemovalDelayDeltaIdx, sizeof(m_cpbRemovalDelayDeltaIdx));
  target.m_picDpbOutputDelay = m_picDpbOutputDelay;
  target.m_picDpbOutputDuDelay = m_picDpbOutputDuDelay;
  target.m_numDecodingUnitsMinus1 = m_numDecodingUnitsMinus1;
  target.m_duCommonCpbRemovalDelayFlag = m_duCommonCpbRemovalDelayFlag;

  target.m_numNalusInDuMinus1 = m_numNalusInDuMinus1;
  target.m_duCpbRemovalDelayMinus1 = m_duCpbRemovalDelayMinus1;
  target.m_cpbAltTimingInfoPresentFlag = m_cpbAltTimingInfoPresentFlag;
  target.m_nalCpbAltInitialRemovalDelayDelta  = m_nalCpbAltInitialRemovalDelayDelta;
  target.m_nalCpbAltInitialRemovalOffsetDelta = m_nalCpbAltInitialRemovalOffsetDelta;
  target.m_nalCpbDelayOffset = m_nalCpbDelayOffset;
  target.m_nalCpbDelayOffset = m_nalCpbDelayOffset;
  target.m_vclCpbAltInitialRemovalDelayDelta  = m_vclCpbAltInitialRemovalDelayDelta;
  target.m_vclCpbAltInitialRemovalOffsetDelta = m_vclCpbAltInitialRemovalOffsetDelta;
  target.m_vclCpbDelayOffset = m_vclCpbDelayOffset;
  target.m_vclCpbDelayOffset = m_vclCpbDelayOffset;
}

void seiDecodingUnitInfo::copyTo (vvdec::seiDecodingUnitInfo& target) const
{
  target.m_decodingUnitIdx = m_decodingUnitIdx;

  ::memcpy(target.m_duiSubLayerDelaysPresentFlag, m_duiSubLayerDelaysPresentFlag, sizeof(m_duiSubLayerDelaysPresentFlag));
  ::memcpy(target.m_duSptCpbRemovalDelayIncrement, m_duSptCpbRemovalDelayIncrement, sizeof(m_duSptCpbRemovalDelayIncrement));

  target.m_dpbOutputDuDelayPresentFlag = m_dpbOutputDuDelayPresentFlag;
  target.m_picSptDpbOutputDuDelay      = m_picSptDpbOutputDuDelay;
}

} // namespace

