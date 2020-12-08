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

/** \file     SEI.cpp
    \brief    helper functions for SEI handling
*/

#include "CommonDef.h"
#include "SEI.h"

#include "dtrace_next.h"

#if ENABLE_TRACING
void xTraceSEIHeader()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== SEI message ===========\n" );
}

void xTraceSEIMessageType( SEI::PayloadType payloadType )
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== %s SEI message ===========\n", SEI::getSEIMessageString( payloadType ) );
}
#endif

SEIMessages getSeisByType(const SEIMessages &seiList, SEI::PayloadType seiType)
{
  SEIMessages result;

  for (SEIMessages::const_iterator it=seiList.begin(); it!=seiList.end(); it++)
  {
    if ((*it)->payloadType() == seiType)
    {
      result.push_back(*it);
    }
  }
  return result;
}

SEIMessages extractSeisByType(SEIMessages &seiList, SEI::PayloadType seiType)
{
  SEIMessages result;

  SEIMessages::iterator it=seiList.begin();
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


void deleteSEIs (SEIMessages &seiList)
{
  for (SEIMessages::iterator it=seiList.begin(); it!=seiList.end(); it++)
  {
    delete (*it);
  }
  seiList.clear();
}

void SEIBufferingPeriod::copyTo (SEIBufferingPeriod& target) const
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

void SEIPictureTiming::copyTo (SEIPictureTiming& target) const
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

// Static member
const char *SEI::getSEIMessageString(SEI::PayloadType payloadType)
{
  switch (payloadType)
  {
    case SEI::BUFFERING_PERIOD:                     return "Buffering period";
    case SEI::PICTURE_TIMING:                       return "Picture timing";
    case SEI::FILLER_PAYLOAD:                       return "Filler payload";                       // not currently decoded
    case SEI::USER_DATA_REGISTERED_ITU_T_T35:       return "User data registered";                 // not currently decoded
    case SEI::USER_DATA_UNREGISTERED:               return "User data unregistered";
    case SEI::FILM_GRAIN_CHARACTERISTICS:           return "Film grain characteristics";           // not currently decoded
    case SEI::FRAME_PACKING:                        return "Frame packing arrangement";
    case SEI::PARAMETER_SETS_INCLUSION_INDICATION:  return "Parameter sets inclusion indication";
    case SEI::DECODING_UNIT_INFO:                   return "Decoding unit information";
    case SEI::SCALABLE_NESTING:                     return "Scalable nesting";
    case SEI::DECODED_PICTURE_HASH:                 return "Decoded picture hash";
    case SEI::DEPENDENT_RAP_INDICATION:             return "Dependent RAP indication";
    case SEI::MASTERING_DISPLAY_COLOUR_VOLUME:      return "Mastering display colour volume";
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
    case SEI::ALTERNATIVE_TRANSFER_CHARACTERISTICS: return "Alternative transfer characteristics";
#endif
    case SEI::CONTENT_LIGHT_LEVEL_INFO:             return "Content light level information";
    case SEI::AMBIENT_VIEWING_ENVIRONMENT:          return "Ambient viewing environment";
    case SEI::CONTENT_COLOUR_VOLUME:                return "Content colour volume";
    case SEI::EQUIRECTANGULAR_PROJECTION:           return "Equirectangular projection";
    case SEI::SPHERE_ROTATION:                      return "Sphere rotation";
    case SEI::REGION_WISE_PACKING:                  return "Region wise packing information";
    case SEI::OMNI_VIEWPORT:                        return "Omni viewport";
    case SEI::GENERALIZED_CUBEMAP_PROJECTION:       return "Generalized cubemap projection";
    case SEI::SAMPLE_ASPECT_RATIO_INFO:             return "Sample aspect ratio information";
    case SEI::SUBPICTURE_LEVEL_INFO:                return "Subpicture level information";
    default:                                        return "Unknown";
  }
}
