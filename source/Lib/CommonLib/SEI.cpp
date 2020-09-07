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
