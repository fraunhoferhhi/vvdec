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

void SEIEquirectangularProjection::copyTo (vvdec::seiEquirectangularProjection& target) const
{
  target.m_erpCancelFlag          = m_erpCancelFlag;
  target.m_erpPersistenceFlag     = m_erpPersistenceFlag;
  target.m_erpGuardBandFlag       = m_erpGuardBandFlag;
  target.m_erpGuardBandType       = m_erpGuardBandType;
  target.m_erpLeftGuardBandWidth  = m_erpLeftGuardBandWidth ;
  target.m_erpRightGuardBandWidth = m_erpRightGuardBandWidth;
}

void SEISphereRotation::copyTo (vvdec::seiSphereRotation& target) const
{
  target.m_sphereRotationCancelFlag      = m_sphereRotationCancelFlag;
  target.m_sphereRotationPersistenceFlag = m_sphereRotationPersistenceFlag;
  target.m_sphereRotationYaw             = m_sphereRotationYaw;
  target.m_sphereRotationPitch           = m_sphereRotationPitch;
  target.m_sphereRotationRoll            = m_sphereRotationRoll;
}

void SEIOmniViewport::copyTo (vvdec::seiOmniViewport& target) const
{
  target.m_omniViewportId              = m_omniViewportId;
  target.m_omniViewportCancelFlag      = m_omniViewportCancelFlag;
  target.m_omniViewportPersistenceFlag = m_omniViewportPersistenceFlag;
  target.m_omniViewportCnt             = m_omniViewportCntMinus1+1;

  target.m_omniViewportRegions.resize(m_omniViewportRegions.size());
  for (size_t i = 0; i < m_omniViewportRegions.size(); i++)
  {
    target.m_omniViewportRegions[i].azimuthCentre   = m_omniViewportRegions[i].azimuthCentre;
    target.m_omniViewportRegions[i].elevationCentre = m_omniViewportRegions[i].elevationCentre;
    target.m_omniViewportRegions[i].tiltCentre      = m_omniViewportRegions[i].tiltCentre;
    target.m_omniViewportRegions[i].horRange        = m_omniViewportRegions[i].horRange;
    target.m_omniViewportRegions[i].verRange        = m_omniViewportRegions[i].verRange;
  }
}

void SEIRegionWisePacking::copyTo (vvdec::seiRegionWisePacking& target) const
{
  target.m_rwpCancelFlag                  = m_rwpCancelFlag;
  target.m_rwpPersistenceFlag             = m_rwpPersistenceFlag;
  target.m_constituentPictureMatchingFlag = m_constituentPictureMatchingFlag;
  target.m_numPackedRegions               = m_numPackedRegions;
  target.m_projPictureWidth               = m_projPictureWidth;
  target.m_projPictureHeight              = m_projPictureHeight;
  target.m_packedPictureWidth             = m_packedPictureWidth;
  target.m_packedPictureHeight            = m_packedPictureHeight;

  std::copy(m_rwpTransformType.begin(), m_rwpTransformType.end(), std::back_inserter(target.m_rwpTransformType));
  std::copy(m_rwpGuardBandFlag.begin(), m_rwpGuardBandFlag.end(), std::back_inserter(target.m_rwpGuardBandFlag));
  std::copy(m_projRegionWidth.begin(), m_projRegionWidth.end(), std::back_inserter(target.m_projRegionWidth));
  std::copy(m_projRegionHeight.begin(), m_projRegionHeight.end(), std::back_inserter(target.m_projRegionHeight));
  std::copy(m_rwpProjRegionTop.begin(), m_rwpProjRegionTop.end(), std::back_inserter(target.m_rwpProjRegionTop));
  std::copy(m_projRegionLeft.begin(), m_projRegionLeft.end(), std::back_inserter(target.m_projRegionLeft));
  std::copy(m_packedRegionWidth.begin(), m_packedRegionWidth.end(), std::back_inserter(target.m_packedRegionWidth));
  std::copy(m_packedRegionHeight.begin(), m_packedRegionHeight.end(), std::back_inserter(target.m_packedRegionHeight));
  std::copy(m_packedRegionTop.begin(), m_packedRegionTop.end(), std::back_inserter(target.m_packedRegionTop));
  std::copy(m_packedRegionLeft.begin(), m_packedRegionLeft.end(), std::back_inserter(target.m_packedRegionLeft));
  std::copy(m_rwpLeftGuardBandWidth.begin(), m_rwpLeftGuardBandWidth.end(), std::back_inserter(target.m_rwpLeftGuardBandWidth));
  std::copy(m_rwpRightGuardBandWidth.begin(), m_rwpRightGuardBandWidth.end(), std::back_inserter(target.m_rwpRightGuardBandWidth));
  std::copy(m_rwpTopGuardBandHeight.begin(), m_rwpTopGuardBandHeight.end(), std::back_inserter(target.m_rwpTopGuardBandHeight));
  std::copy(m_rwpBottomGuardBandHeight.begin(), m_rwpBottomGuardBandHeight.end(), std::back_inserter(target.m_rwpBottomGuardBandHeight));
  std::copy(m_rwpGuardBandNotUsedForPredFlag.begin(), m_rwpGuardBandNotUsedForPredFlag.end(), std::back_inserter(target.m_rwpGuardBandNotUsedForPredFlag));
  std::copy(m_rwpGuardBandType.begin(), m_rwpGuardBandType.end(), std::back_inserter(target.m_rwpGuardBandType));
}

void SEIGeneralizedCubemapProjection::copyTo (vvdec::seiGeneralizedCubemapProjection& target) const
{
  target.m_gcmpCancelFlag      = m_gcmpCancelFlag;
  target.m_gcmpPersistenceFlag = m_gcmpPersistenceFlag;
  target.m_gcmpPackingType     = m_gcmpPackingType;
  target.m_gcmpMappingFunctionType = m_gcmpMappingFunctionType;
  std::copy(m_gcmpFaceIndex.begin(), m_gcmpFaceIndex.end(), std::back_inserter(target.m_gcmpFaceIndex));
  std::copy(m_gcmpFaceRotation.begin(), m_gcmpFaceRotation.end(), std::back_inserter(target.m_gcmpFaceRotation));
  std::copy(m_gcmpFunctionCoeffU.begin(), m_gcmpFunctionCoeffU.end(), std::back_inserter(target.m_gcmpFunctionCoeffU));
  std::copy(m_gcmpFunctionUAffectedByVFlag.begin(), m_gcmpFunctionUAffectedByVFlag.end(), std::back_inserter(target.m_gcmpFunctionUAffectedByVFlag));
  std::copy(m_gcmpFunctionCoeffV.begin(), m_gcmpFunctionCoeffV.end(), std::back_inserter(target.m_gcmpFunctionCoeffV));
  std::copy(m_gcmpFunctionVAffectedByUFlag.begin(), m_gcmpFunctionVAffectedByUFlag.end(), std::back_inserter(target.m_gcmpFunctionVAffectedByUFlag));
  target.m_gcmpGuardBandFlag = m_gcmpGuardBandFlag;
  target.m_gcmpGuardBandType = m_gcmpGuardBandType;
  target.m_gcmpGuardBandBoundaryExteriorFlag = m_gcmpGuardBandBoundaryExteriorFlag;
  target.m_gcmpGuardBandSamples  = m_gcmpGuardBandSamplesMinus1+1;
}

void SEISampleAspectRatioInfo::copyTo (vvdec::seiSampleAspectRatioInfo& target) const
{
  target.m_sariCancelFlag      = m_sariCancelFlag;
  target.m_sariPersistenceFlag = m_sariPersistenceFlag;
  target.m_sariAspectRatioIdc  = m_sariAspectRatioIdc;
  target.m_sariSarWidth        = m_sariSarWidth;
  target.m_sariSarHeight       = m_sariSarHeight;
}

void SEIuserDataUnregistered::copyTo (vvdec::seiUserDataUnregistered& target) const
{
  ::memcpy(target.uuid_iso_iec_11578, uuid_iso_iec_11578, sizeof(uuid_iso_iec_11578));

  target.userDataLength = userDataLength;
  target.userData = new uint8_t[userDataLength];
  for (uint32_t i = 0; i < userDataLength; i++)
  {
    target.userData[i] = userData[i];
  }
}


void SEIDecodedPictureHash::copyTo (vvdec::seiDecodedPictureHash& target) const
{
  target.method = (vvdec::seiDecodedPictureHash::HashType)method;
  target.singleCompFlag = singleCompFlag;

  std::copy(m_pictureHash.hash.begin(), m_pictureHash.hash.end(),
            std::back_inserter(target.m_pictureHash.hash));
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

void SEIBufferingPeriod::copyTo (vvdec::seiBufferingPeriod& target) const
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

void SEIPictureTiming::copyTo (vvdec::seiPictureTiming& target) const
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

void SEIDecodingUnitInfo::copyTo (vvdec::seiDecodingUnitInfo& target) const
{
  target.m_decodingUnitIdx = m_decodingUnitIdx;

  ::memcpy(target.m_duiSubLayerDelaysPresentFlag, m_duiSubLayerDelaysPresentFlag, sizeof(m_duiSubLayerDelaysPresentFlag));
  ::memcpy(target.m_duSptCpbRemovalDelayIncrement, m_duSptCpbRemovalDelayIncrement, sizeof(m_duSptCpbRemovalDelayIncrement));

  target.m_dpbOutputDuDelayPresentFlag = m_dpbOutputDuDelayPresentFlag;
  target.m_picSptDpbOutputDuDelay      = m_picSptDpbOutputDuDelay;
}

void SEIFrameFieldInfo::copyTo (vvdec::seiFrameFieldInfo& target) const
{
  target.m_fieldPicFlag               = m_fieldPicFlag;
  target.m_bottomFieldFlag            = m_bottomFieldFlag;
  target.m_pairingIndicatedFlag       = m_pairingIndicatedFlag;
  target.m_pairedWithNextFieldFlag    = m_pairedWithNextFieldFlag;
  target.m_displayFieldsFromFrameFlag = m_displayFieldsFromFrameFlag;
  target.m_topFieldFirstFlag          = m_topFieldFirstFlag;
  target.m_displayElementalPeriods    = m_displayElementalPeriodsMinus1+1;
  target.m_sourceScanType             = m_sourceScanType;
  target.m_duplicateFlag              = m_duplicateFlag;
}

void SEIFramePacking::copyTo (vvdec::seiFramePacking& target) const
{
  target.m_arrangementId = m_arrangementId;
  target.m_arrangementCancelFlag = m_arrangementCancelFlag;
  target.m_arrangementType = m_arrangementType;
  target.m_quincunxSamplingFlag = m_quincunxSamplingFlag;
  target.m_contentInterpretationType = m_contentInterpretationType;
  target.m_spatialFlippingFlag = m_spatialFlippingFlag;
  target.m_frame0FlippedFlag = m_frame0FlippedFlag;
  target.m_fieldViewsFlag = m_fieldViewsFlag;
  target.m_currentFrameIsFrame0Flag = m_currentFrameIsFrame0Flag;
  target.m_frame0SelfContainedFlag  = m_frame0SelfContainedFlag;
  target.m_frame1SelfContainedFlag  = m_frame1SelfContainedFlag;
  target.m_frame0GridPositionX = m_frame0GridPositionX;
  target.m_frame0GridPositionY = m_frame0GridPositionY;
  target.m_frame1GridPositionX = m_frame1GridPositionX;
  target.m_frame1GridPositionY = m_frame1GridPositionY;
  target.m_arrangementReservedByte = m_arrangementReservedByte;
  target.m_arrangementPersistenceFlag = m_arrangementPersistenceFlag;
  target.m_upsampledAspectRatio = m_upsampledAspectRatio;
}


void SEIParameterSetsInclusionIndication::copyTo (vvdec::seiParameterSetsInclusionIndication& target) const
{
  target.m_selfContainedClvsFlag = m_selfContainedClvsFlag;
}

void SEIMasteringDisplayColourVolume::copyTo(vvdec::seiMasteringDisplayColourVolume& target) const
{
  target.colourVolumeSEIEnabled = values.colourVolumeSEIEnabled;
  target.maxLuminance = values.maxLuminance;
  target.minLuminance = values.minLuminance;
  ::memcpy(target.primaries, values.primaries, sizeof(values.primaries));
  ::memcpy(target.whitePoint, values.whitePoint, sizeof(values.whitePoint));
}

void SEIScalableNesting::copyTo (vvdec::seiScalableNesting& target) const
{
  target.m_snOlsFlag = m_snOlsFlag;
  target.m_snSubpicFlag = m_snSubpicFlag;
  target.m_snNumOlss = m_snNumOlssMinus1+1;
  for( int i = 0; i < MAX_NESTING_NUM_LAYER; i++ )
  {
    target.m_snOlsIdxDelta[i] = m_snOlsIdxDeltaMinus1[i]+1;
    target.m_snOlsIdx[i]      = m_snOlsIdx[i];
    target.m_snLayerId[i]     = m_snLayerId[i];
  }
  target.m_snAllLayersFlag = m_snAllLayersFlag;
  target.m_snNumLayers     = m_snNumLayersMinus1+1;
  target.m_snNumSubpics    = m_snNumSubpics;
  target.m_snSubpicIdLen   = m_snSubpicIdLen;

  std::copy(m_snSubpicId.begin(), m_snSubpicId.end(),
            std::back_inserter(target.m_snSubpicId));

  target.m_snNumSEIs   = m_snNumSEIs;

  for( auto &sei : m_nestedSEIs )
  {
    switch( sei->payloadType() )
    {
      case SEI::BUFFERING_PERIOD                     :
        {
          SEIBufferingPeriod* src = (SEIBufferingPeriod*)sei;
          vvdec::seiBufferingPeriod* t = new vvdec::seiBufferingPeriod;
          src->copyTo(*t);
          target.m_nestedSEIs.push_back( t );
        }
        break;
      case SEI::PICTURE_TIMING                       :
        {
          SEIPictureTiming* src = (SEIPictureTiming*)sei;
          vvdec::seiPictureTiming* t = new vvdec::seiPictureTiming;
          src->copyTo(*t);
          target.m_nestedSEIs.push_back( t );
        }
        break;
      case SEI::FILLER_PAYLOAD                       :
        break;
      case SEI::USER_DATA_REGISTERED_ITU_T_T35       :
        {
          SEIUserDataRegistered* src = (SEIUserDataRegistered*)sei;
          vvdec::seiUserDataRegistered* t = new vvdec::seiUserDataRegistered;
          src->copyTo(*t);
          target.m_nestedSEIs.push_back( t );
        }
        break;
      case SEI::USER_DATA_UNREGISTERED               :
        {
          SEIuserDataUnregistered* src = (SEIuserDataUnregistered*)sei;
          vvdec::seiUserDataUnregistered* t = new vvdec::seiUserDataUnregistered;
          src->copyTo(*t);
          target.m_nestedSEIs.push_back( t );
        }
        break;
      case SEI::FILM_GRAIN_CHARACTERISTICS           :
        {
          SEIFilmGrainCharacteristics* src = (SEIFilmGrainCharacteristics*)sei;
          vvdec::seiFilmGrainCharacteristics* t = new vvdec::seiFilmGrainCharacteristics;
          src->copyTo(*t);
          target.m_nestedSEIs.push_back( t );
        }
        break;

      case SEI::FRAME_PACKING                        :
        {
          SEIFramePacking* src = (SEIFramePacking*)sei;
          vvdec::seiFramePacking* t = new vvdec::seiFramePacking;
          src->copyTo(*t);
          target.m_nestedSEIs.push_back( t );
        }
        break;
      case SEI::PARAMETER_SETS_INCLUSION_INDICATION  :
        {
          SEIParameterSetsInclusionIndication* src = (SEIParameterSetsInclusionIndication*)sei;
          vvdec::seiParameterSetsInclusionIndication* t = new vvdec::seiParameterSetsInclusionIndication;
          src->copyTo(*t);
          target.m_nestedSEIs.push_back( t );
        }
        break;
      case SEI::DECODING_UNIT_INFO                   :
        {
          SEIDecodingUnitInfo* src = (SEIDecodingUnitInfo*)sei;
          vvdec::seiDecodingUnitInfo* t = new vvdec::seiDecodingUnitInfo;
          src->copyTo(*t);
          target.m_nestedSEIs.push_back( t );
        }
        break;
      case SEI::DECODED_PICTURE_HASH                 :
        {
          SEIDecodedPictureHash* src = (SEIDecodedPictureHash*)sei;
          vvdec::seiDecodedPictureHash* t = new vvdec::seiDecodedPictureHash;
          src->copyTo(*t);
          target.m_nestedSEIs.push_back( t );
        }
        break;
      case SEI::SCALABLE_NESTING                     :
        {
          SEIScalableNesting* src = (SEIScalableNesting*)sei;
          vvdec::seiScalableNesting* t = new vvdec::seiScalableNesting;
          src->copyTo(*t);
          target.m_nestedSEIs.push_back( t );
        }
        break;
      case SEI::MASTERING_DISPLAY_COLOUR_VOLUME      :
        {
          SEIMasteringDisplayColourVolume* src = (SEIMasteringDisplayColourVolume*)sei;
          vvdec::seiMasteringDisplayColourVolume* t = new vvdec::seiMasteringDisplayColourVolume;
          src->copyTo(*t);
          target.m_nestedSEIs.push_back( t );
        }
        break;

      case SEI::DEPENDENT_RAP_INDICATION             : break;
      case SEI::EQUIRECTANGULAR_PROJECTION           :
        {
          SEIEquirectangularProjection* src = (SEIEquirectangularProjection*)sei;
          vvdec::seiEquirectangularProjection* t = new vvdec::seiEquirectangularProjection;
          src->copyTo(*t);
          target.m_nestedSEIs.push_back( t );
        }
        break;
      case SEI::SPHERE_ROTATION                      :
        {
          SEISphereRotation* src = (SEISphereRotation*)sei;
          vvdec::seiSphereRotation* t = new vvdec::seiSphereRotation;
          src->copyTo(*t);
          target.m_nestedSEIs.push_back( t );
        }
        break;
      case SEI::REGION_WISE_PACKING                  :
        {
          SEIRegionWisePacking* src = (SEIRegionWisePacking*)sei;
          vvdec::seiRegionWisePacking* t = new vvdec::seiRegionWisePacking;
          src->copyTo(*t);
          target.m_nestedSEIs.push_back( t );
        }
        break;
      case SEI::OMNI_VIEWPORT                        :
        {
          SEIOmniViewport* src = (SEIOmniViewport*)sei;
          vvdec::seiOmniViewport* t = new vvdec::seiOmniViewport;
          src->copyTo(*t);
          target.m_nestedSEIs.push_back( t );
        }
        break;
      case SEI::GENERALIZED_CUBEMAP_PROJECTION       :
        {
          SEIGeneralizedCubemapProjection* src = (SEIGeneralizedCubemapProjection*)sei;
          vvdec::seiGeneralizedCubemapProjection* t = new vvdec::seiGeneralizedCubemapProjection;
          src->copyTo(*t);
          target.m_nestedSEIs.push_back( t );
        }
        break;
      case SEI::FRAME_FIELD_INFO                     :
        {
          SEIFrameFieldInfo* src = (SEIFrameFieldInfo*)sei;
          vvdec::seiFrameFieldInfo* t = new vvdec::seiFrameFieldInfo;
          src->copyTo(*t);
          target.m_nestedSEIs.push_back( t );
        }
        break;
      case SEI::SUBPICTURE_LEVEL_INFO                :
        {
          SEISubpicureLevelInfo* src = (SEISubpicureLevelInfo*)sei;
          vvdec::seiSubpicureLevelInfo* t = new vvdec::seiSubpicureLevelInfo;
          src->copyTo(*t);
          target.m_nestedSEIs.push_back( t );
        }
        break;
      case SEI::SAMPLE_ASPECT_RATIO_INFO             :
        {
          SEISampleAspectRatioInfo* src = (SEISampleAspectRatioInfo*)sei;
          vvdec::seiSampleAspectRatioInfo* t = new vvdec::seiSampleAspectRatioInfo;
          src->copyTo(*t);
          target.m_nestedSEIs.push_back( t );
        }
        break;
      case SEI::CONTENT_LIGHT_LEVEL_INFO             :
        {
          SEIContentLightLevelInfo* src = (SEIContentLightLevelInfo*)sei;
          vvdec::seiContentLightLevelInfo* t = new vvdec::seiContentLightLevelInfo;
          src->copyTo(*t);
          target.m_nestedSEIs.push_back( t );
        }
        break;
      case SEI::ALTERNATIVE_TRANSFER_CHARACTERISTICS :
        {
          SEIAlternativeTransferCharacteristics* src = (SEIAlternativeTransferCharacteristics*)sei;
          vvdec::seiAlternativeTransferCharacteristics* t = new vvdec::seiAlternativeTransferCharacteristics;
          src->copyTo(*t);
          target.m_nestedSEIs.push_back( t );
        }
        break;
      case SEI::AMBIENT_VIEWING_ENVIRONMENT          :
        {
          SEIAmbientViewingEnvironment* src = (SEIAmbientViewingEnvironment*)sei;
          vvdec::seiAmbientViewingEnvironment* t = new vvdec::seiAmbientViewingEnvironment;
          src->copyTo(*t);
          target.m_nestedSEIs.push_back( t );
        }
        break;

      case SEI::CONTENT_COLOUR_VOLUME                :
        {
          SEIContentColourVolume* src = (SEIContentColourVolume*)sei;
          vvdec::seiContentColourVolume* t = new vvdec::seiContentColourVolume;
          src->copyTo(*t);
          target.m_nestedSEIs.push_back( t );
        }
        break;

      default:
        break;
    }
  }
}

void SEIAlternativeTransferCharacteristics::copyTo (vvdec::seiAlternativeTransferCharacteristics& target) const
{
  target.m_preferredTransferCharacteristics = m_preferredTransferCharacteristics;
}

void SEIUserDataRegistered::copyTo (vvdec::seiUserDataRegistered& target) const
{
  target.m_ituCountryCode = m_ituCountryCode;
  std::copy(m_userData.begin(), m_userData.end(),
            std::back_inserter(target.m_userData));
}

void SEIFilmGrainCharacteristics::copyTo (vvdec::seiFilmGrainCharacteristics& target) const
{
  target.m_filmGrainCharacteristicsCancelFlag   = m_filmGrainCharacteristicsCancelFlag;
  target.m_filmGrainModelId                     = m_filmGrainModelId;
  target.m_separateColourDescriptionPresentFlag = m_separateColourDescriptionPresentFlag;
  target.m_filmGrainBitDepthLuma                = m_filmGrainBitDepthLumaMinus8+8;
  target.m_filmGrainBitDepthChroma              = m_filmGrainBitDepthChromaMinus8+8;
  target.m_filmGrainFullRangeFlag               = m_filmGrainFullRangeFlag;
  target.m_filmGrainColourPrimaries             = m_filmGrainColourPrimaries;
  target.m_filmGrainTransferCharacteristics     = m_filmGrainTransferCharacteristics;
  target.m_filmGrainMatrixCoeffs                = m_filmGrainMatrixCoeffs;
  target.m_blendingModeId                       = m_blendingModeId;
  target.m_log2ScaleFactor                      = m_log2ScaleFactor;

  ::memcpy(target.m_compModel, m_compModel, sizeof(m_compModel));

  target.m_filmGrainCharacteristicsPersistenceFlag = m_filmGrainCharacteristicsPersistenceFlag;
}

void SEIContentLightLevelInfo::copyTo (vvdec::seiContentLightLevelInfo& target) const
{
  target.m_maxContentLightLevel    = m_maxContentLightLevel;
  target.m_maxPicAverageLightLevel = m_maxPicAverageLightLevel;
}

void SEIAmbientViewingEnvironment::copyTo (vvdec::seiAmbientViewingEnvironment& target) const
{
  target.m_ambientIlluminance = m_ambientIlluminance;
  target.m_ambientLightX      = m_ambientLightX;
  target.m_ambientLightY      = m_ambientLightY;
}

void SEIContentColourVolume::copyTo (vvdec::seiContentColourVolume& target) const
{
  target.m_ccvCancelFlag = m_ccvCancelFlag;
  target.m_ccvPersistenceFlag = m_ccvPersistenceFlag;
  target.m_ccvPrimariesPresentFlag = m_ccvPrimariesPresentFlag;
  target.m_ccvMinLuminanceValuePresentFlag = m_ccvMinLuminanceValuePresentFlag;
  target.m_ccvMaxLuminanceValuePresentFlag = m_ccvMaxLuminanceValuePresentFlag;
  target.m_ccvAvgLuminanceValuePresentFlag = m_ccvAvgLuminanceValuePresentFlag;

  for( int i = 0; i < MAX_NUM_COMPONENT; i++ )
  {
    target.m_ccvPrimariesX[i] = m_ccvPrimariesX[i];
    target.m_ccvPrimariesY[i] = m_ccvPrimariesY[i];
  }
  target.m_ccvMinLuminanceValue = m_ccvMinLuminanceValue;
  target.m_ccvMaxLuminanceValue = m_ccvMaxLuminanceValue;
  target.m_ccvAvgLuminanceValue = m_ccvAvgLuminanceValue;
}

void SEISubpicureLevelInfo::copyTo (vvdec::seiSubpicureLevelInfo& target) const
{
  target.m_numRefLevels                = m_numRefLevels;
  target.m_explicitFractionPresentFlag = m_explicitFractionPresentFlag;
  target.m_cbrConstraintFlag           = m_cbrConstraintFlag;
  target.m_numSubpics                  = m_numSubpics;
#if JVET_S0176_SLI_SEI
  target.m_sliMaxSublayers             = m_sliMaxSublayers;
  target.m_sliSublayerInfoPresentFlag  = m_sliSublayerInfoPresentFlag;
#if JVET_S0098_SLI_FRACTION
  target.m_nonSubpicLayersFraction.resize(m_nonSubpicLayersFraction.size());
  for (size_t i = 0; i < m_nonSubpicLayersFraction.size(); i++)
  {
    target.m_nonSubpicLayersFraction[i].resize(m_nonSubpicLayersFraction[i].size());
    for (size_t k = 0; k < m_nonSubpicLayersFraction[i].size(); k++)
    {
      target.m_nonSubpicLayersFraction[i][k] = target.m_nonSubpicLayersFraction[i][k];
    }
  }
#endif

  target.m_refLevelIdc.resize(m_refLevelIdc.size());
  for (size_t i = 0; i < m_refLevelIdc.size(); i++)
  {
    target.m_refLevelIdc[i].resize(m_refLevelIdc[i].size());
    for (size_t k = 0; k < m_refLevelIdc[i].size(); k++)
    {
      target.m_refLevelIdc[i][k] = target.m_refLevelIdc[i][k];
    }
  }

  target.m_refLevelFraction.resize(m_refLevelFraction.size());
  for (size_t i = 0; i < m_refLevelFraction.size(); i++)
  {
    target.m_refLevelFraction[i].resize(m_refLevelFraction[i].size());
    for (size_t j = 0; j < m_refLevelFraction[i].size(); j++)
    {
      target.m_refLevelFraction[i][j].resize(m_refLevelFraction[i][j].size());
      for (size_t k = 0; k < m_refLevelFraction[i][j].size(); k++)
      {
        target.m_refLevelFraction[i][j][k] = target.m_refLevelFraction[i][j][k];
      }
    }
  }
#endif
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
