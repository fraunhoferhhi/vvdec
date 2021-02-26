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

#ifndef _VVDEC_SEI_H_
#define _VVDEC_SEI_H_

#include <stdio.h>
#include <stdint.h>

enum SEIPayloadType
{
  VVDEC_BUFFERING_PERIOD                     = 0,
  VVDEC_PICTURE_TIMING                       = 1,
  VVDEC_FILLER_PAYLOAD                       = 3,
  VVDEC_USER_DATA_REGISTERED_ITU_T_T35       = 4,
  VVDEC_USER_DATA_UNREGISTERED               = 5,
  VVDEC_FILM_GRAIN_CHARACTERISTICS           = 19,
  VVDEC_FRAME_PACKING                        = 45,
  VVDEC_PARAMETER_SETS_INCLUSION_INDICATION  = 129,
  VVDEC_DECODING_UNIT_INFO                   = 130,
  VVDEC_DECODED_PICTURE_HASH                 = 132,
  VVDEC_SCALABLE_NESTING                     = 133,
  VVDEC_MASTERING_DISPLAY_COLOUR_VOLUME      = 137,
  VVDEC_DEPENDENT_RAP_INDICATION             = 145,
  VVDEC_EQUIRECTANGULAR_PROJECTION           = 150,
  VVDEC_SPHERE_ROTATION                      = 154,
  VVDEC_REGION_WISE_PACKING                  = 155,
  VVDEC_OMNI_VIEWPORT                        = 156,
  VVDEC_GENERALIZED_CUBEMAP_PROJECTION       = 153,
  VVDEC_FRAME_FIELD_INFO                     = 168,
  VVDEC_SUBPICTURE_LEVEL_INFO                = 203,
  VVDEC_SAMPLE_ASPECT_RATIO_INFO             = 204,
  VVDEC_CONTENT_LIGHT_LEVEL_INFO             = 144,
  VVDEC_ALTERNATIVE_TRANSFER_CHARACTERISTICS = 147,
  VVDEC_AMBIENT_VIEWING_ENVIRONMENT          = 148,
  VVDEC_CONTENT_COLOUR_VOLUME                = 149,
  VVDEC_SEI_UNKNOWN                          = -1,
};
typedef enum SEIPayloadType SEIPayloadType;

enum Level
{
  VVDEC_LEVEL_NONE = 0,
  VVDEC_LEVEL1   = 16,
  VVDEC_LEVEL2   = 32,
  VVDEC_LEVEL2_1 = 35,
  VVDEC_LEVEL3   = 48,
  VVDEC_LEVEL3_1 = 51,
  VVDEC_LEVEL4   = 64,
  VVDEC_LEVEL4_1 = 67,
  VVDEC_LEVEL5   = 80,
  VVDEC_LEVEL5_1 = 83,
  VVDEC_LEVEL5_2 = 86,
  VVDEC_LEVEL6   = 96,
  VVDEC_LEVEL6_1 = 99,
  VVDEC_LEVEL6_2 = 102,
  VVDEC_LEVEL15_5 = 255,
};
typedef enum Level Level;


enum HashType
{
  VVDEC_HASHTYPE_MD5             = 0,
  VVDEC_HASHTYPE_CRC             = 1,
  VVDEC_HASHTYPE_CHECKSUM        = 2,
  VVDEC_HASHTYPE_NONE            = 3,
  VVDEC_NUMBER_OF_HASHTYPES      = 4
};
typedef enum HashType HashType;


typedef struct vvdecSEI
{
    SEIPayloadType            payloadType;     /* payload type as defined in sei.h */
    unsigned int              size;            /* size of payload in bytes */
    void                     *payload;         /* payload structure as defined in sei.h */
}vvdecSEI;

typedef struct vvdecSEIBufferingPeriod
{
  bool     m_bpNalCpbParamsPresentFlag;
  bool     m_bpVclCpbParamsPresentFlag;
  uint32_t m_initialCpbRemovalDelayLength;
  uint32_t m_cpbRemovalDelayLength;
  uint32_t m_dpbOutputDelayLength;
  int      m_bpCpbCnt;
  uint32_t m_duCpbRemovalDelayIncrementLength;
  uint32_t m_dpbOutputDelayDuLength;
  uint32_t m_initialCpbRemovalDelay [7][32][2];
  uint32_t m_initialCpbRemovalOffset[7][32][2];
  bool     m_concatenationFlag;
  uint32_t m_auCpbRemovalDelayDelta;
  bool     m_cpbRemovalDelayDeltasPresentFlag;
  int      m_numCpbRemovalDelayDeltas;
  int      m_bpMaxSubLayers;
  uint32_t m_cpbRemovalDelayDelta[15];
  bool     m_bpDecodingUnitHrdParamsPresentFlag;
  bool     m_decodingUnitCpbParamsInPicTimingSeiFlag;
  bool     m_decodingUnitDpbDuParamsInPicTimingSeiFlag;
  bool     m_sublayerInitialCpbRemovalDelayPresentFlag;
  bool     m_additionalConcatenationInfoPresentFlag;
  uint32_t m_maxInitialRemovalDelayForConcatenation;
  bool     m_sublayerDpbOutputOffsetsPresentFlag;
  uint32_t m_dpbOutputTidOffset[7];
  bool     m_altCpbParamsPresentFlag;
  bool     m_useAltCpbParamsFlag;
} vvdecSEIBufferingPeriod;


typedef struct vvdecSEIPictureTiming
{
  bool     m_ptSubLayerDelaysPresentFlag[7];
  bool     m_cpbRemovalDelayDeltaEnabledFlag[7];
  uint32_t m_cpbRemovalDelayDeltaIdx[7];
  uint32_t m_auCpbRemovalDelay[7];
  uint32_t m_picDpbOutputDelay;
  uint32_t m_picDpbOutputDuDelay;
  uint32_t m_numDecodingUnits;
  bool     m_duCommonCpbRemovalDelayFlag;
  uint32_t m_duCommonCpbRemovalDelay[7];
  uint32_t m_numNalusInDu[32];
  uint32_t m_duCpbRemovalDelay[32*7+7];
  bool     m_cpbAltTimingInfoPresentFlag;
  uint32_t m_nalCpbAltInitialRemovalDelayDelta[7][32];
  uint32_t m_nalCpbAltInitialRemovalOffsetDelta[7][32];
  uint32_t m_nalCpbDelayOffset[7];
  uint32_t m_nalDpbDelayOffset[7];
  uint32_t m_vclCpbAltInitialRemovalDelayDelta[7][32];
  uint32_t m_vclCpbAltInitialRemovalOffsetDelta[7][32];
  uint32_t m_vclCpbDelayOffset[7];
  uint32_t m_vclDpbDelayOffset[7];
  int      m_ptDisplayElementalPeriods;
} vvdecSEIPictureTiming;


typedef struct vvdecSEIUserDataRegistered
{
  uint16_t  ituCountryCode;
  uint32_t  userDataLength;
  uint8_t  *userData;
} vvdecSEIUserDataRegistered;

typedef struct vvdecSEIUserDataUnregistered
{
  uint8_t   uuid_iso_iec_11578[16];
  uint32_t  userDataLength;
  uint8_t  *userData;
} vvdecSEIUserDataUnregistered;


typedef struct vvdecCompModelIntensityValues
{
  uint8_t intensityIntervalLowerBound;
  uint8_t intensityIntervalUpperBound;
  int     compModelValue[6];
}vvdecCompModelIntensityValues;

typedef struct vvdecCompModel
{
  bool                          presentFlag;
  uint8_t                       numModelValues;
  vvdecCompModelIntensityValues intensityValues[265];
}vvdecCompModel;

typedef struct vvdecSEIFilmGrainCharacteristics
{
  bool             m_filmGrainCharacteristicsCancelFlag;
  uint8_t          m_filmGrainModelId;
  bool             m_separateColourDescriptionPresentFlag;
  uint8_t          m_filmGrainBitDepthLuma;
  uint8_t          m_filmGrainBitDepthChroma;
  bool             m_filmGrainFullRangeFlag;
  uint8_t          m_filmGrainColourPrimaries;
  uint8_t          m_filmGrainTransferCharacteristics;
  uint8_t          m_filmGrainMatrixCoeffs;
  uint8_t          m_blendingModeId;
  uint8_t          m_log2ScaleFactor;
  vvdecCompModel   m_compModel[3];
  bool             m_filmGrainCharacteristicsPersistenceFlag;
}vvdecSEIFilmGrainCharacteristics;

typedef struct vvdecSEIFramePacking
{
  int  m_arrangementId;
  bool m_arrangementCancelFlag;
  int  m_arrangementType;
  bool m_quincunxSamplingFlag;
  int  m_contentInterpretationType;
  bool m_spatialFlippingFlag;
  bool m_frame0FlippedFlag;
  bool m_fieldViewsFlag;
  bool m_currentFrameIsFrame0Flag;
  bool m_frame0SelfContainedFlag;
  bool m_frame1SelfContainedFlag;
  int  m_frame0GridPositionX;
  int  m_frame0GridPositionY;
  int  m_frame1GridPositionX;
  int  m_frame1GridPositionY;
  int  m_arrangementReservedByte;
  bool m_arrangementPersistenceFlag;
  bool m_upsampledAspectRatio;
}vvdecSEIFramePacking;

typedef struct vvdecSEIParameterSetsInclusionIndication
{
  int m_selfContainedClvsFlag;
}vvdecSEIParameterSetsInclusionIndication;

typedef struct vvdecSEIDecodingUnitInfo
{
  int  m_decodingUnitIdx;
  bool m_duiSubLayerDelaysPresentFlag[7];
  int  m_duSptCpbRemovalDelayIncrement[7];
  bool m_dpbOutputDuDelayPresentFlag;
  int  m_picSptDpbOutputDuDelay;
}vvdecSEIDecodingUnitInfo;


typedef struct vvdecSEIDecodedPictureHash
{
  HashType      method;
  bool          singleCompFlag;
  int           digist_length;
  unsigned char digest[16*3];
}vvdecSEIDecodedPictureHash;


typedef struct vvdecSEIScalableNesting
{
  bool      m_snOlsFlag;
  bool      m_snSubpicFlag;
  uint32_t  m_snNumOlss;
  uint32_t  m_snOlsIdxDelta[64];
  uint32_t  m_snOlsIdx[64];
  bool      m_snAllLayersFlag;
  uint32_t  m_snNumLayers;
  uint8_t   m_snLayerId[64];
  uint32_t  m_snNumSubpics;
  uint8_t   m_snSubpicIdLen;
  uint16_t  m_snSubpicId[64];
  uint32_t  m_snNumSEIs;

  vvdecSEI* m_nestedSEIs[64];
}vvdecSEIScalableNesting;

typedef struct vvdecSEIMasteringDisplayColourVolume
{
    uint32_t maxLuminance;
    uint32_t minLuminance;
    uint16_t primaries[3][2];
    uint16_t whitePoint[2];
}vvdecSEIMasteringDisplayColourVolume;

typedef struct vvdecSEIDependentRapIndication
{
} vvdecSEIDependentRapIndication;

typedef struct vvdecSEIEquirectangularProjection
{
  bool    m_erpCancelFlag;
  bool    m_erpPersistenceFlag;
  bool    m_erpGuardBandFlag;
  uint8_t m_erpGuardBandType;
  uint8_t m_erpLeftGuardBandWidth;
  uint8_t m_erpRightGuardBandWidth;
}vvdecSEIEquirectangularProjection;

typedef struct vvdecSEISphereRotation
{
  bool  m_sphereRotationCancelFlag;
  bool  m_sphereRotationPersistenceFlag;
  int   m_sphereRotationYaw;
  int   m_sphereRotationPitch;
  int   m_sphereRotationRoll;
}vvdecSEISphereRotation;

typedef struct vvdecSEIRegionWisePacking
{
  bool     m_rwpCancelFlag;
  bool     m_rwpPersistenceFlag;
  bool     m_constituentPictureMatchingFlag;
  int      m_numPackedRegions;
  int      m_projPictureWidth;
  int      m_projPictureHeight;
  int      m_packedPictureWidth;
  int      m_packedPictureHeight;
  uint8_t  m_rwpTransformType[265];
  bool     m_rwpGuardBandFlag[265];
  uint32_t m_projRegionWidth[265];
  uint32_t m_projRegionHeight[265];
  uint32_t m_rwpProjRegionTop[265];
  uint32_t m_projRegionLeft[265];
  uint16_t m_packedRegionWidth[265];
  uint16_t m_packedRegionHeight[265];
  uint16_t m_packedRegionTop[265];
  uint16_t m_packedRegionLeft[265];
  uint8_t  m_rwpLeftGuardBandWidth[265];
  uint8_t  m_rwpRightGuardBandWidth[265];
  uint8_t  m_rwpTopGuardBandHeight[265];
  uint8_t  m_rwpBottomGuardBandHeight[265];
  bool     m_rwpGuardBandNotUsedForPredFlag[265];
  uint8_t  m_rwpGuardBandType[4*265];
}vvdecSEIRegionWisePacking;

typedef struct vvdecOmniViewportRegion
{
  int      azimuthCentre;
  int      elevationCentre;
  int      tiltCentre;
  uint32_t horRange;
  uint32_t verRange;
}vvdecOmniViewportRegion;

typedef struct vvdecSEIOmniViewport
{
  uint32_t                m_omniViewportId;
  bool                    m_omniViewportCancelFlag;
  bool                    m_omniViewportPersistenceFlag;
  uint8_t                 m_omniViewportCnt;
  vvdecOmniViewportRegion m_omniViewportRegions[16];
}vvdecSEIOmniViewport;


typedef struct vvdecSEIGeneralizedCubemapProjection
{
  bool    m_gcmpCancelFlag;
  bool    m_gcmpPersistenceFlag;
  uint8_t m_gcmpPackingType;
  uint8_t m_gcmpMappingFunctionType;
  uint8_t m_gcmpFaceIndex[6];
  uint8_t m_gcmpFaceRotation[6];
  uint8_t m_gcmpFunctionCoeffU[6];
  bool    m_gcmpFunctionUAffectedByVFlag[6];
  uint8_t m_gcmpFunctionCoeffV[6];
  bool    m_gcmpFunctionVAffectedByUFlag[6];
  bool    m_gcmpGuardBandFlag;
  uint8_t m_gcmpGuardBandType;
  bool    m_gcmpGuardBandBoundaryExteriorFlag;
  uint8_t m_gcmpGuardBandSamples;
}vvdecSEIGeneralizedCubemapProjection;

typedef struct vvdecSEIFrameFieldInfo
{
  bool m_fieldPicFlag;
  bool m_bottomFieldFlag;
  bool m_pairingIndicatedFlag;
  bool m_pairedWithNextFieldFlag;
  bool m_displayFieldsFromFrameFlag;
  bool m_topFieldFirstFlag;
  int  m_displayElementalPeriods;
  int  m_sourceScanType;
  bool m_duplicateFlag;
}vvdecSEIFrameFieldInfo;

typedef struct vvdecSEISubpictureLevelInfo
{
  int       m_numRefLevels;
  bool      m_explicitFractionPresentFlag;
  bool      m_cbrConstraintFlag;
  int       m_numSubpics;
  int       m_sliMaxSublayers;
  bool      m_sliSublayerInfoPresentFlag;
  int       m_nonSubpicLayersFraction[6][6];
  Level     m_refLevelIdc[6][6];
  int       m_refLevelFraction[6][64][6];
}vvdecSEISubpictureLevelInfo;

typedef struct vvdecSEISampleAspectRatioInfo
{
  bool    m_sariCancelFlag;
  bool    m_sariPersistenceFlag;
  int     m_sariAspectRatioIdc;
  int     m_sariSarWidth;
  int     m_sariSarHeight;
}vvdecSEISampleAspectRatioInfo;

typedef struct vvdecSEIContentLightLevelInfo
{
  uint16_t m_maxContentLightLevel;
  uint16_t m_maxPicAverageLightLevel;
}vvdecSEIContentLightLevelInfo;

typedef struct vvdecSEIAlternativeTransferCharacteristics
{
  uint8_t preferred_transfer_characteristics;
}vvdecSEIAlternativeTransferCharacteristics;

typedef struct vvdecSEIAmbientViewingEnvironment
{
  uint32_t m_ambientIlluminance;
  uint16_t m_ambientLightX;
  uint16_t m_ambientLightY;
}vvdecSEIAmbientViewingEnvironment;

typedef struct vvdecSEIContentColourVolume
{
  bool      m_ccvCancelFlag;
  bool      m_ccvPersistenceFlag;
  bool      m_ccvPrimariesPresentFlag;
  bool      m_ccvMinLuminanceValuePresentFlag;
  bool      m_ccvMaxLuminanceValuePresentFlag;
  bool      m_ccvAvgLuminanceValuePresentFlag;
  int       m_ccvPrimariesX[3];
  int       m_ccvPrimariesY[3];
  uint32_t  m_ccvMinLuminanceValue;
  uint32_t  m_ccvMaxLuminanceValue;
  uint32_t  m_ccvAvgLuminanceValue;
}vvdecSEIContentColourVolume;

#endif /*_VVDEC_SEI_H_*/
