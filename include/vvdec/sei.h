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


typedef struct vvdec_sei_message
{
    struct vvdec_sei_message* next_sei;        /*!< next SEI in AU */
    SEIPayloadType            payloadType;     /*!< payload type as defined in sei.h */
    unsigned int              size;            /*!< size of payload in bytes */
    void                     *payload;         /*!< payload structure as defined in sei.h */
}vvdec_sei_message_t;

typedef struct
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
} vvdec_sei_buffering_period_t;


typedef struct
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
  uint32_t m_numNalusInDu[666];
  uint32_t m_duCpbRemovalDelay[666];
  bool     m_cpbAltTimingInfoPresentFlag;
  uint32_t m_nalCpbAltInitialRemovalDelayDelta[666][666];
  uint32_t m_nalCpbAltInitialRemovalOffsetDelta[666][666];
  uint32_t m_nalCpbDelayOffset[666];
  uint32_t m_nalDpbDelayOffset[666];
  uint32_t m_vclCpbAltInitialRemovalDelayDelta[666][666];
  uint32_t m_vclCpbAltInitialRemovalOffsetDelta[666][666];
  uint32_t m_vclCpbDelayOffset[666];
  uint32_t m_vclDpbDelayOffset[666];
  int      m_ptDisplayElementalPeriods;
} vvdec_sei_picture_timing_t;


typedef struct
{
  uint16_t  ituCountryCode;
  uint32_t  userDataLength;
  uint8_t  *userData;
} vvdec_sei_user_data_registered_t;

typedef struct
{
  uint8_t   uuid_iso_iec_11578[16];
  uint32_t  userDataLength;
  uint8_t  *userData;
} vvdec_sei_user_data_unregistered_t;


typedef struct
{
  uint8_t intensityIntervalLowerBound;
  uint8_t intensityIntervalUpperBound;
  int     compModelValue[6];
}vvdec_CompModelIntensityValues_t;

typedef struct
{
  bool                             presentFlag;
  uint8_t                          numModelValues;
  vvdec_CompModelIntensityValues_t intensityValues[265];
}vvdec_CompModel_t;

typedef struct
{
  bool        m_filmGrainCharacteristicsCancelFlag;
  uint8_t     m_filmGrainModelId;
  bool        m_separateColourDescriptionPresentFlag;
  uint8_t     m_filmGrainBitDepthLuma;
  uint8_t     m_filmGrainBitDepthChroma;
  bool        m_filmGrainFullRangeFlag;
  uint8_t     m_filmGrainColourPrimaries;
  uint8_t     m_filmGrainTransferCharacteristics;
  uint8_t     m_filmGrainMatrixCoeffs;
  uint8_t     m_blendingModeId;
  uint8_t     m_log2ScaleFactor;

  vvdec_CompModel_t m_compModel[3];
  bool        m_filmGrainCharacteristicsPersistenceFlag;
}vvdec_sei_film_grain_characteristics_t;

typedef struct
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
}vvdec_sei_frame_packing_t;

typedef struct
{
  int m_selfContainedClvsFlag;
}vvdec_sei_parameter_sets_inclusion_indication_t;

typedef struct
{
  int  m_decodingUnitIdx;
  bool m_duiSubLayerDelaysPresentFlag[7];
  int  m_duSptCpbRemovalDelayIncrement[7];
  bool m_dpbOutputDuDelayPresentFlag;
  int  m_picSptDpbOutputDuDelay;
}vvdec_sei_decoding_unit_info_t;


typedef struct
{
  HashType      method;
  bool          singleCompFlag;
  unsigned char digest[3][16];
}vvdec_sei_decoded_picture_hash_t;


typedef struct
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
  uint16_t  m_snSubpicId[600];
  uint32_t  m_snNumSEIs;

  vvdec_sei_message m_nestedSEIs[64];
}vvdec_sei_scalable_nesting_t;

typedef struct
{
    uint32_t maxLuminance;
    uint32_t minLuminance;
    uint16_t primaries[3][2];
    uint16_t whitePoint[2];
}vvdec_sei_mastering_display_colour_volume_t;

typedef struct
{
} vvdec_sei_dependent_rap_indication_t;

typedef struct
{
  bool    m_erpCancelFlag;
  bool    m_erpPersistenceFlag;
  bool    m_erpGuardBandFlag;
  uint8_t m_erpGuardBandType;
  uint8_t m_erpLeftGuardBandWidth;
  uint8_t m_erpRightGuardBandWidth;
}vvdec_sei_equirectangular_projection_t;

typedef struct
{
  bool  m_sphereRotationCancelFlag;
  bool  m_sphereRotationPersistenceFlag;
  int   m_sphereRotationYaw;
  int   m_sphereRotationPitch;
  int   m_sphereRotationRoll;
}vvdec_sei_sphere_rotation_t;

typedef struct
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
}vvdec_sei_region_wise_packing_t;

typedef struct
{
  int      azimuthCentre;
  int      elevationCentre;
  int      tiltCentre;
  uint32_t horRange;
  uint32_t verRange;
}vvdec_OmniViewport;

typedef struct
{
  uint32_t           m_omniViewportId;
  bool               m_omniViewportCancelFlag;
  bool               m_omniViewportPersistenceFlag;
  uint8_t            m_omniViewportCnt;
  vvdec_OmniViewport m_omniViewportRegions[16];
}vvdec_sei_omni_viewport_t;


typedef struct
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
}vvdec_sei_generalized_cubemap_projection_t;

typedef struct
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
}vvdec_sei_frame_field_info_t;

typedef struct
{
  int       m_numRefLevels;
  bool      m_explicitFractionPresentFlag;
  bool      m_cbrConstraintFlag;
  int       m_numSubpics;
  int       m_sliMaxSublayers;
  bool      m_sliSublayerInfoPresentFlag;
  int       m_nonSubpicLayersFraction[6][6];
  Level     m_refLevelIdc[6][6];
  Level     m_refLevelFraction[6][600][6];
}vvdec_sei_subpicture_level_info_t;

typedef struct
{
  bool    m_sariCancelFlag;
  bool    m_sariPersistenceFlag;
  int     m_sariAspectRatioIdc;
  int     m_sariSarWidth;
  int     m_sariSarHeight;
}vvdec_sei_sample_aspect_ratio_info_t;


typedef struct
{
  uint16_t m_maxContentLightLevel;
  uint16_t m_maxPicAverageLightLevel;
}vvdec_sei_content_light_level_info_t;

typedef struct
{
  uint8_t preferred_transfer_characteristics;
}vvdec_sei_alternative_transfer_characteristics_t;

typedef struct
{
  uint32_t m_ambientIlluminance;
  uint16_t m_ambientLightX;
  uint16_t m_ambientLightY;
}vvdec_sei_ambient_viewing_environment_t;


typedef struct
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
}vvdec_sei_content_colour_volume_t;

#endif /*_VVDEC_SEI_H_*/
