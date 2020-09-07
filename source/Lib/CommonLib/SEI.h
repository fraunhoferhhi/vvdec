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

#ifndef __SEI__
#define __SEI__

#pragma once
#include <list>
#include <vector>
#include <cstring>

#include "CommonDef.h"
#include "libmd5/MD5.h"

//! \ingroup CommonLib
//! \{
class SPS;

/**
 * Abstract class representing an SEI message with lightweight RTTI.
 */
class SEI
{
public:
  enum PayloadType
  {
    BUFFERING_PERIOD                     = 0,
    PICTURE_TIMING                       = 1,
    FILLER_PAYLOAD                       = 3,
    USER_DATA_REGISTERED_ITU_T_T35       = 4,
    USER_DATA_UNREGISTERED               = 5,
    FILM_GRAIN_CHARACTERISTICS           = 19,
    FRAME_PACKING                        = 45,
    PARAMETER_SETS_INCLUSION_INDICATION  = 129,
    DECODING_UNIT_INFO                   = 130,
    DECODED_PICTURE_HASH                 = 132,
    SCALABLE_NESTING                     = 133,
    MASTERING_DISPLAY_COLOUR_VOLUME      = 137,
    DEPENDENT_RAP_INDICATION             = 145,
    EQUIRECTANGULAR_PROJECTION           = 150,
    SPHERE_ROTATION                      = 154,
    REGION_WISE_PACKING                  = 155,
    OMNI_VIEWPORT                        = 156,
    GENERALIZED_CUBEMAP_PROJECTION       = 153,
    FRAME_FIELD_INFO                     = 168,
    SUBPICTURE_LEVEL_INFO                = 203,
    SAMPLE_ASPECT_RATIO_INFO             = 204,
    CONTENT_LIGHT_LEVEL_INFO             = 144,
    ALTERNATIVE_TRANSFER_CHARACTERISTICS = 147,
    AMBIENT_VIEWING_ENVIRONMENT          = 148,
    CONTENT_COLOUR_VOLUME                = 149,
  };

  SEI() {}
  virtual ~SEI() {}

  static const char *getSEIMessageString(SEI::PayloadType payloadType);

  virtual PayloadType payloadType() const = 0;
};

class SEIEquirectangularProjection : public SEI
{
public:
  PayloadType payloadType() const { return EQUIRECTANGULAR_PROJECTION; }

  SEIEquirectangularProjection()  {}
  virtual ~SEIEquirectangularProjection() {}

  bool    m_erpCancelFlag;
  bool    m_erpPersistenceFlag;
  bool    m_erpGuardBandFlag;
  uint8_t m_erpGuardBandType;
  uint8_t m_erpLeftGuardBandWidth;
  uint8_t m_erpRightGuardBandWidth;
};

class SEISphereRotation : public SEI
{
public:
  PayloadType payloadType() const { return SPHERE_ROTATION; }

  SEISphereRotation()  {}
  virtual ~SEISphereRotation() {}

  bool  m_sphereRotationCancelFlag;
  bool  m_sphereRotationPersistenceFlag;
  int   m_sphereRotationYaw;
  int   m_sphereRotationPitch;
  int   m_sphereRotationRoll;
};

class SEIOmniViewport : public SEI
{
public:
  PayloadType payloadType() const { return OMNI_VIEWPORT; }

  SEIOmniViewport() {}
  virtual ~SEIOmniViewport() {}

  struct OmniViewport
  {
    int      azimuthCentre;
    int      elevationCentre;
    int      tiltCentre;
    uint32_t horRange;
    uint32_t verRange;
  };

  uint32_t m_omniViewportId;
  bool     m_omniViewportCancelFlag;
  bool     m_omniViewportPersistenceFlag;
  uint8_t  m_omniViewportCntMinus1;
  std::vector<OmniViewport> m_omniViewportRegions;
};

class SEIRegionWisePacking : public SEI
{
public:
  PayloadType payloadType() const { return REGION_WISE_PACKING; }
  SEIRegionWisePacking() {}
  virtual ~SEIRegionWisePacking() {}
  bool                  m_rwpCancelFlag;
  bool                  m_rwpPersistenceFlag;
  bool                  m_constituentPictureMatchingFlag;
  int                   m_numPackedRegions;
  int                   m_projPictureWidth;
  int                   m_projPictureHeight;
  int                   m_packedPictureWidth;
  int                   m_packedPictureHeight;
  std::vector<uint8_t>  m_rwpTransformType;
  std::vector<bool>     m_rwpGuardBandFlag;
  std::vector<uint32_t> m_projRegionWidth;
  std::vector<uint32_t> m_projRegionHeight;
  std::vector<uint32_t> m_rwpProjRegionTop;
  std::vector<uint32_t> m_projRegionLeft;
  std::vector<uint16_t> m_packedRegionWidth;
  std::vector<uint16_t> m_packedRegionHeight;
  std::vector<uint16_t> m_packedRegionTop;
  std::vector<uint16_t> m_packedRegionLeft;
  std::vector<uint8_t>  m_rwpLeftGuardBandWidth;
  std::vector<uint8_t>  m_rwpRightGuardBandWidth;
  std::vector<uint8_t>  m_rwpTopGuardBandHeight;
  std::vector<uint8_t>  m_rwpBottomGuardBandHeight;
  std::vector<bool>     m_rwpGuardBandNotUsedForPredFlag;
  std::vector<uint8_t>  m_rwpGuardBandType;
};

class SEIGeneralizedCubemapProjection : public SEI
{
public:
  PayloadType payloadType() const { return GENERALIZED_CUBEMAP_PROJECTION; }

  SEIGeneralizedCubemapProjection()  {}
  virtual ~SEIGeneralizedCubemapProjection() {}

  bool                 m_gcmpCancelFlag;
  bool                 m_gcmpPersistenceFlag;
  uint8_t              m_gcmpPackingType;
  uint8_t              m_gcmpMappingFunctionType;
  std::vector<uint8_t> m_gcmpFaceIndex;
  std::vector<uint8_t> m_gcmpFaceRotation;
  std::vector<uint8_t> m_gcmpFunctionCoeffU;
  std::vector<bool>    m_gcmpFunctionUAffectedByVFlag;
  std::vector<uint8_t> m_gcmpFunctionCoeffV;
  std::vector<bool>    m_gcmpFunctionVAffectedByUFlag;
  bool                 m_gcmpGuardBandFlag;
  uint8_t              m_gcmpGuardBandType;
  bool                 m_gcmpGuardBandBoundaryExteriorFlag;
  uint8_t              m_gcmpGuardBandSamplesMinus1;
};

class SEISampleAspectRatioInfo : public SEI
{
public:
  PayloadType payloadType() const { return SAMPLE_ASPECT_RATIO_INFO; }
  SEISampleAspectRatioInfo() {}
  virtual ~SEISampleAspectRatioInfo() {}
  bool                  m_sariCancelFlag;
  bool                  m_sariPersistenceFlag;
  int                   m_sariAspectRatioIdc;
  int                   m_sariSarWidth;
  int                   m_sariSarHeight;
};

static const uint32_t ISO_IEC_11578_LEN=16;

class SEIuserDataUnregistered : public SEI
{
public:
  PayloadType payloadType() const { return USER_DATA_UNREGISTERED; }

  SEIuserDataUnregistered()
    : userData(0)
    {}

  virtual ~SEIuserDataUnregistered()
  {
    delete userData;
  }

  uint8_t uuid_iso_iec_11578[ISO_IEC_11578_LEN];
  uint32_t  userDataLength;
  uint8_t *userData;
};

class SEIDecodedPictureHash : public SEI
{
public:
  PayloadType payloadType() const { return DECODED_PICTURE_HASH; }

  SEIDecodedPictureHash() {}
  virtual ~SEIDecodedPictureHash() {}

  HashType method;

  PictureHash m_pictureHash;
};

class SEIDependentRAPIndication : public SEI
{
public:
  PayloadType payloadType() const { return DEPENDENT_RAP_INDICATION; }
  SEIDependentRAPIndication() { }

  virtual ~SEIDependentRAPIndication() { }
};

class SEIBufferingPeriod : public SEI
{
public:
  PayloadType payloadType() const { return BUFFERING_PERIOD; }
  void copyTo (SEIBufferingPeriod& target) const;

  SEIBufferingPeriod()
  : m_bpNalCpbParamsPresentFlag (false)
  , m_bpVclCpbParamsPresentFlag (false)
  , m_initialCpbRemovalDelayLength (0)
  , m_cpbRemovalDelayLength (0)
  , m_dpbOutputDelayLength (0)
  , m_bpCpbCnt(0)
  , m_duCpbRemovalDelayIncrementLength (0)
  , m_dpbOutputDelayDuLength (0)
  , m_cpbRemovalDelayDeltasPresentFlag (false)
  , m_numCpbRemovalDelayDeltas (0)
  , m_bpMaxSubLayers (0)
  , m_bpDecodingUnitHrdParamsPresentFlag (false)
  , m_decodingUnitCpbParamsInPicTimingSeiFlag (false)
  , m_decodingUnitDpbDuParamsInPicTimingSeiFlag(false)
  , m_sublayerInitialCpbRemovalDelayPresentFlag(false)
  , m_additionalConcatenationInfoPresentFlag (false)
  , m_maxInitialRemovalDelayForConcatenation (0)
    , m_sublayerDpbOutputOffsetsPresentFlag (false)
  , m_altCpbParamsPresentFlag (false)
  , m_useAltCpbParamsFlag (false)
  {
    ::memset(m_initialCpbRemovalDelay, 0, sizeof(m_initialCpbRemovalDelay));
    ::memset(m_initialCpbRemovalOffset, 0, sizeof(m_initialCpbRemovalOffset));
    ::memset(m_cpbRemovalDelayDelta, 0, sizeof(m_cpbRemovalDelayDelta));
    ::memset(m_dpbOutputTidOffset, 0, sizeof(m_dpbOutputTidOffset));
  }
  virtual ~SEIBufferingPeriod() {}

  void      setDuCpbRemovalDelayIncrementLength( uint32_t value )        { m_duCpbRemovalDelayIncrementLength = value;        }
  uint32_t  getDuCpbRemovalDelayIncrementLength( ) const                 { return m_duCpbRemovalDelayIncrementLength;         }
  void      setDpbOutputDelayDuLength( uint32_t value )                  { m_dpbOutputDelayDuLength = value;                  }
  uint32_t  getDpbOutputDelayDuLength( ) const                           { return m_dpbOutputDelayDuLength;                   }
  bool m_bpNalCpbParamsPresentFlag;
  bool m_bpVclCpbParamsPresentFlag;
  uint32_t m_initialCpbRemovalDelayLength;
  uint32_t m_cpbRemovalDelayLength;
  uint32_t m_dpbOutputDelayLength;
  int      m_bpCpbCnt;
  uint32_t m_duCpbRemovalDelayIncrementLength;
  uint32_t m_dpbOutputDelayDuLength;
  uint32_t m_initialCpbRemovalDelay         [MAX_TLAYER][MAX_CPB_CNT][2];
  uint32_t m_initialCpbRemovalOffset        [MAX_TLAYER][MAX_CPB_CNT][2];
  bool m_concatenationFlag;
  uint32_t m_auCpbRemovalDelayDelta;
  bool m_cpbRemovalDelayDeltasPresentFlag;
  int  m_numCpbRemovalDelayDeltas;
  int  m_bpMaxSubLayers;
  uint32_t m_cpbRemovalDelayDelta    [15];
  bool m_bpDecodingUnitHrdParamsPresentFlag;
  bool m_decodingUnitCpbParamsInPicTimingSeiFlag;
  bool m_decodingUnitDpbDuParamsInPicTimingSeiFlag;
  bool m_sublayerInitialCpbRemovalDelayPresentFlag;
  bool     m_additionalConcatenationInfoPresentFlag;
  uint32_t m_maxInitialRemovalDelayForConcatenation;
  bool     m_sublayerDpbOutputOffsetsPresentFlag;
  uint32_t m_dpbOutputTidOffset      [MAX_TLAYER];
  bool     m_altCpbParamsPresentFlag;
  bool     m_useAltCpbParamsFlag;
};

class SEIPictureTiming : public SEI
{
public:
  PayloadType payloadType() const { return PICTURE_TIMING; }
  void copyTo (SEIPictureTiming& target) const;

  SEIPictureTiming()
  : m_picDpbOutputDelay (0)
  , m_picDpbOutputDuDelay (0)
  , m_numDecodingUnitsMinus1 (0)
  , m_duCommonCpbRemovalDelayFlag (false)
  , m_cpbAltTimingInfoPresentFlag (false)
#if JVET_Q0818_PT_SEI
  , m_ptDisplayElementalPeriodsMinus1(0)
#endif
  {
    ::memset(m_ptSubLayerDelaysPresentFlag, 0, sizeof(m_ptSubLayerDelaysPresentFlag));
    ::memset(m_duCommonCpbRemovalDelayMinus1, 0, sizeof(m_duCommonCpbRemovalDelayMinus1));
    ::memset(m_cpbRemovalDelayDeltaEnabledFlag, 0, sizeof(m_cpbRemovalDelayDeltaEnabledFlag));
    ::memset(m_cpbRemovalDelayDeltaIdx, 0, sizeof(m_cpbRemovalDelayDeltaIdx));
    ::memset(m_auCpbRemovalDelay, 0, sizeof(m_auCpbRemovalDelay));
  }
  virtual ~SEIPictureTiming()
  {
  }


  bool  m_ptSubLayerDelaysPresentFlag[MAX_TLAYER];
  bool  m_cpbRemovalDelayDeltaEnabledFlag[MAX_TLAYER];
  uint32_t  m_cpbRemovalDelayDeltaIdx[MAX_TLAYER];
  uint32_t  m_auCpbRemovalDelay[MAX_TLAYER];
  uint32_t  m_picDpbOutputDelay;
  uint32_t  m_picDpbOutputDuDelay;
  uint32_t  m_numDecodingUnitsMinus1;
  bool  m_duCommonCpbRemovalDelayFlag;
  uint32_t  m_duCommonCpbRemovalDelayMinus1[MAX_TLAYER];
  std::vector<uint32_t> m_numNalusInDuMinus1;
  std::vector<uint32_t> m_duCpbRemovalDelayMinus1;
  bool     m_cpbAltTimingInfoPresentFlag;
  std::vector<std::vector<uint32_t>> m_nalCpbAltInitialRemovalDelayDelta;
  std::vector<std::vector<uint32_t>> m_nalCpbAltInitialRemovalOffsetDelta;
  std::vector<uint32_t>              m_nalCpbDelayOffset;
  std::vector<uint32_t>              m_nalDpbDelayOffset;
  std::vector<std::vector<uint32_t>> m_vclCpbAltInitialRemovalDelayDelta;
  std::vector<std::vector<uint32_t>> m_vclCpbAltInitialRemovalOffsetDelta;
  std::vector<uint32_t>              m_vclCpbDelayOffset;
  std::vector<uint32_t>              m_vclDpbDelayOffset;
#if JVET_Q0818_PT_SEI
  int m_ptDisplayElementalPeriodsMinus1;
#endif
};

class SEIDecodingUnitInfo : public SEI
{
public:
  PayloadType payloadType() const { return DECODING_UNIT_INFO; }

  SEIDecodingUnitInfo()
    : m_decodingUnitIdx(0)
    , m_dpbOutputDuDelayPresentFlag(false)
    , m_picSptDpbOutputDuDelay(0)
  {
    ::memset(m_duiSubLayerDelaysPresentFlag, 0, sizeof(m_duiSubLayerDelaysPresentFlag));
    ::memset(m_duSptCpbRemovalDelayIncrement, 0, sizeof(m_duSptCpbRemovalDelayIncrement));
  }
  virtual ~SEIDecodingUnitInfo() {}
  int m_decodingUnitIdx;
  bool m_duiSubLayerDelaysPresentFlag[MAX_TLAYER];
  int m_duSptCpbRemovalDelayIncrement[MAX_TLAYER];
  bool m_dpbOutputDuDelayPresentFlag;
  int m_picSptDpbOutputDuDelay;
};

class SEIFrameFieldInfo : public SEI
{
public:
  PayloadType payloadType() const { return FRAME_FIELD_INFO; }

  SEIFrameFieldInfo()
    : m_fieldPicFlag(false)
    , m_bottomFieldFlag (false)
    , m_pairingIndicatedFlag (false)
    , m_pairedWithNextFieldFlag(false)
    , m_displayFieldsFromFrameFlag(false)
    , m_topFieldFirstFlag(false)
    , m_displayElementalPeriodsMinus1(0)
    , m_sourceScanType(0)
    , m_duplicateFlag(false)
  {}
  virtual ~SEIFrameFieldInfo() {}

  bool m_fieldPicFlag;
  bool m_bottomFieldFlag;
  bool m_pairingIndicatedFlag;
  bool m_pairedWithNextFieldFlag;
  bool m_displayFieldsFromFrameFlag;
  bool m_topFieldFirstFlag;
  int  m_displayElementalPeriodsMinus1;
  int  m_sourceScanType;
  bool m_duplicateFlag;
};

class SEIFramePacking : public SEI
{
public:
  PayloadType payloadType() const { return FRAME_PACKING; }

  SEIFramePacking() {}
  virtual ~SEIFramePacking() {}

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
};

class SEIParameterSetsInclusionIndication : public SEI
{
public:
  PayloadType payloadType() const { return PARAMETER_SETS_INCLUSION_INDICATION; }
  SEIParameterSetsInclusionIndication() {}
  virtual ~SEIParameterSetsInclusionIndication() {}

  int m_selfContainedClvsFlag;
};

class SEIMasteringDisplayColourVolume : public SEI
{
public:
    PayloadType payloadType() const { return MASTERING_DISPLAY_COLOUR_VOLUME; }
    SEIMasteringDisplayColourVolume() {}
    virtual ~SEIMasteringDisplayColourVolume(){}

    SEIMasteringDisplay values;
};

typedef std::list<SEI*> SEIMessages;

/// output a selection of SEI messages by payload type. Ownership stays in original message list.
SEIMessages getSeisByType(const SEIMessages &seiList, SEI::PayloadType seiType);

/// remove a selection of SEI messages by payload type from the original list and return them in a new list.
SEIMessages extractSeisByType(SEIMessages &seiList, SEI::PayloadType seiType);

/// delete list of SEI messages (freeing the referenced objects)
void deleteSEIs (SEIMessages &seiList);

class SEIScalableNesting : public SEI
{
public:
  PayloadType payloadType() const { return SCALABLE_NESTING; }

  SEIScalableNesting()
  : m_snOlsFlag (false)
  , m_snSubpicFlag (false)
  , m_snNumOlssMinus1 (0)
  , m_snAllLayersFlag (false)
  , m_snNumLayersMinus1 (0)
  , m_snNumSubpics (1)
  , m_snSubpicIdLen (0)
  , m_snNumSEIs(0)
  {}

  virtual ~SEIScalableNesting()
  {
    deleteSEIs(m_nestedSEIs);
  }

  bool      m_snOlsFlag;
  bool      m_snSubpicFlag;
  uint32_t  m_snNumOlssMinus1;
  uint32_t  m_snOlsIdxDeltaMinus1[MAX_NESTING_NUM_LAYER];
  uint32_t  m_snOlsIdx[MAX_NESTING_NUM_LAYER];
  bool      m_snAllLayersFlag;                      //value valid if m_nestingOlsFlag == 0
  uint32_t  m_snNumLayersMinus1;                    //value valid if m_nestingOlsFlag == 0 and m_nestingAllLayersFlag == 0
  uint8_t   m_snLayerId[MAX_NESTING_NUM_LAYER];     //value valid if m_nestingOlsFlag == 0 and m_nestingAllLayersFlag == 0. This can e.g. be a static array of 64 uint8_t values
  uint32_t  m_snNumSubpics;
  uint8_t   m_snSubpicIdLen;
  std::vector<uint16_t> m_snSubpicId;
  uint32_t  m_snNumSEIs;

  SEIMessages m_nestedSEIs;
};


#if ENABLE_TRACING
void xTraceSEIHeader();
void xTraceSEIMessageType( SEI::PayloadType payloadType );
#endif

#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
class SEIAlternativeTransferCharacteristics : public SEI
{
public:
  PayloadType payloadType() const { return ALTERNATIVE_TRANSFER_CHARACTERISTICS; }

  SEIAlternativeTransferCharacteristics() : m_preferredTransferCharacteristics(18)
  { }

  virtual ~SEIAlternativeTransferCharacteristics() {}

  uint32_t m_preferredTransferCharacteristics;
};
#endif
class SEIUserDataRegistered : public SEI
{
public:
  PayloadType payloadType() const { return USER_DATA_REGISTERED_ITU_T_T35; }

  SEIUserDataRegistered() {}
  virtual ~SEIUserDataRegistered() {}

  uint16_t m_ituCountryCode;
  std::vector<uint8_t> m_userData;
};

class SEIFilmGrainCharacteristics : public SEI
{
public:
  PayloadType payloadType() const { return FILM_GRAIN_CHARACTERISTICS; }

  SEIFilmGrainCharacteristics() {}
  virtual ~SEIFilmGrainCharacteristics() {}

  bool        m_filmGrainCharacteristicsCancelFlag;
  uint8_t     m_filmGrainModelId;
  bool        m_separateColourDescriptionPresentFlag;
  uint8_t     m_filmGrainBitDepthLumaMinus8;
  uint8_t     m_filmGrainBitDepthChromaMinus8;
  bool        m_filmGrainFullRangeFlag;
  uint8_t     m_filmGrainColourPrimaries;
  uint8_t     m_filmGrainTransferCharacteristics;
  uint8_t     m_filmGrainMatrixCoeffs;
  uint8_t     m_blendingModeId;
  uint8_t     m_log2ScaleFactor;

  struct CompModelIntensityValues
  {
    uint8_t intensityIntervalLowerBound;
    uint8_t intensityIntervalUpperBound;
    std::vector<int> compModelValue;
  };

  struct CompModel
  {
    bool  presentFlag;
    uint8_t numModelValues;
    std::vector<CompModelIntensityValues> intensityValues;
  };

  CompModel m_compModel[MAX_NUM_COMPONENT];
  bool      m_filmGrainCharacteristicsPersistenceFlag;
};

class SEIContentLightLevelInfo : public SEI
{
public:
  PayloadType payloadType() const { return CONTENT_LIGHT_LEVEL_INFO; }
  SEIContentLightLevelInfo() { }

  virtual ~SEIContentLightLevelInfo() { }

  uint32_t m_maxContentLightLevel;
  uint32_t m_maxPicAverageLightLevel;
};

class SEIAmbientViewingEnvironment : public SEI
{
public:
  PayloadType payloadType() const { return AMBIENT_VIEWING_ENVIRONMENT; }
  SEIAmbientViewingEnvironment() { }

  virtual ~SEIAmbientViewingEnvironment() { }

  uint32_t m_ambientIlluminance;
  uint16_t m_ambientLightX;
  uint16_t m_ambientLightY;
};

class SEIContentColourVolume : public SEI
{
public:
  PayloadType payloadType() const { return CONTENT_COLOUR_VOLUME; }
  SEIContentColourVolume() {}
  virtual ~SEIContentColourVolume() {}

  bool      m_ccvCancelFlag;
  bool      m_ccvPersistenceFlag;
  bool      m_ccvPrimariesPresentFlag;
  bool      m_ccvMinLuminanceValuePresentFlag;
  bool      m_ccvMaxLuminanceValuePresentFlag;
  bool      m_ccvAvgLuminanceValuePresentFlag;
  int       m_ccvPrimariesX[MAX_NUM_COMPONENT];
  int       m_ccvPrimariesY[MAX_NUM_COMPONENT];
  uint32_t  m_ccvMinLuminanceValue;
  uint32_t  m_ccvMaxLuminanceValue;
  uint32_t  m_ccvAvgLuminanceValue;
};


class SEISubpicureLevelInfo : public SEI
{
public:
  PayloadType payloadType() const { return SUBPICTURE_LEVEL_INFO; }
  SEISubpicureLevelInfo()
  : m_numRefLevels(0)
  , m_explicitFractionPresentFlag (false)
  , m_cbrConstraintFlag (false)
  , m_numSubpics(0)
#if JVET_S0176_SLI_SEI
  , m_sliMaxSublayers(1)
  , m_sliSublayerInfoPresentFlag(false)
#endif
  {}
  virtual ~SEISubpicureLevelInfo() {}

  int       m_numRefLevels;
  bool      m_explicitFractionPresentFlag;
  bool      m_cbrConstraintFlag;
  int       m_numSubpics;
#if JVET_S0176_SLI_SEI
  int       m_sliMaxSublayers;
  bool      m_sliSublayerInfoPresentFlag;
#if JVET_S0098_SLI_FRACTION
  std::vector<std::vector<int>>              m_nonSubpicLayersFraction;
#endif
  std::vector<std::vector<Level::Name>>      m_refLevelIdc;
  std::vector<std::vector<std::vector<int>>> m_refLevelFraction;
#else
  std::vector<Level::Name>      m_refLevelIdc;
  std::vector<std::vector<int>> m_refLevelFraction;
#endif
};
#endif

//! \}
