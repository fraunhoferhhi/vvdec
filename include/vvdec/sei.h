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

#pragma once

#include "vvdec/vvdecDecl.h"

#include <list>
#include <vector>
#include <cstdint>
#include <cstring>

namespace vvdec {


class VVDEC_DECL sei
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
    CONTENT_COLOUR_VOLUME                = 149
  };

  sei() {}
  virtual ~sei() {}

  static const char *getSEIMessageString(sei::PayloadType payloadType);

  /// output a selection of SEI messages by payload type. Ownership stays in original message list.
  static std::list<sei*> getSeisByType(const std::list<sei*> &seiList, sei::PayloadType seiType);

  /// remove a selection of SEI messages by payload type from the original list and return them in a new list.
  static std::list<sei*> extractSeisByType(std::list<sei*> &seiList, sei::PayloadType seiType);

  /// delete list of SEI messages (freeing the referenced objects)
  static void deleteSEIs (std::list<sei*> &seiList);

  virtual PayloadType payloadType() const = 0;
};

typedef std::list<vvdec::sei*> seiMessages;

class VVDEC_DECL seiEquirectangularProjection : public sei
{
public:
  PayloadType payloadType() const { return EQUIRECTANGULAR_PROJECTION; }

  seiEquirectangularProjection()  {}
  virtual ~seiEquirectangularProjection() {}

  bool    m_erpCancelFlag          = false;
  bool    m_erpPersistenceFlag     = false;
  bool    m_erpGuardBandFlag       = false;
  uint8_t m_erpGuardBandType       = 0;
  uint8_t m_erpLeftGuardBandWidth  = 0;
  uint8_t m_erpRightGuardBandWidth = 0;
};

class VVDEC_DECL seiSphereRotation : public sei
{
public:
  PayloadType payloadType() const { return SPHERE_ROTATION; }

  seiSphereRotation()  {}
  virtual ~seiSphereRotation() {}

  bool  m_sphereRotationCancelFlag      = false;
  bool  m_sphereRotationPersistenceFlag = false;
  int   m_sphereRotationYaw             = 0;
  int   m_sphereRotationPitch           = 0;
  int   m_sphereRotationRoll            = 0;
};

class VVDEC_DECL seiOmniViewport : public sei
{
public:
  PayloadType payloadType() const { return OMNI_VIEWPORT; }

  seiOmniViewport() {}
  virtual ~seiOmniViewport() {}

  struct OmniViewport
  {
    int      azimuthCentre   = 0;
    int      elevationCentre = 0;
    int      tiltCentre      = 0;
    uint32_t horRange        = 0;
    uint32_t verRange        = 0;
  };

  uint32_t m_omniViewportId              = 0;
  bool     m_omniViewportCancelFlag      = false;
  bool     m_omniViewportPersistenceFlag = false;
  uint8_t  m_omniViewportCnt             = 0;
  std::vector<OmniViewport> m_omniViewportRegions = {};
};

class VVDEC_DECL seiRegionWisePacking : public sei
{
public:
  PayloadType payloadType() const { return REGION_WISE_PACKING; }
  seiRegionWisePacking() {}
  virtual ~seiRegionWisePacking() {}
  bool                  m_rwpCancelFlag                  = false;
  bool                  m_rwpPersistenceFlag             = false;
  bool                  m_constituentPictureMatchingFlag = false;
  int                   m_numPackedRegions               = 0;
  int                   m_projPictureWidth               = 0;
  int                   m_projPictureHeight              = 0;
  int                   m_packedPictureWidth             = 0;
  int                   m_packedPictureHeight            = 0;
  std::vector<uint8_t>  m_rwpTransformType               = {};
  std::vector<bool>     m_rwpGuardBandFlag               = {};
  std::vector<uint32_t> m_projRegionWidth                = {};
  std::vector<uint32_t> m_projRegionHeight               = {};
  std::vector<uint32_t> m_rwpProjRegionTop               = {};
  std::vector<uint32_t> m_projRegionLeft                 = {};
  std::vector<uint16_t> m_packedRegionWidth              = {};
  std::vector<uint16_t> m_packedRegionHeight             = {};
  std::vector<uint16_t> m_packedRegionTop                = {};
  std::vector<uint16_t> m_packedRegionLeft               = {};
  std::vector<uint8_t>  m_rwpLeftGuardBandWidth          = {};
  std::vector<uint8_t>  m_rwpRightGuardBandWidth         = {};
  std::vector<uint8_t>  m_rwpTopGuardBandHeight          = {};
  std::vector<uint8_t>  m_rwpBottomGuardBandHeight       = {};
  std::vector<bool>     m_rwpGuardBandNotUsedForPredFlag = {};
  std::vector<uint8_t>  m_rwpGuardBandType               = {};
};

class VVDEC_DECL seiGeneralizedCubemapProjection : public sei
{
public:
  PayloadType payloadType() const { return GENERALIZED_CUBEMAP_PROJECTION; }

  seiGeneralizedCubemapProjection()  {}
  virtual ~seiGeneralizedCubemapProjection() {}

  bool                 m_gcmpCancelFlag                    = false;
  bool                 m_gcmpPersistenceFlag               = false;
  uint8_t              m_gcmpPackingType                   = 0;
  uint8_t              m_gcmpMappingFunctionType           = 0;
  std::vector<uint8_t> m_gcmpFaceIndex                     = {};
  std::vector<uint8_t> m_gcmpFaceRotation                  = {};
  std::vector<uint8_t> m_gcmpFunctionCoeffU                = {};
  std::vector<bool>    m_gcmpFunctionUAffectedByVFlag      = {};
  std::vector<uint8_t> m_gcmpFunctionCoeffV                = {};
  std::vector<bool>    m_gcmpFunctionVAffectedByUFlag      = {};
  bool                 m_gcmpGuardBandFlag                 = false;
  uint8_t              m_gcmpGuardBandType                 = 0;
  bool                 m_gcmpGuardBandBoundaryExteriorFlag = false;
  uint8_t              m_gcmpGuardBandSamples              = 0;
};

class VVDEC_DECL seiSampleAspectRatioInfo : public sei
{
public:
  PayloadType payloadType() const { return SAMPLE_ASPECT_RATIO_INFO; }
  seiSampleAspectRatioInfo() {}
  virtual ~seiSampleAspectRatioInfo() {}
  bool                  m_sariCancelFlag      = false;
  bool                  m_sariPersistenceFlag = false;
  int                   m_sariAspectRatioIdc  = 0;
  int                   m_sariSarWidth        = 0;
  int                   m_sariSarHeight       = 0;
};

static const uint32_t ISO_IEC_11578_LEN=16;

class VVDEC_DECL seiUserDataUnregistered : public sei
{
public:
  PayloadType payloadType() const { return USER_DATA_UNREGISTERED; }

  seiUserDataUnregistered()
    {}

  virtual ~seiUserDataUnregistered()
  {
    delete userData;
  }

  uint8_t  uuid_iso_iec_11578[ISO_IEC_11578_LEN];
  uint32_t userDataLength = 0;
  uint8_t *userData;
};

struct PictureHash
{
  std::vector<uint8_t> hash = {};

  bool operator==(const PictureHash &other) const
  {
    if (other.hash.size() != hash.size())
    {
      return false;
    }
    for(uint32_t i=0; i<uint32_t(hash.size()); i++)
    {
      if (other.hash[i] != hash[i])
      {
        return false;
      }
    }
    return true;
  }

  bool operator!=(const PictureHash &other) const
  {
    return !(*this == other);
  }
};

class VVDEC_DECL seiDecodedPictureHash : public sei
{
public:

  enum HashType
  {
    HASHTYPE_MD5             = 0,
    HASHTYPE_CRC             = 1,
    HASHTYPE_CHECKSUM        = 2,
    HASHTYPE_NONE            = 3,
    NUMBER_OF_HASHTYPES      = 4
  };

public:
  PayloadType payloadType() const { return DECODED_PICTURE_HASH; }

  seiDecodedPictureHash() {}
  virtual ~seiDecodedPictureHash() {}

  HashType    method         = HASHTYPE_MD5;
  bool        singleCompFlag = false;
  PictureHash m_pictureHash;
};

class VVDEC_DECL seiDependentRAPIndication : public sei
{
public:
  PayloadType payloadType() const { return DEPENDENT_RAP_INDICATION; }
  seiDependentRAPIndication() { }

  virtual ~seiDependentRAPIndication() { }
};

class VVDEC_DECL seiBufferingPeriod : public sei
{
public:
  PayloadType payloadType() const { return BUFFERING_PERIOD; }

  seiBufferingPeriod()
  {
    ::memset(m_initialCpbRemovalDelay , 0, sizeof(m_initialCpbRemovalDelay));
    ::memset(m_initialCpbRemovalOffset, 0, sizeof(m_initialCpbRemovalOffset));
    ::memset(m_cpbRemovalDelayDelta   , 0, sizeof(m_cpbRemovalDelayDelta));
    ::memset(m_dpbOutputTidOffset     , 0, sizeof(m_dpbOutputTidOffset));
  }
  virtual ~seiBufferingPeriod() {}

  void      setDuCpbRemovalDelayIncrementLength( uint32_t value )        { m_duCpbRemovalDelayIncrementLength = value;        }
  uint32_t  getDuCpbRemovalDelayIncrementLength( ) const                 { return m_duCpbRemovalDelayIncrementLength;         }
  void      setDpbOutputDelayDuLength( uint32_t value )                  { m_dpbOutputDelayDuLength = value;                  }
  uint32_t  getDpbOutputDelayDuLength( ) const                           { return m_dpbOutputDelayDuLength;                   }

  bool     m_bpNalCpbParamsPresentFlag                 = false;
  bool     m_bpVclCpbParamsPresentFlag                 = false;
  uint32_t m_initialCpbRemovalDelayLength              = 0;
  uint32_t m_cpbRemovalDelayLength                     = 0;
  uint32_t m_dpbOutputDelayLength                      = 0;
  int      m_bpCpbCnt                                  = 0;
  uint32_t m_duCpbRemovalDelayIncrementLength          = 0;
  uint32_t m_dpbOutputDelayDuLength                    = 0;
  uint32_t m_initialCpbRemovalDelay         [7][32][2];
  uint32_t m_initialCpbRemovalOffset        [7][32][2];
  bool     m_concatenationFlag                         = false;
  uint32_t m_auCpbRemovalDelayDelta                    = 0;
  bool     m_cpbRemovalDelayDeltasPresentFlag          = false;
  int      m_numCpbRemovalDelayDeltas                  = 0;
  int      m_bpMaxSubLayers                            = 0;
  uint32_t m_cpbRemovalDelayDelta           [15];
  bool     m_bpDecodingUnitHrdParamsPresentFlag        = false;
  bool     m_decodingUnitCpbParamsInPicTimingSeiFlag   = false;
  bool     m_decodingUnitDpbDuParamsInPicTimingSeiFlag = false;
  bool     m_sublayerInitialCpbRemovalDelayPresentFlag = false;
  bool     m_additionalConcatenationInfoPresentFlag    = false;
  uint32_t m_maxInitialRemovalDelayForConcatenation    = 0;
  bool     m_sublayerDpbOutputOffsetsPresentFlag       = false;
  uint32_t m_dpbOutputTidOffset              [7];
  bool     m_altCpbParamsPresentFlag                   = false;
  bool     m_useAltCpbParamsFlag                       = false;
};

class VVDEC_DECL seiPictureTiming : public sei
{
public:
  PayloadType payloadType() const { return PICTURE_TIMING; }

  seiPictureTiming()
  {
    ::memset(m_ptSubLayerDelaysPresentFlag,     0, sizeof(m_ptSubLayerDelaysPresentFlag));
    ::memset(m_duCommonCpbRemovalDelay,         0, sizeof(m_duCommonCpbRemovalDelay));
    ::memset(m_cpbRemovalDelayDeltaEnabledFlag, 0, sizeof(m_cpbRemovalDelayDeltaEnabledFlag));
    ::memset(m_cpbRemovalDelayDeltaIdx,         0, sizeof(m_cpbRemovalDelayDeltaIdx));
    ::memset(m_auCpbRemovalDelay,               0, sizeof(m_auCpbRemovalDelay));
  }
  virtual ~seiPictureTiming()
  {
  }

  bool                  m_ptSubLayerDelaysPresentFlag[7];
  bool                  m_cpbRemovalDelayDeltaEnabledFlag[7];
  uint32_t              m_cpbRemovalDelayDeltaIdx[7];
  uint32_t              m_auCpbRemovalDelay[7];
  uint32_t              m_picDpbOutputDelay           = 0;
  uint32_t              m_picDpbOutputDuDelay         = 0;
  uint32_t              m_numDecodingUnits            = 0;
  bool                  m_duCommonCpbRemovalDelayFlag = false;
  uint32_t              m_duCommonCpbRemovalDelay[7];
  std::vector<uint32_t> m_numNalusInDu                = {};
  std::vector<uint32_t> m_duCpbRemovalDelay           = {};
  bool                  m_cpbAltTimingInfoPresentFlag = false;
  std::vector<std::vector<uint32_t>> m_nalCpbAltInitialRemovalDelayDelta  = {};
  std::vector<std::vector<uint32_t>> m_nalCpbAltInitialRemovalOffsetDelta = {};
  std::vector<uint32_t>              m_nalCpbDelayOffset = {};
  std::vector<uint32_t>              m_nalDpbDelayOffset = {};
  std::vector<std::vector<uint32_t>> m_vclCpbAltInitialRemovalDelayDelta  = {};
  std::vector<std::vector<uint32_t>> m_vclCpbAltInitialRemovalOffsetDelta = {};
  std::vector<uint32_t>              m_vclCpbDelayOffset = {};
  std::vector<uint32_t>              m_vclDpbDelayOffset = {};
  int                                m_ptDisplayElementalPeriods = 0;
};

class VVDEC_DECL seiDecodingUnitInfo : public sei
{
public:
  PayloadType payloadType() const { return DECODING_UNIT_INFO; }

  seiDecodingUnitInfo()
  {
    ::memset(m_duiSubLayerDelaysPresentFlag,  0, sizeof(m_duiSubLayerDelaysPresentFlag));
    ::memset(m_duSptCpbRemovalDelayIncrement, 0, sizeof(m_duSptCpbRemovalDelayIncrement));
  }
  virtual ~seiDecodingUnitInfo() {}
  int  m_decodingUnitIdx = 0;
  bool m_duiSubLayerDelaysPresentFlag[7];
  int  m_duSptCpbRemovalDelayIncrement[7];
  bool m_dpbOutputDuDelayPresentFlag = false;
  int  m_picSptDpbOutputDuDelay      = 0;
};

class VVDEC_DECL seiFrameFieldInfo : public sei
{
public:
  PayloadType payloadType() const { return FRAME_FIELD_INFO; }

  seiFrameFieldInfo()
  {}
  virtual ~seiFrameFieldInfo() {}

  bool m_fieldPicFlag                  = false;
  bool m_bottomFieldFlag               = false;
  bool m_pairingIndicatedFlag          = false;
  bool m_pairedWithNextFieldFlag       = false;
  bool m_displayFieldsFromFrameFlag    = false;
  bool m_topFieldFirstFlag             = false;
  int  m_displayElementalPeriods       = 0;
  int  m_sourceScanType                = 0;
  bool m_duplicateFlag                 = false;
};

class VVDEC_DECL seiFramePacking : public sei
{
public:
  PayloadType payloadType() const { return FRAME_PACKING; }

  seiFramePacking() {}
  virtual ~seiFramePacking() {}

  int  m_arrangementId               = 0;
  bool m_arrangementCancelFlag       = false;
  int  m_arrangementType             = 0;
  bool m_quincunxSamplingFlag        = false;
  int  m_contentInterpretationType   = 0;
  bool m_spatialFlippingFlag         = false;
  bool m_frame0FlippedFlag           = false;
  bool m_fieldViewsFlag              = false;
  bool m_currentFrameIsFrame0Flag    = false;
  bool m_frame0SelfContainedFlag     = false;
  bool m_frame1SelfContainedFlag     = false;
  int  m_frame0GridPositionX         = 0;
  int  m_frame0GridPositionY         = 0;
  int  m_frame1GridPositionX         = 0;
  int  m_frame1GridPositionY         = 0;
  int  m_arrangementReservedByte     = 0;
  bool m_arrangementPersistenceFlag  = false;
  bool m_upsampledAspectRatio        = false;
};

class VVDEC_DECL seiParameterSetsInclusionIndication : public sei
{
public:
  PayloadType payloadType() const { return PARAMETER_SETS_INCLUSION_INDICATION; }
  seiParameterSetsInclusionIndication() {}
  virtual ~seiParameterSetsInclusionIndication() {}

  int m_selfContainedClvsFlag;
};

class VVDEC_DECL seiMasteringDisplayColourVolume : public sei
{
public:
    PayloadType payloadType() const { return MASTERING_DISPLAY_COLOUR_VOLUME; }
    seiMasteringDisplayColourVolume()
    {
      ::memset(primaries  , 0, sizeof(primaries));
      ::memset(whitePoint , 0, sizeof(whitePoint));
    }
    virtual ~seiMasteringDisplayColourVolume(){}

    uint32_t maxLuminance           = 0;
    uint32_t minLuminance           = 0;
    uint16_t primaries[3][2];
    uint16_t whitePoint[2];
};

class VVDEC_DECL seiScalableNesting : public sei
{
public:
  PayloadType payloadType() const { return SCALABLE_NESTING; }

  seiScalableNesting()
  {
    ::memset(m_snOlsIdxDelta , 0, sizeof(m_snOlsIdxDelta));
    ::memset(m_snOlsIdx      , 0, sizeof(m_snOlsIdx));
    ::memset(m_snLayerId     , 0, sizeof(m_snLayerId));
  }

  virtual ~seiScalableNesting()
  {
    deleteSEIs(m_nestedSEIs);
  }

  bool      m_snOlsFlag         = false;
  bool      m_snSubpicFlag      = false;
  uint32_t  m_snNumOlss         = 0;
  uint32_t  m_snOlsIdxDelta[64];
  uint32_t  m_snOlsIdx[64];
  bool      m_snAllLayersFlag   = false;  //value valid if m_nestingOlsFlag == 0
  uint32_t  m_snNumLayers       = 0;      //value valid if m_nestingOlsFlag == 0 and m_nestingAllLayersFlag == 0
  uint8_t   m_snLayerId[64];              //value valid if m_nestingOlsFlag == 0 and m_nestingAllLayersFlag == 0. This can e.g. be a static array of 64 uint8_t values
  uint32_t  m_snNumSubpics      = 0;
  uint8_t   m_snSubpicIdLen     = 0;
  std::vector<uint16_t> m_snSubpicId = {};
  uint32_t  m_snNumSEIs         = 0;

  std::list<sei*> m_nestedSEIs;
};


class VVDEC_DECL seiAlternativeTransferCharacteristics : public sei
{
public:
  PayloadType payloadType() const { return ALTERNATIVE_TRANSFER_CHARACTERISTICS; }

  seiAlternativeTransferCharacteristics() : m_preferredTransferCharacteristics(18)
  { }

  virtual ~seiAlternativeTransferCharacteristics() {}

  uint32_t m_preferredTransferCharacteristics = 0;
};

class VVDEC_DECL seiUserDataRegistered : public sei
{
public:
  PayloadType payloadType() const { return USER_DATA_REGISTERED_ITU_T_T35; }

  seiUserDataRegistered() {}
  virtual ~seiUserDataRegistered() {}

  uint16_t             m_ituCountryCode = 0;
  std::vector<uint8_t> m_userData       = {};
};

class VVDEC_DECL seiFilmGrainCharacteristics : public sei
{
public:
  PayloadType payloadType() const { return FILM_GRAIN_CHARACTERISTICS; }

  seiFilmGrainCharacteristics() {}
  virtual ~seiFilmGrainCharacteristics() {}

  bool        m_filmGrainCharacteristicsCancelFlag   = false;
  uint8_t     m_filmGrainModelId                     = 0;
  bool        m_separateColourDescriptionPresentFlag = false;
  uint8_t     m_filmGrainBitDepthLuma                = 0;
  uint8_t     m_filmGrainBitDepthChroma              = 0;
  bool        m_filmGrainFullRangeFlag               = false;
  uint8_t     m_filmGrainColourPrimaries             = 0;
  uint8_t     m_filmGrainTransferCharacteristics     = 0;
  uint8_t     m_filmGrainMatrixCoeffs                = 0;
  uint8_t     m_blendingModeId                       = 0;
  uint8_t     m_log2ScaleFactor                      = 0;

  struct CompModelIntensityValues
  {
    uint8_t intensityIntervalLowerBound = 0;
    uint8_t intensityIntervalUpperBound = 0;
    std::vector<int> compModelValue     = {};
  };

  struct CompModel
  {
    bool    presentFlag;
    uint8_t numModelValues;
    std::vector<CompModelIntensityValues> intensityValues = {};
  };

  CompModel m_compModel[3];
  bool      m_filmGrainCharacteristicsPersistenceFlag = false;
};

class VVDEC_DECL seiContentLightLevelInfo : public sei
{
public:
  PayloadType payloadType() const { return CONTENT_LIGHT_LEVEL_INFO; }
  seiContentLightLevelInfo() { }

  virtual ~seiContentLightLevelInfo() { }

  uint32_t m_maxContentLightLevel    = 0;
  uint32_t m_maxPicAverageLightLevel = 0;
};

class VVDEC_DECL seiAmbientViewingEnvironment : public sei
{
public:
  PayloadType payloadType() const { return AMBIENT_VIEWING_ENVIRONMENT; }
  seiAmbientViewingEnvironment() { }

  virtual ~seiAmbientViewingEnvironment() { }

  uint32_t m_ambientIlluminance = 0;
  uint16_t m_ambientLightX      = 0;
  uint16_t m_ambientLightY      = 0;
};

class VVDEC_DECL seiContentColourVolume : public sei
{
public:
  PayloadType payloadType() const { return CONTENT_COLOUR_VOLUME; }
  seiContentColourVolume()
  {
    ::memset(m_ccvPrimariesX , 0, sizeof(m_ccvPrimariesX));
    ::memset(m_ccvPrimariesY , 0, sizeof(m_ccvPrimariesY));
  }
  virtual ~seiContentColourVolume() {}

  bool      m_ccvCancelFlag                   = false;
  bool      m_ccvPersistenceFlag              = false;
  bool      m_ccvPrimariesPresentFlag         = false;
  bool      m_ccvMinLuminanceValuePresentFlag = false;
  bool      m_ccvMaxLuminanceValuePresentFlag = false;
  bool      m_ccvAvgLuminanceValuePresentFlag = false;
  int       m_ccvPrimariesX[3];
  int       m_ccvPrimariesY[3];
  uint32_t  m_ccvMinLuminanceValue            = 0;
  uint32_t  m_ccvMaxLuminanceValue            = 0;
  uint32_t  m_ccvAvgLuminanceValue            = 0;
};


class VVDEC_DECL seiSubpicureLevelInfo : public sei
{
public:
  enum Level
  {
    LEVEL_NONE = 0,
    LEVEL_1   = 16,
    LEVEL_2   = 32,
    LEVEL_2_1 = 35,
    LEVEL_3   = 48,
    LEVEL_3_1 = 51,
    LEVEL_4   = 64,
    LEVEL_4_1 = 67,
    LEVEL_5   = 80,
    LEVEL_5_1 = 83,
    LEVEL_5_2 = 86,
    LEVEL_6   = 96,
    LEVEL_6_1 = 99,
    LEVEL_6_2 = 102,
    LEVEL_15_5 = 255,
  };

public:
  PayloadType payloadType() const { return SUBPICTURE_LEVEL_INFO; }
  seiSubpicureLevelInfo()
  {}
  virtual ~seiSubpicureLevelInfo() {}

  int       m_numRefLevels                = 0;
  bool      m_explicitFractionPresentFlag = false;
  bool      m_cbrConstraintFlag           = false;
  int       m_numSubpics                  = 0;
  int       m_sliMaxSublayers             = 0;
  bool      m_sliSublayerInfoPresentFlag  = false;
  std::vector<std::vector<int>>              m_nonSubpicLayersFraction = {};
  std::vector<std::vector<Level>>            m_refLevelIdc             = {};
  std::vector<std::vector<std::vector<int>>> m_refLevelFraction        = {};
};

} // namespace

