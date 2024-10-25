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

/** \file     Slice.h
    \brief    slice header and SPS class (header)
*/

#pragma once

#include "CommonDef.h"
#include "ChromaFormat.h"
#include "Common.h"
#include "BitStream.h"

#include "Utilities/ThreadPool.h"

#include <cstring>
#include <list>
#include <vector>

namespace vvdec
{

// ====================================================================================================================
// Constants
// ====================================================================================================================
class PreCalcValues;
class ParameterSetManager;
struct Picture;
typedef std::list<Picture*> PicList;

static const uint32_t REF_PIC_LIST_NUM_IDX=32;

using PartitionConstraints = std::array<unsigned, 3>;
using AlfApsIdVec          = static_vector<int, MAX_NUM_ALF_APS_IDS>;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

struct DpbParameters
{
  int m_maxDecPicBuffering     [MAX_TLAYER] = { 0 };
  int m_numReorderPics         [MAX_TLAYER] = { 0 };
  int m_maxLatencyIncreasePlus1[MAX_TLAYER] = { 0 };
};

class ReferencePictureList
{
private:
  int   m_numberOfShorttermPictures                = 0;
  int   m_numberOfLongtermPictures                 = 0;
  int   m_isLongtermRefPic      [MAX_NUM_REF_PICS] = { 0 };
  int   m_refPicIdentifier      [MAX_NUM_REF_PICS] = { 0 };   // This can be delta POC for STRP or POC LSB for LTRP
  int   m_POC                   [MAX_NUM_REF_PICS] = { 0 };
  int   m_deltaPOCMSBCycleLT    [MAX_NUM_REF_PICS] = { 0 };
  bool  m_deltaPocMSBPresentFlag[MAX_NUM_REF_PICS] = { 0 };
  bool  m_ltrp_in_slice_header_flag                = false;
  bool  m_interLayerPresentFlag                    = false;
  bool  m_isInterLayerRefPic [MAX_NUM_REF_PICS]    = { 0 };
  int   m_interLayerRefPicIdx[MAX_NUM_REF_PICS]    = { 0 };
  int   m_numberOfInterLayerPictures               = 0;

public:
  ReferencePictureList();
  void clear();

  void setRefPicIdentifier( int idx, int identifier, bool isLongterm, bool isInterLayerRefPic, int interLayerIdx );
  int  getRefPicIdentifier( int idx ) const;
  bool isRefPicLongterm( int idx ) const;

  void setRefPicLongterm( int idx, bool isLongterm );

  void setNumberOfShorttermPictures( int numberOfStrp );
  int  getNumberOfShorttermPictures() const;

  void setNumberOfLongtermPictures( int numberOfLtrp );
  int  getNumberOfLongtermPictures() const;

  void setLtrpInSliceHeaderFlag( bool flag ) { m_ltrp_in_slice_header_flag = flag; }
  bool getLtrpInSliceHeaderFlag() const { return m_ltrp_in_slice_header_flag; }

  void setNumberOfInterLayerPictures( const int numberOfIlrp ) { m_numberOfInterLayerPictures = numberOfIlrp; }
  int  getNumberOfInterLayerPictures() const { return m_numberOfInterLayerPictures; }

  int getNumRefEntries() const { return m_numberOfShorttermPictures + m_numberOfLongtermPictures + m_numberOfInterLayerPictures; }

  void setPOC( int idx, int POC );
  int  getPOC( int idx ) const;

  int  getDeltaPocMSBCycleLT( int i ) const { return m_deltaPOCMSBCycleLT[i]; }
  void setDeltaPocMSBCycleLT( int i, int x );
  bool getDeltaPocMSBPresentFlag( int i ) const { return m_deltaPocMSBPresentFlag[i]; }
  void setDeltaPocMSBPresentFlag( int i, bool x );

  void printRefPicInfo() const;
  bool getInterLayerPresentFlag() const { return m_interLayerPresentFlag; }
  void setInterLayerPresentFlag( bool b ) { m_interLayerPresentFlag = b; }
  bool isInterLayerRefPic( int idx ) const { return m_isInterLayerRefPic[idx]; }
  int  getInterLayerRefPicIdx( int idx ) const { return m_interLayerRefPicIdx[idx]; }
  void setInterLayerRefPicIdx( int idx, int layerIdc );

  bool findInRefPicList( const Picture* checkRefPic, int currPicPoc, int layerId ) const;

  static int calcLTRefPOC( int currPoc, int bitsForPoc, int refPicIdentifier, bool pocMSBPresent, int deltaPocMSBCycle );
  int        calcLTRefPOC( int currPoc, int bitsForPoc, int refPicIdx ) const;
};

bool isLTPocEqual( int poc1, int poc2, int bitsForPoc, bool msbPresent );

/// Reference Picture List set class

typedef std::vector<ReferencePictureList> RPLList;


/// SCALING_LIST class
class ScalingList
{
public:
  ScalingList();
  ~ScalingList() = default;
  CLASS_COPY_MOVE_DEFAULT( ScalingList )

  void       reset();

  static inline int  matrixSize( uint32_t scalingListId )   { return scalingListId < SCALING_LIST_1D_START_4x4 ? 2 : scalingListId < SCALING_LIST_1D_START_8x8 ? 4 : 8; }
  static inline bool isLumaScalingList( int scalingListId ) { return scalingListId % MAX_NUM_COMPONENT == SCALING_LIST_1D_START_4x4 || scalingListId == SCALING_LIST_1D_START_64x64 + 1;}

  void              setScalingListDC(uint32_t scalingListId, uint32_t u)             { m_scalingListDC[scalingListId] = u;                       } //!< set DC value
  int               getScalingListDC(uint32_t scalingListId) const                   { return m_scalingListDC[scalingListId];                    } //!< get DC value

  int*              getScalingListAddress(uint32_t scalingListId)                    { return m_scalingListCoef[scalingListId].data();           } //!< get matrix coefficient
  const int*        getScalingListAddress(uint32_t scalingListId) const              { return m_scalingListCoef[scalingListId].data();           } //!< get matrix coefficient
  std::vector<int>& getScalingListVec( uint32_t scalingListId )                      { return m_scalingListCoef[scalingListId]; }

private:
  int              m_scalingListDC  [28] = { 0 }; //!< the DC value of the matrix coefficient for 16x16
  std::vector<int> m_scalingListCoef[28];         //!< quantization matrix
};

class ConstraintInfo
{
  bool         m_gciPresentFlag                               = false;
  bool         m_noRprConstraintFlag                          = false;
  bool         m_noResChangeInClvsConstraintFlag              = false;
  bool         m_oneTilePerPicConstraintFlag                  = false;
  bool         m_picHeaderInSliceHeaderConstraintFlag         = false;
  bool         m_oneSlicePerPicConstraintFlag                 = false;
  bool         m_noIdrRplConstraintFlag                       = false;
  bool         m_noRectSliceConstraintFlag                    = false;
  bool         m_oneSlicePerSubpicConstraintFlag              = false;
  bool         m_noSubpicInfoConstraintFlag                   = false;
  bool         m_intraOnlyConstraintFlag                      = false;
  uint32_t     m_maxBitDepthConstraintIdc                     = 16;
  ChromaFormat m_maxChromaFormatConstraintIdc                 = CHROMA_444;
  bool         m_onePictureOnlyConstraintFlag                 = false;
  bool         m_lowerBitRateConstraintFlag                   = false;

  bool         m_allLayersIndependentConstraintFlag           = false;
  bool         m_noMrlConstraintFlag                          = false;
  bool         m_noIspConstraintFlag                          = false;
  bool         m_noMipConstraintFlag                          = false;
  bool         m_noLfnstConstraintFlag                        = false;
  bool         m_noMmvdConstraintFlag                         = false;
  bool         m_noSmvdConstraintFlag                         = false;
  bool         m_noProfConstraintFlag                         = false;
  bool         m_noPaletteConstraintFlag                      = false;
  bool         m_noActConstraintFlag                          = false;
  bool         m_noLmcsConstraintFlag                         = false;

  bool         m_noExplicitScaleListConstraintFlag            = false;
  bool         m_noVirtualBoundaryConstraintFlag              = false;
  bool         m_noMttConstraintFlag                          = false;
  bool         m_noChromaQpOffsetConstraintFlag               = false;
  bool         m_noQtbttDualTreeIntraConstraintFlag           = false;
  int          m_maxLog2CtuSizeConstraintIdc                  = 8;
  bool         m_noPartitionConstraintsOverrideConstraintFlag = false;
  bool         m_noSaoConstraintFlag                          = false;
  bool         m_noAlfConstraintFlag                          = false;
  bool         m_noCCAlfConstraintFlag                        = false;
  bool         m_noWeightedPredictionConstraintFlag           = false;
  bool         m_noRefWraparoundConstraintFlag                = false;
  bool         m_noTemporalMvpConstraintFlag                  = false;
  bool         m_noSbtmvpConstraintFlag                       = false;
  bool         m_noAmvrConstraintFlag                         = false;
  bool         m_noBdofConstraintFlag                         = false;
  bool         m_noDmvrConstraintFlag                         = false;
  bool         m_noCclmConstraintFlag                         = false;
  bool         m_noMtsConstraintFlag                          = false;
  bool         m_noSbtConstraintFlag                          = false;
  bool         m_noAffineMotionConstraintFlag                 = false;
  bool         m_noBcwConstraintFlag                          = false;
  bool         m_noIbcConstraintFlag                          = false;
  bool         m_noCiipConstraintFlag                         = false;
  bool         m_noGeoConstraintFlag                          = false;
  bool         m_noLadfConstraintFlag                         = false;
  bool         m_noTransformSkipConstraintFlag                = false;
  bool         m_noLumaTransformSize64ConstraintFlag          = false;
  bool         m_noBDPCMConstraintFlag                        = false;
  bool         m_noJointCbCrConstraintFlag                    = false;
  bool         m_noQpDeltaConstraintFlag                      = false;
  bool         m_noDepQuantConstraintFlag                     = false;
  bool         m_noSignDataHidingConstraintFlag               = false;
  bool         m_noMixedNaluTypesInPicConstraintFlag          = false;
  bool         m_noTrailConstraintFlag                        = false;
  bool         m_noStsaConstraintFlag                         = false;
  bool         m_noRaslConstraintFlag                         = false;
  bool         m_noRadlConstraintFlag                         = false;
  bool         m_noIdrConstraintFlag                          = false;
  bool         m_noCraConstraintFlag                          = false;
  bool         m_noGdrConstraintFlag                          = false;
  bool         m_noApsConstraintFlag                          = false;

public:


  bool          getGciPresentFlag() const { return m_gciPresentFlag; }
  void          setGciPresentFlag(bool b) { m_gciPresentFlag = b; }

  uint32_t      getMaxBitDepthConstraintIdc() const { return m_maxBitDepthConstraintIdc; }
  void          setMaxBitDepthConstraintIdc(uint32_t bitDepth) { m_maxBitDepthConstraintIdc = bitDepth; }

  ChromaFormat  getMaxChromaFormatConstraintIdc() const { return m_maxChromaFormatConstraintIdc; }
  void          setMaxChromaFormatConstraintIdc(ChromaFormat fmt) { m_maxChromaFormatConstraintIdc = fmt; }

  bool          getNoRprConstraintFlag() const { return m_noRprConstraintFlag; }
  void          setNoRprConstraintFlag(bool b) { m_noRprConstraintFlag = b; }

  bool          getNoResChangeInClvsConstraintFlag() const { return m_noResChangeInClvsConstraintFlag; }
  void          setNoResChangeInClvsConstraintFlag(bool b) { m_noResChangeInClvsConstraintFlag = b; }

  bool          getOneTilePerPicConstraintFlag() const { return m_oneTilePerPicConstraintFlag; }
  void          setOneTilePerPicConstraintFlag(bool b) { m_oneTilePerPicConstraintFlag = b; }

  bool          getPicHeaderInSliceHeaderConstraintFlag() const { return m_picHeaderInSliceHeaderConstraintFlag; }
  void          setPicHeaderInSliceHeaderConstraintFlag(bool b) { m_picHeaderInSliceHeaderConstraintFlag = b; }
  bool          getOneSlicePerPicConstraintFlag() const { return m_oneSlicePerPicConstraintFlag; }
  void          setOneSlicePerPicConstraintFlag(bool b) { m_oneSlicePerPicConstraintFlag = b; }

  bool          getNoIdrRplConstraintFlag() const          { return m_noIdrRplConstraintFlag; }
  void          setNoIdrRplConstraintFlag(bool b)          { m_noIdrRplConstraintFlag = b; }

  bool          getNoRectSliceConstraintFlag() const       { return m_noRectSliceConstraintFlag; }
  void          setNoRectSliceConstraintFlag(bool b)       { m_noRectSliceConstraintFlag = b; }

  bool          getOneSlicePerSubpicConstraintFlag() const { return m_oneSlicePerSubpicConstraintFlag; }
  void          setOneSlicePerSubpicConstraintFlag(bool b) { m_oneSlicePerSubpicConstraintFlag = b; }

  bool          getNoSubpicInfoConstraintFlag() const      { return m_noSubpicInfoConstraintFlag; }
  void          setNoSubpicInfoConstraintFlag(bool b)      { m_noSubpicInfoConstraintFlag = b; }

  bool          getIntraOnlyConstraintFlag() const { return m_intraOnlyConstraintFlag; }
  void          setIntraOnlyConstraintFlag(bool b) { m_intraOnlyConstraintFlag = b; }

  bool          getOnePictureOnlyConstraintFlag() const { return m_onePictureOnlyConstraintFlag; }
  void          setOnePictureOnlyConstraintFlag(bool b) { m_onePictureOnlyConstraintFlag = b; }

  bool          getLowerBitRateConstraintFlag() const { return m_lowerBitRateConstraintFlag; }
  void          setLowerBitRateConstraintFlag(bool b) { m_lowerBitRateConstraintFlag = b; }
  bool          getAllLayersIndependentConstraintFlag() const { return m_allLayersIndependentConstraintFlag; }
  void          setAllLayersIndependentConstraintFlag(bool b) { m_allLayersIndependentConstraintFlag = b; }
  bool          getNoMrlConstraintFlag() const { return m_noMrlConstraintFlag; }
  void          setNoMrlConstraintFlag(bool b) { m_noMrlConstraintFlag = b; }
  bool          getNoIspConstraintFlag() const { return m_noIspConstraintFlag; }
  void          setNoIspConstraintFlag(bool b) { m_noIspConstraintFlag = b; }
  bool          getNoMipConstraintFlag() const { return m_noMipConstraintFlag; }
  void          setNoMipConstraintFlag(bool b) { m_noMipConstraintFlag = b; }
  bool          getNoLfnstConstraintFlag() const { return m_noLfnstConstraintFlag; }
  void          setNoLfnstConstraintFlag(bool b) { m_noLfnstConstraintFlag = b; }
  bool          getNoMmvdConstraintFlag() const { return m_noMmvdConstraintFlag; }
  void          setNoMmvdConstraintFlag(bool b) { m_noMmvdConstraintFlag = b; }
  bool          getNoSmvdConstraintFlag() const { return m_noSmvdConstraintFlag; }
  void          setNoSmvdConstraintFlag(bool b) { m_noSmvdConstraintFlag = b; }
  bool          getNoProfConstraintFlag() const { return m_noProfConstraintFlag; }
  void          setNoProfConstraintFlag(bool b) { m_noProfConstraintFlag = b; }
  bool          getNoPaletteConstraintFlag() const { return m_noPaletteConstraintFlag; }
  void          setNoPaletteConstraintFlag(bool b) { m_noPaletteConstraintFlag = b; }
  bool          getNoActConstraintFlag() const { return m_noActConstraintFlag; }
  void          setNoActConstraintFlag(bool b) { m_noActConstraintFlag = b; }
  bool          getNoLmcsConstraintFlag() const { return m_noLmcsConstraintFlag; }
  void          setNoLmcsConstraintFlag(bool b) { m_noLmcsConstraintFlag = b; }
  bool          getNoExplicitScaleListConstraintFlag() const { return m_noExplicitScaleListConstraintFlag; }
  void          setNoExplicitScaleListConstraintFlag(bool b) { m_noExplicitScaleListConstraintFlag = b; }
  bool          getNoVirtualBoundaryConstraintFlag() const { return m_noVirtualBoundaryConstraintFlag; }
  void          setNoVirtualBoundaryConstraintFlag(bool b) { m_noVirtualBoundaryConstraintFlag = b; }
  bool          getNoMttConstraintFlag() const { return m_noMttConstraintFlag; }
  void          setNoMttConstraintFlag(bool bVal) { m_noMttConstraintFlag = bVal; }
  bool          getNoChromaQpOffsetConstraintFlag() const { return m_noChromaQpOffsetConstraintFlag; }
  void          setNoChromaQpOffsetConstraintFlag(bool b) { m_noChromaQpOffsetConstraintFlag = b; }
  bool          getNoQtbttDualTreeIntraConstraintFlag() const { return m_noQtbttDualTreeIntraConstraintFlag; }
  void          setNoQtbttDualTreeIntraConstraintFlag(bool bVal) { m_noQtbttDualTreeIntraConstraintFlag = bVal; }
  int           getMaxLog2CtuSizeConstraintIdc() const { return m_maxLog2CtuSizeConstraintIdc; }
  void          setMaxLog2CtuSizeConstraintIdc(int idc) { m_maxLog2CtuSizeConstraintIdc = idc; }
  bool          getNoPartitionConstraintsOverrideConstraintFlag() const { return m_noPartitionConstraintsOverrideConstraintFlag; }
  void          setNoPartitionConstraintsOverrideConstraintFlag(bool bVal) { m_noPartitionConstraintsOverrideConstraintFlag = bVal; }
  bool          getNoSaoConstraintFlag() const { return m_noSaoConstraintFlag; }
  void          setNoSaoConstraintFlag(bool bVal) { m_noSaoConstraintFlag = bVal; }
  bool          getNoAlfConstraintFlag() const { return m_noAlfConstraintFlag; }
  void          setNoAlfConstraintFlag(bool bVal) { m_noAlfConstraintFlag = bVal; }
  bool          getNoCCAlfConstraintFlag() const { return m_noCCAlfConstraintFlag; }
  void          setNoCCAlfConstraintFlag(bool val) { m_noCCAlfConstraintFlag = val; }
  bool          getNoJointCbCrConstraintFlag() const { return m_noJointCbCrConstraintFlag; }
  void          setNoJointCbCrConstraintFlag(bool bVal) { m_noJointCbCrConstraintFlag = bVal; }
  bool          getNoWeightedPredictionConstraintFlag() const { return m_noWeightedPredictionConstraintFlag; }
  void          setNoWeightedPredictionConstraintFlag(bool bVal) { m_noWeightedPredictionConstraintFlag = bVal; }
  bool          getNoRefWraparoundConstraintFlag() const { return m_noRefWraparoundConstraintFlag; }
  void          setNoRefWraparoundConstraintFlag(bool bVal) { m_noRefWraparoundConstraintFlag = bVal; }
  bool          getNoTemporalMvpConstraintFlag() const { return m_noTemporalMvpConstraintFlag; }
  void          setNoTemporalMvpConstraintFlag(bool bVal) { m_noTemporalMvpConstraintFlag = bVal; }
  bool          getNoSbtmvpConstraintFlag() const { return m_noSbtmvpConstraintFlag; }
  void          setNoSbtmvpConstraintFlag(bool bVal) { m_noSbtmvpConstraintFlag = bVal; }
  bool          getNoAmvrConstraintFlag() const { return m_noAmvrConstraintFlag; }
  void          setNoAmvrConstraintFlag(bool bVal) { m_noAmvrConstraintFlag = bVal; }
  bool          getNoBdofConstraintFlag() const { return m_noBdofConstraintFlag; }
  void          setNoBdofConstraintFlag(bool bVal) { m_noBdofConstraintFlag = bVal; }
  bool          getNoDmvrConstraintFlag() const { return m_noDmvrConstraintFlag; }
  void          setNoDmvrConstraintFlag(bool bVal) { m_noDmvrConstraintFlag = bVal; }
  bool          getNoCclmConstraintFlag() const { return m_noCclmConstraintFlag; }
  void          setNoCclmConstraintFlag(bool bVal) { m_noCclmConstraintFlag = bVal; }
  bool          getNoMtsConstraintFlag() const { return m_noMtsConstraintFlag; }
  void          setNoMtsConstraintFlag(bool bVal) { m_noMtsConstraintFlag = bVal; }
  bool          getNoSbtConstraintFlag() const { return m_noSbtConstraintFlag; }
  void          setNoSbtConstraintFlag(bool bVal) { m_noSbtConstraintFlag = bVal; }
  bool          getNoAffineMotionConstraintFlag() const { return m_noAffineMotionConstraintFlag; }
  void          setNoAffineMotionConstraintFlag(bool bVal) { m_noAffineMotionConstraintFlag = bVal; }
  bool          getNoBcwConstraintFlag() const { return m_noBcwConstraintFlag; }
  void          setNoBcwConstraintFlag(bool bVal) { m_noBcwConstraintFlag = bVal; }
  bool          getNoIbcConstraintFlag() const { return m_noIbcConstraintFlag; }
  void          setNoIbcConstraintFlag(bool bVal) { m_noIbcConstraintFlag = bVal; }
  bool          getNoCiipConstraintFlag() const { return m_noCiipConstraintFlag; }
  void          setNoCiipConstraintFlag(bool bVal) { m_noCiipConstraintFlag = bVal; }
  bool          getNoGeoConstraintFlag() const { return m_noGeoConstraintFlag; }
  void          setNoGeoConstraintFlag(bool bVal) { m_noGeoConstraintFlag = bVal; }
  bool          getNoLadfConstraintFlag() const { return m_noLadfConstraintFlag; }
  void          setNoLadfConstraintFlag(bool bVal) { m_noLadfConstraintFlag = bVal; }
  bool          getNoTransformSkipConstraintFlag() const { return m_noTransformSkipConstraintFlag; }
  void          setNoTransformSkipConstraintFlag(bool bVal) { m_noTransformSkipConstraintFlag = bVal; }
  bool          getNoLumaTransformSize64ConstraintFlag() const { return m_noLumaTransformSize64ConstraintFlag; }
  void          setNoLumaTransformSize64ConstraintFlag(bool bVal) { m_noLumaTransformSize64ConstraintFlag = bVal; }
  bool          getNoBDPCMConstraintFlag() const { return m_noBDPCMConstraintFlag; }
  void          setNoBDPCMConstraintFlag(bool bVal) { m_noBDPCMConstraintFlag = bVal; }
  bool          getNoQpDeltaConstraintFlag() const { return m_noQpDeltaConstraintFlag; }
  void          setNoQpDeltaConstraintFlag(bool bVal) { m_noQpDeltaConstraintFlag = bVal; }
  bool          getNoDepQuantConstraintFlag() const { return m_noDepQuantConstraintFlag; }
  void          setNoDepQuantConstraintFlag(bool bVal) { m_noDepQuantConstraintFlag = bVal; }
  bool          getNoSignDataHidingConstraintFlag() const { return m_noSignDataHidingConstraintFlag; }
  void          setNoSignDataHidingConstraintFlag(bool bVal) { m_noSignDataHidingConstraintFlag = bVal; }
  bool          getNoMixedNaluTypesInPicConstraintFlag() const    { return m_noMixedNaluTypesInPicConstraintFlag; }
  void          setNoMixedNaluTypesInPicConstraintFlag(bool bVal) { m_noMixedNaluTypesInPicConstraintFlag = bVal; }
  bool          getNoTrailConstraintFlag() const { return m_noTrailConstraintFlag; }
  void          setNoTrailConstraintFlag(bool bVal) { m_noTrailConstraintFlag = bVal; }
  bool          getNoStsaConstraintFlag() const { return m_noStsaConstraintFlag; }
  void          setNoStsaConstraintFlag(bool bVal) { m_noStsaConstraintFlag = bVal; }
  bool          getNoRaslConstraintFlag() const { return m_noRaslConstraintFlag; }
  void          setNoRaslConstraintFlag(bool bVal) { m_noRaslConstraintFlag = bVal; }
  bool          getNoRadlConstraintFlag() const { return m_noRadlConstraintFlag; }
  void          setNoRadlConstraintFlag(bool bVal) { m_noRadlConstraintFlag = bVal; }
  bool          getNoIdrConstraintFlag() const { return m_noIdrConstraintFlag; }
  void          setNoIdrConstraintFlag(bool bVal) { m_noIdrConstraintFlag = bVal; }
  bool          getNoCraConstraintFlag() const { return m_noCraConstraintFlag; }
  void          setNoCraConstraintFlag(bool bVal) { m_noCraConstraintFlag = bVal; }
  bool          getNoGdrConstraintFlag() const { return m_noGdrConstraintFlag; }
  void          setNoGdrConstraintFlag(bool bVal) { m_noGdrConstraintFlag = bVal; }
  bool          getNoApsConstraintFlag() const { return m_noApsConstraintFlag; }
  void          setNoApsConstraintFlag(bool bVal) { m_noApsConstraintFlag = bVal; }
  
  friend bool             operator == (const ConstraintInfo& op1, const ConstraintInfo& op2);
  friend bool             operator != (const ConstraintInfo& op1, const ConstraintInfo& op2);
};

class ProfileTierLevel
{
  Tier                  m_tierFlag                = Tier::MAIN;
  Profile::Name         m_profileIdc              = Profile::NONE;
  uint8_t               m_numSubProfile           = 0;
  std::vector<uint32_t> m_subProfileIdc;
  vvdecLevel            m_levelIdc                = vvdecLevel::VVDEC_LEVEL_NONE;
  bool                  m_frameOnlyConstraintFlag = true;
  bool                  m_multiLayerEnabledFlag   = false;
  ConstraintInfo        m_constraintInfo;
  bool                  m_subLayerLevelPresentFlag[MAX_TLAYER - 1] = { 0 };
  vvdecLevel            m_subLayerLevelIdc        [MAX_TLAYER]     = { vvdecLevel::VVDEC_LEVEL_NONE };

public:
  ProfileTierLevel() = default;
  CLASS_COPY_MOVE_DEFAULT( ProfileTierLevel )

  Tier                    getTierFlag() const                         { return m_tierFlag;                    }
  void                    setTierFlag(Tier x)                         { m_tierFlag = x;                       }

  Profile::Name           getProfileIdc() const                       { return m_profileIdc;                  }
  void                    setProfileIdc(Profile::Name x)              { m_profileIdc = x;                     }

  uint8_t                 getNumSubProfile() const                    { return m_numSubProfile; }
  void                    setNumSubProfile(uint8_t x)                 { m_numSubProfile = x; m_subProfileIdc.resize(m_numSubProfile); }

  uint32_t                getSubProfileIdc(int i) const               { return m_subProfileIdc[i]; }
  void                    setSubProfileIdc(int i, uint32_t x)         { m_subProfileIdc[i] = x; }

  vvdecLevel              getLevelIdc() const                         { return m_levelIdc;                    }
  void                    setLevelIdc(vvdecLevel x)                   { m_levelIdc = x;                       }

  bool                    getFrameOnlyConstraintFlag() const          { return m_frameOnlyConstraintFlag; }
  void                    setFrameOnlyConstraintFlag(bool x)          { m_frameOnlyConstraintFlag = x;    }

  bool                    getMultiLayerEnabledFlag() const           { return m_multiLayerEnabledFlag; }
  void                    setMultiLayerEnabledFlag(bool x)           { m_multiLayerEnabledFlag = x;    }

  ConstraintInfo*         getConstraintInfo()                        { return &m_constraintInfo; }
  const ConstraintInfo*   getConstraintInfo() const                  { return &m_constraintInfo; }

  bool                    getSubLayerLevelPresentFlag(int i) const   { return m_subLayerLevelPresentFlag[i];   }
  void                    setSubLayerLevelPresentFlag(int i, bool x) { m_subLayerLevelPresentFlag[i] = x;      }

  vvdecLevel              getSubLayerLevelIdc(int i) const           { return m_subLayerLevelIdc[i];   }
  void                    setSubLayerLevelIdc(int i, vvdecLevel x)   { m_subLayerLevelIdc[i] = x;      }

  friend bool             operator == (const ProfileTierLevel& op1, const ProfileTierLevel& op2);
  friend bool             operator != (const ProfileTierLevel& op1, const ProfileTierLevel& op2);
};

struct HrdSubLayerInfo
{
  bool     fixedPicRateFlag          = 0;
  bool     fixedPicRateWithinCvsFlag = 0;
  uint32_t picDurationInTcMinus1     = 0;
  bool     lowDelayHrdFlag           = 0;
  uint32_t cpbCntMinus1              = 0;
  uint32_t bitRateValueMinus1[MAX_CPB_CNT][2] = { { 0 } };
  uint32_t cpbSizeValue      [MAX_CPB_CNT][2] = { { 0 } };
  uint32_t ducpbSizeValue    [MAX_CPB_CNT][2] = { { 0 } };
  bool     cbrFlag           [MAX_CPB_CNT][2] = { { 0 } };
  uint32_t duBitRateValue    [MAX_CPB_CNT][2] = { { 0 } };
};

class SliceReshapeInfo
{
public:
  bool     sliceReshaperEnableFlag                   = 0;
  bool     sliceReshaperModelPresentFlag             = 0;
  unsigned enableChromaAdj                           = 0;
  uint32_t reshaperModelMinBinIdx                    = 0;
  uint32_t reshaperModelMaxBinIdx                    = 0;
  int      reshaperModelBinCWDelta[PIC_CODE_CW_BINS] = { 0 };
  int      maxNbitsNeededDeltaCW                     = 0;
  int      chrResScalingOffset                       = 0;

  void reset()
  {
    memset( this, 0, sizeof( *this ) );
  }
};

template<class T>
struct BasePS: public std::enable_shared_from_this<T>
{
  std::shared_ptr<T> getSharedPtr()
  {
    return (static_cast<T*>(this))->shared_from_this();
  }
  std::shared_ptr<const T> getSharedPtr() const
  {
    return (static_cast<const T*>(this))->shared_from_this();
  }

  void clearChangedFlag()
  {
    m_changedFlag = false;
  }

  bool getChangedFlag() const
  {
    return m_changedFlag;
  }

  bool             m_changedFlag = false;

  template<class Tf, int MAX_ID> friend class ParameterSetMap;
};

class DCI
{
private:
  int                           m_maxSubLayersMinus1 = 0;
  std::vector<ProfileTierLevel> m_profileTierLevel;

public:

  int                     getMaxSubLayersMinus1()                                 const { return m_maxSubLayersMinus1; }
  void                    setMaxSubLayersMinus1(int val)                                { m_maxSubLayersMinus1 = val; }

  size_t                  getNumPTLs()                                            const { return m_profileTierLevel.size(); }
  const ProfileTierLevel& getProfileTierLevel(int idx)                            const { return m_profileTierLevel[idx]; }
  void                    setProfileTierLevel( std::vector<ProfileTierLevel>&& val )    { m_profileTierLevel = std::move( val ); }

  bool IsIndenticalDCI(const DCI& comparedDCI) const
  {
    if(m_maxSubLayersMinus1 != comparedDCI.m_maxSubLayersMinus1) return false;
    if(m_profileTierLevel != comparedDCI.m_profileTierLevel) return false;
    return true;
  }
};

class OlsHrdParams
{
private:
  bool     m_fixedPicRateGeneralFlag   = false;
  bool     m_fixedPicRateWithinCvsFlag = false;
  uint32_t m_elementDurationInTcMinus1 = false;
  bool     m_lowDelayHrdFlag           = false;

  uint32_t m_bitRateValueMinus1  [MAX_CPB_CNT][2] = { { 0 } };
  uint32_t m_cpbSizeValueMinus1  [MAX_CPB_CNT][2] = { { 0 } };
  uint32_t m_ducpbSizeValueMinus1[MAX_CPB_CNT][2] = { { 0 } };
  uint32_t m_duBitRateValueMinus1[MAX_CPB_CNT][2] = { { 0 } };
  bool     m_cbrFlag             [MAX_CPB_CNT][2] = { { 0 } };
public:

  void      setFixedPicRateGeneralFlag(bool flag)                             { m_fixedPicRateGeneralFlag = flag;    }
  bool      getFixedPicRateGeneralFlag() const                                { return m_fixedPicRateGeneralFlag;    }
  void      setFixedPicRateWithinCvsFlag(bool flag)                           { m_fixedPicRateWithinCvsFlag = flag;  }
  bool      getFixedPicRateWithinCvsFlag() const                              { return m_fixedPicRateWithinCvsFlag;  }
  void      setElementDurationInTcMinus1(uint32_t value)                      { m_elementDurationInTcMinus1 = value; }
  uint32_t  getElementDurationInTcMinus1() const                              { return m_elementDurationInTcMinus1;  }
  void      setLowDelayHrdFlag(bool flag)                                     { m_lowDelayHrdFlag = flag;            }
  bool      getLowDelayHrdFlag() const                                        { return m_lowDelayHrdFlag;            }
  void      setBitRateValueMinus1(int cpbcnt, int nalOrVcl, uint32_t value)   { m_bitRateValueMinus1[cpbcnt][nalOrVcl] = value;   }
  uint32_t  getBitRateValueMinus1(int cpbcnt, int nalOrVcl) const             { return m_bitRateValueMinus1[cpbcnt][nalOrVcl];    }

  void      setCpbSizeValueMinus1(int cpbcnt, int nalOrVcl, uint32_t value)   { m_cpbSizeValueMinus1[cpbcnt][nalOrVcl] = value;   }
  uint32_t  getCpbSizeValueMinus1(int cpbcnt, int nalOrVcl) const             { return m_cpbSizeValueMinus1[cpbcnt][nalOrVcl];    }
  void      setDuCpbSizeValueMinus1(int cpbcnt, int nalOrVcl, uint32_t value) { m_ducpbSizeValueMinus1[cpbcnt][nalOrVcl] = value; }
  uint32_t  getDuCpbSizeValueMinus1(int cpbcnt, int nalOrVcl) const           { return m_ducpbSizeValueMinus1[cpbcnt][nalOrVcl];  }
  void      setDuBitRateValueMinus1(int cpbcnt, int nalOrVcl, uint32_t value) { m_duBitRateValueMinus1[cpbcnt][nalOrVcl] = value; }
  uint32_t  getDuBitRateValueMinus1(int cpbcnt, int nalOrVcl) const           { return m_duBitRateValueMinus1[cpbcnt][nalOrVcl];  }
  void      setCbrFlag(int cpbcnt, int nalOrVcl, bool value)                  { m_cbrFlag[cpbcnt][nalOrVcl] = value;              }
  bool      getCbrFlag(int cpbcnt, int nalOrVcl) const                        { return m_cbrFlag[cpbcnt][nalOrVcl];               }
};

class GeneralHrdParams
{
private:
  uint32_t m_numUnitsInTick                   = 0;
  uint32_t m_timeScale                        = 0;
  bool     m_generalNalHrdParamsPresentFlag   = 0;
  bool     m_generalVclHrdParamsPresentFlag   = 0;
  bool     m_generalSamePicTimingInAllOlsFlag = false;
  uint32_t m_tickDivisorMinus2                = 0;
  bool     m_generalDuHrdParamsPresentFlag    = false;
  uint32_t m_bitRateScale                     = 0;
  uint32_t m_cpbSizeScale                     = 0;
  uint32_t m_cpbSizeDuScale                   = 0;
  uint32_t m_hrdCpbCntMinus1                  = 0;

public:
  void      setNumUnitsInTick(uint32_t value)                { m_numUnitsInTick = value; }
  uint32_t  getNumUnitsInTick() const                        { return m_numUnitsInTick; }

  void      setTimeScale(uint32_t value)                     { m_timeScale = value; }
  uint32_t  getTimeScale() const                             { return m_timeScale; }

  void      setGeneralNalHrdParamsPresentFlag( bool flag )   { m_generalNalHrdParamsPresentFlag = flag; }
  bool      getGeneralNalHrdParamsPresentFlag() const        { return m_generalNalHrdParamsPresentFlag; }

  void      setGeneralVclHrdParamsPresentFlag( bool flag )   { m_generalVclHrdParamsPresentFlag = flag; }
  bool      getGeneralVclHrdParamsPresentFlag() const        { return m_generalVclHrdParamsPresentFlag; }

  void      setGeneralSamePicTimingInAllOlsFlag( bool flag ) { m_generalSamePicTimingInAllOlsFlag = flag; }
  bool      getGeneralSamePicTimingInAllOlsFlag() const      { return m_generalSamePicTimingInAllOlsFlag; }

  void      setTickDivisorMinus2( uint32_t value )           { m_tickDivisorMinus2 = value;            }
  uint32_t  getTickDivisorMinus2( ) const                    { return m_tickDivisorMinus2;             }

  void      setGeneralDuHrdParamsPresentFlag( bool flag )    { m_generalDuHrdParamsPresentFlag = flag; }
  bool      getGeneralDuHrdParamsPresentFlag( ) const        { return m_generalDuHrdParamsPresentFlag; }

  void      setBitRateScale( uint32_t value )                { m_bitRateScale = value;                 }
  uint32_t  getBitRateScale( ) const                         { return m_bitRateScale;                  }

  void      setCpbSizeScale( uint32_t value )                { m_cpbSizeScale = value;                 }
  uint32_t  getCpbSizeScale( ) const                         { return m_cpbSizeScale;                  }
  void      setCpbSizeDuScale( uint32_t value )              { m_cpbSizeDuScale = value;               }
  uint32_t  getCpbSizeDuScale( ) const                       { return m_cpbSizeDuScale;                }

  void      setHrdCpbCntMinus1(uint32_t value)               { m_hrdCpbCntMinus1 = value;              }
  uint32_t  getHrdCpbCntMinus1() const                       { return m_hrdCpbCntMinus1;               }
};

class HRD
{
public:
  HRD()  = default;
  ~HRD() = default;

  void                    setGeneralHrdParameters(GeneralHrdParams &generalHrdParam) { m_generalHrdParams = generalHrdParam; }
  GeneralHrdParams        getGeneralHrdParameters() const { return m_generalHrdParams; }
  const GeneralHrdParams& getGeneralHrdParameters() { return m_generalHrdParams; }

  void                setOlsHrdParameters(int tLayter, OlsHrdParams &olsHrdParam) { m_olsHrdParams[tLayter] = olsHrdParam; }
  OlsHrdParams        getOlsHrdParameters() { return m_olsHrdParams[0]; }
  OlsHrdParams*       getOlsHrdParametersAddr() { return &m_olsHrdParams[0]; }
  const OlsHrdParams& getOlsHrdParameters() const { return m_olsHrdParams[0]; }


  void                           setBufferingPeriodSEI(const vvdecSEIBufferingPeriod* bp)  { m_bufferingPeriodSEI = *bp; m_bufferingPeriodInitialized = true; }
  const vvdecSEIBufferingPeriod* getBufferingPeriodSEI() const                        { return m_bufferingPeriodInitialized ? &m_bufferingPeriodSEI : nullptr; }

  void                           setPictureTimingSEI(const vvdecSEIPictureTiming* pt)  { m_pictureTimingSEI = *pt; m_pictureTimingAvailable = true; }
  const vvdecSEIPictureTiming*   getPictureTimingSEI() const                      { return m_pictureTimingAvailable ? &m_pictureTimingSEI : nullptr; }

protected:
  GeneralHrdParams             m_generalHrdParams;
  OlsHrdParams                 m_olsHrdParams[MAX_TLAYER];
  bool                         m_bufferingPeriodInitialized = false;
  vvdecSEIBufferingPeriod      m_bufferingPeriodSEI;
  bool                         m_pictureTimingAvailable     = false;
  vvdecSEIPictureTiming        m_pictureTimingSEI;

};


class TimingInfo
{
  bool     m_timingInfoPresentFlag       = false;
  uint32_t m_numUnitsInTick              = 1001;
  uint32_t m_timeScale                   = 60000;
  bool     m_pocProportionalToTimingFlag = false;
  int      m_numTicksPocDiffOneMinus1    = 0;

public:
  void     setTimingInfoPresentFlag( bool flag )   { m_timingInfoPresentFlag = flag;       }
  bool     getTimingInfoPresentFlag( ) const       { return m_timingInfoPresentFlag;       }

  void     setNumUnitsInTick( uint32_t value )     { m_numUnitsInTick = value;             }
  uint32_t getNumUnitsInTick( ) const              { return m_numUnitsInTick;              }

  void     setTimeScale( uint32_t value )          { m_timeScale = value;                  }
  uint32_t getTimeScale( ) const                   { return m_timeScale;                   }

  void     setPocProportionalToTimingFlag(bool x)  { m_pocProportionalToTimingFlag = x;    }
  bool     getPocProportionalToTimingFlag( ) const { return m_pocProportionalToTimingFlag; }

  void     setNumTicksPocDiffOneMinus1(int x)      { m_numTicksPocDiffOneMinus1 = x;       }
  int      getNumTicksPocDiffOneMinus1( ) const    { return m_numTicksPocDiffOneMinus1;    }
};

struct ChromaQpOffset
{
  int CbOffset        = 0;
  int CrOffset        = 0;
  int JointCbCrOffset = 0;

  int  get( ComponentID compID ) const { return ( compID == COMPONENT_Y ) ? 0 : ( compID == COMPONENT_Cb ? CbOffset : compID == COMPONENT_Cr ? CrOffset : JointCbCrOffset ); }
  void set( ComponentID compID, int val )
  {
    if( compID == COMPONENT_Cb )
    {
      CbOffset = val;
    }
    else if( compID == COMPONENT_Cr )
    {
      CrOffset = val;
    }
    else if( compID == JOINT_CbCr )
    {
      JointCbCrOffset = val;
    }
    else
    {
      THROW_RECOVERABLE( "Invalid chroma QP offset" );
    }
  }
};

struct ChromaQpMappingTableParams {
  int              m_qpBdOffset                   = 12;
  bool             m_sameCQPTableForAllChromaFlag = true;
  int              m_numQpTables                  = 1;
  int              m_qpTableStartMinus26   [MAX_NUM_CQP_MAPPING_TABLES] = { 0 };
  int              m_numPtsInCQPTableMinus1[MAX_NUM_CQP_MAPPING_TABLES] = { 0 };
  std::vector<int> m_deltaQpInValMinus1    [MAX_NUM_CQP_MAPPING_TABLES];
  std::vector<int> m_deltaQpOutVal         [MAX_NUM_CQP_MAPPING_TABLES];

  ChromaQpMappingTableParams()
  {
    m_deltaQpInValMinus1[0].resize( 1, 0 );
    m_deltaQpOutVal     [0].resize( 1, 0 );
  }
  ~ChromaQpMappingTableParams() = default;
  CLASS_COPY_MOVE_DEFAULT( ChromaQpMappingTableParams )

  void      setSameCQPTableForAllChromaFlag(bool b)                             { m_sameCQPTableForAllChromaFlag = b; }
  bool      getSameCQPTableForAllChromaFlag()                             const { return m_sameCQPTableForAllChromaFlag; }
  void      setNumQpTables(int n)                                               { m_numQpTables = n; }
  int       getNumQpTables()                                              const { return m_numQpTables; }
  void      setQpTableStartMinus26(int tableIdx, int n)                         { m_qpTableStartMinus26[tableIdx] = n; }
  int       getQpTableStartMinus26(int tableIdx)                          const { return m_qpTableStartMinus26[tableIdx]; }
  void      setNumPtsInCQPTableMinus1(int tableIdx, int n)                      { m_numPtsInCQPTableMinus1[tableIdx] = n; }
  int       getNumPtsInCQPTableMinus1(int tableIdx)                       const { return m_numPtsInCQPTableMinus1[tableIdx]; }
  void      setDeltaQpInValMinus1(int tableIdx, std::vector<int> &inVals)       { m_deltaQpInValMinus1[tableIdx] = inVals; }
  int       getDeltaQpInValMinus1(int tableIdx, int idx)                  const { return m_deltaQpInValMinus1[tableIdx][idx]; }
  void      setDeltaQpOutVal(int tableIdx, std::vector<int> &outVals)           { m_deltaQpOutVal[tableIdx] = outVals; }
  int       getDeltaQpOutVal(int tableIdx, int idx)                       const { return m_deltaQpOutVal[tableIdx][idx]; }
};

struct ChromaQpMappingTable : ChromaQpMappingTableParams
{
  ChromaQpMappingTable()  = default;
  ~ChromaQpMappingTable() = default;
  CLASS_COPY_MOVE_DEFAULT( ChromaQpMappingTable )

  explicit ChromaQpMappingTable( const ChromaQpMappingTableParams& params ) : ChromaQpMappingTableParams( params ) {}
  // ChromaQpMappingTable& operator=( const ChromaQpMappingTableParams& params ) { *this = ChromaQpMappingTable( params ); return *this; }

  std::vector<int> m_chromaQpMappingTables[MAX_NUM_CQP_MAPPING_TABLES];

  int       getMappedChromaQpValue(ComponentID compID, const int qpVal)  const { return m_chromaQpMappingTables[m_sameCQPTableForAllChromaFlag ? 0 : (int)compID - 1].at(qpVal + m_qpBdOffset); }
  void      deriveChromaQPMappingTables();
};

class SliceMap
{
private:
  uint32_t               m_sliceID         = 0;               //!< slice identifier (slice index for rectangular slices, slice address for raser-scan slices)
  uint32_t               m_numCtuInSlice   = 0;               //!< number of CTUs in the slice
  std::vector<uint32_t>  m_ctuAddrInSlice;                    //!< raster-scan addresses of all the CTUs in the slice

public:
  SliceMap();
  ~SliceMap();

  void                         setSliceID( uint32_t u )           { m_sliceID = u;            }
  uint32_t                     getSliceID() const                 { return m_sliceID;         }
  uint32_t                     getNumCtuInSlice() const           { return m_numCtuInSlice;   }
  const std::vector<uint32_t>& getCtuAddrList() const             { return m_ctuAddrInSlice;  }
  uint32_t                     getCtuAddrInSlice( int idx ) const { CHECK(idx >= m_ctuAddrInSlice.size(), "CTU index exceeds number of CTUs in slice."); return m_ctuAddrInSlice[idx]; }

  void  resetSliceMap()
  {
    m_sliceID         = 0;
    m_numCtuInSlice   = 0;
    m_ctuAddrInSlice.clear();
  }

  void  addCtusToSlice( uint32_t startX, uint32_t stopX, uint32_t startY, uint32_t stopY, uint32_t picWidthInCtbsY )
  {
    CHECK( startX >= stopX || startY >= stopY, "Invalid slice definition");
    m_ctuAddrInSlice.reserve( m_ctuAddrInSlice.size() + ( stopY - startY ) * ( stopX - startX ) );
    for( uint32_t ctbY = startY; ctbY < stopY; ctbY++ )
    {
      for( uint32_t ctbX = startX; ctbX < stopX; ctbX++ )
      {
        m_ctuAddrInSlice.push_back( ctbY * picWidthInCtbsY + ctbX );
        m_numCtuInSlice++;
      }
    }
  }
};

class RectSlice
{
private:
  uint32_t         m_tileIdx            = 0;                //!< tile index corresponding to the first CTU in the slice
  uint32_t         m_sliceWidthInTiles  = 0;                //!< slice width in units of tiles
  uint32_t         m_sliceHeightInTiles = 0;                //!< slice height in units of tiles
  uint32_t         m_numSlicesInTile    = 0;                //!< number of slices in current tile for the special case of multiple slices inside a single tile
  uint32_t         m_sliceHeightInCtu   = 0;                //!< slice height in units of CTUs for the special case of multiple slices inside a single tile
  
public:
  RectSlice() = default;
  ~RectSlice() = default;

  void             setSliceWidthInTiles( uint32_t u )   { m_sliceWidthInTiles = u;      }
  uint32_t         getSliceWidthInTiles( ) const        { return  m_sliceWidthInTiles;  }
  void             setSliceHeightInTiles( uint32_t u )  { m_sliceHeightInTiles = u;     }
  uint32_t         getSliceHeightInTiles( ) const       { return  m_sliceHeightInTiles; }
  void             setNumSlicesInTile( uint32_t u )     { m_numSlicesInTile = u;        }
  uint32_t         getNumSlicesInTile( ) const          { return  m_numSlicesInTile;    }
  void             setSliceHeightInCtu( uint32_t u )    { m_sliceHeightInCtu = u;       }
  uint32_t         getSliceHeightInCtu( ) const         { return  m_sliceHeightInCtu;   }
  void             setTileIdx( uint32_t u )             { m_tileIdx = u;                }
  uint32_t         getTileIdx( ) const                  { return  m_tileIdx;            }
};

class SubPic
{
private:
  uint32_t         m_subPicID                          = 0;      //!< ID of subpicture
  uint32_t         m_subPicIdx                         = 0;      //!< Index of subpicture
  uint32_t         m_numCTUsInSubPic                   = 0;      //!< number of CTUs contained in this sub-picture
  uint32_t         m_subPicCtuTopLeftX                 = 0;      //!< horizontal position of top left CTU of the subpicture in unit of CTU
  uint32_t         m_subPicCtuTopLeftY                 = 0;      //!< vertical position of top left CTU of the subpicture in unit of CTU
  uint32_t         m_subPicWidth                       = 0;      //!< the width of subpicture in units of CTU
  uint32_t         m_subPicHeight                      = 0;      //!< the height of subpicture in units of CTU
  uint32_t         m_subPicWidthInLumaSample           = 0;      //!< the width of subpicture in units of luma sample
  uint32_t         m_subPicHeightInLumaSample          = 0;      //!< the height of subpicture in units of luma sample
  uint32_t         m_firstCtuInSubPic                  = 0;      //!< the raster scan index of the first CTU in a subpicture
  uint32_t         m_lastCtuInSubPic                   = 0;      //!< the raster scan index of the last CTU in a subpicture
  uint32_t         m_subPicLeft                        = 0;      //!< the position of left boundary
  uint32_t         m_subPicRight                       = 0;      //!< the position of right boundary
  uint32_t         m_subPicTop                         = 0;      //!< the position of top boundary
  uint32_t         m_subPicBottom                      = 0;      //!< the position of bottom boundary
  std::vector<uint32_t> m_ctuAddrInSubPic;                       //!< raster scan addresses of all the CTUs in the slice

  bool             m_treatedAsPicFlag                  = false;  //!< whether the subpicture is treated as a picture in the decoding process excluding in-loop filtering operations
  bool             m_loopFilterAcrossSubPicEnabledFlag = false;  //!< whether in-loop filtering operations may be performed across the boundaries of the subpicture
  uint32_t         m_numSlicesInSubPic                 = 0;      //!< Number of slices contained in this subpicture

public:
  SubPic() = default;
  ~SubPic() = default;

  void             setSubPicID          (uint32_t u)       {         m_subPicID = u;              }
  uint32_t         getSubPicID          ()           const { return  m_subPicID;                  }
  void             setSubPicIdx         (uint32_t u)       {         m_subPicIdx = u;             }
  uint32_t         getSubPicIdx         ()           const { return  m_subPicIdx;                 }
  void             setNumCTUsInSubPic   (uint32_t u)       {         m_numCTUsInSubPic = u;       }
  uint32_t         getNumCTUsInSubPic   ()           const { return  m_numCTUsInSubPic;           }
  void             setSubPicCtuTopLeftX (uint32_t u)       {         m_subPicCtuTopLeftX = u;     }
  uint32_t         getSubPicCtuTopLeftX ()           const { return  m_subPicCtuTopLeftX;         }
  void             setSubPicCtuTopLeftY (uint32_t u)       {         m_subPicCtuTopLeftY = u;     }
  uint32_t         getSubPicCtuTopLeftY ()           const { return  m_subPicCtuTopLeftY;         }
  void             setSubPicWidthInCTUs (uint32_t u)       {         m_subPicWidth = u;           }
  uint32_t         getSubPicWidthInCTUs ()           const { return  m_subPicWidth;               }
  void             setSubPicHeightInCTUs(uint32_t u)       {         m_subPicHeight = u;          }
  uint32_t         getSubPicHeightInCTUs()           const { return  m_subPicHeight;              }
  void             setFirstCTUInSubPic  (uint32_t u)       {         m_firstCtuInSubPic = u;      }
  uint32_t         getFirstCTUInSubPic  ()           const { return  m_firstCtuInSubPic;          }
  void             setLastCTUInSubPic   (uint32_t u)       {         m_lastCtuInSubPic = u;       }
  uint32_t         getLastCTUInSubPic   ()           const { return  m_lastCtuInSubPic;           }
  void             setSubPicLeft        (uint32_t u)       {         m_subPicLeft = u;            }
  uint32_t         getSubPicLeft        ()           const { return  m_subPicLeft;                }
  void             setSubPicRight       (uint32_t u)       {         m_subPicRight = u;           }
  uint32_t         getSubPicRight       ()           const { return  m_subPicRight;               }
  void             setSubPicTop         (uint32_t u)       {         m_subPicTop = u;             }
  uint32_t         getSubPicTop         ()           const { return  m_subPicTop;                 }
  void             setSubPicBottom      (uint32_t u)       {         m_subPicBottom = u;          }
  uint32_t         getSubPicBottom      ()           const { return  m_subPicBottom;              }

  void             setSubPicWidthInLumaSample (uint32_t u) {         m_subPicWidthInLumaSample = u;   }
  uint32_t         getSubPicWidthInLumaSample()      const { return  m_subPicWidthInLumaSample;       }
  void             setSubPicHeightInLumaSample(uint32_t u) {         m_subPicHeightInLumaSample = u;  }
  uint32_t         getSubPicHeightInLumaSample()     const { return  m_subPicHeightInLumaSample;      }

  Area             getArea()                         const { return Area( m_subPicLeft, m_subPicTop, m_subPicWidth,             m_subPicHeight );             }
  Area             getLumaArea()                     const { return Area( m_subPicLeft, m_subPicTop, m_subPicWidthInLumaSample, m_subPicHeightInLumaSample ); }

  const std::vector<uint32_t>& getCtuAddrList  ()    const { return  m_ctuAddrInSubPic;           }
  void                         clearCTUAddrList()          { m_ctuAddrInSubPic.clear();           }
  void                         addCTUsToSubPic( const std::vector<uint32_t>& ctuAddrInSlice )
  {
    const size_t oldSize = m_ctuAddrInSubPic.size();
    m_ctuAddrInSubPic.resize( m_ctuAddrInSubPic.size() + ctuAddrInSlice.size() );
    std::copy( ctuAddrInSlice.cbegin(), ctuAddrInSlice.cend(), m_ctuAddrInSubPic.begin() + oldSize );
  }
  void                         addAllCtusInPicToSubPic( uint32_t startX, uint32_t stopX, uint32_t startY, uint32_t stopY, uint32_t picWidthInCtbsY )
  {
    CHECK(startX >= stopX || startY >= stopY, "Invalid slice definition");
    m_ctuAddrInSubPic.reserve( m_ctuAddrInSubPic.size() + ( stopY - startY ) * ( stopX - startX ) );
    for( uint32_t ctbY = startY; ctbY < stopY; ctbY++ )
    {
      for( uint32_t ctbX = startX; ctbX < stopX; ctbX++ )
      {
        m_ctuAddrInSubPic.push_back( ctbY * picWidthInCtbsY + ctbX );
      }
    }
  }
  bool             isContainingPos(const Position& pos) const
  {
    return pos.x >= m_subPicLeft && pos.x <= m_subPicRight && pos.y >= m_subPicTop  && pos.y <= m_subPicBottom;
  }
  void             setTreatedAsPicFlag                 (bool u)  {         m_treatedAsPicFlag = u;   }
  bool             getTreatedAsPicFlag                 ()  const { return  m_treatedAsPicFlag;       }
  void             setloopFilterAcrossSubPicEnabledFlag(bool u)  {         m_loopFilterAcrossSubPicEnabledFlag = u; }
  bool             getloopFilterAcrossSubPicEnabledFlag()  const { return  m_loopFilterAcrossSubPicEnabledFlag;     }

  bool             isFirstCTUinSubPic(uint32_t ctuAddr)    { return  ctuAddr == m_firstCtuInSubPic;  }
  bool             isLastCTUinSubPic(uint32_t ctuAddr)     { return  ctuAddr == m_lastCtuInSubPic;   }
  void             setNumSlicesInSubPic( uint32_t val )    { m_numSlicesInSubPic = val; }
  uint32_t         getNumSlicesInSubPic() const            { return m_numSlicesInSubPic; }
  bool             containsCtu(const Position& pos) const
  {
    return pos.x >= m_subPicCtuTopLeftX && pos.x < m_subPicCtuTopLeftX + m_subPicWidth &&
           pos.y >= m_subPicCtuTopLeftY && pos.y < m_subPicCtuTopLeftY + m_subPicHeight;
  }
};


class VPS : public BasePS<VPS>
{
private:
  int                   m_VPSId            = 0;
  uint32_t              m_uiMaxLayers      = 1;

  uint32_t                      m_vpsMaxSubLayers                                       = 1;
  uint32_t                      m_vpsLayerId[MAX_VPS_LAYERS]                            = { 0 };
  bool                          m_vpsAllLayersSameNumSubLayersFlag                      = true;
  bool                          m_vpsAllIndependentLayersFlag                           = true;
  uint32_t                      m_vpsCfgPredDirection[MAX_VPS_SUBLAYERS]                = { 0 };
  bool                          m_vpsIndependentLayerFlag[MAX_VPS_LAYERS]               = { 0 };
  bool                          m_vpsDirectRefLayerFlag[MAX_VPS_LAYERS][MAX_VPS_LAYERS] = { { 0 } };
  uint32_t                      m_vpsMaxTidIlRefPicsPlus1[MAX_VPS_LAYERS]               = { 0 };
  bool                          m_vpsEachLayerIsAnOlsFlag                               = true;
  uint32_t                      m_vpsOlsModeIdc                                         = 0;
  uint32_t                      m_vpsNumOutputLayerSets                                 = 0;
  bool                          m_vpsOlsOutputLayerFlag[MAX_NUM_OLSS][MAX_VPS_LAYERS]   = { { 0 } };
  uint32_t                      m_directRefLayerIdx[MAX_VPS_LAYERS][MAX_VPS_LAYERS]     = { { 0 } };
  uint32_t                      m_generalLayerIdx[MAX_VPS_LAYERS]                       = { 0 };
  uint32_t                      m_vpsNumPtls                                            = 1;
  bool                          m_ptPresentFlag[MAX_NUM_OLSS]                           = { 0 };
  uint32_t                      m_ptlMaxTemporalId[MAX_NUM_OLSS]                        = { 0 };
  std::vector<ProfileTierLevel> m_vpsProfileTierLevel;
  uint32_t                      m_olsPtlIdx[MAX_NUM_OLSS]                               = { 0 };
  uint32_t                      m_interLayerRefIdx[MAX_VPS_LAYERS][MAX_VPS_LAYERS]      = { { 0 } };
  bool                          m_vpsExtensionFlag                                      = false;
  bool                          m_vpsGeneralHrdParamsPresentFlag                        = 0;
  bool                          m_vpsSublayerCpbParamsPresentFlag                       = false;
  uint32_t                      m_numOlsHrdParamsMinus1                                 = 0;
  uint32_t                      m_hrdMaxTid[MAX_NUM_OLSS]                               = { 0 };
  uint32_t                      m_olsHrdIdx[MAX_NUM_OLSS]                               = { 0 };
  GeneralHrdParams              m_generalHrdParams;

  std::vector<int>              m_multiLayerOlsIdxToOlsIdx; // mapping from multi-layer OLS index to OLS index. Initialized in deriveOutputLayerSets()
                                                            // m_multiLayerOlsIdxToOlsIdx[n] is the OLSidx of the n-th multi-layer OLS.
  std::vector<Size>             m_olsDpbPicSize;
  std::vector<int>              m_olsDpbParamsIdx;
  std::vector<std::vector<int>> m_outputLayerIdInOls;
public:
  std::vector<std::vector<OlsHrdParams>> m_olsHrdParams;
  int                                    m_totalNumOLSs                   = 0;
  int                                    m_numMultiLayeredOlss            = 0;
  uint32_t                               m_multiLayerOlsIdx[MAX_NUM_OLSS] = { 0 };
  int                                    m_numDpbParams                   = 0;
  std::vector<DpbParameters>             m_dpbParameters;
  bool                                   m_sublayerDpbParamsPresentFlag   = false;
  std::vector<int>                       m_dpbMaxTemporalId;
  std::vector<int>                       m_targetOutputLayerIdSet;                  ///< set of LayerIds to be outputted
  std::vector<int>                       m_targetLayerIdSet;                        ///< set of LayerIds to be included in the sub-bitstream extraction process.
  int                                    m_iTargetLayer                   = -1;
  std::vector<int>                       m_numOutputLayersInOls;
  std::vector<int>                       m_numLayersInOls;
  std::vector<std::vector<int>>          m_layerIdInOls;
  std::vector<int>                       m_olsDpbChromaFormatIdc;
  std::vector<int>                       m_olsDpbBitDepthMinus8;

public:
  VPS()  = default;
  ~VPS() = default;

  int               getVPSId() const                                     { return m_VPSId;                     }
  void              setVPSId(int i)                                      { m_VPSId = i;                        }

  uint32_t          getMaxLayers() const                                 { return m_uiMaxLayers;               }
  void              setMaxLayers(uint32_t l)                             { m_uiMaxLayers = l;                  }

  bool              getVPSExtensionFlag() const                          { return m_vpsExtensionFlag;          }
  void              setVPSExtensionFlag(bool t)                          { m_vpsExtensionFlag = t;             }

  uint32_t          getPredDirection(uint32_t tmplayer) const            { return m_vpsCfgPredDirection[tmplayer]; }
  void              setPredDirection(uint32_t tmplayer, uint32_t t)      { m_vpsCfgPredDirection[tmplayer] = t; }

  uint32_t          getMaxSubLayers() const                              { return m_vpsMaxSubLayers;           }
  void              setMaxSubLayers(uint32_t value)                      { m_vpsMaxSubLayers = value;          }
  bool              getAllLayersSameNumSublayersFlag() const             { return m_vpsAllLayersSameNumSubLayersFlag; }
  void              setAllLayersSameNumSublayersFlag(bool t)             { m_vpsAllLayersSameNumSubLayersFlag = t;    }

  uint32_t          getLayerId(uint32_t layerIdx) const                  { return m_vpsLayerId[layerIdx];    }
  void              setLayerId(uint32_t layerIdx, uint32_t layerId)      { m_vpsLayerId[layerIdx] = layerId; }

  bool              getAllIndependentLayersFlag() const                  { return m_vpsAllIndependentLayersFlag; }
  void              setAllIndependentLayersFlag(bool t)                  { m_vpsAllIndependentLayersFlag = t;    }

  bool              getIndependentLayerFlag(uint32_t layerIdx) const { return m_vpsIndependentLayerFlag[layerIdx]; }
  void              setIndependentLayerFlag(uint32_t layerIdx, bool t) { m_vpsIndependentLayerFlag[layerIdx] = t; }

  uint32_t          getMaxTidIlRefPicsPlus1(uint32_t layerIdx) const { return m_vpsMaxTidIlRefPicsPlus1[layerIdx]; }
  void              setMaxTidIlRefPicsPlus1(uint32_t layerIdx, uint32_t i) { m_vpsMaxTidIlRefPicsPlus1[layerIdx] = i; }

  bool              getDirectRefLayerFlag(uint32_t layerIdx, uint32_t refLayerIdx) const   { return m_vpsDirectRefLayerFlag[layerIdx][refLayerIdx]; }
  void              setDirectRefLayerFlag(uint32_t layerIdx, uint32_t refLayerIdx, bool t) { m_vpsDirectRefLayerFlag[layerIdx][refLayerIdx] = t;    }

  uint32_t          getDirectRefLayerIdx( uint32_t layerIdx, uint32_t refLayerIdc ) const { return m_directRefLayerIdx[layerIdx][refLayerIdc]; }
  void              setDirectRefLayerIdx( uint32_t layerIdx, uint32_t refLayerIdc, uint32_t refLayerIdx ) { m_directRefLayerIdx[layerIdx][refLayerIdc] = refLayerIdx; }

  uint32_t          getInterLayerRefIdc( uint32_t layerIdx, uint32_t refLayerIdx ) const { return m_interLayerRefIdx[layerIdx][refLayerIdx]; }
  void              setInterLayerRefIdc( uint32_t layerIdx, uint32_t refLayerIdx, uint32_t refLayerIdc ) { m_interLayerRefIdx[layerIdx][refLayerIdx] = refLayerIdc; }

  bool              getEachLayerIsAnOlsFlag() const                      { return m_vpsEachLayerIsAnOlsFlag; }
  void              setEachLayerIsAnOlsFlag(bool t)                      { m_vpsEachLayerIsAnOlsFlag = t;    }

  uint32_t          getOlsModeIdc() const                                { return m_vpsOlsModeIdc; }
  void              setOlsModeIdc(uint32_t t)                            { m_vpsOlsModeIdc = t;    }

  uint32_t          getNumOutputLayerSets() const                        { return m_vpsNumOutputLayerSets; }
  void              setNumOutputLayerSets(uint8_t t)                     { m_vpsNumOutputLayerSets = t;    }

  bool              getOlsOutputLayerFlag(uint32_t ols, uint32_t layer) const   { return m_vpsOlsOutputLayerFlag[ols][layer]; }
  void              setOlsOutputLayerFlag(uint32_t ols, uint32_t layer, bool t) { m_vpsOlsOutputLayerFlag[ols][layer] = t;    }

  uint32_t          getGeneralLayerIdx(uint32_t layerId) const { return m_generalLayerIdx[layerId]; }
  void              setGeneralLayerIdx(uint32_t layerId, uint32_t layerIdc) { m_generalLayerIdx[layerId] = layerIdc; }

  uint32_t          getNumPtls()                                   const { return m_vpsNumPtls; }
  void              setNumPtls(uint32_t val)                             { m_vpsNumPtls = val;  }

  bool              getPtPresentFlag(int idx)                      const { return m_ptPresentFlag[idx]; }
  void              setPtPresentFlag(int idx, bool val)                  { m_ptPresentFlag[idx] = val;  }

  uint32_t          getPtlMaxTemporalId(int idx)                   const { return m_ptlMaxTemporalId[idx]; }
  void              setPtlMaxTemporalId(int idx, uint32_t val)           { m_ptlMaxTemporalId[idx] = val;  }

  void              setProfileTierLevel( std::vector<ProfileTierLevel>&& val ) { m_vpsProfileTierLevel = std::move( val ); }
  const ProfileTierLevel& getProfileTierLevel(int idx)             const { return m_vpsProfileTierLevel[idx];      }

  uint32_t          getOlsPtlIdx(int idx)                          const { return m_olsPtlIdx[idx]; }
  void              setOlsPtlIdx(int idx, uint32_t val)                  { m_olsPtlIdx[idx] = val;  }
  bool              getVPSGeneralHrdParamsPresentFlag()            const { return m_vpsGeneralHrdParamsPresentFlag; }
  void              setVPSGeneralHrdParamsPresentFlag(bool t)            { m_vpsGeneralHrdParamsPresentFlag = t; }
  bool              getVPSSublayerCpbParamsPresentFlag()           const { return m_vpsSublayerCpbParamsPresentFlag; }
  void              setVPSSublayerCpbParamsPresentFlag(bool t)           { m_vpsSublayerCpbParamsPresentFlag = t; }
  uint32_t          getNumOlsHrdParamsMinus1()                     const { return m_numOlsHrdParamsMinus1; }
  void              setNumOlsHrdParamsMinus1(uint32_t val)               { m_numOlsHrdParamsMinus1 = val; m_olsHrdParams.resize( val + 1 ); }

  uint32_t          getHrdMaxTid(int olsIdx)                       const { return m_hrdMaxTid[olsIdx]; }
  void              setHrdMaxTid(int olsIdx, uint32_t val)               { m_hrdMaxTid[olsIdx] = val; }
  uint32_t          getOlsHrdIdx(int olsIdx)                       const { return m_olsHrdIdx[olsIdx]; }
  void              setOlsHrdIdx(int olsIdx, uint32_t val)               { m_olsHrdIdx[olsIdx] = val; }

  std::vector<OlsHrdParams>&       getOlsHrdParameters(int olsIdx)       { CHECK( olsIdx >= m_olsHrdParams.size(), "olsIdx out of bounds" ); return m_olsHrdParams[olsIdx]; }
  const std::vector<OlsHrdParams>& getOlsHrdParameters(int olsIdx) const { CHECK( olsIdx >= m_olsHrdParams.size(), "olsIdx out of bounds" ); return m_olsHrdParams[olsIdx]; }

  GeneralHrdParams*        getGeneralHrdParameters()                     { return &m_generalHrdParams; }
  const GeneralHrdParams*  getGeneralHrdParameters()               const { return &m_generalHrdParams; }

  int                      getTargetLayer()                        const { return m_iTargetLayer;              }
  void                     setTargetLayer(uint32_t t)                    { m_iTargetLayer = t;                 }

  int               getMaxDecPicBuffering( int temporalId ) const        { return m_dpbParameters[m_olsDpbParamsIdx[m_iTargetLayer]].m_maxDecPicBuffering[temporalId]; }
  int               getNumReorderPics( int temporalId ) const            { return m_dpbParameters[m_olsDpbParamsIdx[m_iTargetLayer]].m_numReorderPics[temporalId];     }
  int               getTotalNumOLSs() const                              { return m_totalNumOLSs; }
  int               getNumMultiLayeredOlss() const                       { return m_numMultiLayeredOlss; }
  Size              getOlsDpbPicSize( int olsIdx ) const                 { return m_olsDpbPicSize[olsIdx];          }
  void              setOlsDpbPicSize( int olsIdx, Size size )            { m_olsDpbPicSize[olsIdx] = size;          }
  void              setOlsDpbPicWidth( int olsIdx, int width )           { m_olsDpbPicSize[olsIdx].width = width;   }
  void              setOlsDpbPicHeight( int olsIdx, int height )         { m_olsDpbPicSize[olsIdx].height = height; }
  int               getOlsDpbChromaFormatIdc(int olsIdx) const           { return m_olsDpbChromaFormatIdc[olsIdx];  }
  int               getOlsDpbBitDepthMinus8(int olsIdx) const            { return m_olsDpbBitDepthMinus8[olsIdx];   }
  void              setOlsDpbChromaFormatIdc(int olsIdx, int chromaFormatIdc)  { m_olsDpbChromaFormatIdc[olsIdx] = chromaFormatIdc; }
  void              setOlsDpbBitDepthMinus8(int olsIdx, int bitDepthMinus8) { m_olsDpbBitDepthMinus8[olsIdx] = bitDepthMinus8; }

  int               getOlsDpbParamsIdx( int olsIdx ) const               { return m_olsDpbParamsIdx[olsIdx];        }
  void              setOlsDpbParamsIdx( int olsIdx, int paramIdx )       { m_olsDpbParamsIdx[olsIdx] = paramIdx;    }

  void              deriveOutputLayerSets();
  void              deriveTargetOutputLayerSet( int targetOlsIdx );
  void              checkVPS();

  void              setNumLayersInOls(int olsIdx, int numLayers)         { m_numLayersInOls[olsIdx]  = numLayers; }
  int               getNumLayersInOls(int olsIdx)      const             { return m_numLayersInOls[olsIdx]; }

  void              setLayerIdInOls  (int olsIdx, int layerIdx, int layerId)    { m_layerIdInOls[olsIdx][layerIdx] = layerId; }
  uint32_t          getLayerIdInOls  (int olsIdx, int layerIdx) const           { return m_layerIdInOls[olsIdx][layerIdx]   ; }
  const std::vector<int>& getLayerIdsInOls(int targetOlsIdx)                    { return m_layerIdInOls[targetOlsIdx];     }
};

class Window
{
private:
  bool m_enabledFlag     = false;
  int  m_winLeftOffset   = 0;
  int  m_winRightOffset  = 0;
  int  m_winTopOffset    = 0;
  int  m_winBottomOffset = 0;

public:
  bool getWindowEnabledFlag() const   { return m_enabledFlag;                          }
  int  getWindowLeftOffset() const    { return m_enabledFlag ? m_winLeftOffset : 0;    }
  void setWindowLeftOffset(int val)   { m_winLeftOffset = val; m_enabledFlag = true;   }
  int  getWindowRightOffset() const   { return m_enabledFlag ? m_winRightOffset : 0;   }
  void setWindowRightOffset(int val)  { m_winRightOffset = val; m_enabledFlag = true;  }
  int  getWindowTopOffset() const     { return m_enabledFlag ? m_winTopOffset : 0;     }
  void setWindowTopOffset(int val)    { m_winTopOffset = val; m_enabledFlag = true;    }
  int  getWindowBottomOffset() const  { return m_enabledFlag ? m_winBottomOffset: 0;   }
  void setWindowBottomOffset(int val) { m_winBottomOffset = val; m_enabledFlag = true; }

  void setWindow(int offsetLeft, int offsetLRight, int offsetLTop, int offsetLBottom)
  {
    m_enabledFlag     = true;
    m_winLeftOffset   = offsetLeft;
    m_winRightOffset  = offsetLRight;
    m_winTopOffset    = offsetLTop;
    m_winBottomOffset = offsetLBottom;
  }
};


class VUI
{
private:
  bool       m_aspectRatioInfoPresentFlag     = false;
  bool       m_aspectRatioConstantFlag        = false;
  bool       m_nonPackedFlag                  = false;
  bool       m_nonProjectedFlag               = false;
  int        m_aspectRatioIdc                 = 0;
  int        m_sarWidth                       = 0;
  int        m_sarHeight                      = 0;
  bool       m_colourDescriptionPresentFlag   = false;
  int        m_colourPrimaries                = 2;
  int        m_transferCharacteristics        = 2;
  int        m_matrixCoefficients             = 2;
  bool       m_progressiveSourceFlag          = false;
  bool       m_interlacedSourceFlag           = false;
  bool       m_chromaLocInfoPresentFlag       = false;
  int        m_chromaSampleLocTypeTopField    = 6;
  int        m_chromaSampleLocTypeBottomField = 6;
  int        m_chromaSampleLocType            = 6;
  bool       m_overscanInfoPresentFlag        = false;
  bool       m_overscanAppropriateFlag        = false;
  bool       m_videoSignalTypePresentFlag     = false;
  bool       m_videoFullRangeFlag             = false;

public:

  bool              getAspectRatioInfoPresentFlag() const                  { return m_aspectRatioInfoPresentFlag;           }
  void              setAspectRatioInfoPresentFlag(bool i)                  { m_aspectRatioInfoPresentFlag = i;              }
  bool              getAspectRatioConstantFlag() const                     { return m_aspectRatioConstantFlag;              }
  void              setAspectRatioConstantFlag(bool b)                     { m_aspectRatioConstantFlag = b;                 }
  int               getAspectRatioIdc() const                              { return m_aspectRatioIdc;                       }
  void              setAspectRatioIdc(int i)                               { m_aspectRatioIdc = i;                          }

  int               getSarWidth() const                                    { return m_sarWidth;                             }
  void              setSarWidth(int i)                                     { m_sarWidth = i;                                }

  int               getSarHeight() const                                   { return m_sarHeight;                            }
  void              setSarHeight(int i)                                    { m_sarHeight = i;                               }

  bool              getColourDescriptionPresentFlag() const                { return m_colourDescriptionPresentFlag;         }
  void              setColourDescriptionPresentFlag(bool i)                { m_colourDescriptionPresentFlag = i;            }

  int               getColourPrimaries() const                             { return m_colourPrimaries;                      }
  void              setColourPrimaries(int i)                              { m_colourPrimaries = i;                         }

  int               getTransferCharacteristics() const                     { return m_transferCharacteristics;              }
  void              setTransferCharacteristics(int i)                      { m_transferCharacteristics = i;                 }

  int               getMatrixCoefficients() const                          { return m_matrixCoefficients;                   }
  void              setMatrixCoefficients(int i)                           { m_matrixCoefficients = i;                      }

  bool              getProgressiveSourceFlag() const                       { return m_progressiveSourceFlag; }
  void              setProgressiveSourceFlag(bool b)                       { m_progressiveSourceFlag = b; }

  bool              getInterlacedSourceFlag() const                        { return m_interlacedSourceFlag; }
  void              setInterlacedSourceFlag(bool b)                        { m_interlacedSourceFlag = b; }

  bool              getNonPackedFlag() const                               { return m_nonPackedFlag; }
  void              setNonPackedFlag(bool b)                               { m_nonPackedFlag = b; }

  bool              getNonProjectedFlag() const                            { return m_nonProjectedFlag; }
  void              setNonProjectedFlag(bool b)                            { m_nonProjectedFlag = b; }

  bool              getChromaLocInfoPresentFlag() const                    { return m_chromaLocInfoPresentFlag;             }
  void              setChromaLocInfoPresentFlag(bool i)                    { m_chromaLocInfoPresentFlag = i;                }

  int               getChromaSampleLocTypeTopField() const                 { return m_chromaSampleLocTypeTopField;          }
  void              setChromaSampleLocTypeTopField(int i)                  { m_chromaSampleLocTypeTopField = i;             }

  int               getChromaSampleLocTypeBottomField() const              { return m_chromaSampleLocTypeBottomField;       }
  void              setChromaSampleLocTypeBottomField(int i)               { m_chromaSampleLocTypeBottomField = i;          }

  int               getChromaSampleLocType() const                         { return m_chromaSampleLocType;                  }
  void              setChromaSampleLocType(int i)                          { m_chromaSampleLocType = i;                     }

  bool              getOverscanInfoPresentFlag() const                     { return m_overscanInfoPresentFlag;              }
  void              setOverscanInfoPresentFlag(bool i)                     { m_overscanInfoPresentFlag = i;                 }

  bool              getOverscanAppropriateFlag() const                     { return m_overscanAppropriateFlag;              }
  void              setOverscanAppropriateFlag(bool i)                     { m_overscanAppropriateFlag = i;                 }

  bool              getVideoSignalTypePresentFlag() const                  { return m_videoSignalTypePresentFlag;           }
  void              setVideoSignalTypePresentFlag(bool i)                  { m_videoSignalTypePresentFlag = i;              }

  bool              getVideoFullRangeFlag() const                          { return m_videoFullRangeFlag;                   }
  void              setVideoFullRangeFlag(bool i)                          { m_videoFullRangeFlag = i;                      }

};

///// SPS RExt class
//class SPSRExt // Names aligned to text specification
//{
//private:
//  bool m_transformSkipRotationEnabledFlag = false;
//  bool m_transformSkipContextEnabledFlag  = false;
//  bool m_rdpcmEnabledFlag[NUMBER_OF_RDPCM_SIGNALLING_MODES];
//  bool m_extendedPrecisionProcessingFlag     = false;
//  bool m_intraSmoothingDisabledFlag          = false;
//  bool m_highPrecisionOffsetsEnabledFlag     = false;
//  bool m_persistentRiceAdaptationEnabledFlag = false;
//  bool m_cabacBypassAlignmentEnabledFlag     = false;
//
//public:
//  SPSRExt()
//  {
//    for (uint32_t signallingModeIndex = 0; signallingModeIndex < NUMBER_OF_RDPCM_SIGNALLING_MODES; signallingModeIndex++)
//    {
//      m_rdpcmEnabledFlag[signallingModeIndex] = false;
//    }
//  }
//
//  bool settingsDifferFromDefaults() const
//  {
//    return getTransformSkipRotationEnabledFlag()
//        || getTransformSkipContextEnabledFlag()
//        || getRdpcmEnabledFlag(RDPCM_SIGNAL_IMPLICIT)
//        || getRdpcmEnabledFlag(RDPCM_SIGNAL_EXPLICIT)
//        || getExtendedPrecisionProcessingFlag()
//        || getIntraSmoothingDisabledFlag()
//        || getHighPrecisionOffsetsEnabledFlag()
//        || getPersistentRiceAdaptationEnabledFlag()
//        || getCabacBypassAlignmentEnabledFlag();
//  }
//
//
//  bool getTransformSkipRotationEnabledFlag() const                                     { return m_transformSkipRotationEnabledFlag;     }
//  void setTransformSkipRotationEnabledFlag(const bool value)                           { m_transformSkipRotationEnabledFlag = value;    }
//
//  bool getTransformSkipContextEnabledFlag() const                                      { return m_transformSkipContextEnabledFlag;      }
//  void setTransformSkipContextEnabledFlag(const bool value)                            { m_transformSkipContextEnabledFlag = value;     }
//
//  bool getRdpcmEnabledFlag(const RDPCMSignallingMode signallingMode) const             { return m_rdpcmEnabledFlag[signallingMode];     }
//  void setRdpcmEnabledFlag(const RDPCMSignallingMode signallingMode, const bool value) { m_rdpcmEnabledFlag[signallingMode] = value;    }
//
//  bool getExtendedPrecisionProcessingFlag() const                                      { return m_extendedPrecisionProcessingFlag;      }
//  void setExtendedPrecisionProcessingFlag(bool value)                                  { m_extendedPrecisionProcessingFlag = value;     }
//
//  bool getIntraSmoothingDisabledFlag() const                                           { return m_intraSmoothingDisabledFlag;           }
//  void setIntraSmoothingDisabledFlag(bool bValue)                                      { m_intraSmoothingDisabledFlag=bValue;           }
//
//  bool getHighPrecisionOffsetsEnabledFlag() const                                      { return m_highPrecisionOffsetsEnabledFlag;      }
//  void setHighPrecisionOffsetsEnabledFlag(bool value)                                  { m_highPrecisionOffsetsEnabledFlag = value;     }
//
//  bool getPersistentRiceAdaptationEnabledFlag() const                                  { return m_persistentRiceAdaptationEnabledFlag;  }
//  void setPersistentRiceAdaptationEnabledFlag(const bool value)                        { m_persistentRiceAdaptationEnabledFlag = value; }
//
//  bool getCabacBypassAlignmentEnabledFlag() const                                      { return m_cabacBypassAlignmentEnabledFlag;      }
//  void setCabacBypassAlignmentEnabledFlag(const bool value)                            { m_cabacBypassAlignmentEnabledFlag = value;     }
//};
//

/// SPS class
class SPS : public BasePS<SPS>
{
private:
  int               m_SPSId                              = 0;
  int               m_decodingParameterSetId             = 0;
  int               m_VPSId                              = 0;
  int               m_layerId                            = 0;
  bool              m_affineAmvrEnabledFlag              = false;
  bool              m_fpelMmvdEnabledFlag                = false;
  bool              m_DMVR                               = false;
  bool              m_MMVD                               = false;
  bool              m_SBT                                = false;
  bool              m_ISP                                = false;
  ChromaFormat      m_chromaFormatIdc                    = CHROMA_420;
  uint32_t          m_uiMaxTLayers                       = 1;            // maximum number of temporal layers
  bool              m_BdofControlPresentInPhFlag         = false;
  bool              m_DmvrControlPresentInPhFlag         = false;
  bool              m_ProfControlPresentInPhFlag         = false;
  bool              m_ptlDpbHrdParamsPresentFlag         = false;
  bool              m_SubLayerDpbParamsFlag              = false;

  // Structure
  uint32_t          m_maxWidthInLumaSamples              = 0;
  uint32_t          m_maxHeightInLumaSamples             = 0;

  bool              m_subPicInfoPresentFlag              = false;                // indicates the presence of sub-picture info
  uint8_t           m_numSubPics                         = 1;                        //!< number of sub-pictures used
  uint32_t          m_subPicCtuTopLeftX                  [MAX_NUM_SUB_PICS] = { 0 };
  uint32_t          m_subPicCtuTopLeftY                  [MAX_NUM_SUB_PICS] = { 0 };
  uint32_t          m_SubPicWidth                        [MAX_NUM_SUB_PICS] = { 0 };
  uint32_t          m_SubPicHeight                       [MAX_NUM_SUB_PICS] = { 0 };
  bool              m_subPicTreatedAsPicFlag             [MAX_NUM_SUB_PICS] = { 0 };
  bool              m_loopFilterAcrossSubpicEnabledFlag  [MAX_NUM_SUB_PICS] = { 0 };
  bool              m_subPicIdMappingExplicitlySignalledFlag = false;
  bool              m_subPicIdMappingPresentFlag         = false;
  uint32_t          m_subPicIdLen                        = 0;                               //!< sub-picture ID length in bits
  uint16_t          m_subPicId                           [MAX_NUM_SUB_PICS] = { 0 };        //!< sub-picture ID for each sub-picture in the sequence
  int               m_log2MinCodingBlockSize             = 0;
  unsigned          m_CTUSize                            = 0;
  unsigned          m_partitionOverrideEnalbed           = 0;            // enable partition constraints override function
  PartitionConstraints m_minQT                           = PartitionConstraints{ 0, 0, 0 };    // 0: I slice luma; 1: P/B slice luma; 2: I slice chroma
  PartitionConstraints m_maxMTTHierarchyDepth            = PartitionConstraints{ 0, 0, 0 };
  PartitionConstraints m_maxBTSize                       = PartitionConstraints{ 0, 0, 0 };
  PartitionConstraints m_maxTTSize                       = PartitionConstraints{ 0, 0, 0 };
  bool              m_idrRefParamList                    = false;
  unsigned          m_dualITree                          = 0;
  uint32_t          m_uiMaxCUWidth                       = 32;
  uint32_t          m_uiMaxCUHeight                      = 32;
  bool              m_conformanceWindowPresentFlag       = false;
  Window            m_conformanceWindow;
  bool              m_independentSubPicsFlag             = true;
  bool              m_subPicSameSizeFlag                 = false;

  RPLList           m_RPLList[2];
  uint32_t          m_numRPL[2]                          = {0,0};

  bool              m_rpl1CopyFromRpl0Flag               = false;
  bool              m_rpl1IdxPresentFlag                 = false;
  bool              m_allRplEntriesHasSameSignFlag       = true;
  bool              m_bLongTermRefsPresent               = false;
  bool              m_SPSTemporalMVPEnabledFlag          = false;
  int               m_numReorderPics[MAX_TLAYER]         = { 0 };

  // Tool list
  uint32_t          m_uiQuadtreeTULog2MaxSize            = 0;
  uint32_t          m_uiQuadtreeTULog2MinSize            = 0;
  uint32_t          m_uiQuadtreeTUMaxDepthInter          = 0;
  uint32_t          m_uiQuadtreeTUMaxDepthIntra          = 0;
  bool              m_useAMP                             = false;

  bool              m_transformSkipEnabledFlag           = false;
  int               m_log2MaxTransformSkipBlockSize      = 2;
  bool              m_BDPCMEnabledFlag                   = false;
  bool              m_JointCbCrEnabledFlag               = false;
  // Parameter
  BitDepths         m_bitDepths;
  bool              m_entropyCodingSyncEnabledFlag       = false;              //!< Flag for enabling WPP
  bool              m_entryPointPresentFlag              = false;              //!< Flag for indicating the presence of entry points
  int               m_qpBDOffset                         = 0;
  int               m_internalMinusInputBitDepth         = 0;                  //  max(0, internal bitdepth - input bitdepth);                                          }
  bool              m_sbtmvpEnabledFlag                  = false;
  bool              m_disFracMmvdEnabledFlag             = false;

  uint32_t          m_uiBitsForPOC                       =  8;
  bool              m_pocMsbFlag                         = false;
  uint32_t          m_pocMsbLen                          = 0;
  uint32_t          m_numLongTermRefPicSPS               = 0;
  uint32_t          m_ltRefPicPocLsbSps     [MAX_NUM_LONG_TERM_REF_PICS] = { 0 };
  bool              m_usedByCurrPicLtSPSFlag[MAX_NUM_LONG_TERM_REF_PICS] = { 0 };
  int               m_numExtraPHBitsBytes                = 0;
  int               m_numExtraSHBitsBytes                = 0;
  std::vector<bool> m_extraPHBitPresentFlag;
  std::vector<bool> m_extraSHBitPresentFlag;
  uint32_t          m_log2MaxTbSize                      = 6;

  bool             m_useWeightPred                       = false; //!< Use of Weighting Prediction (P_SLICE)
  bool             m_useWeightedBiPred                   = false; //!< Use of Weighting Bi-Prediction (B_SLICE)
  // Max physical transform size
  bool              m_bUseSAO                            = false;

  bool              m_bTemporalIdNestingFlag             = false;      // temporal_id_nesting_flag

  bool              m_scalingListEnabledFlag             = false;
  bool              m_scalingListPresentFlag             = false;
  ScalingList       m_scalingList;
  bool              m_depQuantEnabledFlag                = false;      //!< dependent quantization enabled flag
  bool              m_signDataHidingEnabledFlag          = false;      //!< sign data hiding enabled flag
  bool              m_virtualBoundariesEnabledFlag       = false;      //!< Enable virtual boundaries tool
  bool              m_virtualBoundariesPresentFlag       = false;      //!< disable loop filtering across virtual boundaries
  unsigned          m_numVerVirtualBoundaries            = 0;          //!< number of vertical virtual boundaries
  unsigned          m_numHorVirtualBoundaries            = 0;          //!< number of horizontal virtual boundaries
  unsigned          m_virtualBoundariesPosX[3]           = { 0 };      //!< horizontal position of each vertical virtual boundary
  unsigned          m_virtualBoundariesPosY[3]           = { 0 };      //!< vertical position of each horizontal virtual boundary
  uint32_t          m_uiMaxDecPicBuffering     [MAX_TLAYER];  // init in constructor to 1
  uint32_t          m_uiMaxLatencyIncreasePlus1[MAX_TLAYER] = { 0 };

  uint32_t          m_maxNumMergeCand                    = 0;
  uint32_t          m_maxNumAffineMergeCand              = 0;
  uint32_t          m_maxNumIBCMergeCand                 = 0;
  uint32_t          m_maxNumGeoCand                      = 0;

  bool              m_generalHrdParametersPresentFlag    = false;
  GeneralHrdParams  m_generalHrdParams;
  std::vector<OlsHrdParams> m_olsHrdParams;

  bool              m_fieldSeqFlag                       = false;
  bool              m_vuiParametersPresentFlag           = false;
  unsigned          m_vuiPayloadSize                     = 0;
  VUI               m_vuiParameters;

  //SPSRExt           m_spsRangeExtension;
  static const      int m_winUnitX[NUM_CHROMA_FORMAT];
  static const      int m_winUnitY[NUM_CHROMA_FORMAT];
  ProfileTierLevel  m_profileTierLevel;

  bool              m_useALF                             = false;
  bool              m_useCCALF                           = false;

  bool              m_useWrapAround                      = false;
  bool              m_IBCFlag                            = false;
  bool              m_useColorTrans                      = false;
  bool              m_lumaReshapeEnable                  = false;
  bool              m_AMVREnabledFlag                    = false;
  bool              m_LMChroma                           = false;
  bool              m_horCollocatedChromaFlag            = true;
  bool              m_verCollocatedChromaFlag            = true;   // standard says to be inferred 1, but setting it to 0 correctly decodes some 422 sequences. Seems to be due to getCclmCollocatedChromaFlag()
  bool              m_MTS                                = false;
  bool              m_IntraMTS                           = false;
  bool              m_InterMTS                           = false;
  bool              m_LFNST                              = false;
  bool              m_SMVD                               = false;
  bool              m_Affine                             = false;
  bool              m_AffineType                         = false;
  bool              m_PROF                               = false;
  bool              m_BIO                                = false;
  bool              m_bcw                                = false;                        //
  bool              m_ciip                               = false;
  bool              m_MRL                                = false;
  bool              m_Geo                                = false;

#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
  bool              m_LadfEnabled                        = false;
  int               m_LadfNumIntervals                   = 0;
  int               m_LadfQpOffset          [MAX_LADF_INTERVALS] = { 0 };
  int               m_LadfIntervalLowerBound[MAX_LADF_INTERVALS] = { 0 };
#endif
  bool              m_MIP                                = false;
  ChromaQpMappingTable m_chromaQpMappingTable;
  bool              m_GDREnabledFlag                     = false;
  bool              m_SubLayerCbpParametersPresentFlag   = false;
  bool              m_rprEnabledFlag                     = false;
  bool              m_resChangeInClvsEnabledFlag         = false;
  bool              m_interLayerPresentFlag              = false;
  uint32_t          m_log2ParallelMergeLevelMinus2       = 0;
  bool              m_scalingMatrixAlternativeColourSpaceDisabledFlag = false;
  bool              m_scalingMatrixDesignatedColourSpaceFlag          = true;
  bool              m_disableScalingMatrixForLfnstBlks                = true;
public:

  SPS()
  {
    for ( int i = 0; i < MAX_TLAYER; i++ )
    {
      m_uiMaxDecPicBuffering[i]      = 1;
    }
  }
  ~SPS() = default;

  int                     getSPSId() const                                                                { return m_SPSId;                                                      }
  void                    setSPSId(int i)                                                                 { m_SPSId = i;                                                         }
  void                    setDecodingParameterSetId(int val)                                              { m_decodingParameterSetId = val;                                      }
  int                     getDecodingParameterSetId() const                                               { return m_decodingParameterSetId;                                     }
  int                     getVPSId() const                                                                { return m_VPSId; }
  void                    setVPSId(int i)                                                                 { m_VPSId = i; }
  void                    setLayerId( int i )                                                             { m_layerId = i;                                                       }
  int                     getLayerId() const                                                              { return m_layerId;                                                    }
  ChromaFormat            getChromaFormatIdc () const                                                     { return m_chromaFormatIdc;                                            }
  void                    setChromaFormatIdc (ChromaFormat i)                                             { m_chromaFormatIdc = i;                                               }

  static int              getWinUnitX (int chromaFormatIdc)                                               { CHECK(chromaFormatIdc < 0 || chromaFormatIdc >= NUM_CHROMA_FORMAT, "Invalid chroma format parameter"); return m_winUnitX[chromaFormatIdc]; }
  static int              getWinUnitY (int chromaFormatIdc)                                               { CHECK(chromaFormatIdc < 0 || chromaFormatIdc >= NUM_CHROMA_FORMAT, "Invalid chroma format parameter"); return m_winUnitY[chromaFormatIdc]; }

  // structure
  void                    setMaxPicWidthInLumaSamples( uint32_t u )                                       { m_maxWidthInLumaSamples = u; }
  uint32_t                getMaxPicWidthInLumaSamples() const                                             { return  m_maxWidthInLumaSamples; }
  void                    setMaxPicHeightInLumaSamples( uint32_t u )                                      { m_maxHeightInLumaSamples = u; }
  uint32_t                getMaxPicHeightInLumaSamples() const                                            { return  m_maxHeightInLumaSamples; }
  void                    setConformanceWindowPresentFlag(bool b)                                         { m_conformanceWindowPresentFlag = b;           }
  bool                    getConformanceWindowPresentFlag() const                                         { return m_conformanceWindowPresentFlag;        }
  Window&                 getConformanceWindow()                                                          { return  m_conformanceWindow;                                         }
  const Window&           getConformanceWindow() const                                                    { return  m_conformanceWindow;                                         }
  void                    setConformanceWindow(Window& conformanceWindow )                                { m_conformanceWindow = conformanceWindow;                             }

  void                    setSubPicInfoPresentFlag(bool b)                                                { m_subPicInfoPresentFlag = b;            }
  bool                    getSubPicInfoPresentFlag() const                                                { return m_subPicInfoPresentFlag;         }

  void                    setNumSubPics( uint8_t u )                                                      { m_numSubPics = u;                           }
  void                    setIndependentSubPicsFlag(bool b)                                               { m_independentSubPicsFlag = b;                    }
  bool                    getIndependentSubPicsFlag() const                                               { return m_independentSubPicsFlag;                 }
  void                    setSubPicSameSizeFlag(bool b)                                                   { m_subPicSameSizeFlag = b;                       }
  bool                    getSubPicSameSizeFlag() const                                                   { return m_subPicSameSizeFlag;                    }
  uint8_t                 getNumSubPics( ) const                                                          { return  m_numSubPics;            }
  void                    setSubPicCtuTopLeftX( int i, uint32_t u )                                       { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); m_subPicCtuTopLeftX[i] = u;                     }
  uint32_t                getSubPicCtuTopLeftX( int i ) const                                             { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); return  m_subPicCtuTopLeftX[i];                 }
  void                    setSubPicCtuTopLeftY( int i, uint32_t u )                                       { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); m_subPicCtuTopLeftY[i] = u;                     }
  uint32_t                getSubPicCtuTopLeftY( int i ) const                                             { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); return  m_subPicCtuTopLeftY[i];                 }
  void                    setSubPicWidth( int i, uint32_t u )                                             { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); m_SubPicWidth[i] = u;                           }
  uint32_t                getSubPicWidth( int i ) const                                                   { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); return  m_SubPicWidth[i];                       }
  void                    setSubPicHeight( int i, uint32_t u )                                            { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); m_SubPicHeight[i] = u;                          }
  uint32_t                getSubPicHeight( int i ) const                                                  { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); return  m_SubPicHeight[i];                      }
  void                    setSubPicTreatedAsPicFlag( int i, bool u )                                      { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); m_subPicTreatedAsPicFlag[i] = u;                }
  bool                    getSubPicTreatedAsPicFlag( int i ) const                                        { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); return  m_subPicTreatedAsPicFlag[i];            }
  void                    setLoopFilterAcrossSubpicEnabledFlag( int i, bool u )                           { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); m_loopFilterAcrossSubpicEnabledFlag[i] = u;     }
  bool                    getLoopFilterAcrossSubpicEnabledFlag( int i ) const                             { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); return  m_loopFilterAcrossSubpicEnabledFlag[i]; }
  void                    setSubPicIdMappingExplicitlySignalledFlag( bool b )                             { m_subPicIdMappingExplicitlySignalledFlag = b;    }
  bool                    getSubPicIdMappingExplicitlySignalledFlag() const                               { return m_subPicIdMappingExplicitlySignalledFlag; }
  void                    setSubPicIdMappingPresentFlag( bool b )                                         { m_subPicIdMappingPresentFlag = b;                }
  bool                    getSubPicIdMappingPresentFlag() const                                           { return  m_subPicIdMappingPresentFlag;            }
  void                    setSubPicIdLen( uint32_t u )                                                    { CHECK( u > 16, "Sub-Picture id length exeeds valid range" ); m_subPicIdLen = u;                 }
  uint32_t                getSubPicIdLen() const                                                          { return  m_subPicIdLen;                                                                          }
  void                    setSubPicId( int i, uint16_t u )                                                { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); m_subPicId[i] = u;     }
  uint16_t                getSubPicId( int i ) const                                                      { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); return  m_subPicId[i]; }

  uint32_t                getNumLongTermRefPicSPS() const                                                 { return m_numLongTermRefPicSPS;                                       }
  void                    setNumLongTermRefPicSPS(uint32_t val)                                           { m_numLongTermRefPicSPS = val;                                        }

  uint32_t                getLtRefPicPocLsbSps(uint32_t index) const                                      { CHECK( index >= MAX_NUM_LONG_TERM_REF_PICS, "Index exceeds boundary" ); return m_ltRefPicPocLsbSps[index]; }
  void                    setLtRefPicPocLsbSps(uint32_t index, uint32_t val)                              { CHECK( index >= MAX_NUM_LONG_TERM_REF_PICS, "Index exceeds boundary" ); m_ltRefPicPocLsbSps[index] = val;  }

  bool                    getUsedByCurrPicLtSPSFlag(int i) const                                          { CHECK( i >= MAX_NUM_LONG_TERM_REF_PICS, "Index exceeds boundary" ); return m_usedByCurrPicLtSPSFlag[i];    }
  void                    setUsedByCurrPicLtSPSFlag(int i, bool x)                                        { CHECK( i >= MAX_NUM_LONG_TERM_REF_PICS, "Index exceeds boundary" ); m_usedByCurrPicLtSPSFlag[i] = x;       }

  int                     getLog2MinCodingBlockSize() const                                               { return m_log2MinCodingBlockSize;                                     }
  void                    setLog2MinCodingBlockSize(int val)                                              { m_log2MinCodingBlockSize = val;                                      }
  void                    setCTUSize(unsigned    ctuSize)                                                 { m_CTUSize = ctuSize; }
  unsigned                getCTUSize()                                                              const { return  m_CTUSize; }
  void                    setSplitConsOverrideEnabledFlag(bool b)                                         { m_partitionOverrideEnalbed = b; }
  bool                    getSplitConsOverrideEnabledFlag()                                         const { return m_partitionOverrideEnalbed; }

  const PartitionConstraints& getMinQTSizes()                                                       const { return m_minQT;                                                      }
  const PartitionConstraints& getMaxMTTHierarchyDepths()                                            const { return m_maxMTTHierarchyDepth;                                       }
  const PartitionConstraints& getMaxBTSizes()                                                       const { return m_maxBTSize;                                                  }
  const PartitionConstraints& getMaxTTSizes()                                                       const { return m_maxTTSize;                                                  }

  void                    setMinQTSize           ( unsigned idx, unsigned minQT  )                        { m_minQT[idx] = minQT;                                                }
  void                    setMaxMTTHierarchyDepth( unsigned idx, unsigned maxMTT )                        { m_maxMTTHierarchyDepth[idx] = maxMTT;                                }
  void                    setMaxBTSize           ( unsigned idx, unsigned maxBT  )                        { m_maxBTSize[idx] = maxBT;                                            }
  void                    setMaxTTSize           ( unsigned idx, unsigned maxTT  )                        { m_maxTTSize[idx] = maxTT;                                            }

  void                    setMinQTSizes           ( const PartitionConstraints& minQT )                   { m_minQT = minQT;                                                     }
  void                    setMaxMTTHierarchyDepths( const PartitionConstraints& maxMTT )                  { m_maxMTTHierarchyDepth = maxMTT;                                     }
  void                    setMaxBTSizes           ( const PartitionConstraints& maxBT )                   { m_maxBTSize = maxBT;                                                 }
  void                    setMaxTTSizes           ( const PartitionConstraints& maxTT )                   { m_maxTTSize = maxTT;                                                 }

  unsigned                getMinQTSize           ( SliceType slicetype, ChannelType chType = CHANNEL_TYPE_LUMA ) const { return m_minQT               [chType == CHANNEL_TYPE_LUMA ? slicetype == I_SLICE ?  0 : 1 : 2]; }
  unsigned                getMaxMTTHierarchyDepth( SliceType slicetype, ChannelType chType = CHANNEL_TYPE_LUMA ) const { return m_maxMTTHierarchyDepth[chType == CHANNEL_TYPE_LUMA ? slicetype == I_SLICE ?  0 : 1 : 2]; }
  unsigned                getMaxBTSize           ( SliceType slicetype, ChannelType chType = CHANNEL_TYPE_LUMA ) const { return m_maxBTSize           [chType == CHANNEL_TYPE_LUMA ? slicetype == I_SLICE ?  0 : 1 : 2]; }
  unsigned                getMaxTTSize           ( SliceType slicetype, ChannelType chType = CHANNEL_TYPE_LUMA ) const { return m_maxTTSize           [chType == CHANNEL_TYPE_LUMA ? slicetype == I_SLICE ?  0 : 1 : 2]; }

  unsigned                getMaxMTTHierarchyDepth()                                                 const { return m_maxMTTHierarchyDepth[1]; }
  unsigned                getMaxMTTHierarchyDepthI()                                                const { return m_maxMTTHierarchyDepth[0]; }
  unsigned                getMaxMTTHierarchyDepthIChroma()                                          const { return m_maxMTTHierarchyDepth[2]; }

  unsigned                getMaxBTSize()                                                            const { return m_maxBTSize[1]; }
  unsigned                getMaxBTSizeI()                                                           const { return m_maxBTSize[0]; }
  unsigned                getMaxBTSizeIChroma()                                                     const { return m_maxBTSize[2]; }

  unsigned                getMaxTTSize()                                                            const { return m_maxTTSize[1]; }
  unsigned                getMaxTTSizeI()                                                           const { return m_maxTTSize[0]; }
  unsigned                getMaxTTSizeIChroma()                                                     const { return m_maxTTSize[2]; }

  void                    setIDRRefParamListPresent(bool b)                                               { m_idrRefParamList = b; }
  bool                    getIDRRefParamListPresent()                                               const { return m_idrRefParamList; }
  void                    setUseDualITree(bool b) { m_dualITree = b; }
  bool                    getUseDualITree()                                                         const { return m_dualITree; }

  void                    setMaxCUWidth( uint32_t u )                                                     { m_uiMaxCUWidth = u;                                                  }
  uint32_t                getMaxCUWidth() const                                                           { return  m_uiMaxCUWidth;                                              }
  void                    setMaxCUHeight( uint32_t u )                                                    { m_uiMaxCUHeight = u;                                                 }
  uint32_t                getMaxCUHeight() const                                                          { return  m_uiMaxCUHeight;                                             }
  bool                    getTransformSkipEnabledFlag() const                                             { return m_transformSkipEnabledFlag;                                   }
  void                    setTransformSkipEnabledFlag( bool b )                                           { m_transformSkipEnabledFlag = b;                                      }
  uint32_t                getLog2MaxTransformSkipBlockSize() const                                        { return m_log2MaxTransformSkipBlockSize;                              }
  void                    setLog2MaxTransformSkipBlockSize(uint32_t u)                                    { m_log2MaxTransformSkipBlockSize = u;                                 }
  bool                    getBDPCMEnabledFlag() const                                                     { return m_BDPCMEnabledFlag;                                           }
  void                    setBDPCMEnabledFlag( bool b )                                                   { m_BDPCMEnabledFlag = b;                                              }
  void                    setBitsForPOC( uint32_t u )                                                     { m_uiBitsForPOC = u;                                                  }
  uint32_t                getBitsForPOC() const                                                           { return m_uiBitsForPOC;                                               }
  void                    setPocMsbFlag(bool b)                                                           { m_pocMsbFlag = b;                                                    }
  bool                    getPocMsbFlag() const                                                           { return m_pocMsbFlag;                                                 }
  void                    setPocMsbLen(uint32_t u)                                                        { m_pocMsbLen = u;                                                     }
  uint32_t                getPocMsbLen() const                                                            { return m_pocMsbLen;                                                  }
  void                    setNumExtraPHBitsBytes(int i)                                                   { m_numExtraPHBitsBytes = i;                                           }
  int                     getNumExtraPHBitsBytes() const                                                  { return m_numExtraPHBitsBytes;                                        }
  void                    setNumExtraSHBitsBytes(int i)                                                   { m_numExtraSHBitsBytes = i;                                           }
  int                     getNumExtraSHBitsBytes() const                                                  { return m_numExtraSHBitsBytes;                                        }
  void                     setExtraPHBitPresentFlags( const std::vector<bool>&& b )                       { m_extraPHBitPresentFlag = std::move( b );                            }
  const std::vector<bool>& getExtraPHBitPresentFlags() const                                              { return m_extraPHBitPresentFlag;                                      }
  void                     setExtraSHBitPresentFlags( const std::vector<bool>&& b )                       { m_extraSHBitPresentFlag = std::move( b );                            }
  const std::vector<bool>& getExtraSHBitPresentFlags() const                                              { return m_extraSHBitPresentFlag;                                      }
  bool                    getUseAMP() const                                                               { return m_useAMP;                                                     }
  void                    setUseAMP( bool b )                                                             { m_useAMP = b;                                                        }
  void                    setQuadtreeTULog2MaxSize( uint32_t u )                                          { m_uiQuadtreeTULog2MaxSize = u;                                       }
  uint32_t                getQuadtreeTULog2MaxSize() const                                                { return m_uiQuadtreeTULog2MaxSize;                                    }
  void                    setQuadtreeTULog2MinSize( uint32_t u )                                          { m_uiQuadtreeTULog2MinSize = u;                                       }
  uint32_t                getQuadtreeTULog2MinSize() const                                                { return m_uiQuadtreeTULog2MinSize;                                    }
  void                    setQuadtreeTUMaxDepthInter( uint32_t u )                                        { m_uiQuadtreeTUMaxDepthInter = u;                                     }
  void                    setQuadtreeTUMaxDepthIntra( uint32_t u )                                        { m_uiQuadtreeTUMaxDepthIntra = u;                                     }
  uint32_t                getQuadtreeTUMaxDepthInter() const                                              { return m_uiQuadtreeTUMaxDepthInter;                                  }
  uint32_t                getQuadtreeTUMaxDepthIntra() const                                              { return m_uiQuadtreeTUMaxDepthIntra;                                  }
  void                    setNumReorderPics(int i, uint32_t tlayer)                                       { m_numReorderPics[tlayer] = i;                                        }
  int                     getNumReorderPics(uint32_t tlayer) const                                        { return m_numReorderPics[tlayer];                                     }
  RPLList&                createRPLList( int l, int numRPL );
  const RPLList&          getRPLList( int l ) const                                                       { return m_RPLList[l];                                                 }
  uint32_t                getNumRPL( int l ) const                                                        { return m_numRPL[l];                                                  }
  void                    setRPL1CopyFromRPL0Flag(bool isCopy)                                            { m_rpl1CopyFromRpl0Flag = isCopy;                                     }
  bool                    getRPL1CopyFromRPL0Flag() const                                                 { return m_rpl1CopyFromRpl0Flag;                                       }
  bool                    getRPL1IdxPresentFlag() const                                                   { return m_rpl1IdxPresentFlag;                                         }
  void                    setAllActiveRplEntriesHasSameSignFlag(bool isAllSame)                           { m_allRplEntriesHasSameSignFlag = isAllSame;                          }
  bool                    getAllActiveRplEntriesHasSameSignFlag() const                                   { return m_allRplEntriesHasSameSignFlag;                               }
  bool                    getLongTermRefsPresent() const                                                  { return m_bLongTermRefsPresent;                                       }
  void                    setLongTermRefsPresent(bool b)                                                  { m_bLongTermRefsPresent=b;                                            }
  bool                    getSPSTemporalMVPEnabledFlag() const                                            { return m_SPSTemporalMVPEnabledFlag;                                  }
  void                    setSPSTemporalMVPEnabledFlag(bool b)                                            { m_SPSTemporalMVPEnabledFlag=b;                                       }
  void                    setLog2MaxTbSize( uint32_t u )                                                  { m_log2MaxTbSize = u;                                                 }
  uint32_t                getLog2MaxTbSize() const                                                        { return  m_log2MaxTbSize;                                             }
  uint32_t                getMaxTbSize() const                                                            { return  1 << m_log2MaxTbSize;                                        }
  // Bit-depth
  int                     getBitDepth() const                                                             { return m_bitDepths.recon;                                            }
  void                    setBitDepth(int u )                                                             { m_bitDepths.recon = u;                                               }
  const BitDepths&        getBitDepths() const                                                            { return m_bitDepths;                                                  }

  bool                    getEntropyCodingSyncEnabledFlag() const                                         { return m_entropyCodingSyncEnabledFlag;                               }
  void                    setEntropyCodingSyncEnabledFlag(bool val)                                       { m_entropyCodingSyncEnabledFlag = val;                                }
  bool                    getEntryPointsPresentFlag() const                                               { return m_entryPointPresentFlag;                                      }
  void                    setEntryPointsPresentFlag(bool val)                                             { m_entryPointPresentFlag = val;                                       }

  static constexpr int    getMaxLog2TrDynamicRange( ChannelType )            			                        { return 15; }

  int                     getQpBDOffset() const                                                           { return m_qpBDOffset;                                                 }
  void                    setQpBDOffset(int i)                                                            { m_qpBDOffset = i;                                                    }
  int                     getInternalMinusInputBitDepth() const                                           { return m_internalMinusInputBitDepth;                                 }
  void                    setInternalMinusInputBitDepth(int i)                                            { m_internalMinusInputBitDepth = i;                                    }
  void                    setUseSAO(bool bVal)                                                            { m_bUseSAO = bVal;                                                    }
  bool                    getUseSAO() const                                                               { return m_bUseSAO;                                                    }

  void                    setJointCbCrEnabledFlag(bool bVal)                                              { m_JointCbCrEnabledFlag = bVal; }
  bool                    getJointCbCrEnabledFlag() const                                                 { return m_JointCbCrEnabledFlag; }

  bool                    getSBTMVPEnabledFlag() const                                                    { return m_sbtmvpEnabledFlag; }
  void                    setSBTMVPEnabledFlag(bool b)                                                    { m_sbtmvpEnabledFlag = b; }

  bool                    getDisFracMmvdEnabledFlag() const                                               { return m_disFracMmvdEnabledFlag; }
  void                    setDisFracMmvdEnabledFlag( bool b )                                             { m_disFracMmvdEnabledFlag = b;    }

  bool                    getFpelMmvdEnabledFlag() const                                                  { return m_fpelMmvdEnabledFlag; }
  void                    setFpelMmvdEnabledFlag( bool b )                                                { m_fpelMmvdEnabledFlag = b;    }

  bool                    getUseDMVR()const                                                               { return m_DMVR; }
  void                    setUseDMVR(bool b)                                                              { m_DMVR = b;    }

  bool                    getUseMMVD()const                                                               { return m_MMVD; }
  void                    setUseMMVD(bool b)                                                              { m_MMVD = b;    }

  bool                    getBdofControlPresentInPhFlag()const                                            { return m_BdofControlPresentInPhFlag; }
  void                    setBdofControlPresentInPhFlag(bool b)                                           { m_BdofControlPresentInPhFlag = b;    }

  bool                    getDmvrControlPresentInPhFlag()const                                            { return m_DmvrControlPresentInPhFlag; }
  void                    setDmvrControlPresentInPhFlag(bool b)                                           { m_DmvrControlPresentInPhFlag = b;    }

  bool                    getProfControlPresentInPhFlag()const                                            { return m_ProfControlPresentInPhFlag; }
  void                    setProfControlPresentInPhFlag(bool b)                                           { m_ProfControlPresentInPhFlag = b;    }

  uint32_t                getMaxTLayers() const                                                           { return m_uiMaxTLayers; }
  void                    setMaxTLayers( uint32_t uiMaxTLayers )                                          { CHECK( uiMaxTLayers > MAX_TLAYER, "Invalid number T-layers" ); m_uiMaxTLayers = uiMaxTLayers; }

  bool                    getPtlDpbHrdParamsPresentFlag()  const                                          { return m_ptlDpbHrdParamsPresentFlag;     }
  void                    setPtlDpbHrdParamsPresentFlag(bool b)                                           {        m_ptlDpbHrdParamsPresentFlag = b; }
  bool                    getSubLayerDpbParamsFlag()  const                                               { return m_SubLayerDpbParamsFlag;          }
  void                    setSubLayerDpbParamsFlag(bool b)                                                {        m_SubLayerDpbParamsFlag = b;      }
  bool                    getTemporalIdNestingFlag() const                                                { return m_bTemporalIdNestingFlag;                                     }
  void                    setTemporalIdNestingFlag( bool bValue )                                         { m_bTemporalIdNestingFlag = bValue;                                   }

  bool                    getScalingListFlag() const                                                      { return m_scalingListEnabledFlag;                                     }
  void                    setScalingListFlag( bool b )                                                    { m_scalingListEnabledFlag  = b;                                       }
  bool                    getScalingListPresentFlag() const                                               { return m_scalingListPresentFlag;                                     }
  void                    setScalingListPresentFlag( bool b )                                             { m_scalingListPresentFlag  = b;                                       }
  ScalingList&            getScalingList()                                                                { return m_scalingList; }
  const ScalingList&      getScalingList() const                                                          { return m_scalingList; }
  void                    setDepQuantEnabledFlag(bool b)                                                  { m_depQuantEnabledFlag = b; }
  bool                    getDepQuantEnabledFlag() const                                                  { return m_depQuantEnabledFlag; }
  void                    setSignDataHidingEnabledFlag(bool b)                                            { m_signDataHidingEnabledFlag = b; }
  bool                    getSignDataHidingEnabledFlag() const                                            { return m_signDataHidingEnabledFlag; }
  void                    setVirtualBoundariesEnabledFlag( bool b )                                       { m_virtualBoundariesEnabledFlag = b;                                  }
  bool                    getVirtualBoundariesEnabledFlag() const                                         { return m_virtualBoundariesEnabledFlag;                               }
  void                    setVirtualBoundariesPresentFlag( bool b )                                       { m_virtualBoundariesPresentFlag = b; }
  bool                    getVirtualBoundariesPresentFlag() const                                         { return m_virtualBoundariesPresentFlag; }
  void                    setNumVerVirtualBoundaries(unsigned u)                                          { m_numVerVirtualBoundaries = u;                                       }
  unsigned                getNumVerVirtualBoundaries() const                                              { return m_numVerVirtualBoundaries;                                    }
  void                    setNumHorVirtualBoundaries(unsigned u)                                          { m_numHorVirtualBoundaries = u;                                       }
  unsigned                getNumHorVirtualBoundaries() const                                              { return m_numHorVirtualBoundaries;                                    }
  void                    setVirtualBoundariesPosX(unsigned u, unsigned idx)                              { CHECK( idx >= 3, "vitrual boundary index exceeds valid range" ); m_virtualBoundariesPosX[idx] = u;    }
  unsigned                getVirtualBoundariesPosX(unsigned idx) const                                    { CHECK( idx >= 3, "vitrual boundary index exceeds valid range" ); return m_virtualBoundariesPosX[idx]; }
  void                    setVirtualBoundariesPosY(unsigned u, unsigned idx)                              { CHECK( idx >= 3, "vitrual boundary index exceeds valid range" ); m_virtualBoundariesPosY[idx] = u;    }
  unsigned                getVirtualBoundariesPosY(unsigned idx) const                                    { CHECK( idx >= 3, "vitrual boundary index exceeds valid range" ); return m_virtualBoundariesPosY[idx]; }

  uint32_t                getMaxDecPicBuffering(uint32_t tlayer) const                                    { return m_uiMaxDecPicBuffering[tlayer];                               }
  void                    setMaxDecPicBuffering( uint32_t ui, uint32_t tlayer )                           { CHECK(tlayer >= MAX_TLAYER, "Invalid T-layer"); m_uiMaxDecPicBuffering[tlayer] = ui;    }
  uint32_t                getMaxLatencyIncreasePlus1(uint32_t tlayer) const                               { return m_uiMaxLatencyIncreasePlus1[tlayer];                          }
  void                    setMaxLatencyIncreasePlus1( uint32_t ui , uint32_t tlayer)                      { m_uiMaxLatencyIncreasePlus1[tlayer] = ui;                            }

  uint32_t                getMaxNumMergeCand() const                                                      { return m_maxNumMergeCand; }
  void                    setMaxNumMergeCand(uint32_t u)                                                  { m_maxNumMergeCand = u; }
  uint32_t                getMaxNumAffineMergeCand() const                                                { return m_maxNumAffineMergeCand; }
  void                    setMaxNumAffineMergeCand(uint32_t u)                                            { m_maxNumAffineMergeCand = u; }
  uint32_t                getMaxNumIBCMergeCand() const                                                   { return m_maxNumIBCMergeCand; }
  void                    setMaxNumIBCMergeCand(uint32_t u)                                               { m_maxNumIBCMergeCand = u; }
  uint32_t                getMaxNumGeoCand() const                                                        { return m_maxNumGeoCand; }
  void                    setMaxNumGeoCand(uint32_t u)                                                    { m_maxNumGeoCand = u; }
  
  void                    setAffineAmvrEnabledFlag( bool val )                                            { m_affineAmvrEnabledFlag = val;                                       }
  bool                    getAffineAmvrEnabledFlag() const                                                { return m_affineAmvrEnabledFlag;                                      }
  bool                    getGeneralHrdParametersPresentFlag() const { return m_generalHrdParametersPresentFlag; }
  void                    setGeneralHrdParametersPresentFlag(bool b) { m_generalHrdParametersPresentFlag = b; }
  std::vector<OlsHrdParams>&       getOlsHrdParameters()             { return m_olsHrdParams; }
  const std::vector<OlsHrdParams>& getOlsHrdParameters() const       { return m_olsHrdParams; }

  GeneralHrdParams*       getGeneralHrdParameters() { return &m_generalHrdParams; }
  const GeneralHrdParams* getGeneralHrdParameters() const { return &m_generalHrdParams; }
  bool                    getFieldSeqFlag() const                                                         { return m_fieldSeqFlag;                         }
  void                    setFieldSeqFlag(bool i)                                                         { m_fieldSeqFlag = i;                            }
  bool                    getVuiParametersPresentFlag() const                                             { return m_vuiParametersPresentFlag;                                   }
  void                    setVuiParametersPresentFlag(bool b)                                             { m_vuiParametersPresentFlag = b;                                      }
  unsigned                getVuiPayloadSize() const                                                       { return m_vuiPayloadSize; }
  void                    setVuiPayloadSize(unsigned i)                                                   { m_vuiPayloadSize = i; }
  VUI*                    getVuiParameters()                                                              { return &m_vuiParameters;                                             }
  const VUI*              getVuiParameters() const                                                        { return &m_vuiParameters;                                             }
  const ProfileTierLevel* getProfileTierLevel() const                                                     { return &m_profileTierLevel; }
  ProfileTierLevel*       getProfileTierLevel()                                                           { return &m_profileTierLevel; }

  //const SPSRExt&          getSpsRangeExtension() const                                                    { return m_spsRangeExtension;                                          }
  //SPSRExt&                getSpsRangeExtension()                                                          { return m_spsRangeExtension;                                          }


  bool                    getUseALF() const                                                               { return m_useALF;                                                     }
  void                    setUseALF( bool b )                                                             { m_useALF = b;                                                        }
  bool                    getUseCCALF() const                                                             { return m_useCCALF; }
  void                    setUseCCALF( bool b )                                                           { m_useCCALF = b; }

  void                    setUseWrapAround(bool b)                                                        { m_useWrapAround = b;                                                 }
  bool                    getUseWrapAround() const                                                        { return m_useWrapAround;                                              }
  void                    setUseReshaper(bool b)                                                          { m_lumaReshapeEnable = b;                                                   }
  bool                    getUseReshaper() const                                                          { return m_lumaReshapeEnable;                                                }
  void                    setIBCFlag(bool IBCFlag)                                                    { m_IBCFlag = IBCFlag; }
  bool                    getIBCFlag() const                                                              { return m_IBCFlag; }
  void                    setUseColorTrans(bool value)                                                    { m_useColorTrans = value; }
  bool                    getUseColorTrans() const                                                        { return m_useColorTrans; }
  void                    setUseSBT( bool b )                                                             { m_SBT = b; }
  bool                    getUseSBT() const                                                               { return m_SBT; }
  void                    setUseISP( bool b )                                                             { m_ISP = b; }
  bool                    getUseISP() const                                                               { return m_ISP; }

  void      setAMVREnabledFlag    ( bool b )                                        { m_AMVREnabledFlag = b; }
  bool      getAMVREnabledFlag    ()                                      const     { return m_AMVREnabledFlag; }

  void      setUseAffine          ( bool b )                                        { m_Affine = b; }
  bool      getUseAffine          ()                                      const     { return m_Affine; }
  void      setUseAffineType      ( bool b )                                        { m_AffineType = b; }
  bool      getUseAffineType      ()                                      const     { return m_AffineType; }
  void      setUsePROF            ( bool b )                                        { m_PROF = b; }
  bool      getUsePROF            ()                                      const     { return m_PROF; }
  void      setUseBcw             ( bool b )                                        { m_bcw = b; }
  bool      getUseBcw             ()                                      const     { return m_bcw; }
  void      setUseCiip            ( bool b )                                        { m_ciip = b; }
  bool      getUseCiip            ()                                      const     { return m_ciip; }
  void      setUseBIO(bool b)                                                       { m_BIO = b; }
  bool      getUseBIO()                                                   const     { return m_BIO; }

  void      setUseLMChroma        ( bool b )                                        { m_LMChroma = b; }
  bool      getUseLMChroma        ()                                      const     { return m_LMChroma; }
  void      setHorCollocatedChromaFlag( bool b )                                    { m_horCollocatedChromaFlag = b;    }
  bool      getHorCollocatedChromaFlag()                                  const     { return m_horCollocatedChromaFlag; }
  void      setVerCollocatedChromaFlag( bool b )                                    { m_verCollocatedChromaFlag = b;    }
  bool      getVerCollocatedChromaFlag()                                  const     { return m_verCollocatedChromaFlag; }
  bool      getCclmCollocatedChromaFlag()                                 const     { return m_verCollocatedChromaFlag; }
  void      setUseMTS             ( bool b )                                        { m_MTS = b; }
  bool      getUseMTS             ()                                      const     { return m_MTS; }
  bool      getUseImplicitMTS     ()                                      const     { return m_MTS && !m_IntraMTS; }
  void      setUseIntraMTS        ( bool b )                                        { m_IntraMTS = b; }
  bool      getUseIntraMTS        ()                                      const     { return m_IntraMTS; }
  void      setUseInterMTS        ( bool b )                                        { m_InterMTS = b; }
  bool      getUseInterMTS        ()                                      const     { return m_InterMTS; }
  void      setUseLFNST           ( bool b )                                        { m_LFNST = b; }
  bool      getUseLFNST           ()                                      const     { return m_LFNST; }
  void      setUseSMVD(bool b)                                                      { m_SMVD = b; }
  bool      getUseSMVD()                                                  const     { return m_SMVD; }
#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
  void      setLadfEnabled        ( bool b )                                        { m_LadfEnabled = b; }
  bool      getLadfEnabled        ()                                      const     { return m_LadfEnabled; }
  void      setLadfNumIntervals   ( int i )                                         { m_LadfNumIntervals = i; }
  int       getLadfNumIntervals   ()                                      const     { return m_LadfNumIntervals; }
  void      setLadfQpOffset       ( int value, int idx )                            { m_LadfQpOffset[ idx ] = value; }
  int       getLadfQpOffset       ( int idx )                             const     { return m_LadfQpOffset[ idx ]; }
  void      setLadfIntervalLowerBound( int value, int idx )                         { m_LadfIntervalLowerBound[ idx ] = value; }
  int       getLadfIntervalLowerBound( int idx )                          const     { return m_LadfIntervalLowerBound[ idx ]; }
#endif
  void      setUseMRL             ( bool b )                                        { m_MRL = b; }
  bool      getUseMRL             ()                                      const     { return m_MRL; }
  void      setUseGeo             ( bool b )                                        { m_Geo = b; }
  bool      getUseGeo             ()                                      const     { return m_Geo; }
  void      setUseMIP             ( bool b )                                        { m_MIP = b; }
  bool      getUseMIP             ()                                      const     { return m_MIP; }

  bool      getUseWP              ()                                      const     { return m_useWeightPred; }
  bool      getUseWPBiPred        ()                                      const     { return m_useWeightedBiPred; }
  void      setUseWP              ( bool b )                                        { m_useWeightPred = b; }
  void      setUseWPBiPred        ( bool b )                                        { m_useWeightedBiPred = b; }
  void      setChromaQpMappingTableFromParams( const ChromaQpMappingTableParams& params )  { m_chromaQpMappingTable = ChromaQpMappingTable( params ); }
  void      setChromaQpMappingTableFromParams( const ChromaQpMappingTableParams&& params ) { m_chromaQpMappingTable = ChromaQpMappingTable( std::move( params ) ); }
  void      deriveChromaQPMappingTables()                                           { m_chromaQpMappingTable.deriveChromaQPMappingTables(); }
  const ChromaQpMappingTable& getChromaQpMappingTable()                   const     { return m_chromaQpMappingTable;}
  int       getMappedChromaQpValue(ComponentID compID, int qpVal)         const     { return m_chromaQpMappingTable.getMappedChromaQpValue(compID, qpVal); }
  void      setGDREnabledFlag     ( bool b )                                        { m_GDREnabledFlag = b;    }
  bool      getGDREnabledFlag()                                           const     { return m_GDREnabledFlag; }
  void      setSubLayerParametersPresentFlag(bool flag)                             { m_SubLayerCbpParametersPresentFlag = flag; }
  bool      getSubLayerParametersPresentFlag()                            const     { return m_SubLayerCbpParametersPresentFlag;  }
  bool      getRprEnabledFlag()                                           const     { return m_rprEnabledFlag; }
  void      setRprEnabledFlag( bool flag )                                          { m_rprEnabledFlag = flag; }
  bool      getInterLayerPresentFlag()                                    const     { return m_interLayerPresentFlag; }
  void      setInterLayerPresentFlag( bool b )                                      { m_interLayerPresentFlag = b; }
  bool      getResChangeInClvsEnabledFlag()                               const     { return m_resChangeInClvsEnabledFlag; }
  void      setResChangeInClvsEnabledFlag(bool flag)                                { m_resChangeInClvsEnabledFlag = flag; }

  uint32_t   getLog2ParallelMergeLevelMinus2()                            const     { return m_log2ParallelMergeLevelMinus2; }
  void       setLog2ParallelMergeLevelMinus2(uint32_t mrgLevel)                     { m_log2ParallelMergeLevelMinus2 = mrgLevel; }
  bool       getScalingMatrixForAlternativeColourSpaceDisabledFlag()      const     { return m_scalingMatrixAlternativeColourSpaceDisabledFlag; }
  void       setScalingMatrixForAlternativeColourSpaceDisabledFlag(bool b)          { m_scalingMatrixAlternativeColourSpaceDisabledFlag = b; }
  bool       getScalingMatrixDesignatedColourSpaceFlag()                  const     { return m_scalingMatrixDesignatedColourSpaceFlag; }
  void       setScalingMatrixDesignatedColourSpaceFlag(bool b)                      { m_scalingMatrixDesignatedColourSpaceFlag = b; }
  bool       getDisableScalingMatrixForLfnstBlks()                        const     { return m_disableScalingMatrixForLfnstBlks; }
  void       setDisableScalingMatrixForLfnstBlks(bool flag)                         { m_disableScalingMatrixForLfnstBlks = flag; }
};


/// Reference Picture Lists class

struct CodingUnit;

/// PPS class
class PPS : public BasePS<PPS>
{
private:
  int              m_PPSId                             = 0;       // pic_parameter_set_id
  int              m_SPSId                             = 0;       // seq_parameter_set_id
  int              m_layerId                           = 0;
  int              m_picInitQPMinus26                  = 0;
  bool             m_useDQP                            = false;
  bool             m_usePPSChromaTool                  = false;
  bool             m_bConstrainedIntraPred             = false;   // constrained_intra_pred_flag
  bool             m_bSliceChromaQpFlag                = false;   // slicelevel_chroma_qp_flag

  // access channel

  ChromaQpOffset   m_chromaQpOffset;
  bool             m_chromaJointCbCrQpOffsetPresentFlag= false;

  // Chroma QP Adjustments
  int              m_chromaQpOffsetListLen             = 0; // size (excludes the null entry used in the following array).
  ChromaQpOffset   m_ChromaQpAdjTableIncludingNullEntry[1+MAX_QP_OFFSET_LIST_SIZE]; //!< Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0, and entries [cu_chroma_qp_offset_idx+1...] otherwise

  uint32_t         m_numRefIdxL0DefaultActive          = 1;
  uint32_t         m_numRefIdxL1DefaultActive          = 1;

  bool             m_rpl1IdxPresentFlag                = false;

  bool             m_bUseWeightPred                    = false;   //!< Use of Weighting Prediction (P_SLICE)
  bool             m_useWeightedBiPred                 = false;   //!< Use of Weighting Bi-Prediction (B_SLICE)
  bool             m_OutputFlagPresentFlag             = false;   //!< Indicates the presence of output_flag in slice header
  uint8_t          m_numSubPics                        = 1;       //!< number of sub-pictures used - must match SPS
  bool             m_subPicIdMappingPresentFlag        = false;
  uint32_t         m_subPicIdLen                       = 0;       //!< sub-picture ID length in bits
  uint16_t         m_subPicId[MAX_NUM_SUB_PICS]        = { 0 };   //!< sub-picture ID for each sub-picture in the sequence
  bool             m_noPicPartitionFlag                = false;   //!< no picture partitioning flag - single slice, single tile
  uint8_t          m_log2CtuSize                       = 0;       //!< log2 of the CTU size - required to match corresponding value in SPS
  uint8_t          m_ctuSize                           = 0;       //!< CTU size
  uint32_t         m_picWidthInCtu                     = 0;       //!< picture width in units of CTUs
  uint32_t         m_picHeightInCtu                    = 0;       //!< picture height in units of CTUs
  uint32_t         m_numExpTileCols                    = 0;       //!< number of explicitly specified tile columns
  uint32_t         m_numExpTileRows                    = 0;       //!< number of explicitly specified tile rows
  uint32_t         m_numTileCols                       = 0;       //!< number of tile columns
  uint32_t         m_numTileRows                       = 0;       //!< number of tile rows
  std::vector<int> m_tileColumnWidth;
  std::vector<int> m_tileRowHeight;

  bool                   m_rectSliceFlag            = true;
  bool                   m_singleSlicePerSubPicFlag = false;   //!< single slice per sub-picture flag
  uint32_t               m_numSlicesInPic           = 1;       //!< number of rectangular slices in the picture (raster-scan slice specified at slice level)
  bool                   m_tileIdxDeltaPresentFlag  = false;   //!< tile index delta present flag
  std::vector<uint32_t>  m_tileColBd;                          //!< tile column left-boundaries in units of CTUs
  std::vector<uint32_t>  m_tileRowBd;                          //!< tile row top-boundaries in units of CTUs
  std::vector<uint32_t>  m_ctuToTileCol;                       //!< mapping between CTU horizontal address and tile column index
  std::vector<uint32_t>  m_ctuToTileRow;                       //!< mapping between CTU vertical address and tile row index
  std::vector<RectSlice> m_rectSlices;                         //!< list of rectangular slice signalling parameters
  std::vector<SliceMap>  m_sliceMap;                           //!< list of CTU maps for each slice in the picture
  std::vector<SubPic>    m_subPics;                            //!< list of subpictures in the picture

  bool             m_cabacInitPresentFlag                = false;

  bool             m_pictureHeaderExtensionPresentFlag   = false;   //< picture header extension flags present in picture headers or not
  bool             m_sliceHeaderExtensionPresentFlag     = false;
  bool             m_loopFilterAcrossTilesEnabledFlag    = false;   //!< loop filtering applied across tiles flag
  bool             m_loopFilterAcrossSlicesEnabledFlag   = false;
  bool             m_deblockingFilterControlPresentFlag  = false;
  bool             m_deblockingFilterOverrideEnabledFlag = false;
  bool             m_ppsDeblockingFilterDisabledFlag     = false;
  int              m_deblockingFilterBetaOffsetDiv2      = 0;       //< beta offset for deblocking filter
  int              m_deblockingFilterTcOffsetDiv2        = 0;       //< tc offset for deblocking filter
  int              m_deblockingFilterCbBetaOffsetDiv2    = 0;       //< beta offset for Cb deblocking filter
  int              m_deblockingFilterCbTcOffsetDiv2      = 0;       //< tc offset for Cb deblocking filter
  int              m_deblockingFilterCrBetaOffsetDiv2    = 0;       //< beta offset for Cr deblocking filter
  int              m_deblockingFilterCrTcOffsetDiv2      = 0;       //< tc offset for Cr deblocking filter
  bool             m_listsModificationPresentFlag        = false;
  int              m_numExtraSliceHeaderBits             = 0;

  bool             m_rplInfoInPhFlag                     = false;
  bool             m_dbfInfoInPhFlag                     = false;
  bool             m_saoInfoInPhFlag                     = false;
  bool             m_alfInfoInPhFlag                     = false;
  bool             m_wpInfoInPhFlag                      = false;
  bool             m_qpDeltaInfoInPhFlag                 = false;
  bool             m_mixedNaluTypesInPicFlag             = false;

  bool             m_scalingListPresentFlag              = false;
  ScalingList      m_scalingList;                       //!< ScalingList class
  uint32_t         m_picWidthInLumaSamples               = 0;
  uint32_t         m_picHeightInLumaSamples              = 0;
  bool             m_conformanceWindowPresentFlag        = false;
  Window           m_conformanceWindow;
  Window           m_scalingWindow;

  bool             m_useWrapAround                       = false;   //< reference wrap around enabled or not
  unsigned         m_picWidthMinusWrapAroundOffset       = 0;       //< pic_width_in_minCbSizeY - wraparound_offset_in_minCbSizeY
  unsigned         m_wrapAroundOffset                    = 0;       //< reference wrap around offset in luma samples

public:
  std::unique_ptr<PreCalcValues> pcv;

  PPS()  = default;
  ~PPS() = default;

  int                    getPPSId() const                                                 { return m_PPSId;                               }
  void                   setPPSId(int i)                                                  { m_PPSId = i;                                  }
  int                    getSPSId() const                                                 { return m_SPSId;                               }
  void                   setSPSId(int i)                                                  { m_SPSId = i;                                  }
  void                   setLayerId( int i )                                              { m_layerId = i;                                }
  int                    getLayerId()                                               const { return m_layerId;                             }
  int                    getPicInitQPMinus26() const                                      { return  m_picInitQPMinus26;                   }
  void                   setPicInitQPMinus26( int i )                                     { m_picInitQPMinus26 = i;                       }
  bool                   getUseDQP() const                                                { return m_useDQP;                              }
  void                   setUseDQP( bool b )                                              { m_useDQP   = b;                               }
  bool                   getPPSChromaToolFlag()                                     const { return  m_usePPSChromaTool;                   }
  void                   setPPSChromaToolFlag(bool b)                                     { m_usePPSChromaTool = b;                       }
  bool                   getConstrainedIntraPred() const                                  { return  m_bConstrainedIntraPred;              }
  void                   setConstrainedIntraPred( bool b )                                { m_bConstrainedIntraPred = b;                  }
  bool                   getSliceChromaQpFlag() const                                     { return  m_bSliceChromaQpFlag;                 }
  void                   setSliceChromaQpFlag( bool b )                                   { m_bSliceChromaQpFlag = b;                     }


  bool                   getJointCbCrQpOffsetPresentFlag() const                          { return m_chromaJointCbCrQpOffsetPresentFlag;   }
  void                   setJointCbCrQpOffsetPresentFlag(bool b)                          { m_chromaJointCbCrQpOffsetPresentFlag = b;      }

  void                   setQpOffset( ComponentID compID, int val )                       { m_chromaQpOffset.set( compID, val ); }
  int                    getQpOffset( ComponentID compID ) const                          { return m_chromaQpOffset.get( compID ); }

  bool                   getCuChromaQpOffsetEnabledFlag() const                           { return getChromaQpOffsetListLen()>0;            }
  int                    getChromaQpOffsetListLen() const                                 { return m_chromaQpOffsetListLen;                 }
  void                   clearChromaQpOffsetList()                                        { m_chromaQpOffsetListLen = 0;                    }

  const ChromaQpOffset&  getChromaQpOffsetListEntry( int cuChromaQpOffsetIdxPlus1 ) const
  {
    CHECK(cuChromaQpOffsetIdxPlus1 >= m_chromaQpOffsetListLen+1, "Invalid chroma QP offset");
    return m_ChromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1]; // Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0, and entries [cu_chroma_qp_offset_idx+1...] otherwise
  }

  void                   setChromaQpOffsetListEntry( int cuChromaQpOffsetIdxPlus1, int cbOffset, int crOffset, int jointCbCrOffset )
  {
    CHECK(cuChromaQpOffsetIdxPlus1 == 0 || cuChromaQpOffsetIdxPlus1 > MAX_QP_OFFSET_LIST_SIZE, "Invalid chroma QP offset");
    // Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0, and entries [cu_chroma_qp_offset_idx+1...] otherwise
    m_ChromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1].CbOffset        = cbOffset;
    m_ChromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1].CrOffset        = crOffset;
    m_ChromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1].JointCbCrOffset = jointCbCrOffset;
    m_chromaQpOffsetListLen                                                        = std::max( m_chromaQpOffsetListLen, cuChromaQpOffsetIdxPlus1 );
  }
  
  void                   setNumRefIdxL0DefaultActive(uint32_t ui)                         { m_numRefIdxL0DefaultActive=ui;                }
  uint32_t               getNumRefIdxL0DefaultActive() const                              { return m_numRefIdxL0DefaultActive;            }
  void                   setNumRefIdxL1DefaultActive(uint32_t ui)                         { m_numRefIdxL1DefaultActive=ui;                }
  uint32_t               getNumRefIdxL1DefaultActive() const                              { return m_numRefIdxL1DefaultActive;            }

  void                   setRpl1IdxPresentFlag(bool isPresent)                            { m_rpl1IdxPresentFlag = isPresent;             }
  uint32_t               getRpl1IdxPresentFlag() const                                    { return m_rpl1IdxPresentFlag;                  }

  bool                   getUseWP() const                                                 { return m_bUseWeightPred;                      }
  bool                   getWPBiPred() const                                              { return m_useWeightedBiPred;                   }
  void                   setUseWP( bool b )                                               { m_bUseWeightPred = b;                         }
  void                   setWPBiPred( bool b )                                            { m_useWeightedBiPred = b;                      }

  void                   setUseWrapAround(bool b)                                         { m_useWrapAround = b;                          }
  bool                   getUseWrapAround() const                                         { return m_useWrapAround;                       }
  void                   setPicWidthMinusWrapAroundOffset(unsigned offset)                { m_picWidthMinusWrapAroundOffset = offset;     }
  unsigned               getPicWidthMinusWrapAroundOffset() const                         { return m_picWidthMinusWrapAroundOffset;       }
  void                   setWrapAroundOffset(unsigned offset)                             { m_wrapAroundOffset = offset;                  }
  unsigned               getWrapAroundOffset() const                                      { return m_wrapAroundOffset;                    }
  void                   setOutputFlagPresentFlag( bool b )                               { m_OutputFlagPresentFlag = b;                  }
  bool                   getOutputFlagPresentFlag() const                                 { return m_OutputFlagPresentFlag;               }
  void                   setNumSubPics( uint8_t u )                                       { m_numSubPics = u;                             }
  uint8_t                getNumSubPics( ) const                                           { return  m_numSubPics;                         }
  void                   setSubPicIdMappingPresentFlag( bool b )                          { m_subPicIdMappingPresentFlag = b;             }
  bool                   getSubPicIdMappingPresentFlag() const                            { return m_subPicIdMappingPresentFlag;          }
  void                   setSubPicIdLen( uint32_t u )                                     { CHECK( u > 16, "Sub-picture id len exceeds valid range" ); m_subPicIdLen = u;                   }
  uint32_t               getSubPicIdLen() const                                           { return  m_subPicIdLen;                                                                          }
  void                   setSubPicId( int i, uint16_t u )                                 { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); m_subPicId[i] = u;     }
  uint16_t               getSubPicId( int i ) const                                       { CHECK( i >= MAX_NUM_SUB_PICS, "Sub-picture index exceeds valid range" ); return  m_subPicId[i]; }
  uint32_t               getSubPicIdxFromSubPicId( uint32_t subPicId ) const;
  void                   setNoPicPartitionFlag( bool b )                                  { m_noPicPartitionFlag = b;                     }
  bool                   getNoPicPartitionFlag( ) const                                   { return  m_noPicPartitionFlag;                 }
  void                   setLog2CtuSize( uint8_t u )                                      { m_log2CtuSize = u; m_ctuSize = 1 << m_log2CtuSize;
                                                                                            m_picWidthInCtu = (m_picWidthInLumaSamples  + m_ctuSize - 1) / m_ctuSize;
                                                                                            m_picHeightInCtu = (m_picHeightInLumaSamples  + m_ctuSize - 1) / m_ctuSize; }
  uint8_t                getLog2CtuSize( ) const                                          { return  m_log2CtuSize;                        }
  uint8_t                getCtuSize( ) const                                              { return  m_ctuSize;                            }
  uint8_t                getPicWidthInCtu( ) const                                        { return  m_picWidthInCtu;                      }
  uint8_t                getPicHeightInCtu( ) const                                       { return  m_picHeightInCtu;                     }
  void                   setNumExpTileColumns( uint32_t u )                               { m_numExpTileCols = u;                         }
  uint32_t               getNumExpTileColumns( ) const                                    { return  m_numExpTileCols;                     }
  void                   setNumExpTileRows( uint32_t u )                                  { m_numExpTileRows = u;                         }
  uint32_t               getNumExpTileRows( ) const                                       { return  m_numExpTileRows;                     }
  void                   setNumTileColumns( uint32_t u )                                  { m_numTileCols = u;                            }
  uint32_t               getNumTileColumns( ) const                                       { return  m_numTileCols;                        }
  void                   setNumTileRows( uint32_t u )                                     { m_numTileRows = u;                            }
  uint32_t               getNumTileRows( ) const                                          { return  m_numTileRows;                        }
  void                   addTileColumnWidth( uint32_t u )                                 { CHECK( m_tileColumnWidth.size()  >= MAX_TILE_COLS, "Number of tile columns exceeds valid range" ); m_tileColumnWidth.push_back(u);    }
  void                   addTileRowHeight( uint32_t u )                                   { m_tileRowHeight.push_back(u);   }

  void                   setTileColumnWidth(const std::vector<int>& columnWidth )         { m_tileColumnWidth = columnWidth;              }
  uint32_t               getTileColumnWidth(uint32_t columnIdx) const                     { return  m_tileColumnWidth[columnIdx];         }
  void                   setTileRowHeight(const std::vector<int>& rowHeight)              { m_tileRowHeight = rowHeight;                  }
  uint32_t               getTileRowHeight(uint32_t rowIdx) const                          { return m_tileRowHeight[rowIdx];               }
  uint32_t               getNumTiles() const                                              { return m_numTileCols * m_numTileRows;        }
  uint32_t               ctuToTileCol( int ctuX ) const                                   { CHECK( ctuX >= m_ctuToTileCol.size(), "CTU address index exceeds valid range" ); return  m_ctuToTileCol[ctuX];                  }
  uint32_t               ctuToTileRow( int ctuY ) const                                   { CHECK( ctuY >= m_ctuToTileRow.size(), "CTU address index exceeds valid range" ); return  m_ctuToTileRow[ctuY];                  }
  uint32_t               ctuToTileColBd( int ctuX ) const                                 { return  getTileColumnBd(ctuToTileCol( ctuX ));                                                                                  }
  uint32_t               ctuToTileRowBd( int ctuY ) const                                 { return  getTileRowBd(ctuToTileRow( ctuY ));                                                                                     }
  bool                   ctuIsTileColBd( int ctuX ) const                                 { return  ctuX == ctuToTileColBd( ctuX );                                                                                         }
  bool                   ctuIsTileRowBd( int ctuY ) const                                 { return  ctuY == ctuToTileRowBd( ctuY );                                                                                         }
  uint32_t               getTileIdx( uint32_t ctuX, uint32_t ctuY ) const                 { return (ctuToTileRow( ctuY ) * getNumTileColumns()) + ctuToTileCol( ctuX );                                                     }
  uint32_t               getTileIdx( uint32_t ctuRsAddr) const                            { return getTileIdx( ctuRsAddr % m_picWidthInCtu,  ctuRsAddr / m_picWidthInCtu );                                                 }
  uint32_t               getTileIdx( const Position& pos ) const                          { return getTileIdx( pos.x / m_ctuSize, pos.y / m_ctuSize );                                                                      }
  bool                   getRectSliceFlag() const                                         { return m_rectSliceFlag;                       }
  void                   setRectSliceFlag(bool val)                                       { m_rectSliceFlag = val;                        }
  void                   setSingleSlicePerSubPicFlag( bool b )                            { m_singleSlicePerSubPicFlag = b;                                                                                                 }
  bool                   getSingleSlicePerSubPicFlag( ) const                             { return  m_singleSlicePerSubPicFlag;                                                                                             }
  void                   setNumSlicesInPic( uint32_t u )                                  { CHECK( u > MAX_SLICES, "Number of slices in picture exceeds valid range" ); m_numSlicesInPic = u;                               }
  uint32_t               getNumSlicesInPic( ) const                                       { return  m_numSlicesInPic;                                                                                                       }
  void                   setTileIdxDeltaPresentFlag( bool b )                             { m_tileIdxDeltaPresentFlag = b;                                                                                                  }
  bool                   getTileIdxDeltaPresentFlag( ) const                              { return  m_tileIdxDeltaPresentFlag;                                                                                              }
  uint32_t               getTileColumnBd( int idx ) const                                 { CHECK( idx >= m_tileColBd.size(), "Tile column index exceeds valid range" );                    return  m_tileColBd[idx];       }
  uint32_t               getTileRowBd( int idx ) const                                    { CHECK( idx >= m_tileRowBd.size(), "Tile row index exceeds valid range" );                       return  m_tileRowBd[idx];       }
  void                   setSliceWidthInTiles( int idx, uint32_t u )                      { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    m_rectSlices[idx].setSliceWidthInTiles( u );            }
  uint32_t               getSliceWidthInTiles( int idx ) const                            { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    return  m_rectSlices[idx].getSliceWidthInTiles( );      }
  void                   setSliceHeightInTiles( int idx, uint32_t u )                     { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    m_rectSlices[idx].setSliceHeightInTiles( u );           }
  uint32_t               getSliceHeightInTiles( int idx ) const                           { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    return  m_rectSlices[idx].getSliceHeightInTiles( );     }
  void                   setNumSlicesInTile( int idx, uint32_t u )                        { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    m_rectSlices[idx].setNumSlicesInTile( u );              }
  uint32_t               getNumSlicesInTile( int idx ) const                              { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    return  m_rectSlices[idx].getNumSlicesInTile( );        }
  void                   setSliceHeightInCtu( int idx, uint32_t u )                       { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    m_rectSlices[idx].setSliceHeightInCtu( u );             }
  uint32_t               getSliceHeightInCtu( int idx ) const                             { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    return  m_rectSlices[idx].getSliceHeightInCtu( );       }
  void                   setSliceTileIdx(  int idx, uint32_t u )                          { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    m_rectSlices[idx].setTileIdx( u );                      }
  uint32_t               getSliceTileIdx( int idx ) const                                 { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    return  m_rectSlices[idx].getTileIdx( );                }
  void                   resetTileSliceInfo();
  void                   initTiles();
  void                   initRectSlices();
  void                   initRectSliceMap(const SPS *sps);
  const SubPic&          getSubPic(uint32_t idx) const                                    { return m_subPics[idx]; }
  void                   initSubPic(const SPS &sps);
  const SubPic&          getSubPicFromPos(const Position& pos)  const;
  const SubPic&          getSubPicFromCU (const CodingUnit& cu) const;
  void                   checkSliceMap();

  SliceMap               getSliceMap( int idx ) const                                     { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    return m_sliceMap[idx];                             }

  void                   setCabacInitPresentFlag( bool flag )                             { m_cabacInitPresentFlag = flag;                }
  bool                   getCabacInitPresentFlag() const                                  { return m_cabacInitPresentFlag;                }
  void                   setDeblockingFilterControlPresentFlag( bool val )                { m_deblockingFilterControlPresentFlag = val;   }
  bool                   getDeblockingFilterControlPresentFlag() const                    { return m_deblockingFilterControlPresentFlag;  }
  void                   setDeblockingFilterOverrideEnabledFlag( bool val )               { m_deblockingFilterOverrideEnabledFlag = val;  }
  bool                   getDeblockingFilterOverrideEnabledFlag() const                   { return m_deblockingFilterOverrideEnabledFlag; }
  void                   setPPSDeblockingFilterDisabledFlag(bool val)                     { m_ppsDeblockingFilterDisabledFlag = val;      } //!< set offset for deblocking filter disabled
  bool                   getPPSDeblockingFilterDisabledFlag() const                       { return m_ppsDeblockingFilterDisabledFlag;     } //!< get offset for deblocking filter disabled
  void                   setDeblockingFilterBetaOffsetDiv2(int val)                       { m_deblockingFilterBetaOffsetDiv2 = val;       } //!< set beta offset for deblocking filter
  int                    getDeblockingFilterBetaOffsetDiv2() const                        { return m_deblockingFilterBetaOffsetDiv2;      } //!< get beta offset for deblocking filter
  void                   setDeblockingFilterTcOffsetDiv2(int val)                         { m_deblockingFilterTcOffsetDiv2 = val;         } //!< set tc offset for deblocking filter
  int                    getDeblockingFilterTcOffsetDiv2() const                          { return m_deblockingFilterTcOffsetDiv2;        } //!< get tc offset for deblocking filter
  void                   setDeblockingFilterCbBetaOffsetDiv2(int val)                     { m_deblockingFilterCbBetaOffsetDiv2 = val;     } //!< set beta offset for Cb deblocking filter
  int                    getDeblockingFilterCbBetaOffsetDiv2() const                      { return m_deblockingFilterCbBetaOffsetDiv2;    } //!< get beta offset for Cb deblocking filter
  void                   setDeblockingFilterCbTcOffsetDiv2(int val)                       { m_deblockingFilterCbTcOffsetDiv2 = val;       } //!< set tc offset for Cb deblocking filter
  int                    getDeblockingFilterCbTcOffsetDiv2() const                        { return m_deblockingFilterCbTcOffsetDiv2;      } //!< get tc offset for Cb deblocking filter
  void                   setDeblockingFilterCrBetaOffsetDiv2(int val)                     { m_deblockingFilterCrBetaOffsetDiv2 = val;     } //!< set beta offset for Cr deblocking filter
  int                    getDeblockingFilterCrBetaOffsetDiv2() const                      { return m_deblockingFilterCrBetaOffsetDiv2;    } //!< get beta offset for Cr deblocking filter
  void                   setDeblockingFilterCrTcOffsetDiv2(int val)                       { m_deblockingFilterCrTcOffsetDiv2 = val;       } //!< set tc offset for Cr deblocking filter
  int                    getDeblockingFilterCrTcOffsetDiv2() const                        { return m_deblockingFilterCrTcOffsetDiv2;      } //!< get tc offset for Cr deblocking filter
  bool                   getListsModificationPresentFlag() const                          { return m_listsModificationPresentFlag;        }
  void                   setListsModificationPresentFlag( bool b )                        { m_listsModificationPresentFlag = b;           }
  int                    getNumExtraSliceHeaderBits() const                               { return m_numExtraSliceHeaderBits;             }
  void                   setNumExtraSliceHeaderBits(int i)                                { m_numExtraSliceHeaderBits = i;                }
  void                   setLoopFilterAcrossTilesEnabledFlag( bool b )                    { m_loopFilterAcrossTilesEnabledFlag = b;                                                                                         }
  bool                   getLoopFilterAcrossTilesEnabledFlag( ) const                     { return  m_loopFilterAcrossTilesEnabledFlag;                                                                                     }
  void                   setLoopFilterAcrossSlicesEnabledFlag( bool bValue )              { m_loopFilterAcrossSlicesEnabledFlag = bValue; }
  bool                   getLoopFilterAcrossSlicesEnabledFlag() const                     { return m_loopFilterAcrossSlicesEnabledFlag;   }
  bool                   getPictureHeaderExtensionPresentFlag() const                     { return m_pictureHeaderExtensionPresentFlag;     }
  void                   setPictureHeaderExtensionPresentFlag(bool val)                   { m_pictureHeaderExtensionPresentFlag = val;      }
  bool                   getSliceHeaderExtensionPresentFlag() const                       { return m_sliceHeaderExtensionPresentFlag;     }
  void                   setSliceHeaderExtensionPresentFlag(bool val)                     { m_sliceHeaderExtensionPresentFlag = val;      }

  void                   setRplInfoInPhFlag(bool flag)                                    { m_rplInfoInPhFlag = flag;                     }
  bool                   getRplInfoInPhFlag() const                                       { return m_rplInfoInPhFlag;                     }
  void                   setDbfInfoInPhFlag(bool flag)                                    { m_dbfInfoInPhFlag = flag;                     }
  bool                   getDbfInfoInPhFlag() const                                       { return m_dbfInfoInPhFlag;                     }
  void                   setSaoInfoInPhFlag(bool flag)                                    { m_saoInfoInPhFlag = flag;                     }
  bool                   getSaoInfoInPhFlag() const                                       { return m_saoInfoInPhFlag;                     }
  void                   setAlfInfoInPhFlag(bool flag)                                    { m_alfInfoInPhFlag = flag;                     }
  bool                   getAlfInfoInPhFlag() const                                       { return m_alfInfoInPhFlag;                     }
  void                   setWpInfoInPhFlag(bool flag)                                     { m_wpInfoInPhFlag = flag;                      }
  bool                   getWpInfoInPhFlag() const                                        { return m_wpInfoInPhFlag;                      }
  void                   setQpDeltaInfoInPhFlag(bool flag)                                { m_qpDeltaInfoInPhFlag = flag;                 }
  bool                   getQpDeltaInfoInPhFlag() const                                   { return m_qpDeltaInfoInPhFlag; }

  bool                   getScalingListPresentFlag() const                                { return m_scalingListPresentFlag;              }
  void                   setScalingListPresentFlag( bool b )                              { m_scalingListPresentFlag  = b;                }
  ScalingList&           getScalingList()                                                 { return m_scalingList;                         }
  const ScalingList&     getScalingList() const                                           { return m_scalingList;                         }

  void                    setPicWidthInLumaSamples( uint32_t u )                          { m_picWidthInLumaSamples = u; }
  uint32_t                getPicWidthInLumaSamples() const                                { return  m_picWidthInLumaSamples; }
  void                    setPicHeightInLumaSamples( uint32_t u )                         { m_picHeightInLumaSamples = u; }
  uint32_t                getPicHeightInLumaSamples() const                               { return  m_picHeightInLumaSamples; }

  void                    setConformanceWindowPresentFlag(bool b)                         { m_conformanceWindowPresentFlag = b;           }
  bool                    getConformanceWindowPresentFlag() const                         { return m_conformanceWindowPresentFlag;        }
  Window&                 getConformanceWindow()                                          { return  m_conformanceWindow; }
  const Window&           getConformanceWindow() const                                    { return  m_conformanceWindow; }
  void                    setConformanceWindow( const Window& conformanceWindow )         { m_conformanceWindow = conformanceWindow; }
  Window&                 getScalingWindow()                                              { return  m_scalingWindow; }
  const Window&           getScalingWindow()                                        const { return  m_scalingWindow; }
  void                    setScalingWindow( const Window& scalingWindow )                 { m_scalingWindow = scalingWindow; }
  int                     getMixedNaluTypesInPicFlag() const                              { return m_mixedNaluTypesInPicFlag; }
  void                    setMixedNaluTypesInPicFlag( const bool flag )                   { m_mixedNaluTypesInPicFlag = flag; }
  void                    finalizePPSPartitioning( const SPS* pcSPS );
};

class APS: public BasePS<APS>
{
private:
  int                    m_APSId                = 0;              // adaptation_parameter_set_id
  int                    m_APSType              = 0;              // aps_params_type
  int                    m_layerId              = 0;
  AlfSliceParam          m_alfAPSParam;
  CcAlfFilterParam       m_ccAlfAPSParam;
  SliceReshapeInfo       m_reshapeAPSInfo;
  ScalingList            m_scalingListApsInfo;
  bool                   m_hasPrefixNalUnitType = false;

public:
  bool                   chromaPresentFlag      = false;

  APS()  = default;
  ~APS() = default;

  int                    getAPSId() const                                                 { return m_APSId;                               }
  void                   setAPSId(int i)                                                  { m_APSId = i;                                  }

  int                    getAPSType() const                                               { return m_APSType;                             }
  void                   setAPSType(int type)                                             { m_APSType = type;                             }

  void                   setAlfAPSParam( AlfSliceParam& alfAPSParam )                     { m_alfAPSParam = alfAPSParam;                  }
        AlfSliceParam&   getAlfAPSParam()                                                 { return m_alfAPSParam;                         }
  const AlfSliceParam&   getAlfAPSParam() const                                           { return m_alfAPSParam;                         }
  AlfSliceParam&         getMutableAlfAPSParam() const                                    { m_alfAPSParam.recostructMutex.lock(); return const_cast<AlfSliceParam&>( m_alfAPSParam ); }
  void                   releaseMutableAlfAPSParam( AlfSliceParam& alfAPSParam ) const    { alfAPSParam.recostructMutex.unlock(); }
  void                   setCcAlfAPSParam( CcAlfFilterParam& ccAlfAPSParam )              { m_ccAlfAPSParam = ccAlfAPSParam;              }
  CcAlfFilterParam&      getCcAlfAPSParam()                                               { return m_ccAlfAPSParam;                       }
  const CcAlfFilterParam& getCcAlfAPSParam() const                                        { return m_ccAlfAPSParam;                       }

  void                   setTemporalId(int i)                                             { m_alfAPSParam.tLayer = i;                     }
  int                    getTemporalId()                                            const { return m_alfAPSParam.tLayer;                  }
  void                   setLayerId( int i )                                              { m_layerId = i;                                }
  int                    getLayerId()                                               const { return m_layerId;                             }
  void                   setReshaperAPSInfo( SliceReshapeInfo& reshapeAPSInfo )           { m_reshapeAPSInfo = reshapeAPSInfo;            }
  const SliceReshapeInfo& getReshaperAPSInfo()                                      const { return m_reshapeAPSInfo;                      }
  SliceReshapeInfo&      getReshaperAPSInfo()                                             { return m_reshapeAPSInfo;                      }
  void                   setScalingList( ScalingList&& scalingListAPSInfo )               { m_scalingListApsInfo = std::move( scalingListAPSInfo ); }
  const ScalingList&     getScalingList()                                           const { return m_scalingListApsInfo;                  }
  ScalingList&           getScalingList()                                                 { return m_scalingListApsInfo;                  }
  void                   setHasPrefixNalUnitType( bool b )                                { m_hasPrefixNalUnitType = b;                   }
  bool                   getHasPrefixNalUnitType() const                                  { return m_hasPrefixNalUnitType;                }
};

struct WPScalingParam
{
  // Explicit weighted prediction parameters parsed in slice header,
  // or Implicit weighted prediction parameters (8 bits depth values).
  bool     bPresentFlag      = false;
  uint32_t uiLog2WeightDenom = 0;
  int      iWeight           = 0;
  int      iOffset           = 0;

  // Weighted prediction scaling values built from above parameters (bitdepth scaled):
  int w      = 0;
  int o      = 0;
  int offset = 0;
  int shift  = 0;
  int round  = 0;
};
struct WPACDCParam
{
  int64_t iAC = 0;
  int64_t iDC = 0;
};

// picture header class
class PicHeader
{
private:
  bool                        m_valid                                         = false;   //!< picture header is valid yet or not
  Picture*                    m_pcPic                                         = nullptr; //!< pointer to picture structure
  int                         m_pocLsb                                        = -1;      //!< least significant bits of picture order count
  bool                        m_nonReferencePictureFlag                       = false;   //!< non-reference picture flag
  bool                        m_gdrOrIrapPicFlag                              = false;   //!< gdr or irap picture flag
  bool                        m_pocMsbPresentFlag                             = false;  //!< ph_poc_msb_present_flag
  int                         m_pocMsbVal                                     = 0;      //!< poc_msb_val
  bool                        m_gdrPicFlag                                    = false;   //!< gradual decoding refresh picture flag
  bool                        m_handleCraAsCvsStartFlag                       = false;                                //!< HandleCraAsCvsStartFlag
  bool                        m_handleGdrAsCvsStartFlag                       = false;                                //!< HandleGdrAsCvsStartFlag
  uint32_t                    m_recoveryPocCnt                                = 0;       //!< recovery POC count
  bool                        m_noOutputBeforeRecoveryFlag                    = false;                             //!< NoOutputBeforeRecoveryFlag
  int                         m_spsId                                         = -1;      //!< sequence parameter set ID
  int                         m_ppsId                                         = -1;      //!< picture parameter set ID
  bool                        m_virtualBoundariesPresentFlag                  = false;   //!< disable loop filtering across virtual boundaries
  unsigned                    m_numVerVirtualBoundaries                       = 0;       //!< number of vertical virtual boundaries
  unsigned                    m_numHorVirtualBoundaries                       = 0;       //!< number of horizontal virtual boundaries
  unsigned                    m_virtualBoundariesPosX[3]                      = { 0, 0, 0 }; //!< horizontal virtual boundary positions
  unsigned                    m_virtualBoundariesPosY[3]                      = { 0, 0, 0 }; //!< vertical virtual boundary positions
  bool                        m_picOutputFlag                                 = true;    //!< picture output flag
  ReferencePictureList        m_RPL[2];                                                  //!< RPL for L0/L1 when present in picture header
  int                         m_RPLIdx[2]                                     = { 0, 0 };//!< index of used RPL L0/L1 in the SPS or -1 for local RPL in the picture header
  bool                        m_picInterSliceAllowedFlag                      = false;  //!< inter slice allowed flag in PH
  bool                        m_picIntraSliceAllowedFlag                      = true;   //!< intra slice allowed flag in PH
  bool                        m_splitConsOverrideFlag                         = false;  //!< partitioning constraint override flag
  uint32_t                    m_cuQpDeltaSubdivIntra                          = 0;      //!< CU QP delta maximum subdivision for intra slices
  uint32_t                    m_cuQpDeltaSubdivInter                          = 0;      //!< CU QP delta maximum subdivision for inter slices
  uint32_t                    m_cuChromaQpOffsetSubdivIntra                   = 0;      //!< CU chroma QP offset maximum subdivision for intra slices
  uint32_t                    m_cuChromaQpOffsetSubdivInter                   = 0;      //!< CU chroma QP offset maximum subdivision for inter slices
  bool                        m_enableTMVPFlag                                = false;  //!< enable temporal motion vector prediction
  bool                        m_picColFromL0Flag                              = false;  //!< syntax element collocated_from_l0_flag
  uint32_t                    m_colRefIdx                                     = 0;
  bool                        m_mvdL1ZeroFlag                                 = true;   //!< L1 MVD set to zero flag
  uint32_t                    m_maxNumAffineMergeCand                         = AFFINE_MRG_MAX_NUM_CANDS; //!< max number of sub-block merge candidates
  bool                        m_disFracMMVD                                   = false;  //!< fractional MMVD offsets disabled flag
  bool                        m_disBdofFlag                                   = false;  //!< picture level BDOF disable flag
  bool                        m_disDmvrFlag                                   = false;  //!< picture level DMVR disable flag
  bool                        m_disProfFlag                                   = false;  //!< picture level PROF disable flag
  uint32_t                    m_maxNumGeoCand                                 = 0;      //!< max number of geometric merge candidates
  uint32_t                    m_maxNumIBCMergeCand                            = IBC_MRG_MAX_NUM_CANDS; //!< max number of IBC merge candidates
  bool                        m_jointCbCrSignFlag                             = false;  //!< joint Cb/Cr residual sign flag
  int                         m_qpDelta                                       = 0;      //!< value of Qp delta
  bool                        m_saoEnabledFlag[MAX_NUM_CHANNEL_TYPE]          = { false, false }; //!< sao enabled flags for each channel
  bool                        m_alfEnabledFlag[MAX_NUM_COMPONENT]             = { false, false, false }; //!< alf enabled flags for each component
  int                         m_numAlfAps                                     = 0;      //!< number of alf aps active for the picture
  AlfApsIdVec                 m_alfApsId;                                               //!< list of alf aps for the picture
  bool                        m_ccalfEnabledFlag[MAX_NUM_COMPONENT]           = { false, false, false };
  int                         m_ccalfCbApsId                                  = -1;
  int                         m_ccalfCrApsId                                  = -1;
  int                         m_alfChromaApsId                                = 0;      //!< chroma alf aps ID
  bool                        m_deblockingFilterOverrideFlag                  = false;  //!< deblocking filter override controls enabled
  bool                        m_deblockingFilterDisable                       = false;  //!< deblocking filter disabled flag
  int                         m_deblockingFilterBetaOffsetDiv2                = 0;      //!< beta offset for deblocking filter
  int                         m_deblockingFilterTcOffsetDiv2                  = 0;      //!< tc offset for deblocking filter
  int                         m_deblockingFilterCbBetaOffsetDiv2              = 0;                       //!< beta offset for deblocking filter
  int                         m_deblockingFilterCbTcOffsetDiv2                = 0;                         //!< tc offset for deblocking filter
  int                         m_deblockingFilterCrBetaOffsetDiv2              = 0;                       //!< beta offset for deblocking filter
  int                         m_deblockingFilterCrTcOffsetDiv2                = 0;                         //!< tc offset for deblocking filter
  bool                        m_lmcsEnabledFlag                               = false;  //!< lmcs enabled flag
  int                         m_lmcsApsId                                     = -1;     //!< lmcs APS ID
  std::shared_ptr<const APS>  m_lmcsAps                                       = nullptr; //!< lmcs APS
  bool                        m_lmcsChromaResidualScaleFlag                   = false;  //!< lmcs chroma residual scale flag
  bool                        m_explicitScalingListEnabledFlag                = false;  //!< explicit quantization scaling list enabled
  int                         m_scalingListApsId                              = -1;     //!< quantization scaling list APS ID
  std::shared_ptr<const APS>  m_scalingListAps                                = nullptr; //!< quantization scaling list APS
  PartitionConstraints        m_minQT                                         = PartitionConstraints{ 0, 0, 0 }; //!< minimum quad-tree size  0: I slice luma; 1: P/B slice luma; 2: I slice chroma
  PartitionConstraints        m_maxMTTHierarchyDepth                          = PartitionConstraints{ 0, 0, 0 }; //!< maximum MTT depth
  PartitionConstraints        m_maxBTSize                                     = PartitionConstraints{ 0, 0, 0 }; //!< maximum BT size
  PartitionConstraints        m_maxTTSize                                     = PartitionConstraints{ 0, 0, 0 }; //!< maximum TT size

  WPScalingParam              m_weightPredTable[NUM_REF_PIC_LIST_01][MAX_NUM_REF][MAX_NUM_COMPONENT];   // [REF_PIC_LIST_0 or REF_PIC_LIST_1][refIdx][0:Y, 1:U, 2:V]
  int                         m_numL0Weights                                  = 0;  //!< number of weights for L0 list
  int                         m_numL1Weights                                  = 0;  //!< number of weights for L1 list

public:
                              PicHeader() = default;
                              ~PicHeader() { m_alfApsId.resize(0); }
//  void                        initPicHeader();
  bool                        isValid() const                                           { return m_valid;                                                                              }
  void                        setValid()                                                { m_valid = true;                                                                              }
  void                        setPic( Picture* p )                                      { m_pcPic = p;                                                                                 }
  Picture*                    getPic()                                                  { return m_pcPic;                                                                              }
  const Picture*              getPic() const                                            { return m_pcPic;                                                                              }
  void                        setPocLsb(int i)                                          { m_pocLsb = i;                                                                                }
  int                         getPocLsb() const                                         { return m_pocLsb;                                                                             }
  void                        setNonReferencePictureFlag( bool b )                      { m_nonReferencePictureFlag = b;                                                               }
  bool                        getNonReferencePictureFlag() const                        { return m_nonReferencePictureFlag;                                                            }
  void                        setGdrOrIrapPicFlag( bool b )                             { m_gdrOrIrapPicFlag = b;                                                                      }
  bool                        getGdrOrIrapPicFlag() const                               { return m_gdrOrIrapPicFlag;                                                                   }
  void                        setGdrPicFlag( bool b )                                   { m_gdrPicFlag = b;                                                                            }
  bool                        getGdrPicFlag() const                                     { return m_gdrPicFlag;                                                                         }
  void                        setRecoveryPocCnt( uint32_t u )                           { m_recoveryPocCnt = u;                                                                        }
  uint32_t                    getRecoveryPocCnt() const                                 { return m_recoveryPocCnt;                                                                     }
  void                        setSPSId( uint32_t u )                                    { m_spsId = u;                                                                                 }
  uint32_t                    getSPSId() const                                          { return m_spsId;                                                                              }
  void                        setPPSId( uint32_t u )                                    { m_ppsId = u;                                                                                 }
  uint32_t                    getPPSId() const                                          { return m_ppsId;                                                                              }
  void                        setPocMsbPresentFlag(bool b)                              { m_pocMsbPresentFlag = b;                                                                     }
  bool                        getPocMsbPresentFlag() const                              { return m_pocMsbPresentFlag;                                                                  }
  void                        setPocMsbVal(int i)                                       { m_pocMsbVal = i;                                                                             }
  int                         getPocMsbVal() const                                      { return m_pocMsbVal;                                                                          }
  void                        setVirtualBoundariesPresentFlag( bool b )                 { m_virtualBoundariesPresentFlag = b;                                                          }
  bool                        getVirtualBoundariesPresentFlag() const                   { return m_virtualBoundariesPresentFlag;                                                       }
  void                        setNumVerVirtualBoundaries(unsigned u)                    { m_numVerVirtualBoundaries = u;                                                               }
  unsigned                    getNumVerVirtualBoundaries() const                        { return m_numVerVirtualBoundaries;                                                            }
  void                        setNumHorVirtualBoundaries(unsigned u)                    { m_numHorVirtualBoundaries = u;                                                               }
  unsigned                    getNumHorVirtualBoundaries() const                        { return m_numHorVirtualBoundaries;                                                            }
  void                        setVirtualBoundariesPosX(unsigned u, unsigned idx)        { CHECK( idx >= 3, "boundary index exceeds valid range" ); m_virtualBoundariesPosX[idx] = u;   }
  unsigned                    getVirtualBoundariesPosX(unsigned idx) const              { CHECK( idx >= 3, "boundary index exceeds valid range" ); return m_virtualBoundariesPosX[idx];}
  void                        setVirtualBoundariesPosY(unsigned u, unsigned idx)        { CHECK( idx >= 3, "boundary index exceeds valid range" ); m_virtualBoundariesPosY[idx] = u;   }
  unsigned                    getVirtualBoundariesPosY(unsigned idx) const              { CHECK( idx >= 3, "boundary index exceeds valid range" ); return m_virtualBoundariesPosY[idx];}
  void                        setPicOutputFlag( bool b )                                { m_picOutputFlag = b;                                                                         }
  bool                        getPicOutputFlag() const                                  { return m_picOutputFlag;                                                                      }
  void                        clearRPL( RefPicList l )                                  { m_RPL[l].clear();                                                                            }
  void                        setRPL( RefPicList l, const ReferencePictureList& rpl )   { m_RPL[l] = rpl;                                                                              }
  ReferencePictureList*       getRPL( RefPicList l )                                    { return &m_RPL[l];                                                                            }
  const ReferencePictureList* getRPL( RefPicList l ) const                              { return &m_RPL[l];                                                                            }
  void                        setRPLIdx( RefPicList l, int RPLIdx)                      { m_RPLIdx[l] = RPLIdx;                                                                        }
  int                         getRPLIdx( RefPicList l ) const                           { return m_RPLIdx[l];                                                                          }
  void                        setPicInterSliceAllowedFlag(bool b)                       { m_picInterSliceAllowedFlag = b; }
  bool                        getPicInterSliceAllowedFlag() const                       { return m_picInterSliceAllowedFlag; }
  void                        setPicIntraSliceAllowedFlag(bool b)                       { m_picIntraSliceAllowedFlag = b; }
  bool                        getPicIntraSliceAllowedFlag() const                       { return m_picIntraSliceAllowedFlag; }
  void                        setSplitConsOverrideFlag( bool b )                        { m_splitConsOverrideFlag = b;                                                                 }
  bool                        getSplitConsOverrideFlag() const                          { return m_splitConsOverrideFlag;                                                              }  
  void                        setCuQpDeltaSubdivIntra( uint32_t u )                     { m_cuQpDeltaSubdivIntra = u;                                                                  }
  uint32_t                    getCuQpDeltaSubdivIntra() const                           { return m_cuQpDeltaSubdivIntra;                                                               }
  void                        setCuQpDeltaSubdivInter( uint32_t u )                     { m_cuQpDeltaSubdivInter = u;                                                                  }
  uint32_t                    getCuQpDeltaSubdivInter() const                           { return m_cuQpDeltaSubdivInter;                                                               }
  void                        setCuChromaQpOffsetSubdivIntra( uint32_t u )              { m_cuChromaQpOffsetSubdivIntra = u;                                                           }
  uint32_t                    getCuChromaQpOffsetSubdivIntra() const                    { return m_cuChromaQpOffsetSubdivIntra;                                                        }
  void                        setCuChromaQpOffsetSubdivInter( uint32_t u )              { m_cuChromaQpOffsetSubdivInter = u;                                                           }
  uint32_t                    getCuChromaQpOffsetSubdivInter() const                    { return m_cuChromaQpOffsetSubdivInter;                                                        }
  void                        setEnableTMVPFlag( bool b )                               { m_enableTMVPFlag = b;                                                                        }
  bool                        getEnableTMVPFlag() const                                 { return m_enableTMVPFlag;                                                                     }
  void                        setPicColFromL0Flag(bool val)                             { m_picColFromL0Flag = val;                                                                    }
  bool                        getPicColFromL0Flag() const                               { return m_picColFromL0Flag;                                                                   }
  void                        setColRefIdx( uint32_t refIdx)                            { m_colRefIdx = refIdx;                                                                        }
  uint32_t                    getColRefIdx() const                                      { return m_colRefIdx;                                                                          }
  void                        setMvdL1ZeroFlag( bool b )                                { m_mvdL1ZeroFlag = b;                                                                         }
  bool                        getMvdL1ZeroFlag() const                                  { return m_mvdL1ZeroFlag;                                                                      }  
  void                        setMaxNumAffineMergeCand( uint32_t val )                  { m_maxNumAffineMergeCand = val;                                                               }
  uint32_t                    getMaxNumAffineMergeCand() const                          { return m_maxNumAffineMergeCand;                                                              }
  void                        setDisFracMMVD( bool val )                                { m_disFracMMVD = val;                                                                         }
  bool                        getDisFracMMVD() const                                    { return m_disFracMMVD;                                                                        }  
  void                        setDisBdofFlag( bool val )                                { m_disBdofFlag = val;                                                                         }
  bool                        getDisBdofFlag() const                                    { return m_disBdofFlag;                                                                        }
  void                        setDisDmvrFlag( bool val )                                { m_disDmvrFlag = val;                                                                         }
  bool                        getDisDmvrFlag() const                                    { return m_disDmvrFlag;                                                                        }
  void                        setDisProfFlag( bool val )                                { m_disProfFlag = val;                                                                         }
  bool                        getDisProfFlag() const                                    { return m_disProfFlag;                                                                        }
  void                        setMaxNumGeoCand(uint32_t b)                              { m_maxNumGeoCand = b; }
  uint32_t                    getMaxNumGeoCand() const                                  { return m_maxNumGeoCand; }
  void                        setMaxNumIBCMergeCand( uint32_t b )                       { m_maxNumIBCMergeCand = b;                                                                    }
  uint32_t                    getMaxNumIBCMergeCand() const                             { return m_maxNumIBCMergeCand;                                                                 } 
  void                        setJointCbCrSignFlag( bool b )                            { m_jointCbCrSignFlag = b;                                                                     }
  bool                        getJointCbCrSignFlag() const                              { return m_jointCbCrSignFlag;                                                                  }
  void                        setQpDelta(int b)                                         { m_qpDelta = b;                                                                               }
  int                         getQpDelta() const                                        { return m_qpDelta;                                                                            }
  void                        setSaoEnabledFlag(ChannelType chType, bool b)             { m_saoEnabledFlag[chType] = b;                                                                }
  bool                        getSaoEnabledFlag(ChannelType chType) const               { return m_saoEnabledFlag[chType];                                                             }  
  void                        setAlfEnabledFlag(ComponentID compId, bool b)             { m_alfEnabledFlag[compId] = b;                                                                }
  bool                        getAlfEnabledFlag(ComponentID compId) const               { return m_alfEnabledFlag[compId];                                                             }
  void                        setNumAlfAps(int i)                                       { m_numAlfAps = i;                                                                             }
  int                         getNumAlfAps() const                                      { return m_numAlfAps;                                                                          }
  void                        setAlfApsIdChroma(int i)                                  { m_alfChromaApsId = i;                                                                        }
  int                         getAlfApsIdChroma() const                                 { return m_alfChromaApsId;                                                                     }  
  void                        setCcAlfEnabledFlag(ComponentID compId, bool b)           { m_ccalfEnabledFlag[compId] = b; }
  bool                        getCcAlfEnabledFlag(ComponentID compId) const             { return m_ccalfEnabledFlag[compId]; }

  void                        setCcAlfCbApsId(int i)                                    { m_ccalfCbApsId = i; }
  int                         getCcAlfCbApsId() const                                   { return m_ccalfCbApsId; }
  void                        setCcAlfCrApsId(int i)                                    { m_ccalfCrApsId = i; }
  int                         getCcAlfCrApsId() const                                   { return m_ccalfCrApsId; }
  void                        setDeblockingFilterOverrideFlag( bool b )                 { m_deblockingFilterOverrideFlag = b;                                                          }
  bool                        getDeblockingFilterOverrideFlag() const                   { return m_deblockingFilterOverrideFlag;                                                       }    
  void                        setDeblockingFilterDisable( bool b )                      { m_deblockingFilterDisable= b;                                                                }  
  bool                        getDeblockingFilterDisable() const                        { return m_deblockingFilterDisable;                                                            }
  void                        setDeblockingFilterBetaOffsetDiv2( int i )                { m_deblockingFilterBetaOffsetDiv2 = i;                                                        }  
  int                         getDeblockingFilterBetaOffsetDiv2()const                  { return m_deblockingFilterBetaOffsetDiv2;                                                     }
  void                        setDeblockingFilterTcOffsetDiv2( int i )                  { m_deblockingFilterTcOffsetDiv2 = i;                                                          }  
  int                         getDeblockingFilterTcOffsetDiv2() const                   { return m_deblockingFilterTcOffsetDiv2;                                                       }    
  void                        setDeblockingFilterCbBetaOffsetDiv2( int i )              { m_deblockingFilterCbBetaOffsetDiv2 = i;                                                      }
  int                         getDeblockingFilterCbBetaOffsetDiv2()const                { return m_deblockingFilterCbBetaOffsetDiv2;                                                   }
  void                        setDeblockingFilterCbTcOffsetDiv2( int i )                { m_deblockingFilterCbTcOffsetDiv2 = i;                                                        }
  int                         getDeblockingFilterCbTcOffsetDiv2() const                 { return m_deblockingFilterCbTcOffsetDiv2;                                                     }
  void                        setDeblockingFilterCrBetaOffsetDiv2( int i )              { m_deblockingFilterCrBetaOffsetDiv2 = i;                                                      }
  int                         getDeblockingFilterCrBetaOffsetDiv2()const                { return m_deblockingFilterCrBetaOffsetDiv2;                                                   }
  void                        setDeblockingFilterCrTcOffsetDiv2( int i )                { m_deblockingFilterCrTcOffsetDiv2 = i;                                                        }
  int                         getDeblockingFilterCrTcOffsetDiv2() const                 { return m_deblockingFilterCrTcOffsetDiv2;                                                     }
  void                        setLmcsEnabledFlag(bool b)                                { m_lmcsEnabledFlag = b;                                                                       }
  bool                        getLmcsEnabledFlag()                                      { return m_lmcsEnabledFlag;                                                                    }
  const bool                  getLmcsEnabledFlag() const                                { return m_lmcsEnabledFlag;                                                                    }
  void                        setLmcsAPS(std::shared_ptr<const APS> aps)                { m_lmcsAps = aps; m_lmcsApsId = (aps) ? aps->getAPSId() : -1;                                 }
  std::shared_ptr<const APS>  getLmcsAPS() const                                        { return m_lmcsAps;                                                                            }
  void                        setLmcsAPSId(int id)                                      { m_lmcsApsId = id;                                                                            }
  int                         getLmcsAPSId() const                                      { return m_lmcsApsId;                                                                          }
  void                        setLmcsChromaResidualScaleFlag(bool b)                    { m_lmcsChromaResidualScaleFlag = b;                                                           }
  bool                        getLmcsChromaResidualScaleFlag()                          { return m_lmcsChromaResidualScaleFlag;                                                        }
  const bool                  getLmcsChromaResidualScaleFlag() const                    { return m_lmcsChromaResidualScaleFlag;                                                        }
  void                        setScalingListAPS(std::shared_ptr<const APS> aps)         { m_scalingListAps = aps; m_scalingListApsId = ( aps ) ? aps->getAPSId() : -1;                 }
  std::shared_ptr<const APS>  getScalingListAPS() const                                 { return m_scalingListAps;                                                                     }
  void                        setScalingListAPSId( int id )                             { m_scalingListApsId = id;                                                                     }
  int                         getScalingListAPSId() const                               { return m_scalingListApsId;                                                                   }
  void                        setExplicitScalingListEnabledFlag( bool b )               { m_explicitScalingListEnabledFlag = b;                                                        }
  bool                        getExplicitScalingListEnabledFlag()                       { return m_explicitScalingListEnabledFlag;                                                     }
  bool                        getExplicitScalingListEnabledFlag() const                 { return m_explicitScalingListEnabledFlag;                                                     }

  const PartitionConstraints& getMinQTSizes() const                                     { return m_minQT;                                                                              }
  const PartitionConstraints& getMaxBTSizes() const                                     { return m_maxBTSize;                                                                          }
  const PartitionConstraints& getMaxTTSizes() const                                     { return m_maxTTSize;                                                                          }

  void                        setMinQTSize           ( unsigned idx, unsigned minQT  )  { m_minQT[idx] = minQT;                                                                        }
  void                        setMaxMTTHierarchyDepth( unsigned idx, unsigned maxMTT )  { m_maxMTTHierarchyDepth[idx] = maxMTT;                                                        }
  void                        setMaxBTSize           ( unsigned idx, unsigned maxBT  )  { m_maxBTSize[idx] = maxBT;                                                                    }
  void                        setMaxTTSize           ( unsigned idx, unsigned maxTT  )  { m_maxTTSize[idx] = maxTT;                                                                    }

  void                        setMinQTSizes           ( const PartitionConstraints& minQT )  { m_minQT = minQT;                                                                        }
  void                        setMaxMTTHierarchyDepths( const PartitionConstraints& maxMTT ) { m_maxMTTHierarchyDepth = maxMTT;                                                        }
  void                        setMaxBTSizes           ( const PartitionConstraints& maxBT )  { m_maxBTSize = maxBT;                                                                    }
  void                        setMaxTTSizes           ( const PartitionConstraints& maxTT )  { m_maxTTSize = maxTT;                                                                    }

  unsigned                    getMinQTSize           ( SliceType slicetype, ChannelType chType = CHANNEL_TYPE_LUMA ) const { return m_minQT               [chType == CHANNEL_TYPE_LUMA ? slicetype == I_SLICE ?  0 : 1 : 2]; }
  unsigned                    getMaxMTTHierarchyDepth( SliceType slicetype, ChannelType chType = CHANNEL_TYPE_LUMA ) const { return m_maxMTTHierarchyDepth[chType == CHANNEL_TYPE_LUMA ? slicetype == I_SLICE ?  0 : 1 : 2]; }
  unsigned                    getMaxBTSize           ( SliceType slicetype, ChannelType chType = CHANNEL_TYPE_LUMA ) const { return m_maxBTSize           [chType == CHANNEL_TYPE_LUMA ? slicetype == I_SLICE ?  0 : 1 : 2]; }
  unsigned                    getMaxTTSize           ( SliceType slicetype, ChannelType chType = CHANNEL_TYPE_LUMA ) const { return m_maxTTSize           [chType == CHANNEL_TYPE_LUMA ? slicetype == I_SLICE ?  0 : 1 : 2]; }


  void                        setAlfAPSIds( const AlfApsIdVec& apsIDs )                { m_alfApsId = std::move( apsIDs ); }
  const AlfApsIdVec&          getAlfAPSIds() const                                     { return m_alfApsId;                }

  void                        setWpScaling(WPScalingParam *wp)
  {
    memcpy(m_weightPredTable, wp, sizeof(WPScalingParam) * NUM_REF_PIC_LIST_01 * MAX_NUM_REF * MAX_NUM_COMPONENT);
  }
  void                        getWpScaling(RefPicList e, int iRefIdx, WPScalingParam *&wp) const;
  WPScalingParam*             getWpScalingAll()                                        { return (WPScalingParam *) m_weightPredTable; }
  void                        setNumL0Weights(int b)                                   { m_numL0Weights = b;                          }
  int                         getNumL0Weights()                                        { return m_numL0Weights;                       }
  void                        setNumL1Weights(int b)                                   { m_numL1Weights = b;                          }
  int                         getNumL1Weights()                                        { return m_numL1Weights;                       }

  void                        setNoOutputBeforeRecoveryFlag( bool val )                { m_noOutputBeforeRecoveryFlag = val;  }
  bool                        getNoOutputBeforeRecoveryFlag() const                    { return m_noOutputBeforeRecoveryFlag; }
  void                        setHandleCraAsCvsStartFlag( bool val )                   { m_handleCraAsCvsStartFlag = val;     }
  bool                        getHandleCraAsCvsStartFlag() const                       { return m_handleCraAsCvsStartFlag;    }
  void                        setHandleGdrAsCvsStartFlag( bool val )                   { m_handleGdrAsCvsStartFlag = val;     }
  bool                        getHandleGdrAsCvsStartFlag() const                       { return m_handleGdrAsCvsStartFlag;    }
};

/// slice header class
class Slice
{
private:
  //  Bitstream writing
  bool                       m_saoEnabledFlag[MAX_NUM_CHANNEL_TYPE] = { false, false };
  int                        m_iPOC                             = 0;
  int                        m_iLastIDR                         = 0;
  int                        m_iAssociatedIRAP                  = 0;
  NalUnitType                m_iAssociatedIRAPType              = NAL_UNIT_INVALID;
  ReferencePictureList       m_RPL[2];                                                               //< RPL for L0/L1 when present in slice header
  int                        m_RPLIdx[2]                        = { -1, -1 };                        //< index of used RPL in the SPS or -1 for local RPL in the slice header
  NalUnitType                m_eNalUnitType                     = NAL_UNIT_CODED_SLICE_IDR_W_RADL;   ///< Nal unit type for the slice
  bool                       m_pictureHeaderInSliceHeader       = false;
  uint32_t                   m_nuhLayerId                       = 0;   ///< Nal unit layer id
#if TBC
  int                        m_prevGDRSubpicPOC                 = 0;
  int                        m_prevIRAPSubpicPOC                = 0;
  NalUnitType                m_prevIRAPSubpicType               = NAL_UNIT_INVALID;
#endif
  SliceType                  m_eSliceType                       = I_SLICE;
  bool                       m_noOutputOfPriorPicsFlag          = false;           //!< no output of prior pictures flag
  int                        m_iSliceQp                         = 0;

  bool                       m_ChromaQpAdjEnabled               = false;
  bool                       m_deblockingFilterDisable          = false;
  bool                       m_deblockingFilterOverrideFlag     = false;   //< offsets for deblocking filter inherit from PPS
  int                        m_deblockingFilterBetaOffsetDiv2   = 0;       //< beta offset for deblocking filter
  int                        m_deblockingFilterTcOffsetDiv2     = 0;       //< tc offset for deblocking filter
  int                        m_deblockingFilterCbBetaOffsetDiv2 = 0;       //< beta offset for deblocking filter
  int                        m_deblockingFilterCbTcOffsetDiv2   = 0;       //< tc offset for deblocking filter
  int                        m_deblockingFilterCrBetaOffsetDiv2 = 0;       //< beta offset for deblocking filter
  int                        m_deblockingFilterCrTcOffsetDiv2   = 0;       //< tc offset for deblocking filter
  bool                       m_depQuantEnabledFlag              = false;   //!< dependent quantization enabled flag
  bool                       m_signDataHidingEnabledFlag        = false;   //!< sign data hiding enabled flag
  bool                       m_tsResidualCodingDisabledFlag     = false;
  bool                       m_lmcsEnabledFlag                  = false;
  bool                       m_explicitScalingListUsed          = false;
  int                        m_aiNumRefIdx[NUM_REF_PIC_LIST_01] = { 0, 0 }; //  for multiple reference of current slice

  bool                       m_bCheckLDC                        = false;

  bool                       m_biDirPred                        = false;
  int                        m_symRefIdx[2]                     = { -1, -1 };

  //  Data
  int                        m_iSliceChromaQpDelta[MAX_NUM_COMPONENT + 1]                = { 0 };
  Picture*                   m_apcRefPicList      [NUM_REF_PIC_LIST_01][MAX_NUM_REF + 1] = { { nullptr } };   // entry 0 in m_apcRefPicList is nullptr!
  int                        m_aiRefPOCList       [NUM_REF_PIC_LIST_01][MAX_NUM_REF + 1] = { { 0 } };         // this is needed to get the POC of the a reference picture, when this slice is used as a collocated reference and the pictures referenced by this one have already been reused. (needed for TMVP)
  bool                       m_bIsUsedAsLongTerm  [NUM_REF_PIC_LIST_01][MAX_NUM_REF + 1] = { { false } };
  std::pair<int, int>        m_scalingRatio       [NUM_REF_PIC_LIST_01][MAX_NUM_REF_PICS];

  // access channel
  const VPS*                 m_pcVPS                         = nullptr;
  const SPS*                 m_pcSPS                         = nullptr;
  const PPS*                 m_pcPPS                         = nullptr;
  Picture*                   m_pcPic                         = nullptr;
  PicHeader*                 m_pcPicHeader                   = nullptr;    //!< pointer to picture header structure
  bool                       m_colFromL0Flag                 = true;   // collocated picture from List0 flag

  uint32_t                   m_colRefIdx                     = 0;
  uint32_t                   m_maxNumIBCMergeCand            = 0;
  bool                       m_disBdofDmvrFlag               = false;

  uint32_t                   m_uiTLayer                      = false;
  bool                       m_bTLayerSwitchingFlag          = false;

  uint32_t                   m_independentSliceIdx           = 0;

  WPScalingParam             m_weightPredTable[NUM_REF_PIC_LIST_01][MAX_NUM_REF][MAX_NUM_COMPONENT];   // [REF_PIC_LIST_0 or REF_PIC_LIST_1][refIdx][0:Y, 1:U, 2:V]
  WPACDCParam                m_weightACDCParam[MAX_NUM_COMPONENT];
  ClpRngs                    m_clpRngs;
  std::vector<uint32_t>      m_substreamSizes;
  uint32_t                   m_numEntryPoints                = 0;
  bool                       m_cabacInitFlag                 = false;
  SliceMap                   m_sliceMap;                     //!< list of CTUs in current slice - raster scan CTU addresses

  uint32_t                   m_sliceSubPicId                 = 0;

  const APS*                 m_alfApss[ALF_CTB_MAX_NUM_APS]      = { 0 };
  bool                       m_alfEnabledFlag[MAX_NUM_COMPONENT] = { false, false, false };
  int                        m_numAlfAps                         = 0;
  AlfApsIdVec                m_lumaAlfApsId;
  int                        m_chromaAlfApsId                    = -1;
  bool                       m_ccAlfEnabledFlags[2]              = { false, false };
  int                        m_ccAlfCbApsId                      = -1;
  int                        m_ccAlfCrApsId                      = -1;

public:
                              Slice();
  void                        initSlice();
  void                        inheritFromPicHeader( const PicHeader* picHeader, const PPS *pps, const SPS *sps );
  void                        setPicHeader( PicHeader* pcPicHeader )                 { m_pcPicHeader = pcPicHeader;                                  }
  PicHeader*                  getPicHeader() const                                   { return m_pcPicHeader;                                         }
  int                         getRefIdx4MVPair( RefPicList eCurRefPicList, int nCurRefIdx );

  void                        setVPS( const VPS* pcVPS )                             { m_pcVPS = pcVPS;                                              }
  const VPS*                  getVPS() const                                         { CHECK_NULLPTR( m_pcVPS ); return m_pcVPS;                     }
  const VPS*                  getVPS_nothrow() const                                 { return m_pcVPS;                                               }

  void                        setSPS( const SPS* pcSPS )                             { m_pcSPS = pcSPS;                                              }
  const SPS*                  getSPS() const                                         { CHECK_NULLPTR( m_pcSPS ); return m_pcSPS;                     }

  void                        setPPS( const PPS* pcPPS )                             { m_pcPPS = pcPPS;                                              }
  const PPS*                  getPPS() const                                         { CHECK_NULLPTR( m_pcPPS ); return m_pcPPS;                     }

  void                        setAlfApss( std::shared_ptr<const APS> apss[ALF_CTB_MAX_NUM_APS] ) { for( int i = 0; i < ALF_CTB_MAX_NUM_APS; ++i ) { m_alfApss[i] = apss[i].get(); } }
  void                        setAlfApss(                 const APS *apss[ALF_CTB_MAX_NUM_APS] ) { for( int i = 0; i < ALF_CTB_MAX_NUM_APS; ++i ) { m_alfApss[i] = apss[i]; } }
  void                        clearAlfAPSs()                                         { memset( m_alfApss, 0, sizeof( m_alfApss ) );                  }
  const APS**                 getAlfAPSs()                                           { return m_alfApss;                                             }
  const APS* const*           getAlfAPSs() const                                     { return m_alfApss;                                             }
  void                        setSaoEnabledFlag(ChannelType chType, bool s)          { m_saoEnabledFlag[chType] = s;                                 }
  bool                        getSaoEnabledFlag(ChannelType chType) const            { return m_saoEnabledFlag[chType];                              }
  void                        clearRPL( RefPicList l )                               { m_RPL[l].clear();                                             }
  void                        setRPL( RefPicList l, const ReferencePictureList& rpl ){ m_RPL[l] = rpl;                                               }
  ReferencePictureList*       getRPL( RefPicList l )                                 { return &m_RPL[l];                                             }
  const ReferencePictureList* getRPL( RefPicList l ) const                           { return &m_RPL[l];                                             }
  void                        setRPLIdx( RefPicList l, int RPLIdx )                  { m_RPLIdx[l] = RPLIdx;                                         }
  int                         getRPLIdx( RefPicList l ) const                        { return m_RPLIdx[l];                                           }
  void                        setLastIDR(int iIDRPOC)                                { m_iLastIDR = iIDRPOC;                                         }
  int                         getLastIDR() const                                     { return m_iLastIDR;                                            }
  void                        setAssociatedIRAPPOC(int iAssociatedIRAPPOC)           { m_iAssociatedIRAP = iAssociatedIRAPPOC;                       }
  int                         getAssociatedIRAPPOC() const                           { return m_iAssociatedIRAP;                                     }
  void                        setAssociatedIRAPType(NalUnitType associatedIRAPType)  { m_iAssociatedIRAPType = associatedIRAPType;                   }
  NalUnitType                 getAssociatedIRAPType() const                          { return m_iAssociatedIRAPType;                                 }
#if TBC
  void                        setPrevGDRSubpicPOC(int poc)                           { m_prevGDRSubpicPOC = poc;                                     }
  int                         getPrevGDRSubpicPOC() const                            { return m_prevGDRSubpicPOC;                                    }
  void                        setPrevIRAPSubpicPOC(int poc)                          { m_prevIRAPSubpicPOC = poc;                                    }
  int                         getPrevIRAPSubpicPOC() const                           { return m_prevIRAPSubpicPOC;                                   }
  void                        setPrevIRAPSubpicType(NalUnitType type)                { m_prevIRAPSubpicType = type;                                  }
  NalUnitType                 getPrevIRAPSubpicType() const                          { return m_prevIRAPSubpicType;                                  }
  void                        checkSubpicTypeConstraints(PicList& rcListPic, const ReferencePictureList* pRPL0, const ReferencePictureList* pRPL1, const int prevIRAPSubpicDecOrderNo);
#endif
  SliceType                   getSliceType() const                                   { return m_eSliceType;                                          }
  void                        setNoOutputOfPriorPicsFlag(bool b)                     { m_noOutputOfPriorPicsFlag = b;                                }
  bool                        getNoOutputOfPriorPicsFlag() const                     { return m_noOutputOfPriorPicsFlag;                             }
  int                         getPOC() const                                         { return m_iPOC;                                                }
  int                         getSliceQp() const                                     { return m_iSliceQp;                                            }
  int                         getSliceChromaQpDelta(ComponentID compID) const        { return isLuma(compID) ? 0 : m_iSliceChromaQpDelta[compID];    }
  bool                        getUseChromaQpAdj() const                              { return m_ChromaQpAdjEnabled;                                  }
  bool                        getDeblockingFilterDisable() const                     { return m_deblockingFilterDisable;                             }
  bool                        getDeblockingFilterOverrideFlag() const                { return m_deblockingFilterOverrideFlag;                        }
  int                         getDeblockingFilterBetaOffsetDiv2()const               { return m_deblockingFilterBetaOffsetDiv2;                      }
  int                         getDeblockingFilterTcOffsetDiv2() const                { return m_deblockingFilterTcOffsetDiv2;                        }
  int                         getDeblockingFilterCbBetaOffsetDiv2()const             { return m_deblockingFilterCbBetaOffsetDiv2;                    }
  int                         getDeblockingFilterCbTcOffsetDiv2() const              { return m_deblockingFilterCbTcOffsetDiv2;                      }
  int                         getDeblockingFilterCrBetaOffsetDiv2()const             { return m_deblockingFilterCrBetaOffsetDiv2;                    }
  int                         getDeblockingFilterCrTcOffsetDiv2() const              { return m_deblockingFilterCrTcOffsetDiv2;                      }

  int                         getNumRefIdx( RefPicList e ) const                     { return m_aiNumRefIdx[e];                                      }
  Picture*                    getPic()                                               { return m_pcPic;                                               }
  const Picture*              getPic() const                                         { return m_pcPic;                                               }
        Picture*              getNoConstRefPic( RefPicList e, int iRefIdx) const     { return m_apcRefPicList[e][iRefIdx];                           }
  const Picture*              getRefPic( RefPicList e, int iRefIdx) const            { return m_apcRefPicList[e][iRefIdx];                           }
  int                         getRefPOC( RefPicList e, int iRefIdx) const            { return m_aiRefPOCList[e][iRefIdx];                            }
  bool                        getColFromL0Flag() const                               { return m_colFromL0Flag;                                       }
  uint32_t                    getColRefIdx() const                                   { return m_colRefIdx;                                           }
  void                        checkColRefIdx(uint32_t curSliceSegmentIdx, const Picture* pic);
  bool                        getIsUsedAsLongTerm(int i, int j) const                {
    return (j>=0) ? m_bIsUsedAsLongTerm[i][j] : m_bIsUsedAsLongTerm[i][0];                             }
  void                        setIsUsedAsLongTerm(int i, int j, bool value)          { m_bIsUsedAsLongTerm[i][j] = value;                            }
  bool                        getCheckLDC() const                                    { return m_bCheckLDC;                                           }
  void                        setPOC( int i )                                        { m_iPOC              = i;                                      }
  bool                        getPictureHeaderInSliceHeader() const                  { return m_pictureHeaderInSliceHeader;                         }
  void                        setPictureHeaderInSliceHeader( bool e )                { m_pictureHeaderInSliceHeader = e;                            }
  void                        setNalUnitType( NalUnitType e )                        { m_eNalUnitType      = e;                                      }
  NalUnitType                 getNalUnitType() const                                 { return m_eNalUnitType;                                        }
  void                        setNalUnitLayerId( uint32_t i )                        { m_nuhLayerId = i;                                             }
  uint32_t                    getNalUnitLayerId() const                              { return m_nuhLayerId;                                          }
  bool                        getRapPicFlag() const;
  bool                        getIdrPicFlag() const                                  { return getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP; }
  bool                        isIRAP() const                                         { return getNalUnitType() >= NAL_UNIT_CODED_SLICE_IDR_W_RADL && getNalUnitType() <= NAL_UNIT_CODED_SLICE_CRA;      }
#if GDR_ADJ
  bool                        isClvssPu() const                                      { return m_eNalUnitType >= NAL_UNIT_CODED_SLICE_IDR_W_RADL && m_eNalUnitType <= NAL_UNIT_CODED_SLICE_GDR && !m_pcPPS->getMixedNaluTypesInPicFlag() && m_pcPicHeader->getNoOutputBeforeRecoveryFlag(); }
#else
  bool                        isClvssPu() const                                      { return m_eNalUnitType >= NAL_UNIT_CODED_SLICE_IDR_W_RADL && m_eNalUnitType <= NAL_UNIT_CODED_SLICE_GDR && !m_pcPPS->getMixedNaluTypesInPicFlag(); }
#endif
  bool                        isIDR() const                                          { return m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL || m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP; }
  bool                        isCRAorGDR() const                                     { return m_eNalUnitType == NAL_UNIT_CODED_SLICE_CRA || m_eNalUnitType == NAL_UNIT_CODED_SLICE_GDR; }
  void                        checkCRA( int& pocCRA, NalUnitType& associatedIRAPType, const PicList& rcListPic );
  void                        checkSTSA( const PicList& rcListPic );
  void                        checkRPL( const ReferencePictureList* pRPL0, const ReferencePictureList* pRPL1, const int associatedIRAPDecodingOrderNumber, const PicList& rcListPic );
  void                        setSliceType( SliceType e )                            { m_eSliceType        = e;                                      }
  void                        setSliceQp( int i )                                    { m_iSliceQp          = i;                                      }
  void                        setSliceChromaQpDelta( ComponentID compID, int i )     { m_iSliceChromaQpDelta[compID] = isLuma(compID) ? 0 : i;       }
  void                        setUseChromaQpAdj( bool b )                            { m_ChromaQpAdjEnabled = b;                                     }
  void                        setDeblockingFilterDisable( bool b )                   { m_deblockingFilterDisable= b;                                 }
  void                        setDeblockingFilterOverrideFlag( bool b )              { m_deblockingFilterOverrideFlag = b;                           }
  void                        setDeblockingFilterBetaOffsetDiv2( int i )             { m_deblockingFilterBetaOffsetDiv2 = i;                         }
  void                        setDeblockingFilterTcOffsetDiv2( int i )               { m_deblockingFilterTcOffsetDiv2 = i;                           }
  void                        setDeblockingFilterCbBetaOffsetDiv2( int i )           { m_deblockingFilterCbBetaOffsetDiv2 = i;                         }
  void                        setDeblockingFilterCbTcOffsetDiv2( int i )             { m_deblockingFilterCbTcOffsetDiv2 = i;                           }
  void                        setDeblockingFilterCrBetaOffsetDiv2( int i )           { m_deblockingFilterCrBetaOffsetDiv2 = i;                         }
  void                        setDeblockingFilterCrTcOffsetDiv2( int i )             { m_deblockingFilterCrTcOffsetDiv2 = i;                           }
  void                        setDepQuantEnabledFlag( bool b )                       { m_depQuantEnabledFlag = b;                                                                   }
  bool                        getDepQuantEnabledFlag() const                         { return m_depQuantEnabledFlag;                                                                }  
  void                        setSignDataHidingEnabledFlag( bool b )                 { m_signDataHidingEnabledFlag = b;                                                             }
  bool                        getSignDataHidingEnabledFlag() const                   { return m_signDataHidingEnabledFlag;                                                          }  
  void                        setTSResidualCodingDisabledFlag(bool b)                { m_tsResidualCodingDisabledFlag = b; }
  bool                        getTSResidualCodingDisabledFlag() const                { return m_tsResidualCodingDisabledFlag; }
  void                        setLmcsEnabledFlag(bool b)                             { m_lmcsEnabledFlag = b;                                       }
  bool                        getLmcsEnabledFlag()                                   { return m_lmcsEnabledFlag;                                    }
  const bool                  getLmcsEnabledFlag() const                             { return m_lmcsEnabledFlag;                                    }

  void                        setExplicitScalingListUsed(bool b)                     { m_explicitScalingListUsed = b;                               }
  bool                        getExplicitScalingListUsed() const                     { return m_explicitScalingListUsed;                            }
  void                        setNumRefIdx( RefPicList e, int i )                    { m_aiNumRefIdx[e]    = i;                                      }
  void                        setPic( Picture* p )                                   { m_pcPic             = p;                                      }

  void                        constructRefPicLists( const PicList& rcPicList );
  void                        constructSingleRefPicList( const PicList& rcPicList, RefPicList listId );

  void                        setRefPOCList();

  void                        setColFromL0Flag( bool colFromL0 )                     { m_colFromL0Flag = colFromL0;                                  }
  void                        setColRefIdx( uint32_t refIdx)                         { m_colRefIdx = refIdx;                                         }
  void                        setCheckLDC( bool b )                                  { m_bCheckLDC = b;                                              }

  void                        setBiDirPred( bool b, int refIdx0, int refIdx1 ) { m_biDirPred = b; m_symRefIdx[0] = refIdx0; m_symRefIdx[1] = refIdx1; }
  bool                        getBiDirPred() const { return m_biDirPred; }
  int                         getSymRefIdx( int refList ) const { return m_symRefIdx[refList]; }

  bool                        isIntra() const                                        { return m_eSliceType == I_SLICE;                               }
  bool                        isInterB() const                                       { return m_eSliceType == B_SLICE;                               }
  bool                        isInterP() const                                       { return m_eSliceType == P_SLICE;                               }


  uint32_t                    getCuQpDeltaSubdiv() const                             { return this->isIntra() ? m_pcPicHeader->getCuQpDeltaSubdivIntra() : m_pcPicHeader->getCuQpDeltaSubdivInter(); }
  uint32_t                    getCuChromaQpOffsetSubdiv() const                      { return this->isIntra() ? m_pcPicHeader->getCuChromaQpOffsetSubdivIntra() : m_pcPicHeader->getCuChromaQpOffsetSubdivInter(); }

  void                        setList1IdxToList0Idx();

  uint32_t                    getTLayer() const                                      { return m_uiTLayer;                                            }
  void                        setTLayer( uint32_t uiTLayer )                         { m_uiTLayer = uiTLayer;                                        }

  void                        checkLeadingPictureRestrictions( const PicList& rcListPic ) const;
  bool                        checkThatAllRefPicsAreAvailable( const PicList& rcListPic,
                                                               const ReferencePictureList* pRPL,
                                                               int                         numActiveRefPics,
                                                               int*                        missingPOC,
                                                               int*                        missingRefPicIndex ) const;

  void                        setSliceMap( SliceMap map )                            { m_sliceMap = map;                                                         }
  uint32_t                    getFirstCtuRsAddrInSlice() const                       { return m_sliceMap.getCtuAddrInSlice(0);                                   }
  void                        setSliceID( uint32_t u )                               { m_sliceMap.setSliceID( u );                                               }
  uint32_t                    getSliceID() const                                     { return m_sliceMap.getSliceID();                                           }
  uint32_t                    getNumCtuInSlice() const                               { return m_sliceMap.getNumCtuInSlice();                                     }
  uint32_t                    getCtuAddrInSlice( int idx ) const                     { return m_sliceMap.getCtuAddrInSlice( idx );                               }
  void                        resetSliceMap()                                        { m_sliceMap.resetSliceMap();                                                }
  void                        addCtusToSlice( uint32_t startX, uint32_t stopX,
                                              uint32_t startY, uint32_t stopY,
                                              uint32_t picWidthInCtbsY )             { m_sliceMap.addCtusToSlice(startX, stopX, startY, stopY, picWidthInCtbsY); }
  void                        setIndependentSliceIdx( uint32_t i)                    { m_independentSliceIdx = i;                                    }
  uint32_t                    getIndependentSliceIdx() const                         { return  m_independentSliceIdx;                                }
  void                        copySliceInfo(Slice *pcSliceSrc, bool cpyAlmostAll = true);
  void                        setWpScaling( WPScalingParam wp[NUM_REF_PIC_LIST_01][MAX_NUM_REF][MAX_NUM_COMPONENT] )
  {
    memcpy( m_weightPredTable, wp, sizeof( WPScalingParam ) * NUM_REF_PIC_LIST_01 * MAX_NUM_REF * MAX_NUM_COMPONENT );
  }
  void                        getWpScaling( RefPicList e, int iRefIdx, const WPScalingParam*& wp ) const;
  void                        getWpScaling( RefPicList e, int iRefIdx,       WPScalingParam*& wp );

  void                        resetWpScaling();
  void                        initWpScaling(const SPS *sps);

  void                        setWpAcDcParam( WPACDCParam wp[MAX_NUM_COMPONENT] )    { memcpy(m_weightACDCParam, wp, sizeof(WPACDCParam)*MAX_NUM_COMPONENT); }

  void                        getWpAcDcParam( const WPACDCParam *&wp ) const;
  void                        initWpAcDcParam();

  void                        clearSubstreamSizes( )                                 { return m_substreamSizes.clear();                              }
  uint32_t                    getNumberOfSubstreamSizes( )                           { return (uint32_t) m_substreamSizes.size();                    }
  void                        addSubstreamSize( uint32_t size )                      { m_substreamSizes.push_back(size);                             }
  uint32_t                    getSubstreamSize( uint32_t idx )                       { CHECK(idx>=getNumberOfSubstreamSizes(),"Invalid index"); return m_substreamSizes[idx]; }
  void                        setSubstreamSizes( std::vector<uint32_t>&& offsets )   { m_substreamSizes = std::move( offsets ); }

  void                        setCabacInitFlag( bool val )                           { m_cabacInitFlag = val;                                        } //!< set CABAC initial flag
  bool                        getCabacInitFlag()                               const { return m_cabacInitFlag;                                       } //!< get CABAC initial flag
  void                        setSliceSubPicId(int i)                                { m_sliceSubPicId = i;   }
  uint32_t                    getSliceSubPicId() const                               { return m_sliceSubPicId; }

  void                        setDefaultClpRng( const SPS& sps );
  const ClpRngs&              clpRngs()                                         const { return m_clpRngs; }
  const ClpRng&               clpRng( ComponentID id)                           const { return m_clpRngs; }
  ClpRngs&                    getClpRngs()                                            { return m_clpRngs; }

  void                        resetAlfEnabledFlag()                          { memset(m_alfEnabledFlag, 0, sizeof(m_alfEnabledFlag)); }
  bool                        getAlfEnabledFlag(ComponentID compId)    const { return m_alfEnabledFlag[compId]; }
  void                        setAlfEnabledFlag(ComponentID compId, bool b)  { m_alfEnabledFlag[compId] = b; }
  int                         getNumAlfAps()                           const { return m_numAlfAps; }
  void                        setNumAlfAps(int i)                            { m_numAlfAps = i; }
  int                         getAlfApsIdChroma()                      const { return m_chromaAlfApsId; }
  void                        setAlfApsIdChroma( int i )                     { m_chromaAlfApsId = i; }
  const AlfApsIdVec&          getAlfApsIdsLuma()                       const { return m_lumaAlfApsId; }
  void                        setAlfApsIdsLuma( const AlfApsIdVec& apsIDs )  { m_lumaAlfApsId = apsIDs; }
  void                        resetCcAlfEnabledFlags()                       { m_ccAlfEnabledFlags[0] = false; m_ccAlfEnabledFlags[1] = false; }
  
  void                        setCcAlfCbEnabledFlag(bool b)                  { m_ccAlfEnabledFlags[0] = b; }
  void                        setCcAlfCrEnabledFlag(bool b)                  { m_ccAlfEnabledFlags[1] = b; }
  void                        setCcAlfCbApsId(int i)                         { m_ccAlfCbApsId = i; }
  void                        setCcAlfCrApsId(int i)                         { m_ccAlfCrApsId = i; }

  bool                        getCcAlfEnabledFlag(int cmpntIdx)       const  { return m_ccAlfEnabledFlags[cmpntIdx-1]; }

  bool                        getCcAlfCbEnabledFlag()                 const { return m_ccAlfEnabledFlags[0]; }
  bool                        getCcAlfCrEnabledFlag()                 const { return m_ccAlfEnabledFlags[1]; }
  int                         getCcAlfCbApsId()                       const { return m_ccAlfCbApsId; }
  int                         getCcAlfCrApsId()                       const { return m_ccAlfCrApsId; }

  void                        scaleRefPicList( const PicHeader* picHeader );
  const std::pair<int, int>&  getScalingRatio( const RefPicList refPicList, const int refIdx )  const { return m_scalingRatio[refPicList][refIdx]; }
  void                        setNumEntryPoints( const SPS *sps, const PPS *pps );
  uint32_t                    getNumEntryPoints( ) const { return m_numEntryPoints;  }

  struct ParseTaskParams
  {
    void init( class DecLibParser* dec, InputBitstream bits )
    {
      decLibParser = dec;
      bitstream    = std::move( bits );
    }

    class DecLibParser* decLibParser = nullptr;
    InputBitstream      bitstream;
  } parseTaskParams;

  Barrier parseDone;

private:
  Picture*              xGetRefPic        ( const PicList& rcListPic, int poc, const int layerId );
  Picture*              xGetLongTermRefPic( const PicList& rcListPic, int poc, bool pocHasMsb, const int layerId, const bool getCandidate = true );
};   // END CLASS DEFINITION Slice


class PreCalcValues
{
public:
  PreCalcValues( const SPS& sps, const PPS& pps )
    : chrFormat           ( sps.getChromaFormatIdc() )
    , maxCUWidth          ( sps.getMaxCUWidth() )
    , maxCUHeight         ( sps.getMaxCUHeight() )
    , maxCUWidthMask      ( maxCUWidth  - 1 )
    , maxCUHeightMask     ( maxCUHeight - 1 )
    , maxCUWidthLog2      ( getLog2( maxCUWidth  ) )
    , maxCUHeightLog2     ( getLog2( maxCUHeight ) )
    , minCUWidth          ( 1 << MIN_CU_LOG2 )
    , minCUHeight         ( 1 << MIN_CU_LOG2 )
    , minCUWidthLog2      ( getLog2( minCUWidth  ) )
    , minCUHeightLog2     ( getLog2( minCUHeight ) )
    , widthInCtus         ( (pps.getPicWidthInLumaSamples () + sps.getMaxCUWidth () - 1) / sps.getMaxCUWidth () )
    , heightInCtus        ( (pps.getPicHeightInLumaSamples() + sps.getMaxCUHeight() - 1) / sps.getMaxCUHeight() )
    , sizeInCtus          ( widthInCtus * heightInCtus )
    , lumaWidth           ( pps.getPicWidthInLumaSamples() )
    , lumaHeight          ( pps.getPicHeightInLumaSamples() )
    , num4x4CtuBlks       ( ( maxCUWidth * maxCUHeight ) >> ( MIN_CU_LOG2 << 1 ) )
    , num8x8CtuBlks       ( num4x4CtuBlks >> 2 )
  {}

  bool isCorrect( const SPS& sps, const PPS& pps )
  {
    if( chrFormat != sps.getChromaFormatIdc() )
      return false;
    if( maxCUWidth != sps.getMaxCUWidth() )
      return false;
    if( maxCUHeight != sps.getMaxCUHeight() )
      return false;
    if( chrFormat != sps.getChromaFormatIdc() )
      return false;
    if( lumaWidth != pps.getPicWidthInLumaSamples() )
      return false;
    if( lumaHeight != pps.getPicHeightInLumaSamples() )
      return false;
    return true;
  }

  const ChromaFormat chrFormat;
  const unsigned     maxCUWidth;
  const unsigned     maxCUHeight;
  // to get CTU position, use (x & maxCUWidthMask) rather than (x % maxCUWidth)
  const unsigned     maxCUWidthMask;
  const unsigned     maxCUHeightMask;
  const unsigned     maxCUWidthLog2;
  const unsigned     maxCUHeightLog2;
  const unsigned     minCUWidth;
  const unsigned     minCUHeight;
  const unsigned     minCUWidthLog2;
  const unsigned     minCUHeightLog2;
  const unsigned     widthInCtus;
  const unsigned     heightInCtus;
  const unsigned     sizeInCtus;
  const unsigned     lumaWidth;
  const unsigned     lumaHeight;
  const unsigned     num4x4CtuBlks;
  const unsigned     num8x8CtuBlks;
};

struct LevelTierFeatures
{
  vvdecLevel  level                            = VVDEC_LEVEL_NONE;
  uint32_t    maxLumaPs                        = 0;
  uint32_t    maxCpb[Tier::NUMBER_OF_TIERS]    = { 0 };   // in units of CpbVclFactor or CpbNalFactor bits
  uint32_t    maxSlicesPerAu                   = 0;
  uint32_t    maxTilesPerAu                    = 0;
  uint32_t    maxTileCols                      = 0;
  uint64_t    maxLumaSr                        = 0;
  uint32_t    maxBr[Tier::NUMBER_OF_TIERS]     = { 0 };   // in units of BrVclFactor or BrNalFactor bits/s
  uint32_t    minCrBase[Tier::NUMBER_OF_TIERS] = { 0 };
  uint32_t    getMaxPicWidthInLumaSamples()  const;
  uint32_t    getMaxPicHeightInLumaSamples() const;
};


struct ProfileFeatures
{
  Profile::Name            profile         = Profile::NONE;
  const char*              pNameString     = 0;
  uint32_t                 maxBitDepth     = 0;
  ChromaFormat             maxChromaFormat = CHROMA_400;

  bool                     canUseLevel15p5             = 0;
  uint32_t                 cpbVclFactor                = 0;
  uint32_t                 cpbNalFactor                = 0;
  uint32_t                 formatCapabilityFactorx1000 = 0;
  uint32_t                 minCrScaleFactorx100        = 0;
  const LevelTierFeatures* pLevelTiersListInfo         = nullptr;
  bool                     onePictureOnlyFlagMustBe1   = 0;

  static const ProfileFeatures *getProfileFeatures(const Profile::Name p);
};

class ProfileLevelTierFeatures
{
  private:
    const ProfileFeatures*   m_pProfile   = nullptr;
    const LevelTierFeatures* m_pLevelTier = nullptr;
    Tier                     m_tier       = Tier::MAIN;

  public:
    void extractPTLInformation(const SPS &sps);

    const ProfileFeatures     *getProfileFeatures()   const { return m_pProfile; }
    const LevelTierFeatures   *getLevelTierFeatures() const { return m_pLevelTier; }
    Tier                       getTier()              const { return m_tier; }
    uint64_t                   getCpbSizeInBits()     const;
    uint32_t                   getMaxDpbSize( uint32_t picSizeMaxInSamplesY ) const;
};


#if ENABLE_TRACING
void xTraceVPSHeader();
void xTraceDCIHeader();
void xTraceSPSHeader();
void xTracePPSHeader();
void xTraceAPSHeader();
void xTracePictureHeader();
void xTraceSliceHeader();
void xTraceAccessUnitDelimiter();
#endif

}
