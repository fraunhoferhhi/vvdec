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

/** \file     UnitTool.h
 *  \brief    defines operations for basic units
 */

#pragma once

#include "Unit.h"
#include "UnitPartitioner.h"
#include "ContextModelling.h"
#include "InterPrediction.h"

namespace vvdec
{

UnitArea getArea( const Slice &slice, const UnitArea &area, const ChannelType chType, const TreeType treeType = TREE_D );

// CU tools
namespace CU
{
  static inline bool isIntra          (const CodingUnit &cu) { return cu.predMode() == MODE_INTRA; }
  static inline bool isInter          (const CodingUnit &cu) { return cu.predMode() == MODE_INTER; }
  static inline bool isIBC            (const CodingUnit &cu) { return cu.predMode() == MODE_IBC;   }

  static inline bool isSepTree        (const CodingUnit &cu) { return cu.treeType() != TREE_D;     }
  static inline bool isConsInter      (const CodingUnit &cu) { return cu.modeType() == MODE_TYPE_INTER; }
  static inline bool isConsIntra      (const CodingUnit &cu) { return cu.modeType() == MODE_TYPE_INTRA; }

  bool     isDualITree                (const CodingUnit &cu);

  bool isSameCtu                      (const CodingUnit &cu, const CodingUnit &cu2);
  bool isSameSlice                    (const CodingUnit &cu, const CodingUnit &cu2);
  bool isSameTile                     (const CodingUnit &cu, const CodingUnit &cu2);
  bool isSameSliceAndTile             (const CodingUnit &cu, const CodingUnit &cu2);
  bool isSameSubPic                   (const CodingUnit &cu, const CodingUnit &cu2);
  bool isAvailable                    (const CodingUnit &cu, const CodingUnit &cu2, const bool bEnforceSliceRestriction, const bool bEnforceTileRestriction, const bool bEnforceSubPicRestriction);

  uint32_t getCtuAddr                 (const CodingUnit &cu);

  int  predictQP                      (const CodingUnit& cu, const int prevQP );
  bool isMIP                          (const CodingUnit &cu, const ChannelType &chType = CHANNEL_TYPE_LUMA);
  bool isPredRegDiffFromTB            (const CodingUnit& cu, const ComponentID compID);
  bool isFirstTBInPredReg             (const CodingUnit& cu, const ComponentID compID, const CompArea &area);
  void adjustPredArea                 (CompArea &area);
  bool  isBcwIdxCoded                 (const CodingUnit& cu);
  void  setBcwIdx                     (CodingUnit& cu, uint8_t uh);
  bool bdpcmAllowed                   (const CodingUnit& cu, const ComponentID compID);
  bool isMTSAllowed                   (const CodingUnit& cu, const ComponentID compID);


  bool      divideTuInRows            ( const CodingUnit &cu );
  PartSplit getISPType                ( const CodingUnit &cu,                         const ComponentID compID );
  ISPType   canUseISPSplit            ( const CodingUnit &cu,                         const ComponentID compID );
  ISPType   canUseISPSplit            ( const int width, const int height, const int maxTrSize = MAX_TB_SIZEY );
  bool      canUseLfnstWithISP        ( const CompArea& cuArea, const ISPType ispSplitType );
  bool      canUseLfnstWithISP        ( const CodingUnit& cu, const ChannelType chType );
  uint32_t  getISPSplitDim            ( const int width, const int height, const PartSplit ispType );

  bool  hasSubCUNonZeroMVd            (const CodingUnit& cu);
  bool  hasSubCUNonZeroAffineMVd      (const CodingUnit& cu);

  uint8_t getSbtIdx                   (const uint8_t sbtInfo);
  uint8_t getSbtPos                   (const uint8_t sbtInfo);
  uint8_t targetSbtAllowed            (uint8_t idx, uint8_t sbtAllowed);


  uint8_t getSbtIdx                   (const CodingUnit& cu);
  uint8_t getSbtPos                   (const CodingUnit& cu);
  void    setSbtIdx                   (      CodingUnit& cu, uint8_t idx);
  void    setSbtPos                   (      CodingUnit& cu, uint8_t pos);
  uint8_t getSbtTuSplit               (const CodingUnit& cu);
  uint8_t checkAllowedSbt             (const CodingUnit& cu);
  PartSplit getSplitAtDepth           (const CodingUnit& cu, const unsigned depth);
  bool    checkCCLMAllowed            (const CodingUnit &cu);
  bool    getRprScaling               ( const SPS* sps, const PPS* curPPS, const PPS* refPPS, int& xScale, int& yScale );
#if JVET_R0058
  void    checkConformanceILRP        (Slice *slice);
#endif
}
// PU tools
namespace PU
{
  int  getLMSymbolList                (const PredictionUnit &pu, int *pModeList);
  int  getIntraMPMs                   (const PredictionUnit &pu, unsigned *mpm, const ChannelType &channelType = CHANNEL_TYPE_LUMA);
#if JVET_R0350_MIP_CHROMA_444_SINGLETREE
  bool          isDMChromaMIP         (const PredictionUnit &pu);
#endif
  int           getMipSizeId          (const PredictionUnit &pu);
  uint32_t      getIntraDirLuma       (const PredictionUnit &pu);
  void getIntraChromaCandModes        (const PredictionUnit &pu, unsigned modeList[NUM_CHROMA_MODE]);
#if JVET_R0350_MIP_CHROMA_444_SINGLETREE
  const PredictionUnit &getCoLocatedLumaPU(const PredictionUnit &pu);
#endif
  uint32_t getFinalIntraMode          (const PredictionUnit &pu, const ChannelType &chType);
  uint32_t getCoLocatedIntraLumaMode  (const PredictionUnit &pu);
  int getWideAngIntraMode             ( const TransformUnit &tu, const uint32_t dirMode, const ComponentID compID );
  void getInterMergeCandidates        (const PredictionUnit &pu, MergeCtx& mrgCtx, MotionHist &hist, const int& mrgCandIdx = -1);
  void getIBCMergeCandidates          (const PredictionUnit &pu, MergeCtx& mrgCtx, MotionHist &hist, const int& mrgCandIdx = -1);
  void getInterMMVDMergeCandidates    (const PredictionUnit &pu, MergeCtx& mrgCtx, const int& mrgCandIdx = -1);
  int getDistScaleFactor              (const int &currPOC, const int &currRefPOC, const int &colPOC, const int &colRefPOC);
  bool isDiffMER                      (const Position &pos1, const Position &pos2, const unsigned plevel);
  bool getColocatedMVP                (const PredictionUnit &pu, const RefPicList &eRefPicList, const Position &pos, Mv& rcMv, const int &refIdx);
  void fillMvpCand                    (      PredictionUnit &pu, const RefPicList &eRefPicList, const int &refIdx, AMVPInfo &amvpInfo, MotionHist& hist);
  void fillIBCMvpCand                 (      PredictionUnit &pu, AMVPInfo &amvpInfo, MotionHist& hist);
  void fillAffineMvpCand              (      PredictionUnit &pu, const RefPicList &eRefPicList, const int &refIdx, AffineAMVPInfo &affiAMVPInfo);
  bool addMVPCandUnscaled             (const PredictionUnit &pu, const RefPicList &eRefPicList, const int &iRefIdx, const Position &pos, const MvpDir &eDir, AMVPInfo &amvpInfo);
  void xInheritedAffineMv             (const PredictionUnit &pu, const PredictionUnit* puNeighbour, RefPicList eRefPicList, Mv rcMv[3] );
  bool xCheckSimilarMotion            (const int mergeCandIndex, const int prevCnt, const MergeCtx& mergeCandList, bool hasPruned[MRG_MAX_NUM_CANDS]);
  bool addMergeHMVPCand               (const CodingStructure &cs, MergeCtx& mrgCtx, MotionHist& hist, bool canFastExit, const int& mrgCandIdx, const uint32_t maxNumMergeCandMin1, int &cnt, const int prevCnt, bool isAvailableSubPu, unsigned subPuMvpPos, bool ibcFlag, bool isGt4x4, bool isInterB  );
  void addAMVPHMVPCand                (const PredictionUnit &pu, MotionHist& hist, const RefPicList eRefPicList, const RefPicList eRefPicList2nd, const int currRefPOC, AMVPInfo &info, uint8_t imv);
  bool addAffineMVPCandUnscaled       (const PredictionUnit &pu, const RefPicList &refPicList, const int &refIdx, const Position &pos, const MvpDir &dir, AffineAMVPInfo &affiAmvpInfo );
  bool isBipredRestriction            (const PredictionUnit &pu);
  void spanMotionInfo                 (      PredictionUnit &pu, const MergeCtx &mrgCtx = MergeCtx() );
  void applyImv                       (      PredictionUnit &pu, MotionHist& hist);
  void getAffineControlPointCand      (const PredictionUnit &pu, MotionInfo mi[4], bool isAvailable[4], int verIdx[4], int8_t gbiIdx, int modelIdx, int verNum, AffineMergeCtx& affMrgCtx);
  void getAffineMergeCand             (const PredictionUnit &pu, AffineMergeCtx& affMrgCtx, const int mrgCandIdx = -1 );
  void setAllAffineMvField            (      PredictionUnit &pu, MvField *mvField, RefPicList eRefList );
  void setAllAffineMv                 (      PredictionUnit &pu, Mv affLT, Mv affRT, Mv affLB, RefPicList eRefList, bool clipCPMVs = false );
  bool getInterMergeSubPuMvpCand      (const PredictionUnit &pu, MergeCtx &mrgCtx, bool& LICFlag, const int count);
  bool isBiPredFromDifferentDirEqDistPoc
                                      (const PredictionUnit &pu);
  void restrictBiPredMergeCandsOne    (      PredictionUnit &pu);

  bool isLMCMode                      (                          unsigned mode);
  void getGeoMergeCandidates          (const PredictionUnit &pu, MergeCtx &GeoMrgCtx, MotionHist& hist);
  void spanGeoMotionInfo              (      PredictionUnit &pu, MergeCtx &GeoMrgCtx, const uint8_t splitDir, const uint8_t candIdx0, const uint8_t candIdx1);
  bool checkDMVRCondition             (const PredictionUnit& pu);

  bool isRefPicSameSize               (const PredictionUnit& pu);
}

// TU tools
namespace TU
{
  bool getCbf                         (const TransformUnit &tu, const ComponentID &compID);
  void setCbf                         (      TransformUnit &tu, const ComponentID &compID, const bool &cbf);
  bool isTSAllowed                    (const TransformUnit &tu, const ComponentID  compID);
  bool checkTuNoResidual              (      TransformUnit &tu, unsigned idx );

  bool needsSqrt2Scale                (const TransformUnit &tu, const ComponentID &compID);
  bool needsBlockSizeTrafoScale       (const TransformUnit &tu, const ComponentID &compID);
  const TransformUnit* getPrevTU      (const TransformUnit &tu, const ComponentID  compID);
  bool getPrevTUCbf                   (const TransformUnit &tu, const ComponentID  compID);
  int getICTMode                      (const TransformUnit &tu, bool sign);
  int getTbAreaAfterCoefZeroOut       (const TransformUnit &tu, const ComponentID compID);
}

uint32_t getCtuAddr        (const Position& pos, const PreCalcValues &pcv);
UnitArea getCtuArea        (const CodingStructure& cs, unsigned col, unsigned line, bool clipToPic = false );
UnitArea getLineArea       (const CodingStructure& cs, unsigned line, bool clipToPic = false );

int  getNumModesMip   (const Size& block);
int getMipSizeId      (const Size& block);
bool allowLfnstWithMip(const Size& block);

bool isCrossedByVirtualBoundaries( const PicHeader* picHeader,
                                   const Area&      area,
                                   int&             numHorVirBndry,
                                   int&             numVerVirBndry,
                                   int              horVirBndryPos[],
                                   int              verVirBndryPos[] );

}
