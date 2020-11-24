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

/** \file     UnitTool.h
 *  \brief    defines operations for basic units
 */

#ifndef __UNITTOOLS__
#define __UNITTOOLS__

#include "Unit.h"
#include "UnitPartitioner.h"
#include "ContextModelling.h"
#include "InterPrediction.h"

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
#if JVET_O1143_SUBPIC_BOUNDARY
  bool isSameSubPic                   (const CodingUnit &cu, const CodingUnit &cu2);
#endif
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
  bool isDiffMER                      (const PredictionUnit &pu, const PredictionUnit &pu2);
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
#endif
