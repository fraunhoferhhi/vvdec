/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2018-2022, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVdeC Authors.
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

/** \file     UnitTool.cpp
 *  \brief    defines operations for basic units
 */

#include "UnitTools.h"

#include "dtrace_next.h"

#include "Unit.h"
#include "Slice.h"
#include "Picture.h"

#include <utility>
#include <algorithm>

#if defined( TARGET_SIMD_X86 )
#include "CommonDefX86.h"
#include <simde/x86/sse4.2.h>
#endif

namespace vvdec
{

static bool isDualITree( const Slice &slice )
{
#if GDR_ADJ
  return slice.isIntra() && slice.getSPS()->getUseDualITree();
#else
  return slice.isIRAP() && slice.getSPS()->getUseDualITree();
#endif
}


bool CU::isDualITree( const CodingUnit &cu )
{
#if GDR_ADJ
  return cu.slice->isIntra() && cu.sps->getUseDualITree();
#else
  return cu.slice->isIRAP() && cu.sps->getUseDualITree();
#endif
}

UnitArea getArea( const Slice &slice, const UnitArea &area, const ChannelType chType, const TreeType treeType )
{
  return isDualITree( slice ) || treeType != TREE_D ? area.singleChan( chType ) : area;
}

// CU tools

bool CU::getRprScaling( const SPS* sps, const PPS* curPPS, const PPS* refPPS, int& xScale, int& yScale )
{
  const Window& curScalingWindow = curPPS->getScalingWindow();
  int curPicWidth = curPPS->getPicWidthInLumaSamples() - (curScalingWindow.getWindowLeftOffset() + curScalingWindow.getWindowRightOffset()) * SPS::getWinUnitX(sps->getChromaFormatIdc());
  int curPicHeight = curPPS->getPicHeightInLumaSamples() - (curScalingWindow.getWindowTopOffset() + curScalingWindow.getWindowBottomOffset()) * SPS::getWinUnitY(sps->getChromaFormatIdc());
  const Window& refScalingWindow = refPPS->getScalingWindow();
  int refPicWidth = refPPS->getPicWidthInLumaSamples() - (refScalingWindow.getWindowLeftOffset() + refScalingWindow.getWindowRightOffset()) * SPS::getWinUnitX(sps->getChromaFormatIdc());
  int refPicHeight = refPPS->getPicHeightInLumaSamples() - (refScalingWindow.getWindowTopOffset() + refScalingWindow.getWindowBottomOffset()) * SPS::getWinUnitY(sps->getChromaFormatIdc());

  xScale = ( ( refPicWidth << SCALE_RATIO_BITS ) + ( curPicWidth >> 1 ) ) / curPicWidth;
  yScale = ( ( refPicHeight << SCALE_RATIO_BITS ) + ( curPicHeight >> 1 ) ) / curPicHeight;

  int curSeqMaxPicWidthY = sps->getMaxPicWidthInLumaSamples();                  // pic_width_max_in_luma_samples
  int curSeqMaxPicHeightY = sps->getMaxPicHeightInLumaSamples();                // pic_height_max_in_luma_samples
  int curPicWidthY = curPPS->getPicWidthInLumaSamples();                        // pic_width_in_luma_samples
  int curPicHeightY = curPPS->getPicHeightInLumaSamples();                      // pic_height_in_luma_samples
  int max8MinCbSizeY = std::max((int)8, (1<<sps->getLog2MinCodingBlockSize())); // Max(8, MinCbSizeY)

  CHECK((curPicWidth * curSeqMaxPicWidthY) < refPicWidth * (curPicWidthY - max8MinCbSizeY), "(curPicWidth * curSeqMaxPicWidthY) should be greater than or equal to refPicWidth * (curPicWidthY - max8MinCbSizeY))");
  CHECK((curPicHeight * curSeqMaxPicHeightY) < refPicHeight * (curPicHeightY - max8MinCbSizeY), "(curPicHeight * curSeqMaxPicHeightY) should be greater than or equal to refPicHeight * (curPicHeightY - max8MinCbSizeY))");

  CHECK(curPicWidth * 2 < refPicWidth, "curPicWidth * 2 shall be greater than or equal to refPicWidth");
  CHECK(curPicHeight * 2 < refPicHeight, "curPicHeight * 2 shall be greater than or equal to refPicHeight");
  CHECK(curPicWidth > refPicWidth * 8, "curPicWidth shall be less than or equal to refPicWidth * 8");
  CHECK(curPicHeight > refPicHeight * 8, "curPicHeight shall be less than or equal to refPicHeight * 8");

  int subWidthC = SPS::getWinUnitX(sps->getChromaFormatIdc());
  int subHeightC = SPS::getWinUnitY(sps->getChromaFormatIdc());

  CHECK(subWidthC * curScalingWindow.getWindowLeftOffset() < (-curPicWidthY) * 15, "The value of SubWidthC * pps_scaling_win_left_offset shall be greater than or equal to -pps_pic_width_in_luma_samples * 15");
  CHECK(subWidthC * curScalingWindow.getWindowLeftOffset() >= curPicWidthY, "The value of SubWidthC * pps_scaling_win_left_offset shall be less than pic_width_in_luma_samples");
  CHECK(subWidthC * curScalingWindow.getWindowRightOffset() < (-curPicWidthY) * 15, "The value of SubWidthC * pps_scaling_win_right_offset shall be greater than or equal to -pps_pic_width_in_luma_samples * 15");
  CHECK(subWidthC * curScalingWindow.getWindowRightOffset() >= curPicWidthY, "The value of SubWidthC * pps_scaling_win_right_offset shall be less than pic_width_in_luma_samples");

  CHECK(subHeightC * curScalingWindow.getWindowTopOffset() < (-curPicHeightY) * 15, "The value of SubHeightC * pps_scaling_win_top_offset shall be greater than or equal to -pps_pic_height_in_luma_samples * 15");
  CHECK(subHeightC * curScalingWindow.getWindowTopOffset() >= curPicHeightY, "The value of SubHeightC * pps_scaling_win_top_offset shall be less than pps_pic_height_in_luma_samples");
  CHECK(subHeightC * curScalingWindow.getWindowBottomOffset() < (-curPicHeightY) * 15, "The value of SubHeightC *pps_scaling_win_bottom_offset shall be greater than or equal to -pps_pic_height_in_luma_samples * 15");
  CHECK(subHeightC * curScalingWindow.getWindowBottomOffset() >= curPicHeightY, "The value of SubHeightC *pps_scaling_win_bottom_offset shall be less than pps_pic_height_in_luma_samples");

  CHECK(subWidthC * (curScalingWindow.getWindowLeftOffset() + curScalingWindow.getWindowRightOffset()) < (-curPicWidthY) * 15, "The value of SubWidthC * ( pps_scaling_win_left_offset + pps_scaling_win_right_offset ) shall be greater than or equal to -pps_pic_width_in_luma_samples * 15");
  CHECK(subWidthC * (curScalingWindow.getWindowLeftOffset() + curScalingWindow.getWindowRightOffset()) >= curPicWidthY, "The value of SubWidthC * ( pps_scaling_win_left_offset + pps_scaling_win_right_offset ) shall be less than pic_width_in_luma_samples");
  CHECK(subHeightC * (curScalingWindow.getWindowTopOffset() + curScalingWindow.getWindowBottomOffset()) < (-curPicHeightY) * 15, "The value of SubHeightC * ( pps_scaling_win_top_offset + pps_scaling_win_bottom_offset ) shall be greater than or equal to -pps_pic_height_in_luma_samples * 15");
  CHECK(subHeightC * (curScalingWindow.getWindowTopOffset() + curScalingWindow.getWindowBottomOffset()) >= curPicHeightY, "The value of SubHeightC * ( pps_scaling_win_top_offset + pps_scaling_win_bottom_offset ) shall be less than pic_height_in_luma_samples");

  return false; // return whatever, because it's not used... to be changed
}

void CU::checkConformanceILRP(Slice *slice)
{
  const int numRefList = (slice->getSliceType() == B_SLICE) ? (2) : (1);

  int currentSubPicIdx = NOT_VALID;

  // derive sub-picture index for the current slice
  for( int subPicIdx = 0; subPicIdx < slice->getPic()->cs->sps->getNumSubPics(); subPicIdx++ )
  {
    if( slice->getPic()->cs->pps->getSubPic( subPicIdx ).getSubPicID() == slice->getSliceSubPicId() )
    {
      currentSubPicIdx = subPicIdx;
      break;
    }
  }

  CHECK( currentSubPicIdx == NOT_VALID, "Sub-picture was not found" );

  if( !slice->getPic()->cs->sps->getSubPicTreatedAsPicFlag( currentSubPicIdx ) )
  {
    return;
  }

  //constraint 1: The picture referred to by each active entry in RefPicList[ 0 ] or RefPicList[ 1 ] has the same subpicture layout as the current picture
  bool isAllRefSameSubpicLayout = true;
  for (int refList = 0; refList < numRefList; refList++) // loop over l0 and l1
  {
    RefPicList  eRefPicList = (refList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
    for (int refIdx = 0; refIdx < slice->getNumRefIdx(eRefPicList); refIdx++)
    {
      const Picture* refPic = slice->getRefPic( eRefPicList, refIdx );

      if( refPic->subPictures.size() != slice->getPic()->cs->pps->getNumSubPics() )
      {
        isAllRefSameSubpicLayout = false;
        refList = numRefList;
        break;
      }
      else
      {
        for( int i = 0; i < refPic->subPictures.size(); i++ )
        {
          const SubPic& refSubPic = refPic->subPictures[i];
          const SubPic& curSubPic = slice->getPic()->cs->pps->getSubPic( i );

          if( refSubPic.getSubPicWidthInCTUs() != curSubPic.getSubPicWidthInCTUs()
            || refSubPic.getSubPicHeightInCTUs() != curSubPic.getSubPicHeightInCTUs()
            || refSubPic.getSubPicCtuTopLeftX() != curSubPic.getSubPicCtuTopLeftX()
            || refSubPic.getSubPicCtuTopLeftY() != curSubPic.getSubPicCtuTopLeftY()
            || ( refPic->layerId != slice->getPic()->layerId && refSubPic.getSubPicID() != curSubPic.getSubPicID() )
            || refSubPic.getTreatedAsPicFlag() != curSubPic.getTreatedAsPicFlag())
          {
            isAllRefSameSubpicLayout = false;
            refIdx = slice->getNumRefIdx(eRefPicList);
            refList = numRefList;
            break;
          }
        }

        // A picture with different sub-picture ID of the collocated sub-picture cannot be used as an active reference picture in the same layer
        if( refPic->layerId == slice->getPic()->layerId )
        {
          isAllRefSameSubpicLayout = isAllRefSameSubpicLayout && refPic->subPictures[currentSubPicIdx].getSubPicID() == slice->getSliceSubPicId();
        }
      }
    }
  }

  //constraint 2: The picture referred to by each active entry in RefPicList[ 0 ] or RefPicList[ 1 ] is an ILRP for which the value of sps_num_subpics_minus1 is equal to 0
  if (!isAllRefSameSubpicLayout)
  {
    for (int refList = 0; refList < numRefList; refList++) // loop over l0 and l1
    {
      RefPicList  eRefPicList = (refList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
      for (int refIdx = 0; refIdx < slice->getNumRefIdx(eRefPicList); refIdx++)
      {
        const Picture* refPic = slice->getRefPic( eRefPicList, refIdx );
        CHECK( refPic->layerId == slice->getPic()->layerId || refPic->subPictures.size() > 1, "The inter-layer reference shall contain a single subpicture or have same subpicture layout with the current picture" );
      }
    }
  }

  return;
}

bool CU::isSameSlice(const CodingUnit& cu, const CodingUnit& cu2)
{
  return cu.slice->getIndependentSliceIdx() == cu2.slice->getIndependentSliceIdx();
}

bool CU::isSameTile(const CodingUnit& cu, const CodingUnit& cu2)
{
  return cu.tileIdx == cu2.tileIdx;
}

bool CU::isSameSliceAndTile(const CodingUnit& cu, const CodingUnit& cu2)
{
  return isSameSlice(cu, cu2) && isSameTile(cu, cu2);
}

bool CU::isSameSubPic(const CodingUnit& cu, const CodingUnit& cu2)
{
  return (cu.pps->getSubPicFromCU(cu).getSubPicIdx() == cu2.pps->getSubPicFromCU(cu2).getSubPicIdx()) ;
}

bool CU::isSameCtu(const CodingUnit& cu, const CodingUnit& cu2)
{
  uint32_t ctuSizeBit = getLog2(cu.sps->getMaxCUWidth());

  Position pos1Ctu(cu.lumaPos().x  >> ctuSizeBit, cu.lumaPos().y  >> ctuSizeBit);
  Position pos2Ctu(cu2.lumaPos().x >> ctuSizeBit, cu2.lumaPos().y >> ctuSizeBit);

  return pos1Ctu.x == pos2Ctu.x && pos1Ctu.y == pos2Ctu.y;
}

bool CU::isAvailable( const CodingUnit& cu, const CodingUnit& cu2, const bool bEnforceSliceRestriction, const bool bEnforceTileRestriction, const bool bEnforceSubPicRestriction )
{
  return ( !bEnforceSliceRestriction || CU::isSameSlice( cu, cu2 ) )
         && ( !bEnforceTileRestriction || CU::isSameTile( cu, cu2 ) )
         && ( !bEnforceSubPicRestriction || CU::isSameSubPic( cu, cu2 ) );
}

uint32_t CU::getCtuAddr( const CodingUnit &cu )
{
  return getCtuAddr( cu.blocks[cu.chType()].lumaPos( cu.chromaFormat ), *cu.cs->pcv );
}

int CU::predictQP( const CodingUnit& cu, const int prevQP )
{
  const ChannelType      chType  = cu.chType();
  const CodingStructure& cs      = *cu.cs;
  const CodingUnit*      cuAbove = cs.getCU( cu.blocks[chType].pos().offset( 0, -1 ), chType );
  const CodingUnit*      cuLeft  = cs.getCU( cu.blocks[chType].pos().offset( -1, 0 ), chType );

  uint32_t  ctuRsAddr       = getCtuAddr( cu );
  uint32_t  ctuXPosInCtus   = ctuRsAddr % cs.pcv->widthInCtus;
  uint32_t  tileColIdx      = cu.pps->ctuToTileCol( ctuXPosInCtus );
  uint32_t  tileXPosInCtus  = cu.pps->getTileColumnBd( tileColIdx );
  if( ctuXPosInCtus == tileXPosInCtus &&
      !( cu.blocks[chType].x & ( cs.pcv->maxCUWidthMask  >> getChannelTypeScaleX( chType, cu.chromaFormat ) ) ) &&
      !( cu.blocks[chType].y & ( cs.pcv->maxCUHeightMask >> getChannelTypeScaleY( chType, cu.chromaFormat ) ) ) &&
       cuAbove != nullptr && CU::isSameSliceAndTile( *cuAbove, cu ) )
  {
    return cuAbove->qp;
  }
  else
  {
    const int a = ( cu.blocks[chType].y & ( cs.pcv->maxCUHeightMask >> getChannelTypeScaleY( chType, cu.chromaFormat ) ) ) ? cuAbove->qp : prevQP;
    const int b = ( cu.blocks[chType].x & ( cs.pcv->maxCUWidthMask  >> getChannelTypeScaleX( chType, cu.chromaFormat ) ) ) ? cuLeft->qp  : prevQP;

    return ( a + b + 1 ) >> 1;
  }
}

bool CU::divideTuInRows( const CodingUnit &cu )
{
  CHECK( cu.ispMode() != HOR_INTRA_SUBPARTITIONS && cu.ispMode() != VER_INTRA_SUBPARTITIONS, "Intra Subpartitions type not recognized!" );
  return cu.ispMode() == HOR_INTRA_SUBPARTITIONS ? true : false;
}

PartSplit CU::getISPType( const CodingUnit &cu, const ComponentID compID )
{
  if( cu.ispMode() && isLuma( compID ) )
  {
    const bool tuIsDividedInRows = CU::divideTuInRows( cu );

    return tuIsDividedInRows ? TU_1D_HORZ_SPLIT : TU_1D_VERT_SPLIT;
  }
  return TU_NO_ISP;
}

ISPType CU::canUseISPSplit( const CodingUnit &cu, const ComponentID compID )
{
  const int width     = cu.blocks[compID].width;
  const int height    = cu.blocks[compID].height;
  const int maxTrSize = cu.sps->getMaxTbSize();

  return CU::canUseISPSplit( width, height, maxTrSize );
}

bool CU::canUseLfnstWithISP( const CompArea& cuArea, const ISPType ispSplitType )
{
  if( ispSplitType == NOT_INTRA_SUBPARTITIONS )
  {
    return false;
  }

  const Size tuSize = ( ispSplitType == HOR_INTRA_SUBPARTITIONS )
    ? Size( cuArea.width, CU::getISPSplitDim( cuArea.width, cuArea.height, TU_1D_HORZ_SPLIT ) )
    : Size( CU::getISPSplitDim( cuArea.width, cuArea.height, TU_1D_VERT_SPLIT ), cuArea.height );

  if( !( tuSize.width >= MIN_TB_SIZEY && tuSize.height >= MIN_TB_SIZEY ) )
  {
    return false;
  }
  return true;
}

bool CU::canUseLfnstWithISP( const CodingUnit& cu, const ChannelType chType )
{
  CHECK( !isLuma( chType ), "Wrong ISP mode!" );
  return CU::canUseLfnstWithISP( cu.blocks[chType == CHANNEL_TYPE_LUMA ? 0 : 1], (ISPType)cu.ispMode() );
}

ISPType CU::canUseISPSplit( const int width, const int height, const int maxTrSize )
{
  const uint32_t minTuSizeForISP  = MIN_TB_SIZEY;
  bool  notEnoughSamplesToSplit   = ( getLog2( width ) + getLog2( height ) <= ( getLog2( minTuSizeForISP ) << 1 ) );
  bool  cuSizeLargerThanMaxTrSize = width  > maxTrSize || height > maxTrSize;

  int   widthCanBeUsed  = ( !cuSizeLargerThanMaxTrSize && !notEnoughSamplesToSplit ) ? 4 : 2;
  int   heightCanBeUsed = ( !cuSizeLargerThanMaxTrSize && !notEnoughSamplesToSplit ) ? 0 : 2;

  return ISPType( widthCanBeUsed >> heightCanBeUsed );

  //  widthCanBeUsed &&  heightCanBeUsed -> 4
  // !widthCanBeUsed &&  heightCanBeUsed -> 2
  //  widthCanBeUsed && !heightCanBeUsed -> 1
  // !widthCanBeUsed && !heightCanBeUsed -> 0
}

uint32_t CU::getISPSplitDim( const int width, const int height, const PartSplit ispType )
{
  bool divideTuInRows = ispType == TU_1D_HORZ_SPLIT;
  uint32_t splitDimensionSize, nonSplitDimensionSize, partitionSize, divShift = 2;

  if( divideTuInRows )
  {
    splitDimensionSize    = height;
    nonSplitDimensionSize = width;
  }
  else
  {
    splitDimensionSize    = width;
    nonSplitDimensionSize = height;
  }

  const int minNumberOfSamplesPerCu = 1 << ( ( getLog2(MIN_TB_SIZEY) << 1 ) );
  const int factorToMinSamples = nonSplitDimensionSize < minNumberOfSamplesPerCu ? minNumberOfSamplesPerCu >> getLog2(nonSplitDimensionSize) : 1;
  partitionSize = ( splitDimensionSize >> divShift ) < factorToMinSamples ? factorToMinSamples : ( splitDimensionSize >> divShift );

  CHECK( getLog2(partitionSize) + getLog2(nonSplitDimensionSize) < getLog2(minNumberOfSamplesPerCu), "A partition has less than the minimum amount of samples!" );
  return partitionSize;
}

// PU tools

int PU::getIntraMPMs( const PredictionUnit &pu, unsigned* mpm, const ChannelType &channelType /*= CHANNEL_TYPE_LUMA*/ )
{
  const int numMPMs = NUM_MOST_PROBABLE_MODES;
  {
    CHECK(channelType != CHANNEL_TYPE_LUMA, "Not harmonized yet");
    int numCand      = -1;
    int leftIntraDir = PLANAR_IDX, aboveIntraDir = PLANAR_IDX;

    const CompArea &area = pu.block(getFirstComponentOfChannel(channelType));
    const Position posRT = area.topRight();
    const Position posLB = area.bottomLeft();
    const CodingUnit &cu = pu;

    // Get intra direction of left PU
    const CodingUnit *cuLeft = cu.cs->getCURestricted(posLB.offset(-1, 0), cu, channelType, cu.left);
    if( cuLeft && CU::isIntra( *cuLeft ) )
    {
      leftIntraDir = PU::getIntraDirLuma( *cuLeft );
    }

    // Get intra direction of above PU
    const CodingUnit *cuAbove = cu.cs->getCURestricted(posRT.offset(0, -1), cu, channelType, cu.above);
    if( cuAbove && CU::isIntra( *cuAbove ) && CU::isSameCtu( cu, *cuAbove ) )
    {
      aboveIntraDir = PU::getIntraDirLuma( *cuAbove );
    }

    CHECK(2 >= numMPMs, "Invalid number of most probable modes");

    const int offset = (int)NUM_LUMA_MODE - 6;
    const int mod = offset + 3;

    {
      mpm[0] = PLANAR_IDX;
      mpm[1] = DC_IDX;
      mpm[2] = VER_IDX;
      mpm[3] = HOR_IDX;
      mpm[4] = VER_IDX - 4;
      mpm[5] = VER_IDX + 4;

      if (leftIntraDir == aboveIntraDir)
      {
        numCand = 1;
        if (leftIntraDir > DC_IDX)
        {
          mpm[0] = PLANAR_IDX;
          mpm[1] = leftIntraDir;
          mpm[2] = ((leftIntraDir + offset) % mod) + 2;
          mpm[3] = ((leftIntraDir - 1) % mod) + 2;
          mpm[4] = ((leftIntraDir + offset - 1) % mod) + 2;
          mpm[5] = ( leftIntraDir               % mod) + 2;
        }
      }
      else //L!=A
      {
        numCand = 2;
        int  maxCandModeIdx = mpm[0] > mpm[1] ? 0 : 1;

        if ((leftIntraDir > DC_IDX) && (aboveIntraDir > DC_IDX))
        {
          mpm[0] = PLANAR_IDX;
          mpm[1] = leftIntraDir;
          mpm[2] = aboveIntraDir;
          maxCandModeIdx = mpm[1] > mpm[2] ? 1 : 2;
          int minCandModeIdx = mpm[1] > mpm[2] ? 2 : 1;
          if (mpm[maxCandModeIdx] - mpm[minCandModeIdx] == 1)
          {
            mpm[3] = ((mpm[minCandModeIdx] + offset)     % mod) + 2;
            mpm[4] = ((mpm[maxCandModeIdx] - 1)          % mod) + 2;
            mpm[5] = ((mpm[minCandModeIdx] + offset - 1) % mod) + 2;
          }
          else if (mpm[maxCandModeIdx] - mpm[minCandModeIdx] >= 62)
          {
            mpm[3] = ((mpm[minCandModeIdx] - 1)      % mod) + 2;
            mpm[4] = ((mpm[maxCandModeIdx] + offset) % mod) + 2;
            mpm[5] = ( mpm[minCandModeIdx]           % mod) + 2;
          }
          else if (mpm[maxCandModeIdx] - mpm[minCandModeIdx] == 2)
          {
            mpm[3] = ((mpm[minCandModeIdx] - 1)      % mod) + 2;
            mpm[4] = ((mpm[minCandModeIdx] + offset) % mod) + 2;
            mpm[5] = ((mpm[maxCandModeIdx] - 1)      % mod) + 2;
          }
          else
          {
            mpm[3] = ((mpm[minCandModeIdx] + offset) % mod) + 2;
            mpm[4] = ((mpm[minCandModeIdx] - 1)      % mod) + 2;
            mpm[5] = ((mpm[maxCandModeIdx] + offset) % mod) + 2;
          }
        }
        else if (leftIntraDir + aboveIntraDir >= 2)
        {
          mpm[0] = PLANAR_IDX;
          mpm[1] = (leftIntraDir < aboveIntraDir) ? aboveIntraDir : leftIntraDir;
          maxCandModeIdx = 1;
          mpm[2] = ((mpm[maxCandModeIdx] + offset)     % mod) + 2;
          mpm[3] = ((mpm[maxCandModeIdx] - 1)          % mod) + 2;
          mpm[4] = ((mpm[maxCandModeIdx] + offset - 1) % mod) + 2;
          mpm[5] = ( mpm[maxCandModeIdx]               % mod) + 2;
        }
      }
    }
    for (int i = 0; i < numMPMs; i++)
    {
      CHECK(mpm[i] >= NUM_LUMA_MODE, "Invalid MPM");
    }
    CHECK(numCand == 0, "No candidates found");
    return numCand;
  }
}

bool CU::isMIP( const CodingUnit &cu, const ChannelType &chType )
{
  if( chType == CHANNEL_TYPE_LUMA )
  {
    // Default case if chType is omitted.
    return cu.mipFlag();
  }
  else
  {
    return PU::isDMChromaMIP(cu) && (cu.intraDir[CHANNEL_TYPE_CHROMA] == DM_CHROMA_IDX);
  }
}

bool PU::isDMChromaMIP(const PredictionUnit &pu)
{
  return !CU::isSepTree( pu ) && (pu.chromaFormat == CHROMA_444) && getCoLocatedLumaPU(pu).mipFlag();
}

int PU::getMipSizeId(const PredictionUnit &pu)
{
  if( ( pu.lwidth() == 4 ) && ( pu.lheight() == 4 ) )
  {
    return 0; // MIP with 16x4 matrix
  }
  else if( pu.lwidth() <= 8 && pu.lheight() <= 8 )
  {
    return 1; // MIP with 16x8 matrix
  }
  else
  {
    return 2; // MIP with 64x8 matrix
  }
}


uint32_t PU::getIntraDirLuma( const PredictionUnit &pu )
{
  if( CU::isMIP( pu ) )
  {
    return PLANAR_IDX;
  }
  else
  {
    return pu.intraDir[CHANNEL_TYPE_LUMA];
  }
}


void PU::getIntraChromaCandModes( const PredictionUnit &pu, unsigned modeList[NUM_CHROMA_MODE] )
{
  modeList[0] = PLANAR_IDX;
  modeList[1] = VER_IDX;
  modeList[2] = HOR_IDX;
  modeList[3] = DC_IDX;
  modeList[4] = LM_CHROMA_IDX;
  modeList[5] = MDLM_L_IDX;
  modeList[6] = MDLM_T_IDX;
  modeList[7] = DM_CHROMA_IDX;

  // If Direct Mode is MIP, mode cannot be already in the list.
  if( isDMChromaMIP(pu) )
  {
    return;
  }

  const uint32_t lumaMode = getCoLocatedIntraLumaMode( pu );
  for( int i = 0; i < 4; i++ )
  {
    if( lumaMode == modeList[i] )
    {
      modeList[i] = VDIA_IDX;
      break;
    }
  }
}


bool PU::isLMCMode(unsigned mode)
{
  return mode >= LM_CHROMA_IDX && mode <= MDLM_T_IDX;
}

int PU::getLMSymbolList( const PredictionUnit &pu, int *pModeList )
{
  pModeList[0] = LM_CHROMA_IDX;
  pModeList[1] = MDLM_L_IDX;
  pModeList[2] = MDLM_T_IDX;

  return 3;
}

uint32_t PU::getFinalIntraMode( const PredictionUnit &pu, const ChannelType &chType )
{
  uint32_t uiIntraMode = pu.intraDir[chType];

  if( uiIntraMode == DM_CHROMA_IDX && !isLuma( chType ) )
  {
    uiIntraMode = getCoLocatedIntraLumaMode( pu );
  }
  if( pu.chromaFormat == CHROMA_422 && !isLuma( chType ) && uiIntraMode < NUM_LUMA_MODE ) // map directional, planar and dc
  {
    uiIntraMode = g_chroma422IntraAngleMappingTable[uiIntraMode];
  }
  return uiIntraMode;
}

const PredictionUnit &PU::getCoLocatedLumaPU( const PredictionUnit &pu )
{
  Position              topLeftPos = pu.blocks[pu.chType()].lumaPos ( pu.chromaFormat );
  Position refPos     = topLeftPos.offset( pu.blocks[pu.chType()].lumaSize( pu.chromaFormat ).width >> 1, pu.blocks[pu.chType()].lumaSize( pu.chromaFormat ).height >> 1 );

  const PredictionUnit &lumaPU = CU::isSepTree( pu ) ? *pu.cs->getCU( refPos, CHANNEL_TYPE_LUMA ) : pu;

  return lumaPU;
}

uint32_t PU::getCoLocatedIntraLumaMode( const PredictionUnit &pu )
{
  return PU::getIntraDirLuma( PU::getCoLocatedLumaPU(pu) );
}

int PU::getWideAngIntraMode( const TransformUnit &tu, const uint32_t dirMode, const ComponentID compID )
{
  if( dirMode < 2 )
  {
    return ( int ) dirMode;
  }
  const CompArea&  area         = tu.cu->ispMode() && isLuma(compID) ? tu.cu->blocks[compID] : tu.blocks[ compID ];
  int              width        = int( area.width );
  int              height       = int( area.height );
  static const int modeShift[ ] = { 0, 6, 10, 12, 14, 15 };
  int              deltaSize    = abs( getLog2( width ) - getLog2( height ) );
  int              predMode     = dirMode;

  if( width > height && dirMode < 2 + modeShift[ deltaSize ] )
  {
    predMode += ( VDIA_IDX - 1 );
  }
  else if( height > width && predMode > VDIA_IDX - modeShift[ deltaSize ] )
  {
    predMode -= ( VDIA_IDX + 1 );
  }

  return predMode;
}

bool PU::xCheckSimilarMotion(const int mergeCandIndex, const int prevCnt, const MergeCtx& mergeCandList, bool hasPruned[MRG_MAX_NUM_CANDS])
{
  for (uint32_t ui = 0; ui < prevCnt; ui++)
  {
    if (hasPruned[ui])
    {
      continue;
    }
    if (mergeCandList.interDirNeighbours[ui] == mergeCandList.interDirNeighbours[mergeCandIndex])
    {
      if (mergeCandList.interDirNeighbours[ui] == 3)
      {
        int offset0 = (ui * 2);
        int offset1 = (mergeCandIndex * 2);
        if (mergeCandList.mvFieldNeighbours[offset0    ].mfRefIdx == mergeCandList.mvFieldNeighbours[offset1    ].mfRefIdx &&
            mergeCandList.mvFieldNeighbours[offset0 + 1].mfRefIdx == mergeCandList.mvFieldNeighbours[offset1 + 1].mfRefIdx &&
            mergeCandList.mvFieldNeighbours[offset0    ].mv     == mergeCandList.mvFieldNeighbours[offset1    ].mv     &&
            mergeCandList.mvFieldNeighbours[offset0 + 1].mv     == mergeCandList.mvFieldNeighbours[offset1 + 1].mv
          )
        {
          hasPruned[ui] = true;
          return true;
        }
      }
      else
      {
        int offset0 = (ui             * 2) + mergeCandList.interDirNeighbours[ui] - 1;
        int offset1 = (mergeCandIndex * 2) + mergeCandList.interDirNeighbours[ui] - 1;
        if (mergeCandList.mvFieldNeighbours[offset0].mfRefIdx == mergeCandList.mvFieldNeighbours[offset1].mfRefIdx &&
            mergeCandList.mvFieldNeighbours[offset0].mv       == mergeCandList.mvFieldNeighbours[offset1].mv
          )
        {
          hasPruned[ui] = true;
          return true;
        }
      }
    }
  }

  return false;
}


bool PU::addMergeHMVPCand(const CodingStructure &cs, MergeCtx& mrgCtx, MotionHist& hist, bool canFastExit, const int& mrgCandIdx, const uint32_t maxNumMergeCandMin1, int &cnt, const int prevCnt, bool isAvailableSubPu, unsigned subPuMvpPos, bool ibcFlag, bool isGt4x4, bool isInterB)
{
  bool hasPruned[MRG_MAX_NUM_CANDS];
  memset(hasPruned, 0, MRG_MAX_NUM_CANDS * sizeof(bool));
  if (isAvailableSubPu)
  {
    hasPruned[subPuMvpPos] = true;
  }
  auto &lut = ibcFlag ? hist.motionLutIbc : hist.motionLut;
  int num_avai_candInLUT = (int) lut.size();

  for (int mrgIdx = 1; mrgIdx <= num_avai_candInLUT; mrgIdx++)
  {
    const HPMVInfo &miNeighbor = lut[(num_avai_candInLUT - mrgIdx)];
    mrgCtx.interDirNeighbours[cnt] = miNeighbor.interDir();
    mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miNeighbor.mv[0], miNeighbor.mhRefIdx[0]);
    mrgCtx.useAltHpelIf[cnt] = !ibcFlag && miNeighbor.useAltHpelIf;
    if (isInterB)
    {
      mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miNeighbor.mv[1], miNeighbor.mhRefIdx[1]);
    }
    if (mrgIdx > 2 || ((mrgIdx > 1 || !isGt4x4) && ibcFlag) || !xCheckSimilarMotion(cnt, prevCnt, mrgCtx, hasPruned))
    {
      mrgCtx.BcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? miNeighbor.BcwIdx : BCW_DEFAULT;
      if (mrgCandIdx == cnt && canFastExit)
      {
        return true;
      }
      cnt ++;
      if (cnt  == maxNumMergeCandMin1)
      {
        break;
      }
    }
  }
  if (cnt < maxNumMergeCandMin1)
  {
    mrgCtx.useAltHpelIf[cnt] = false;
  }
  return false;
}

void PU::getIBCMergeCandidates(const PredictionUnit &pu, MergeCtx& mrgCtx, MotionHist& hist, const int& mrgCandIdx)
{
  const CodingUnit      &cu = pu;
  const CodingStructure &cs = *cu.cs;
  const Slice &slice = *cu.slice;
  const uint32_t maxNumMergeCand = slice.getPicHeader()->getMaxNumIBCMergeCand();
  const bool canFastExit = true; // TODO: remove this

  for (uint32_t ui = 0; ui < maxNumMergeCand; ++ui)
  {
    mrgCtx.BcwIdx[ui] = BCW_DEFAULT;
    mrgCtx.interDirNeighbours[ui] = 0;
    mrgCtx.mrgTypeNeighbours[ui] = MRG_TYPE_IBC;
    mrgCtx.mvFieldNeighbours[ui * 2].mfRefIdx = MF_NOT_VALID;
    mrgCtx.mvFieldNeighbours[ui * 2 + 1].mfRefIdx = MF_NOT_VALID;
    mrgCtx.useAltHpelIf[ui] = false;
  }

  mrgCtx.numValidMergeCand = maxNumMergeCand;
  // compute the location of the current PU

  int cnt = 0;

  const Position posRT = pu.Y().topRight();
  const Position posLB = pu.Y().bottomLeft();

  MotionInfo miAbove, miLeft, miAboveLeft, miAboveRight, miBelowLeft;

  //left
  const CodingUnit* cuLeft = cs.getCURestricted(posLB.offset(-1, 0), cu, CH_L, cu.left);
  const bool isAvailableA1 = cuLeft && CU::isIBC(*cuLeft);
  bool isGt4x4 = pu.lwidth() * pu.lheight() > 16;
  if (isGt4x4 && isAvailableA1)
  {
    miLeft             = cuLeft->getMotionInfo(posLB.offset(-1, 0));
    miLeft.miRefIdx[0] = MI_NOT_VALID + 1;

    // get Inter Dir
    mrgCtx.interDirNeighbours[cnt] = miLeft.interDir();
    // get Mv from Left
    mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miLeft.mv[0], 0 );
    if (mrgCandIdx == cnt && canFastExit)
    {
      return;
    }
    cnt++;
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }


  // above
  const CodingUnit *cuAbove = cs.getCURestricted(posRT.offset(0, -1), cu, CH_L, cu.above);
  bool isAvailableB1 = cuAbove && CU::isIBC(*cuAbove);
  if (isGt4x4 && isAvailableB1)
  {
    miAbove             = cuAbove->getMotionInfo(posRT.offset(0, -1));
    miAbove.miRefIdx[0] = MI_NOT_VALID + 1;

    if (!isAvailableA1 || cuAbove->slice->getIndependentSliceIdx() != cuLeft->slice->getIndependentSliceIdx() || miAbove != miLeft)
    {
      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miAbove.interDir();
      // get Mv from Above
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miAbove.mv[0], 0 );
      if (mrgCandIdx == cnt && canFastExit)
      {
        return;
      }

      cnt++;
    }
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

  int spatialCandPos = cnt;


  int maxNumMergeCandMin1 = maxNumMergeCand;
  if( cnt != maxNumMergeCandMin1 )
  {
    bool isAvailableSubPu = false;
    unsigned subPuMvpPos = 0;

    bool bFound = addMergeHMVPCand( cs, mrgCtx, hist, canFastExit, mrgCandIdx, maxNumMergeCandMin1, cnt, spatialCandPos, isAvailableSubPu, subPuMvpPos, true, isGt4x4, pu.slice->isInterB() );
    if( bFound )
    {
      return;
    }
  }

  while( cnt < maxNumMergeCand )
  {
    mrgCtx.mvFieldNeighbours [cnt * 2].setMvField( Mv( 0, 0 ), NOT_VALID );
    mrgCtx.interDirNeighbours[cnt] = 1;
    cnt++;
    if( mrgCandIdx == cnt && canFastExit )
    {
      return;
    }
  }

  mrgCtx.numValidMergeCand = cnt;
}

void PU::getInterMergeCandidates( const PredictionUnit &pu, MergeCtx& mrgCtx, MotionHist& hist, const int& mrgCandIdx )
{
  const unsigned plevel      = pu.sps->getLog2ParallelMergeLevelMinus2() + 2;
  const CodingStructure &cs  = *pu.cs;
  const Slice &slice         = *pu.slice;
  const uint32_t maxNumMergeCand = pu.sps->getMaxNumMergeCand();// slice.getPicHeader()->getMaxNumMergeCand();
  const bool canFastExit = true; // TODO: remove this
  const CodingUnit &cu       = pu;


  for (uint32_t ui = 0; ui < maxNumMergeCand; ++ui)
  {
    mrgCtx.BcwIdx[ui] = BCW_DEFAULT;
    mrgCtx.interDirNeighbours[ui] = 0;
    mrgCtx.mrgTypeNeighbours [ui] = MRG_TYPE_DEFAULT_N;
    mrgCtx.mvFieldNeighbours[(ui << 1)    ].mfRefIdx = MF_NOT_VALID;
    mrgCtx.mvFieldNeighbours[(ui << 1) + 1].mfRefIdx = MF_NOT_VALID;
    mrgCtx.useAltHpelIf[ui] = false;
  }

  mrgCtx.numValidMergeCand = maxNumMergeCand;
  // compute the location of the current PU

  int cnt = 0;

  const Position posLT = pu.Y().topLeft();
  const Position posRT = pu.Y().topRight();
  const Position posLB = pu.Y().bottomLeft();
  MotionInfo miAbove, miLeft, miAboveLeft, miAboveRight, miBelowLeft;

  // above
  const CodingUnit *cuAbove = cs.getCURestricted( posRT.offset( 0, -1 ), cu, CH_L, cu.above );

  bool isAvailableB1 = cuAbove && CU::isInter( *cuAbove ) && isDiffMER( pu.lumaPos(), posRT.offset( 0, -1 ), plevel );

  if( isAvailableB1 )
  {
    miAbove = cuAbove->getMotionInfo( posRT.offset( 0, -1 ) );

    // get Inter Dir
    mrgCtx.interDirNeighbours[cnt] = miAbove.interDir();
    mrgCtx.useAltHpelIf[cnt] = cuAbove->imv() == IMV_HPEL;
    // get Mv from Above
    mrgCtx.BcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? cuAbove->BcwIdx() : BCW_DEFAULT;
    mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miAbove.mv[0], miAbove.miRefIdx[0] - 1 );

    if( slice.isInterB() )
    {
      mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miAbove.mv[1], miAbove.miRefIdx[1] - 1 );
    }
    if (mrgCandIdx == cnt && canFastExit)
    {
      return;
    }

    cnt++;
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

  //left
  const CodingUnit* cuLeft = cs.getCURestricted( posLB.offset( -1, 0 ), cu, CH_L, cu.left );

  const bool isAvailableA1 = cuLeft && CU::isInter( *cuLeft ) && isDiffMER( pu.lumaPos(), posLB.offset( -1, 0 ), plevel );

  if( isAvailableA1 )
  {
    miLeft = cuLeft->getMotionInfo( posLB.offset(-1, 0) );

    if (!isAvailableB1 || cuAbove->slice->getIndependentSliceIdx() != cuLeft->slice->getIndependentSliceIdx() || miAbove != miLeft)
    {
      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miLeft.interDir();
      mrgCtx.useAltHpelIf[cnt] = cuLeft->imv() == IMV_HPEL;
      mrgCtx.BcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? cuLeft->BcwIdx() : BCW_DEFAULT;
      // get Mv from Left
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miLeft.mv[0], miLeft.miRefIdx[0] - 1);

      if (slice.isInterB())
      {
        mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miLeft.mv[1], miLeft.miRefIdx[1] - 1);
      }
      if (mrgCandIdx == cnt && canFastExit)
      {
        return;
      }

    cnt++;
  }
}

    // early termination
    if( cnt == maxNumMergeCand )
    {
      return;
    }

  int spatialCandPos = cnt;

  // above right
  const CodingUnit *cuAboveRight = cs.getCURestricted( posRT.offset( 1, -1 ), cu, CH_L, cuAbove );

  bool isAvailableB0 = cuAboveRight && CU::isInter( *cuAboveRight ) && isDiffMER( pu.lumaPos(), posRT.offset(1, -1), plevel);

  if( isAvailableB0 )
  {
    miAboveRight = cuAboveRight->getMotionInfo( posRT.offset( 1, -1 ) );

    if( !isAvailableB1 || cuAbove->slice->getIndependentSliceIdx() != cuAboveRight->slice->getIndependentSliceIdx() || miAbove != miAboveRight )
    {

      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miAboveRight.interDir();
      mrgCtx.useAltHpelIf[cnt] = cuAboveRight->imv() == IMV_HPEL;
      // get Mv from Above-right
      mrgCtx.BcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? cuAboveRight->BcwIdx() : BCW_DEFAULT;
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miAboveRight.mv[0], miAboveRight.miRefIdx[0] - 1 );

      if( slice.isInterB() )
      {
        mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miAboveRight.mv[1], miAboveRight.miRefIdx[1] - 1 );
      }

      if (mrgCandIdx == cnt && canFastExit)
      {
        return;
      }

      cnt++;
    }

    // early termination
    if( cnt == maxNumMergeCand )
    {
      return;
    }
  }

  //left bottom
  const CodingUnit *cuLeftBottom = cs.getCURestricted( posLB.offset( -1, 1 ), cu, CH_L, cuLeft );

  bool isAvailableA0 = cuLeftBottom && CU::isInter( *cuLeftBottom ) && isDiffMER( pu.lumaPos(), posLB.offset(-1, 1), plevel);

  if( isAvailableA0 )
  {
    miBelowLeft = cuLeftBottom->getMotionInfo( posLB.offset( -1, 1 ) );

    if( !isAvailableA1 || cuLeftBottom->slice->getIndependentSliceIdx() != cuLeft->slice->getIndependentSliceIdx() || miBelowLeft != miLeft )
    {

      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miBelowLeft.interDir();
      mrgCtx.useAltHpelIf[cnt] = cuLeftBottom->imv() == IMV_HPEL;
      mrgCtx.BcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? cuLeftBottom->BcwIdx() : BCW_DEFAULT;
      // get Mv from Bottom-Left
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miBelowLeft.mv[0], miBelowLeft.miRefIdx[0] - 1 );

      if( slice.isInterB() )
      {
        mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miBelowLeft.mv[1], miBelowLeft.miRefIdx[1] - 1 );
      }

      if (mrgCandIdx == cnt && canFastExit)
      {
        return;
      }

      cnt++;
    }

    // early termination
    if( cnt == maxNumMergeCand )
    {
      return;
    }
  }

  // above left
  if ( cnt < 4 )
  {
    const CodingUnit *cuAboveLeft = cs.getCURestricted( posLT.offset( -1, -1 ), cu, CH_L, cu.left ? cu.left : cu.above );

    bool isAvailableB2 = cuAboveLeft && CU::isInter( *cuAboveLeft ) && isDiffMER( pu.lumaPos(), posLT.offset(-1, -1), plevel );

    if( isAvailableB2 )
    {
      miAboveLeft = cuAboveLeft->getMotionInfo( posLT.offset( -1, -1 ) );

      if( ( !isAvailableA1 || cuLeft->slice->getIndependentSliceIdx() != cuAboveLeft->slice->getIndependentSliceIdx() || miLeft != miAboveLeft ) && ( !isAvailableB1 || cuAbove->slice->getIndependentSliceIdx() != cuAboveLeft->slice->getIndependentSliceIdx() || miAbove != miAboveLeft ) )
      {

        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = miAboveLeft.interDir();
        mrgCtx.useAltHpelIf[cnt] = cuAboveLeft->imv() == IMV_HPEL;
        mrgCtx.BcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? cuAboveLeft->BcwIdx() : BCW_DEFAULT;
        // get Mv from Above-Left
        mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miAboveLeft.mv[0], miAboveLeft.miRefIdx[0] - 1 );

        if( slice.isInterB() )
        {
          mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miAboveLeft.mv[1], miAboveLeft.miRefIdx[1] - 1 );
        }

        if (mrgCandIdx == cnt && canFastExit)
        {
          return;
        }

        cnt++;
      }
    }

    // early termination
    if( cnt == maxNumMergeCand )
    {
      return;
    }
  }

  if( slice.getPicHeader()->getEnableTMVPFlag() && (pu.lumaSize().width + pu.lumaSize().height > 12) )
  {
    //>> MTK colocated-RightBottom
    // offset the pos to be sure to "point" to the same position the uiAbsPartIdx would've pointed to

//    Position posRB = pu.shareParentArea.topLeft().offset(pu.shareParentArea.width-3, pu.shareParentArea.height - 3);
    Position posRB = pu.Y().bottomRight().offset( -3, -3 );
    const PreCalcValues& pcv = *cs.pcv;

    Position posC0;
    Position posC1 = pu.Y().center();
    bool C0Avail = false;

    bool boundaryCond = ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight);
    const SubPic& curSubPic = cu.pps->getSubPicFromPos( pu.lumaPos() );
    if (curSubPic.getTreatedAsPicFlag())
    {
      boundaryCond = ((posRB.x + pcv.minCUWidth) <= curSubPic.getSubPicRight() &&
                      (posRB.y + pcv.minCUHeight) <= curSubPic.getSubPicBottom());
    }
    if (boundaryCond)
    {
      {
        Position posInCtu( posRB.x & pcv.maxCUWidthMask, posRB.y & pcv.maxCUHeightMask );

        if( ( posInCtu.x + 4 < pcv.maxCUWidth ) &&           // is not at the last column of CTU
            ( posInCtu.y + 4 < pcv.maxCUHeight ) )           // is not at the last row    of CTU
        {
          posC0 = posRB.offset( 4, 4 );
          C0Avail = true;
        }
        else if( posInCtu.x + 4 < pcv.maxCUWidth )           // is not at the last column of CTU But is last row of CTU
        {
          posC0 = posRB.offset( 4, 4 );
          // in the reference the CTU address is not set - thus probably resulting in no using this C0 possibility
        }
        else if( posInCtu.y + 4 < pcv.maxCUHeight )          // is not at the last row of CTU But is last column of CTU
        {
          posC0 = posRB.offset( 4, 4 );
          C0Avail = true;
        }
        else //is the right bottom corner of CTU
        {
          posC0 = posRB.offset( 4, 4 );
          // same as for last column but not last row
        }
      }
    }

    Mv        cColMv;
    int       iRefIdx     = 0;
    int       dir         = 0;
    unsigned  uiArrayAddr = cnt;
    bool      bExistMV    = ( C0Avail && getColocatedMVP( pu, REF_PIC_LIST_0, posC0, cColMv, iRefIdx ) )
                                      || getColocatedMVP( pu, REF_PIC_LIST_0, posC1, cColMv, iRefIdx );
    if (bExistMV)
    {
      dir     |= 1;
      mrgCtx.mvFieldNeighbours[2 * uiArrayAddr].setMvField(cColMv, iRefIdx);
    }

    if (slice.isInterB())
    {
      bExistMV = ( C0Avail && getColocatedMVP( pu, REF_PIC_LIST_1, posC0, cColMv, iRefIdx ) )
                           || getColocatedMVP( pu, REF_PIC_LIST_1, posC1, cColMv, iRefIdx );
      if (bExistMV)
      {
        dir     |= 2;
        mrgCtx.mvFieldNeighbours[2 * uiArrayAddr + 1].setMvField(cColMv, iRefIdx);
      }
    }

    if( dir != 0 )
    {
      bool addTMvp = true;
      if( addTMvp )
      {
        mrgCtx.interDirNeighbours[uiArrayAddr] = dir;
        mrgCtx.BcwIdx[uiArrayAddr] = BCW_DEFAULT;
        mrgCtx.useAltHpelIf[uiArrayAddr] = false;
        if (mrgCandIdx == cnt && canFastExit)
        {
          return;
        }

        cnt++;
      }
    }

    // early termination
    if( cnt == maxNumMergeCand )
    {
      return;
    }
  }

  int maxNumMergeCandMin1 = maxNumMergeCand - 1;
  if( cnt != maxNumMergeCandMin1 )
  {
    bool isAvailableSubPu = false;
    unsigned subPuMvpPos = 0;
    bool bFound = addMergeHMVPCand( cs, mrgCtx, hist, canFastExit, mrgCandIdx, maxNumMergeCandMin1, cnt, spatialCandPos, isAvailableSubPu, subPuMvpPos, CU::isIBC( pu ), true, pu.slice->isInterB() );

    if( bFound )
    {
      return;
    }
  }

  // pairwise-average candidates
  {
    if (cnt > 1 && cnt < maxNumMergeCand)
    {

      mrgCtx.mvFieldNeighbours[cnt * 2    ].setMvField( Mv( 0, 0 ), MF_NOT_VALID );
      mrgCtx.mvFieldNeighbours[cnt * 2 + 1].setMvField( Mv( 0, 0 ), MF_NOT_VALID );
      // calculate average MV for L0 and L1 seperately
      unsigned char interDir = 0;


      mrgCtx.useAltHpelIf[cnt] = (mrgCtx.useAltHpelIf[0] == mrgCtx.useAltHpelIf[1]) ? mrgCtx.useAltHpelIf[0] : false;
      for( int refListId = 0; refListId < (slice.isInterB() ? 2 : 1); refListId++ )
      {
        const short refIdxI = mrgCtx.mvFieldNeighbours[0 * 2 + refListId].mfRefIdx;
        const short refIdxJ = mrgCtx.mvFieldNeighbours[1 * 2 + refListId].mfRefIdx;

        // both MVs are invalid, skip
        if( (refIdxI == NOT_VALID) && (refIdxJ == NOT_VALID) )
        {
          continue;
        }

        interDir += 1 << refListId;
        // both MVs are valid, average these two MVs
        if( (refIdxI != NOT_VALID) && (refIdxJ != NOT_VALID) )
        {
          const Mv& MvI = mrgCtx.mvFieldNeighbours[0 * 2 + refListId].mv;
          const Mv& MvJ = mrgCtx.mvFieldNeighbours[1 * 2 + refListId].mv;

          // average two MVs
          Mv avgMv = MvI;
          avgMv += MvJ;
          roundAffineMv(avgMv.hor, avgMv.ver, 1);



          mrgCtx.mvFieldNeighbours[cnt * 2 + refListId].setMvField( avgMv, refIdxI );
        }
        // only one MV is valid, take the only one MV
        else if( refIdxI != NOT_VALID )
        {
          Mv singleMv = mrgCtx.mvFieldNeighbours[0 * 2 + refListId].mv;
          mrgCtx.mvFieldNeighbours[cnt * 2 + refListId].setMvField( singleMv, refIdxI );
        }
        else if( refIdxJ != NOT_VALID )
        {
          Mv singleMv = mrgCtx.mvFieldNeighbours[1 * 2 + refListId].mv;
          mrgCtx.mvFieldNeighbours[cnt * 2 + refListId].setMvField( singleMv, refIdxJ );
        }
      }

      mrgCtx.interDirNeighbours[cnt] = interDir;
      if( interDir > 0 )
      {
        cnt++;
      }
    }

    // early termination
    if( cnt == maxNumMergeCand )
    {
      return;
    }
  }

  int iArrayAddr = cnt;

  int iNumRefIdx = slice.isInterB() ? std::min(slice.getNumRefIdx(REF_PIC_LIST_0), slice.getNumRefIdx(REF_PIC_LIST_1)) : slice.getNumRefIdx(REF_PIC_LIST_0);

  int r = 0;
  int refcnt = 0;
  // second condition needed for gcc-10 overflow checking. Required for now. TODO: fix properly
  while (iArrayAddr < maxNumMergeCand && iArrayAddr < MRG_MAX_NUM_CANDS)
  {
    mrgCtx.interDirNeighbours [iArrayAddr     ] = 1;
    mrgCtx.BcwIdx             [iArrayAddr     ] = BCW_DEFAULT;
    mrgCtx.mvFieldNeighbours  [iArrayAddr << 1] . setMvField(Mv(0, 0), r);
    mrgCtx.useAltHpelIf       [iArrayAddr     ] = false;

    if (slice.isInterB())
    {
      mrgCtx.interDirNeighbours [ iArrayAddr          ] = 3;
      mrgCtx.mvFieldNeighbours  [(iArrayAddr << 1) + 1].setMvField(Mv(0, 0), r);
    }

    if ( mrgCtx.interDirNeighbours[iArrayAddr] == 1 && cu.slice->getRefPOC(REF_PIC_LIST_0, mrgCtx.mvFieldNeighbours[iArrayAddr << 1].mfRefIdx) == cu.slice->getPOC())
    {
      mrgCtx.mrgTypeNeighbours[iArrayAddr] = MRG_TYPE_IBC;
    }

    iArrayAddr++;

    if (refcnt == iNumRefIdx - 1)
    {
      r = 0;
    }
    else
    {
      ++r;
      ++refcnt;
    }
  }
  mrgCtx.numValidMergeCand = iArrayAddr;
}

bool PU::checkDMVRCondition( const PredictionUnit& pu )
{
  WPScalingParam *wp0;
  WPScalingParam *wp1;
  int refIdx0 = pu.refIdx[REF_PIC_LIST_0];
  int refIdx1 = pu.refIdx[REF_PIC_LIST_1];
  pu.slice->getWpScaling( REF_PIC_LIST_0, refIdx0, wp0 );
  pu.slice->getWpScaling( REF_PIC_LIST_1, refIdx1, wp1 );

  if( pu.sps->getUseDMVR() && !pu.cs->picHeader->getDisDmvrFlag() )
  {
    return pu.mergeFlag()
        && pu.mergeType() == MRG_TYPE_DEFAULT_N
        && !pu.ciipFlag()
        && !pu.affineFlag()
        && !pu.mmvdFlag()
        && PU::isBiPredFromDifferentDirEqDistPoc( pu )
        && pu.lheight()  >=   8
        && pu.lwidth()   >=   8
        && pu.Y().area() >= 128
        && pu.BcwIdx() == BCW_DEFAULT
        && !wp0[COMPONENT_Y].bPresentFlag
        && !wp1[COMPONENT_Y].bPresentFlag
        && !wp0[COMPONENT_Cb].bPresentFlag
        && !wp0[COMPONENT_Cr].bPresentFlag
        && !wp1[COMPONENT_Cb].bPresentFlag
        && !wp1[COMPONENT_Cr].bPresentFlag
        && PU::isRefPicSameSize( pu )
        ;
  }
  else
  {
    return false;
  }
}

static int xGetDistScaleFactor(const int &iCurrPOC, const int &iCurrRefPOC, const int &iColPOC, const int &iColRefPOC)
{
  const int iDiffPocD = iColPOC  - iColRefPOC;
  const int iDiffPocB = iCurrPOC - iCurrRefPOC;

  if (iDiffPocD == iDiffPocB)
  {
    return 4096;
  }
  else
  {
    const int iTDB    = Clip3( -128, 127, iDiffPocB );
    const int iTDD    = Clip3( -128, 127, iDiffPocD );
    const int iX      = ( 0x4000 + abs( iTDD / 2 ) ) / iTDD;
    const int iScale  = Clip3( -4096, 4095, ( iTDB * iX + 32 ) >> 6 );
    return iScale;
  }
}

int convertMvFixedToFloat(int32_t val)
{
  int sign  = val >> 31;
  int scale = getLog2((val ^ sign) | MV_MANTISSA_UPPER_LIMIT) - (MV_MANTISSA_BITCOUNT - 1);

  int exponent;
  int mantissa;
  if (scale >= 0)
  {
    int round = (1 << scale) >> 1;
    int n     = (val + round) >> scale;
    exponent  = scale + ((n ^ sign) >> (MV_MANTISSA_BITCOUNT - 1));
    mantissa  = (n & MV_MANTISSA_UPPER_LIMIT) | (sign << (MV_MANTISSA_BITCOUNT - 1));
  }
  else
  {
    exponent = 0;
    mantissa = val;
  }

  return exponent | (mantissa << MV_EXPONENT_BITCOUNT);
}

int convertMvFloatToFixed(int val)
{
  int exponent = val & MV_EXPONENT_MASK;
  int mantissa = val >> MV_EXPONENT_BITCOUNT;
  return exponent == 0 ? mantissa : (mantissa ^ MV_MANTISSA_LIMIT) << (exponent - 1);
}

int roundMvComp(int x)
{
  return convertMvFloatToFixed(convertMvFixedToFloat(x));
}

int PU::getDistScaleFactor(const int &currPOC, const int &currRefPOC, const int &colPOC, const int &colRefPOC)
{
  return xGetDistScaleFactor(currPOC, currRefPOC, colPOC, colRefPOC);
}

void PU::getInterMMVDMergeCandidates(const PredictionUnit &pu, MergeCtx& mrgCtx, const int& mrgCandIdx)
{
  int refIdxList0, refIdxList1;
  int k;
  int currBaseNum = 0;
  const uint16_t maxNumMergeCand = mrgCtx.numValidMergeCand;

  for (k = 0; k < maxNumMergeCand; k++)
  {
    if (mrgCtx.mrgTypeNeighbours[k] == MRG_TYPE_DEFAULT_N)
    {
      refIdxList0 = mrgCtx.mvFieldNeighbours[(k << 1)].mfRefIdx;
      refIdxList1 = mrgCtx.mvFieldNeighbours[(k << 1) + 1].mfRefIdx;

      if ((refIdxList0 >= 0) && (refIdxList1 >= 0))
      {
        mrgCtx.mmvdBaseMv[currBaseNum][0] = mrgCtx.mvFieldNeighbours[(k << 1)];
        mrgCtx.mmvdBaseMv[currBaseNum][1] = mrgCtx.mvFieldNeighbours[(k << 1) + 1];
      }
      else if (refIdxList0 >= 0)
      {
        mrgCtx.mmvdBaseMv[currBaseNum][0] = mrgCtx.mvFieldNeighbours[(k << 1)];
        mrgCtx.mmvdBaseMv[currBaseNum][1] = MvField(Mv(0, 0), -1);
      }
      else if (refIdxList1 >= 0)
      {
        mrgCtx.mmvdBaseMv[currBaseNum][0] = MvField(Mv(0, 0), -1);
        mrgCtx.mmvdBaseMv[currBaseNum][1] = mrgCtx.mvFieldNeighbours[(k << 1) + 1];
      }
      mrgCtx.mmvdUseAltHpelIf[currBaseNum] = mrgCtx.useAltHpelIf[k];

      currBaseNum++;

      if (currBaseNum == MMVD_BASE_MV_NUM)
        break;
    }
  }
}
bool PU::getColocatedMVP(const PredictionUnit &pu, const RefPicList &eRefPicList, const Position &pos, Mv& rcMv, const int &refIdx )
{
  if( CU::isIBC( pu ) )
  {
    return false;
  }

  const Slice &slice = *pu.slice;

  // use coldir.
  const Picture* const pColPic = slice.getRefPic(RefPicList(slice.isInterB() ? 1 - slice.getColFromL0Flag() : 0), slice.getColRefIdx());

  if( !pColPic )
  {
    return false;
  }

  // Check the position of colocated block is within a subpicture
  const SubPic& curSubPic = pu.pps->getSubPicFromPos( pu.lumaPos() );
  if( curSubPic.getTreatedAsPicFlag() )
  {
    if (!curSubPic.isContainingPos(pos))
      return false;
  }
  RefPicList eColRefPicList = slice.getCheckLDC() ? eRefPicList : RefPicList(slice.getColFromL0Flag());

  const Slice* pColSlice;
  const ColocatedMotionInfo&
               mi = pColPic->cs->getColInfo( pos, pColSlice );

  if( !mi.isInter() )
  {
    return false;
  }

  int iColRefIdx = mi.coRefIdx[eColRefPicList] - 1;

  if( iColRefIdx < 0 )
  {
    eColRefPicList = RefPicList( 1 - eColRefPicList );
    iColRefIdx     = mi.coRefIdx[eColRefPicList] - 1;

    if( iColRefIdx < 0 )
    {
      return false;
    }
  }

  CHECK( pColSlice == nullptr, "Slice segment not found" );

  const Slice &colSlice = *pColSlice;

  const bool bIsCurrRefLongTerm = slice.getIsUsedAsLongTerm(eRefPicList, refIdx);
  const bool bIsColRefLongTerm  = colSlice.getIsUsedAsLongTerm(eColRefPicList, iColRefIdx);

  if (bIsCurrRefLongTerm != bIsColRefLongTerm)
  {
    return false;
  }


  // Scale the vector.
  Mv cColMv = mi.mv[eColRefPicList];
  cColMv.setHor(roundMvComp(cColMv.getHor()));
  cColMv.setVer(roundMvComp(cColMv.getVer()));

  if (bIsCurrRefLongTerm /*|| bIsColRefLongTerm*/)
  {
    rcMv = cColMv;
    rcMv.clipToStorageBitDepth();
  }
  else
  {
    const int currPOC    = slice.getPOC();
    const int colPOC     = colSlice.getPOC();
    const int colRefPOC  = colSlice.getRefPOC(eColRefPicList, iColRefIdx);
    const int currRefPOC = slice.getRefPOC(eRefPicList, refIdx);
    const int distscale  = xGetDistScaleFactor(currPOC, currRefPOC, colPOC, colRefPOC);

    if (distscale == 4096)
    {
      rcMv = cColMv;
      rcMv.clipToStorageBitDepth();
    }
    else
    {
      rcMv = cColMv.scaleMv(distscale);
    }
  }

  return true;
}

bool PU::isDiffMER( const Position& pos1, const Position& pos2, const unsigned plevel )
{
  const unsigned xN = pos1.x;
  const unsigned yN = pos1.y;
  const unsigned xP = pos2.x;
  const unsigned yP = pos2.y;

  if( ( xN >> plevel ) != ( xP >> plevel ) )
  {
    return true;
  }

  if( ( yN >> plevel ) != ( yP >> plevel ) )
  {
    return true;
  }
  return false;
}

/**
 * Constructs a list of candidates for IBC AMVP (See specification, section "Derivation process for motion vector predictor candidates")
 */
void PU::fillIBCMvpCand( PredictionUnit &pu, AMVPInfo &amvpInfo, MotionHist& hist )
{

  AMVPInfo *pInfo = &amvpInfo;

  pInfo->numCand = 0;

  MergeCtx mergeCtx;
  PU::getIBCMergeCandidates(pu, mergeCtx, hist, AMVP_MAX_NUM_CANDS - 1);
  int candIdx = 0;
  while (pInfo->numCand < AMVP_MAX_NUM_CANDS)
  {
    pInfo->mvCand[pInfo->numCand] = mergeCtx.mvFieldNeighbours[(candIdx << 1) + 0].mv;;
    pInfo->numCand++;
    candIdx++;
  }

  for (Mv &mv : pInfo->mvCand)
  {
    mv.roundToPrecision( MV_PRECISION_INTERNAL, pu.imv() == 2 ? MV_PRECISION_4PEL : MV_PRECISION_INT );
  }
}

/** Constructs a list of candidates for AMVP (See specification, section "Derivation process for motion vector predictor candidates")
* \param uiPartIdx
* \param uiPartAddr
* \param eRefPicList
* \param iRefIdx
* \param pInfo
*/
void PU::fillMvpCand(PredictionUnit &pu, const RefPicList &eRefPicList, const int &refIdx, AMVPInfo &amvpInfo, MotionHist& hist)
{
  const CodingStructure &cs = *pu.cs;
  AMVPInfo *pInfo = &amvpInfo;

  pInfo->numCand = 0;

  if (refIdx < 0)
  {
    return;
  }

  //-- Get Spatial MV
  const Position posLT = pu.Y().topLeft();
  const Position posRT = pu.Y().topRight();
  const Position posLB = pu.Y().bottomLeft();

  bool bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, *pInfo );

  if( !bAdded )
  {
    bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_LEFT, *pInfo );
  }

  // Above predictor search
  {
    bool bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, *pInfo );

    if( !bAdded )
    {
      bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE, *pInfo );

      if( !bAdded )
      {
        addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, *pInfo );
      }
    }
  }


  for( int i = 0; i < pInfo->numCand; i++ )
  {
    pInfo->mvCand[i].roundToAmvrSignalPrecision( MV_PRECISION_INTERNAL, pu.imv() );
  }

  if( pInfo->numCand == 2 )
  {
    if( pInfo->mvCand[0] == pInfo->mvCand[1] )
    {
      pInfo->numCand = 1;
    }
  }

  if( cs.picHeader->getEnableTMVPFlag() && pInfo->numCand < AMVP_MAX_NUM_CANDS && ( pu.lumaSize().width + pu.lumaSize().height > 12 ) )
  {
    // Get Temporal Motion Predictor
    const int refIdx_Col = refIdx;

    Position posRB = pu.Y().bottomRight().offset(-3, -3);

    const PreCalcValues& pcv = *cs.pcv;

    Position posC0;
    bool C0Avail = false;
    Position posC1 = pu.Y().center();
    Mv cColMv;

    bool boundaryCond = ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight);
    const SubPic& curSubPic = pu.pps->getSubPicFromPos( pu.lumaPos() );
    if( curSubPic.getTreatedAsPicFlag() )
    {
      boundaryCond = ((posRB.x + pcv.minCUWidth) <= curSubPic.getSubPicRight() &&
                      (posRB.y + pcv.minCUHeight) <= curSubPic.getSubPicBottom());
    }
    if( boundaryCond )
    {
      Position posInCtu( posRB.x & pcv.maxCUWidthMask, posRB.y & pcv.maxCUHeightMask );

      if ((posInCtu.x + 4 < pcv.maxCUWidth) &&           // is not at the last column of CTU
          (posInCtu.y + 4 < pcv.maxCUHeight))             // is not at the last row    of CTU
      {
        posC0 = posRB.offset(4, 4);
        C0Avail = true;
      }
      else if (posInCtu.x + 4 < pcv.maxCUWidth)           // is not at the last column of CTU But is last row of CTU
      {
        // in the reference the CTU address is not set - thus probably resulting in no using this C0 possibility
        posC0 = posRB.offset(4, 4);
      }
      else if (posInCtu.y + 4 < pcv.maxCUHeight)          // is not at the last row of CTU But is last column of CTU
      {
        posC0 = posRB.offset(4, 4);
        C0Avail = true;
      }
      else //is the right bottom corner of CTU
      {
        // same as for last column but not last row
        posC0 = posRB.offset(4, 4);
      }
    }

    if ( ( C0Avail && getColocatedMVP( pu, eRefPicList, posC0, cColMv, refIdx_Col ) ) || getColocatedMVP( pu, eRefPicList, posC1, cColMv, refIdx_Col ) )
    {
      cColMv.roundToAmvrSignalPrecision( MV_PRECISION_INTERNAL, pu.imv() );
      pInfo->mvCand[pInfo->numCand++] = cColMv;
    }
  }
  if (pInfo->numCand < AMVP_MAX_NUM_CANDS)
  {
    const int        currRefPOC = pu.slice->getRefPOC(eRefPicList, refIdx);
    const RefPicList eRefPicList2nd = (eRefPicList == REF_PIC_LIST_0) ? REF_PIC_LIST_1 : REF_PIC_LIST_0;
    addAMVPHMVPCand( pu, hist, eRefPicList, eRefPicList2nd, currRefPOC, *pInfo, pu.imv() );
  }
  if (pInfo->numCand > AMVP_MAX_NUM_CANDS)
  {
    pInfo->numCand = AMVP_MAX_NUM_CANDS;
  }

  while (pInfo->numCand < AMVP_MAX_NUM_CANDS)
  {
    pInfo->mvCand[pInfo->numCand] = Mv( 0, 0 );
    pInfo->numCand++;
  }
  for (Mv &mv : pInfo->mvCand)
  {
    mv.roundToAmvrSignalPrecision( MV_PRECISION_INTERNAL, pu.imv() );
  }
}

bool PU::addAffineMVPCandUnscaled( const PredictionUnit &pu, const RefPicList &refPicList, const int &refIdx, const Position &pos, const MvpDir &dir, AffineAMVPInfo &affiAMVPInfo )
{
  const CodingUnit      &cu = pu;
  const CodingStructure &cs = *cu.cs;
  const CodingUnit  *neibCU = nullptr;
  const CodingUnit  *guess  = cu.left;
  Position neibPos;

  switch ( dir )
  {
  case MD_LEFT:
    neibPos = pos.offset( -1, 0 );
    break;
  case MD_ABOVE:
    guess = cu.above;
    neibPos = pos.offset( 0, -1 );
    break;
  case MD_ABOVE_RIGHT:
    guess = cu.above;
    neibPos = pos.offset( 1, -1 );
    break;
  case MD_BELOW_LEFT:
    neibPos = pos.offset( -1, 1 );
    break;
  case MD_ABOVE_LEFT:
    guess = guess ? guess : cu.above;
    neibPos = pos.offset( -1, -1 );
    break;
  default:
    break;
  }

  neibCU = cs.getCURestricted( neibPos, cu, CH_L, guess );

  if ( !neibCU || !CU::isInter( *neibCU ) || !neibCU->affineFlag() || neibCU->mergeType() != MRG_TYPE_DEFAULT_N )
  {
    return false;
  }

  Mv outputAffineMv[3];
  const MotionInfo& neibMi = neibCU->getMotionInfo( neibPos );

  const int        currRefPOC = pu.slice->getRefPOC( refPicList, refIdx );
  const RefPicList refPicList2nd = (refPicList == REF_PIC_LIST_0) ? REF_PIC_LIST_1 : REF_PIC_LIST_0;

  for ( int predictorSource = 0; predictorSource < 2; predictorSource++ ) // examine the indicated reference picture list, then if not available, examine the other list.
  {
    const RefPicList eRefPicListIndex = (predictorSource == 0) ? refPicList : refPicList2nd;
    const int        neibRefIdx = neibMi.miRefIdx[eRefPicListIndex] - 1;

    if( ( ( neibCU->interDir() & ( eRefPicListIndex + 1 ) ) == 0 ) || pu.slice->getRefPOC( eRefPicListIndex, neibRefIdx ) != currRefPOC )
    {
      continue;
    }

    xInheritedAffineMv( pu, pu.affineType() == AFFINEMODEL_6PARAM, neibCU, eRefPicListIndex, outputAffineMv );

    if( pu.imv() == 0 )
    {
      outputAffineMv[0].roundToPrecision(MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER);
      outputAffineMv[1].roundToPrecision(MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER);
    }
    else if( pu.imv() == 2 )
    {
      outputAffineMv[0].roundToPrecision( MV_PRECISION_INTERNAL, MV_PRECISION_INT );
      outputAffineMv[1].roundToPrecision( MV_PRECISION_INTERNAL, MV_PRECISION_INT );
    }

    affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = outputAffineMv[0];
    affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = outputAffineMv[1];

    if( pu.affineType() == AFFINEMODEL_6PARAM )
    {
      if( pu.imv() == 0 )
      {
        outputAffineMv[2].roundToPrecision( MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER );
      }
      else if( pu.imv() == 2 )
      {
        outputAffineMv[2].roundToPrecision( MV_PRECISION_INTERNAL, MV_PRECISION_INT );
      }
      affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = outputAffineMv[2];
    }
    affiAMVPInfo.numCand++;
    return true;
  }

  return false;
}

void PU::xInheritedAffineMv( const PredictionUnit &pu, bool is6param, const PredictionUnit* puNeighbour, RefPicList eRefPicList, Mv rcMv[3] )
{
  int posNeiX = puNeighbour->Y().pos().x;
  int posNeiY = puNeighbour->Y().pos().y;
  int posCurX = pu.Y().pos().x;
  int posCurY = pu.Y().pos().y;

  int neiW = puNeighbour->Y().width;
  int curW = pu.Y().width;
  int neiH = puNeighbour->Y().height;
  int curH = pu.Y().height;

  Mv mvLT, mvRT, mvLB;
  mvLT = puNeighbour->mv[eRefPicList][0];
  mvRT = puNeighbour->mv[eRefPicList][1];
  mvLB = puNeighbour->mv[eRefPicList][2];

  bool isTopCtuBoundary = false;
  if ( (posNeiY + neiH) % pu.sps->getCTUSize() == 0 && (posNeiY + neiH) == posCurY )
  {
    // use bottom-left and bottom-right sub-block MVs for inheritance
    const Position posRB = puNeighbour->Y().bottomRight();
    const Position posLB = puNeighbour->Y().bottomLeft();
    mvLT = puNeighbour->getMotionInfo( posLB ).mv[eRefPicList];
    mvRT = puNeighbour->getMotionInfo( posRB ).mv[eRefPicList];
    posNeiY += neiH;
    isTopCtuBoundary = true;
  }

  int shift = MAX_CU_DEPTH;
  int iDMvHorX, iDMvHorY, iDMvVerX, iDMvVerY;

  iDMvHorX = (mvRT - mvLT).getHor() << (shift - getLog2(neiW));
  iDMvHorY = (mvRT - mvLT).getVer() << (shift - getLog2(neiW));
  if ( puNeighbour->affineType() == AFFINEMODEL_6PARAM && !isTopCtuBoundary )
  {
    iDMvVerX = (mvLB - mvLT).getHor() << (shift - getLog2(neiH));
    iDMvVerY = (mvLB - mvLT).getVer() << (shift - getLog2(neiH));
  }
  else
  {
    iDMvVerX = -iDMvHorY;
    iDMvVerY = iDMvHorX;
  }

  int iMvScaleHor = mvLT.getHor() << shift;
  int iMvScaleVer = mvLT.getVer() << shift;
  int horTmp, verTmp;

  // v0
  horTmp = iMvScaleHor + iDMvHorX * (posCurX - posNeiX) + iDMvVerX * (posCurY - posNeiY);
  verTmp = iMvScaleVer + iDMvHorY * (posCurX - posNeiX) + iDMvVerY * (posCurY - posNeiY);
  roundAffineMv( horTmp, verTmp, shift );
  rcMv[0].hor = horTmp;
  rcMv[0].ver = verTmp;
  rcMv[0].clipToStorageBitDepth();

  // v1
  horTmp = iMvScaleHor + iDMvHorX * (posCurX + curW - posNeiX) + iDMvVerX * (posCurY - posNeiY);
  verTmp = iMvScaleVer + iDMvHorY * (posCurX + curW - posNeiX) + iDMvVerY * (posCurY - posNeiY);
  roundAffineMv( horTmp, verTmp, shift );
  rcMv[1].hor = horTmp;
  rcMv[1].ver = verTmp;
  rcMv[1].clipToStorageBitDepth();

  // v2
  if ( is6param )
  {
    horTmp = iMvScaleHor + iDMvHorX * (posCurX - posNeiX) + iDMvVerX * (posCurY + curH - posNeiY);
    verTmp = iMvScaleVer + iDMvHorY * (posCurX - posNeiX) + iDMvVerY * (posCurY + curH - posNeiY);
    roundAffineMv( horTmp, verTmp, shift );
    rcMv[2].hor = horTmp;
    rcMv[2].ver = verTmp;
    rcMv[2].clipToStorageBitDepth();
  }
}


void PU::fillAffineMvpCand(PredictionUnit &pu, const RefPicList &eRefPicList, const int &refIdx, AffineAMVPInfo &affiAMVPInfo)
{
  affiAMVPInfo.numCand = 0;

  if (refIdx < 0)
  {
    return;
  }


  // insert inherited affine candidates
  Mv outputAffineMv[3];
  Position posLT = pu.Y().topLeft();
  Position posRT = pu.Y().topRight();
  Position posLB = pu.Y().bottomLeft();

  // check left neighbor
  if( !addAffineMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, affiAMVPInfo ) )
  {
    addAffineMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_LEFT, affiAMVPInfo );
  }

  // check above neighbor
  if( !addAffineMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, affiAMVPInfo ) )
  {
    if( !addAffineMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE, affiAMVPInfo ) )
    {
      addAffineMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, affiAMVPInfo );
    }
  }

  if( affiAMVPInfo.numCand >= AMVP_MAX_NUM_CANDS )
  {
    for( int i = 0; i < affiAMVPInfo.numCand; i++ )
    {
      if( pu.imv() != 1 )
      {
        affiAMVPInfo.mvCandLT[i].changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER);
        affiAMVPInfo.mvCandRT[i].changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER);
        affiAMVPInfo.mvCandLB[i].changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER);
      }
    }
    return;
  }

  // insert constructed affine candidates
  int cornerMVPattern = 0;

  //-------------------  V0 (START) -------------------//
  AMVPInfo amvpInfo0;
  amvpInfo0.numCand = 0;

  // A->C: Above Left, Above, Left
  addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, amvpInfo0 );
  if ( amvpInfo0.numCand < 1 )
  {
    addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_ABOVE, amvpInfo0 );
  }
  if ( amvpInfo0.numCand < 1 )
  {
    addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_LEFT, amvpInfo0 );
  }
  cornerMVPattern = cornerMVPattern | amvpInfo0.numCand;

  //-------------------  V1 (START) -------------------//
  AMVPInfo amvpInfo1;
  amvpInfo1.numCand = 0;

  // D->E: Above, Above Right
  addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE, amvpInfo1 );
  if ( amvpInfo1.numCand < 1 )
  {
    addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, amvpInfo1 );
  }
  cornerMVPattern = cornerMVPattern | (amvpInfo1.numCand << 1);

  //-------------------  V2 (START) -------------------//
  AMVPInfo amvpInfo2;
  amvpInfo2.numCand = 0;

  // F->G: Left, Below Left
  addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_LEFT, amvpInfo2 );
  if( amvpInfo2.numCand < 1 )
  {
    addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, amvpInfo2 );
  }
  cornerMVPattern = cornerMVPattern | ( amvpInfo2.numCand << 2 );

  outputAffineMv[0] = amvpInfo0.mvCand[0];
  outputAffineMv[1] = amvpInfo1.mvCand[0];
  outputAffineMv[2] = amvpInfo2.mvCand[0];

  if( pu.imv() == 0 )
  {
    outputAffineMv[0].roundToPrecision( MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER );
    outputAffineMv[1].roundToPrecision( MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER );
    outputAffineMv[2].roundToPrecision( MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER );
  }
  else if( pu.imv() == 2 )
  {
    outputAffineMv[0].roundToPrecision( MV_PRECISION_INTERNAL, MV_PRECISION_INT );
    outputAffineMv[1].roundToPrecision( MV_PRECISION_INTERNAL, MV_PRECISION_INT );
    outputAffineMv[2].roundToPrecision( MV_PRECISION_INTERNAL, MV_PRECISION_INT );
  }

  if( cornerMVPattern == 7 || ( cornerMVPattern == 3 && pu.affineType() == AFFINEMODEL_4PARAM ) )
  {
    affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = outputAffineMv[0];
    affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = outputAffineMv[1];
    affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = outputAffineMv[2];
    affiAMVPInfo.numCand++;
  }


  if ( affiAMVPInfo.numCand < 2 )
  {
    // check corner MVs
    for ( int i = 2; i >= 0 && affiAMVPInfo.numCand < AMVP_MAX_NUM_CANDS; i-- )
    {
      if ( cornerMVPattern & (1 << i) ) // MV i exist
      {
        affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = outputAffineMv[i];
        affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = outputAffineMv[i];
        affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = outputAffineMv[i];
        affiAMVPInfo.numCand++;
      }
    }

    // Get Temporal Motion Predictor
    if ( affiAMVPInfo.numCand < 2 && pu.cs->picHeader->getEnableTMVPFlag() )
    {
      const int refIdxCol = refIdx;

      Position posRB = pu.Y().bottomRight().offset( -3, -3 );

      const PreCalcValues& pcv = *pu.cs->pcv;

      Position posC0;
      bool C0Avail = false;
      Position posC1 = pu.Y().center();
      Mv cColMv;
      bool boundaryCond = ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight);
      const SubPic& curSubPic = pu.pps->getSubPicFromPos( pu.lumaPos() );
      if( curSubPic.getTreatedAsPicFlag() )
      {
        boundaryCond = ((posRB.x + pcv.minCUWidth) <= curSubPic.getSubPicRight() &&
          (posRB.y + pcv.minCUHeight) <= curSubPic.getSubPicBottom());
      }
      if( boundaryCond )
      {
        Position posInCtu( posRB.x & pcv.maxCUWidthMask, posRB.y & pcv.maxCUHeightMask );

        if ( (posInCtu.x + 4 < pcv.maxCUWidth) &&           // is not at the last column of CTU
          (posInCtu.y + 4 < pcv.maxCUHeight) )             // is not at the last row    of CTU
        {
          posC0 = posRB.offset( 4, 4 );
          C0Avail = true;
        }
        else if ( posInCtu.x + 4 < pcv.maxCUWidth )           // is not at the last column of CTU But is last row of CTU
        {
          // in the reference the CTU address is not set - thus probably resulting in no using this C0 possibility
          posC0 = posRB.offset( 4, 4 );
        }
        else if ( posInCtu.y + 4 < pcv.maxCUHeight )          // is not at the last row of CTU But is last column of CTU
        {
          posC0 = posRB.offset( 4, 4 );
          C0Avail = true;
        }
        else //is the right bottom corner of CTU
        {
          // same as for last column but not last row
          posC0 = posRB.offset( 4, 4 );
        }
      }

      if( ( C0Avail && getColocatedMVP( pu, eRefPicList, posC0, cColMv, refIdxCol ) ) || getColocatedMVP( pu, eRefPicList, posC1, cColMv, refIdxCol ) )
      {
        if( pu.imv() == 0 )
        {
          cColMv.roundToPrecision( MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER );
        }
        else if( pu.imv() == 2 )
        {
          cColMv.roundToPrecision( MV_PRECISION_INTERNAL, MV_PRECISION_INT );
        }
        affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = cColMv;
        affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = cColMv;
        affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = cColMv;
        affiAMVPInfo.numCand++;
      }
      }

    if( affiAMVPInfo.numCand < 2 )
    {
      // add zero MV
      for( int i = affiAMVPInfo.numCand; i < AMVP_MAX_NUM_CANDS; i++ )
      {
        affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand].setZero();
        affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand].setZero();
        affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand].setZero();
        affiAMVPInfo.numCand++;
      }
    }
    }

  for( int i = 0; i < affiAMVPInfo.numCand; i++ )
  {
    if( pu.imv() != 1 )
    {
      affiAMVPInfo.mvCandLT[i].changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER);
      affiAMVPInfo.mvCandRT[i].changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER);
      affiAMVPInfo.mvCandLB[i].changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER);
    }
  }
}

bool PU::addMVPCandUnscaled( const PredictionUnit &pu, const RefPicList &eRefPicList, const int &iRefIdx, const Position &pos, const MvpDir &eDir, AMVPInfo &info )
{
  const CodingUnit &cu      = pu;
        CodingStructure &cs = *cu.cs;
  const CodingUnit *neibCU  = NULL;
  const CodingUnit  *guess  = cu.left;
        Position neibPos;

  switch (eDir)
  {
  case MD_LEFT:
    neibPos = pos.offset( -1,  0 );
    break;
  case MD_ABOVE:
    guess = cu.above;
    neibPos = pos.offset(  0, -1 );
    break;
  case MD_ABOVE_RIGHT:
    guess = cu.above;
    neibPos = pos.offset(  1, -1 );
    break;
  case MD_BELOW_LEFT:
    neibPos = pos.offset( -1,  1 );
    break;
  case MD_ABOVE_LEFT:
    guess = cu.left ? cu.left : cu.above;
    neibPos = pos.offset( -1, -1 );
    break;
  default:
    break;
  }

  neibCU = cs.getCURestricted( neibPos, cu, CH_L, guess );

  if( neibCU == NULL || !CU::isInter( *neibCU ) )
  {
    return false;
  }

  const MotionInfo& neibMi        = neibCU->getMotionInfo( neibPos );

  const int        currRefPOC     = cu.slice->getRefPOC( eRefPicList, iRefIdx );
  const RefPicList eRefPicList2nd = ( eRefPicList == REF_PIC_LIST_0 ) ? REF_PIC_LIST_1 : REF_PIC_LIST_0;

  for( int predictorSource = 0; predictorSource < 2; predictorSource++ ) // examine the indicated reference picture list, then if not available, examine the other list.
  {
    const RefPicList eRefPicListIndex = ( predictorSource == 0 ) ? eRefPicList : eRefPicList2nd;
    const int        neibRefIdx       = neibMi.miRefIdx[eRefPicListIndex] - 1;

    if( neibRefIdx >= 0 && currRefPOC == cu.slice->getRefPOC( eRefPicListIndex, neibRefIdx ) )
    {
      info.mvCand[info.numCand++] = neibMi.mv[eRefPicListIndex];
      return true;
    }
  }

  return false;
}


void PU::addAMVPHMVPCand( const PredictionUnit &pu, MotionHist& hist, const RefPicList eRefPicList, const RefPicList eRefPicList2nd, const int currRefPOC, AMVPInfo &info, uint8_t imv )
{
  const Slice &slice = *pu.slice;

  auto &lut = CU::isIBC( pu ) ? hist.motionLutIbc : hist.motionLut;
  int num_avai_candInLUT = (int) lut.size();
  int num_allowedCand = std::min(MAX_NUM_HMVP_AVMPCANDS, num_avai_candInLUT);

  for( int mrgIdx = 1; mrgIdx <= num_allowedCand; mrgIdx++ )
  {
    if( info.numCand >= AMVP_MAX_NUM_CANDS )
    {
      return;
    }

    const HPMVInfo &neibMi = lut[mrgIdx - 1];

    for( int predictorSource = 0; predictorSource < 2; predictorSource++ )
    {
      const RefPicList eRefPicListIndex = predictorSource == 0 ? eRefPicList : eRefPicList2nd;
      const int        neibRefIdx       = neibMi.mhRefIdx[eRefPicListIndex];

      if( neibRefIdx >= 0 && ( CU::isIBC( pu ) || currRefPOC == slice.getRefPOC( eRefPicListIndex, neibRefIdx ) ) )
      {
        Mv pmv = neibMi.mv[eRefPicListIndex];
        pmv.roundToAmvrSignalPrecision( MV_PRECISION_INTERNAL, pu.imv() );

        info.mvCand[info.numCand++] = pmv;

        if( info.numCand >= AMVP_MAX_NUM_CANDS )
        {
          return;
        }
      }
    }
  }
}

bool PU::isBipredRestriction( const PredictionUnit &pu )
{
  const SizeType w = pu.lwidth(), h = pu.lheight();
  /* disable bi-prediction for 4x8/8x4 */
  return ( w + h <= 12 );
}

void PU::getAffineControlPointCand( const PredictionUnit &pu, MotionInfo mi[4], bool isAvailable[4], int verIdx[4], int8_t bcwIdx, int modelIdx, int verNum, AffineMergeCtx& affMrgType )
{
  int cuW = pu.Y().width;
  int cuH = pu.Y().height;
  int vx, vy;
  int shift = MAX_CU_DEPTH;
  int shiftHtoW = shift + getLog2(cuW) - getLog2(cuH);

  // motion info
  Mv cMv[2][4];
  int refIdx[2] = { -1, -1 };
  int dir = 0;
  AffineModel curType = (verNum == 2) ? AFFINEMODEL_4PARAM : AFFINEMODEL_6PARAM;

  if ( verNum == 2 )
  {
    int idx0 = verIdx[0], idx1 = verIdx[1];
    if ( !isAvailable[idx0] || !isAvailable[idx1] )
    {
      return;
    }

    for ( int l = 0; l < 2; l++ )
    {
      if ( mi[idx0].miRefIdx[l] != MI_NOT_VALID && mi[idx1].miRefIdx[l] != MI_NOT_VALID )
      {
        // check same refidx and different mv
        if ( mi[idx0].miRefIdx[l] == mi[idx1].miRefIdx[l])
        {
          dir |= (l + 1);
          refIdx[l] = mi[idx0].miRefIdx[l] - 1;
        }
      }
    }

  }
  else if ( verNum == 3 )
  {
    int idx0 = verIdx[0], idx1 = verIdx[1], idx2 = verIdx[2];
    if ( !isAvailable[idx0] || !isAvailable[idx1] || !isAvailable[idx2] )
    {
      return;
    }

    for ( int l = 0; l < 2; l++ )
    {
      if ( mi[idx0].miRefIdx[l] != MI_NOT_VALID && mi[idx1].miRefIdx[l] != MI_NOT_VALID && mi[idx2].miRefIdx[l] != MI_NOT_VALID )
      {
        // check same refidx and different mv
        if ( mi[idx0].miRefIdx[l] == mi[idx1].miRefIdx[l] && mi[idx0].miRefIdx[l] == mi[idx2].miRefIdx[l])
        {
          dir |= (l + 1);
          refIdx[l] = mi[idx0].miRefIdx[l] - 1;
        }
      }
    }
  }

  if ( dir == 0 )
  {
    return;
  }


  for ( int l = 0; l < 2; l++ )
  {
    if ( dir & (l + 1) )
    {
      for ( int i = 0; i < verNum; i++ )
      {
        cMv[l][verIdx[i]] = mi[verIdx[i]].mv[l];
      }

      // convert to LT, RT[, [LB]]
      switch ( modelIdx )
      {
      case 0: // 0 : LT, RT, LB
        break;

      case 1: // 1 : LT, RT, RB
        cMv[l][2].hor = cMv[l][3].hor + cMv[l][0].hor - cMv[l][1].hor;
        cMv[l][2].ver = cMv[l][3].ver + cMv[l][0].ver - cMv[l][1].ver;
        cMv[l][2].clipToStorageBitDepth();
        break;

      case 2: // 2 : LT, LB, RB
        cMv[l][1].hor = cMv[l][3].hor + cMv[l][0].hor - cMv[l][2].hor;
        cMv[l][1].ver = cMv[l][3].ver + cMv[l][0].ver - cMv[l][2].ver;
        cMv[l][1].clipToStorageBitDepth();
        break;

      case 3: // 3 : RT, LB, RB
        cMv[l][0].hor = cMv[l][1].hor + cMv[l][2].hor - cMv[l][3].hor;
        cMv[l][0].ver = cMv[l][1].ver + cMv[l][2].ver - cMv[l][3].ver;
        cMv[l][0].clipToStorageBitDepth();
        break;

      case 4: // 4 : LT, RT
        break;

      case 5: // 5 : LT, LB
        vx = (cMv[l][0].hor << shift) + ((cMv[l][2].ver - cMv[l][0].ver) << shiftHtoW);
        vy = (cMv[l][0].ver << shift) - ((cMv[l][2].hor - cMv[l][0].hor) << shiftHtoW);
        roundAffineMv( vx, vy, shift );
        cMv[l][1].set( vx, vy );
        cMv[l][1].clipToStorageBitDepth();
        break;

      default:
        CHECK( 1, "Invalid model index!\n" );
        break;
      }
    }
    else
    {
      for ( int i = 0; i < 4; i++ )
      {
        cMv[l][i].hor = 0;
        cMv[l][i].ver = 0;
      }
    }
  }

  for ( int i = 0; i < 3; i++ )
  {
    affMrgType.mvFieldNeighbours[(affMrgType.numValidMergeCand << 1) + 0][i].mv       = cMv   [0][i];
    affMrgType.mvFieldNeighbours[(affMrgType.numValidMergeCand << 1) + 0][i].mfRefIdx = refIdx[0];

    affMrgType.mvFieldNeighbours[(affMrgType.numValidMergeCand << 1) + 1][i].mv       = cMv   [1][i];
    affMrgType.mvFieldNeighbours[(affMrgType.numValidMergeCand << 1) + 1][i].mfRefIdx = refIdx[1];
  }

  affMrgType.interDirNeighbours[affMrgType.numValidMergeCand] = dir;
  affMrgType.affineType        [affMrgType.numValidMergeCand] = curType;
  affMrgType.BcwIdx            [affMrgType.numValidMergeCand] = dir == 3 ? bcwIdx : BCW_DEFAULT;
  affMrgType.numValidMergeCand++;


  return;
}

int getAvailableAffineNeighboursForLeftPredictor( const PredictionUnit &pu, const PredictionUnit* npu[] )
{
  const CodingUnit &cu = pu;
  const Position posLB = pu.Y().bottomLeft();
  const unsigned plevel = pu.sps->getLog2ParallelMergeLevelMinus2() + 2;
  int num = 0;

  const CodingUnit *cuLeftBottom = cu.cs->getCURestricted( posLB.offset( -1, 1 ), cu, CH_L, cu.left );
  if( cuLeftBottom && cuLeftBottom->affineFlag() && cuLeftBottom->mergeType() == MRG_TYPE_DEFAULT_N && PU::isDiffMER( pu.lumaPos(), posLB.offset( -1, 1 ), plevel ) )
  {
    npu[num++] = cuLeftBottom;
    return num;
  }

  const CodingUnit* cuLeft = cu.cs->getCURestricted( posLB.offset( -1, 0 ), cu, CH_L, cu.left );
  if( cuLeft && cuLeft->affineFlag() && cuLeft->mergeType() == MRG_TYPE_DEFAULT_N && PU::isDiffMER( pu.lumaPos(), posLB.offset( -1, 0 ), plevel ) )
  {
    npu[num++] = cuLeft;
    return num;
  }

  return num;
}

int getAvailableAffineNeighboursForAbovePredictor( const PredictionUnit &pu, const PredictionUnit* npu[], int numAffNeighLeft )
{
  const CodingUnit &cu = pu;
  const Position posLT = pu.Y().topLeft();
  const Position posRT = pu.Y().topRight();
  const unsigned plevel = pu.sps->getLog2ParallelMergeLevelMinus2() + 2;
  int num = numAffNeighLeft;

  const CodingUnit* cuAboveRight = cu.cs->getCURestricted( posRT.offset( 1, -1 ), cu, CH_L, cu.above );
  if( cuAboveRight && cuAboveRight->affineFlag() && cuAboveRight->mergeType() == MRG_TYPE_DEFAULT_N && PU::isDiffMER( pu.lumaPos(), posRT.offset( 1, -1 ), plevel ) )
  {
    npu[num++] = cuAboveRight;
    return num;
  }

  const CodingUnit* cuAbove = cu.cs->getCURestricted( posRT.offset( 0, -1 ), cu, CH_L, cu.above );
  if( cuAbove && cuAbove->affineFlag() && cuAbove->mergeType() == MRG_TYPE_DEFAULT_N && PU::isDiffMER( pu.lumaPos(), posRT.offset( 0, -1 ), plevel ) )
  {
    npu[num++] = cuAbove;
    return num;
  }

  const CodingUnit *cuAboveLeft = cu.cs->getCURestricted( posLT.offset( -1, -1 ), cu, CH_L, cu.left ? cu.left : cu.above );
  if( cuAboveLeft && cuAboveLeft->affineFlag() && cuAboveLeft->mergeType() == MRG_TYPE_DEFAULT_N && PU::isDiffMER( pu.lumaPos(), posLT.offset( -1, -1 ), plevel ) )
  {
    npu[num++] = cuAboveLeft;
    return num;
  }

  return num;
}

void PU::getAffineMergeCand( const PredictionUnit &pu, AffineMergeCtx& affMrgCtx, const int mrgCandIdx )
{
  const CodingUnit      &cu =  pu;
  const CodingStructure &cs = *cu.cs;
  const Slice &slice        = *cu.slice;
  const uint32_t maxNumAffineMergeCand = slice.getPicHeader()->getMaxNumAffineMergeCand();
  const unsigned plevel = pu.sps->getLog2ParallelMergeLevelMinus2() + 2;

  for ( int i = 0; i < maxNumAffineMergeCand; i++ )
  {
    for ( int mvNum = 0; mvNum < 3; mvNum++ )
    {
      affMrgCtx.mvFieldNeighbours[(i << 1) + 0][mvNum].setMvField( Mv(), MF_NOT_VALID );
      affMrgCtx.mvFieldNeighbours[(i << 1) + 1][mvNum].setMvField( Mv(), MF_NOT_VALID );
    }
    affMrgCtx.interDirNeighbours[i] = 0;
    affMrgCtx.affineType[i] = AFFINEMODEL_4PARAM;
    affMrgCtx.mergeType[i] = MRG_TYPE_DEFAULT_N;
    affMrgCtx.BcwIdx[i] = BCW_DEFAULT;
  }

  affMrgCtx.numValidMergeCand = 0;
  affMrgCtx.maxNumMergeCand = maxNumAffineMergeCand;
  bool enableSubPuMvp = slice.getSPS()->getSBTMVPEnabledFlag() && !(slice.getPOC() == slice.getRefPOC(REF_PIC_LIST_0, 0) && slice.isIRAP());
  bool isAvailableSubPu = false;
  if ( enableSubPuMvp && slice.getPicHeader()->getEnableTMVPFlag() )
  {
    MergeCtx mrgCtx = *affMrgCtx.mrgCtx;
    bool tmpLICFlag = false;

    CHECKD( mrgCtx.subPuMvpMiBuf.area() == 0 || !mrgCtx.subPuMvpMiBuf.buf, "Buffer not initialized" );

    int pos = 0;
    // Get spatial MV
    const Position posCurLB = pu.Y().bottomLeft();

    //left
    const CodingUnit* cuLeft = cs.getCURestricted( posCurLB.offset( -1, 0 ), cu, CH_L, cu.left );
    const bool isAvailableA1 = cuLeft && CU::isInter( *cuLeft ) && isDiffMER( pu.lumaPos(), posCurLB.offset( -1, 0 ), plevel );
    if ( isAvailableA1 )
    {
      const MotionInfo& miLeft = cuLeft->getMotionInfo( posCurLB.offset( -1, 0 ) );
      // get Inter Dir
      mrgCtx.interDirNeighbours[pos] = miLeft.interDir();

      // get Mv from Left
      mrgCtx.mvFieldNeighbours[pos << 1].setMvField( miLeft.mv[0], miLeft.miRefIdx[0] - 1 );

      if ( slice.isInterB() )
      {
        mrgCtx.mvFieldNeighbours[(pos << 1) + 1].setMvField( miLeft.mv[1], miLeft.miRefIdx[1] - 1 );
      }
      pos++;
    }

    mrgCtx.numValidMergeCand = pos;

    isAvailableSubPu = getInterMergeSubPuMvpCand( pu, mrgCtx, tmpLICFlag, pos );

    if( isAvailableSubPu )
    {
      affMrgCtx.mergeType[affMrgCtx.numValidMergeCand] = MRG_TYPE_SUBPU_ATMVP;

      if( affMrgCtx.numValidMergeCand == mrgCandIdx )
      {
        return;
      }

      affMrgCtx.numValidMergeCand++;

      // early termination
      if( affMrgCtx.numValidMergeCand == maxNumAffineMergeCand )
      {
        return;
      }
    }
  }

  if ( slice.getSPS()->getUseAffine() )
  {
    ///> Start: inherited affine candidates
    const PredictionUnit* npu[5];
    int numAffNeighLeft = getAvailableAffineNeighboursForLeftPredictor( pu, npu );
    int numAffNeigh     = getAvailableAffineNeighboursForAbovePredictor( pu, npu, numAffNeighLeft );
    for( int idx = 0; idx < numAffNeigh; idx++ )
    {
      // derive Mv from Neigh affine PU
      Mv cMv[2][3];
      const PredictionUnit* puNeigh = npu[idx];

      if( puNeigh->interDir() != 2 )
      {
        xInheritedAffineMv( pu, puNeigh->affineType() == AFFINEMODEL_6PARAM, puNeigh, REF_PIC_LIST_0, cMv[0] );
      }

      if( slice.isInterB() )
      {
        if( puNeigh->interDir() != 1 )
        {
          xInheritedAffineMv( pu, puNeigh->affineType() == AFFINEMODEL_6PARAM, puNeigh, REF_PIC_LIST_1, cMv[1] );
        }
      }

      for( int mvNum = 0; mvNum < 3; mvNum++ )
      {
        affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 0][mvNum].setMvField( cMv[0][mvNum], puNeigh->refIdx[0] );
        affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 1][mvNum].setMvField( cMv[1][mvNum], puNeigh->refIdx[1] );
      }

      affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] = puNeigh->interDir();
      affMrgCtx.affineType[affMrgCtx.numValidMergeCand]         = puNeigh->affineType();
      affMrgCtx.BcwIdx[affMrgCtx.numValidMergeCand]             = puNeigh->BcwIdx();

      if( affMrgCtx.numValidMergeCand == mrgCandIdx )
      {
        return;
      }

      // early termination
      affMrgCtx.numValidMergeCand++;
      if( affMrgCtx.numValidMergeCand == maxNumAffineMergeCand )
      {
        return;
      }
    }
    ///> End: inherited affine candidates

    ///> Start: Constructed affine candidates
    {
      MotionInfo mi[4];
      bool isAvailable[4] = { false };
      int8_t neighBcw[2] = { BCW_DEFAULT, BCW_DEFAULT };
      // control point: LT B2->B3->A2
      const Position posLT[3] = { pu.Y().topLeft().offset( -1, -1 ), pu.Y().topLeft().offset( 0, -1 ), pu.Y().topLeft().offset( -1, 0 ) };
      const CodingUnit* guess[3] = { cu.left ? cu.left : cu.above, cu.above, cu.left };
      for ( int i = 0; i < 3; i++ )
      {
        const Position pos = posLT[i];
        const CodingUnit* cuNeigh = cs.getCURestricted( pos, cu, CH_L, guess[i] );

        if( cuNeigh && CU::isInter( *cuNeigh ) && PU::isDiffMER( pu.lumaPos(), pos, plevel ) )
        {
          isAvailable[0] = true;
          mi[0] = cuNeigh->getMotionInfo( pos );
          neighBcw[0] = cuNeigh->BcwIdx();
          break;
        }
      }

      // control point: RT B1->B0
      const Position posRT[2] = { pu.Y().topRight().offset( 0, -1 ), pu.Y().topRight().offset( 1, -1 ) };
      for ( int i = 0; i < 2; i++ )
      {
        const Position pos = posRT[i];
        const CodingUnit* cuNeigh = cs.getCURestricted( pos, cu, CH_L, cu.above );

        if( cuNeigh && CU::isInter( *cuNeigh ) && PU::isDiffMER( pu.lumaPos(), pos, plevel ) )
        {
          isAvailable[1] = true;
          mi[1] = cuNeigh->getMotionInfo( pos );
          neighBcw[1] = cuNeigh->BcwIdx();
          break;
        }
      }

      // control point: LB A1->A0
      const Position posLB[2] = { pu.Y().bottomLeft().offset( -1, 0 ), pu.Y().bottomLeft().offset( -1, 1 ) };
      for ( int i = 0; i < 2; i++ )
      {
        const Position pos = posLB[i];
        const CodingUnit* cuNeigh = cs.getCURestricted( pos, cu, CH_L, cu.left );

        if( cuNeigh && CU::isInter( *cuNeigh ) && PU::isDiffMER( pu.lumaPos(), pos, plevel ) )
        {
          isAvailable[2] = true;
          mi[2] = cuNeigh->getMotionInfo( pos );
          break;
        }
      }

      // control point: RB
      if ( slice.getPicHeader()->getEnableTMVPFlag() )
      {
        //>> MTK colocated-RightBottom
        // offset the pos to be sure to "point" to the same position the uiAbsPartIdx would've pointed to
        Position posRB = pu.Y().bottomRight().offset( -3, -3 );

        const PreCalcValues& pcv = *cs.pcv;
        Position posC0;
        bool C0Avail = false;

        bool boundaryCond = ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight);
        const SubPic& curSubPic = cu.pps->getSubPicFromPos( pu.lumaPos() );
        if( curSubPic.getTreatedAsPicFlag() )
        {
          boundaryCond = ((posRB.x + pcv.minCUWidth) <= curSubPic.getSubPicRight() &&
            (posRB.y + pcv.minCUHeight) <= curSubPic.getSubPicBottom());
        }
        if( boundaryCond )
        {
          Position posInCtu( posRB.x & pcv.maxCUWidthMask, posRB.y & pcv.maxCUHeightMask );

          if ( (posInCtu.x + 4 < pcv.maxCUWidth) &&  // is not at the last column of CTU
            (posInCtu.y + 4 < pcv.maxCUHeight) )     // is not at the last row    of CTU
          {
            posC0 = posRB.offset( 4, 4 );
            C0Avail = true;
          }
          else if ( posInCtu.x + 4 < pcv.maxCUWidth ) // is not at the last column of CTU But is last row of CTU
          {
            posC0 = posRB.offset( 4, 4 );
            // in the reference the CTU address is not set - thus probably resulting in no using this C0 possibility
          }
          else if ( posInCtu.y + 4 < pcv.maxCUHeight ) // is not at the last row of CTU But is last column of CTU
          {
            posC0 = posRB.offset( 4, 4 );
            C0Avail = true;
          }
          else //is the right bottom corner of CTU
          {
            posC0 = posRB.offset( 4, 4 );
            // same as for last column but not last row
          }
        }

        Mv        cColMv;
        int       refIdx = 0;
        bool      bExistMV = C0Avail && getColocatedMVP( pu, REF_PIC_LIST_0, posC0, cColMv, refIdx );
        if ( bExistMV )
        {
          mi[3].mv[0] = cColMv;
          mi[3].miRefIdx[0] = refIdx + 1;
          isAvailable[3] = true;
        }

        if ( slice.isInterB() )
        {
          bExistMV = C0Avail && getColocatedMVP( pu, REF_PIC_LIST_1, posC0, cColMv, refIdx );
          if ( bExistMV )
          {
            mi[3].mv[1] = cColMv;
            mi[3].miRefIdx[1] = refIdx + 1;
            isAvailable[3] = true;
          }
        }
      }

      //-------------------  insert model  -------------------//
      int order[6] = { 0, 1, 2, 3, 4, 5 };
      int modelNum = 6;
      int model[6][4] = {
        { 0, 1, 2 },          // 0:  LT, RT, LB
        { 0, 1, 3 },          // 1:  LT, RT, RB
        { 0, 2, 3 },          // 2:  LT, LB, RB
        { 1, 2, 3 },          // 3:  RT, LB, RB
        { 0, 1 },             // 4:  LT, RT
        { 0, 2 },             // 5:  LT, LB
      };

      int verNum[6] = { 3, 3, 3, 3, 2, 2 };
      int startIdx = cu.sps->getUseAffineType() ? 0 : 4;
      for ( int idx = startIdx; idx < modelNum; idx++ )
      {
        int modelIdx = order[idx];
        getAffineControlPointCand( pu, mi, isAvailable, model[modelIdx], modelIdx == 3 ? neighBcw[1] : neighBcw[0], modelIdx, verNum[modelIdx], affMrgCtx );
        if ( affMrgCtx.numValidMergeCand != 0 && affMrgCtx.numValidMergeCand - 1 == mrgCandIdx )
        {
          return;
        }

        // early termination
        if ( affMrgCtx.numValidMergeCand == maxNumAffineMergeCand )
        {
          return;
        }
      }
    }
    ///> End: Constructed affine candidates
  }

  ///> zero padding
  int cnt = affMrgCtx.numValidMergeCand;
  while ( cnt < maxNumAffineMergeCand )
  {
    for ( int mvNum = 0; mvNum < 3; mvNum++ )
    {
      affMrgCtx.mvFieldNeighbours[(cnt << 1) + 0][mvNum].setMvField( Mv( 0, 0 ), 0 );
    }
    affMrgCtx.interDirNeighbours[cnt] = 1;

    if ( slice.isInterB() )
    {
      for ( int mvNum = 0; mvNum < 3; mvNum++ )
      {
        affMrgCtx.mvFieldNeighbours[(cnt << 1) + 1][mvNum].setMvField( Mv( 0, 0 ), 0 );
      }
      affMrgCtx.interDirNeighbours[cnt] = 3;
    }
    affMrgCtx.affineType[cnt] = AFFINEMODEL_4PARAM;
    cnt++;

    if ( cnt == maxNumAffineMergeCand )
    {
      return;
    }
  }
}

void PU::setAllAffineMvField( PredictionUnit &pu, MvField *mvField, RefPicList eRefList )
{
  // Set RefIdx
  CHECK( mvField[0].mfRefIdx != mvField[1].mfRefIdx || mvField[0].mfRefIdx != mvField[2].mfRefIdx, "Affine mv corners don't have the same refIdx." );
  pu.refIdx[eRefList] = mvField[0].mfRefIdx;

  setAllAffineMv( pu, mvField[0].mv, mvField[1].mv, mvField[2].mv, eRefList );
}

void PU::setAllAffineMv( PredictionUnit& pu, Mv affLT, Mv affRT, Mv affLB, RefPicList eRefList, bool clipCPMVs )
{
        int width  = pu.lwidth();
        int height = pu.lheight();
  const int shift  = MAX_CU_DEPTH;

  if( clipCPMVs )
  {
    affLT.mvCliptoStorageBitDepth();
    affRT.mvCliptoStorageBitDepth();
    if( pu.affineType() == AFFINEMODEL_6PARAM )
    {
      affLB.mvCliptoStorageBitDepth();
    }
  }

  int deltaMvHorX, deltaMvHorY, deltaMvVerX, deltaMvVerY;

  deltaMvHorX = ( affRT - affLT ).getHor() << ( shift - getLog2( width ) );
  deltaMvHorY = ( affRT - affLT ).getVer() << ( shift - getLog2( width ) );

  if ( pu.affineType() == AFFINEMODEL_6PARAM )
  {
    deltaMvVerX = ( affLB - affLT ).getHor() << ( shift - getLog2( height ) );
    deltaMvVerY = ( affLB - affLT ).getVer() << ( shift - getLog2( height ) );
  }
  else
  {
    deltaMvVerX = -deltaMvHorY;
    deltaMvVerY =  deltaMvHorX;
  }

  const int mvScaleHor = affLT.getHor() << shift;
  const int mvScaleVer = affLT.getVer() << shift;

  MotionBuf mb = pu.getMotionBuf();

  const bool subblkMVSpreadOverLimit = InterPrediction::isSubblockVectorSpreadOverLimit( deltaMvHorX, deltaMvHorY, deltaMvVerX, deltaMvVerY, pu.interDir() );

  static_assert( AFFINE_MIN_BLOCK_SIZE ==   4,                  "" );
  static_assert( AFFINE_MIN_BLOCK_SIZE == ( 1 << MIN_CU_LOG2 ), "" );

  const int halfBH = 2;

  // fallback motion vector
  Mv flbMv( 0, 0 );

  if( subblkMVSpreadOverLimit )
  {
    flbMv.hor = mvScaleHor + deltaMvHorX * ( width >> 1 ) + deltaMvVerX * ( height >> 1 );
    flbMv.ver = mvScaleVer + deltaMvHorY * ( width >> 1 ) + deltaMvVerY * ( height >> 1 );

    roundAffineMv( flbMv.hor, flbMv.ver, shift );
    flbMv.clipToStorageBitDepth();
  }

  width  >>= MIN_CU_LOG2;
  height >>= MIN_CU_LOG2;

#if ENABLE_SIMD_OPT && defined( TARGET_SIMD_X86 )
  __m128i xvbase = _mm_setr_epi32( mvScaleHor, mvScaleVer, mvScaleHor, mvScaleVer );
  __m128i xvdvxy = _mm_setr_epi32( deltaMvVerX, deltaMvVerY, deltaMvVerX, deltaMvVerY );
  __m128i xhdhxy = _mm_setr_epi32( deltaMvHorX, deltaMvHorY, deltaMvHorX, deltaMvHorY );

#endif
  for( int h = 0; h < height; h++ )
  {
#if ENABLE_SIMD_OPT && defined( TARGET_SIMD_X86 )
    __m128i
    xvoff = _mm_set1_epi32 ( halfBH + ( h << MIN_CU_LOG2 ) );
    xvoff = _mm_mullo_epi32( xvoff, xvdvxy );
    xvoff = _mm_add_epi32  ( xvoff, xvbase );
#endif
    if( subblkMVSpreadOverLimit )
    {
      for( int w = 0; w < width; w++ )
      {
        MotionInfo &mi = mb.at( w, h );

        mi.mv[eRefList] = flbMv;
      }
    }
    else
    {
#if ENABLE_SIMD_OPT && defined( TARGET_SIMD_X86 )
      for( int w = 0; w < width; w += 2 )
      {
        MotionInfo *mi = &mb.at( w, h );

        __m128i
        xhoff = _mm_set1_epi32 ( 2 + ( w << MIN_CU_LOG2 ) );
        xhoff = _mm_add_epi32  ( xhoff, _mm_setr_epi32( 0, 0, 1 << MIN_CU_LOG2, 1 << MIN_CU_LOG2 ) );
        xhoff = _mm_mullo_epi32( xhoff, xhdhxy );
        xhoff = _mm_add_epi32  ( xhoff, xvoff );
        __m128i
        xmv   = _mm_add_epi32  ( xhoff, _mm_set1_epi32( 1 << ( shift - 1 ) ) );
        xmv   = _mm_add_epi32  ( xmv, _mm_cmpgt_epi32( xhoff, _mm_set1_epi32( -1 ) ) );
        xmv   = _mm_srai_epi32 ( xmv, shift );
        xmv   = _mm_max_epi32  ( _mm_set1_epi32( -( 1 << 17 ) ), _mm_min_epi32( _mm_set1_epi32( ( 1 << 17 ) - 1 ), xmv ) );

        _mm_storel_epi64( ( __m128i* ) &mi[0].mv[eRefList], xmv );
        _mm_storel_epi64( ( __m128i* ) &mi[1].mv[eRefList], _mm_unpackhi_epi64( xmv, _mm_setzero_si128() ) );
      }
#else
      for( int w = 0; w < width; w++ )
      {
        MotionInfo &mi = mb.at( w, h );

        int mvHor = mvScaleHor + deltaMvHorX * ( 2 + ( w << MIN_CU_LOG2 ) ) + deltaMvVerX * ( halfBH + ( h << MIN_CU_LOG2 ) );
        int mvVer = mvScaleVer + deltaMvHorY * ( 2 + ( w << MIN_CU_LOG2 ) ) + deltaMvVerY * ( halfBH + ( h << MIN_CU_LOG2 ) );

        roundAffineMv( mvHor, mvVer, shift );

        Mv rndMv( mvHor, mvVer );
        rndMv.clipToStorageBitDepth();

        mi.mv[eRefList] = rndMv;
      }
#endif
    }
  }

  pu.mv[eRefList][0] = affLT;
  pu.mv[eRefList][1] = affRT;
  pu.mv[eRefList][2] = affLB;
}

static bool deriveScaledMotionTemporal( const Slice&      slice,
                                        const ColocatedMotionInfo&
                                                          mi,
                                        const Picture*    pColPic,
                                        const Slice*      pColSlice,
                                        const RefPicList  eCurrRefPicList,
                                              Mv&         cColMv,
                                        const RefPicList  eFetchRefPicList )
{
  CHECK( pColSlice == nullptr, "Couldn't find the colocated slice" );

  const Slice &colSlice = *pColSlice;

  int iColPOC, iColRefPOC, iCurrPOC, iCurrRefPOC, iScale;
  bool bAllowMirrorMV = true;
  RefPicList eColRefPicList = slice.getCheckLDC() ? eCurrRefPicList : RefPicList(1 - eFetchRefPicList);
  if( pColPic == slice.getRefPic( RefPicList( slice.isInterB() ? 1 - slice.getColFromL0Flag() : 0 ), slice.getColRefIdx() ) )
  {
    eColRefPicList = eCurrRefPicList;   //67 -> disable, 64 -> enable
    bAllowMirrorMV = false;
  }

  // Although it might make sense to keep the unavailable motion field per direction still be unavailable, I made the MV prediction the same way as in TMVP
  // So there is an interaction between MV0 and MV1 of the corresponding blocks identified by TV.

  // Grab motion and do necessary scaling.{{
  iCurrPOC = slice.getPOC();

  int iColRefIdx = mi.coRefIdx[eColRefPicList] - 1;

  if (iColRefIdx < 0 && (slice.getCheckLDC() || bAllowMirrorMV))
  {
    eColRefPicList = RefPicList(1 - eColRefPicList);
    iColRefIdx = mi.coRefIdx[eColRefPicList] - 1;

    if (iColRefIdx < 0)
    {
      return false;
    }
  }

  const bool bIsCurrRefLongTerm = slice.getIsUsedAsLongTerm( eCurrRefPicList, 0 );
  const bool bIsColRefLongTerm  = colSlice.getIsUsedAsLongTerm( eCurrRefPicList, iColRefIdx );
  if( bIsCurrRefLongTerm != bIsColRefLongTerm )
  {
    return false;
  }

  if (iColRefIdx >= 0 && slice.getNumRefIdx(eCurrRefPicList) > 0)
  {
    iColPOC = pColSlice->getPOC();
    iColRefPOC = pColSlice->getRefPOC(eColRefPicList, iColRefIdx);
    if (iColPOC == iColRefPOC)
      return false;
    ///////////////////////////////////////////////////////////////
    // Set the target reference index to 0, may be changed later //
    ///////////////////////////////////////////////////////////////
    iCurrRefPOC = slice.getRefPOC(eCurrRefPicList, 0);
    // Scale the vector.
    // Assume always short-term for now
    iScale = xGetDistScaleFactor(iCurrPOC, iCurrRefPOC, iColPOC, iColRefPOC);
    cColMv.setHor(roundMvComp(mi.mv[eColRefPicList].getHor()));
    cColMv.setVer(roundMvComp(mi.mv[eColRefPicList].getVer()));

    if (iScale != 4096)
    {
      cColMv = cColMv.scaleMv(iScale);
    }

    return true;
  }
  return false;
}

void clipColPos(int& posX, int& posY, const PredictionUnit& pu)
{
  Position puPos = pu.lumaPos();
  int log2CtuSize = getLog2(pu.sps->getCTUSize());
  int ctuX = ((puPos.x >> log2CtuSize) << log2CtuSize);
  int ctuY = ((puPos.y >> log2CtuSize) << log2CtuSize);
  int horMax;
  const SubPic& curSubPic = pu.pps->getSubPicFromPos( puPos );
  if( curSubPic.getTreatedAsPicFlag() )
  {
    horMax = std::min((int)curSubPic.getSubPicRight(), ctuX + (int)pu.sps->getCTUSize() + 3);
  }
  else
  {
    horMax = std::min((int)pu.pps->getPicWidthInLumaSamples() - 1, ctuX + (int)pu.sps->getCTUSize() + 3);
  }
  int horMin = std::max((int)0, ctuX);
  int verMax = std::min( (int)pu.pps->getPicHeightInLumaSamples() - 1, ctuY + (int)pu.sps->getCTUSize() - 1 );
  int verMin = std::max((int)0, ctuY);

  posX = std::min(horMax, std::max(horMin, posX));
  posY = std::min(verMax, std::max(verMin, posY));
}

bool PU::getInterMergeSubPuMvpCand(const PredictionUnit &pu, MergeCtx& mrgCtx, bool& LICFlag, const int count )
{
  const Slice   &slice = *pu.slice;

  const Picture *pColPic = slice.getRefPic(RefPicList(slice.isInterB() ? 1 - slice.getColFromL0Flag() : 0), slice.getColRefIdx());
  Mv cTMv;
  RefPicList fetchRefPicList = RefPicList(slice.isInterB() ? 1 - slice.getColFromL0Flag() : 0);

  if( count )
  {
    if(                          ( mrgCtx.interDirNeighbours[0] & ( 1 << REF_PIC_LIST_0 ) ) && slice.getRefPic( REF_PIC_LIST_0, mrgCtx.mvFieldNeighbours[REF_PIC_LIST_0].mfRefIdx ) == pColPic )
    {
      cTMv = mrgCtx.mvFieldNeighbours[REF_PIC_LIST_0].mv;
    }
    else if( slice.isInterB() && ( mrgCtx.interDirNeighbours[0] & ( 1 << REF_PIC_LIST_1 ) ) && slice.getRefPic( REF_PIC_LIST_1, mrgCtx.mvFieldNeighbours[REF_PIC_LIST_1].mfRefIdx ) == pColPic )
    {
      cTMv = mrgCtx.mvFieldNeighbours[REF_PIC_LIST_1].mv;
    }
  }

  ///////////////////////////////////////////////////////////////////////
  ////////          GET Initial Temporal Vector                  ////////
  ///////////////////////////////////////////////////////////////////////
  Mv cTempVector    = cTMv;
  bool  tempLICFlag = false;

  // compute the location of the current PU
  Position puPos  = pu.lumaPos();
  Size     puSize = pu.lumaSize();
  static constexpr int puHeight    = 1 << ATMVP_SUB_BLOCK_SIZE;
  static constexpr int puWidth     = 1 << ATMVP_SUB_BLOCK_SIZE;

  Mv cColMv;
  // use coldir.
  bool    bBSlice = slice.isInterB();

  Position centerPos;

  bool found  = false;
  cTempVector = cTMv;

  cTempVector.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
  int tempX = cTempVector.getHor();
  int tempY = cTempVector.getVer();

  centerPos.x = puPos.x + (puSize.width  >> 1) + tempX;
  centerPos.y = puPos.y + (puSize.height >> 1) + tempY;

  clipColPos( centerPos.x, centerPos.y, pu );

  // derivation of center motion parameters from the collocated CU
  const Slice* pColSlice;
  const ColocatedMotionInfo&
               mi = pColPic->cs->getColInfo( centerPos, pColSlice );

  if (mi.isInter())
  {
    mrgCtx.interDirNeighbours[count] = 0;

    for (unsigned currRefListId = 0; currRefListId < (bBSlice ? 2 : 1); currRefListId++)
    {
      RefPicList  currRefPicList = RefPicList(currRefListId);

      if (deriveScaledMotionTemporal(slice, mi, pColPic, pColSlice, currRefPicList, cColMv, fetchRefPicList))
      {
        // set as default, for further motion vector field spanning
        mrgCtx.mvFieldNeighbours[(count << 1) + currRefListId].setMvField(cColMv, 0);
        mrgCtx.interDirNeighbours[count] |= (1 << currRefListId);
        LICFlag = tempLICFlag;
        mrgCtx.BcwIdx[count] = BCW_DEFAULT;
        found = true;
      }
      else
      {
        mrgCtx.mvFieldNeighbours[(count << 1) + currRefListId].setMvField(Mv(), MF_NOT_VALID);
        mrgCtx.interDirNeighbours[count] &= ~(1 << currRefListId);
      }
    }
  }

  if( !found )
  {
    return false;
  }

  if( true )
  {
    int xOff = (puWidth >> 1) + tempX;
    int yOff = (puHeight >> 1) + tempY;

    MotionBuf& mb = mrgCtx.subPuMvpMiBuf;

    const bool isBiPred = isBipredRestriction(pu);

    MotionInfo mi;

    MotionInfo* miPtr = mb.buf;

    for( int y = puPos.y; y < puPos.y + puSize.height; y += puHeight, miPtr += g_miScaling.scaleVer( puHeight ) * mb.stride )
    {
      MotionInfo* miLinePtr = miPtr;
      for( int x = puPos.x; x < puPos.x + puSize.width; x += puWidth, miLinePtr += g_miScaling.scaleHor( puWidth ) )
      {
        mi.miRefIdx[0] = mi.miRefIdx[1] = MI_NOT_VALID;
        GCC_WARNING_DISABLE_class_memaccess
        memset( mi.mv, 0, sizeof( mi.mv ) );
        GCC_WARNING_RESET

        Position colPos{ x + xOff, y + yOff };

        clipColPos( colPos.x, colPos.y, pu );

        const Slice* pColSlice;
        const ColocatedMotionInfo&
                     colMi = pColPic->cs->getColInfo( colPos, pColSlice );

        if (colMi.isInter())
        {
          for (unsigned currRefListId = 0; currRefListId < (!isBiPred && bBSlice ? 2 : 1); currRefListId++)
          {
            RefPicList currRefPicList = RefPicList(currRefListId);
            if (deriveScaledMotionTemporal(slice, colMi, pColPic, pColSlice, currRefPicList, cColMv, fetchRefPicList))
            {
              mi.miRefIdx[currRefListId] = 1;
              mi.mv[currRefListId] = cColMv;
            }
          }
        }
        else
        {
          // intra coded, in this case, no motion vector is available for list 0 or list 1, so use default
          mi.mv[0] = mrgCtx.mvFieldNeighbours[(count << 1) + 0].mv;
          mi.mv[1] = mrgCtx.mvFieldNeighbours[(count << 1) + 1].mv;
          mi.miRefIdx[0] = mrgCtx.mvFieldNeighbours[(count << 1) + 0].mfRefIdx + 1;
          mi.miRefIdx[1] = mrgCtx.mvFieldNeighbours[(count << 1) + 1].mfRefIdx + 1;
        }

        if( isBiPred && mi.interDir() == 3 )
        {
          mi.mv[1]      = Mv();
          mi.miRefIdx[1]  = MI_NOT_VALID;
        }

        *  miLinePtr                   = mi;
        *( miLinePtr             + 1 ) = mi;
        *( miLinePtr + mb.stride     ) = mi;
        *( miLinePtr + mb.stride + 1 ) = mi;
      }
    }
  }
  return true;
}

void PU::spanMotionInfo( PredictionUnit &pu, const MergeCtx &mrgCtx )
{
  MotionBuf mb = pu.getMotionBuf();

  if( !pu.mergeFlag() || pu.mergeType() == MRG_TYPE_DEFAULT_N || pu.mergeType() == MRG_TYPE_IBC )
  {
    MotionInfo mi;

    bool isIbc = CU::isIBC( pu );

    for( int i = 0; i < NUM_REF_PIC_LIST_01; i++ )
    {
      mi.mv[i]       = pu.mv[i][0];
      mi.miRefIdx[i] = isIbc ? MI_NOT_VALID : ( pu.refIdx[i] + 1 );
    }

    if( pu.affineFlag() )
    {
      for( int y = 0; y < mb.height; y++ )
      {
        for( int x = 0; x < mb.width; x++ )
        {
          MotionInfo &dest  = mb.at( x, y );

          for( int i = 0; i < NUM_REF_PIC_LIST_01; i++ )
          {
            if( mi.miRefIdx[i] == MI_NOT_VALID )
            {
              dest.mv[i] = Mv();
            }
            dest.miRefIdx[i] = mi.miRefIdx[i];
          }
        }
      }
    }
    else
    {
      mb.fill( mi );
    }
  }
  else
  {
    // already done
  }
}

void PU::applyImv( PredictionUnit& pu, MotionHist& hist )
{
  CHECKD( pu.mergeFlag(), "IMV should never be applied to merge!" );

  Mv mvd;

  if( pu.interDir() != 2 /* PRED_L1 */ )
  {
    mvd = pu.mv[0][0];
    mvd.changePrecisionAmvr( pu.imv(), MV_PRECISION_INTERNAL );
    unsigned mvp_idx = pu.mvpIdx[0];
    AMVPInfo amvpInfo;

    if( CU::isIBC( pu ) )
      PU::fillIBCMvpCand( pu, amvpInfo, hist );
    else
      PU::fillMvpCand( pu, REF_PIC_LIST_0, pu.refIdx[0], amvpInfo, hist );

    pu.mvpIdx[0]    = mvp_idx;
    pu.mv    [0][0] = amvpInfo.mvCand[mvp_idx] + mvd;
    pu.mv    [0][0] . mvCliptoStorageBitDepth();
  }

  if( pu.interDir() != 1 /* PRED_L0 */ )
  {
    mvd = pu.mv[1][0];

    if( !( pu.cs->picHeader->getMvdL1ZeroFlag() && pu.interDir() == 3 ) && pu.imv() )/* PRED_BI */
    {
      mvd.changePrecisionAmvr( pu.imv(), MV_PRECISION_INTERNAL );
    }

    unsigned mvp_idx = pu.mvpIdx[1];
    AMVPInfo amvpInfo;
    PU::fillMvpCand( pu, REF_PIC_LIST_1, pu.refIdx[1], amvpInfo, hist );
    pu.mvpIdx[1]    = mvp_idx;
    pu.mv    [1][0] = amvpInfo.mvCand[mvp_idx] + mvd;
    pu.mv    [1][0] . mvCliptoStorageBitDepth();
  }
}


bool PU::isBiPredFromDifferentDirEqDistPoc( const PredictionUnit& pu )
{
  if( pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0 )
  {
    if( pu.slice->getRefPic(REF_PIC_LIST_0, pu.refIdx[0])->longTerm || pu.slice->getRefPic(REF_PIC_LIST_1, pu.refIdx[1])->longTerm )
    {
      return false;
    }
    const int poc0 = pu.slice->getRefPOC( REF_PIC_LIST_0, pu.refIdx[0] );
    const int poc1 = pu.slice->getRefPOC( REF_PIC_LIST_1, pu.refIdx[1] );
    const int poc  = pu.slice->getPOC();

    return ( poc - poc0 ) == ( poc1 - poc );
  }
  return false;
}

void PU::restrictBiPredMergeCandsOne( PredictionUnit &pu )
{
  if( PU::isBipredRestriction( pu ) )
  {
    if( pu.interDir() == 3 )
    {
      pu.setInterDir(  1 );
      pu.refIdx[1]  = -1;
      pu.mv[1][0]   = Mv( 0, 0 );
      pu.setBcwIdx  ( BCW_DEFAULT );
    }
  }
}

void PU::getGeoMergeCandidates( const PredictionUnit &pu, MergeCtx& geoMrgCtx, MotionHist& hist )
{
  MergeCtx tmpMergeCtx;

  const Slice &slice = *pu.slice;
  const uint32_t maxNumMergeCand = slice.getSPS()->getMaxNumMergeCand();

  geoMrgCtx.numValidMergeCand = 0;

  for (int32_t i = 0; i < GEO_MAX_NUM_UNI_CANDS; i++)
  {
    geoMrgCtx.BcwIdx[i] = BCW_DEFAULT;
    geoMrgCtx.interDirNeighbours[i] = 0;
    geoMrgCtx.mrgTypeNeighbours[i] = MRG_TYPE_DEFAULT_N;
    geoMrgCtx.mvFieldNeighbours[(i << 1)].mfRefIdx = MF_NOT_VALID;
    geoMrgCtx.mvFieldNeighbours[(i << 1) + 1].mfRefIdx = MF_NOT_VALID;
    geoMrgCtx.mvFieldNeighbours[(i << 1)].mv = Mv();
    geoMrgCtx.mvFieldNeighbours[(i << 1) + 1].mv = Mv();
    geoMrgCtx.useAltHpelIf[i] = false;
  }

  PU::getInterMergeCandidates(pu, tmpMergeCtx, hist);

  for (int32_t i = 0; i < maxNumMergeCand; i++)
  {
    int parity = i & 1;
    if( tmpMergeCtx.interDirNeighbours[i] & (0x01 + parity) )
    {
      geoMrgCtx.interDirNeighbours[geoMrgCtx.numValidMergeCand] = 1 + parity;
      geoMrgCtx.mrgTypeNeighbours[geoMrgCtx.numValidMergeCand] = MRG_TYPE_DEFAULT_N;
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + !parity].mv = Mv(0, 0);
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + parity].mv = tmpMergeCtx.mvFieldNeighbours[(i << 1) + parity].mv;
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + !parity].mfRefIdx = -1;
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + parity].mfRefIdx = tmpMergeCtx.mvFieldNeighbours[(i << 1) + parity].mfRefIdx;
      geoMrgCtx.numValidMergeCand++;
      if (geoMrgCtx.numValidMergeCand == GEO_MAX_NUM_UNI_CANDS)
      {
        return;
      }
      continue;
    }

    if (tmpMergeCtx.interDirNeighbours[i] & (0x02 - parity))
    {
      geoMrgCtx.interDirNeighbours[geoMrgCtx.numValidMergeCand] = 2 - parity;
      geoMrgCtx.mrgTypeNeighbours[geoMrgCtx.numValidMergeCand] = MRG_TYPE_DEFAULT_N;
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + !parity].mv = tmpMergeCtx.mvFieldNeighbours[(i << 1) + !parity].mv;
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + parity].mv = Mv(0, 0);
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + !parity].mfRefIdx = tmpMergeCtx.mvFieldNeighbours[(i << 1) + !parity].mfRefIdx;
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + parity].mfRefIdx = -1;
      geoMrgCtx.numValidMergeCand++;
      if (geoMrgCtx.numValidMergeCand == GEO_MAX_NUM_UNI_CANDS)
      {
        return;
      }
    }
  }
}

void PU::spanGeoMotionInfo( PredictionUnit &pu, MergeCtx &geoMrgCtx, const uint8_t splitDir, const uint8_t candIdx0, const uint8_t candIdx1)
{
  uint8_t off0 = geoMrgCtx.interDirNeighbours[candIdx0] == 1 ? 0 : 1;
  uint8_t off1 = geoMrgCtx.interDirNeighbours[candIdx1] == 1 ? 0 : 1;

  pu.mv[0][1] = geoMrgCtx.mvFieldNeighbours[( candIdx0 << 1 ) + off0].mv;
  pu.mv[1][1] = geoMrgCtx.mvFieldNeighbours[( candIdx1 << 1 ) + off1].mv;

  uint8_t val0 = geoMrgCtx.interDirNeighbours[candIdx0];
  uint8_t val1 = geoMrgCtx.interDirNeighbours[candIdx1];
  val0 <<= 4;
  val1 <<= 4;
  val0 += geoMrgCtx.mvFieldNeighbours[( candIdx0 << 1 ) + off0].mfRefIdx;
  val1 += geoMrgCtx.mvFieldNeighbours[( candIdx1 << 1 ) + off1].mfRefIdx;

  pu.setInterDirrefIdxGeo0( val0 );
  pu.setInterDirrefIdxGeo1( val1 );

  MotionBuf mb = pu.getMotionBuf();

  MotionInfo biMv;

  if( geoMrgCtx.interDirNeighbours[candIdx0] == 1 && geoMrgCtx.interDirNeighbours[candIdx1] == 2 )
  {
    biMv.mv[0]     = geoMrgCtx.mvFieldNeighbours[ candIdx0 << 1     ].mv;
    biMv.mv[1]     = geoMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].mv;
    biMv.miRefIdx[0] = geoMrgCtx.mvFieldNeighbours[ candIdx0 << 1     ].mfRefIdx + 1;
    biMv.miRefIdx[1] = geoMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].mfRefIdx + 1;
  }
  else if( geoMrgCtx.interDirNeighbours[candIdx0] == 2 && geoMrgCtx.interDirNeighbours[candIdx1] == 1 )
  {
    biMv.mv[0]     = geoMrgCtx.mvFieldNeighbours[ candIdx1 << 1     ].mv;
    biMv.mv[1]     = geoMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].mv;
    biMv.miRefIdx[0] = geoMrgCtx.mvFieldNeighbours[ candIdx1 << 1     ].mfRefIdx + 1;
    biMv.miRefIdx[1] = geoMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].mfRefIdx + 1;
  }
  else if( geoMrgCtx.interDirNeighbours[candIdx0] == 1 && geoMrgCtx.interDirNeighbours[candIdx1] == 1 )
  {
    biMv.mv[0] = geoMrgCtx.mvFieldNeighbours[candIdx1 << 1].mv;
    biMv.mv[1] = Mv(0, 0);
    biMv.miRefIdx[0] = geoMrgCtx.mvFieldNeighbours[candIdx1 << 1].mfRefIdx + 1;
    biMv.miRefIdx[1] = MI_NOT_VALID;
  }
  else if( geoMrgCtx.interDirNeighbours[candIdx0] == 2 && geoMrgCtx.interDirNeighbours[candIdx1] == 2 )
  {
    biMv.mv[0] = Mv(0, 0);
    biMv.mv[1] = geoMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].mv;
    biMv.miRefIdx[0] = MI_NOT_VALID;
    biMv.miRefIdx[1] = geoMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].mfRefIdx + 1;
  }

  int16_t angle = g_GeoParams[splitDir][0];
  int tpmMask = 0;
  int lookUpY = 0, motionIdx = 0;
  bool isFlip = angle >= 13 && angle <= 27;
  int distanceIdx = g_GeoParams[splitDir][1];
  int distanceX = angle;
  int distanceY = (distanceX + (GEO_NUM_ANGLES >> 2)) % GEO_NUM_ANGLES;
  int offsetX = (-(int)pu.lwidth()) >> 1;
  int offsetY = (-(int)pu.lheight()) >> 1;
  if (distanceIdx > 0)
  {
    if (angle % 16 == 8 || (angle % 16 != 0 && pu.lheight() >= pu.lwidth()))
      offsetY += angle < 16 ? ((distanceIdx * pu.lheight()) >> 3) : -(int)((distanceIdx * pu.lheight()) >> 3);
    else
      offsetX += angle < 16 ? ((distanceIdx * pu.lwidth()) >> 3) : -(int)((distanceIdx * pu.lwidth()) >> 3);
  }
  for (int y = 0; y < mb.height; y++)
  {
    lookUpY = (((4 * y + offsetY) << 1) + 5) * g_Dis[distanceY];
    for (int x = 0; x < mb.width; x++)
    {
      motionIdx = (((4 * x + offsetX) << 1) + 5) * g_Dis[distanceX] + lookUpY;
      tpmMask = abs(motionIdx) < 32 ? 2 : (motionIdx <= 0 ? (1 - isFlip) : isFlip);
      if (tpmMask == 2)
      {
        mb.at(x, y).miRefIdx[0] = biMv.miRefIdx[0];
        mb.at(x, y).miRefIdx[1] = biMv.miRefIdx[1];
        mb.at(x, y).mv[0] = biMv.mv[0];
        mb.at(x, y).mv[1] = biMv.mv[1];
      }
      else if (tpmMask == 0)
      {
        mb.at(x, y).miRefIdx[0] = geoMrgCtx.mvFieldNeighbours[candIdx0 << 1].mfRefIdx + 1;
        mb.at(x, y).miRefIdx[1] = geoMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].mfRefIdx + 1;
        mb.at(x, y).mv[0] = geoMrgCtx.mvFieldNeighbours[candIdx0 << 1].mv;
        mb.at(x, y).mv[1] = geoMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].mv;
      }
      else
      {
        mb.at(x, y).miRefIdx[0] = geoMrgCtx.mvFieldNeighbours[candIdx1 << 1].mfRefIdx + 1;
        mb.at(x, y).miRefIdx[1] = geoMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].mfRefIdx + 1;
        mb.at(x, y).mv[0] = geoMrgCtx.mvFieldNeighbours[candIdx1 << 1].mv;
        mb.at(x, y).mv[1] = geoMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].mv;
      }
    }
  }
}

// CU

bool CU::hasSubCUNonZeroMVd( const CodingUnit& cu )
{
  bool bNonZeroMvd = false;

  const PredictionUnit &pu = cu;

  if( pu.interDir() != 2 /* PRED_L1 */ )
  {
    bNonZeroMvd |= pu.mv[REF_PIC_LIST_0][0].getHor() != 0;
    bNonZeroMvd |= pu.mv[REF_PIC_LIST_0][0].getVer() != 0;
  }
  if( pu.interDir() != 1 /* PRED_L0 */ )
  {
    if( !pu.cs->picHeader->getMvdL1ZeroFlag() || pu.interDir() != 3 /* PRED_BI */ )
    {
      bNonZeroMvd |= pu.mv[REF_PIC_LIST_1][0].getHor() != 0;
      bNonZeroMvd |= pu.mv[REF_PIC_LIST_1][0].getVer() != 0;
    }
  }

  return bNonZeroMvd;
}

bool CU::hasSubCUNonZeroAffineMVd( const CodingUnit& cu )
{
  bool nonZeroAffineMvd = false;

  const PredictionUnit &pu = cu;

  if( pu.interDir() != 2 /* PRED_L1 */ )
  {
    for( int i = 0; !nonZeroAffineMvd && i < ( cu.affineType() == AFFINEMODEL_6PARAM ? 3 : 2 ); i++ )
    {
      nonZeroAffineMvd |= pu.mv[REF_PIC_LIST_0][i].getHor() != 0;
      nonZeroAffineMvd |= pu.mv[REF_PIC_LIST_0][i].getVer() != 0;
    }
  }

  if( !nonZeroAffineMvd && pu.interDir() != 1 /* PRED_L0 */ )
  {
    if ( !pu.cs->picHeader->getMvdL1ZeroFlag() || pu.interDir() != 3 /* PRED_BI */ )
    {
      for( int i = 0; !nonZeroAffineMvd && i < ( cu.affineType() == AFFINEMODEL_6PARAM ? 3 : 2 ); i++ )
      {
        nonZeroAffineMvd |= pu.mv[REF_PIC_LIST_1][i].getHor() != 0;
        nonZeroAffineMvd |= pu.mv[REF_PIC_LIST_1][i].getVer() != 0;
      }
    }
  }

  return nonZeroAffineMvd;
}

uint8_t CU::getSbtIdx( const uint8_t sbtInfo )
{
  return ( sbtInfo >> 0 ) & 0xf;
}

uint8_t CU::getSbtPos( const uint8_t sbtInfo )
{
  return ( sbtInfo >> 4 ) & 0x3;
}

uint8_t CU::targetSbtAllowed( uint8_t sbtIdx, uint8_t sbtAllowed )
{
  return ( sbtAllowed >> sbtIdx ) & 0x1;
}

uint8_t CU::getSbtIdx( const CodingUnit& cu )              { CHECKD( ( ( cu.sbtInfo() >> 0 ) & 0xf ) >= NUMBER_SBT_IDX, "wrong" ); return ( cu.sbtInfo() >> 0 ) & 0xf; }
uint8_t CU::getSbtPos( const CodingUnit& cu )              { return ( cu.sbtInfo() >> 4 ) & 0x3; }
void    CU::setSbtIdx(       CodingUnit& cu, uint8_t idx ) { CHECKD( idx >= NUMBER_SBT_IDX, "sbt_idx wrong" ); cu.setSbtInfo( ( idx << 0 ) + ( cu.sbtInfo() & 0xf0 ) ); }
void    CU::setSbtPos(       CodingUnit& cu, uint8_t pos ) { CHECKD( pos >= 4, "sbt_pos wrong" ); cu.setSbtInfo( ( pos << 4 ) + ( cu.sbtInfo() & 0xcf ) ); }

uint8_t CU::checkAllowedSbt( const CodingUnit& cu )
{
  //check on prediction mode
  if( !cu.slice->getSPS()->getUseSBT() || cu.predMode() != MODE_INTER || cu.ciipFlag() ) //intra or IBC or triangle
  {
    return 0;
  }

  uint8_t sbtAllowed = 0;
  int cuWidth  = cu.lwidth();
  int cuHeight = cu.lheight();

  //parameter
  const int maxSbtCUSize = cu.sps->getMaxTbSize();

  //check on size
  if( cuWidth > maxSbtCUSize || cuHeight > maxSbtCUSize )
  {
    return 0;
  }

  const int minSbtCUSize = 1 << ( MIN_CU_LOG2 + 1 );

  sbtAllowed |= ( cuWidth   >= minSbtCUSize )          << SBT_VER_HALF;
  sbtAllowed |= ( cuHeight  >= minSbtCUSize )          << SBT_HOR_HALF;
  sbtAllowed |= ( cuWidth   >= ( minSbtCUSize << 1 ) ) << SBT_VER_QUAD;
  sbtAllowed |= ( cuHeight  >= ( minSbtCUSize << 1 ) ) << SBT_HOR_QUAD;

  return sbtAllowed;
}

uint8_t CU::getSbtTuSplit( const CodingUnit& cu )
{
  uint8_t sbtTuSplitType = 0;

  switch( getSbtIdx( cu ) )
  {
  case SBT_VER_HALF: sbtTuSplitType = ( getSbtPos( cu ) == SBT_POS0 ? 0 : 1 ) + SBT_VER_HALF_POS0_SPLIT; break;
  case SBT_HOR_HALF: sbtTuSplitType = ( getSbtPos( cu ) == SBT_POS0 ? 0 : 1 ) + SBT_HOR_HALF_POS0_SPLIT; break;
  case SBT_VER_QUAD: sbtTuSplitType = ( getSbtPos( cu ) == SBT_POS0 ? 0 : 1 ) + SBT_VER_QUAD_POS0_SPLIT; break;
  case SBT_HOR_QUAD: sbtTuSplitType = ( getSbtPos( cu ) == SBT_POS0 ? 0 : 1 ) + SBT_HOR_QUAD_POS0_SPLIT; break;
  default: CHECK( true, "wrong" );  break;
  }

  CHECK( !( sbtTuSplitType <= SBT_HOR_QUAD_POS1_SPLIT && sbtTuSplitType >= SBT_VER_HALF_POS0_SPLIT ), "wrong" );
  return sbtTuSplitType;
}

static bool isMinWidthPredEnabledForBlkSize( const int w, const int h )
{
  return ( ( w == 8 && h > 4 ) || w == 4 );
}

bool CU::isPredRegDiffFromTB( const CodingUnit &cu, const ComponentID compID )
{
  return isLuma( compID ) && cu.ispMode() == VER_INTRA_SUBPARTITIONS && isMinWidthPredEnabledForBlkSize( cu.blocks[compID].width, cu.blocks[compID].height );
}

bool CU::isFirstTBInPredReg( const CodingUnit& cu, const ComponentID compID, const CompArea &area )
{
  return isLuma( compID ) && cu.ispMode() && ( ( area.topLeft().x - cu.Y().topLeft().x ) % PRED_REG_MIN_WIDTH == 0 );
}

void CU::adjustPredArea(CompArea &area)
{
  area.width = std::max<int>( PRED_REG_MIN_WIDTH, area.width );
}

PartSplit CU::getSplitAtDepth( const CodingUnit& cu, const unsigned depth )
{
  CHECK( depth >= 3, "Only works up to the split depth of '3'" );

  if( depth >= cu.depth ) return CU_DONT_SPLIT;

  const PartSplit cuSplitType = PartSplit( ( cu.splitSeries >> ( depth * SPLIT_DMULT ) ) & SPLIT_MASK );

  if     ( cuSplitType == CU_QUAD_SPLIT    ) return CU_QUAD_SPLIT;

  else if( cuSplitType == CU_HORZ_SPLIT    ) return CU_HORZ_SPLIT;

  else if( cuSplitType == CU_VERT_SPLIT    ) return CU_VERT_SPLIT;

  else if( cuSplitType == CU_TRIH_SPLIT    ) return CU_TRIH_SPLIT;
  else if( cuSplitType == CU_TRIV_SPLIT    ) return CU_TRIV_SPLIT;
  else   { THROW( "Unknown split mode"    ); return CU_QUAD_SPLIT; }
}

bool CU::checkCCLMAllowed( const CodingUnit& cu )
{
  bool allowCCLM = false;

  if( !CU::isDualITree( cu ) ) //single tree I slice or non-I slice (Note: judging chType is no longer equivalent to checking dual-tree I slice since the local dual-tree is introduced)
  {
    allowCCLM = true;
  }
  else if( cu.sps->getCTUSize() <= 32 ) //dual tree, CTUsize < 64
  {
    allowCCLM = true;
  }
  else //dual tree, CTU size 64 or 128
  {
    const int       depthFor64x64Node = cu.sps->getCTUSize() == 128 ? 1 : 0;
    const PartSplit cuSplitTypeDepth1 = CU::getSplitAtDepth( cu, depthFor64x64Node );
    const PartSplit cuSplitTypeDepth2 = CU::getSplitAtDepth( cu, depthFor64x64Node + 1 );

    //allow CCLM if 64x64 chroma tree node uses QT split or HBT+VBT split combination
    if( cuSplitTypeDepth1 == CU_QUAD_SPLIT || ( cuSplitTypeDepth1 == CU_HORZ_SPLIT && cuSplitTypeDepth2 == CU_VERT_SPLIT ) )
    {
      allowCCLM = true;
    }
    //allow CCLM if 64x64 chroma tree node uses NS (No Split) and becomes a chroma CU containing 32x32 chroma blocks
    else if( cuSplitTypeDepth1 == CU_DONT_SPLIT )
    {
      allowCCLM = true;
    }
    //allow CCLM if 64x32 chroma tree node uses NS and becomes a chroma CU containing 32x16 chroma blocks
    else if( cuSplitTypeDepth1 == CU_HORZ_SPLIT && cuSplitTypeDepth2 == CU_DONT_SPLIT )
    {
      allowCCLM = true;
    }

    //further check luma conditions
    if( allowCCLM )
    {
      //disallow CCLM if luma 64x64 block uses BT or TT or NS with ISP
      const Position lumaRefPos( cu.chromaPos().x << getComponentScaleX( COMPONENT_Cb, cu.chromaFormat ), cu.chromaPos().y << getComponentScaleY( COMPONENT_Cb, cu.chromaFormat ) );
      const CodingUnit* colLumaCu = cu.cs->getCU( lumaRefPos, CHANNEL_TYPE_LUMA );

      if( colLumaCu->depth > depthFor64x64Node && colLumaCu->qtDepth == depthFor64x64Node ) //further split at 64x64 luma node
      {
        allowCCLM = false;
      }
      else if( colLumaCu->depth == depthFor64x64Node && colLumaCu->ispMode() ) //not split at 64x64 luma node and use ISP mode
      {
        allowCCLM = false;
      }
    }
  }

  return allowCCLM;
}

bool CU::isBcwIdxCoded( const CodingUnit &cu )
{
  if( cu.sps->getUseBcw() == false )
  {
    CHECK( cu.BcwIdx() != BCW_DEFAULT, "Error: cu.BcwIdx != BCW_DEFAULT" );
    return false;
  }

  if( cu.predMode() == MODE_IBC || cu.predMode() == MODE_INTRA || cu.slice->isInterP() || cu.interDir() != 3 )
  {
    return false;
  }

  if( cu.lwidth() * cu.lheight() < BCW_SIZE_CONSTRAINT )
  {
    return false;
  }

  WPScalingParam *wp0;
  WPScalingParam *wp1;
  int refIdx0 = cu.refIdx[REF_PIC_LIST_0];
  int refIdx1 = cu.refIdx[REF_PIC_LIST_1];

  cu.slice->getWpScaling(REF_PIC_LIST_0, refIdx0, wp0);
  cu.slice->getWpScaling(REF_PIC_LIST_1, refIdx1, wp1);

  if ((wp0[COMPONENT_Y].bPresentFlag || wp0[COMPONENT_Cb].bPresentFlag || wp0[COMPONENT_Cr].bPresentFlag
    || wp1[COMPONENT_Y].bPresentFlag || wp1[COMPONENT_Cb].bPresentFlag || wp1[COMPONENT_Cr].bPresentFlag))
  {
    return false;
  }
  return true;
}

void CU::setBcwIdx( CodingUnit &cu, uint8_t uh )
{
  int8_t uhCnt = 0;

  if( cu.interDir() == 3 && !cu.mergeFlag() )
  {
    cu.setBcwIdx( uh );
    ++uhCnt;
  }
  else if( cu.interDir()== 3 && cu.mergeFlag() && cu.mergeType() == MRG_TYPE_DEFAULT_N )
  {
    // This is intended to do nothing here.
  }
  else if( cu.mergeFlag() && cu.mergeType() == MRG_TYPE_SUBPU_ATMVP )
  {
    cu.setBcwIdx( BCW_DEFAULT );
  }
  else
  {
    cu.setBcwIdx( BCW_DEFAULT );
  }

  CHECK(uhCnt <= 0, " uhCnt <= 0 ");
}


bool CU::bdpcmAllowed( const CodingUnit& cu, const ComponentID compID )
{
  const SizeType transformSkipMaxSize = 1 << cu.sps->getLog2MaxTransformSkipBlockSize();
  const Size&    blkSize              = cu.blocks[compID].size();

  bool bdpcmAllowed = cu.sps->getBDPCMEnabledFlag() &&
                    ( isLuma( compID ) || !cu.colorTransform() ) &&
                      blkSize.width <= transformSkipMaxSize &&
                      blkSize.height <= transformSkipMaxSize;

  return bdpcmAllowed;
}

bool CU::isMTSAllowed(const CodingUnit &cu, const ComponentID compID)
{
  SizeType tsMaxSize = 1 << cu.sps->getLog2MaxTransformSkipBlockSize();
  const int maxSize  = CU::isIntra( cu ) ? MTS_INTRA_MAX_CU_SIZE : MTS_INTER_MAX_CU_SIZE;
  const int cuWidth  = cu.lumaSize().width;
  const int cuHeight = cu.lumaSize().height;
  bool mtsAllowed    = cu.chType() == CHANNEL_TYPE_LUMA && compID == COMPONENT_Y;

  mtsAllowed &= CU::isIntra( cu ) ? cu.sps->getUseIntraMTS() : cu.sps->getUseInterMTS() && CU::isInter( cu );
  mtsAllowed &= cuWidth <= maxSize && cuHeight <= maxSize;
  mtsAllowed &= !cu.ispMode();
  mtsAllowed &= !cu.sbtInfo();
  mtsAllowed &= !(cu.bdpcmMode() && cuWidth <= tsMaxSize && cuHeight <= tsMaxSize);
  return mtsAllowed;
}

// TU tools

bool TU::getCbf( const TransformUnit &tu, const ComponentID &compID )
{
  return ( tu.cbf >> compID ) & 1;
}

void TU::setCbf(TransformUnit &tu, const ComponentID &compID, const bool &cbf)
{
  // first clear the CBF at the depth
  tu.cbf &= ~(1  << compID);
  // then set the CBF
  tu.cbf |= ((cbf ? 1 : 0) << compID);
}

bool TU::isTSAllowed(const TransformUnit &tu, const ComponentID compID)
{
  const int maxSize = tu.cu->sps->getLog2MaxTransformSkipBlockSize();
  bool tsAllowed = tu.cu->sps->getTransformSkipEnabledFlag();
  tsAllowed &= ( !tu.cu->ispMode() || !isLuma( compID ) );
  SizeType transformSkipMaxSize = 1 << maxSize;
  tsAllowed &= !(tu.cu->bdpcmMode() && isLuma(compID));
  tsAllowed &= !(tu.cu->bdpcmModeChroma() && isChroma(compID));
  tsAllowed &= tu.blocks[compID].width <= transformSkipMaxSize && tu.blocks[compID].height <= transformSkipMaxSize;
  tsAllowed &= !tu.cu->sbtInfo();

  return tsAllowed;
}


int TU::getICTMode( const TransformUnit& tu, bool sign )
{
  return g_ictModes[ sign ][ tu.jointCbCr ];
}



bool TU::needsSqrt2Scale( const TransformUnit &tu, const ComponentID &compID )
{
  const Size &size            = tu.blocks[compID];
  const bool  isTransformSkip = tu.mtsIdx( compID ) == 1;
  return !isTransformSkip && ( ( getLog2( size.width ) + getLog2( size.height ) ) & 1 ) == 1;
}

bool TU::needsBlockSizeTrafoScale( const TransformUnit &tu, const ComponentID &compID )
{
  return needsSqrt2Scale( tu, compID );
}

const TransformUnit* TU::getPrevTU( const TransformUnit &tu, const ComponentID compID )
{
  const TransformUnit* prevTU = nullptr;

  for( auto &currTu : cTUTraverser( &tu.cu->firstTU, &tu ) )
  {
    if( &tu == &currTu ) break;
    prevTU = &currTu;
  }

  if( prevTU != nullptr && ( prevTU->cu != tu.cu || !prevTU->blocks[compID].valid() ) )
  {
    prevTU = nullptr;
  }

  return prevTU;
}

bool TU::getPrevTUCbf( const TransformUnit &currentTu, const ComponentID compID )
{
  const TransformUnit* prevTU = getPrevTU( currentTu, compID );
  return ( prevTU != nullptr ) ? TU::getCbf( *prevTU, compID ) : false;
}


bool TU::checkTuNoResidual( TransformUnit &tu, unsigned idx )
{
  if( CU::getSbtIdx( tu.cu->sbtInfo() ) == SBT_OFF_DCT )
  {
    return false;
  }

  if( ( CU::getSbtPos( tu.cu->sbtInfo() ) == SBT_POS0 && idx == 1 ) || ( CU::getSbtPos( tu.cu->sbtInfo() ) == SBT_POS1 && idx == 0 ) )
  {
    return true;
  }

  return false;
}


int TU::getTbAreaAfterCoefZeroOut(const TransformUnit &tu, const ComponentID compID)
{
  int tbZeroOutWidth  = tu.blocks[compID].width;
  int tbZeroOutHeight = tu.blocks[compID].height;

  if( compID == COMPONENT_Y && ( tu.mtsIdx( compID ) > MTS_SKIP || ( tu.cu->sps->getUseMTS() && tu.cu->sbtInfo() != 0 && tbZeroOutWidth <= 32 && tbZeroOutHeight <= 32 ) ) )
  {
    tbZeroOutWidth  = (tbZeroOutWidth  == 32) ? 16 : tbZeroOutWidth;
    tbZeroOutHeight = (tbZeroOutHeight == 32) ? 16 : tbZeroOutHeight;
  }

  tbZeroOutWidth  = std::min<int>( JVET_C0024_ZERO_OUT_TH, tbZeroOutWidth );
  tbZeroOutHeight = std::min<int>( JVET_C0024_ZERO_OUT_TH, tbZeroOutHeight );
  return tbZeroOutWidth * tbZeroOutHeight;
}


// other tools

uint32_t getCtuAddr( const Position& pos, const PreCalcValues& pcv )
{
  return ( pos.x >> pcv.maxCUWidthLog2 ) + ( pos.y >> pcv.maxCUHeightLog2 ) * pcv.widthInCtus;
}


UnitArea getLineArea(const CodingStructure & cs, unsigned line, bool clipToPic)
{
  const unsigned widthInCtus = cs.pcv->widthInCtus;
  const unsigned maxCUWidth  = cs.pcv->maxCUWidth;
  const unsigned maxCUHeight = cs.pcv->maxCUHeight;

  if( !clipToPic )
  {
    return UnitArea( cs.area.chromaFormat, Area( 0, line * maxCUHeight, maxCUWidth * widthInCtus, maxCUHeight ) );
  }

  const unsigned width  = cs.pcv->lumaWidth;
  const unsigned height = std::min( cs.pcv->lumaHeight - line * maxCUHeight, maxCUHeight );

  return UnitArea( cs.area.chromaFormat, Area( 0, line * maxCUHeight, width, height ) );
}

UnitArea getCtuArea(const CodingStructure & cs, unsigned col, unsigned line, bool clipToPic)
{
  const unsigned maxCUWidth  = cs.pcv->maxCUWidth;
  const unsigned maxCUHeight = cs.pcv->maxCUHeight;
  const int      xPos        = col  * maxCUWidth;
  const int      yPos        = line * maxCUHeight;

  if( !clipToPic )
  {
    return UnitArea( cs.area.chromaFormat, Area( xPos, yPos, maxCUWidth, maxCUHeight ) );
  }

  CHECKD( (unsigned)xPos > cs.pcv->lumaWidth,  "Block start lies outside of the picture!" );
  CHECKD( (unsigned)yPos > cs.pcv->lumaHeight, "Block start lies outside of the picture!" );

  const unsigned width  = std::min( cs.pcv->lumaWidth  - xPos, maxCUWidth );
  const unsigned height = std::min( cs.pcv->lumaHeight - yPos, maxCUHeight );

  return UnitArea( cs.area.chromaFormat, Area( xPos, yPos, width, height ) );
}

int getNumModesMip( const Size& block )
{
  switch( getMipSizeId(block) )
  {
    case 0: return 16;
    case 1: return  8;
    case 2: return  6;
    default: THROW( "Invalid mipSizeId" );
  }
}


int getMipSizeId(const Size& block)
{
  if( block.width == 4 && block.height == 4 )
  {
    return 0;
  }
  else if( block.width == 4 || block.height == 4 || (block.width == 8 && block.height == 8) )
  {
    return 1;
  }
  else
  {
    return 2;
  }

}

bool allowLfnstWithMip( const Size& block )
{
  return block.width >= 16 && block.height >= 16;
}

bool PU::isRefPicSameSize( const PredictionUnit& pu )
{
  bool samePicSize = true;
  int curPicWidth  = pu.pps->getPicWidthInLumaSamples();
  int curPicHeight = pu.pps->getPicHeightInLumaSamples();

  if( pu.refIdx[0] >= 0 )
  {
    int refPicWidth  = pu.slice->getRefPic( REF_PIC_LIST_0, pu.refIdx[0] )->unscaledPic->cs->pps->getPicWidthInLumaSamples();
    int refPicHeight = pu.slice->getRefPic( REF_PIC_LIST_0, pu.refIdx[0] )->unscaledPic->cs->pps->getPicHeightInLumaSamples();

    samePicSize = refPicWidth == curPicWidth && refPicHeight == curPicHeight;
  }

  if( pu.refIdx[1] >= 0 )
  {
    int refPicWidth  = pu.slice->getRefPic( REF_PIC_LIST_1, pu.refIdx[1] )->unscaledPic->cs->pps->getPicWidthInLumaSamples();
    int refPicHeight = pu.slice->getRefPic( REF_PIC_LIST_1, pu.refIdx[1] )->unscaledPic->cs->pps->getPicHeightInLumaSamples();

    samePicSize = samePicSize && ( refPicWidth == curPicWidth && refPicHeight == curPicHeight );
  }

  return samePicSize;
}

bool isCrossedByVirtualBoundaries( const PicHeader* picHeader,
                                   const Area&      area,
                                   int&             numHorVirBndry,
                                   int&             numVerVirBndry,
                                   int              horVirBndryPos[],
                                   int              verVirBndryPos[] )
{
  numHorVirBndry = 0;
  numVerVirBndry = 0;
  if( !picHeader->getVirtualBoundariesPresentFlag() )
  {
    return false;
  }

  for( int i = 0; i < picHeader->getNumHorVirtualBoundaries(); i++ )
  {
    if( area.y <= picHeader->getVirtualBoundariesPosY( i ) && picHeader->getVirtualBoundariesPosY( i ) <= area.y + area.height )
    {
      horVirBndryPos[numHorVirBndry++] = picHeader->getVirtualBoundariesPosY( i );
    }
  }
  for( int i = 0; i < picHeader->getNumVerVirtualBoundaries(); i++ )
  {
    if( area.x <= picHeader->getVirtualBoundariesPosX( i ) && picHeader->getVirtualBoundariesPosX( i ) <= area.x + area.width )
    {
      verVirBndryPos[numVerVirBndry++] = picHeader->getVirtualBoundariesPosX( i );
    }
  }

  return numHorVirBndry > 0 || numVerVirBndry > 0;
}

}
