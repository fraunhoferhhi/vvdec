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

/** \file     WeightPrediction.h
    \brief    weighting prediction class (header)
*/

// Include files
#include "CommonDef.h"
#include "Unit.h"
#include "InterpolationFilter.h"
#include "WeightPrediction.h"
#include "CodingStructure.h"

namespace vvdec
{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

WeightPrediction::WeightPrediction()
{
}



void  WeightPrediction::getWpScaling(const Slice                *pcSlice,
                                     const int                  &iRefIdx0,
                                     const int                  &iRefIdx1,
                                           WPScalingParam       *wp0,
                                           WPScalingParam       *wp1)
{
  CHECK(iRefIdx0 < 0 && iRefIdx1 < 0, "Both picture reference list indizes smaller than '0'");

  const bool wpBiPred        = pcSlice->getPPS()->getWPBiPred();
  const bool bBiPred         = (iRefIdx0 >= 0 && iRefIdx1 >= 0);
  const bool bUniPred        = !bBiPred;

  const WPScalingParam* wp0org;
  const WPScalingParam* wp1org;

  if (bUniPred || wpBiPred)
  {
    // explicit --------------------
    if (iRefIdx0 >= 0)
    {
      pcSlice->getWpScaling(REF_PIC_LIST_0, iRefIdx0, wp0org);
    }
    if (iRefIdx1 >= 0)
    {
      pcSlice->getWpScaling(REF_PIC_LIST_1, iRefIdx1, wp1org);
    }
  }
  else
  {
    THROW_RECOVERABLE( "Unsupported WP configuration" );
  }

  const uint32_t numValidComponent = getNumberValidComponents(pcSlice->getSPS()->getChromaFormatIdc());

  if( iRefIdx0 < 0 )
  {
    for( int yuv = 0; yuv < numValidComponent; yuv++ )
    {
      wp0[yuv].bPresentFlag = false;
    }
  }
  if( iRefIdx1 < 0 )
  {
    for( int yuv = 0; yuv < numValidComponent; yuv++ )
    {
      wp1[yuv].bPresentFlag = false;
    }
  }

  const bool bUseHighPrecisionPredictionWeighting = false; // pcSlice->getSPS()->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag();

  if (bBiPred)
  {
    // Bi-predictive case
    for (int yuv = 0; yuv < numValidComponent; yuv++)
    {
      const int bitDepth = pcSlice->getSPS()->getBitDepth();
      const int offsetScalingFactor = bUseHighPrecisionPredictionWeighting ? 1 : (1 << (bitDepth - 8));

      wp0[yuv] = wp0org[yuv];
      wp1[yuv] = wp1org[yuv];

      wp0[yuv].w = wp0org[yuv].iWeight;
      wp1[yuv].w = wp1org[yuv].iWeight;
      wp0[yuv].o = wp0org[yuv].iOffset * offsetScalingFactor;
      wp1[yuv].o = wp1org[yuv].iOffset * offsetScalingFactor;
      wp0[yuv].offset = wp1[yuv].offset = wp0[yuv].o + wp1[yuv].o;
      wp0[yuv].shift  = wp1[yuv].shift  = wp0org[yuv].uiLog2WeightDenom + 1;
      wp0[yuv].round  = wp1[yuv].round  = ( 1 << wp0org[yuv].uiLog2WeightDenom );
    }
  }
  else
  {
    // UniPred
    const WPScalingParam* const pwporg = ( iRefIdx0 >= 0 ) ? wp0org : wp1org;
          WPScalingParam* const pwp    = ( iRefIdx0 >= 0 ) ? wp0 : wp1;

    for (int yuv = 0; yuv < numValidComponent; yuv++)
    {
      const int bitDepth            = pcSlice->getSPS()->getBitDepth();
      const int offsetScalingFactor = bUseHighPrecisionPredictionWeighting ? 1 : (1 << (bitDepth - 8));
      
      pwp[yuv]        = pwporg[yuv];

      pwp[yuv].w      = pwporg[yuv].iWeight;
      pwp[yuv].offset = pwporg[yuv].iOffset * offsetScalingFactor;
      pwp[yuv].shift  = pwporg[yuv].uiLog2WeightDenom;
      pwp[yuv].round  = ( pwporg[yuv].uiLog2WeightDenom >= 1 ) ? ( 1 << ( pwporg[yuv].uiLog2WeightDenom - 1 ) ) : ( 0 );
    }
  }
}

static inline Pel weightBidir( int w0, Pel P0, int w1, Pel P1, int round, int shift, int offset, const ClpRng& clpRng )
{
  return ClipPel( ( ( w0*( P0 + IF_INTERNAL_OFFS ) + w1 * ( P1 + IF_INTERNAL_OFFS ) + round + ( offset * ( 1 << ( shift - 1 ) ) ) ) >> shift ), clpRng );
}

void WeightPrediction::addWeightBi(const PelUnitBuf           &pcYuvSrc0,
                                   const PelUnitBuf           &pcYuvSrc1,
                                   const ClpRngs              &clpRngs,
                                   const WPScalingParam *const wp0,
                                   const WPScalingParam *const wp1,
                                         PelUnitBuf           &rpcYuvDst
                                  )
{
  const uint32_t numValidComponent = (const uint32_t)pcYuvSrc0.bufs.size();

  for (int componentIndex = 0; componentIndex < numValidComponent; componentIndex++)
  {
    const ComponentID compID = ComponentID(componentIndex);

    const Pel* pSrc0 = pcYuvSrc0.bufs[compID].buf;
    const Pel* pSrc1 = pcYuvSrc1.bufs[compID].buf;
          Pel* pDst  = rpcYuvDst.bufs[compID].buf;

    const ptrdiff_t iSrc0Stride = pcYuvSrc0.bufs[compID].stride;
    const ptrdiff_t iSrc1Stride = pcYuvSrc1.bufs[compID].stride;
    const ptrdiff_t iDstStride  = rpcYuvDst.bufs[compID].stride;

    const ClpRng& clpRng = clpRngs;
    const int  w0       = wp0[compID].w;
    const int  offset   = wp0[compID].offset;
    const int  clipBD   = clpRng.bd;
    const int  shiftNum = std::max<int>(2, (IF_INTERNAL_PREC - clipBD));
    const int  shift    = wp0[compID].shift + shiftNum;
    const int  round    = 1 << shift >> 1;
    const int  w1       = wp1[compID].w;

    const int  iHeight  = rpcYuvDst.bufs[compID].height;
    const int  iWidth   = rpcYuvDst.bufs[compID].width;
    const int  applyOffset = round + ( offset * ( 1 << ( shift - 1 ) ) ) + ( w0 + w1 ) * IF_INTERNAL_OFFS;

    if( ( iWidth & 7 ) == 0 )
    {
      g_pelBufOP.wghtAvg8( pSrc0, iSrc0Stride, pSrc1, iSrc1Stride, pDst, iDstStride, iWidth, iHeight, shift, applyOffset, w0, w1, clpRngs );
    }
    else if( ( iWidth & 3 ) == 0 )
      g_pelBufOP.wghtAvg4( pSrc0, iSrc0Stride, pSrc1, iSrc1Stride, pDst, iDstStride, iWidth, iHeight, shift, applyOffset, w0, w1, clpRngs );
    else
    {
      CHECK( iWidth != 2, "Should only happen for width '2'" );

      for (int y = iHeight - 1; y >= 0; y--)
      {
        pDst[0] = weightBidir(w0, pSrc0[0], w1, pSrc1[0], round, shift, offset, clpRng );;
        pDst[1] = weightBidir(w0, pSrc0[1], w1, pSrc1[1], round, shift, offset, clpRng );;

        pSrc0 += iSrc0Stride;
        pSrc1 += iSrc1Stride;
        pDst += iDstStride;
      } // y loop
    }
  } // compID loop
}


static inline Pel weightUnidir( int w0, Pel P0, int round, int shift, int offset, const ClpRng& clpRng )
{
  return ClipPel( ( ( w0*( P0 + IF_INTERNAL_OFFS ) + round ) >> shift ) + offset, clpRng );
}

static inline Pel noWeightUnidir( Pel P0, int round, int shift, int offset, const ClpRng& clpRng )
{
  return ClipPel( ( ( ( P0 + IF_INTERNAL_OFFS ) + round ) >> shift ) + offset, clpRng );
}

static inline Pel noWeightOffsetUnidir( Pel P0, int round, int shift, const ClpRng& clpRng )
{
  return ClipPel( ( ( ( P0 + IF_INTERNAL_OFFS ) + round ) >> shift ), clpRng );
}

void  WeightPrediction::addWeightUni(const PelUnitBuf           &pcYuvSrc0,
                                     const ClpRngs              &clpRngs,
                                     const WPScalingParam *const wp0,
                                           PelUnitBuf           &rpcYuvDst
                                    )
{
  const uint32_t numValidComponent = (const uint32_t)pcYuvSrc0.bufs.size();

  for (int componentIndex = 0; componentIndex < numValidComponent; componentIndex++)
  {
    const ComponentID compID = ComponentID(componentIndex);

    const Pel* pSrc0 = pcYuvSrc0.bufs[compID].buf;
          Pel* pDst  = rpcYuvDst.bufs[compID].buf;

    // Luma : --------------------------------------------
    const ClpRng& clpRng    = clpRngs;
    const int  w0           = wp0[compID].w;
    const int  offset       = wp0[compID].offset;
    const int  clipBD       = clpRng.bd;
    const int  shiftNum     = std::max<int>(2, (IF_INTERNAL_PREC - clipBD));
    const int  shift        = wp0[compID].shift + shiftNum;
    const ptrdiff_t iSrc0Stride  = pcYuvSrc0.bufs[compID].stride;
    const ptrdiff_t iDstStride   = rpcYuvDst.bufs[compID].stride;
    const int  iHeight      = rpcYuvDst.bufs[compID].height;
    const int  iWidth       = rpcYuvDst.bufs[compID].width;

    if (w0 != 1 << wp0[compID].shift)
    {
      const int  round = (shift > 0) ? (1 << (shift - 1)) : 0;
      for (int y = iHeight - 1; y >= 0; y--)
      {
        int x = iWidth - 1;
        for (; x >= 3; )
        {
          pDst[x] = weightUnidir(w0, pSrc0[x], round, shift, offset, clpRng); x--;
          pDst[x] = weightUnidir(w0, pSrc0[x], round, shift, offset, clpRng); x--;
          pDst[x] = weightUnidir(w0, pSrc0[x], round, shift, offset, clpRng); x--;
          pDst[x] = weightUnidir(w0, pSrc0[x], round, shift, offset, clpRng); x--;
        }
        for (; x >= 0; x--)
        {
          pDst[x] = weightUnidir(w0, pSrc0[x], round, shift, offset, clpRng);
        }
        pSrc0 += iSrc0Stride;
        pDst += iDstStride;
      }
    }
    else
    {
      const int  round = (shiftNum > 0) ? (1 << (shiftNum - 1)) : 0;
      if (offset == 0)
      {
        for (int y = iHeight - 1; y >= 0; y--)
        {
          int x = iWidth - 1;
          for (; x >= 3; )
          {
            pDst[x] = noWeightOffsetUnidir(pSrc0[x], round, shiftNum, clpRng); x--;
            pDst[x] = noWeightOffsetUnidir(pSrc0[x], round, shiftNum, clpRng); x--;
            pDst[x] = noWeightOffsetUnidir(pSrc0[x], round, shiftNum, clpRng); x--;
            pDst[x] = noWeightOffsetUnidir(pSrc0[x], round, shiftNum, clpRng); x--;
          }
          for (; x >= 0; x--)
          {
            pDst[x] = noWeightOffsetUnidir(pSrc0[x], round, shiftNum, clpRng);
          }
          pSrc0 += iSrc0Stride;
          pDst += iDstStride;
        }
      }
      else
      {
        for (int y = iHeight - 1; y >= 0; y--)
        {
          int x = iWidth - 1;
          for (; x >= 3; )
          {
            pDst[x] = noWeightUnidir(pSrc0[x], round, shiftNum, offset, clpRng); x--;
            pDst[x] = noWeightUnidir(pSrc0[x], round, shiftNum, offset, clpRng); x--;
            pDst[x] = noWeightUnidir(pSrc0[x], round, shiftNum, offset, clpRng); x--;
            pDst[x] = noWeightUnidir(pSrc0[x], round, shiftNum, offset, clpRng); x--;
          }
          for (; x >= 0; x--)
          {
            pDst[x] = noWeightUnidir(pSrc0[x], round, shiftNum, offset, clpRng);
          }
          pSrc0 += iSrc0Stride;
          pDst += iDstStride;
        }
      }
    }
  }
}

void  WeightPrediction::xWeightedPredictionUni(const CodingUnit       &cu,
                                               const PelUnitBuf           &pcYuvSrc,
                                               const RefPicList           &eRefPicList,
                                                     PelUnitBuf           &pcYuvPred,
                                               const int                   iRefIdx_input/* = -1*/
                                              )
{
  WPScalingParam  pwp[MAX_NUM_COMPONENT], pwpTmp[MAX_NUM_COMPONENT];

  int iRefIdx = iRefIdx_input;
  if (iRefIdx < 0)
  {
    iRefIdx = cu.refIdx[eRefPicList];
  }

  CHECK(iRefIdx < 0, "Negative reference picture list index");

  if (eRefPicList == REF_PIC_LIST_0)
  {
    getWpScaling(cu.slice, iRefIdx, -1, pwp, pwpTmp);
  }
  else
  {
    getWpScaling(cu.slice, -1, iRefIdx, pwpTmp, pwp);
  }
  addWeightUni(pcYuvSrc, cu.slice->clpRngs(), pwp, pcYuvPred);
}

void  WeightPrediction::xWeightedPredictionBi(const CodingUnit       &cu,
                                              const PelUnitBuf           &pcYuvSrc0,
                                              const PelUnitBuf           &pcYuvSrc1,
                                                    PelUnitBuf           &rpcYuvDst
                                             )
{
  const int iRefIdx0 = cu.refIdx[0];
  const int iRefIdx1 = cu.refIdx[1];
  WPScalingParam  pwp0[MAX_NUM_COMPONENT];
  WPScalingParam  pwp1[MAX_NUM_COMPONENT];

  CHECK( !cu.pps->getWPBiPred(), "Weighted Bi-prediction disabled" );

  getWpScaling(cu.slice, iRefIdx0, iRefIdx1, pwp0, pwp1);

  if (iRefIdx0 >= 0 && iRefIdx1 >= 0)
  {
    addWeightBi(pcYuvSrc0, pcYuvSrc1, cu.slice->clpRngs(), pwp0, pwp1, rpcYuvDst);
  }
  else if (iRefIdx0 >= 0 && iRefIdx1 < 0)
  {
    addWeightUni(pcYuvSrc0, cu.slice->clpRngs(), pwp0, rpcYuvDst);
  }
  else if (iRefIdx0 < 0 && iRefIdx1 >= 0)
  {
    addWeightUni(pcYuvSrc1, cu.slice->clpRngs(), pwp1, rpcYuvDst);
  }
  else
  {
    THROW_RECOVERABLE( "Both reference picture list indices are negative" );
  }
}

}
