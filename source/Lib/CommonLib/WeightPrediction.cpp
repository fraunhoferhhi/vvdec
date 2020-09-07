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

/** \file     WeightPrediction.h
    \brief    weighting prediction class (header)
*/

// Include files
#include "CommonDef.h"
#include "Unit.h"
#include "InterpolationFilter.h"
#include "WeightPrediction.h"
#include "CodingStructure.h"


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

  WPScalingParam* wp0org;
  WPScalingParam* wp1org;

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
    THROW( "Unsupported WP configuration" );
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

  const bool bUseHighPrecisionPredictionWeighting = pcSlice->getSPS()->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag();

  if (bBiPred)
  {
    // Bi-predictive case
    for (int yuv = 0; yuv < numValidComponent; yuv++)
    {
      const int bitDepth = pcSlice->getSPS()->getBitDepth(toChannelType(ComponentID(yuv)));
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
    WPScalingParam *const pwporg = (iRefIdx0 >= 0) ? wp0org : wp1org;
    WPScalingParam *const pwp    = (iRefIdx0 >= 0) ? wp0 : wp1;

    for (int yuv = 0; yuv < numValidComponent; yuv++)
    {
      const int bitDepth            = pcSlice->getSPS()->getBitDepth(toChannelType(ComponentID(yuv)));
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
  return ClipPel( ( ( w0*( P0 + IF_INTERNAL_OFFS ) + w1 * ( P1 + IF_INTERNAL_OFFS ) + round + ( offset << ( shift - 1 ) ) ) >> shift ), clpRng );
}

void WeightPrediction::addWeightBi(const PelUnitBuf           &pcYuvSrc0,
                                   const PelUnitBuf           &pcYuvSrc1,
                                   const ClpRngs              &clpRngs,
                                   const WPScalingParam *const wp0,
                                   const WPScalingParam *const wp1,
                                         PelUnitBuf           &rpcYuvDst,
                                   const bool                  bRoundLuma /*= true*/
                                  )
{
  const bool enableRounding[MAX_NUM_COMPONENT] = { bRoundLuma, true, true };

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
    const int  round    = (enableRounding[compID] && (shift > 0)) ? (1 << (shift - 1)) : 0;
    const int  w1       = wp1[compID].w;
    const int  applyOffset = round + ( offset << ( shift - 1 ) ) + ( w0 + w1 ) * IF_INTERNAL_OFFS;

    const int  iHeight  = rpcYuvDst.bufs[compID].height;
    const int  iWidth   = rpcYuvDst.bufs[compID].width;

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

void  WeightPrediction::xWeightedPredictionUni(const PredictionUnit       &pu,
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
    iRefIdx = pu.refIdx[eRefPicList];
  }

  CHECK(iRefIdx < 0, "Negative reference picture list index");

  if (eRefPicList == REF_PIC_LIST_0)
  {
    getWpScaling(pu.slice, iRefIdx, -1, pwp, pwpTmp);
  }
  else
  {
    getWpScaling(pu.slice, -1, iRefIdx, pwpTmp, pwp);
  }
  addWeightUni(pcYuvSrc, pu.slice->clpRngs(), pwp, pcYuvPred);
}

void  WeightPrediction::xWeightedPredictionBi(const PredictionUnit       &pu,
                                              const PelUnitBuf           &pcYuvSrc0,
                                              const PelUnitBuf           &pcYuvSrc1,
                                                    PelUnitBuf           &rpcYuvDst
                                             )
{
  const int iRefIdx0 = pu.refIdx[0];
  const int iRefIdx1 = pu.refIdx[1];
  WPScalingParam  pwp0[MAX_NUM_COMPONENT];
  WPScalingParam  pwp1[MAX_NUM_COMPONENT];

  CHECK( !pu.cs->pps->getWPBiPred(), "Weighted Bi-prediction disabled" );

  getWpScaling(pu.slice, iRefIdx0, iRefIdx1, pwp0, pwp1);

  if (iRefIdx0 >= 0 && iRefIdx1 >= 0)
  {
    addWeightBi(pcYuvSrc0, pcYuvSrc1, pu.slice->clpRngs(), pwp0, pwp1, rpcYuvDst, true);
  }
  else if (iRefIdx0 >= 0 && iRefIdx1 < 0)
  {
    addWeightUni(pcYuvSrc0, pu.slice->clpRngs(), pwp0, rpcYuvDst);
  }
  else if (iRefIdx0 < 0 && iRefIdx1 >= 0)
  {
    addWeightUni(pcYuvSrc1, pu.slice->clpRngs(), pwp1, rpcYuvDst);
  }
  else
  {
    THROW( "Both reference picture list indizes are negative" );
  }
}
