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

/** \file     Mv.cpp
    \brief    motion vector class
*/

#include "Mv.h"

#include "Common.h"
#include "Slice.h"

const MvPrecision Mv::m_amvrPrecision[4] = { MV_PRECISION_QUARTER, MV_PRECISION_INT, MV_PRECISION_4PEL, MV_PRECISION_HALF }; // for cu.imv=0, 1, 2 and 3

void roundAffineMv( int& mvx, int& mvy, int nShift )
{
  const int nOffset = 1 << (nShift - 1);
  mvx = (mvx + nOffset - (mvx >= 0)) >> nShift;
  mvy = (mvy + nOffset - (mvy >= 0)) >> nShift;
}

#if JVET_R0058
void clipMvInPic ( Mv& rcMv, const Position& pos, const struct Size& size, const SPS& sps, const PPS& pps )
{
  if( sps.getUseWrapAround() )
  {
    wrapClipMv( rcMv, pos, size, sps, pps );
    return;
  }

  int mvShift = MV_FRACTIONAL_BITS_INTERNAL;
  int offset = 8;
  int horMax = (pps.getPicWidthInLumaSamples() + offset - (int)pos.x - 1) << mvShift;
  int horMin = (-(int)sps.getMaxCUWidth() - offset - (int)pos.x + 1) << mvShift;

  int verMax = (pps.getPicHeightInLumaSamples() + offset - (int)pos.y - 1) << mvShift;
  int verMin = (-(int)sps.getMaxCUHeight() - offset - (int)pos.y + 1) << mvShift;

  rcMv.setHor(std::min(horMax, std::max(horMin, rcMv.getHor())));
  rcMv.setVer(std::min(verMax, std::max(verMin, rcMv.getVer())));
}

void clipMvInSubpic ( Mv& rcMv, const struct Position& pos, const struct Size& size, const SPS& sps, const PPS& pps )
{
  if( sps.getUseWrapAround() )
  {
    wrapClipMv( rcMv, pos, size, sps, pps );
    return;
  }

  int mvShift = MV_FRACTIONAL_BITS_INTERNAL;
  int offset = 8;
  int horMax = (pps.getPicWidthInLumaSamples() + offset - (int)pos.x - 1) << mvShift;
  int horMin = (-(int)sps.getMaxCUWidth() - offset - (int)pos.x + 1) << mvShift;

  int verMax = (pps.getPicHeightInLumaSamples() + offset - (int)pos.y - 1) << mvShift;
  int verMin = (-(int)sps.getMaxCUHeight() - offset - (int)pos.y + 1) << mvShift;
  const SubPic& curSubPic = pps.getSubPicFromPos(pos);
  if (curSubPic.getTreatedAsPicFlag())
  {
    horMax = ((curSubPic.getSubPicRight() + 1) + offset - (int)pos.x - 1) << mvShift;
    horMin = (-(int)sps.getMaxCUWidth() - offset - ((int)pos.x - curSubPic.getSubPicLeft()) + 1) << mvShift;

    verMax = ((curSubPic.getSubPicBottom() + 1) + offset - (int)pos.y - 1) << mvShift;
    verMin = (-(int)sps.getMaxCUHeight() - offset - ((int)pos.y - curSubPic.getSubPicTop()) + 1) << mvShift;
  }
  rcMv.setHor(std::min(horMax, std::max(horMin, rcMv.getHor())));
  rcMv.setVer(std::min(verMax, std::max(verMin, rcMv.getVer())));
}

bool wrapClipMv( Mv& rcMv, const Position& pos, const struct Size& size, const SPS& sps, const PPS& pps )
{
  bool wrapRef = true;
  int iMvShift = MV_FRACTIONAL_BITS_INTERNAL;
  int iOffset = 8;
  int iHorMax = ( pps.getPicWidthInLumaSamples() + sps.getMaxCUWidth() - size.width + iOffset - (int)pos.x - 1 ) << iMvShift;
  int iHorMin = ( -( int ) sps.getMaxCUWidth()                                      - iOffset - ( int ) pos.x + 1 ) << iMvShift;
  int iVerMax = ( pps.getPicHeightInLumaSamples() + iOffset - (int)pos.y - 1 ) << iMvShift;
  int iVerMin = ( -( int ) sps.getMaxCUHeight()   - iOffset - ( int ) pos.y + 1 ) << iMvShift;
  int mvX = rcMv.getHor();

  if(mvX > iHorMax)
  {
#if JVET_Q0764_WRAP_AROUND_WITH_RPR
    mvX -= ( pps.getWrapAroundOffset() << iMvShift );
#else
    mvX -= ( sps.getWrapAroundOffset() << iMvShift );
#endif
    mvX = std::min( iHorMax, std::max( iHorMin, mvX ) );
    wrapRef = false;
  }
  if(mvX < iHorMin)
  {
#if JVET_Q0764_WRAP_AROUND_WITH_RPR
    mvX += ( pps.getWrapAroundOffset() << iMvShift );
#else
    mvX += ( sps.getWrapAroundOffset() << iMvShift );
#endif
    mvX = std::min( iHorMax, std::max( iHorMin, mvX ) );
    wrapRef = false;
  }

  rcMv.setHor( mvX );
  rcMv.setVer( std::min( iVerMax, std::max( iVerMin, rcMv.getVer() ) ) );
  return wrapRef;
}

#else
void clipMv( Mv& rcMv, const Position& pos, const SPS& sps, const PPS& pps, const int w, const int h )
{
  clipMv( rcMv.hor, rcMv.ver, pos, sps, pps, w, h );
}

void clipMv( int &mvx, int &mvy, const Position& pos, const SPS& sps, const PPS& pps, const int w, const int h )
{
#if JVET_Q0764_WRAP_AROUND_WITH_RPR
  if( pps.getUseWrapAround() )
#else
  if( sps.getUseWrapAround() )
 #endif
  {
    wrapClipMv( mvx, mvy, pos, Size( w, h ), sps, pps );
    return;
  }

  const int iMvShift = MV_FRACTIONAL_BITS_INTERNAL;
  const int iOffset = 8;

  int iHorMax = ( pps.getPicWidthInLumaSamples() + iOffset - ( int ) pos.x - 1 ) << iMvShift;
  int iHorMin = ( -( int ) sps.getMaxCUWidth()   - iOffset - ( int ) pos.x + 1 ) << iMvShift;

  int iVerMax = ( pps.getPicHeightInLumaSamples() + iOffset - ( int ) pos.y - 1 ) << iMvShift;
  int iVerMin = ( -( int ) sps.getMaxCUHeight()   - iOffset - ( int ) pos.y + 1 ) << iMvShift;
#if JVET_O1143_MV_ACROSS_SUBPIC_BOUNDARY 
  
  if (sps.getSubPicInfoPresentFlag())
  {
    const SubPic& curSubPic = pps.getSubPicFromPos(pos);
    if( curSubPic.getTreatedAsPicFlag() )
    {
      iHorMax = (curSubPic.getSubPicWidthInLumaSample() + iOffset - (int)pos.x - 1 ) << iMvShift;
      iHorMin = (-(int)sps.getMaxCUWidth() -  iOffset - ((int)pos.x - curSubPic.getSubPicLeft()) + 1) << iMvShift;

      iVerMax = (curSubPic.getSubPicHeightInLumaSample()+ iOffset - (int)pos.y - 1) << iMvShift;
      iVerMin = (-(int)sps.getMaxCUHeight() - iOffset - ((int)pos.y - curSubPic.getSubPicTop()) + 1) << iMvShift;
    }
  }
#endif
  mvx = std::min( iHorMax, std::max( iHorMin, mvx ) );
  mvy = std::min( iVerMax, std::max( iVerMin, mvy ) );
}

bool wrapClipMv( Mv& rcMv, const Position& pos, const struct Size& size, const SPS& sps, const PPS& pps )
{
  return wrapClipMv( rcMv.hor, rcMv.ver, pos, size, sps, pps );
}

bool wrapClipMv( int &mvx, int &mvy, const Position& pos, const struct Size& size, const SPS& sps, const PPS& pps )
{
  const int iMvShift = MV_FRACTIONAL_BITS_INTERNAL;
  const int iOffset = 8;

  const int iHorMax = ( pps.getPicWidthInLumaSamples() + sps.getMaxCUWidth() - size.width + iOffset - ( int ) pos.x - 1 ) << iMvShift;
  const int iHorMin = ( -( int ) sps.getMaxCUWidth()                                      - iOffset - ( int ) pos.x + 1 ) << iMvShift;

  const int iVerMax = ( pps.getPicHeightInLumaSamples() + iOffset - ( int ) pos.y - 1 ) << iMvShift;
  const int iVerMin = ( -( int ) sps.getMaxCUHeight()   - iOffset - ( int ) pos.y + 1 ) << iMvShift;

  bool wrapRef = true;
  int mvX = mvx;
  if(mvX > iHorMax)
  {
    mvX -= ( sps.getWrapAroundOffset() << iMvShift );
    mvX = std::min( iHorMax, std::max( iHorMin, mvX ) );
    wrapRef = false;
  }
  else if(mvX < iHorMin)
  {
    mvX += ( sps.getWrapAroundOffset() << iMvShift );
    mvX = std::min( iHorMax, std::max( iHorMin, mvX ) );
    wrapRef = false;
  }

  mvx = mvX;
  mvy =  std::min( iVerMax, std::max( iVerMin, mvy ) );
  return wrapRef;
}
#endif
//! \}
