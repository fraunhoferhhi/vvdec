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

Copyright (c) 2018-2020, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. 
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
