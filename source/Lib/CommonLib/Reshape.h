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

 /** \file     Reshape.h
     \brief    reshaping header and class (header)
 */

#pragma once

#include "CommonDef.h"
#include "Buffer.h"
#include "Slice.h"

namespace vvdec
{

class CodingStructure;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

class Reshape
{
protected:
  SliceReshapeInfo        m_sliceReshapeInfo;
  Pel*                    m_invLUT;
  std::vector<int>        m_chromaAdjHelpLUT;
  std::vector<uint16_t>   m_binCW;
  uint16_t                m_initCW;
  std::vector<Pel>        m_reshapePivot;
  std::vector<Pel>        m_inputPivot;
  std::vector<Pel>        m_fwdScaleCoef;
  std::vector<Pel>        m_invScaleCoef;
  int                     m_lumaBD;
  int                     m_reshapeLUTSize;
  int                     m_chromaScale;
  int                     m_vpduX;
  int                     m_vpduY;
public:
  Reshape();
  ~Reshape();

  void createDec(int bitDepth);
  void destroy();

  void initSlice( int nalUnitLayerId, const PicHeader& picHeader, const VPS* vps );
  void rspCtuBcw( CodingStructure &cs, int col, int ln ) const;
  void rspBufFwd( PelBuf& buf ) const;

  bool getCTUFlag( const Slice& slice ) const;

  int  calculateChromaAdj(Pel avgLuma) const;
  int  getPWLIdxInv(int lumaVal) const;
  SliceReshapeInfo& getSliceReshaperInfo() { return m_sliceReshapeInfo; }
  void copySliceReshaperInfo(SliceReshapeInfo& tInfo, SliceReshapeInfo& sInfo);

  void constructReshaper();
  int  calculateChromaAdjVpduNei(TransformUnit &tu, const Position pos);
  void setVPDULoc(int x, int y) { m_vpduX = x, m_vpduY = y; }
  bool isVPDUprocessed(int x, int y) { return ((x == m_vpduX) && (y == m_vpduY)); }
  void setChromaScale (int chromaScale) { m_chromaScale = chromaScale; }
  int  getChromaScale() { return m_chromaScale; }
};// END CLASS DEFINITION Reshape

}   // namespace vvdec
