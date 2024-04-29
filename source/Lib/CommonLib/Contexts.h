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

/** \file     Contexts.h
 *  \brief    Classes providing probability descriptions and contexts (header)
 */

#pragma once

#include "CommonDef.h"

#include <array>
#include <vector>

namespace vvdec
{

static constexpr int     PROB_BITS   = 15;   // Nominal number of bits to represent probabilities
static constexpr int     PROB_BITS_0 = 10;   // Number of bits to represent 1st estimate
static constexpr int     PROB_BITS_1 = 14;   // Number of bits to represent 2nd estimate
static constexpr int     MASK_0      = ~(~0u << PROB_BITS_0) << (PROB_BITS - PROB_BITS_0);
static constexpr int     MASK_1      = ~(~0u << PROB_BITS_1) << (PROB_BITS - PROB_BITS_1);
static constexpr uint8_t DWS         = 8;   // 0x47 Default window sizes
static constexpr uint8_t CNU         = 35;

class ProbModelTables
{
protected:
  static const uint8_t      m_RenormTable_32  [ 32];          // Std         MP   MPI
};

class BinProbModel : public ProbModelTables
{
public:
  BinProbModel()
  {
    uint16_t half = 1 << (PROB_BITS - 1);
    m_state[0]    = half;
    m_state[1]    = half;
    m_rate0 = 0;
    m_rate1 = DWS;
    m_delta0[0] = ((0xFFFFU) >> (16 - m_rate0));
    m_delta0[1] = ((0xFFFFU) >> (16 - PROB_BITS));
    m_delta1[0] = ((0xFFFFU) >> (16 - m_rate1));
    m_delta1[1] = ((0xFFFFU) >> (16 - PROB_BITS));
  }
  ~BinProbModel() {}
public:
  void            init              ( int qp, int initId );
  void update(unsigned bin)
  {
//    m_state[0] -= (m_state[0] >> m_rate0) & MASK_0;
//    m_state[1] -= (m_state[1] >> m_rate1) & MASK_1;
//    m_state[0] += ( ( bin ? 0x7fffu : 0x0u ) >> m_rate0 ) & MASK_0;
//    m_state[1] += ( ( bin ? 0x7fffu : 0x0u ) >> m_rate1 ) & MASK_1;
    int delta0 = m_delta0[bin] - m_state[0];
    int delta1 = m_delta1[bin] - m_state[1];
    delta0 = delta0 >> m_rate0;
    delta1 = delta1 >> m_rate1;
    m_state[0] += delta0 *(1<< 5);
    m_state[1] += delta1 *2;
  }
  void setLog2WindowSize(uint8_t log2WindowSize)
  {
    m_rate0 = 2 + ((log2WindowSize >> 2) & 3);
    m_rate1 = 3 + m_rate0 + (log2WindowSize & 3);
    CHECK(m_rate1 > 9, "Second window size is too large!");
    m_rate0 += 5;
    m_rate1 += 1;
    m_delta0[0] = ((0xFFFFU) >> (16 - m_rate0));
    m_delta1[0] = ((0xFFFFU) >> (16 - m_rate1));
  }
public:
  uint8_t state() const { return (m_state[0] + m_state[1]) >> 8; }
  uint8_t mps() const { return state() >> 7; }
  uint8_t lps( unsigned range ) const
  {
    uint16_t q = state();
    if (q & 0x80)
      q = q ^ 0xff;
    return ((q >> 2) * (range >> 5) >> 1) + 4;
  }
  void lpsmps( unsigned range, unsigned& lps, unsigned& bin ) const
  {
    const uint16_t q = (m_state[0] + m_state[1]) >> 8;
    bin = q >> 7;
    
    lps = ((((q ^ (-static_cast<int>(bin) & 0xff)) >> 2) * (range >> 5)) >> 1) + 4;
    
    //if( q & 0x80 )
    //{
    //  CHECK( lps != ( ( ( ( q ^ 0xff ) >> 2 ) * ( range >> 5 ) >> 1 ) + 4 ), "" );
    //  lps = ( ( ( q ^ 0xff ) >> 2 ) * ( range >> 5 ) >> 1 ) + 4;
    //}
    //else
    //{
    //  CHECK( lps != ( ( ( q >> 2 ) * ( range >> 5 ) >> 1 ) + 4 ), "" );
    //  lps = ( ( q >> 2 ) * ( range >> 5 ) >> 1 ) + 4;
    //}
  }
  static uint8_t  getRenormBitsLPS  ( unsigned LPS )                    { return    m_RenormTable_32  [LPS>>3]; }
  static uint8_t  getRenormBitsRange( unsigned range )                  { return    1; }
private:
  uint16_t m_state[2];
  uint16_t m_rate0;
  uint16_t m_rate1;
  uint16_t m_delta0[2];
  uint16_t m_delta1[2];
};


class CtxSet
{
public:
  CtxSet( uint16_t offset, uint16_t size ) : Offset( offset ), Size( size ) {}
  CtxSet( std::initializer_list<CtxSet> ctxSets );

  uint16_t operator()() const
  {
    return Offset;
  }
  uint16_t operator()( uint16_t inc ) const
  {
    CHECKD( inc >= Size, "Specified context increment (" << inc << ") exceed range of context set [0;" << Size - 1 << "]." );

    return Offset + inc;
  }

private:
  uint16_t Offset;
  uint16_t Size;
};


class ContextSetCfg
{
public:
  // context sets: specify offset and size
  static const CtxSet   SplitFlag;
  static const CtxSet   SplitQtFlag;
  static const CtxSet   SplitHvFlag;
  static const CtxSet   Split12Flag;
  static const CtxSet   ModeConsFlag;
  static const CtxSet   SkipFlag;
  static const CtxSet   MergeFlag;
  static const CtxSet   RegularMergeFlag;
  static const CtxSet   MergeIdx;
  static const CtxSet   PredMode;
  static const CtxSet   MultiRefLineIdx;
  static const CtxSet   IntraLumaPlanarFlag;
  static const CtxSet   CclmModeFlag;
  static const CtxSet   CclmModeIdx;
  static const CtxSet   IPredMode       [2];    // [ ChannelType ]
  static const CtxSet   MipFlag;
  static const CtxSet   DeltaQP;
  static const CtxSet   InterDir;
  static const CtxSet   RefPic;
  static const CtxSet   MmvdFlag;
  static const CtxSet   MmvdMergeIdx;
  static const CtxSet   MmvdStepMvpIdx;
  static const CtxSet   SubblockMergeFlag;
  static const CtxSet   AffineFlag;
  static const CtxSet   AffineType;
  static const CtxSet   AffMergeIdx;
  static const CtxSet   Mvd;
  static const CtxSet   BDPCMMode;
  static const CtxSet   QtRootCbf;
  static const CtxSet   ACTFlag;
  static const CtxSet   QtCbf           [3];    // [ channel ]
  static const CtxSet   SigCoeffGroup   [2];    // [ ChannelType ]
  static const CtxSet   LastX           [2];    // [ ChannelType ]
  static const CtxSet   LastY           [2];    // [ ChannelType ]
  static const CtxSet   SigFlag         [6];    // [ ChannelType + State ]
  static const CtxSet   ParFlag         [2];    // [ ChannelType ]
  static const CtxSet   GtxFlag         [4];    // [ ChannelType + x ]
  static const CtxSet   TsSigCoeffGroup;
  static const CtxSet   TsSigFlag;
  static const CtxSet   TsParFlag;
  static const CtxSet   TsGtxFlag;
  static const CtxSet   TsLrg1Flag;
  static const CtxSet   TsResidualSign;
  static const CtxSet   MVPIdx;
  static const CtxSet   SaoMergeFlag;
  static const CtxSet   SaoTypeIdx;
  static const CtxSet   MTSIndex;
  static const CtxSet   LFNSTIdx;
  static const CtxSet   RdpcmFlag;
  static const CtxSet   RdpcmDir;
  static const CtxSet   SbtFlag;
  static const CtxSet   SbtQuadFlag;
  static const CtxSet   SbtHorFlag;
  static const CtxSet   SbtPosFlag;
  static const CtxSet   ChromaQpAdjFlag;
  static const CtxSet   ChromaQpAdjIdc;
  static const CtxSet   ImvFlag;
  static const CtxSet   CcAlfFilterControlFlag;
  static const CtxSet   BcwIdx;
  static const CtxSet   ctbAlfFlag;
  static const CtxSet   ctbAlfAlternative;
  static const CtxSet   AlfUseTemporalFilt;
  static const CtxSet   CiipFlag;
  static const CtxSet   SmvdFlag;
  static const CtxSet   IBCFlag;
  static const CtxSet   ISPMode;
  static const CtxSet   JointCbCrFlag;
  static const unsigned NumberOfContexts;

  // combined sets for less complex copying
  // NOTE: The contained CtxSet's should directly follow each other in the initalization list;
  //       otherwise, you will copy more elements than you want !!!
  static const CtxSet   Sao;
  static const CtxSet   Alf;

public:
  static const std::vector<uint8_t>&  getInitTable( unsigned initId );

private:
  static std::array<std::vector<uint8_t>, NUMBER_OF_SLICE_TYPES + 1> sm_InitTables;
  static CtxSet addCtxSet( std::initializer_list<std::initializer_list<uint8_t> > initSet2d );
};

class Ctx : public ContextSetCfg
{
public:
  Ctx() = default;

  void init( int qp, int initId );

  const BinProbModel& operator[]( unsigned ctxId ) const { return m_CtxBuffer[ctxId]; }
        BinProbModel& operator[]( unsigned ctxId )       { return m_CtxBuffer[ctxId]; }

private:
  static_vector<BinProbModel, 372> m_CtxBuffer{ ContextSetCfg::NumberOfContexts };
};

}
