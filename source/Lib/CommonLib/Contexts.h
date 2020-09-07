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

/** \file     Contexts.h
 *  \brief    Classes providing probability descriptions and contexts (header)
 */

#ifndef __CONTEXTS__
#define __CONTEXTS__

#include "CommonDef.h"
#include "Slice.h"

#include <vector>

static constexpr int     PROB_BITS   = 15;   // Nominal number of bits to represent probabilities
static constexpr int     PROB_BITS_0 = 10;   // Number of bits to represent 1st estimate
static constexpr int     PROB_BITS_1 = 14;   // Number of bits to represent 2nd estimate
static constexpr int     MASK_0      = ~(~0u << PROB_BITS_0) << (PROB_BITS - PROB_BITS_0);
static constexpr int     MASK_1      = ~(~0u << PROB_BITS_1) << (PROB_BITS - PROB_BITS_1);
static constexpr uint8_t DWS         = 8;   // 0x47 Default window sizes

struct BinFracBits
{
  uint32_t intBits[2];
};

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
    m_rate        = DWS;
  }
  ~BinProbModel() {}
public:
  void            init              ( int qp, int initId );
  void update(unsigned bin)
  {
    int rate0 = m_rate >> 4;
    int rate1 = m_rate & 15;

    m_state[0] -= (m_state[0] >> rate0) & MASK_0;
    m_state[1] -= (m_state[1] >> rate1) & MASK_1;

    m_state[0] += ( ( -static_cast< int >( bin ) & 0x7fffu ) >> rate0 ) & MASK_0;
    m_state[1] += ( ( -static_cast< int >( bin ) & 0x7fffu ) >> rate1 ) & MASK_1;
    
    //if (bin)
    //{
    //  m_state[0] += (0x7fffu >> rate0) & MASK_0;
    //  m_state[1] += (0x7fffu >> rate1) & MASK_1;
    //}
  }
  void setLog2WindowSize(uint8_t log2WindowSize)
  {
    int rate0 = 2 + ((log2WindowSize >> 2) & 3);
    int rate1 = 3 + rate0 + (log2WindowSize & 3);
    m_rate    = 16 * rate0 + rate1;
    CHECK(rate1 > 9, "Second window size is too large!");
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
    const uint8_t q = state();
    bin = q >> 7;
    
    lps = ( ( ( ( ( ( q ^ 0xff ) &    -static_cast< int >( bin ) ) |
                  (   q          & ~( -static_cast< int >( bin ) ) ) ) >> 2 
              ) * ( range >> 5 ) ) >> 1 ) + 4;

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
  uint8_t  m_rate;
};






class CtxSet
{
public:
  CtxSet( uint16_t offset, uint16_t size ) : Offset( offset ), Size( size ) {}
  CtxSet( const CtxSet& ctxSet ) : Offset( ctxSet.Offset ), Size( ctxSet.Size ) {}
  CtxSet( std::initializer_list<CtxSet> ctxSets );
public:
  uint16_t  operator()  ()  const
  {
    return Offset;
  }
  uint16_t  operator()  ( uint16_t inc )  const
  {
    CHECKD( inc >= Size, "Specified context increment (" << inc << ") exceed range of context set [0;" << Size - 1 << "]." );

    return Offset + inc;
  }
public:
  uint16_t  Offset;
  uint16_t  Size;
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
  static const CtxSet   PdpcFlag;
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
  static const CtxSet   TransSubdivFlag;
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
  static const CtxSet   CrossCompPred;
  static const CtxSet   ChromaQpAdjFlag;
  static const CtxSet   ChromaQpAdjIdc;
  static const CtxSet   ImvFlag;
  static const CtxSet   CcAlfFilterControlFlag;
  static const CtxSet   BcwIdx;
  static const CtxSet   ctbAlfFlag;
  static const CtxSet   ctbAlfAlternative;
  static const CtxSet   AlfUseTemporalFilt;
  static const CtxSet   CiipFlag;
  static const CtxSet   TriangleIdx;
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
  static std::vector<std::vector<uint8_t> > sm_InitTables;
  static CtxSet addCtxSet( std::initializer_list<std::initializer_list<uint8_t> > initSet2d );
};

class CtxStore
{
public:
  CtxStore();
  CtxStore( bool dummy );
  CtxStore( const CtxStore& ctxStore );
public:
  void init       ( int qp, int initId );

  const BinProbModel& operator[]      ( unsigned  ctxId  )  const { return m_Ctx[ctxId]; }
  BinProbModel&       operator[]      ( unsigned  ctxId  )        { return m_Ctx[ctxId]; }
private:
  std::vector<BinProbModel> m_CtxBuffer;
  BinProbModel*             m_Ctx;
};

class Ctx : public ContextSetCfg
{
public:
  Ctx();
  Ctx( const Ctx&                 ctx   );

public:

  void  init ( int qp, int initId )
  {
    m_CtxStore_Std.init( qp, initId );
  }

public:
  const Ctx&          getCtx          ()                        const { return *this; }
  Ctx&                getCtx          ()                              { return *this; }

  explicit operator const CtxStore&()         const;
  explicit operator       CtxStore&();

private:
  CtxStore m_CtxStore_Std;
};



#endif
