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

/** \file     ContextModelling.h
 *  \brief    Classes providing probability descriptions and contexts (header)
 */

#ifndef __CONTEXTMODELLING__
#define __CONTEXTMODELLING__


#include "CommonDef.h"
#include "Contexts.h"
#include "Slice.h"
#include "Unit.h"
#include "UnitPartitioner.h"

#include <bitset>

#include "CommonLib/dtrace_next.h"


struct CoeffCodingContext
{
public:
  CoeffCodingContext( const TransformUnit& tu, ComponentID component, bool signHide );
public:
  void  initSubblock     ( int SubsetId, bool sigGroupFlag = false );
public:
  void  setSigGroup     ()                      { m_sigCoeffGroupFlag.set( m_subSetPos ); }
  bool  noneSigGroup    ()                      { return m_sigCoeffGroupFlag.none(); }
  int   lastSubSet      ()                      { return ( maxNumCoeff() - 1 ) >> log2CGSize(); }
  bool  isLastSubSet    ()                      { return lastSubSet() == m_subSetId; }
  bool  only1stSigGroup ()                      { return m_sigCoeffGroupFlag.count()-m_sigCoeffGroupFlag[lastSubSet()]==0; }
  void  setScanPosLast  ( int       posLast )   { m_scanPosLast = posLast; }
public:
  int             subSetId        ()                        const { return m_subSetId; }
  int             subSetPos       ()                        const { return m_subSetPos; }
  int             cgPosY          ()                        const { return m_subSetPosY; }
  int             cgPosX          ()                        const { return m_subSetPosX; }
  unsigned        width           ()                        const { return m_width; }
  unsigned        height          ()                        const { return m_height; }
  unsigned        log2CGSize      ()                        const { return m_log2CGSize; }
  unsigned        log2CGWidth     ()                        const { return m_log2CGWidth; }
  unsigned        log2CGHeight    ()                        const { return m_log2CGHeight; }
  unsigned        log2BlockWidth  ()                        const { return m_log2BlockWidth; }
  unsigned        log2BlockHeight ()                        const { return m_log2BlockHeight; }
  unsigned        log2BlockSize   ()                        const { return m_log2BlockSize; }
  int             maxLog2TrDRange ()                        const { return m_maxLog2TrDynamicRange; }
  unsigned        maxNumCoeff     ()                        const { return m_maxNumCoeff; }
  int             scanPosLast     ()                        const { return m_scanPosLast; }
  int             minSubPos       ()                        const { return m_minSubPos; }
  int             maxSubPos       ()                        const { return m_maxSubPos; }
  bool            isLast          ()                        const { return ( ( m_scanPosLast >> m_log2CGSize ) == m_subSetId ); }
  bool            isNotFirst      ()                        const { return ( m_subSetId != 0 ); }
  bool            isSigGroup      ()                        const { return m_sigCoeffGroupFlag[ m_subSetPos ]; }
  bool            signHiding      ()                        const { return m_signHiding; }
  bool            hideSign        ( int       posFirst,
                                    int       posLast   )   const { return ( m_signHiding && ( posLast - posFirst >= SBH_THRESHOLD ) ); }
  CoeffScanType   scanType        ()                        const { return m_scanType; }
  unsigned        blockPos        ( int       scanPos   )   const { return m_scan[ scanPos ]; }
  unsigned        posX            ( int       blkPos    )   const { return blkPos & ( ( 1 << m_log2BlockWidth ) - 1 ); }
  unsigned        posY            ( int       blkPos    )   const { return blkPos >> m_log2BlockWidth; }
  unsigned        maxLastPosX     ()                        const { return m_maxLastPosX; }
  unsigned        maxLastPosY     ()                        const { return m_maxLastPosY; }
  unsigned        lastXCtxId      ( unsigned  posLastX  )   const { return m_CtxSetLastX( m_lastOffsetX + ( posLastX >> m_lastShiftX ) ); }
  unsigned        lastYCtxId      ( unsigned  posLastY  )   const { return m_CtxSetLastY( m_lastOffsetY + ( posLastY >> m_lastShiftY ) ); }
  int             numCtxBins      ()                        const { return   m_remainingContextBins;      }
  void            setNumCtxBins   ( int n )                       {          m_remainingContextBins  = n; }
  unsigned        sigGroupCtxId   ( bool ts = false     )   const { return ts ? m_sigGroupCtxIdTS : m_sigGroupCtxId; }
  bool            bdpcm           ()                        const { return m_bdpcm; }
  int             regBinLimit     ()                        const { return m_regBinLimit; }
  void            setRegBinLimit  ( int n )                       {        m_regBinLimit = n; }

  void            decNumCtxBins   (int n)                         { m_remainingContextBins -= n; }
  void            incNumCtxBins   (int n)                         { m_remainingContextBins += n; }
  bool            checkTplBnd     ()                        const { return m_checkTplBnd; }

  template<bool checkBnd = false>
  unsigned sigCtxIdAbs( int blkPos, const TCoeffSig* coeff, const int state )
  {
    const uint32_t    posY  = blkPos >> m_log2BlockWidth;
    const uint32_t    posX  = blkPos & ( ( 1 << m_log2BlockWidth ) - 1 );
    const TCoeffSig*  pData = coeff + posX + ( posY << m_log2BlockWidth );
    const int     diag      = posX + posY;
    int           numPos    = 0;
    int           sumAbs    = 0;
#define UPDATE(x) {int a=x;sumAbs+=std::min(4+(a&1),a);numPos+=!!a;}
    if( checkBnd )
    {
      const int xLtWmin1 = ( int( posX ) + 1 - int( m_width ) ) >> 31;
      const int xLtWmin2 = ( int( posX ) + 2 - int( m_width ) ) >> 31;

      UPDATE( ( pData[1] & xLtWmin1 ) );
      UPDATE( ( pData[2] & xLtWmin2 ) );
      UPDATE(   pData[m_width] );
      UPDATE( ( pData[m_width+1] & xLtWmin1 ) );
      UPDATE(   pData[m_width<<1] );
    }
    else
    {
      UPDATE( pData[1] );
      UPDATE( pData[2] );
      UPDATE( pData[m_width] );
      UPDATE( pData[m_width+1] );
      UPDATE( pData[m_width<<1] );
    }
#undef UPDATE

    int ctxOfs = std::min( ( sumAbs + 1 ) >> 1, 3 ) + ( diag < 2 ? 4 : 0 );

    if( m_chType == CHANNEL_TYPE_LUMA )
    {
      ctxOfs += diag < 5 ? 4 : 0;
    }
    m_tmplCpDiag = diag;
    m_tmplCpSum1 = sumAbs - numPos;
    return m_sigFlagCtxSet[std::max( 0, state-1 )]( ctxOfs );
  }

  uint8_t ctxOffsetAbs()
  {
    int offset = 0;
    if( m_tmplCpDiag != -1 )
    {
      offset  = std::min( m_tmplCpSum1, 4 ) + 1;
      offset += ( !m_tmplCpDiag ? ( m_chType == CHANNEL_TYPE_LUMA ? 15 : 5 ) : m_chType == CHANNEL_TYPE_LUMA ? m_tmplCpDiag < 3 ? 10 : ( m_tmplCpDiag < 10 ? 5 : 0 ) : 0 );
    }
    return uint8_t(offset);
  }

  unsigned parityCtxIdAbs   ( uint8_t offset )  const { return m_parFlagCtxSet   ( offset ); }
  unsigned greater1CtxIdAbs ( uint8_t offset )  const { return m_gtxFlagCtxSet[1]( offset ); }
  unsigned greater2CtxIdAbs ( uint8_t offset )  const { return m_gtxFlagCtxSet[0]( offset ); }

  unsigned templateAbsSum( int blkPos, const TCoeffSig* coeff, int baseLevel )
  {
    const uint32_t   posY  = blkPos >> m_log2BlockWidth;
    const uint32_t   posX  = blkPos & ( ( 1 << m_log2BlockWidth ) - 1 );
    const TCoeffSig* pData = coeff + posX + ( posY << m_log2BlockWidth );
    int              sum   = 0;

    if (posX+2 < m_width)
    {
      sum += pData[1];
      sum += pData[2];
      if (posY+1 < m_height)
      {
        sum += pData[m_width + 1];
      }
      }
    else if (posX+1 < m_width)
    {
      sum += pData[1];
      if (posY+1 < m_height)
      {
        sum += pData[m_width + 1];
      }
    }
    if (posY+2 < m_height)
    {
      sum += pData[m_width];
        sum += pData[m_width << 1];
      }
    else if (posY+1 < m_height)
    {
      sum += pData[m_width];
    }
    return std::max(std::min(sum - 5 * baseLevel, 31), 0);
  }

  unsigned sigCtxIdAbsTS( int blkPos, const TCoeffSig* coeff )
  {
    const uint32_t   posY  = blkPos >> m_log2BlockWidth;
    const uint32_t   posX  = blkPos & ( ( 1 << m_log2BlockWidth ) - 1 );
    const TCoeffSig* posC  = coeff + posX + posY * m_width;
    int             numPos = 0;
#define UPDATE(x) {int a=abs(x);numPos+=!!a;}
    if( posX > 0 )
    {
      UPDATE( posC[-1] );
    }
    if( posY > 0 )
    {
      UPDATE( posC[-(int)m_width] );
    }
#undef UPDATE

    return m_tsSigFlagCtxSet( numPos );
  }

  unsigned parityCtxIdAbsTS   ()                  const { return m_tsParFlagCtxSet(      0 ); }
  unsigned greaterXCtxIdAbsTS ( uint8_t offset )  const { return m_tsGtxFlagCtxSet( offset ); }

  unsigned lrg1CtxIdAbsTS(int blkPos, const TCoeffSig* coeff, int bdpcm)
  {
    const uint32_t  posY = blkPos >> m_log2BlockWidth;
    const uint32_t  posX = blkPos & ( ( 1 << m_log2BlockWidth ) - 1 );
    const TCoeffSig*   posC = coeff + posX + posY * m_width;

    int             numPos = 0;
#define UPDATE(x) {int a=abs(x);numPos+=!!a;}

    if (bdpcm)
    {
      numPos = 3;
    }
    else
    {
      if (posX > 0)
      {
        UPDATE(posC[-1]);
      }
      if (posY > 0)
      {
        UPDATE(posC[-(int)m_width]);
      }
    }

#undef UPDATE
    return m_tsLrg1FlagCtxSet(numPos);
  }

  unsigned signCtxIdAbsTS(int blkPos, const TCoeffSig* coeff, int bdpcm)
  {
    const uint32_t  posY = blkPos >> m_log2BlockWidth;
    const uint32_t  posX = blkPos & ( ( 1 << m_log2BlockWidth ) - 1 );
    
    const TCoeffSig*   pData = coeff + posX + posY * m_width;

    int rightSign = 0, belowSign = 0;
    unsigned signCtx = 0;

    if (posX > 0)
    {
      rightSign = pData[-1];
    }
    if (posY > 0)
    {
      belowSign = pData[-(int)m_width];
    }

    if ((rightSign == 0 && belowSign == 0) || ((rightSign*belowSign) < 0))
    {
      signCtx = 0;
    }
    else if (rightSign >= 0 && belowSign >= 0)
    {
      signCtx = 1;
    }
    else
    {
      signCtx = 2;
    }
    if (bdpcm)
    {
      signCtx += 3;
    }

    DTRACE( g_trace_ctx, D_SYNTAX_RESI, "signCtxIdAbsTS() pos=(%d;%d)  signCtx=%d  rightSign=%d belowSign=%d\n", posX, posY, signCtx, rightSign, belowSign );

    return m_tsSignFlagCtxSet(signCtx);
  }

  void neighTS(int &rightPixel, int &belowPixel, int blkPos, const TCoeffSig* coeff)
  {
    const uint32_t  posY = blkPos >> m_log2BlockWidth;
    const uint32_t  posX = blkPos & ( ( 1 << m_log2BlockWidth ) - 1 );
    const TCoeffSig*   data = coeff + posX + posY * m_width;

    rightPixel = belowPixel = 0;

    if (posX > 0)
    {
      rightPixel = data[-1];
    }
    if (posY > 0)
    {
      belowPixel = data[-(int)m_width];
    }
  }

  int deriveModCoeff(int rightPixel, int belowPixel, int absCoeff, int bdpcm = 0)
  {
    if (absCoeff == 0)
      return 0;
    int pred1, absBelow = abs(belowPixel), absRight = abs(rightPixel);

    int absCoeffMod = absCoeff;

    if (bdpcm == 0)
    {
      pred1 = std::max(absBelow, absRight);

      if (absCoeff == pred1)
      {
        absCoeffMod = 1;
      }
      else
      {
        absCoeffMod = absCoeff < pred1 ? absCoeff + 1 : absCoeff;
      }
    }

    return(absCoeffMod);
  }

  int decDeriveModCoeff(int rightPixel, int belowPixel, int absCoeff)
  {
    if (absCoeff == 0)
      return 0;

    int pred1, absBelow = abs(belowPixel), absRight = abs(rightPixel);
    pred1 = std::max(absBelow, absRight);

    int absCoeffMod;

    if (absCoeff == 1 && pred1 > 0)
    {
      absCoeffMod = pred1;
    }
    else
    {
      absCoeffMod = absCoeff - (absCoeff <= pred1);
    }
    return(absCoeffMod);
  }

  unsigned templateAbsSumTS( int blkPos, const TCoeffSig* coeff )
  {
    return 1;
  }

private:
  // constant
  const ChannelType         m_chType;
  const unsigned            m_width;
  const unsigned            m_height;
  const unsigned            m_log2CGWidth;
  const unsigned            m_log2CGHeight;
  const unsigned            m_log2CGSize;
  const unsigned            m_widthInGroups;
  const unsigned            m_heightInGroups;
  const unsigned            m_log2BlockWidth;
  const unsigned            m_log2BlockHeight;
  const unsigned            m_log2BlockSize;
  const unsigned            m_maxNumCoeff;
  const bool                m_signHiding;
  const int                 m_maxLog2TrDynamicRange;
  CoeffScanType             m_scanType;
  const unsigned*           m_scan;
  const unsigned*           m_scanCG;
  const CtxSet              m_CtxSetLastX;
  const CtxSet              m_CtxSetLastY;
  const unsigned            m_maxLastPosX;
  const unsigned            m_maxLastPosY;
  const int                 m_lastOffsetX;
  const int                 m_lastOffsetY;
  const int                 m_lastShiftX;
  const int                 m_lastShiftY;
  // modified
  bool                      m_checkTplBnd;
  int                       m_scanPosLast;
  int                       m_subSetId;
  int                       m_subSetPos;
  int                       m_subSetPosX;
  int                       m_subSetPosY;
  int                       m_minSubPos;
  int                       m_maxSubPos;
  unsigned                  m_sigGroupCtxId;
  int                       m_tmplCpSum1;
  int                       m_tmplCpDiag;
  CtxSet                    m_sigFlagCtxSet[3];
  CtxSet                    m_parFlagCtxSet;
  CtxSet                    m_gtxFlagCtxSet[2];
  unsigned                  m_sigGroupCtxIdTS;
  CtxSet                    m_tsSigFlagCtxSet;
  CtxSet                    m_tsParFlagCtxSet;
  CtxSet                    m_tsGtxFlagCtxSet;
  CtxSet                    m_tsLrg1FlagCtxSet;
  CtxSet                    m_tsSignFlagCtxSet;
  int                       m_remainingContextBins;
  std::bitset<MLS_GRP_NUM>  m_sigCoeffGroupFlag;
  const bool                m_bdpcm;
  int                       m_regBinLimit;
  const bool                m_ts;
};


class CUCtx
{
public:
  CUCtx()              : isDQPCoded(false), isChromaQpAdjCoded(false), qgStart(false), lfnstLastScanPos(false)
                         {
                           violatesLfnstConstrained[CHANNEL_TYPE_LUMA  ] = false;
                           violatesLfnstConstrained[CHANNEL_TYPE_CHROMA] = false;
                           violatesMtsCoeffConstraint                    = false;
                           mtsLastScanPos                                = false;
                         }
  CUCtx(int _qp)       : isDQPCoded(false), isChromaQpAdjCoded(false), qgStart(false), lfnstLastScanPos(false), qp(_qp)
                         {
                           violatesLfnstConstrained[CHANNEL_TYPE_LUMA  ] = false;
                           violatesLfnstConstrained[CHANNEL_TYPE_CHROMA] = false;
                           violatesMtsCoeffConstraint                    = false;
                           mtsLastScanPos                                = false;
                         }
  ~CUCtx() {}
public:
  bool      isDQPCoded;
  bool      isChromaQpAdjCoded;
  bool      qgStart;
  bool      lfnstLastScanPos;
  int8_t    qp;                   // used as a previous(last) QP and for QP prediction
  bool      violatesLfnstConstrained[MAX_NUM_CHANNEL_TYPE];
  bool      violatesMtsCoeffConstraint;
  bool      mtsLastScanPos;
};

class MergeCtx
{
public:
  MergeCtx() : numValidMergeCand( 0 ) { memset( mrgTypeNeighbours, 0, sizeof( mrgTypeNeighbours ) ); }
  ~MergeCtx() {}
public:
  MvField       mvFieldNeighbours [ MRG_MAX_NUM_CANDS << 1 ]; // double length for mv of both lists
  uint8_t       BcwIdx            [ MRG_MAX_NUM_CANDS      ];
  unsigned char interDirNeighbours[ MRG_MAX_NUM_CANDS      ];
  MergeType     mrgTypeNeighbours [ MRG_MAX_NUM_CANDS      ];
  int           numValidMergeCand;

  MotionBuf     subPuMvpMiBuf;
  MvField       mmvdBaseMv        [MMVD_BASE_MV_NUM        ][2];

  void setMmvdMergeCandiInfo( PredictionUnit& pu, int candIdx );
  bool          mmvdUseAltHpelIf  [ MMVD_BASE_MV_NUM ];
  bool          useAltHpelIf      [ MRG_MAX_NUM_CANDS ];
  void setMergeInfo         ( PredictionUnit& pu, int candIdx );
  void init()               { numValidMergeCand = 0; memset( mrgTypeNeighbours, 0, sizeof( mrgTypeNeighbours ) ); }
};

class AffineMergeCtx
{
public:
  AffineMergeCtx() : numValidMergeCand( 0 ) { for ( unsigned i = 0; i < AFFINE_MRG_MAX_NUM_CANDS; i++ ) affineType[i] = AFFINEMODEL_4PARAM; }
  ~AffineMergeCtx() {}
public:
  MvField       mvFieldNeighbours [AFFINE_MRG_MAX_NUM_CANDS << 1][3]; // double length for mv of both lists
  unsigned char interDirNeighbours[AFFINE_MRG_MAX_NUM_CANDS     ];
  AffineModel   affineType        [AFFINE_MRG_MAX_NUM_CANDS     ];
  uint8_t       BcwIdx            [AFFINE_MRG_MAX_NUM_CANDS     ];
  int           numValidMergeCand;
  int           maxNumMergeCand;

  MergeCtx     *mrgCtx;
  MergeType     mergeType         [AFFINE_MRG_MAX_NUM_CANDS     ];
};


namespace DeriveCtx
{
void     CtxSplit     ( const CodingStructure& cs, Partitioner& partitioner, unsigned& ctxSpl, unsigned& ctxQt, unsigned& ctxHv, unsigned& ctxHorBt, unsigned& ctxVerBt, bool* canSplit = nullptr );
unsigned CtxModeConsFlag( const CodingStructure& cs, Partitioner& partitioner );
unsigned CtxQtCbf     ( const ComponentID compID, const bool prevCbCbf = false, const int ispIdx = 0 );
unsigned CtxInterDir  ( const PredictionUnit& pu );
unsigned CtxSkipFlag  ( const CodingUnit& cu );
unsigned CtxAffineFlag( const CodingUnit& cu );
unsigned CtxPredModeFlag( const CodingUnit& cu );
unsigned CtxIBCFlag   (const CodingUnit& cu);
unsigned CtxMipFlag   ( const CodingUnit& cu );
}

#endif // __CONTEXTMODELLING__
