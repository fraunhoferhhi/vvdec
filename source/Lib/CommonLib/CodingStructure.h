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

/** \file     CodingStructure.h
 *  \brief    A class managing the coding information for a specific image part
 */

#ifndef __CODINGSTRUCTURE__
#define __CODINGSTRUCTURE__

#include "Unit.h"
#include "Buffer.h"
#include "CommonDef.h"
#include "UnitPartitioner.h"
#include "Slice.h"

#include <vector>


struct Picture;


enum PictureType
{
  PIC_RECONSTRUCTION,
  PIC_RECON_WRAP,
  PIC_PREDICTION,
  NUM_PIC_TYPES
};

extern ThreadSafeCUCache g_globalUnitCache;

#define NUM_PARTS_IN_CTU ( MAX_CU_SIZE * MAX_CU_SIZE ) >> ( MIN_CU_LOG2 << 1 )

struct CtuData
{
  SAOBlkParam          saoParam;

  CodingUnit*     cuPtr  [MAX_NUM_CHANNEL_TYPE][NUM_PARTS_IN_CTU];
  LoopFilterParam lfParam[NUM_EDGE_DIR]        [NUM_PARTS_IN_CTU];
  MotionInfo      motion                       [NUM_PARTS_IN_CTU];
};

// ---------------------------------------------------------------------------
// coding structure
// ---------------------------------------------------------------------------

class CodingStructure
{
public:

  UnitArea         area;

  Picture         *picture;

  UnitScale        unitScale[MAX_NUM_COMPONENT];
  int              chromaQpAdj;
  std::shared_ptr<const VPS> vps;
  std::shared_ptr<const SPS> sps;
  std::shared_ptr<const PPS> pps;
  PicHeader                 *picHeader;
  std::shared_ptr<APS>       alfApss[ALF_CTB_MAX_NUM_APS];
  std::shared_ptr<APS>       lmcsAps;
  const PreCalcValues*       pcv;

  CtuData*          m_ctuData;
  size_t            m_ctuDataSize;


  CodingStructure(std::shared_ptr<CUCache>, std::shared_ptr<TUCache>);

  void create(const UnitArea &_unit);
  void create(const ChromaFormat &_chromaFormat, const Area& _area);
  void destroy();

  void rebindPicBufs();

  // ---------------------------------------------------------------------------
  // global accessors
  // ---------------------------------------------------------------------------

#if _DEBUG
  const CodingUnit*     getCU(Position pos, ChannelType _chType) const
  {
    if( area.blocks[_chType].contains( pos ) )
    {
      ptrdiff_t rsAddr = ctuRsAddr( pos, _chType );
      ptrdiff_t inCtu  = inCtuPos ( pos, _chType );
      return getCtuData( rsAddr ).cuPtr[_chType][inCtu];
    }
    else return nullptr;
  }

  CodingUnit*     getCU(Position pos, ChannelType _chType)
  {
    if( area.blocks[_chType].contains( pos ) )
    {
      ptrdiff_t rsAddr = ctuRsAddr( pos, _chType );
      ptrdiff_t inCtu  = inCtuPos ( pos, _chType );
      return getCtuData( rsAddr ).cuPtr[_chType][inCtu];
    }
    else return nullptr;
  }
#else
  const CodingUnit*     getCU(Position pos, ChannelType _chType) const { if( area.blocks[_chType].contains( pos ) ) return getCtuData( ctuRsAddr( pos, _chType ) ).cuPtr[_chType][inCtuPos( pos, _chType )]; else return nullptr; }
        CodingUnit*     getCU(Position pos, ChannelType _chType)       { if( area.blocks[_chType].contains( pos ) ) return getCtuData( ctuRsAddr( pos, _chType ) ).cuPtr[_chType][inCtuPos( pos, _chType )]; else return nullptr; }
#endif

  const CodingUnit*     getCURestricted(const Position &pos, const Position curPos, const unsigned curSliceIdx, const unsigned curTileIdx, const ChannelType _chType) const;
  const CodingUnit*     getCURestricted(const Position &pos, const CodingUnit& curCu,                                                      const ChannelType _chType, const CodingUnit* guess = nullptr) const;

  CodingUnit&     addCU(const UnitArea &unit, const ChannelType _chType, const TreeType treeType, const ModeType modeType, const CodingUnit* cuLeft, const CodingUnit* cuAbove );
  TransformUnit&  addTU(const UnitArea &unit, const ChannelType _chType, CodingUnit &cu);
  void            addEmptyTUs(Partitioner &partitioner, CodingUnit& cu);
  CUTraverser     traverseCUs(const UnitArea& _unit);

  cCUTraverser    traverseCUs(const UnitArea& _unit) const;

  void initStructData();

private:
  void createInternals(const UnitArea& _unit);

private:

  Pel*            m_predBuf    [MAX_NUM_COMPONENT];

  Mv*             m_dmvrMvCache;
  size_t          m_dmvrMvCacheSize;
  ptrdiff_t       m_dmvrMvCacheOffset;

  unsigned                 m_numCUs;
  unsigned                 m_numTUs;

  std::shared_ptr<CUCache> m_cuCache;
  std::shared_ptr<TUCache> m_tuCache;

  PelStorage m_reco;
  PelStorage m_rec_wrap;
  PelStorage m_pred;

  CodingUnit* m_lastCU = nullptr;

  size_t               m_widthInCtus;
  PosType              m_ctuSizeMask[2];
  PosType              m_ctuWidthLog2[2];

public:

  // in CTU coordinates
  ptrdiff_t ctuRsAddr( int col, int line ) const { return col + ( line * m_widthInCtus ); }
  // in sample coordinates
  ptrdiff_t ctuRsAddr( Position pos, ChannelType chType ) const { Position posL = recalcPosition( area.chromaFormat, chType, CH_L, pos ); return ctuRsAddr( posL.x >> pcv->maxCUWidthLog2, posL.y >> pcv->maxCUHeightLog2 ); }
  // 4x4 luma block position within the CTU
  ptrdiff_t inCtuPos ( Position pos, ChannelType chType ) const { return ( unitScale[chType].scaleHor( pos.x ) & m_ctuSizeMask[chType] ) + ( ( unitScale[chType].scaleVer( pos.y ) & m_ctuSizeMask[chType] ) << m_ctuWidthLog2[chType] ); }

        CtuData& getCtuData( int col, int line )       { return m_ctuData[ctuRsAddr( col, line )]; }
  const CtuData& getCtuData( int col, int line ) const { return m_ctuData[ctuRsAddr( col, line )]; }

        CtuData& getCtuData( int addr )       { return m_ctuData[addr]; }
  const CtuData& getCtuData( int addr ) const { return m_ctuData[addr]; }

  int m_IBCBufferWidth;
  std::vector<PelStorage> m_virtualIBCbuf;
  void initVIbcBuf( int numCtuLines, ChromaFormat chromaFormatIDC, int ctuSize );
  void fillIBCbuffer( CodingUnit &cu, int lineIdx );
  
  PelStorage m_alfBuf;

  MotionBuf getMotionBuf( const     Area& _area );
  MotionBuf getMotionBuf( const UnitArea& _area ) { return getMotionBuf( _area.Y() ); }

  const CMotionBuf getMotionBuf( const     Area& _area ) const;
  const CMotionBuf getMotionBuf( const UnitArea& _area ) const { return getMotionBuf( _area.Y() ); }

        MotionInfo& getMotionInfo( const Position& pos )       { return getCtuData( ctuRsAddr( pos, CH_L ) ).motion[inCtuPos( pos, CH_L )]; }
  const MotionInfo& getMotionInfo( const Position& pos ) const { return getCtuData( ctuRsAddr( pos, CH_L ) ).motion[inCtuPos( pos, CH_L )]; }

  LoopFilterParam const* getLFPMapPtr   ( const DeblockEdgeDir edgeDir, ptrdiff_t _ctuRsAddr ) const { return m_ctuData[_ctuRsAddr].lfParam[edgeDir]; }
  LoopFilterParam      * getLFPMapPtr   ( const DeblockEdgeDir edgeDir, ptrdiff_t _ctuRsAddr )       { return m_ctuData[_ctuRsAddr].lfParam[edgeDir]; }
  ptrdiff_t              getLFPMapStride() const { return ( ptrdiff_t( 1 ) << m_ctuWidthLog2[CH_L] ); }

  UnitScale getScaling( const UnitScale::ScaliningType type, const ChannelType chType = CH_L ) const
  {
    return type == UnitScale::MI_MAP ? g_miScaling : unitScale[chType];
  }

public:

         PelBuf       getRecoBuf(const CompArea &blk)        { return m_reco.bufs[blk.compID].subBuf( blk ); }
  const CPelBuf       getRecoBuf(const CompArea &blk) const  { return m_reco.bufs[blk.compID].subBuf( blk ); }
         PelUnitBuf   getRecoBuf(const UnitArea &unit)       { return m_reco.subBuf( unit ); }
  const CPelUnitBuf   getRecoBuf(const UnitArea &unit) const { return m_reco.subBuf( unit ); }



  // reco buffer
         PelBuf       getRecoBuf(const ComponentID compID, bool wrap=false)         { return wrap ? m_rec_wrap.get(compID) : m_reco.get(compID); }
  const CPelBuf       getRecoBuf(const ComponentID compID, bool wrap=false)   const { return wrap ? m_rec_wrap.get(compID) : m_reco.get(compID); }
         PelUnitBuf   getRecoBuf(bool wrap=false)                                   { return wrap ? m_rec_wrap : m_reco; }
  const CPelUnitBuf   getRecoBuf(bool wrap=false)                             const { return wrap ? m_rec_wrap : m_reco; }

         PelUnitBuf   getPredBuf(const CodingUnit &cu);
  const CPelUnitBuf   getPredBuf(const CodingUnit &cu) const;
};

#endif

