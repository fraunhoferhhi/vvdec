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

// ---------------------------------------------------------------------------
// coding structure
// ---------------------------------------------------------------------------

class CodingStructure
{
public:

  UnitArea         area;

  Picture         *picture;
  Slice           *slice;

  UnitScale        unitScale[MAX_NUM_COMPONENT];
  ChannelType      chType;
  int              chromaQpAdj;
  Position         sharedBndPos;
  Size             sharedBndSize;
  bool             isLossless;
  std::shared_ptr<const VPS> vps;
  std::shared_ptr<const SPS> sps;
  std::shared_ptr<const PPS> pps;
  PicHeader       *picHeader;
  std::shared_ptr<APS>       alfApss[ALF_CTB_MAX_NUM_APS];
  std::shared_ptr<APS>       lmcsAps;
  const PreCalcValues*       pcv;

  CodingStructure(std::shared_ptr<CUCache>, std::shared_ptr<TUCache> );
  void create( const UnitArea &_unit, const bool isTopLayer );
  void create( const ChromaFormat &_chromaFormat, const Area& _area, const bool isTopLayer );
  void destroy();

  void rebindPicBufs();

  // ---------------------------------------------------------------------------
  // global accessors
  // ---------------------------------------------------------------------------

  const CodingUnit*     getCU(Position pos, ChannelType _chType) const { return m_cuIdx[_chType][rsAddr( unitScale[_chType].scale( pos ), m_mapOrigSize[_chType].width )]; }
        CodingUnit*     getCU(Position pos, ChannelType _chType)       { return m_cuIdx[_chType][rsAddr( unitScale[_chType].scale( pos ), m_mapOrigSize[_chType].width )]; }

  const CodingUnit*     getCURestricted(const Position &pos, const Position curPos, const unsigned curSliceIdx, const unsigned curTileIdx, const ChannelType _chType) const;
  const CodingUnit*     getCURestricted(const Position &pos, const CodingUnit& curCu,                               const ChannelType _chType, const CodingUnit* guess = nullptr) const;

  CodingUnit&     addCU(const UnitArea &unit, const ChannelType _chType, const TreeType treeType, const ModeType modeType, const CodingUnit* cuLeft, const CodingUnit* cuAbove );
  TransformUnit&  addTU(const UnitArea &unit, const ChannelType _chType, CodingUnit &cu);
  void            addEmptyTUs(Partitioner &partitioner, CodingUnit& cu);
  CUTraverser     traverseCUs(const UnitArea& _unit);

  cCUTraverser    traverseCUs(const UnitArea& _unit) const;

  void initStructData();

private:
  void createInternals(const UnitArea& _unit, const bool isTopLayer);

private:
  Size            m_mapSize    [MAX_NUM_CHANNEL_TYPE];
  Size            m_mapOrigSize[MAX_NUM_CHANNEL_TYPE];
  Size            m_mapOrigSizeUnscaled[MAX_NUM_CHANNEL_TYPE];
  CodingUnit    **m_cuIdxOrigin[MAX_NUM_CHANNEL_TYPE];
  CodingUnit    **m_cuIdx      [MAX_NUM_CHANNEL_TYPE];

  Pel*            m_predBuf    [MAX_NUM_COMPONENT];

  std::vector<Mv> m_dmvrMvCache;
  ptrdiff_t       m_dmvrMvCacheOffset;

  unsigned                 m_numCUs;

  std::shared_ptr<CUCache> m_cuCache;
  std::shared_ptr<TUCache> m_tuCache;

  std::vector<SAOBlkParam> m_sao;

  PelStorage m_reco;
  PelStorage m_rec_wrap;
  PelStorage m_pred;

  CodingUnit* m_lastCU = nullptr;

  MotionInfo      *m_motionBuf;
  LoopFilterParam *m_lfParam[NUM_EDGE_DIR];

public:

  int m_IBCBufferWidth;
  std::vector<PelStorage> m_virtualIBCbuf;
  void initVIbcBuf( int numCtuLines, ChromaFormat chromaFormatIDC, int ctuSize );
  void fillIBCbuffer( CodingUnit &cu, int lineIdx );
  
  PelStorage m_alfBuf;

  MotionBuf getMotionBuf( const     Area& _area );
  MotionBuf getMotionBuf( const UnitArea& _area ) { return getMotionBuf( _area.Y() ); }
  MotionBuf getMotionBuf()                        { return getMotionBuf(  area.Y() ); }

  const CMotionBuf getMotionBuf( const     Area& _area ) const;
  const CMotionBuf getMotionBuf( const UnitArea& _area ) const { return getMotionBuf( _area.Y() ); }
  const CMotionBuf getMotionBuf()                        const { return getMotionBuf(  area.Y() ); }

        MotionInfo& getMotionInfo( const Position& pos );
  const MotionInfo& getMotionInfo( const Position& pos ) const;

  MotionInfo const* getMiMapPtr()    const { return m_motionBuf; }
  MotionInfo      * getMiMapPtr()          { return m_motionBuf; }
  ptrdiff_t         getMiMapStride() const { return ( ptrdiff_t ) g_miScaling.scaleHor( area.Y().width ); }

         LFPBuf getLoopFilterParamBuf( const DeblockEdgeDir& edgeDir );
  const CLFPBuf getLoopFilterParamBuf( const DeblockEdgeDir& edgeDir ) const;

  LoopFilterParam const* getLFPMapPtr   ( const DeblockEdgeDir edgeDir ) const { return m_lfParam[edgeDir]; }
  LoopFilterParam      * getLFPMapPtr   ( const DeblockEdgeDir edgeDir )       { return m_lfParam[edgeDir]; }
  ptrdiff_t              getLFPMapStride() const { return ( ptrdiff_t ) m_mapSize[CH_L].width; }

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

#if JVET_Q0438_MONOCHROME_BUGFIXES
static inline uint32_t getNumberValidTBlocks( const PreCalcValues& pcv )
{
  return ( pcv.chrFormat == CHROMA_400 ) ? 1 : MAX_NUM_COMPONENT;
}
#endif
#endif

