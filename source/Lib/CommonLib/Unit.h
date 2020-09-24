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

/** \file     Unit.h
 *  \brief    defines unit as a set of blocks and basic unit types (coding, prediction, transform)
 */

#ifndef __UNIT__
#define __UNIT__

#include "CommonDef.h"
#include "Common.h"
#include "Mv.h"
#include "MotionInfo.h"
#include "ChromaFormat.h"


// ---------------------------------------------------------------------------
// tools
// ---------------------------------------------------------------------------
inline Position recalcPosition(const ChromaFormat _cf, const ComponentID srcCId, const ComponentID dstCId, const Position &pos)
{
  if( toChannelType( srcCId ) == toChannelType( dstCId ) )
  {
    return pos;
  }
  else if (isLuma(srcCId) && isChroma(dstCId))
  {
    return Position(pos.x >> getComponentScaleX(dstCId, _cf), pos.y >> getComponentScaleY(dstCId, _cf));
  }
  else
  {
    return Position(pos.x << getComponentScaleX(srcCId, _cf), pos.y << getComponentScaleY(srcCId, _cf));
  }
}

inline Position recalcPosition( const ChromaFormat _cf, const ChannelType srcCHt, const ChannelType dstCHt, const Position &pos )
{
  if( srcCHt == dstCHt )
  {
    return pos;
  }
  else if( isLuma( srcCHt ) && isChroma( dstCHt ) )
  {
    return Position( pos.x >> getChannelTypeScaleX( dstCHt, _cf ), pos.y >> getChannelTypeScaleY( dstCHt, _cf ) );
  }
  else
  {
    return Position( pos.x << getChannelTypeScaleX( srcCHt, _cf ), pos.y << getChannelTypeScaleY( srcCHt, _cf ) );
  }
}

inline Size recalcSize( const ChromaFormat _cf, const ComponentID srcCId, const ComponentID dstCId, const Size &size )
{
  if( toChannelType( srcCId ) == toChannelType( dstCId ) )
  {
    return size;
  }
  else if( isLuma( srcCId ) && isChroma( dstCId ) )
  {
    return Size( size.width >> getComponentScaleX( dstCId, _cf ), size.height >> getComponentScaleY( dstCId, _cf ) );
  }
  else
  {
    return Size( size.width << getComponentScaleX( srcCId, _cf ), size.height << getComponentScaleY( srcCId, _cf ) );
  }
}

inline Size recalcSize( const ChromaFormat _cf, const ChannelType srcCHt, const ChannelType dstCHt, const Size &size )
{
  if( srcCHt == dstCHt )
  {
    return size;
  }
  else if( isLuma( srcCHt ) && isChroma( dstCHt ) )
  {
    return Size( size.width >> getChannelTypeScaleX( dstCHt, _cf ), size.height >> getChannelTypeScaleY( dstCHt, _cf ) );
  }
  else
  {
    return Size( size.width << getChannelTypeScaleX( srcCHt, _cf ), size.height << getChannelTypeScaleY( srcCHt, _cf ) );
  }
}

// ---------------------------------------------------------------------------
// block definition
// ---------------------------------------------------------------------------

struct CompArea : public Area
{
  CompArea() : Area(), compID(MAX_NUM_TBLOCKS)                                                                                                            { }
  CompArea(const ComponentID _compID, const Area &_area)                                                          : Area(_area),          compID(_compID) { }
  CompArea(const ComponentID _compID, const Position& _pos, const Size& _size)                                    : Area(_pos, _size),    compID(_compID) { }
  CompArea(const ComponentID _compID, const uint32_t _x, const uint32_t _y, const uint32_t _w, const uint32_t _h) : Area(_x, _y, _w, _h), compID(_compID) { }

  ComponentID compID;

  Position chromaPos(const ChromaFormat chromaFormat) const;
  Position lumaPos(const ChromaFormat chromaFormat)   const;

  Size     chromaSize(const ChromaFormat chromaFormat) const;
  Size     lumaSize(const ChromaFormat chromaFormat)   const;

  Position compPos(const ChromaFormat chromaFormat, const ComponentID compID) const;
  Position chanPos(const ChromaFormat chromaFormat, const ChannelType chType) const;

  Position topLeftComp    (const ChromaFormat chromaFormat, const ComponentID _compID) const { return recalcPosition(chromaFormat, compID, _compID, *this);                                                     }
  Position topRightComp   (const ChromaFormat chromaFormat, const ComponentID _compID) const { return recalcPosition(chromaFormat, compID, _compID, { (PosType) (x + width - 1), y                          }); }
  Position bottomLeftComp (const ChromaFormat chromaFormat, const ComponentID _compID) const { return recalcPosition(chromaFormat, compID, _compID, { x                        , (PosType) (y + height - 1 )}); }
  Position bottomRightComp(const ChromaFormat chromaFormat, const ComponentID _compID) const { return recalcPosition(chromaFormat, compID, _compID, { (PosType) (x + width - 1), (PosType) (y + height - 1 )}); }

  bool valid() const { return compID < MAX_NUM_TBLOCKS && width != 0 && height != 0; }

  bool operator==(const CompArea &other) const
  {
    if (compID       != other.compID)       return false;

    return Position::operator==(other) && Size::operator==(other);
  }

  bool operator!=(const CompArea &other) const { return !(operator==(other)); }

  void     repositionTo      (const Position& newPos)       { Position::repositionTo(newPos); }
  void     positionRelativeTo(const CompArea& origCompArea) { Position::relativeTo(origCompArea); }
};

inline CompArea clipArea(const CompArea &compArea, const Area &boundingBox)
{
  return CompArea(compArea.compID, clipArea((const Area&) compArea, boundingBox));
}

// ---------------------------------------------------------------------------
// unit definition
// ---------------------------------------------------------------------------

typedef static_vector<CompArea, MAX_NUM_TBLOCKS> UnitBlocksType;

struct UnitArea
{
  ChromaFormat chromaFormat;
  UnitBlocksType blocks;

  UnitArea() : chromaFormat(NUM_CHROMA_FORMAT) { }
  UnitArea(const ChromaFormat _chromaFormat);
  UnitArea(const ChromaFormat _chromaFormat, const Area &area);
  UnitArea(const ChromaFormat _chromaFormat, const CompArea  &blkY);
  UnitArea(const ChromaFormat _chromaFormat,       CompArea &&blkY);
  UnitArea(const ChromaFormat _chromaFormat, const CompArea  &blkY, const CompArea  &blkCb, const CompArea  &blkCr);
  UnitArea(const ChromaFormat _chromaFormat,       CompArea &&blkY,       CompArea &&blkCb,       CompArea &&blkCr);

        CompArea& Y()                                  { return blocks[COMPONENT_Y];  }
  const CompArea& Y()                            const { return blocks[COMPONENT_Y];  }
        CompArea& Cb()                                 { return blocks[COMPONENT_Cb]; }
  const CompArea& Cb()                           const { return blocks[COMPONENT_Cb]; }
        CompArea& Cr()                                 { return blocks[COMPONENT_Cr]; }
  const CompArea& Cr()                           const { return blocks[COMPONENT_Cr]; }

        CompArea& block(const ComponentID comp)       { return blocks[comp]; }
  const CompArea& block(const ComponentID comp) const { return blocks[comp]; }

  bool contains(const UnitArea& other) const;
  bool contains(const UnitArea& other, const ChannelType chType) const;

        CompArea& operator[]( const int n )       { return blocks[n]; }
  const CompArea& operator[]( const int n ) const { return blocks[n]; }

  bool operator==(const UnitArea &other) const
  {
    if (chromaFormat != other.chromaFormat)   return false;
    if (blocks.size() != other.blocks.size()) return false;

    for (uint32_t i = 0; i < blocks.size(); i++)
    {
      if (blocks[i] != other.blocks[i]) return false;
    }

    return true;
  }

  void repositionTo(const UnitArea& unit);

  bool operator!=(const UnitArea &other) const { return !(*this == other); }

  const Position& lumaPos () const { return Y(); }
  const Size&     lumaSize() const { return Y(); }
  const Area&     lumaArea() const { return Y(); }

  const Position& chromaPos () const { return Cb(); }
  const Size&     chromaSize() const { return Cb(); }
  const Area&     chromaArea() const { return Cb(); }

  const UnitArea  singleComp(const ComponentID compID) const;
  const UnitArea  singleChan(const ChannelType chType) const;

  SizeType  lwidth()  const { return Y().width; }  /*! luma width  */
  SizeType  lheight() const { return Y().height; } /*! luma height */

  PosType   lx() const { return Y().x; }           /*! luma x-pos */
  PosType   ly() const { return Y().y; }           /*! luma y-pos */

  bool valid() const { return chromaFormat != NUM_CHROMA_FORMAT && blocks.size() > 0; }
};

inline UnitArea clipArea(const UnitArea &area, const UnitArea &boundingBox)
{
  UnitArea ret(area.chromaFormat);

  for (uint32_t i = 0; i < area.blocks.size(); i++)
  {
    ret.blocks.push_back(clipArea(area.blocks[i], boundingBox.blocks[i]));
  }

  return ret;
}

struct UnitAreaRelative : public UnitArea
{
  UnitAreaRelative(const UnitArea& origUnit, const UnitArea& unit)
  {
    *((UnitArea*)this) = unit;
    for(uint32_t i = 0; i < blocks.size(); i++)
    {
      blocks[i].positionRelativeTo(origUnit.blocks[i]);
    }
  }
};

class SPS;
class VPS;
class PPS;
class Slice;
struct CodingUnit;

#include "Buffer.h"


// ---------------------------------------------------------------------------
// transform unit
// ---------------------------------------------------------------------------

struct TransformUnit : public UnitArea
{
  CodingUnit     *cu;
  TransformUnit  *next;

  RDPCMMode       rdpcm        [MAX_NUM_TBLOCKS];
  uint8_t         maxScanPosX  [MAX_NUM_TBLOCKS];
  uint8_t         maxScanPosY  [MAX_NUM_TBLOCKS];
  uint8_t         mtsIdx       [MAX_NUM_TBLOCKS];
  int8_t          chromaQp     [2];

  ChannelType     chType;
  uint8_t         jointCbCr;
  int8_t          cbf;

  TransformUnit() : chType( CH_L ) {}
};

// ---------------------------------------------------------------------------
// coding unit
// ---------------------------------------------------------------------------

class  CodingStructure;

struct CodingUnit : public UnitArea
{
  TransformUnit     firstTU;
  TransformUnit    *lastTU;

  struct CtuData   *ctuData;
  CodingStructure  *cs;
  Slice            *slice;
        CodingUnit *next;
  const CodingUnit *above;
  const CodingUnit *left;
  Mv               *mvdL0SubPu; // 7 ptr (8 byte)
  Pel              *predBuf[MAX_NUM_COMPONENT];
  unsigned          idx;
  uint32_t          tileIdx;
  int32_t           flags2;
  int32_t           flags0; // 4 int (4 byte)

  bool              planeCbf[MAX_NUM_COMPONENT];
  
  Mv                mv       [NUM_REF_PIC_LIST_01][3];

  uint8_t           mvpIdx   [NUM_REF_PIC_LIST_01];
  int8_t            refIdx   [NUM_REF_PIC_LIST_01];
  int8_t            intraDir [MAX_NUM_CHANNEL_TYPE];

  SplitSeries       splitSeries;

  int8_t            flags1;
  uint8_t           geoSplitDir;
  uint8_t           mmvdIdx;
  
  uint8_t           qtDepth; // number of applied quad-splits, before switching to the multi-type-tree (mtt)
  int8_t            chromaQpAdj;
  int8_t            qp;
  uint8_t           depth;   // number of all splits, applied with generalized splits

  ChannelType       chType()                 const        { return ChannelType( ( flags2 >>  0 ) &   1 ); }
  bool              rootCbf()                const        { return bool       ( ( flags2 >>  1 ) &   1 ); }
  bool              skip()                   const        { return bool       ( ( flags2 >>  2 ) &   1 ); }
  bool              colorTransform()         const        { return bool       ( ( flags2 >>  3 ) &   1 ); }
  TreeType          treeType()               const        { return TreeType   ( ( flags2 >>  4 ) &   3 ); }
  ModeType          modeType()               const        { return ModeType   ( ( flags2 >>  6 ) &   3 ); }
  PredMode          predMode()               const        { return PredMode   ( ( flags2 >>  8 ) &   3 ); }
  int               ispMode()                const        { return int        ( ( flags2 >> 10 ) &   3 ); }
  int               bdpcmMode()              const        { return int        ( ( flags2 >> 12 ) &   3 ); }
  int               bdpcmModeChroma()        const        { return int        ( ( flags2 >> 14 ) &   3 ); }
  int               lfnstIdx()               const        { return int        ( ( flags2 >> 16 ) &   3 ); }
  uint8_t           sbtInfo()                const        { return uint8_t    ( ( flags2 >> 18 ) & 255 ); }

  void              setChType( ChannelType ch )           { flags2 = ( flags2 & ~(   1 <<  0 ) ) | ( ch <<  0 ); }
  void              setRootCbf( bool b )                  { flags2 = ( flags2 & ~(   1 <<  1 ) ) | (  b <<  1 ); }
  void              setSkip( bool b )                     { flags2 = ( flags2 & ~(   1 <<  2 ) ) | (  b <<  2 ); }
  void              setColorTransform( bool b )           { flags2 = ( flags2 & ~(   1 <<  3 ) ) | (  b <<  3 ); }
  void              setTreeType( TreeType n )             { flags2 = ( flags2 & ~(   3 <<  4 ) ) | (  n <<  4 ); }
  void              setModeType( ModeType n )             { flags2 = ( flags2 & ~(   3 <<  6 ) ) | (  n <<  6 ); }
  void              setPredMode( PredMode n )             { flags2 = ( flags2 & ~(   3 <<  8 ) ) | (  n <<  8 ); }
  void              setIspMode( int n )                   { flags2 = ( flags2 & ~(   3 << 10 ) ) | (  n << 10 ); }
  void              setBdpcmMode( int n )                 { flags2 = ( flags2 & ~(   3 << 12 ) ) | (  n << 12 ); }
  void              setBdpcmModeChroma( int n )           { flags2 = ( flags2 & ~(   3 << 14 ) ) | (  n << 14 ); }
  void              setLfnstIdx( int n )                  { flags2 = ( flags2 & ~(   3 << 16 ) ) | (  n << 16 ); }
  void              setSbtInfo( int n )                   { flags2 = ( flags2 & ~( 255 << 18 ) ) | (  n << 18 ); }

  // Prediction Unit Part
  
  bool              dmvrCondition()          const         { return bool       ( ( flags0 >>  0 ) &  1 ); }
  bool              mipTransposedFlag()      const         { return bool       ( ( flags0 >>  1 ) &  1 ); }
  bool              ciipFlag()               const         { return bool       ( ( flags0 >>  2 ) &  1 ); }
  bool              mergeFlag()              const         { return bool       ( ( flags0 >>  3 ) &  1 ); }
  bool              mmvdFlag()               const         { return bool       ( ( flags0 >>  5 ) &  1 ); }
  bool              affineFlag()             const         { return bool       ( ( flags0 >>  6 ) &  1 ); }
  MergeType         mergeType()              const         { return MergeType  ( ( flags0 >>  7 ) &  3 ); }
  AffineModel       affineType()             const         { return AffineModel( ( flags0 >>  9 ) &  1 ); }
  bool              geoFlag()                const         { return bool       ( ( flags0 >> 10 ) &  1 ); }
  int               mergeIdx()               const         { return int        ( ( flags0 >> 11 ) &  7 ); }
  int               geoMergeIdx0()           const         { return int        ( ( flags0 >> 11 ) &  7 ); }
  int               geoMergeIdx1()           const         { return int        ( ( flags0 >> 14 ) &  7 ); }
  int               interDir()               const         { return int        ( ( flags0 >> 17 ) &  3 ); }
  int               multiRefIdx()            const         { return int        ( ( flags0 >> 19 ) &  3 ); }
  bool              mipFlag()                const         { return bool       ( ( flags0 >> 21 ) &  1 ); }
  int               imv()                    const         { return int        ( ( flags0 >> 22 ) &  3 ); }
  int               smvdMode()               const         { return int        ( ( flags0 >> 24 ) &  3 ); }
  int               BcwIdx()                 const         { return int        ( ( flags0 >> 26 ) &  7 ); }
                    
  int               interDirrefIdxGeo0()     const         { return int        ( ( flags1 >>  0 ) & 15 ); }
  int               interDirrefIdxGeo1()     const         { return int        ( ( flags1 >>  4 ) & 15 ); }
                    
  void              setDmvrCondition( bool b )             { flags0 = ( flags0 & ~(  1 <<  0 ) ) | ( b  <<  0 ); }
  void              setMipTransposedFlag( bool b )         { flags0 = ( flags0 & ~(  1 <<  1 ) ) | ( b  <<  1 ); }
  void              setCiipFlag( bool b )                  { flags0 = ( flags0 & ~(  1 <<  2 ) ) | ( b  <<  2 ); }
  void              setMergeFlag( bool b )                 { flags0 = ( flags0 & ~(  1 <<  3 ) ) | ( b  <<  3 ); }
  void              setMmvdFlag( bool b )                  { flags0 = ( flags0 & ~(  1 <<  5 ) ) | ( b  <<  5 ); }
  void              setAffineFlag( bool b )                { flags0 = ( flags0 & ~(  1 <<  6 ) ) | ( b  <<  6 ); }
  void              setMergeType( MergeType mt )           { flags0 = ( flags0 & ~(  3 <<  7 ) ) | ( mt <<  7 ); }
  void              setAffineType( AffineModel at )        { flags0 = ( flags0 & ~(  1 <<  9 ) ) | ( at <<  9 ); CHECKD( at >= AFFINE_MODEL_NUM, "Needs to be '0' or '1'!" ); }
  void              setGeoFlag( bool b )                   { flags0 = ( flags0 & ~(  1 << 10 ) ) | ( b  << 10 ); }
  void              setMergeIdx( int id )                  { flags0 = ( flags0 & ~(  7 << 11 ) ) | ( id << 11 ); CHECKD( id >=  8, "Merge index needs to be smaller than '8'!"); }
  void              setGeoMergeIdx0( int id )              { flags0 = ( flags0 & ~(  7 << 11 ) ) | ( id << 11 ); CHECKD( id >=  8, "Merge index needs to be smaller than '8'!"); }
  void              setGeoMergeIdx1( int id )              { flags0 = ( flags0 & ~(  7 << 14 ) ) | ( id << 14 ); CHECKD( id >=  8, "Merge index needs to be smaller than '8'!"); }
  void              setInterDir( int id )                  { flags0 = ( flags0 & ~(  3 << 17 ) ) | ( id << 17 ); CHECKD( id >=  4, "Inter dir needs to be smaller than '4'!"); }
  void              setMultiRefIdx( int id )               { flags0 = ( flags0 & ~(  3 << 19 ) ) | ( id << 19 ); CHECKD( id >=  3, "Multi-ref. index needs to be smaller than '3'!"); }
  void              setMipFlag( bool b )                   { flags0 = ( flags0 & ~(  1 << 21 ) ) | ( b  << 21 ); }
  void              setImv( int id )                       { flags0 = ( flags0 & ~(  3 << 22 ) ) | ( id << 22 ); CHECKD( id >=  4, "IMV needs to be smaller than '4'!"); }
  void              setSmvdMode( int id )                  { flags0 = ( flags0 & ~(  3 << 24 ) ) | ( id << 24 ); CHECKD( id >=  4, "SMVD mode needs to be smaller than '4'!"); }
  void              setBcwIdx( int id )                    { flags0 = ( flags0 & ~(  7 << 26 ) ) | ( id << 26 ); CHECKD( id >=  5, "BCW idx needs to be smaller than '5'!"); }
                    
  void              setInterDirrefIdxGeo0( int id )        { flags1 = ( flags1 & ~( 15 <<  0 ) ) | ( id <<  0 ); CHECKD( id >= 16, "Inter dir needs to be smaller than '16'!"); }
  void              setInterDirrefIdxGeo1( int id )        { flags1 = ( flags1 & ~( 15 <<  4 ) ) | ( id <<  4 ); CHECKD( id >= 16, "Inter dir needs to be smaller than '16'!"); }
  
  CodingUnit() : flags2( 0 ), flags0( 0 ), flags1( 0 ) { }

  CodingUnit& operator=(const MotionInfo& mi);

  // for accessing motion information, which can have higher resolution than PUs (should always be used, when accessing neighboring motion information)
  const MotionInfo& getMotionInfo() const;
  const MotionInfo& getMotionInfo( const Position& pos ) const;
  MotionBuf         getMotionBuf();
  CMotionBuf        getMotionBuf() const;

  void              minInit( const UnitArea& unit );
};

typedef CodingUnit PredictionUnit;

// ---------------------------------------------------------------------------
// Utility class for easy for-each like unit traversing
// ---------------------------------------------------------------------------

#include <iterator>

template<typename T>
class UnitIterator : public std::iterator<std::forward_iterator_tag, T>
{
private:
  T* m_punit;

public:
  UnitIterator(           ) : m_punit( nullptr ) { }
  UnitIterator( T* _punit ) : m_punit( _punit  ) { }

  typedef T&       reference;
  typedef T const& const_reference;
  typedef T*       pointer;
  typedef T const* const_pointer;

  reference        operator*()                                      { return *m_punit; }
  const_reference  operator*()                                const { return *m_punit; }
  pointer          operator->()                                     { return  m_punit; }
  const_pointer    operator->()                               const { return  m_punit; }

  UnitIterator<T>& operator++()                                     { m_punit = m_punit->next; return *this; }
  UnitIterator<T>  operator++( int )                                { auto x = *this; ++( *this ); return x; }
  bool             operator!=( const UnitIterator<T>& other ) const { return m_punit != other.m_punit; }
  bool             operator==( const UnitIterator<T>& other ) const { return m_punit == other.m_punit; }
};

template<typename T>
class UnitTraverser
{
private:
  T* m_begin;
  T* m_end;

public:
  UnitTraverser(                    ) : m_begin( nullptr ), m_end( nullptr ) { }
  UnitTraverser( T* _begin, T* _end ) : m_begin( _begin  ), m_end( _end    ) { }

  typedef T                     value_type;
  typedef size_t                size_type;
  typedef T&                    reference;
  typedef T const&              const_reference;
  typedef T*                    pointer;
  typedef T const*              const_pointer;
  typedef UnitIterator<T>       iterator;
  typedef UnitIterator<const T> const_iterator;

  iterator        begin()        { return UnitIterator<T>( m_begin ); }
  const_iterator  begin()  const { return UnitIterator<T>( m_begin ); }
  const_iterator  cbegin() const { return UnitIterator<T>( m_begin ); }
  iterator        end()          { return UnitIterator<T>( m_end   ); }
  const_iterator  end()    const { return UnitIterator<T>( m_end   ); }
  const_iterator  cend()   const { return UnitIterator<T>( m_end   ); }
};

typedef UnitTraverser<CodingUnit>            CUTraverser;
typedef UnitTraverser<const CodingUnit>     cCUTraverser;

typedef UnitTraverser<      TransformUnit>   TUTraverser;
typedef UnitTraverser<const TransformUnit>  cTUTraverser;

#include <memory>
#include <mutex>

// ---------------------------------------------------------------------------
// dynamic cache
// ---------------------------------------------------------------------------

static constexpr size_t DYN_CACHE_CHUNK_SIZE = 1024;

template<typename T>
class dynamic_cache
{
  std::vector<T*> m_cache;
  std::vector<T*> m_cacheChunks;

public:

  ~dynamic_cache()
  {
    deleteEntries();
  }

  void deleteEntries()
  {
    for( auto &chunk : m_cacheChunks )
    {
      free( chunk );
    }

    m_cache      .clear();
    m_cacheChunks.clear();
  }

  T* get()
  {
    T* ret;

    if( !m_cache.empty() )
    {
      ret = m_cache.back();
      m_cache.pop_back();
    }
    else
    {
      T* chunk = (T*) malloc( DYN_CACHE_CHUNK_SIZE * sizeof( T ) );

      //GCC_WARNING_DISABLE_class_memaccess
      //memset( chunk, 0, DYN_CACHE_CHUNK_SIZE * sizeof( T ) );
      //GCC_WARNING_RESET

      m_cacheChunks.push_back( chunk );
      m_cache      .reserve( m_cache.size() + DYN_CACHE_CHUNK_SIZE );

      for( ptrdiff_t p = 0; p < DYN_CACHE_CHUNK_SIZE; p++ )
      {
        //m_cache.push_back( &chunk[DYN_CACHE_CHUNK_SIZE - p - 1] );
        m_cache.push_back( &chunk[p] );
      }

      ret = m_cache.back();
      m_cache.pop_back();
    }

    return ret;
  }

  void defragment()
  {
    m_cache.clear();

    for( T* chunk : m_cacheChunks )
    {
      //GCC_WARNING_DISABLE_class_memaccess
      //memset( chunk, 0, DYN_CACHE_CHUNK_SIZE * sizeof( T ) );
      //GCC_WARNING_RESET

      for( ptrdiff_t p = 0; p < DYN_CACHE_CHUNK_SIZE; p++ )
      {
        //m_cache.push_back( &chunk[DYN_CACHE_CHUNK_SIZE - p - 1] );
        m_cache.push_back( &chunk[p] );
      }
    }
  }

  void cache( std::vector<T*>& vel )
  {
    m_cache.insert( m_cache.end(), vel.begin(), vel.end() );
    vel.clear();
  }
};

typedef dynamic_cache<struct CodingUnit> CUCache;
typedef dynamic_cache<struct TransformUnit> TUCache;

struct ThreadSafeCUCache
{
  std::shared_ptr<CUCache> getCuCache();
  std::shared_ptr<TUCache> getTuCache();

private:
  std::mutex m_mutex;

  // we use the shared_ptr reference-count to track if a cache is in use or available
  std::vector< std::shared_ptr<CUCache> > m_cuCaches;
  std::vector< std::shared_ptr<TUCache> > m_tuCaches;
};


#endif

