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

/** \file     Unit.h
 *  \brief    defines unit as a set of blocks and basic unit types (coding, prediction, transform)
 */

#pragma once

#include "CommonDef.h"
#include "Common.h"
#include "Mv.h"
#include "MotionInfo.h"
#include "ChromaFormat.h"

#include <mutex>

namespace vvdec
{

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
  CompArea() : Area()                                                                                                                                                          { }
  CompArea(const ComponentID _compID, const Area &_area)                                                          : Area(_area.x, _area.y, _area.width, _area.height, _compID) { }
  CompArea(const ComponentID _compID, const Position& _pos, const Size& _size)                                    : Area(_pos, Size( _size.width, _size.height, _compID))      { }
  CompArea(const ComponentID _compID, const uint32_t _x, const uint32_t _y, const uint32_t _w, const uint32_t _h) : Area(_x, _y, _w, _h, _compID)                              { }

  Position chromaPos(const ChromaFormat chromaFormat) const;
  Position lumaPos(const ChromaFormat chromaFormat)   const;

  Size     chromaSize(const ChromaFormat chromaFormat) const;
  Size     lumaSize(const ChromaFormat chromaFormat)   const;

  Position compPos(const ChromaFormat chromaFormat, const ComponentID compID) const;
  Position chanPos(const ChromaFormat chromaFormat, const ChannelType chType) const;

  Position topLeftComp    (const ChromaFormat chromaFormat, const ComponentID _compID) const { return recalcPosition(chromaFormat, compID(), _compID, *this);                                                     }
  Position topRightComp   (const ChromaFormat chromaFormat, const ComponentID _compID) const { return recalcPosition(chromaFormat, compID(), _compID, { (PosType) (x + width - 1), y                          }); }
  Position bottomLeftComp (const ChromaFormat chromaFormat, const ComponentID _compID) const { return recalcPosition(chromaFormat, compID(), _compID, { x                        , (PosType) (y + height - 1 )}); }
  Position bottomRightComp(const ChromaFormat chromaFormat, const ComponentID _compID) const { return recalcPosition(chromaFormat, compID(), _compID, { (PosType) (x + width - 1), (PosType) (y + height - 1 )}); }

  bool valid() const { return compID() < MAX_NUM_TBLOCKS && width != 0 && height != 0; }

  bool operator==(const CompArea &other) const
  {
    if (compID()       != other.compID())       return false;

    return Position::operator==(other) && Size::operator==(other);
  }

  bool operator!=(const CompArea &other) const { return !(operator==(other)); }

  void     repositionTo      (const Position& newPos)       { Position::repositionTo(newPos); }
  void     positionRelativeTo(const CompArea& origCompArea) { Position::relativeTo(origCompArea); }
};

inline CompArea clipArea(const CompArea &compArea, const Area &boundingBox)
{
  return CompArea(compArea.compID(), clipArea((const Area&) compArea, boundingBox));
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

}

#include "Buffer.h"

namespace vvdec
{

// ---------------------------------------------------------------------------
// transform unit
// ---------------------------------------------------------------------------

struct TransformUnit : public UnitArea
{
  CodingUnit     *cu;
  TransformUnit  *next;
  unsigned        idx;

  uint8_t         maxScanPosX  [MAX_NUM_TBLOCKS];
  uint8_t         maxScanPosY  [MAX_NUM_TBLOCKS];
  int8_t          chromaQp     [2];

  uint8_t         _chType    : 2;
  uint8_t         jointCbCr  : 2;
  uint8_t         cbf        : 3;
  uint8_t         _mtsIdxL   : 3;
  uint8_t         _mtsIdxU   : 1;
  uint8_t         _mtsIdxV   : 1;

  ChannelType     chType()                const { return ChannelType( _chType ); }
  void            setChType( ChannelType ch )   { _chType = ch; }
  uint8_t         mtsIdx   ( int c )      const { return !c ? _mtsIdxL :             c == 1 ? _mtsIdxU :         _mtsIdxV; }
  void            setMtsIdx( int c, uint8_t v ) { if   ( !c ) _mtsIdxL = v; else if( c == 1 ) _mtsIdxU = v; else _mtsIdxV = v; }
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
  const Slice      *slice;
  const PPS        *pps;
  const SPS        *sps;
        CodingUnit *next;
  const CodingUnit *above;
  const CodingUnit *left;
  ptrdiff_t         mvdL0SubPuOff; // 7 ptr (8 byte)
  ptrdiff_t         predBufOff;
  uint32_t          idx;
  uint32_t          tileIdx;

  Mv                mv       [NUM_REF_PIC_LIST_01][3];

  uint8_t           mvpIdx   [NUM_REF_PIC_LIST_01];
  int8_t            refIdx   [NUM_REF_PIC_LIST_01];
  int8_t            intraDir [MAX_NUM_CHANNEL_TYPE];

  SplitSeries       splitSeries;

  uint8_t           geoSplitDir;
  uint8_t           mmvdIdx;

  int8_t            chromaQpAdj;
  int8_t            qp;
  uint8_t           _interDir       : 2;
  uint8_t           _imv            : 2;
  uint8_t           _bcw            : 3;
  uint8_t           _mergeType      : 2;
  bool              _dmvrCond;      // avoid data races in a separate bit

  uint8_t           _sbtInfo;
  uint8_t           qtDepth         : 3;
  uint8_t           depth           : 4;
  uint8_t           _chType         : 1;

  bool              _rootCbf        : 1;
  bool              _skip           : 1;
  bool              _colorTransform : 1;
  bool              _mipTranspose   : 1;
  uint8_t           _treeType       : 2;
  uint8_t           _modeType       : 2;

  uint8_t           _predMode       : 2;
  uint8_t           _bdpcmL         : 2;
  uint8_t           _bdpcmC         : 2;
  uint8_t           _lfnstIdx       : 2;

  uint8_t           _ispIdx         : 2;
  bool              _ciipFlag       : 1;
  bool              _mergeFlag      : 1;
  bool              _mmvdFlag       : 1;
  bool              _affineFlag     : 1;
  bool              _geoFlag        : 1;

  uint8_t           _mrgIdx         : 3;
  uint8_t           _geoMrgIdx0     : 3;

  uint8_t           _geoMrgIdx1     : 3;
  uint8_t           _affineType     : 2;
  bool              _mipFlag        : 1;

  uint8_t           _multiRefIdx    : 2;
  bool              planeCbfY       : 1;
  bool              planeCbfU       : 1;
  bool              planeCbfV       : 1;

  uint8_t           _smvd           : 2;

  uint8_t            _geoDir0, _geoDir1;

  uint8_t           sbtInfo()                const        { return _sbtInfo; }
  ChannelType       chType()                 const        { return ChannelType( _chType ); }
  bool              rootCbf()                const        { return _rootCbf; }
  bool              skip()                   const        { return _skip; }
  bool              colorTransform()         const        { return _colorTransform; }
  TreeType          treeType()               const        { return TreeType( _treeType ); }
  ModeType          modeType()               const        { return ModeType( _modeType ); }
  PredMode          predMode()               const        { return PredMode( _predMode ); }
  uint8_t           ispMode()                const        { return _ispIdx; }
  uint8_t           bdpcmMode()              const        { return _bdpcmL; }
  uint8_t           bdpcmModeChroma()        const        { return _bdpcmC; }
  uint8_t           lfnstIdx()               const        { return _lfnstIdx; }
  bool              planeCbf( int c )        const        { return !c ? planeCbfY : c == 1 ? planeCbfU : planeCbfV; }

  void              setChType( ChannelType ch )           { _chType         = ch ; }
  void              setRootCbf( bool b )                  { _rootCbf        =  b ; }
  void              setSkip( bool b )                     { _skip           =  b ; }
  void              setColorTransform( bool b )           { _colorTransform =  b ; }
  void              setTreeType( TreeType n )             { _treeType       =  n ; }
  void              setModeType( ModeType n )             { _modeType       =  n ; }
  void              setPredMode( PredMode n )             { _predMode       =  n ; }
  void              setIspMode( uint8_t n )               { _ispIdx         =  n ; }
  void              setBdpcmMode( uint8_t n )             { _bdpcmL         =  n ; }
  void              setBdpcmModeChroma( uint8_t n )       { _bdpcmC         =  n ; }
  void              setLfnstIdx( uint8_t n )              { _lfnstIdx       =  n ; }
  void              setSbtInfo( uint8_t n )               { _sbtInfo        =  n ; }
  void              setPlaneCbf( int c, bool b )          { if( !c ) planeCbfY = b; else if( c == 1 ) planeCbfU = b; else planeCbfV = b; }

  // Prediction Unit Part

  bool              dmvrCondition()          const         { return _dmvrCond; }
  bool              mipTransposedFlag()      const         { return _mipTranspose; }
  bool              ciipFlag()               const         { return _ciipFlag; }
  bool              mergeFlag()              const         { return _mergeFlag; }
  bool              mmvdFlag()               const         { return _mmvdFlag; }
  bool              affineFlag()             const         { return _affineFlag; }
  MergeType         mergeType()              const         { return MergeType( _mergeType ); }
  AffineModel       affineType()             const         { return AffineModel( _affineType ); }
  bool              geoFlag()                const         { return _geoFlag; }
  uint8_t           mergeIdx()               const         { return _mrgIdx; }
  uint8_t           geoMergeIdx0()           const         { return _geoMrgIdx0; }
  uint8_t           geoMergeIdx1()           const         { return _geoMrgIdx1; }
  uint8_t           interDir()               const         { return _interDir; }
  uint8_t           multiRefIdx()            const         { return _multiRefIdx; }
  bool              mipFlag()                const         { return _mipFlag; }
  uint8_t           imv()                    const         { return _imv; }
  uint8_t           smvdMode()               const         { return _smvd; }
  uint8_t           BcwIdx()                 const         { return _bcw; }

  uint8_t           interDirrefIdxGeo0()     const         { return _geoDir0; }
  uint8_t           interDirrefIdxGeo1()     const         { return _geoDir1; }

  void              setDmvrCondition( bool b )             { _dmvrCond      = b ; }
  void              setMipTransposedFlag( bool b )         { _mipTranspose  = b ; }
  void              setCiipFlag( bool b )                  { _ciipFlag      = b ; }
  void              setMergeFlag( bool b )                 { _mergeFlag     = b ; }
  void              setMmvdFlag( bool b )                  { _mmvdFlag      = b ; }
  void              setAffineFlag( bool b )                { _affineFlag    = b ; }
  void              setMergeType( MergeType mt )           { _mergeType     = mt; }
  void              setAffineType( AffineModel at )        { _affineType    = at; CHECKD( at >= AFFINE_MODEL_NUM, "Needs to be '0' or '1'!" ); }
  void              setGeoFlag( bool b )                   { _geoFlag       = b ; }
  void              setMergeIdx( uint8_t id )              { _mrgIdx        = id; CHECKD( id >=  8, "Merge index needs to be smaller than '8'!"); }
  void              setGeoMergeIdx0( uint8_t id )          { _geoMrgIdx0    = id; CHECKD( id >=  8, "Merge index needs to be smaller than '8'!"); }
  void              setGeoMergeIdx1( uint8_t id )          { _geoMrgIdx1    = id; CHECKD( id >=  8, "Merge index needs to be smaller than '8'!"); }
  void              setInterDir( uint8_t id )              { _interDir      = id; CHECKD( id >=  4, "Inter dir needs to be smaller than '4'!"); }
  void              setMultiRefIdx( uint8_t id )           { _multiRefIdx   = id; CHECKD( id >=  3, "Multi-ref. index needs to be smaller than '3'!"); }
  void              setMipFlag( bool b )                   { _mipFlag       = b ; }
  void              setImv( uint8_t id )                   { _imv           = id; CHECKD( id >=  4, "IMV needs to be smaller than '4'!"); }
  void              setSmvdMode( uint8_t id )              { _smvd          = id; CHECKD( id >=  4, "SMVD mode needs to be smaller than '4'!"); }
  void              setBcwIdx( uint8_t id )                { _bcw           = id; CHECKD( id >=  5, "BCW idx needs to be smaller than '5'!"); }

  void              setInterDirrefIdxGeo0( uint8_t id )    { _geoDir0       = id; }
  void              setInterDirrefIdxGeo1( uint8_t id )    { _geoDir1       = id; }

  CodingUnit& operator=(const MotionInfo& mi);

  // for accessing motion information, which can have higher resolution than PUs (should always be used, when accessing neighboring motion information)
  const MotionInfo& getMotionInfo() const;
  const MotionInfo& getMotionInfo( const Position& pos ) const;
  MotionBuf         getMotionBuf();
  CMotionBuf        getMotionBuf() const;

  void              minInit( const UnitArea& unit );
};

// ---------------------------------------------------------------------------
// Utility class for easy for-each like unit traversing
// ---------------------------------------------------------------------------

}

#include <iterator>

namespace vvdec {

template<typename T>
class UnitIterator
{
private:
  T* m_punit = nullptr;

public:
  explicit UnitIterator( T* _punit ) : m_punit( _punit ) {}

  using iterator_category = std::forward_iterator_tag;
  using value_type        = T;
  using pointer           = T*;
  using const_pointer     = T const*;
  using reference         = T&;
  using const_reference   = T const&;
  using difference_type   = ptrdiff_t;

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

}

#include <memory>
#include <mutex>

namespace vvdec {

// ---------------------------------------------------------------------------
// dynamic cache
// ---------------------------------------------------------------------------

static constexpr size_t DYN_CACHE_CHUNK_SIZE = 1024;

template<typename T>
class thread_safe_chunk_cache
{
  std::vector<T*> m_cacheChunks;
  std::mutex      m_mutex;

public:

  ~thread_safe_chunk_cache()
  {
    clear_chunks();
  }

  void clear_chunks()
  {
    for( auto& chunk : m_cacheChunks )
    {
      free( chunk );
    }

    m_cacheChunks.clear();
  }

  T* get()
  {
    std::unique_lock<std::mutex> l( m_mutex );

    if( m_cacheChunks.empty() )
    {
      l.unlock();
      return ( T* ) malloc( DYN_CACHE_CHUNK_SIZE * sizeof( T ) );
    }
    else
    {
      T* chunk = m_cacheChunks.back();
      m_cacheChunks.pop_back();
      return chunk;
    }
  }

  void cache( std::vector<T*>& chunks )
  {
    std::unique_lock<std::mutex> l( m_mutex );

    m_cacheChunks.insert( m_cacheChunks.end(), chunks.begin(), chunks.end() );
    chunks.clear();
  }
};

template<typename T>
class dynamic_cache
{
  ptrdiff_t       m_lastIdx;
  std::vector<T*> m_cache;

  thread_safe_chunk_cache<T> *m_chunkCache;

  void clear_chunks()
  {
    m_chunkCache->cache( m_cache );
  }

public:

  explicit dynamic_cache( thread_safe_chunk_cache<T>* chunkCache ) : m_lastIdx( DYN_CACHE_CHUNK_SIZE ), m_chunkCache( chunkCache )
  {
  }

  ~dynamic_cache()
  {
    clear_chunks();
  }

  T* get()
  {
    T* ret;

    if( m_lastIdx < DYN_CACHE_CHUNK_SIZE )
    {
      ret = &m_cache.back()[m_lastIdx++];
    }
    else
    {
      T* chunk = m_chunkCache->get();

      m_cache.push_back( chunk );

      ret = &chunk[0];
      m_lastIdx = 1;
    }

    return ret;
  }

  void releaseAll()
  {
    clear_chunks();
    m_lastIdx = DYN_CACHE_CHUNK_SIZE;
  }
};

typedef dynamic_cache<struct CodingUnit>    CUCache;
typedef dynamic_cache<struct TransformUnit> TUCache;

typedef thread_safe_chunk_cache<struct CodingUnit>    CUChunkCache;
typedef thread_safe_chunk_cache<struct TransformUnit> TUChunkCache;

}

