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

/** \file     Picture.cpp
 *  \brief    Description of a coded picture
 */

#include "Picture.h"
#include "ChromaFormat.h"

// ---------------------------------------------------------------------------
// picture methods
// ---------------------------------------------------------------------------

namespace vvdec
{

void paddPicBorderTopCore(Pel *pi, ptrdiff_t stride,int width,int xmargin,int ymargin)
{
  for( int x = 0; x < xmargin; x++ )
  {
    pi[-xmargin + x] = pi[0];
    pi[width + x] = pi[width - 1];
  }
  pi -= xmargin;
  // pi is now (-marginX, 0)
  for( int y = 0; y < ymargin; y++ )
  {
    ::memcpy( pi - ( y + 1 )*stride, pi, sizeof( Pel )*( width + ( xmargin << 1 ) ) );
  }
}

void paddPicBorderBotCore(Pel *pi, ptrdiff_t stride,int width,int xmargin,int ymargin)
{
  for( int x = 0; x < xmargin; x++ )
  {
    pi[-xmargin + x] = pi[0];
    pi[width  + x] = pi[width - 1];
  }
  pi -= xmargin;
  // pi is now the (-marginX, height-1)
  for( int y = 0; y < ymargin; y++ )
  {
    ::memcpy( pi + ( y + 1 )*stride, pi, sizeof( Pel )*( width + ( xmargin << 1 ) ) );
  }
}

void paddPicBorderLeftRightCore(Pel *pi, ptrdiff_t stride,int width,int xmargin,int height)
{
  for( int y = 1; y < ( height - 1 ); y++ )
   {
     for( int x = 0; x < xmargin; x++ )
     {
       pi[-xmargin + x] = pi[0];
       pi[width + x] = pi[width - 1];
     }
     pi += stride;
   }
}

void Picture::create(const ChromaFormat &_chromaFormat, const Size &size, const unsigned _maxCUSize, const unsigned _margin, const int _layerId, UserAllocator* _userAllocator )
{
  layerId = _layerId;
  UnitArea::operator=( UnitArea( _chromaFormat, Area( Position{ 0, 0 }, size ) ) );
  margin            = _margin;
  m_bufs[PIC_RECONSTRUCTION].create( _chromaFormat, size, _maxCUSize, _margin, MEMORY_ALIGN_DEF_SIZE, true, _userAllocator );
}

void Picture::createWrapAroundBuf( const bool isWrapAround, const unsigned _maxCUSize )
{
  if( isWrapAround )
    m_bufs[PIC_RECON_WRAP].create( chromaFormat, Y().size(), _maxCUSize, margin, MEMORY_ALIGN_DEF_SIZE );
}

void Picture::resetForUse( int _layerId )
{
  CHECK( lockedByApplication, "the picture can not be re-used, because it has not been unlocked by the application." );

  if( cs )
  {
    cs->resetForUse();
  }

  m_subPicRefBufs.clear();

  m_dProcessingTime       = 0;
  subPicExtStarted        = false;
  borderExtStarted        = false;
  dpbReferenceMark        = unreferenced;
  stillReferenced         = false;
  isReferencePic          = false;
  progress                = Picture::init;
  neededForOutput         = false;
  wasLost                 = false;
  error                   = false;
  exceptionThrownOut      = false;
  topField                = false;
  fieldPic                = false;
  nonReferencePictureFlag = false;
  skippedDecCount         = 0;

  picCheckedDPH = false;
  subpicsCheckedDPH.clear();
  dphMismatch   = false;

  lockedByApplication = false;

  poc                 = 0;
  cts                 = 0;
  dts                 = 0;
  tempLayer           = std::numeric_limits<uint32_t>::max();
  depth               = 0;
  layerId             = _layerId;
  eNalUnitType        = NAL_UNIT_INVALID;
  bits                = 0;
  rap                 = 0;
  decodingOrderNumber = 0;

  sliceSubpicIdx.clear();
  subPictures.clear();

  subLayerNonReferencePictureDueToSTSA = 0;

  m_divTasksCounter     .clearException();
  m_ctuTaskCounter      .clearException();
  m_borderExtTaskCounter.clearException();
  m_copyWrapBufDone     .clearException();
  reconDone             .clearException();
  parseDone             .clearException();
#if RECO_WHILE_PARSE
  std::for_each( ctuParsedBarrier.begin(), ctuParsedBarrier.end(), []( auto& b ) { b.clearException(); } );
#endif

  clearSliceBuffer();

  SEI_internal::deleteSEIs( seiMessageList );

  reconDone.lock();
}

void Picture::destroy()
{
  CHECK( lockedByApplication, "the picture can not be destroyed, because it has not been unlocked by the application." );

  for (uint32_t t = 0; t < NUM_PIC_TYPES; t++)
  {
    m_bufs[t].destroy();
  }

  if( cs )
  {
    cs->destroy();
    delete cs;
    cs = nullptr;
  }

#if  RECO_WHILE_PARSE
  ctuParsedBarrier.clear();
#endif

  clearSliceBuffer();

  SEI_internal::deleteSEIs( seiMessageList );

  subpicsCheckedDPH.clear();

  m_divTasksCounter     .clearException();
  m_ctuTaskCounter      .clearException();
  m_borderExtTaskCounter.clearException();
  m_copyWrapBufDone     .clearException();
  reconDone             .clearException();
  parseDone             .clearException();
#if RECO_WHILE_PARSE
  std::for_each( ctuParsedBarrier.begin(), ctuParsedBarrier.end(), []( auto& b ) { b.clearException(); } );
#endif
}

       PelBuf     Picture::getRecoBuf(const ComponentID compID, bool wrap)       { return getBuf(compID, wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION); }
const CPelBuf     Picture::getRecoBuf(const ComponentID compID, bool wrap) const { return getBuf(compID, wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION); }
       PelBuf     Picture::getRecoBuf(const CompArea &blk, bool wrap)            { return getBuf(blk,    wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION); }
const CPelBuf     Picture::getRecoBuf(const CompArea &blk, bool wrap)      const { return getBuf(blk,    wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION); }
       PelUnitBuf Picture::getRecoBuf(const UnitArea &unit, bool wrap)           { return getBuf(unit,   wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION); }
const CPelUnitBuf Picture::getRecoBuf(const UnitArea &unit, bool wrap)     const { return getBuf(unit,   wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION); }
       PelUnitBuf Picture::getRecoBuf( bool wrap )                               { return wrap ? m_bufs[PIC_RECON_WRAP] : m_bufs[PIC_RECONSTRUCTION]; }
const CPelUnitBuf Picture::getRecoBuf( bool wrap )                         const { return wrap ? m_bufs[PIC_RECON_WRAP] : m_bufs[PIC_RECONSTRUCTION]; }

void Picture::finalInit( CUChunkCache* cuChunkCache, TUChunkCache* tuChunkCache, const SPS *sps, const PPS *pps, const std::shared_ptr<PicHeader>& ph, const APS* const alfApss[ALF_CTB_MAX_NUM_APS], const APS* lmcsAps, const APS* scalingListAps, bool phPSupdate )
{
  SEI_internal::deleteSEIs( seiMessageList );
  clearSliceBuffer();

  const ChromaFormat chromaFormatIDC = sps->getChromaFormatIdc();
  const int          iWidth          = pps->getPicWidthInLumaSamples();
  const int          iHeight         = pps->getPicHeightInLumaSamples();

  if( !cs )
  {
    cs = new CodingStructure( cuChunkCache, tuChunkCache );
    cs->create( chromaFormatIDC, Area( 0, 0, iWidth, iHeight ) );
  }

#if RECO_WHILE_PARSE
  if( ctuParsedBarrier.size() != pps->pcv->sizeInCtus )
  {
    ctuParsedBarrier = std::vector<Barrier>( pps->pcv->sizeInCtus );
  }
#endif

  parseDone   . lock();
  cs->picture   = this;
  cs->picHeader = ph;
  cs->pps       = pps ? pps->getSharedPtr() : nullptr;
  cs->sps       = sps ? sps->getSharedPtr() : nullptr;

  if( phPSupdate )
  {
    ph->setSPSId         ( sps->getSPSId() );
    ph->setPPSId         ( pps->getPPSId() );
    ph->setLmcsAPS       ( lmcsAps        ? lmcsAps       ->getSharedPtr() : nullptr );
    ph->setScalingListAPS( scalingListAps ? scalingListAps->getSharedPtr() : nullptr );
  }
  nonReferencePictureFlag = ph->getNonReferencePictureFlag();

  for( int i = 0; i < ALF_CTB_MAX_NUM_APS; ++i )
  {
    cs->alfApss[i] = alfApss[i] ? alfApss[i]->getSharedPtr() : nullptr;
  }
  if( lmcsAps )
  {
    cs->lmcsAps = lmcsAps ? lmcsAps->getSharedPtr() : nullptr;
  }

  cs->pcv = pps->pcv.get();
  cs->allocTempInternals();
  cs->rebindPicBufs();

  resetProcessingTime();

  paddPicBorderBot       = paddPicBorderBotCore;
  paddPicBorderTop       = paddPicBorderTopCore;
  paddPicBorderLeftRight = paddPicBorderLeftRightCore;

#if ENABLE_SIMD_OPT_PICTURE && defined( TARGET_SIMD_X86 )
  initPictureX86();
#endif
}

Slice* Picture::allocateNewSlice( Slice** pilot )
{
  if( pilot )
  {
    slices.push_back( *pilot );
    *pilot = new Slice;
    if( slices.size() >= 2 )
    {
      ( *pilot )->copySliceInfo( slices[slices.size() - 2] );
      ( *pilot )->initSlice();
    }
    ( *pilot )->setSPS( 0 );
    ( *pilot )->setPPS( 0 );
    ( *pilot )->setVPS( 0 );
    ( *pilot )->clearAlfAPSs();
  }
  else
  {
    slices.push_back( new Slice );
    if( slices.size() >= 2 )
    {
      slices.back()->copySliceInfo( slices[slices.size() - 2] );
      slices.back()->initSlice();
    }
  }

  Slice* slice = slices.back();

  slice->setPPS( cs->pps.get() );
  slice->setSPS( cs->sps.get() );
  slice->setVPS( cs->vps.get() );
  slice->setAlfApss( cs->alfApss );
  slice->setPic( this );

  return slice;
}

void Picture::clearSliceBuffer()
{
  for( auto &s: slices )
  {
    delete s;
  }
  slices.clear();
}

bool Picture::lastSliceOfPicPresent() const
{
  if( slices.empty() )
  {
    return false;
  }
  const Slice*   lastSlice      = slices.back();
  const unsigned lastCtuInSlice = lastSlice->getCtuAddrInSlice( lastSlice->getNumCtuInSlice() - 1 );
  return lastCtuInSlice == lastSlice->getPPS()->pcv->sizeInCtus - 1;
}

void Picture::waitForAllTasks()
{
  m_ctuTaskCounter.wait_nothrow();
  m_borderExtTaskCounter.wait_nothrow();
  m_divTasksCounter.wait_nothrow();   // this waits for the slice parsing and the finishPic Task
}

void Picture::ensureUsableAsRef()
{
#if RECO_WHILE_PARSE
  ctuParsedBarrier.clear();
#endif
  neededForOutput = false;

  // set referenced to true, because we don't know if it has been set correctly, but that way it will be available as a reference pic
  dpbReferenceMark = ShortTerm;
  stillReferenced  = true;
  isReferencePic   = true;

  // ensure cs->m_colMiMap is set to zero
  cs->initStructData();

  CHECK_FATAL( reconDone.hasException(), "to be usable as reference the picture should not have an Exception reconDone barrier" );
}

void Picture::fillGrey( const SPS* sps )
{
  // fill in grey buffer for missing reference pictures (GDR or broken bitstream)
  const uint32_t yFill = 1 << ( sps->getBitDepth() - 1 );
  const uint32_t cFill = 1 << ( sps->getBitDepth() - 1 );
  getRecoBuf().Y().fill( yFill );
  getRecoBuf().Cb().fill( cFill );
  getRecoBuf().Cr().fill( cFill );

  progress = Picture::reconstructed;
  reconDone.unlock();
}

void Picture::extendPicBorder( bool top, bool bottom, bool leftrightT, bool leftrightB, ChannelType chType )
{
  if( cs->pps->getUseWrapAround() )
  {
    extendPicBorderWrap( top, bottom, leftrightT, leftrightB, chType );
  }

  extendPicBorderBuf( m_bufs[PIC_RECONSTRUCTION], top, bottom, leftrightT, leftrightB, chType );
}

void Picture::extendPicBorderWrap( bool top, bool bottom, bool leftrightT, bool leftrightB, ChannelType chType )
{
  for( int comp = 0; comp < getNumberValidComponents( cs->area.chromaFormat ); comp++ )
  {
    ComponentID compID = ComponentID( comp );

    if( chType != MAX_NUM_CHANNEL_TYPE && toChannelType( compID ) != chType )
      continue;

    PelBuf prw = m_bufs[PIC_RECON_WRAP].get( compID );

    const int xmargin = margin >> getComponentScaleX( compID, cs->area.chromaFormat );
    const int ymargin = margin >> getComponentScaleY( compID, cs->area.chromaFormat );

    int xoffset = cs->pps->getWrapAroundOffset() >> getComponentScaleX( compID, cs->area.chromaFormat );
    if( leftrightT )
    {
      Pel* piprw = prw.bufAt( 0, 1 );

      for( int y = 1; y < prw.height / 2; y++ )
      {
        for( int x = 0; x < xmargin; x++ )
        {
          if( x < xoffset )
          {
            piprw[-x - 1]        = piprw[-x - 1 + xoffset];
            piprw[prw.width + x] = piprw[prw.width + x - xoffset];
          }
          else
          {
            piprw[-x - 1]        = piprw[0];
            piprw[prw.width + x] = piprw[prw.width - 1];
          }
        }
        piprw += prw.stride;
      }
    }
    if( leftrightB )
    {
      Pel* piprw = prw.bufAt( 0, prw.height / 2 );

      for( int y = 1; y < prw.height / 2; y++ )
      {
        for( int x = 0; x < xmargin; x++ )
        {
          if( x < xoffset )
          {
            piprw[-x - 1]        = piprw[-x - 1 + xoffset];
            piprw[prw.width + x] = piprw[prw.width + x - xoffset];
          }
          else
          {
            piprw[-x - 1]        = piprw[0];
            piprw[prw.width + x] = piprw[prw.width - 1];
          }
        }
        piprw += prw.stride;
      }
    }

    if( bottom )
    {
      Pel* piprw = prw.bufAt( 0, prw.height - 1 );

      for( int x = 0; x < xmargin; x++ )
      {
        if( x < xoffset )
        {
          piprw[-x - 1]        = piprw[-x - 1 + xoffset];
          piprw[prw.width + x] = piprw[prw.width + x - xoffset];
        }
        else
        {
          piprw[-x - 1]        = piprw[0];
          piprw[prw.width + x] = piprw[prw.width - 1];
        }
      }
      piprw -= xmargin;
      // pi is now the (-marginX, height-1)
      for( int y = 0; y < ymargin; y++ )
      {
        ::memcpy( piprw + ( y + 1 ) * prw.stride, piprw, sizeof( Pel ) * ( prw.width + ( xmargin << 1 ) ) );
      }
    }
    if( top )
    {
      Pel* piprw = prw.bufAt( 0, 0 );

      for( int x = 0; x < xmargin; x++ )
      {
        if( x < xoffset )
        {
          piprw[-x - 1]        = piprw[-x - 1 + xoffset];
          piprw[prw.width + x] = piprw[prw.width + x - xoffset];
        }
        else
        {
          piprw[-x - 1]        = piprw[0];
          piprw[prw.width + x] = piprw[prw.width - 1];
        }
      }
      piprw -= xmargin;
      // pi is now (-marginX, 0)
      for( int y = 0; y < ymargin; y++ )
      {
        ::memcpy( piprw - ( y + 1 ) * prw.stride, piprw, sizeof( Pel ) * ( prw.width + ( xmargin << 1 ) ) );
      }
    }
  }
}

void Picture::extendPicBorderBuf( PelStorage& storage, bool top, bool bottom, bool leftrightT, bool leftrightB, ChannelType chType )
{
  for( int comp = 0; comp < getNumberValidComponents( cs->area.chromaFormat ); comp++ )
  {
    ComponentID compID = ComponentID( comp );

    if( chType != MAX_NUM_CHANNEL_TYPE && toChannelType( compID ) != chType )
      continue;

    PelBuf p = storage.bufs[compID];

    const int xmargin = margin >> getComponentScaleX( compID, cs->area.chromaFormat );
    const int ymargin = margin >> getComponentScaleY( compID, cs->area.chromaFormat );

    if( leftrightT )
    {
      Pel* pi = p.bufAt( 0, 1 );
      paddPicBorderLeftRight( pi, p.stride, p.width, xmargin, 1 + p.height / 2 );
    }
    if( leftrightB )
    {
      Pel* pi = p.bufAt( 0, p.height / 2 );
      paddPicBorderLeftRight( pi, p.stride, p.width, xmargin, 1 + p.height / 2 );
    }
    if( bottom )
    {
      // pi is now the (0,height) (bottom left of image within bigger picture
      Pel* pi = p.bufAt( 0, p.height - 1 );
      paddPicBorderBot( pi, p.stride, p.width, xmargin, ymargin );
    }
    if( top )
    {
      // pi is now the (0,height) (bottom left of image within bigger picture
      Pel* pi = p.bufAt( 0, 0 );
      paddPicBorderTop( pi, p.stride, p.width, xmargin, ymargin );
    }
  }
}

PelBuf Picture::getBuf( const CompArea &blk, const PictureType &type )
{
  if( !blk.valid() )
  {
    return PelBuf();
  }

  return m_bufs[type].getBuf( blk );
}

const CPelBuf Picture::getBuf( const CompArea &blk, const PictureType &type ) const
{
  if( !blk.valid() )
  {
    return PelBuf();
  }

  return m_bufs[type].getBuf( blk );
}

PelUnitBuf Picture::getBuf( const UnitArea &unit, const PictureType &type )
{
  if( chromaFormat == CHROMA_400 )
  {
    return PelUnitBuf( chromaFormat, getBuf( unit.Y(), type ) );
  }
  else
  {
    return PelUnitBuf( chromaFormat, getBuf( unit.Y(), type ), getBuf( unit.Cb(), type ), getBuf( unit.Cr(), type ) );
  }
}

const CPelUnitBuf Picture::getBuf( const UnitArea &unit, const PictureType &type ) const
{
  if( chromaFormat == CHROMA_400 )
  {
    return CPelUnitBuf( chromaFormat, getBuf( unit.Y(), type ) );
  }
  else
  {
    return CPelUnitBuf( chromaFormat, getBuf( unit.Y(), type ), getBuf( unit.Cb(), type ), getBuf( unit.Cr(), type ) );
  }
}

Pel* Picture::getOrigin( const PictureType &type, const ComponentID compID ) const
{
  return m_bufs[type].getOrigin( compID );
}

PelBuf Picture::getOriginBuf( const PictureType &type, const ComponentID compID )
{
  return m_bufs[type].getOriginBuf( compID );
}

Size Picture::getBufSize( const PictureType &type, const ComponentID compID ) const
{
  return m_bufs[type].getBufSize( compID );
}

void* Picture::getBufAllocator( const ComponentID compID )
{
  return m_bufs[PIC_RECONSTRUCTION].getBufAllocator( compID );
}

bool Picture::isExternAllocator() const
{
  return m_bufs[PIC_RECONSTRUCTION].isExternAllocator();
}

const UserAllocator* Picture::getUserAllocator() const
{
  return m_bufs[PIC_RECONSTRUCTION].getUserAllocator();
}

std::vector<Picture*> Picture::buildAllRefPicsVec()
{
  std::vector<Picture*> refPics;
  for( const Slice* slice : this->slices )
  {
    if( slice->isIntra() )
    {
      continue;
    }

    for( int iDir = REF_PIC_LIST_0; iDir < NUM_REF_PIC_LIST_01; ++iDir )
    {
      for( int iRefIdx = 0; iRefIdx < slice->getNumRefIdx( ( RefPicList ) iDir ); iRefIdx++ )
      {
        Picture* refPic = slice->getNoConstRefPic( ( RefPicList ) iDir, iRefIdx );
        if( std::find( refPics.cbegin(), refPics.cend(), refPic ) == refPics.cend() )
        {
          refPics.push_back( refPic );
        }
      }
    }
  }

  return refPics;
}

void Picture::startProcessingTimer()
{
  std::lock_guard<std::mutex> lock( m_timerMutex );
  m_processingStartTime = std::chrono::steady_clock::now();
}

void Picture::stopProcessingTimer()
{
  std::lock_guard<std::mutex> lock( m_timerMutex );
  auto endTime = std::chrono::steady_clock::now();
  m_dProcessingTime += std::chrono::duration<double>(endTime - m_processingStartTime).count();
}

}   // namespace vvdec
