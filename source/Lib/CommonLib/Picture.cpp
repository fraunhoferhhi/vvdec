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

/** \file     Picture.cpp
 *  \brief    Description of a coded picture
 */

#include "Picture.h"
#include "SEI.h"
#include "ChromaFormat.h"

// ---------------------------------------------------------------------------
// picture methods
// ---------------------------------------------------------------------------


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


void Picture::create(const ChromaFormat &_chromaFormat, const Size &size, const unsigned _maxCUSize, const unsigned _margin, const int _layerId)
{
  layerId = _layerId;
  UnitArea::operator=( UnitArea( _chromaFormat, Area( Position{ 0, 0 }, size ) ) );
  margin            = _margin;
  m_bufs[PIC_RECONSTRUCTION].create( _chromaFormat, size, _maxCUSize, _margin, MEMORY_ALIGN_DEF_SIZE );
  m_bufs[PIC_RECON_WRAP    ].create( _chromaFormat, size, _maxCUSize, _margin, MEMORY_ALIGN_DEF_SIZE );
  m_bufs[PIC_PREDICTION    ].create( _chromaFormat, size );

  neededForOutput = true;
}

void Picture::resetForUse()
{
  CHECK( lockedByApplication, "the picture can not be re-used, because it has not been unlocked by the application." );

  m_subPicBufs.clear();

  isBorderExtended = false;
#if JVET_Q0764_WRAP_AROUND_WITH_RPR
  wrapAroundValid  = false;
  wrapAroundOffset = 0;
#endif
  neededForOutput  = true;
  reconstructed    = false;
  inProgress       = false;
  wasLost          = false;
  skippedDecCount  = 0;
  done.lock();
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
  if( ctuParsedBarrier )
  {
    delete[] ctuParsedBarrier;
    ctuParsedBarrier = nullptr;
  }

#endif
  for( auto &ps : slices )
  {
    delete ps;
  }
  slices.clear();

  for( auto &psei : SEIs )
  {
    delete psei;
  }
  SEIs.clear();

  if (m_spliceIdx)
  {
    delete[] m_spliceIdx;
    m_spliceIdx = NULL;
  }
}

       PelBuf     Picture::getRecoBuf(const ComponentID compID, bool wrap)       { return getBuf(compID, wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION); }
const CPelBuf     Picture::getRecoBuf(const ComponentID compID, bool wrap) const { return getBuf(compID, wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION); }
       PelBuf     Picture::getRecoBuf(const CompArea &blk, bool wrap)            { return getBuf(blk,    wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION); }
const CPelBuf     Picture::getRecoBuf(const CompArea &blk, bool wrap)      const { return getBuf(blk,    wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION); }
       PelUnitBuf Picture::getRecoBuf(const UnitArea &unit, bool wrap)           { return getBuf(unit,   wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION); }
const CPelUnitBuf Picture::getRecoBuf(const UnitArea &unit, bool wrap)     const { return getBuf(unit,   wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION); }
       PelUnitBuf Picture::getRecoBuf( bool wrap )                               { return wrap ? m_bufs[PIC_RECON_WRAP] : m_bufs[PIC_RECONSTRUCTION]; }
const CPelUnitBuf Picture::getRecoBuf( bool wrap )                         const { return wrap ? m_bufs[PIC_RECON_WRAP] : m_bufs[PIC_RECONSTRUCTION]; }

void Picture::finalInit( const SPS *sps, const PPS *pps, PicHeader* picHeader, APS* alfApss[ALF_CTB_MAX_NUM_APS], APS* lmcsAps, APS* scalingListAps )
{
  for( auto &sei : SEIs )
  {
    delete sei;
  }

  SEIs.clear();
  clearSliceBuffer();

  const ChromaFormat chromaFormatIDC = sps->getChromaFormatIdc();
  const int          iWidth          = pps->getPicWidthInLumaSamples();
  const int          iHeight         = pps->getPicHeightInLumaSamples();

  if( !cs )
  {
    cs = new CodingStructure( g_globalUnitCache.getCuCache(), g_globalUnitCache.getTuCache() );
    cs->create( chromaFormatIDC, Area( 0, 0, iWidth, iHeight ) );
  }

#if RECO_WHILE_PARSE
  if( !ctuParsedBarrier )
  {
    ctuParsedBarrier = new Barrier[pps->pcv->sizeInCtus];
  }

#endif
  parseDone   . lock();
  cs->picture = this;
  cs->pps     = pps ? pps->getSharedPtr() : nullptr;
  cs->sps     = sps ? sps->getSharedPtr() : nullptr;

  picHeader->setSPSId         ( sps->getSPSId() );
  picHeader->setPPSId         ( pps->getPPSId() );
  picHeader->setLmcsAPS       ( lmcsAps        ? lmcsAps       ->getSharedPtr() : nullptr );
  picHeader->setScalingListAPS( scalingListAps ? scalingListAps->getSharedPtr() : nullptr );

  for( int i = 0; i < ALF_CTB_MAX_NUM_APS; ++i )
  {
    cs->alfApss[i] = alfApss[i] ? alfApss[i]->getSharedPtr() : nullptr;
  }
  if( lmcsAps )
  {
    cs->lmcsAps = lmcsAps ? lmcsAps->getSharedPtr() : nullptr;
  }

  cs->pcv     = pps->pcv.get();

  cs->rebindPicBufs();

  if( m_spliceIdx == NULL )
  {
    m_ctuNums   = cs->pcv->sizeInCtus;
    m_spliceIdx = new int[m_ctuNums];
    memset( m_spliceIdx, 0, m_ctuNums * sizeof( int ) );
  }

  paddPicBorderBot       = paddPicBorderBotCore;
  paddPicBorderTop       = paddPicBorderTopCore;
  paddPicBorderLeftRight = paddPicBorderLeftRightCore;

#if ENABLE_SIMD_OPT_PICTURE
  initPictureX86();
#endif
}

void Picture::allocateNewSlice()
{
  slices.push_back(new Slice);
  Slice& slice = *slices.back();
  for( int i=0; i<ALF_CTB_MAX_NUM_APS; ++i )
  {
    slice.getAlfAPSs()[i] = cs->alfApss[i].get();
  }


  slice.setPPS( cs->pps.get() );
  slice.setSPS( cs->sps.get() );
  slice.setVPS( cs->vps.get() );
  if(slices.size()>=2)
  {
    slice.copySliceInfo( slices[slices.size()-2] );
    slice.initSlice();
  }
}

Slice* Picture::swapSliceObject( Slice* s, uint32_t i )
{
  s->setSPS( cs->sps.get() );
  s->setPPS( cs->pps.get() );
  s->setVPS( cs->vps.get() );
  s->setAlfAPSs( cs->alfApss );

  Slice * pTmp = slices[i];
  slices[i] = s;
  pTmp->setSPS( nullptr );
  pTmp->setPPS( nullptr );
  pTmp->setVPS( nullptr );
  memset( pTmp->getAlfAPSs(), 0, sizeof( pTmp->getAlfAPSs()[0] ) * ALF_CTB_MAX_NUM_APS );

  return pTmp;
}

void Picture::setPicHead( const std::shared_ptr<PicHeader>& ph )
{
  cs->picHeader   = ph.get();
  this->picHeader = ph;
}

void Picture::clearSliceBuffer()
{
  for( size_t i = 0; i < slices.size(); i++ )
  {
    delete slices[i];
  }

  slices.clear();
}

const TFilterCoeff DownsamplingFilterSRC[8][16][12] =
{
    { // D = 1
      {   0,   0,   0,   0,   0, 128,   0,   0,   0,   0,   0,   0 },
      {   0,   0,   0,   2,  -6, 127,   7,  -2,   0,   0,   0,   0 },
      {   0,   0,   0,   3, -12, 125,  16,  -5,   1,   0,   0,   0 },
      {   0,   0,   0,   4, -16, 120,  26,  -7,   1,   0,   0,   0 },
      {   0,   0,   0,   5, -18, 114,  36, -10,   1,   0,   0,   0 },
      {   0,   0,   0,   5, -20, 107,  46, -12,   2,   0,   0,   0 },
      {   0,   0,   0,   5, -21,  99,  57, -15,   3,   0,   0,   0 },
      {   0,   0,   0,   5, -20,  89,  68, -18,   4,   0,   0,   0 },
      {   0,   0,   0,   4, -19,  79,  79, -19,   4,   0,   0,   0 },
      {   0,   0,   0,   4, -18,  68,  89, -20,   5,   0,   0,   0 },
      {   0,   0,   0,   3, -15,  57,  99, -21,   5,   0,   0,   0 },
      {   0,   0,   0,   2, -12,  46, 107, -20,   5,   0,   0,   0 },
      {   0,   0,   0,   1, -10,  36, 114, -18,   5,   0,   0,   0 },
      {   0,   0,   0,   1,  -7,  26, 120, -16,   4,   0,   0,   0 },
      {   0,   0,   0,   1,  -5,  16, 125, -12,   3,   0,   0,   0 },
      {   0,   0,   0,   0,  -2,   7, 127,  -6,   2,   0,   0,   0 }
    },
    { // D = 1.5
      {   0,   2,   0, -14,  33,  86,  33, -14,   0,   2,   0,   0 },
      {   0,   1,   1, -14,  29,  85,  38, -13,  -1,   2,   0,   0 },
      {   0,   1,   2, -14,  24,  84,  43, -12,  -2,   2,   0,   0 },
      {   0,   1,   2, -13,  19,  83,  48, -11,  -3,   2,   0,   0 },
      {   0,   0,   3, -13,  15,  81,  53, -10,  -4,   3,   0,   0 },
      {   0,   0,   3, -12,  11,  79,  57,  -8,  -5,   3,   0,   0 },
      {   0,   0,   3, -11,   7,  76,  62,  -5,  -7,   3,   0,   0 },
      {   0,   0,   3, -10,   3,  73,  65,  -2,  -7,   3,   0,   0 },
      {   0,   0,   3,  -9,   0,  70,  70,   0,  -9,   3,   0,   0 },
      {   0,   0,   3,  -7,  -2,  65,  73,   3, -10,   3,   0,   0 },
      {   0,   0,   3,  -7,  -5,  62,  76,   7, -11,   3,   0,   0 },
      {   0,   0,   3,  -5,  -8,  57,  79,  11, -12,   3,   0,   0 },
      {   0,   0,   3,  -4, -10,  53,  81,  15, -13,   3,   0,   0 },
      {   0,   0,   2,  -3, -11,  48,  83,  19, -13,   2,   1,   0 },
      {   0,   0,   2,  -2, -12,  43,  84,  24, -14,   2,   1,   0 },
      {   0,   0,   2,  -1, -13,  38,  85,  29, -14,   1,   1,   0 }
    },
    { // D = 2
      {   0,   5,   -6,  -10,  37,  76,   37,  -10,  -6,    5,  0,   0}, //0
      {   0,   5,   -4,  -11,  33,  76,   40,  -9,    -7,    5,  0,   0}, //1
      //{   0,   5,   -3,  -12,  28,  75,   44,  -7,    -8,    5,  1,   0}, //2
      {  -1,   5,   -3,  -12,  29,  75,   45,  -7,    -8,   5,  0,   0}, //2 new coefficients in m24499
      {  -1,   4,   -2,  -13,  25,  75,   48,  -5,    -9,    5,  1,   0}, //3
      {  -1,   4,   -1,  -13,  22,  73,   52,  -3,    -10,  4,  1,   0}, //4
      {  -1,   4,   0,    -13,  18,  72,   55,  -1,    -11,  4,  2,  -1}, //5
      {  -1,   4,   1,    -13,  14,  70,   59,  2,    -12,  3,  2,  -1}, //6
      {  -1,   3,   1,    -13,  11,  68,   62,  5,    -12,  3,  2,  -1}, //7
      {  -1,   3,   2,    -13,  8,  65,   65,  8,    -13,  2,  3,  -1}, //8
      {  -1,   2,   3,    -12,  5,  62,   68,  11,    -13,  1,  3,  -1}, //9
      {  -1,   2,   3,    -12,  2,  59,   70,  14,    -13,  1,  4,  -1}, //10
      {  -1,   2,   4,    -11,  -1,  55,   72,  18,    -13,  0,  4,  -1}, //11
      {   0,   1,   4,    -10,  -3,  52,   73,  22,    -13,  -1,  4,  -1}, //12
      {   0,   1,   5,    -9,    -5,  48,   75,  25,    -13,  -2,  4,  -1}, //13
      //{   0,   1,   5,    -8,    -7,  44,   75,  28,    -12,  -3,  5,   0}, //14
      {    0,   0,   5,    -8,   -7,  45,   75,  29,    -12,  -3,  5,  -1}  , //14 new coefficients in m24499
      {   0,   0,   5,    -7,    -9,  40,   76,  33,    -11,  -4,  5,   0}, //15
    },
    { // D = 2.5
      {   2,  -3,   -9,  6,   39,  58,   39,  6,   -9,  -3,    2,    0}, // 0
      {   2,  -3,   -9,  4,   38,  58,   43,  7,   -9,  -4,    1,    0}, // 1
      {   2,  -2,   -9,  2,   35,  58,   44,  9,   -8,  -4,    1,    0}, // 2
      {   1,  -2,   -9,  1,   34,  58,   46,  11,   -8,  -5,    1,    0}, // 3
      //{   1,  -1,   -8,  -1,   31,  57,   48,  13,   -8,  -5,    1,    0}, // 4
      {   1,  -1,   -8,  -1,   31,  57,   47,  13,   -7,  -5,    1,    0},  // 4 new coefficients in m24499
      {   1,  -1,   -8,  -2,   29,  56,   49,  15,   -7,  -6,    1,    1}, // 5
      {   1,  0,   -8,  -3,   26,  55,   51,  17,   -7,  -6,    1,    1}, // 6
      {   1,  0,   -7,  -4,   24,  54,   52,  19,   -6,  -7,    1,    1}, // 7
      {   1,  0,   -7,  -5,   22,  53,   53,  22,   -5,  -7,    0,    1}, // 8
      {   1,  1,   -7,  -6,   19,  52,   54,  24,   -4,  -7,    0,    1}, // 9
      {   1,  1,   -6,  -7,   17,  51,   55,  26,   -3,  -8,    0,    1}, // 10
      {   1,  1,   -6,  -7,   15,  49,   56,  29,   -2,  -8,    -1,    1}, // 11
      //{   0,  1,   -5,  -8,   13,  48,   57,  31,   -1,  -8,    -1,    1}, // 12 new coefficients in m24499
      {   0,  1,   -5,  -7,   13,  47,  57,  31,  -1,    -8,   -1,    1}, // 12
      {   0,  1,   -5,  -8,   11,  46,   58,  34,   1,    -9,    -2,    1}, // 13
      {   0,  1,   -4,  -8,   9,    44,   58,  35,   2,    -9,    -2,    2}, // 14
      {   0,  1,   -4,  -9,   7,    43,   58,  38,   4,    -9,    -3,    2}, // 15
    },
    { // D = 3
      {  -2,  -7,   0,  17,  35,  43,  35,  17,   0,  -7,  -5,   2 },
      {  -2,  -7,  -1,  16,  34,  43,  36,  18,   1,  -7,  -5,   2 },
      {  -1,  -7,  -1,  14,  33,  43,  36,  19,   1,  -6,  -5,   2 },
      {  -1,  -7,  -2,  13,  32,  42,  37,  20,   3,  -6,  -5,   2 },
      {   0,  -7,  -3,  12,  31,  42,  38,  21,   3,  -6,  -5,   2 },
      {   0,  -7,  -3,  11,  30,  42,  39,  23,   4,  -6,  -6,   1 },
      {   0,  -7,  -4,  10,  29,  42,  40,  24,   5,  -6,  -6,   1 },
      {   1,  -7,  -4,   9,  27,  41,  40,  25,   6,  -5,  -6,   1 },
      {   1,  -6,  -5,   7,  26,  41,  41,  26,   7,  -5,  -6,   1 },
      {   1,  -6,  -5,   6,  25,  40,  41,  27,   9,  -4,  -7,   1 },
      {   1,  -6,  -6,   5,  24,  40,  42,  29,  10,  -4,  -7,   0 },
      {   1,  -6,  -6,   4,  23,  39,  42,  30,  11,  -3,  -7,   0 },
      {   2,  -5,  -6,   3,  21,  38,  42,  31,  12,  -3,  -7,   0 },
      {   2,  -5,  -6,   3,  20,  37,  42,  32,  13,  -2,  -7,  -1 },
      {   2,  -5,  -6,   1,  19,  36,  43,  33,  14,  -1,  -7,  -1 },
      {   2,  -5,  -7,   1,  18,  36,  43,  34,  16,  -1,  -7,  -2 }
    },
    { // D = 3.5
      {  -6,  -3,   5,  19,  31,  36,  31,  19,   5,  -3,  -6,   0 },
      {  -6,  -4,   4,  18,  31,  37,  32,  20,   6,  -3,  -6,  -1 },
      {  -6,  -4,   4,  17,  30,  36,  33,  21,   7,  -3,  -6,  -1 },
      {  -5,  -5,   3,  16,  30,  36,  33,  22,   8,  -2,  -6,  -2 },
      {  -5,  -5,   2,  15,  29,  36,  34,  23,   9,  -2,  -6,  -2 },
      {  -5,  -5,   2,  15,  28,  36,  34,  24,  10,  -2,  -6,  -3 },
      {  -4,  -5,   1,  14,  27,  36,  35,  24,  10,  -1,  -6,  -3 },
      {  -4,  -5,   0,  13,  26,  35,  35,  25,  11,   0,  -5,  -3 },
      {  -4,  -6,   0,  12,  26,  36,  36,  26,  12,   0,  -6,  -4 },
      {  -3,  -5,   0,  11,  25,  35,  35,  26,  13,   0,  -5,  -4 },
      {  -3,  -6,  -1,  10,  24,  35,  36,  27,  14,   1,  -5,  -4 },
      {  -3,  -6,  -2,  10,  24,  34,  36,  28,  15,   2,  -5,  -5 },
      {  -2,  -6,  -2,   9,  23,  34,  36,  29,  15,   2,  -5,  -5 },
      {  -2,  -6,  -2,   8,  22,  33,  36,  30,  16,   3,  -5,  -5 },
      {  -1,  -6,  -3,   7,  21,  33,  36,  30,  17,   4,  -4,  -6 },
      {  -1,  -6,  -3,   6,  20,  32,  37,  31,  18,   4,  -4,  -6 }
    },
    { // D = 4
      {  -9,   0,   9,  20,  28,  32,  28,  20,   9,   0,  -9,   0 },
      {  -9,   0,   8,  19,  28,  32,  29,  20,  10,   0,  -4,  -5 },
      {  -9,  -1,   8,  18,  28,  32,  29,  21,  10,   1,  -4,  -5 },
      {  -9,  -1,   7,  18,  27,  32,  30,  22,  11,   1,  -4,  -6 },
      {  -8,  -2,   6,  17,  27,  32,  30,  22,  12,   2,  -4,  -6 },
      {  -8,  -2,   6,  16,  26,  32,  31,  23,  12,   2,  -4,  -6 },
      {  -8,  -2,   5,  16,  26,  31,  31,  23,  13,   3,  -3,  -7 },
      {  -8,  -3,   5,  15,  25,  31,  31,  24,  14,   4,  -3,  -7 },
      {  -7,  -3,   4,  14,  25,  31,  31,  25,  14,   4,  -3,  -7 },
      {  -7,  -3,   4,  14,  24,  31,  31,  25,  15,   5,  -3,  -8 },
      {  -7,  -3,   3,  13,  23,  31,  31,  26,  16,   5,  -2,  -8 },
      {  -6,  -4,   2,  12,  23,  31,  32,  26,  16,   6,  -2,  -8 },
      {  -6,  -4,   2,  12,  22,  30,  32,  27,  17,   6,  -2,  -8 },
      {  -6,  -4,   1,  11,  22,  30,  32,  27,  18,   7,  -1,  -9 },
      {  -5,  -4,   1,  10,  21,  29,  32,  28,  18,   8,  -1,  -9 },
      {  -5,  -4,   0,  10,  20,  29,  32,  28,  19,   8,   0,  -9 }
    },
    { // D = 5.5
      {  -8,   7,  13,  18,  22,  24,  22,  18,  13,   7,   2, -10 },
      {  -8,   7,  13,  18,  22,  23,  22,  19,  13,   7,   2, -10 },
      {  -8,   6,  12,  18,  22,  23,  22,  19,  14,   8,   2, -10 },
      {  -9,   6,  12,  17,  22,  23,  23,  19,  14,   8,   3, -10 },
      {  -9,   6,  12,  17,  21,  23,  23,  19,  14,   9,   3, -10 },
      {  -9,   5,  11,  17,  21,  23,  23,  20,  15,   9,   3, -10 },
      {  -9,   5,  11,  16,  21,  23,  23,  20,  15,   9,   4, -10 },
      {  -9,   5,  10,  16,  21,  23,  23,  20,  15,  10,   4, -10 },
      { -10,   5,  10,  16,  20,  23,  23,  20,  16,  10,   5, -10 },
      { -10,   4,  10,  15,  20,  23,  23,  21,  16,  10,   5,  -9 },
      { -10,   4,   9,  15,  20,  23,  23,  21,  16,  11,   5,  -9 },
      { -10,   3,   9,  15,  20,  23,  23,  21,  17,  11,   5,  -9 },
      { -10,   3,   9,  14,  19,  23,  23,  21,  17,  12,   6,  -9 },
      { -10,   3,   8,  14,  19,  23,  23,  22,  17,  12,   6,  -9 },
      { -10,   2,   8,  14,  19,  22,  23,  22,  18,  12,   6,  -8 },
      { -10,   2,   7,  13,  19,  22,  23,  22,  18,  13,   7,  -8 }
    }
};

#if RPR_FIX
void Picture::sampleRateConv( const std::pair<int, int> scalingRatio, const std::pair<int, int> compScale,
                             const CPelBuf& beforeScale, const int beforeScaleLeftOffset, const int beforeScaleTopOffset,
                             const PelBuf& afterScale, const int afterScaleLeftOffset, const int afterScaleTopOffset,
                             const int bitDepth, const bool useLumaFilter, const bool downsampling,
                             const bool horCollocatedPositionFlag, const bool verCollocatedPositionFlag )
{
  const Pel* orgSrc = beforeScale.buf;
  const int orgWidth = beforeScale.width;
  const int orgHeight = beforeScale.height;
  const int orgStride = beforeScale.stride;
  
  Pel* scaledSrc = afterScale.buf;
  const int scaledWidth = afterScale.width;
  const int scaledHeight = afterScale.height;
  const int scaledStride = afterScale.stride;
  
  if( orgWidth == scaledWidth && orgHeight == scaledHeight && scalingRatio == SCALE_1X && !beforeScaleLeftOffset && !beforeScaleTopOffset && !afterScaleLeftOffset && !afterScaleTopOffset )
  {
    for( int j = 0; j < orgHeight; j++ )
    {
      memcpy( scaledSrc + j * scaledStride, orgSrc + j * orgStride, sizeof( Pel ) * orgWidth );
    }
    
    return;
  }
  
  const TFilterCoeff* filterHor = useLumaFilter ? &InterpolationFilter::m_lumaFilter[0][0] : &InterpolationFilter::m_chromaFilter[0][0];
  const TFilterCoeff* filterVer = useLumaFilter ? &InterpolationFilter::m_lumaFilter[0][0] : &InterpolationFilter::m_chromaFilter[0][0];
  const int numFracPositions = useLumaFilter ? 15 : 31;
  const int numFracShift = useLumaFilter ? 4 : 5;
  const int posShiftX = SCALE_RATIO_BITS - numFracShift + compScale.first;
  const int posShiftY = SCALE_RATIO_BITS - numFracShift + compScale.second;
  int addX = ( 1 << ( posShiftX - 1 ) ) + ( beforeScaleLeftOffset << SCALE_RATIO_BITS ) + ( ( int( 1 - horCollocatedPositionFlag ) * 8 * ( scalingRatio.first - SCALE_1X.first ) + ( 1 << ( 2 + compScale.first ) ) ) >> ( 3 + compScale.first ) );
  int addY = ( 1 << ( posShiftY - 1 ) ) + ( beforeScaleTopOffset << SCALE_RATIO_BITS ) + ( ( int( 1 - verCollocatedPositionFlag ) * 8 * ( scalingRatio.second - SCALE_1X.second ) + ( 1 << ( 2 + compScale.second ) ) ) >> ( 3 + compScale.second ) );
  
  if( downsampling )
  {
    int verFilter = 0;
    int horFilter = 0;
    
    if (scalingRatio.first > (15 << SCALE_RATIO_BITS) / 4)
    {
      horFilter = 7;
    }
    else if (scalingRatio.first > (20 << SCALE_RATIO_BITS) / 7)
    {
      horFilter = 6;
    }
    else if (scalingRatio.first > (5 << SCALE_RATIO_BITS) / 2)
    {
      horFilter = 5;
    }
    else if (scalingRatio.first > (2 << SCALE_RATIO_BITS))
    {
      horFilter = 4;
    }
    else if (scalingRatio.first > (5 << SCALE_RATIO_BITS) / 3)
    {
      horFilter = 3;
    }
    else if (scalingRatio.first > (5 << SCALE_RATIO_BITS) / 4)
    {
      horFilter = 2;
    }
    else if (scalingRatio.first > (20 << SCALE_RATIO_BITS) / 19)
    {
      horFilter = 1;
    }
    
    if (scalingRatio.second > (15 << SCALE_RATIO_BITS) / 4)
    {
      verFilter = 7;
    }
    else if (scalingRatio.second > (20 << SCALE_RATIO_BITS) / 7)
    {
      verFilter = 6;
    }
    else if (scalingRatio.second > (5 << SCALE_RATIO_BITS) / 2)
    {
      verFilter = 5;
    }
    else if (scalingRatio.second > (2 << SCALE_RATIO_BITS))
    {
      verFilter = 4;
    }
    else if (scalingRatio.second > (5 << SCALE_RATIO_BITS) / 3)
    {
      verFilter = 3;
    }
    else if (scalingRatio.second > (5 << SCALE_RATIO_BITS) / 4)
    {
      verFilter = 2;
    }
    else if (scalingRatio.second > (20 << SCALE_RATIO_BITS) / 19)
    {
      verFilter = 1;
    }
    
    filterHor = &DownsamplingFilterSRC[horFilter][0][0];
    filterVer = &DownsamplingFilterSRC[verFilter][0][0];
  }
  
  const int filterLength = downsampling ? 12 : ( useLumaFilter ? NTAPS_LUMA : NTAPS_CHROMA );
  const int log2Norm = downsampling ? 14 : 12;
  
  int *buf = new int[orgHeight * scaledWidth];
  int maxVal = ( 1 << bitDepth ) - 1;
  
  CHECK( bitDepth > 17, "Overflow may happen!" );
  
  for( int i = 0; i < scaledWidth; i++ )
  {
    const Pel* org = orgSrc;
    int refPos = ( ( ( i << compScale.first ) - afterScaleLeftOffset ) * scalingRatio.first + addX ) >> posShiftX;
    int integer = refPos >> numFracShift;
    int frac = refPos & numFracPositions;
    int* tmp = buf + i;
    
    for( int j = 0; j < orgHeight; j++ )
    {
      int sum = 0;
      const TFilterCoeff* f = filterHor + frac * filterLength;
      
      for( int k = 0; k < filterLength; k++ )
      {
        int xInt = std::min<int>( std::max( 0, integer + k - filterLength / 2 + 1 ), orgWidth - 1 );
        sum += f[k] * org[xInt]; // postpone horizontal filtering gain removal after vertical filtering
      }
      
      *tmp = sum;
      
      tmp += scaledWidth;
      org += orgStride;
    }
  }
  
  Pel* dst = scaledSrc;
  
  for( int j = 0; j < scaledHeight; j++ )
  {
    int refPos = ( ( ( j << compScale.second ) - afterScaleTopOffset ) * scalingRatio.second + addY ) >> posShiftY;
    int integer = refPos >> numFracShift;
    int frac = refPos & numFracPositions;
    
    for( int i = 0; i < scaledWidth; i++ )
    {
      int sum = 0;
      int* tmp = buf + i;
      const TFilterCoeff* f = filterVer + frac * filterLength;
      
      for( int k = 0; k < filterLength; k++ )
      {
        int yInt = std::min<int>( std::max( 0, integer + k - filterLength / 2 + 1 ), orgHeight - 1 );
        sum += f[k] * tmp[yInt*scaledWidth];
      }
      
      dst[i] = std::min<int>( std::max( 0, ( sum + ( 1 << ( log2Norm - 1 ) ) ) >> log2Norm ), maxVal );
    }
    
    dst += scaledStride;
  }
  
  delete[] buf;
}
#else
void Picture::sampleRateConv( const Pel* orgSrc, SizeType orgWidth, SizeType orgHeight, ptrdiff_t orgStride, Pel* scaledSrc, SizeType scaledWidth, SizeType scaledHeight, SizeType paddedWidth, SizeType paddedHeight, ptrdiff_t scaledStride, const int bitDepth, const bool useLumaFilter, const bool downsampling )
{
  if( orgWidth == scaledWidth && orgHeight == scaledHeight )
  {
    for( int j = 0; j < orgHeight; j++ )
    {
      memcpy( scaledSrc + j * scaledStride, orgSrc + j * orgStride, sizeof( Pel ) * orgWidth );
    }

    return;
  }

  const TFilterCoeff* filterHor = useLumaFilter ? &InterpolationFilter::m_lumaFilter[0][0] : &InterpolationFilter::m_chromaFilter[0][0];
  const TFilterCoeff* filterVer = useLumaFilter ? &InterpolationFilter::m_lumaFilter[0][0] : &InterpolationFilter::m_chromaFilter[0][0];
  const int numFracPositions = useLumaFilter ? 15 : 31;

  if( downsampling )
  {
    int verFilter = 0;
    int horFilter = 0;
    if( 4 * orgHeight > 15 * scaledHeight )   verFilter = 7;
    else if( 7 * orgHeight > 20 * scaledHeight )   verFilter = 6;
    else if( 2 * orgHeight > 5 * scaledHeight )   verFilter = 5;
    else if( 1 * orgHeight > 2 * scaledHeight )   verFilter = 4;
    else if( 3 * orgHeight > 5 * scaledHeight )   verFilter = 3;
    else if( 4 * orgHeight > 5 * scaledHeight )   verFilter = 2;
    else if( 19 * orgHeight > 20 * scaledHeight )   verFilter = 1;

    if( 4 * orgWidth > 15 * scaledWidth )   horFilter = 7;
    else if( 7 * orgWidth > 20 * scaledWidth )   horFilter = 6;
    else if( 2 * orgWidth > 5 * scaledWidth )   horFilter = 5;
    else if( 1 * orgWidth > 2 * scaledWidth )   horFilter = 4;
    else if( 3 * orgWidth > 5 * scaledWidth )   horFilter = 3;
    else if( 4 * orgWidth > 5 * scaledWidth )   horFilter = 2;
    else if( 19 * orgWidth > 20 * scaledWidth )   horFilter = 1;

    filterHor = &DownsamplingFilterSRC[horFilter][0][0];
    filterVer = &DownsamplingFilterSRC[verFilter][0][0];
  }

  const int filerLength = downsampling ? 12 : ( useLumaFilter ? NTAPS_LUMA : NTAPS_CHROMA );
  const int log2Norm = downsampling ? 14 : 12;

  int *buf = new int[orgHeight * paddedWidth];
  int maxVal = ( 1 << bitDepth ) - 1;

  CHECK( bitDepth > 17, "Overflow may happen!" );

  for( int i = 0; i < paddedWidth; i++ )
  {
    const Pel* org = orgSrc;
    int integer = ( i * orgWidth ) / scaledWidth;
    int frac = ( ( i * orgWidth << 4 ) / scaledWidth ) & numFracPositions;

    int* tmp = buf + i;

    for( int j = 0; j < orgHeight; j++ )
    {
      int sum = 0;
      const TFilterCoeff* f = filterHor + frac * filerLength;

      for( int k = 0; k < filerLength; k++ )
      {
        int xInt = std::min<int>( std::max( 0, integer + k - filerLength / 2 + 1 ), orgWidth - 1 );
        sum += f[k] * org[xInt]; // postpone horizontal filtering gain removal after vertical filtering
      }

      *tmp = sum;

      tmp += paddedWidth;
      org += orgStride;
    }
  }

  Pel* dst = scaledSrc;

  for( int j = 0; j < paddedHeight; j++ )
  {
    int integer = ( j * orgHeight ) / scaledHeight;
    int frac = ( ( j * orgHeight << 4 ) / scaledHeight ) & numFracPositions;

    for( int i = 0; i < paddedWidth; i++ )
    {
      int sum = 0;
      int* tmp = buf + i;
      const TFilterCoeff* f = filterVer + frac * filerLength;

      for( int k = 0; k < filerLength; k++ )
      {
        int yInt = std::min<int>( std::max( 0, integer + k - filerLength / 2 + 1 ), orgHeight - 1 );
        sum += f[k] * tmp[yInt*paddedWidth];
      }

      dst[i] = std::min<int>( std::max( 0, ( sum + ( 1 << ( log2Norm - 1 ) ) ) >> log2Norm ), maxVal );
    }

    dst += scaledStride;
  }

  delete[] buf;
}
#endif

#if RPR_FIX
void Picture::rescalePicture( const std::pair<int, int> scalingRatio, const CPelUnitBuf& beforeScaling, const Window& confBefore, const PelUnitBuf& afterScaling, const Window& confAfter, const ChromaFormat chromaFormatIDC, const BitDepths& bitDepths, const bool useLumaFilter, const bool downsampling, const bool horCollocatedChromaFlag, const bool verCollocatedChromaFlag )
#else
void Picture::rescalePicture( const CPelUnitBuf& beforeScaling, const Window& confBefore, const PelUnitBuf& afterScaling, const Window& confAfter, const ChromaFormat chromaFormatIDC, const BitDepths& bitDepths, const bool useLumaFilter, const bool downsampling )
#endif
{
  for( int comp = 0; comp < ::getNumberValidComponents( chromaFormatIDC ); comp++ )
  {
    ComponentID compID = ComponentID( comp );
    const CPelBuf& beforeScale = beforeScaling.get( compID );
    const PelBuf& afterScale = afterScaling.get( compID );

#if RPR_FIX
    sampleRateConv( scalingRatio, std::pair<int, int>( ::getComponentScaleX( compID, chromaFormatIDC ), ::getComponentScaleY( compID, chromaFormatIDC ) ),
                    beforeScale, confBefore.getWindowLeftOffset() * SPS::getWinUnitX( chromaFormatIDC ), confBefore.getWindowTopOffset() * SPS::getWinUnitY( chromaFormatIDC ),
                    afterScale, confAfter.getWindowLeftOffset() * SPS::getWinUnitX( chromaFormatIDC ), confAfter.getWindowTopOffset() * SPS::getWinUnitY( chromaFormatIDC ),
                    bitDepths.recon[toChannelType(compID)], downsampling || useLumaFilter ? true : isLuma( compID ), downsampling,
                    isLuma( compID ) ? 1 : horCollocatedChromaFlag, isLuma( compID ) ? 1 : verCollocatedChromaFlag );
#else
    int widthBefore = beforeScale.width - (((confBefore.getWindowLeftOffset() + confBefore.getWindowRightOffset()) * SPS::getWinUnitX(chromaFormatIDC)) >> getChannelTypeScaleX((ChannelType)(comp > 0), chromaFormatIDC));
    int heightBefore = beforeScale.height - (((confBefore.getWindowTopOffset() + confBefore.getWindowBottomOffset()) * SPS::getWinUnitY(chromaFormatIDC)) >> getChannelTypeScaleY((ChannelType)(comp > 0), chromaFormatIDC));
    int widthAfter = afterScale.width - (((confAfter.getWindowLeftOffset() + confAfter.getWindowRightOffset()) * SPS::getWinUnitX(chromaFormatIDC)) >> getChannelTypeScaleX((ChannelType)(comp > 0), chromaFormatIDC));
    int heightAfter = afterScale.height - (((confAfter.getWindowTopOffset() + confAfter.getWindowBottomOffset()) * SPS::getWinUnitY(chromaFormatIDC)) >> getChannelTypeScaleY((ChannelType)(comp > 0), chromaFormatIDC));

    Picture::sampleRateConv( beforeScale.buf, widthBefore, heightBefore, beforeScale.stride, afterScale.buf, widthAfter, heightAfter, afterScale.width, afterScale.height, afterScale.stride, bitDepths.recon[toChannelType(compID)], downsampling || useLumaFilter ? true : isLuma(compID), downsampling );
#endif
  }
}

#if JVET_O1143_MV_ACROSS_SUBPIC_BOUNDARY
void Picture::extendSubPicBorder( const PelStorage& subPicBuf, Area subPicArea )
{
  for (int comp = 0; comp < getNumberValidComponents(cs->area.chromaFormat); comp++)
  {
    ComponentID compID = ComponentID(comp);

    // 2.1 measure the margin for each component
    int xmargin = margin >> getComponentScaleX(compID, cs->area.chromaFormat);
    int ymargin = margin >> getComponentScaleY(compID, cs->area.chromaFormat);

    // 2.2 calculate the origin of the Subpicture
    int left = subPicArea.x >> getComponentScaleX(compID, cs->area.chromaFormat);
    int top  = subPicArea.y >> getComponentScaleY( compID, cs->area.chromaFormat );

    // 2.3 calculate the width/height of the Subpicture
    int width  = subPicArea.width >> getComponentScaleX( compID, cs->area.chromaFormat );
    int height = subPicArea.height >> getComponentScaleY( compID, cs->area.chromaFormat );

    // 3.1 set reconstructed picture
    PelBuf s = subPicBuf.get(compID);
    Pel *src = s.bufAt(left, top);

    // 4.1 apply padding for left and right
    {
      Pel *dstLeft  = src - xmargin;
      Pel *dstRight = src + width;
      Pel *srcLeft  = src + 0;
      Pel *srcRight = src + width - 1;

      for (int y = 0; y < height; y++)
      {
        for (int x = 0; x < xmargin; x++)
        {
          dstLeft[x]  = *srcLeft;
          dstRight[x] = *srcRight;
        }
        dstLeft  += s.stride;
        dstRight += s.stride;
        srcLeft  += s.stride;
        srcRight += s.stride;
      }
    }

    // 4.2 apply padding on bottom
    Pel *srcBottom = src + s.stride * (height - 1) - xmargin;
    Pel *dstBottom = srcBottom + s.stride;
    for (int y = 0; y < ymargin; y++)
    {
      ::memcpy(dstBottom, srcBottom, sizeof(Pel)*(2 * xmargin + width));
      dstBottom += s.stride;
    }

    // 4.3 apply padding for top
    // si is still (-marginX, SubpictureHeight-1)
    Pel *srcTop = src - xmargin;
    Pel *dstTop = srcTop - s.stride;
    // si is now (-marginX, 0)
    for (int y = 0; y < ymargin; y++)
    {
      ::memcpy(dstTop, srcTop, sizeof(Pel)*(2 * xmargin + width));
      dstTop -= s.stride;
    }

    CHECK( cs->sps->getUseWrapAround(), "Wraparound + subpics not implemented" );
//    // Appy padding for recon wrap buffer
//    if (cs->sps->getUseWrapAround())
//    {
//      // set recon wrap picture
//      PelBuf sWrap = m_bufs[PIC_RECON_WRAP].get(compID);
//      Pel *srcWrap = sWrap.bufAt(left, top);

//      // apply padding on bottom
//      Pel *srcBottomWrap = srcWrap + sWrap.stride * (height - 1) - xmargin;
//      Pel *dstBottomWrap = srcBottomWrap + sWrap.stride;
//      for (int y = 0; y < ymargin; y++)
//      {
//        ::memcpy(dstBottomWrap, srcBottomWrap, sizeof(Pel)*(2 * xmargin + width));
//        dstBottomWrap += sWrap.stride;
//      }

//      // apply padding for top
//      // si is still (-marginX, SubpictureHeight-1)
//      Pel *srcTopWrap = srcWrap - xmargin;
//      Pel *dstTopWrap = srcTopWrap - sWrap.stride;
//      // si is now (-marginX, 0)
//      for (int y = 0; y < ymargin; y++)
//      {
//        ::memcpy(dstTopWrap, srcTopWrap, sizeof(Pel)*(2 * xmargin + width));
//        dstTopWrap -= sWrap.stride;
//      }
//    }
  } // end of for
}
#endif

void Picture::extendPicBorder( bool top, bool bottom, bool leftrightT, bool leftrightB, ChannelType chType )
{
  for( int comp = 0; comp < getNumberValidComponents( cs->area.chromaFormat ); comp++ )
  {
    ComponentID compID = ComponentID( comp );

    if( chType != MAX_NUM_CHANNEL_TYPE && toChannelType( compID ) != chType )
      continue;

    PelBuf p = m_bufs[PIC_RECONSTRUCTION].get( compID );

    int xmargin = margin >> getComponentScaleX( compID, cs->area.chromaFormat );
    int ymargin = margin >> getComponentScaleY( compID, cs->area.chromaFormat );

#if JVET_Q0764_WRAP_AROUND_WITH_RPR
    if( cs->pps->getUseWrapAround() )
#else
    if( cs->sps->getUseWrapAround() )
#endif
    {
      PelBuf prw = m_bufs[PIC_RECON_WRAP].get( compID );

#if JVET_Q0764_WRAP_AROUND_WITH_RPR
      int xoffset = cs->pps->getWrapAroundOffset() >> getComponentScaleX( compID, cs->area.chromaFormat );
#else
      int xoffset = cs->sps->getWrapAroundOffset() >> getComponentScaleX( compID, cs->area.chromaFormat );
#endif
      if( leftrightT )
      {
        Pel* piprw = prw.bufAt( 0, 1 );

        for( int y = 1; y < p.height / 2; y++ )
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
          piprw += p.stride;
        }
      }
      if( leftrightB )
      {
        Pel* piprw = prw.bufAt( 0, p.height / 2 );

        for( int y = 1; y < p.height / 2; y++ )
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
          piprw += p.stride;
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

void Picture::createSpliceIdx(int nums)
{
  m_ctuNums = nums;
  m_spliceIdx = new int[m_ctuNums];
  memset(m_spliceIdx, 0, m_ctuNums * sizeof(int));
}

bool Picture::getSpliceFull()
{
  int count = 0;
  for (int i = 0; i < m_ctuNums; i++)
  {
    if (m_spliceIdx[i] != 0)
      count++;
  }
  if (count < m_ctuNums * 0.25)
    return false;
  return true;
}

void Picture::startProcessingTimer()
{
  m_processingStartTime = std::chrono::steady_clock::now();
}

void Picture::stopProcessingTimer()
{
  auto endTime = std::chrono::steady_clock::now();
  m_dProcessingTime += std::chrono::duration<double>(endTime - m_processingStartTime).count();
}
