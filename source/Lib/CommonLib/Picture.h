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

Copyright (c) 2018-2021, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. 
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

/** \file     Picture.h
 *  \brief    Description of a coded picture
 */

#ifndef __PICTURE__
#define __PICTURE__

#include "CommonDef.h"

#include "Common.h"
#include "Unit.h"
#include "Buffer.h"
#include "Unit.h"
#include "Slice.h"
#include "CodingStructure.h"
#include <deque>

#include "InterpolationFilter.h"

typedef std::list<SEI_internal*> seiMessages;

struct Picture : public UnitArea
{
  Picture() = default;
  ~Picture() = default;

  void create(const ChromaFormat &_chromaFormat, const Size &size, const unsigned _maxCUSize, const unsigned margin, const int layerId);
  void resetForUse();
  void destroy();
  
         Pel*      getRecoBufPtr   (const ComponentID compID, bool wrap=false)       { return m_bufs[wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION].bufs[compID].buf; }
  const  Pel*      getRecoBufPtr   (const ComponentID compID, bool wrap=false) const { return m_bufs[wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION].bufs[compID].buf; }
         ptrdiff_t getRecoBufStride(const ComponentID compID, bool wrap=false)       { return m_bufs[wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION].bufs[compID].stride; }
  const  ptrdiff_t getRecoBufStride(const ComponentID compID, bool wrap=false) const { return m_bufs[wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION].bufs[compID].stride; }
         PelBuf     getRecoBuf(const ComponentID compID, bool wrap=false);
  const CPelBuf     getRecoBuf(const ComponentID compID, bool wrap=false) const;
         PelBuf     getRecoBuf(const CompArea &blk,      bool wrap=false);
  const CPelBuf     getRecoBuf(const CompArea &blk,      bool wrap=false) const;
         PelUnitBuf getRecoBuf(const UnitArea &unit,     bool wrap=false);
  const CPelUnitBuf getRecoBuf(const UnitArea &unit,     bool wrap=false) const;
         PelUnitBuf getRecoBuf(bool wrap=false);
  const CPelUnitBuf getRecoBuf(bool wrap=false) const;

  // This returns a CPelBuf, with the origin at the picture origin (0,0), but the actual storage size of the sub picture.
  // It can be used the same way as the full picture buffer, but you should only reference the data within the actual sub picture.
  // Also, handle with care, because stride < width.
  const CPelBuf getSubPicBuf( int subPicIdx, const ComponentID compID, bool wrap = false ) const
  {
    CHECK( wrap, "wraparound for subpics not supported yet" );

    Position subPicPos( subPictures[subPicIdx].getSubPicLeft() >> getComponentScaleX( compID, m_subPicRefBufs[subPicIdx].chromaFormat ),
                        subPictures[subPicIdx].getSubPicTop()  >> getComponentScaleY( compID, m_subPicRefBufs[subPicIdx].chromaFormat ) );

    Size targetSize( m_bufs[PIC_RECONSTRUCTION].get( compID ) );

    const auto& subPicComp = m_subPicRefBufs[subPicIdx].bufs[compID];
    return CPelBuf( subPicComp.bufAt( -subPicPos.x, -subPicPos.y ), subPicComp.stride, targetSize );
  }
  const Pel*      getSubPicBufPtr   ( int subPicIdx, const ComponentID compID, bool wrap = false ) const { return getSubPicBuf( subPicIdx, compID, wrap ).buf;    }
  const ptrdiff_t getSubPicBufStride( int subPicIdx, const ComponentID compID, bool wrap = false ) const { return getSubPicBuf( subPicIdx, compID, wrap ).stride; }

private:
         PelBuf     getBuf(const ComponentID compID, const PictureType &type)       { return m_bufs[type].bufs[ compID ]; }
  const CPelBuf     getBuf(const ComponentID compID, const PictureType &type) const { return m_bufs[type].bufs[ compID ]; }
         PelBuf     getBuf(const CompArea &blk,      const PictureType &type);
  const CPelBuf     getBuf(const CompArea &blk,      const PictureType &type) const;
         PelUnitBuf getBuf(const UnitArea &unit,     const PictureType &type);
  const CPelUnitBuf getBuf(const UnitArea &unit,     const PictureType &type) const;

public:
  void extendPicBorder    (                  bool top = true, bool bottom = true, bool leftrightT = true, bool leftrightB = true, ChannelType chType = MAX_NUM_CHANNEL_TYPE );
  void extendPicBorderBuf ( PelStorage& buf, bool top = true, bool bottom = true, bool leftrightT = true, bool leftrightB = true, ChannelType chType = MAX_NUM_CHANNEL_TYPE );
  void extendPicBorderWrap(                  bool top = true, bool bottom = true, bool leftrightT = true, bool leftrightB = true, ChannelType chType = MAX_NUM_CHANNEL_TYPE );

  void (*paddPicBorderBot) (Pel *pi, ptrdiff_t stride,int width,int xmargin,int ymargin);
  void (*paddPicBorderTop) (Pel *pi, ptrdiff_t stride,int width,int xmargin,int ymargin);
  void (*paddPicBorderLeftRight) (Pel *pi, ptrdiff_t stride,int width,int xmargin,int height);

  void finalInit( const SPS * sps, const PPS * pps, PicHeader *picHeader, APS* alfApss[ALF_CTB_MAX_NUM_APS], APS * lmcsAps, APS* scalingListAps );
  int      getPOC()                           const { return poc; }
  uint64_t getCts()                           const { return cts; }
  uint64_t getDts()                           const { return dts; }
  uint32_t getTLayer()                        const { return layer; }
  uint64_t getNaluBits()                      const { return bits; }
  bool     getRap()                           const { return rap; }

  void   setBorderExtension( bool bFlag )           { borderExtStarted = bFlag;}
  Pel*   getOrigin( const PictureType &type, const ComponentID compID ) const;
  PelBuf getOriginBuf( const PictureType &type, const ComponentID compID );

  int  getDecodingOrderNumber()               const { return decodingOrderNumber; }
  void setDecodingOrderNumber(const int val)        { decodingOrderNumber = val;  }
  
  int           getSpliceIdx(uint32_t idx) const { return m_spliceIdx[idx]; }
  void          setSpliceIdx(uint32_t idx, int poc) { m_spliceIdx[idx] = poc; }
  void          createSpliceIdx(int nums);
  bool          getSpliceFull();
#if RPR_FIX
  static void   sampleRateConv( const std::pair<int, int> scalingRatio, const std::pair<int, int> compScale,
                                const CPelBuf& beforeScale, const int beforeScaleLeftOffset, const int beforeScaleTopOffset,
                                const PelBuf& afterScale, const int afterScaleLeftOffset, const int afterScaleTopOffset,
                                const int bitDepth, const bool useLumaFilter, const bool downsampling,
                                const bool horCollocatedPositionFlag, const bool verCollocatedPositionFlag );
  static void   rescalePicture( const std::pair<int, int> scalingRatio, const CPelUnitBuf& beforeScaling, const Window& confBefore, const PelUnitBuf& afterScaling, const Window& confAfter, const ChromaFormat chromaFormatIDC, const BitDepths& bitDepths, const bool useLumaFilter, const bool downsampling, const bool horCollocatedChromaFlag, const bool verCollocatedChromaFlag );
#else
  static void   sampleRateConv( const Pel* orgSrc, SizeType orgWidth, SizeType orgHeight, ptrdiff_t orgStride, Pel* scaledSrc, SizeType scaledWidth, SizeType scaledHeight, SizeType paddedWidth, SizeType paddedHeight, ptrdiff_t scaledStride, const int bitDepth, const bool useLumaFilter, const bool downsampling = false );

  static void   rescalePicture(const CPelUnitBuf& beforeScaling, const Window& confBefore, const PelUnitBuf& afterScaling, const Window& confAfter, const ChromaFormat chromaFormatIDC, const BitDepths& bitDepths, const bool useLumaFilter, const bool downsampling = false);
#endif
public:
#if JVET_O1143_MV_ACROSS_SUBPIC_BOUNDARY
  std::vector<PelStorage> m_subPicRefBufs;   // used as reference for subpictures, that are treated as pictures
#endif

  void startProcessingTimer();
  void stopProcessingTimer();
  void resetProcessingTime() { m_dProcessingTime = 0; }
  double getProcessingTime() const { return m_dProcessingTime; }

  std::chrono::time_point<std::chrono::steady_clock> m_processingStartTime;
  double                                             m_dProcessingTime = 0;

  bool borderExtStarted               = false;
#if JVET_Q0764_WRAP_AROUND_WITH_RPR
  bool wrapAroundValid                = false;
  unsigned wrapAroundOffset           = 0;
#endif
  bool referenced                     = false;
  bool reconstructed                  = false;
  bool inProgress                     = false;
  bool neededForOutput                = false;
  bool wasLost                        = false;
  bool longTerm                       = false;
  bool topField                       = false;
  bool fieldPic                       = false;
  int  skippedDecCount                = 0;
  int  m_prevQP[MAX_NUM_CHANNEL_TYPE] = { -1, -1 };
#if JVET_S0124_UNAVAILABLE_REFERENCE
  bool nonReferencePictureFlag        = false;
#endif

  // As long as this field is true, the picture will not be reused or deleted.
  // An external application needs to call DecLib::releasePicture(), when it is done using the picture buffer.
  bool lockedByApplication = false;

  int         poc          = 0;
  uint64_t    cts          = 0;   // composition time stamp
  uint64_t    dts          = 0;   // decoding time stamp
  uint32_t    layer        = std::numeric_limits<uint32_t>::max();
  uint32_t    depth        = 0;
  int         layerId      = NOT_VALID;
  NalUnitType eNalUnitType = NAL_UNIT_INVALID;
  uint64_t    bits         = 0;   // input nal bit count
  bool        rap          = 0;   // random access point flag
  int         decodingOrderNumber = 0;
#if JVET_S0258_SUBPIC_CONSTRAINTS
  std::vector<int> sliceSubpicIdx;
  std::vector<SubPic> subPictures;
  int numSlices            = 1;
#else
#if JVET_R0058
  int  numSubpics          = 1;
  std::vector<int> subpicWidthInCTUs;
  std::vector<int> subpicHeightInCTUs;
  std::vector<int> subpicCtuTopLeftX;
  std::vector<int> subpicCtuTopLeftY;
#endif
#endif

  bool        subLayerNonReferencePictureDueToSTSA = 0;

  int*        m_spliceIdx  = nullptr;
  int         m_ctuNums    = 0;

  PelStorage  m_bufs[NUM_PIC_TYPES];
  uint32_t    margin       = 0;
  const Picture*           unscaledPic;

#if !JVET_S0258_SUBPIC_CONSTRAINTS
#if JVET_R0276_REORDERED_SUBPICS
  std::vector<int> subPicIDs;
#endif
#endif

  WaitCounter     m_ctuTaskCounter;
  WaitCounter     m_dmvrTaskCounter;
  WaitCounter     m_borderExtTaskCounter;
  Barrier         m_copyWrapBufDone;
  BlockingBarrier done;
#if RECO_WHILE_PARSE
  Barrier        *ctuParsedBarrier = nullptr;
#endif
#if ALLOW_MIDER_LF_DURING_PICEXT
  CBarrierVec     refPicExtDepBarriers;
#endif
  Barrier         parseDone;

  CodingStructure*   cs    = nullptr;
  std::vector<Slice*> slices;
  seiMessages  SEIs;

  bool               isRefScaled( const PPS* pps ) const
  {
    const PPS*  pps0     = slices[0]->getPPS();
    const Size& recoSize = m_bufs[PIC_RECONSTRUCTION].bufs[COMPONENT_Y];
    return ( recoSize.width  != pps->getPicWidthInLumaSamples()    ||
             recoSize.height != pps->getPicHeightInLumaSamples() ) ||
           ( ( pps0->getScalingWindow().getWindowEnabledFlag()   || pps->getScalingWindow().getWindowEnabledFlag() ) && (
               pps0->getScalingWindow().getWindowLeftOffset()    != pps->getScalingWindow().getWindowLeftOffset()  ||
               pps0->getScalingWindow().getWindowRightOffset()   != pps->getScalingWindow().getWindowRightOffset() ||
               pps0->getScalingWindow().getWindowTopOffset()     != pps->getScalingWindow().getWindowTopOffset()   ||
               pps0->getScalingWindow().getWindowBottomOffset()  != pps->getScalingWindow().getWindowBottomOffset() ) );
  }

  std::shared_ptr<PicHeader> picHeader;
  void                       setPicHead( const std::shared_ptr<PicHeader>& ph );

#if JVET_Q0764_WRAP_AROUND_WITH_RPR
  bool         isWrapAroundEnabled( const PPS* pps ) const  { return  pps->getUseWrapAround() && !isRefScaled( pps ); }
#endif

  void         allocateNewSlice();
  Slice        *swapSliceObject(Slice * p, uint32_t i);
  void         clearSliceBuffer();

#if TRACE_ENABLE_ITT
  __itt_domain* m_itt_decLibInst;
#endif

public:
  std::vector<uint8_t>  m_ccAlfFilterControl[2];
        uint8_t*        getccAlfFilterControl( int compIdx )       { return m_ccAlfFilterControl[compIdx].data(); }
  const uint8_t*        getccAlfFilterControl( int compIdx ) const { return m_ccAlfFilterControl[compIdx].data(); }
  std::vector<uint8_t>* getccAlfFilterControl()                    { return m_ccAlfFilterControl; }
  void                  resizeccAlfFilterControl( int numEntries )
  {
    for( int compIdx = 0; compIdx < 2; compIdx++ )
    {
      m_ccAlfFilterControl[compIdx].resize( numEntries );
      m_ccAlfFilterControl[compIdx].assign( numEntries, 0 );
    }
  }
  
  std::vector<uint8_t>  m_alfCtuEnableFlag[MAX_NUM_COMPONENT];
  uint8_t*              getAlfCtuEnableFlag( int compIdx ) { return m_alfCtuEnableFlag[compIdx].data(); }
  std::vector<uint8_t>* getAlfCtuEnableFlag()              { return m_alfCtuEnableFlag; }
  void                  resizeAlfCtuEnableFlag( int numEntries )
  {
    for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
    {
      m_alfCtuEnableFlag[compIdx].resize( numEntries );
      m_alfCtuEnableFlag[compIdx].assign( numEntries, 0 );
    }
  }

  std::vector<short>  m_alfCtbFilterIndex;
        short*        getAlfCtbFilterIndex()       { return m_alfCtbFilterIndex.data(); }
  const short*        getAlfCtbFilterIndex() const { return m_alfCtbFilterIndex.data(); }
  std::vector<short>& getAlfCtbFilterIndexVec() { return m_alfCtbFilterIndex; }
  void                resizeAlfCtbFilterIndex( int numEntries )
  {
    m_alfCtbFilterIndex.resize( numEntries );
    m_alfCtbFilterIndex.assign( numEntries, 0 );
  }

  std::vector<uint8_t>  m_alfCtuAlternative[MAX_NUM_COMPONENT-1];
  uint8_t*              getAlfCtuAlternativeData( int compIdx ) { return m_alfCtuAlternative[compIdx].data(); }
  std::vector<uint8_t>* getAlfCtuAlternative()                 { return m_alfCtuAlternative; }
  void resizeAlfCtuAlternative( int numEntries )
  {
    for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT-1; compIdx++ )
    {
      m_alfCtuAlternative[compIdx].resize( numEntries );
      m_alfCtuAlternative[compIdx].assign( numEntries, 0 );
    }
  }

#if  ENABLE_SIMD_OPT_PICTURE
  void initPictureX86();
  template <X86_VEXT vext>
  void _initPictureX86();
#endif
};

int calcAndPrintHashStatus(const CPelUnitBuf& pic, const class seiDecodedPictureHash* pictureHashSEI, const BitDepths &bitDepths, const MsgLevel msgl);


#endif
