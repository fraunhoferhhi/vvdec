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

/** \file     Picture.h
 *  \brief    Description of a coded picture
 */

#pragma once

#include "CommonDef.h"

#include "Common.h"
#include "Unit.h"
#include "Buffer.h"
#include "Unit.h"
#include "Slice.h"
#include "CodingStructure.h"
#include "SEI_internal.h"

#include "vvdec/sei.h"

namespace vvdec
{
using namespace x86_simd;

struct Picture;

typedef std::list<Picture*> PicList;

struct PicListRange
{
  PicList::const_iterator m_begin;
  PicList::const_iterator m_end;

  const PicList::const_iterator begin() const { return m_begin; }
  const PicList::const_iterator end  () const { return m_end;   }
};


struct Picture : public UnitArea
{
  Picture() = default;
  ~Picture() = default;
  CLASS_COPY_MOVE_DELETE( Picture )

  void create(const ChromaFormat &_chromaFormat, const Size &size, const unsigned _maxCUSize, const unsigned margin, const int layerId, UserAllocator* userAlloc = nullptr );
  void createWrapAroundBuf( const bool isWrapAround, const unsigned _maxCUSize );
  void resetForUse( int _layerId );
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

  void finalInit( CUChunkCache* cuChunkCache, TUChunkCache* tuChunkCache, const SPS * sps, const PPS * pps, const std::shared_ptr<PicHeader>& ph, const APS* const alfApss[ALF_CTB_MAX_NUM_APS], const APS * lmcsAps, const APS* scalingListAps, bool phPSupdate = true );

  int      getPOC()                           const { return poc; }
  uint64_t getCts()                           const { return cts; }
  uint64_t getDts()                           const { return dts; }
  uint32_t getTLayer()                        const { return tempLayer; }
  uint32_t getNaluBits()                      const { return bits; }
  bool     getRap()                           const { return rap; }

  bool     isCLVSS()                          const { return !slices.empty() && slices[0]->isClvssPu(); }

  Pel*   getOrigin( const PictureType &type, const ComponentID compID ) const;
  PelBuf getOriginBuf( const PictureType &type, const ComponentID compID );

  Size   getBufSize( const PictureType &type, const ComponentID compID ) const;
  void*  getBufAllocator( const ComponentID compID );
  bool   isExternAllocator() const;
  const  UserAllocator* getUserAllocator() const;

  int  getDecodingOrderNumber()               const { return decodingOrderNumber; }
  void setDecodingOrderNumber(const int val)        { decodingOrderNumber = val;  }

  bool getMixedNaluTypesInPicFlag()           const { return slices[0]->getPPS()->getMixedNaluTypesInPicFlag(); }

  std::vector<Picture*> buildAllRefPicsVec();

public:
  void startProcessingTimer();
  void stopProcessingTimer();
  void resetProcessingTime() { m_dProcessingTime = 0; }
  double getProcessingTime() const { return m_dProcessingTime; }

  std::chrono::time_point<std::chrono::steady_clock> m_processingStartTime;
  double                                             m_dProcessingTime = 0;
  std::mutex                                         m_timerMutex;

  enum PicStateEnum
  {
    init,
    parsing,
    parsed,
    reconstructing,
    reconstructed,
    finished
  };
  using PicState = std::atomic<PicStateEnum>;
  PicState progress{ init };

  enum RefMark : uint8_t
  {
    unreferenced = 0,
    ShortTerm,
    LongTerm
  };

  bool    subPicExtStarted        = false;
  bool    borderExtStarted        = false;
  RefMark dpbReferenceMark        = unreferenced;   // only used during parsing, manage the DPB and to build the reference picture lists
  bool    neededForOutput         = false;
  bool    stillReferenced         = false;   // set as long as there is a picture in progress, that references this one. ('referenced' might be unset during parsing)
  bool    isReferencePic          = false;   // mainly for setting vvdecPicAttributes::isRefPic for the library output frame
  bool    wasLost                 = false;
  bool    error                   = false;
  bool    exceptionThrownOut      = false;
  bool    topField                = false;
  bool    fieldPic                = false;
  bool    nonReferencePictureFlag = false;
  int     skippedDecCount         = 0;

  bool              picCheckedDPH = false;
  std::vector<bool> subpicsCheckedDPH;
  bool              dphMismatch   = false;

  // As long as this field is true, the picture will not be reused or deleted.
  // An external application needs to call DecLib::releasePicture(), when it is done using the picture buffer.
  bool lockedByApplication = false;

  int         poc          = 0;
  uint64_t    cts          = 0;   // composition time stamp
  uint64_t    dts          = 0;   // decoding time stamp
  uint32_t    tempLayer    = std::numeric_limits<uint32_t>::max();
  uint32_t    depth        = 0;
  int         layerId      = NOT_VALID;
  NalUnitType eNalUnitType = NAL_UNIT_INVALID;
  uint32_t    bits         = 0;   // input nal bit count
  bool        rap          = 0;   // random access point flag
  int         decodingOrderNumber = 0;

  std::vector<int>        sliceSubpicIdx;
  std::vector<SubPic>     subPictures;
  std::vector<PelStorage> m_subPicRefBufs;   // used as reference for subpictures, that are treated as pictures

  bool subLayerNonReferencePictureDueToSTSA = 0;

  PelStorage     m_bufs[NUM_PIC_TYPES];
  uint32_t       margin      = 0;

  WaitCounter     m_divTasksCounter;        // for all tasks, that are not covered by the other WaitCounters => only needed for cleanup during exception handling
  WaitCounter     m_ctuTaskCounter;
  WaitCounter     m_borderExtTaskCounter;
  Barrier         m_copyWrapBufDone;
  BlockingBarrier parseDone;
  BlockingBarrier reconDone;
#if RECO_WHILE_PARSE
  std::vector<Barrier> ctuParsedBarrier;
#endif
#if ALLOW_MIDER_LF_DURING_PICEXT
  CBarrierVec     refPicExtDepBarriers;
#endif

  CodingStructure*    cs = nullptr;
  std::vector<Slice*> slices;

  seiMessages        seiMessageList;

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

  bool         isWrapAroundEnabled( const PPS* pps ) const  { return  pps->getUseWrapAround() && !isRefScaled( pps ); }

  Slice*       allocateNewSlice( Slice** pilot = nullptr );
  void         clearSliceBuffer();
  bool         lastSliceOfPicPresent() const;

  void         waitForAllTasks();
  void         ensureUsableAsRef();
  void         fillGrey( const SPS* sps );

#if TRACE_ENABLE_ITT
  __itt_domain* m_itt_decLibInst;
#endif

public:

#if  ENABLE_SIMD_OPT_PICTURE && defined( TARGET_SIMD_X86 )
  void initPictureX86();
  template <X86_VEXT vext>
  void _initPictureX86();
#endif
};

int calcAndPrintHashStatus(const CPelUnitBuf& pic, const vvdecSEIDecodedPictureHash* pictureHashSEI, const BitDepths &bitDepths, const MsgLevel msgl);

}   // namespace vvdec
