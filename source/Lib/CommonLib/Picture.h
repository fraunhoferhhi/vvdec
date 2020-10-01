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


class SEI;
typedef std::list<SEI*> SEIMessages;



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

         PelBuf     getBuf(const ComponentID compID, const PictureType &type)       { return m_bufs[type].bufs[ compID ]; }
  const CPelBuf     getBuf(const ComponentID compID, const PictureType &type) const { return m_bufs[type].bufs[ compID ]; }
         PelBuf     getBuf(const CompArea &blk,      const PictureType &type);
  const CPelBuf     getBuf(const CompArea &blk,      const PictureType &type) const;
         PelUnitBuf getBuf(const UnitArea &unit,     const PictureType &type);
  const CPelUnitBuf getBuf(const UnitArea &unit,     const PictureType &type) const;

  void extendPicBorder( bool top = true, bool bottom = true, bool leftrightT = true, bool leftrightB = true, ChannelType chType = MAX_NUM_CHANNEL_TYPE );
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

  void   setBorderExtension( bool bFlag)            { isBorderExtended = bFlag;}
  Pel*   getOrigin( const PictureType &type, const ComponentID compID ) const;
  PelBuf getOriginBuf( const PictureType &type, const ComponentID compID );

  int  getDecodingOrderNumber()               const { return decodingOrderNumber; }
  void setDecodingOrderNumber(const int val)        { decodingOrderNumber = val;  }
  
  int           getSpliceIdx(uint32_t idx) const { return m_spliceIdx[idx]; }
  void          setSpliceIdx(uint32_t idx, int poc) { m_spliceIdx[idx] = poc; }
  void          createSpliceIdx(int nums);
  bool          getSpliceFull();
  static void   sampleRateConv( const Pel* orgSrc, SizeType orgWidth, SizeType orgHeight, ptrdiff_t orgStride, Pel* scaledSrc, SizeType scaledWidth, SizeType scaledHeight, SizeType paddedWidth, SizeType paddedHeight, ptrdiff_t scaledStride, const int bitDepth, const bool useLumaFilter, const bool downsampling = false );

  static void   rescalePicture(const CPelUnitBuf& beforeScaling, const Window& confBefore, const PelUnitBuf& afterScaling, const Window& confAfter, const ChromaFormat chromaFormatIDC, const BitDepths& bitDepths, const bool useLumaFilter, const bool downsampling = false);

public:
#if JVET_O1143_MV_ACROSS_SUBPIC_BOUNDARY  
  bool m_isSubPicBorderSaved = false;

  PelStorage m_bufSubPicAbove;
  PelStorage m_bufSubPicBelow;
  PelStorage m_bufSubPicLeft;
  PelStorage m_bufSubPicRight;

  void    saveSubPicBorder(int POC, int subPicX0, int subPicY0, int subPicWidth, int subPicHeight);
  void  extendSubPicBorder(int POC, int subPicX0, int subPicY0, int subPicWidth, int subPicHeight);
  void restoreSubPicBorder(int POC, int subPicX0, int subPicY0, int subPicWidth, int subPicHeight);

  bool getSubPicSaved()          { return m_isSubPicBorderSaved; }
  void setSubPicSaved(bool bVal) { m_isSubPicBorderSaved = bVal; }
#endif

  bool isBorderExtended               = false;
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
  SEIMessages        SEIs;

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

  PicHeader   *picHeader = nullptr;
  void         allocatePicHead() { picHeader = new PicHeader; }
  PicHeader   *swapPicHead( PicHeader *ph );

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
  uint8_t*              getccAlfFilterControl( int compIdx ) { return m_ccAlfFilterControl[compIdx].data(); }
  std::vector<uint8_t>* getccAlfFilterControl()              { return m_ccAlfFilterControl; }
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
  short *             getAlfCtbFilterIndex() { return m_alfCtbFilterIndex.data(); }
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

int calcAndPrintHashStatus(const CPelUnitBuf& pic, const class SEIDecodedPictureHash* pictureHashSEI, const BitDepths &bitDepths, const MsgLevel msgl);


#endif
