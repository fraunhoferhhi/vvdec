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

/** \file     DecSlice.cpp
    \brief    slice decoder class
*/

#include "DecSlice.h"

#include "CommonLib/TrQuant.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_next.h"
#include "CommonLib/TimeProfiler.h"
#include "CommonLib/AdaptiveLoopFilter.h"

#include <vector>

//! \ingroup DecoderLib
//! \{

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

void DecSlice::parseSlice( Slice* slice, InputBitstream* bitstream, int threadId )
{
  PROFILER_SCOPE_AND_STAGE_EXT( 1, g_timeProfiler, P_CONTROL_PARSE_DERIVE_LL, *slice->getPic()->cs, CH_L );
  const unsigned numSubstreams = slice->getNumberOfSubstreamSizes() + 1;
  // Table of extracted substreams.
  std::vector<std::shared_ptr<InputBitstream>> ppcSubstreams( numSubstreams );
  for( unsigned idx = 0; idx < numSubstreams; idx++ )
  {
    ppcSubstreams[idx].reset( bitstream->extractSubstream( idx + 1 < numSubstreams ? ( slice->getSubstreamSize( idx ) << 3 )
                                                                                   : bitstream->getNumBitsLeft() ) );
  }

  const SPS*     sps          = slice->getSPS();
  Picture*       pic          = slice->getPic();

  // setup coding structure
  CodingStructure& cs = *pic->cs;
  cs.slice            = slice;
  cs.chromaQpAdj      = 0;

  const int       startCtuTsAddr              = slice->getFirstCtuRsAddrInSlice();
  const unsigned  widthInCtus                 = cs.pcv->widthInCtus;
  const bool      wavefrontsEnabled           = cs.sps->getEntropyCodingSyncEnabledFlag();
#if JVET_R0165_OPTIONAL_ENTRY_POINT
  const bool      entryPointPresent           = cs.sps->getEntryPointsPresentFlag();
#else
  const bool      wavefrontsEntryPointPresent = cs.sps->getEntropyCodingSyncEntryPointsPresentFlag();
#endif

  if( startCtuTsAddr == 0 )
  {
    cs.picture->resizeccAlfFilterControl( cs.pcv->sizeInCtus );
    cs.picture->resizeAlfCtuEnableFlag  ( cs.pcv->sizeInCtus );
    cs.picture->resizeAlfCtbFilterIndex ( cs.pcv->sizeInCtus );
    cs.picture->resizeAlfCtuAlternative ( cs.pcv->sizeInCtus );
  }

  AdaptiveLoopFilter::reconstructCoeffAPSs( *slice );
                                              
  CABACDecoder cabacDecoder;
  CABACReader&  cabacReader  = *cabacDecoder.getCABACReader();
  cabacReader.initBitstream( ppcSubstreams[0].get() );
  cabacReader.initCtxModels( *slice );

  // if first slice, finish init of the coding structure
  if( startCtuTsAddr == 0 )
  {
    cs.initStructData();
  }

  // Quantization parameter
  pic->m_prevQP[0] = pic->m_prevQP[1] = slice->getSliceQp();

  CHECK( pic->m_prevQP[0] == std::numeric_limits<int>::max(), "Invalid previous QP" );

  DTRACE( g_trace_ctx, D_HEADER, "=========== POC: %d ===========\n", slice->getPOC() );

  unsigned subStrmId = 0;
  for( unsigned ctuIdx = 0; ctuIdx < slice->getNumCtuInSlice(); ctuIdx++ )
  {
    const unsigned  ctuRsAddr       = slice->getCtuAddrInSlice(ctuIdx);
    const unsigned  ctuXPosInCtus   = ctuRsAddr % widthInCtus;
    const unsigned  ctuYPosInCtus   = ctuRsAddr / widthInCtus;
    const unsigned  tileColIdx      = slice->getPPS()->ctuToTileCol( ctuXPosInCtus );
    const unsigned  tileRowIdx      = slice->getPPS()->ctuToTileRow( ctuYPosInCtus );
    const unsigned  tileXPosInCtus  = slice->getPPS()->getTileColumnBd( tileColIdx );
    const unsigned  tileYPosInCtus  = slice->getPPS()->getTileRowBd( tileRowIdx );
    const unsigned  tileColWidth    = slice->getPPS()->getTileColumnWidth( tileColIdx );
    const unsigned  tileRowHeight   = slice->getPPS()->getTileRowHeight( tileRowIdx );
    const unsigned  tileIdx         = slice->getPPS()->getTileIdx( ctuXPosInCtus, ctuYPosInCtus);
    const unsigned  maxCUSize       = sps->getMaxCUWidth();
    Position pos( ctuXPosInCtus*maxCUSize, ctuYPosInCtus*maxCUSize) ;
    UnitArea ctuArea(cs.area.chromaFormat, Area( pos.x, pos.y, maxCUSize, maxCUSize ) );
#if JVET_O1143_MV_ACROSS_SUBPIC_BOUNDARY
    const SubPic& curSubPic = slice->getPPS()->getSubPicFromPos( pos );
    // padding/restore at slice level
    if( slice->getPPS()->getNumSubPics() >= 2 && curSubPic.getTreatedAsPicFlag() && ctuIdx == 0 )
    {
      int subPicX      = (int)curSubPic.getSubPicLeft();
      int subPicY      = (int)curSubPic.getSubPicTop();
      int subPicWidth  = (int)curSubPic.getSubPicWidthInLumaSample();
      int subPicHeight = (int)curSubPic.getSubPicHeightInLumaSample();
      for( int rlist = REF_PIC_LIST_0; rlist < NUM_REF_PIC_LIST_01; rlist++ )
      {
        int n = slice->getNumRefIdx((RefPicList)rlist);
        for (int idx = 0; idx < n; idx++) 
        {
          Picture *refPic = slice->getRefPic((RefPicList)rlist, idx);
#if JVET_R0058
#if JVET_S0258_SUBPIC_CONSTRAINTS
          if( !refPic->getSubPicSaved() && refPic->subPictures.size() > 1 )
#else
          if( !refPic->getSubPicSaved() && refPic->numSubpics > 1 )
#endif
#else
          if (!refPic->getSubPicSaved()) 
#endif
          {
            refPic->saveSubPicBorder  ( refPic->getPOC(), subPicX, subPicY, subPicWidth, subPicHeight );
            refPic->extendSubPicBorder( refPic->getPOC(), subPicX, subPicY, subPicWidth, subPicHeight );
            refPic->setSubPicSaved    ( true) ;
          }
        }
      }
    }
#endif

    DTRACE_UPDATE( g_trace_ctx, std::make_pair( "ctu", ctuRsAddr ) );

    cabacReader.initBitstream( ppcSubstreams[subStrmId].get() );

    // set up CABAC contexts' state for this CTU
    if( ctuXPosInCtus == tileXPosInCtus && ctuYPosInCtus == tileYPosInCtus )
    {
      if( ctuIdx != 0 ) // if it is the first CTU, then the entropy coder has already been reset
      {
        cabacReader.initCtxModels( *slice );
      }
      pic->m_prevQP[0] = pic->m_prevQP[1] = slice->getSliceQp();
    }
    else if( ctuXPosInCtus == tileXPosInCtus && wavefrontsEnabled )
    {
      // Synchronize cabac probabilities with upper-right CTU if it's available and at the start of a line.
      if( ctuIdx != 0 ) // if it is the first CTU, then the entropy coder has already been reset
      {
        cabacReader.initCtxModels( *slice );
      }
      if( cs.getCURestricted( pos.offset(0, -1), pos, slice->getIndependentSliceIdx(), tileIdx, CH_L ) )
      {
        cabacReader.getCtx() = m_entropyCodingSyncContextState[threadId]; // TODO: sync this
      }
      pic->m_prevQP[0] = pic->m_prevQP[1] = slice->getSliceQp();
    }

    bool updateBcwCodingOrder = cs.slice->getSliceType() == B_SLICE && ctuIdx == 0;

    if( updateBcwCodingOrder )
    {
      resetBcwCodingOrder( true, cs );
    }

    //memset( cs.getCtuData( ctuRsAddr ).cuPtr, 0, sizeof( CtuData::cuPtr ) );

    cabacReader.coding_tree_unit( cs, ctuArea, pic->m_prevQP, ctuRsAddr );

    if( ctuXPosInCtus == tileXPosInCtus && wavefrontsEnabled )
    {
      m_entropyCodingSyncContextState[threadId] = cabacReader.getCtx();  // TODO: sync this
    }

    if( ctuIdx == slice->getNumCtuInSlice()-1 )
    {
      unsigned binVal = cabacReader.terminating_bit();
      CHECK( !binVal, "Expecting a terminating bit" );
#if DECODER_CHECK_SUBSTREAM_AND_SLICE_TRAILING_BYTES
      cabacReader.remaining_bytes( false );
#endif
    }
    else if( ( ctuXPosInCtus + 1 == tileXPosInCtus + tileColWidth ) &&
             ( ctuYPosInCtus + 1 == tileYPosInCtus + tileRowHeight || wavefrontsEnabled ) )
    {
      // The sub-stream/stream should be terminated after this CTU.
      // (end of slice-segment, end of tile, end of wavefront-CTU-row)
      unsigned binVal = cabacReader.terminating_bit();
      CHECK( !binVal, "Expecting a terminating bit" );
      
#if JVET_R0165_OPTIONAL_ENTRY_POINT
      if( entryPointPresent )
#else
      bool isLastTileCtu = (ctuXPosInCtus + 1 == tileXPosInCtus + tileColWidth) && (ctuYPosInCtus + 1 == tileYPosInCtus + tileRowHeight);
      if( isLastTileCtu || wavefrontsEntryPointPresent )
#endif
      {
#if DECODER_CHECK_SUBSTREAM_AND_SLICE_TRAILING_BYTES
        cabacReader.remaining_bytes( true );
#endif
        subStrmId++;
      }
    }
#if JVET_O1143_MV_ACROSS_SUBPIC_BOUNDARY
    if( slice->getPPS()->getNumSubPics() >= 2 && curSubPic.getTreatedAsPicFlag() && ctuIdx == (slice->getNumCtuInSlice() - 1) )
    // for last Ctu in the slice
    {
      int subPicX = (int)curSubPic.getSubPicLeft();
      int subPicY = (int)curSubPic.getSubPicTop();
      int subPicWidth = (int)curSubPic.getSubPicWidthInLumaSample();
      int subPicHeight = (int)curSubPic.getSubPicHeightInLumaSample();
      for( int rlist = REF_PIC_LIST_0; rlist < NUM_REF_PIC_LIST_01; rlist++ )
      {
        int n = slice->getNumRefIdx((RefPicList)rlist);
        for( int idx = 0; idx < n; idx++ )
        {
          Picture *refPic = slice->getRefPic((RefPicList)rlist, idx);
          if( refPic->getSubPicSaved() )
          {
            refPic->restoreSubPicBorder( refPic->getPOC(), subPicX, subPicY, subPicWidth, subPicHeight );
            refPic->setSubPicSaved(false);
          }
        }
      }
    }
#endif
#if RECO_WHILE_PARSE

    pic->ctuParsedBarrier[ctuRsAddr].unlock();
#endif

    if( ctuRsAddr + 1 == pic->cs->pcv->sizeInCtus ) pic->parseDone.unlock();
  }
}

//! \}
