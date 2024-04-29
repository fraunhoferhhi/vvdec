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

/** \file     DecSlice.cpp
    \brief    slice decoder class
*/

#include "CABACReader.h"
#include "DecSlice.h"
#include "Picture.h"
#include "Slice.h"

#include "CommonLib/dtrace_next.h"
#include "CommonLib/TimeProfiler.h"
#include "CommonLib/AdaptiveLoopFilter.h"

#include <vector>


namespace vvdec
{

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

void DecSlice::parseSlice( Slice* slice, InputBitstream* bitstream, int threadId )
{
  PROFILER_SCOPE_AND_STAGE_EXT( 1, g_timeProfiler, P_CONTROL_PARSE_DERIVE_LL, *slice->getPic()->cs, CH_L );
  const unsigned numSubstreams = slice->getNumberOfSubstreamSizes() + 1;
  // Table of extracted substreams.
  std::vector<std::unique_ptr<InputBitstream>> ppcSubstreams( numSubstreams );
  for( unsigned idx = 0; idx < numSubstreams; idx++ )
  {
    ppcSubstreams[idx] = bitstream->extractSubstream( idx + 1 < numSubstreams ? ( slice->getSubstreamSize( idx ) << 3 ) : bitstream->getNumBitsLeft() );
  }

  const SPS* sps = slice->getSPS();
  Picture*   pic = slice->getPic();

  // setup coding structure
  CodingStructure& cs = *pic->cs;
  cs.chromaQpAdj      = 0;

  const int       startCtuTsAddr              = slice->getFirstCtuRsAddrInSlice();
  const unsigned  widthInCtus                 = cs.pcv->widthInCtus;
  const bool      wavefrontsEnabled           = cs.sps->getEntropyCodingSyncEnabledFlag();
  const bool      entryPointPresent           = cs.sps->getEntryPointsPresentFlag();

  AdaptiveLoopFilter::reconstructCoeffAPSs( *slice );

  CABACReader cabacReader;
  cabacReader.initBitstream( ppcSubstreams[0].get() );
  cabacReader.initCtxModels( *slice );

  // if first slice, finish init of the coding structure
  if( startCtuTsAddr == 0 )
  {
    cs.initStructData();
  }

  // Quantization parameter
  int prevQP[2] = { slice->getSliceQp(), slice->getSliceQp() };
  CHECK( prevQP[0] == std::numeric_limits<int>::max(), "Invalid previous QP" );

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
    const Position pos( ctuXPosInCtus*maxCUSize, ctuYPosInCtus*maxCUSize) ;
    const UnitArea ctuArea( cs.area.chromaFormat, Area( pos.x, pos.y, maxCUSize, maxCUSize ) );

    // these checks only work, since we wait for the previous slice to finish parsing
    CHECK( ctuXPosInCtus > 0 && cs.getCtuData( ctuRsAddr - 1           ).slice == nullptr, "CTU left not available RS:" << ctuRsAddr - 1 );
    CHECK( ctuYPosInCtus > 0 && cs.getCtuData( ctuRsAddr - widthInCtus ).slice == nullptr, "CTU above not available RS:" << ctuRsAddr - widthInCtus );

    DTRACE_UPDATE( g_trace_ctx, std::make_pair( "ctu", ctuRsAddr ) );

    cabacReader.initBitstream( ppcSubstreams[subStrmId].get() );

    // set up CABAC contexts' state for this CTU
    if( ctuXPosInCtus == tileXPosInCtus && ctuYPosInCtus == tileYPosInCtus )
    {
      if( ctuIdx != 0 ) // if it is the first CTU, then the entropy coder has already been reset
      {
        cabacReader.initCtxModels( *slice );
      }
      prevQP[0] = prevQP[1] = slice->getSliceQp();
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
        cabacReader.setCtx( m_entropyCodingSyncContextState[threadId] );
      }
      prevQP[0] = prevQP[1] = slice->getSliceQp();
    }

    //memset( cs.getCtuData( ctuRsAddr ).cuPtr, 0, sizeof( CtuData::cuPtr ) );
    CtuData& ctuData = cs.getCtuData( ctuRsAddr );
    ctuData.slice = slice;
    ctuData.pps   = slice->getPPS();
    ctuData.sps   = slice->getSPS();
    ctuData.ph    = slice->getPicHeader();

    cabacReader.coding_tree_unit( cs, slice, ctuArea, prevQP, ctuRsAddr );

    if( ctuXPosInCtus == tileXPosInCtus && wavefrontsEnabled )
    {
      m_entropyCodingSyncContextState[threadId] = cabacReader.getCtx();
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

      if( entryPointPresent )
      {
#if DECODER_CHECK_SUBSTREAM_AND_SLICE_TRAILING_BYTES
        cabacReader.remaining_bytes( true );
#endif
        subStrmId++;
      }
    }

#if RECO_WHILE_PARSE
    pic->ctuParsedBarrier[ctuRsAddr].unlock();
#endif

    if( ctuRsAddr + 1 == pic->cs->pcv->sizeInCtus )
    {
      Picture::PicStateEnum expected = Picture::parsing;
      pic->progress.compare_exchange_strong( expected, Picture::parsed );   // if RECO_WHILE_PARSE reconstruction can already have started, so we make sure to not overwrite that state

      pic->parseDone.unlock();
    }
  }
}

}
