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

/** \file     SampleAdaptiveOffset.cpp
    \brief    sample adaptive offset class
*/

#include "SampleAdaptiveOffset.h"

#include "UnitTools.h"
#include "UnitPartitioner.h"
#include "CodingStructure.h"
#include "CommonLib/dtrace_codingstruct.h"
#include "CommonLib/dtrace_buffer.h"
#include "CommonLib/TimeProfiler.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

namespace vvdec
{

void SampleAdaptiveOffset::offsetBlock_core( const int            channelBitDepth,
                                             const ClpRng&        clpRng,
                                             int                  typeIdx,
                                             int*                 offset,
                                             int                  startIdx,
                                             const Pel*           srcBlk,
                                             Pel*                 resBlk,
                                             ptrdiff_t            srcStride,
                                             ptrdiff_t            resStride,
                                             int                  width,
                                             int                  height,
                                             bool                 isLeftAvail,
                                             bool                 isRightAvail,
                                             bool                 isAboveAvail,
                                             bool                 isBelowAvail,
                                             bool                 isAboveLeftAvail,
                                             bool                 isAboveRightAvail,
                                             bool                 isBelowLeftAvail,
                                             bool                 isBelowRightAvail,
                                             std::vector<int8_t>* m_signLineBuf1,
                                             std::vector<int8_t>* m_signLineBuf2,
                                             bool                 isCtuCrossedByVirtualBoundaries,
                                             int                  horVirBndryPos[],
                                             int                  verVirBndryPos[],
                                             int                  numHorVirBndry,
                                             int                  numVerVirBndry )
{
  int x,y, startX, startY, endX, endY, edgeType;
  int firstLineStartX, firstLineEndX, lastLineStartX, lastLineEndX;
  int8_t signLeft, signRight, signDown;


  const Pel* srcLine = srcBlk;
  Pel* resLine = resBlk;

  switch(typeIdx)
  {
  case SAO_TYPE_EO_0:
  {
    offset += 2;
    startX = isLeftAvail ? 0 : 1;
    endX   = isRightAvail ? width : (width -1);
    for (y=0; y< height; y++)
    {
      signLeft = (int8_t)sgn(srcLine[startX] - srcLine[startX-1]);
      for (x=startX; x< endX; x++)
      {
        signRight = (int8_t)sgn(srcLine[x] - srcLine[x+1]);
        if( isCtuCrossedByVirtualBoundaries && isProcessDisabled( x, y, numVerVirBndry, 0, verVirBndryPos, horVirBndryPos ) )
        {
          signLeft = -signRight;
          continue;
        }
        edgeType =  signRight + signLeft;
        signLeft  = -signRight;

        resLine[x] = ClipPel<int>( srcLine[x] + offset[edgeType], clpRng);
      }

      srcLine  += srcStride;
      resLine += resStride;
    }
  }
  break;

  case SAO_TYPE_EO_90:
  {
    offset += 2;
    int8_t *signUpLine = &m_signLineBuf1->front();

    startY = isAboveAvail ? 0 : 1;
    endY   = isBelowAvail ? height : height-1;
    if (!isAboveAvail)
    {
      srcLine += srcStride;
      resLine += resStride;
    }

    const Pel* srcLineAbove= srcLine- srcStride;
    for (x=0; x< width; x++)
    {
      signUpLine[x] = (int8_t)sgn(srcLine[x] - srcLineAbove[x]);
    }

    const Pel* srcLineBelow;
    for (y=startY; y<endY; y++)
    {
      srcLineBelow= srcLine+ srcStride;

      for (x=0; x< width; x++)
      {
        signDown  = (int8_t)sgn(srcLine[x] - srcLineBelow[x]);
        if( isCtuCrossedByVirtualBoundaries && isProcessDisabled( x, y, 0, numHorVirBndry, verVirBndryPos, horVirBndryPos ) )
        {
          signUpLine[x] = -signDown;
          continue;
        }
        edgeType = signDown + signUpLine[x];
        signUpLine[x]= -signDown;

        resLine[x] = ClipPel<int>(srcLine[x] + offset[edgeType], clpRng);
      }
      srcLine += srcStride;
      resLine += resStride;
    }
  }
  break;
  case SAO_TYPE_EO_135:
  {
    offset += 2;
    int8_t *signUpLine, *signDownLine, *signTmpLine;

    signUpLine  = &m_signLineBuf1->front();
    signDownLine= &m_signLineBuf2->front();

    startX = isLeftAvail ? 0 : 1 ;
    endX   = isRightAvail ? width : (width-1);

    //prepare 2nd line's upper sign
    const Pel* srcLineBelow= srcLine+ srcStride;
    for (x=startX; x< endX+1; x++)
    {
      signUpLine[x] = (int8_t)sgn(srcLineBelow[x] - srcLine[x- 1]);
    }

    //1st line
    const Pel* srcLineAbove= srcLine- srcStride;
    firstLineStartX = isAboveLeftAvail ? 0 : 1;
    firstLineEndX   = isAboveAvail? endX: 1;
    for(x= firstLineStartX; x< firstLineEndX; x++)
    {
      if( isCtuCrossedByVirtualBoundaries && isProcessDisabled( x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos ) )
      {
        continue;
      }
      edgeType  =  sgn(srcLine[x] - srcLineAbove[x- 1]) - signUpLine[x+1];

      resLine[x] = ClipPel<int>( srcLine[x] + offset[edgeType], clpRng);
    }
    srcLine  += srcStride;
    resLine  += resStride;


    //middle lines
    for (y= 1; y< height-1; y++)
    {
      srcLineBelow= srcLine+ srcStride;

      for (x=startX; x<endX; x++)
      {
        signDown =  (int8_t)sgn(srcLine[x] - srcLineBelow[x+ 1]);
        if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
        {
          signDownLine[x + 1] = -signDown;
          continue;
        }
        edgeType =  signDown + signUpLine[x];
        resLine[x] = ClipPel<int>( srcLine[x] + offset[edgeType], clpRng);

        signDownLine[x+1] = -signDown;
      }
      signDownLine[startX] = (int8_t)sgn(srcLineBelow[startX] - srcLine[startX-1]);

      signTmpLine  = signUpLine;
      signUpLine   = signDownLine;
      signDownLine = signTmpLine;

      srcLine += srcStride;
      resLine += resStride;
    }

    //last line
    srcLineBelow= srcLine+ srcStride;
    lastLineStartX = isBelowAvail ? startX : (width -1);
    lastLineEndX   = isBelowRightAvail ? width : (width -1);
    for(x= lastLineStartX; x< lastLineEndX; x++)
    {
      if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, height - 1, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
      {
        continue;
      }
      edgeType =  sgn(srcLine[x] - srcLineBelow[x+ 1]) + signUpLine[x];
      resLine[x] = ClipPel<int>( srcLine[x] + offset[edgeType], clpRng);
    }
  }
  break;
  case SAO_TYPE_EO_45:
  {
    offset += 2;
    int8_t *signUpLine = &m_signLineBuf1->at(1);

    startX = isLeftAvail ? 0 : 1;
    endX   = isRightAvail ? width : (width -1);

    //prepare 2nd line upper sign
    const Pel* srcLineBelow= srcLine+ srcStride;
    for (x=startX-1; x< endX; x++)
    {
      signUpLine[x] = (int8_t)sgn(srcLineBelow[x] - srcLine[x+1]);
    }


    //first line
    const Pel* srcLineAbove= srcLine- srcStride;
    firstLineStartX = isAboveAvail ? startX : (width -1 );
    firstLineEndX   = isAboveRightAvail ? width : (width-1);
    for(x= firstLineStartX; x< firstLineEndX; x++)
    {
      if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
      {
        continue;
      }
      edgeType = sgn(srcLine[x] - srcLineAbove[x+1]) -signUpLine[x-1];
      resLine[x] = ClipPel<int>(srcLine[x] + offset[edgeType], clpRng);
    }
    srcLine += srcStride;
    resLine += resStride;

    //middle lines
    for (y= 1; y< height-1; y++)
    {
      srcLineBelow= srcLine+ srcStride;

      for(x= startX; x< endX; x++)
      {
        signDown =  (int8_t)sgn(srcLine[x] - srcLineBelow[x-1]);
        if( isCtuCrossedByVirtualBoundaries && isProcessDisabled( x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos ) )
        {
          signUpLine[x - 1] = -signDown;
          continue;
        }
        edgeType =  signDown + signUpLine[x];
        resLine[x] = ClipPel<int>(srcLine[x] + offset[edgeType], clpRng);
        signUpLine[x-1] = -signDown;
      }
      signUpLine[endX-1] = (int8_t)sgn(srcLineBelow[endX-1] - srcLine[endX]);
      srcLine  += srcStride;
      resLine += resStride;
    }

    //last line
    srcLineBelow= srcLine+ srcStride;
    lastLineStartX = isBelowLeftAvail ? 0 : 1;
    lastLineEndX   = isBelowAvail ? endX : 1;
    for(x= lastLineStartX; x< lastLineEndX; x++)
    {
      if( isCtuCrossedByVirtualBoundaries && isProcessDisabled( x, height - 1, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos ) )
      {
        continue;
      }
      edgeType = sgn(srcLine[x] - srcLineBelow[x-1]) + signUpLine[x];
      resLine[x] = ClipPel<int>(srcLine[x] + offset[edgeType], clpRng);

    }
  }
  break;
  case SAO_TYPE_BO:
  {
    const int shiftBits = channelBitDepth - NUM_SAO_BO_CLASSES_LOG2;

    for (y=0; y< height; y++)
    {
      for (x=0; x< width; x++)
      {
        resLine[x] = ClipPel<int>(srcLine[x] + offset[srcLine[x] >> shiftBits], clpRng );
      }
      srcLine += srcStride;
      resLine += resStride;
    }
  }
  break;
  default:
  {
    THROW_FATAL( "Not a supported SAO types\n" );
  }
  }
}

SampleAdaptiveOffset::~SampleAdaptiveOffset()
{
  destroy();
}

void SampleAdaptiveOffset::create( int picWidth, int picHeight, ChromaFormat format, uint32_t maxCUWidth, uint32_t maxCUHeight, uint32_t maxCUDepth, uint32_t bitShift, PelUnitBuf& unitBuf )
{
  offsetBlock = offsetBlock_core;
#if ENABLE_SIMD_OPT_SAO && defined( TARGET_SIMD_X86 )
  initSampleAdaptiveOffsetX86();
#endif

  m_tempBuf = unitBuf;

  //bit-depth related
  m_offsetStepLog2 = bitShift;
  m_numberOfComponents = getNumberValidComponents(format);
}

void SampleAdaptiveOffset::destroy()
{
}

#if 0
void SampleAdaptiveOffset::SAOProcess( CodingStructure& cs )
{
  const PreCalcValues& pcv = *cs.pcv;
  for( uint32_t yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight )
  {
    for( uint32_t xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth )
    {
      const uint32_t width  = (xPos + pcv.maxCUWidth  > pcv.lumaWidth)  ? (pcv.lumaWidth - xPos)  : pcv.maxCUWidth;
      const uint32_t height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;

      SAOProcessCTU( cs, UnitArea(cs.area.chromaFormat, Area(xPos , yPos, width, height)) );
    }
  }

  DTRACE_UPDATE(g_trace_ctx, (std::make_pair("poc", cs.slice->getPOC())));
  DTRACE_PIC_COMP(D_REC_CB_LUMA_SAO, cs, cs.getRecoBuf(), COMPONENT_Y);
  DTRACE_PIC_COMP(D_REC_CB_CHROMA_SAO, cs, cs.getRecoBuf(), COMPONENT_Cb);
  DTRACE_PIC_COMP(D_REC_CB_CHROMA_SAO, cs, cs.getRecoBuf(), COMPONENT_Cr);

  DTRACE    ( g_trace_ctx, D_CRC, "SAO" );
  DTRACE_CRC( g_trace_ctx, D_CRC, cs, cs.getRecoBuf() );
}
#endif

void SampleAdaptiveOffset::SAOPrepareCTULine( CodingStructure &cs, const UnitArea &lineArea )
{
  PROFILER_SCOPE_AND_STAGE( 1, g_timeProfiler, P_SAO );

  const PreCalcValues& pcv = *cs.pcv;
  PelUnitBuf           rec = cs.getRecoBuf();

  const int height = lineArea.lumaSize().height;
  const int y      = lineArea.lumaPos().y;

  const int cpyY       = y == 0 ? 0 : y + 2;
        int cpyHeight  = height;
            cpyHeight -= y == 0 ? 0 : 2;
            cpyHeight -= cpyY + cpyHeight >= pcv.lumaHeight ? 0 : 2;
            cpyHeight  = std::min<int>( cpyHeight, pcv.lumaHeight - cpyY );

  for( int x = 0; x < pcv.lumaWidth; x += pcv.maxCUWidth )
  {
    const int ctuRsAddr = getCtuAddr( Position( x, y ), *cs.pcv );

    SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES] = { nullptr, nullptr };
    getMergeList( cs, ctuRsAddr, mergeList );

    reconstructBlkSAOParam( cs.m_ctuData[ctuRsAddr].saoParam, mergeList );
  }

  const int numComp = getNumberValidComponents( pcv.chrFormat );

  for( int i = 0; i < numComp; i++ )
  {
    const ComponentID compID = ComponentID( i );

    const int currCpyY =   cpyY            >> getComponentScaleY( compID, pcv.chrFormat );
    const int currCpyH =   cpyHeight       >> getComponentScaleY( compID, pcv.chrFormat );
    const int currCtuW =   pcv.maxCUWidth  >> getComponentScaleX( compID, pcv.chrFormat );
    const int currPicW =   pcv.lumaWidth   >> getComponentScaleX( compID, pcv.chrFormat );
    
    const ptrdiff_t src_stride =       rec.bufs[ compID ].stride;
    const ptrdiff_t dst_stride = m_tempBuf.bufs[ compID ].stride;

    const Pel* src =       rec.bufs[ compID ].bufAt( 0, currCpyY );
          Pel* dst = m_tempBuf.bufs[ compID ].bufAt( 0, currCpyY );

    for( int j = currCpyY; j < ( currCpyY + currCpyH ); j++, src += src_stride, dst += dst_stride )
    {
      int numCpy   = 0;
      int cpyStart = 0;

      int ctuRsAddr = getCtuAddr( Position( 0, y ), *cs.pcv );

      for( int x = 0, currCpyX = 0; x <= pcv.widthInCtus; x++, currCpyX += currCtuW, ctuRsAddr++ )
      {
        if( x < pcv.widthInCtus && cs.m_ctuData[ctuRsAddr].saoParam[i].modeIdc != SAO_MODE_OFF )
        {
          if( !numCpy ) cpyStart = currCpyX - !!x;
          numCpy++;
        }
        else if( numCpy )
        {
          // copy
          int cpyLen = numCpy * currCtuW + !!cpyStart + 1;
          if( cpyStart + cpyLen > currPicW ) cpyLen = currPicW - cpyStart;
          memcpy( dst + cpyStart, src + cpyStart, cpyLen * sizeof( Pel ) );
          numCpy = 0;
        }
      }
    }
  }

  const int cpyY2      = cpyY + cpyHeight;
  const int cpyHeight2 = 4;
  
  if( cpyY2 < pcv.lumaHeight )
  {
    const UnitArea cpyLine2( pcv.chrFormat, Area( 0, cpyY2, pcv.lumaWidth, cpyHeight2 ) );
  
    m_tempBuf.subBuf( cpyLine2 ).copyFrom( rec.subBuf( cpyLine2 ) );
  }
}

void SampleAdaptiveOffset::SAOProcessCTULine( CodingStructure &cs, const UnitArea &lineArea )
{
  PROFILER_SCOPE_AND_STAGE( 1, g_timeProfiler, P_SAO );

  const PreCalcValues& pcv = *cs.pcv;
  PelUnitBuf           rec = cs.getRecoBuf();

  const int height         = lineArea.lumaSize().height;
  const int y              = lineArea.lumaPos().y;
  
  bool anySaoBlk = false;
  
  for( int x = 0; x < pcv.lumaWidth; x += pcv.maxCUWidth )
  {
    const int ctuRsAddr = getCtuAddr( Position( x, y ), *cs.pcv );

    for( int i = 0; i < MAX_NUM_COMPONENT; i++ )
    {
      if( cs.m_ctuData[ctuRsAddr].saoParam[i].modeIdc != SAO_MODE_OFF )
      {
        anySaoBlk = true;
      }
    }
  }

  if( !anySaoBlk ) return;

  std::vector<int8_t> signLineBuf1;
  std::vector<int8_t> signLineBuf2;

  for( int x = 0; x < pcv.lumaWidth; x += pcv.maxCUWidth )
  {
    const int width = ( x + pcv.maxCUWidth > pcv.lumaWidth ) ? ( pcv.lumaWidth - x ) : pcv.maxCUWidth;

    const UnitArea ctuArea( pcv.chrFormat, Area( x, y, width, height ) );

    const int ctuRsAddr = getCtuAddr( ctuArea.lumaPos(), *cs.pcv );

    offsetCTU( ctuArea, m_tempBuf, rec, cs.m_ctuData[ctuRsAddr].saoParam, cs, signLineBuf1, signLineBuf2 );
  }
}

void SampleAdaptiveOffset::SAOProcessCTU( CodingStructure &cs, const UnitArea &ctuArea )
{
  PROFILER_SCOPE_AND_STAGE( 1, g_timeProfiler, P_SAO );

  PelUnitBuf           rec = cs.getRecoBuf();

  const int ctuRsAddr = getCtuAddr( ctuArea.lumaPos(), *cs.pcv );

  bool anySaoBlk = false;

  for( int i = 0; i < MAX_NUM_COMPONENT; i++ )
  {
    if( cs.m_ctuData[ctuRsAddr].saoParam[i].modeIdc != SAO_MODE_OFF )
    {
      anySaoBlk = true;
    }
  }

  if( !anySaoBlk ) return;

  std::vector<int8_t> signLineBuf1;
  std::vector<int8_t> signLineBuf2;

  offsetCTU( ctuArea, m_tempBuf, rec, cs.m_ctuData[ctuRsAddr].saoParam, cs, signLineBuf1, signLineBuf2 );
}

void SampleAdaptiveOffset::invertQuantOffsets(ComponentID compIdx, int typeIdc, int typeAuxInfo, int* dstOffsets, int* srcOffsets) const
{
  int codedOffset[MAX_NUM_SAO_CLASSES];

  ::memcpy(codedOffset, srcOffsets, sizeof(int)*MAX_NUM_SAO_CLASSES);
  ::memset(dstOffsets, 0, sizeof(int)*MAX_NUM_SAO_CLASSES);

  if(typeIdc == SAO_TYPE_START_BO)
  {
    for(int i=0; i< 4; i++)
    {
      dstOffsets[(typeAuxInfo+ i)%NUM_SAO_BO_CLASSES] = codedOffset[(typeAuxInfo+ i)%NUM_SAO_BO_CLASSES]*(1<<m_offsetStepLog2);
    }
  }
  else //EO
  {
    for(int i=0; i< NUM_SAO_EO_CLASSES; i++)
    {
      dstOffsets[i] = codedOffset[i] *(1<<m_offsetStepLog2);
    }
    CHECK(dstOffsets[SAO_CLASS_EO_PLAIN] != 0, "EO offset is not '0'"); //keep EO plain offset as zero
  }

}

int SampleAdaptiveOffset::getMergeList(CodingStructure& cs, int ctuRsAddr, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES])
{
  const PreCalcValues& pcv = *cs.pcv;

  const int ctuX       = ctuRsAddr % pcv.widthInCtus;
  const int ctuY       = ctuRsAddr / pcv.widthInCtus;
  const CodingUnit& cu = *cs.getCtuData( ctuRsAddr ).cuPtr[CH_L][0];
  int mergedCTUPos;
  int numValidMergeCandidates = 0;

  for( auto mergeType: { SAO_MERGE_LEFT, SAO_MERGE_ABOVE } )
  {
    SAOBlkParam* mergeCandidate = NULL;

    switch(mergeType)
    {
    case SAO_MERGE_ABOVE:
      if( ctuY > 0 )
      {
        mergedCTUPos = ctuRsAddr - pcv.widthInCtus;
        if( cu.above )
        {
          mergeCandidate = &( cs.m_ctuData[mergedCTUPos].saoParam );
        }
      }
      break;
    case SAO_MERGE_LEFT:
      if( ctuX > 0 )
      {
        mergedCTUPos = ctuRsAddr - 1;
        if( cu.left )
        {
          mergeCandidate = &( cs.m_ctuData[mergedCTUPos].saoParam );
        }
      }
      break;
    default:
      THROW_FATAL( "not a supported merge type" );
    }

    mergeList[mergeType] = mergeCandidate;
    if (mergeCandidate != NULL)
    {
      numValidMergeCandidates++;
    }
  }

  return numValidMergeCandidates;
}


void SampleAdaptiveOffset::reconstructBlkSAOParam(SAOBlkParam& recParam, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES]) const
{
  const int numberOfComponents = m_numberOfComponents;
  for(int compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    const ComponentID component = ComponentID(compIdx);
    SAOOffset& offsetParam = recParam[component];

    if(offsetParam.modeIdc == SAO_MODE_OFF)
    {
      continue;
    }

    switch(offsetParam.modeIdc)
    {
    case SAO_MODE_NEW:
      {
        invertQuantOffsets(component, offsetParam.typeIdc, offsetParam.typeAuxInfo, offsetParam.offset, offsetParam.offset);
      }
      break;
    case SAO_MODE_MERGE:
      {
        const SAOBlkParam* mergeTarget = mergeList[offsetParam.typeIdc];
        CHECK(mergeTarget == NULL, "Merge target does not exist");

        offsetParam = (*mergeTarget)[component];
      }
      break;
    default:
      {
        THROW_RECOVERABLE( "Not a supported mode" );
      }
    }
  }
}


void SampleAdaptiveOffset::offsetCTU( const UnitArea& area, const CPelUnitBuf& src, PelUnitBuf& res, SAOBlkParam& saoblkParam, CodingStructure& cs, std::vector<int8_t> &signLineBuf1, std::vector<int8_t> &signLineBuf2 )
{
  const uint32_t numberOfComponents = getNumberValidComponents( area.chromaFormat );
  if( std::all_of( &saoblkParam[0],
                   &saoblkParam[numberOfComponents],
                   [](const SAOOffset & p){ return p.modeIdc == SAO_MODE_OFF; } ) )
  {
    return;
  }

  bool isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail, isBelowLeftAvail, isBelowRightAvail;

  // block boundary availability
  deriveLoopFilterBoundaryAvailibility( cs, area.Y(), isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail, isBelowLeftAvail, isBelowRightAvail );

  const size_t lineBufferSize = area.Y().width + 1;
  if (signLineBuf1.size() < lineBufferSize)
  {
    signLineBuf1.resize(lineBufferSize);
    signLineBuf2.resize(lineBufferSize);
  }

  int  numHorVirBndry       = 0;
  int  numVerVirBndry       = 0;
  int  horVirBndryPos[]     = { -1, -1, -1 };
  int  verVirBndryPos[]     = { -1, -1, -1 };
  int  horVirBndryPosComp[] = { -1, -1, -1 };
  int  verVirBndryPosComp[] = { -1, -1, -1 };
  bool isCtuCrossedByVirtualBoundaries = isCrossedByVirtualBoundaries( cs.picHeader.get(),
                                                                       area.Y(),
                                                                       numHorVirBndry, numVerVirBndry,
                                                                       horVirBndryPos, verVirBndryPos );
  if( isCtuCrossedByVirtualBoundaries )
  {
    CHECK( numHorVirBndry >= (int)( sizeof(horVirBndryPos) / sizeof(horVirBndryPos[0]) ), "Too many virtual boundaries" );
    CHECK( numHorVirBndry >= (int)( sizeof(verVirBndryPos) / sizeof(verVirBndryPos[0]) ), "Too many virtual boundaries" );
  }

  for(int compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    const ComponentID compID = ComponentID(compIdx);
    const CompArea& compArea = area.block(compID);
    SAOOffset& ctbOffset     = saoblkParam[compIdx];

    if(ctbOffset.modeIdc != SAO_MODE_OFF)
    {
      ptrdiff_t  srcStride = src.get(compID).stride;
      const Pel* srcBlk    = src.get(compID).bufAt(compArea);
      ptrdiff_t  resStride = res.get(compID).stride;
      Pel* resBlk          = res.get(compID).bufAt(compArea);

      for (int i = 0; i < numHorVirBndry; i++)
      {
        horVirBndryPosComp[i] = (horVirBndryPos[i] >> getComponentScaleY(compID, area.chromaFormat)) - compArea.y;
      }
      for (int i = 0; i < numVerVirBndry; i++)
      {
        verVirBndryPosComp[i] = (verVirBndryPos[i] >> getComponentScaleX(compID, area.chromaFormat)) - compArea.x;
      }

      offsetBlock( cs.sps->getBitDepth(),
                   cs.getCtuData(cs.ctuRsAddr(area.Y().pos(), CH_L)).cuPtr[0][0]->slice->clpRng(compID),
                   ctbOffset.typeIdc, ctbOffset.offset,ctbOffset.typeAuxInfo
                  , srcBlk, resBlk, srcStride, resStride, compArea.width, compArea.height
                  , isLeftAvail, isRightAvail
                  , isAboveAvail, isBelowAvail
                  , isAboveLeftAvail, isAboveRightAvail
                  , isBelowLeftAvail, isBelowRightAvail
                  , &signLineBuf1
                  , &signLineBuf2
                  , isCtuCrossedByVirtualBoundaries
                  , horVirBndryPosComp
                  , verVirBndryPosComp
                  , numHorVirBndry
                  , numVerVirBndry
                  );
    }
  } //compIdx
}

void SampleAdaptiveOffset::deriveLoopFilterBoundaryAvailibility( CodingStructure& cs,
                                                                 const Position&  pos,
                                                                 bool&            isLeftAvail,
                                                                 bool&            isRightAvail,
                                                                 bool&            isAboveAvail,
                                                                 bool&            isBelowAvail,
                                                                 bool&            isAboveLeftAvail,
                                                                 bool&            isAboveRightAvail,
                                                                 bool&            isBelowLeftAvail,
                                                                 bool&            isBelowRightAvail ) const
{
  const int ctusz  = cs.pcv->maxCUWidth;
  const int ctuX   = pos.x / ctusz;
  const int ctuY   = pos.y / ctusz;
  const int width  = cs.pcv->widthInCtus;
  const int height = cs.pcv->heightInCtus;

  const CodingUnit* cuCurr        =                       cs.getCtuData( ctuX,     ctuY     ).cuPtr[0][0];
  const CodingUnit* cuLeft        = ctuX     > 0        ? cs.getCtuData( ctuX - 1, ctuY     ).cuPtr[0][0] : nullptr;
  const CodingUnit* cuRight       = ctuX + 1 < width    ? cs.getCtuData( ctuX + 1, ctuY     ).cuPtr[0][0] : nullptr;
  const CodingUnit* cuAbove       = ctuY     > 0        ? cs.getCtuData( ctuX,     ctuY - 1 ).cuPtr[0][0] : nullptr;
  const CodingUnit* cuBelow       = ctuY + 1 < height   ? cs.getCtuData( ctuX,     ctuY + 1 ).cuPtr[0][0] : nullptr;
  const CodingUnit* cuAboveLeft   = cuLeft  && cuAbove  ? cs.getCtuData( ctuX - 1, ctuY - 1 ).cuPtr[0][0] : nullptr;
  const CodingUnit* cuAboveRight  = cuRight && cuAbove  ? cs.getCtuData( ctuX + 1, ctuY - 1 ).cuPtr[0][0] : nullptr;
  const CodingUnit* cuBelowLeft   = cuLeft  && cuBelow  ? cs.getCtuData( ctuX - 1, ctuY + 1 ).cuPtr[0][0] : nullptr;
  const CodingUnit* cuBelowRight  = cuRight && cuBelow  ? cs.getCtuData( ctuX + 1, ctuY + 1 ).cuPtr[0][0] : nullptr;

  isLeftAvail       = (cuLeft       != NULL);
  isAboveAvail      = (cuAbove      != NULL);
  isRightAvail      = (cuRight      != NULL);
  isBelowAvail      = (cuBelow      != NULL);
  isAboveLeftAvail  = (cuAboveLeft  != NULL);
  isAboveRightAvail = (cuAboveRight != NULL);
  isBelowLeftAvail  = (cuBelowLeft  != NULL);
  isBelowRightAvail = (cuBelowRight != NULL);

  // check cross slice flags
  const bool isLoopFilterAcrossSlicePPS = cs.pps->getLoopFilterAcrossSlicesEnabledFlag();
  if (!isLoopFilterAcrossSlicePPS)
  {
    isLeftAvail       = isLeftAvail       && CU::isSameSlice(*cuCurr, *cuLeft);
    isAboveAvail      = isAboveAvail      && CU::isSameSlice(*cuCurr, *cuAbove);
    isRightAvail      = isRightAvail      && CU::isSameSlice(*cuCurr, *cuRight);
    isBelowAvail      = isBelowAvail      && CU::isSameSlice(*cuCurr, *cuBelow);
    isAboveLeftAvail  = isAboveLeftAvail  && CU::isSameSlice(*cuCurr, *cuAboveLeft);
    isAboveRightAvail = isAboveRightAvail && CU::isSameSlice(*cuCurr, *cuAboveRight);
    isBelowLeftAvail  = isBelowLeftAvail  && CU::isSameSlice(*cuCurr, *cuBelowLeft);
    isBelowRightAvail = isBelowRightAvail && CU::isSameSlice(*cuCurr, *cuBelowRight);
  }

  // check cross tile flags
  const bool isLoopFilterAcrossTilePPS = cs.pps->getLoopFilterAcrossTilesEnabledFlag();
  if (!isLoopFilterAcrossTilePPS)
  {
    isLeftAvail       = isLeftAvail       && CU::isSameTile(*cuCurr, *cuLeft);
    isAboveAvail      = isAboveAvail      && CU::isSameTile(*cuCurr, *cuAbove);
    isRightAvail      = isRightAvail      && CU::isSameTile(*cuCurr, *cuRight);
    isBelowAvail      = isBelowAvail      && CU::isSameTile(*cuCurr, *cuBelow);
    isAboveLeftAvail  = isAboveLeftAvail  && CU::isSameTile(*cuCurr, *cuAboveLeft);
    isAboveRightAvail = isAboveRightAvail && CU::isSameTile(*cuCurr, *cuAboveRight);
    isBelowLeftAvail  = isBelowLeftAvail  && CU::isSameTile(*cuCurr, *cuBelowLeft);
    isBelowRightAvail = isBelowRightAvail && CU::isSameTile(*cuCurr, *cuBelowRight);
  }

  // check cross subpic flags
  if( cs.sps->getSubPicInfoPresentFlag() )
  {
    const SubPic& curSubPic = cs.pps->getSubPicFromCU(*cuCurr);
    if( !curSubPic.getloopFilterAcrossSubPicEnabledFlag() )
    {
      isLeftAvail       = isLeftAvail       && CU::isSameSubPic(*cuCurr, *cuLeft);
      isAboveAvail      = isAboveAvail      && CU::isSameSubPic(*cuCurr, *cuAbove);
      isRightAvail      = isRightAvail      && CU::isSameSubPic(*cuCurr, *cuRight);
      isBelowAvail      = isBelowAvail      && CU::isSameSubPic(*cuCurr, *cuBelow);
      isAboveLeftAvail  = isAboveLeftAvail  && CU::isSameSubPic(*cuCurr, *cuAboveLeft);
      isAboveRightAvail = isAboveRightAvail && CU::isSameSubPic(*cuCurr, *cuAboveRight);
      isBelowLeftAvail  = isBelowLeftAvail  && CU::isSameSubPic(*cuCurr, *cuBelowLeft);
      isBelowRightAvail = isBelowRightAvail && CU::isSameSubPic(*cuCurr, *cuBelowRight);
    }
  }
}

bool SampleAdaptiveOffset::isProcessDisabled( int xPos, int yPos, int numVerVirBndry, int numHorVirBndry, int verVirBndryPos[], int horVirBndryPos[] )
{
  for (int i = 0; i < numVerVirBndry; i++)
  {
    if ((xPos == verVirBndryPos[i]) || (xPos == verVirBndryPos[i] - 1))
    {
      return true;
    }
  }
  for (int i = 0; i < numHorVirBndry; i++)
  {
    if ((yPos == horVirBndryPos[i]) || (yPos == horVirBndryPos[i] - 1))
    {
      return true;
    }
  }

  return false;
}

}
