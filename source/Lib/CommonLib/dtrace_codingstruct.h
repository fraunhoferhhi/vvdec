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

/** \file     dtrace_codingstruct.h
 *  \brief    Easy to use dtrace calls concerning coding structures
 */

#pragma once

#if ENABLE_TRACING

#include "dtrace.h"
#include "dtrace_next.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/CodingStructure.h"
#include "CommonLib/Slice.h"
#include "CommonLib/Mv.h"
#include "CommonLib/Unit.h"
#include "CommonLib/UnitTools.h"

namespace vvdec
{

inline void dtracePicComp( DTRACE_CHANNEL channel, CodingStructure& cs, const CPelUnitBuf& pelUnitBuf, ComponentID compId )
{
  if( !g_trace_ctx ) return;
  if( pelUnitBuf.chromaFormat == CHROMA_400 && compId != COMPONENT_Y )  return;
  const Pel* piSrc    = pelUnitBuf.bufs[compId].buf;
  ptrdiff_t      uiStride = pelUnitBuf.bufs[compId].stride;
  uint32_t       uiWidth  = pelUnitBuf.bufs[compId].width;
  uint32_t       uiHeight = pelUnitBuf.bufs[compId].height;
  uint32_t       uiChromaScaleX = getComponentScaleX( compId, pelUnitBuf.chromaFormat );
  uint32_t       uiChromaScaleY = getComponentScaleY( compId, pelUnitBuf.chromaFormat );

  DTRACE                ( g_trace_ctx, channel, "\n%s: poc = %d, size=%dx%d\n\n", g_trace_ctx->getChannelName(channel), cs.picture->poc, uiWidth, uiHeight );
  DTRACE_FRAME_BLOCKWISE( g_trace_ctx, channel, piSrc, uiStride, uiWidth, uiHeight, cs.sps->getMaxCUWidth() >> uiChromaScaleX, cs.sps->getMaxCUHeight() >> uiChromaScaleY);
}

}   // namespace vvdec

#define DTRACE_PIC_COMP(...)             dtracePicComp( __VA_ARGS__ )
#define DTRACE_STAT(...)                 dtraceComprPicStat(__VA_ARGS__)

#else   // !ENABLE_TRACING

#define DTRACE_STAT( ... )
#define DTRACE_PIC_COMP( ... )

#endif   // !ENABLE_TRACING
