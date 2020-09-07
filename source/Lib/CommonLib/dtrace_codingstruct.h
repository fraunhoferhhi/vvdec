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

/** \file     dtrace_codingstruct.h
 *  \brief    Easy to use dtrace calls concerning coding structures
 */

#ifndef _DTRACE_CODINGSTRUCT_H_
#define _DTRACE_CODINGSTRUCT_H_

#include "dtrace.h"
#include "dtrace_next.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/CodingStructure.h"
#include "CommonLib/Slice.h"
#include "CommonLib/Mv.h"
#include "CommonLib/Unit.h"
#include "CommonLib/UnitTools.h"

#include <cmath>

#if ENABLE_TRACING

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

  DTRACE                ( g_trace_ctx, channel, "\n%s: poc = %d, size=%dx%d\n\n", g_trace_ctx->getChannelName(channel), cs.slice->getPOC(), uiWidth, uiHeight );
  DTRACE_FRAME_BLOCKWISE( g_trace_ctx, channel, piSrc, uiStride, uiWidth, uiHeight, cs.sps->getMaxCUWidth() >> uiChromaScaleX, cs.sps->getMaxCUHeight() >> uiChromaScaleY);
}

#define DTRACE_PIC_COMP(...)             dtracePicComp( __VA_ARGS__ )
#define DTRACE_STAT(...)                 dtraceComprPicStat(__VA_ARGS__)

#else

#define DTRACE_STAT(...)
#define DTRACE_PIC_COMP(...)

#endif

#endif // _DTRACE_HEVC_H_
