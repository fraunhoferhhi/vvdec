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

/** \file     DecLib.h
    \brief    decoder class (header)
*/

#pragma once

#include "DecLibParser.h"
#include "DecLibRecon.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/Picture.h"

#include "Utilities/ThreadPool.h"

namespace vvdec
{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// decoder wrapper class
class DecLib
{
  PicListManager           m_picListManager;
  PicHeader*               m_picHeader = nullptr;
  DecLibParser             m_decLibParser{ *this, m_picListManager, m_picHeader };
  std::list<DecLibRecon>   m_decLibRecon{ 2 };

  std::unique_ptr<ThreadPool> m_decodeThreadPool;

  unsigned int m_parseFrameDelay = 0;
#if RPR_YUV_OUTPUT
  unsigned int m_upscaledOutput  = 0;
#endif

  int         m_decodedPictureHashSEIEnabled   = 0;   ///< Checksum(3)/CRC(2)/MD5(1)/disable(0) acting on decoded picture hash SEI message
  uint32_t    m_numberOfChecksumErrorsDetected = 0;
  std::string m_sDecoderCapabilities;

  int  m_iMaxTemporalLayer  = -1;
  bool m_checkMissingOutput = false;

  std::vector<NalUnitType> m_pictureUnitNals;
  std::list<InputNALUnit>  m_pictureSeiNalus;

public:
#if JVET_Q0044_SLICE_IDX_WITH_SUBPICS
  int m_maxDecSubPicIdx         = 0;
  int m_maxDecSliceAddrInSubPic = -1;
#endif
#if JVET_O1143_SUBPIC_BOUNDARY
  int  m_targetSubPicIdx    = 0;
#endif

  DecLib();
  ~DecLib() = default;

#if RPR_YUV_OUTPUT
  void create( int numDecThreads, int parserFrameDelay, int upscaledOutput );
#else
  void create( int numDecThreads, int parserFrameDelay );
#endif
  void destroy();

  const char* getDecoderCapabilities() const { return m_sDecoderCapabilities.c_str(); }

  void     setTargetDecLayer  ( int layer ) { m_decLibParser.setTargetDecLayer( layer ); }
  void     setMaxTemporalLayer( int layer ) { m_iMaxTemporalLayer = layer; }

#if JVET_P0288_PIC_OUTPUT
  Picture* decode( InputNALUnit& nalu, int* pSkipFrame = nullptr, int iTargetLayer = -1 );
#else
  Picture* decode( InputNALUnit& nalu, int* pSkipFrame = nullptr );
#endif
  Picture* flushPic();
  void     releasePicture   ( Picture* pcPic ) { m_picListManager.releasePicture( pcPic ); }   // notify the decoder-lib that the picture buffers can be reused

  void     setDecodedSEIMessageOutputStream( std::ostream* pOpStream ) { m_decLibParser.setDecodedSEIMessageOutputStream( pOpStream ); }
  void     setDecodedPictureHashSEIEnabled( int enabled ) { m_decodedPictureHashSEIEnabled = enabled; }
  uint32_t getNumberOfChecksumErrorsDetected() const { return m_numberOfChecksumErrorsDetected; }
  void     checkPictureHashSEI( Picture* pcPic );

  void     checkNalUnitConstraints( uint32_t naluType );
  void     checkSeiInPictureUnit();
  void     resetPictureSeiNalus();
  void checkAPSInPictureUnit();
  void resetPictureUnitNals() { m_pictureUnitNals.clear(); }

#if RPR_YUV_OUTPUT
  unsigned int getUpscaledOutput() { return m_upscaledOutput; }
#endif

private:
  void     decompressPicture( Picture* pcPic );
#if JVET_R0270
  int      finishPicture    ( Picture* pcPic, MsgLevel msgl = INFO, bool associatedWithNewClvs );
#else
  int      finishPicture    ( Picture* pcPic, MsgLevel msgl = INFO );
#endif
  Picture* getNextOutputPic ( bool bFlush = false );
  void     xCheckNalUnitConstraintFlags( const ConstraintInfo *cInfo, uint32_t naluType );
};

}
