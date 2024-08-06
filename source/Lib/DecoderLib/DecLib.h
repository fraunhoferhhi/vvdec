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

/** \file     DecLib.h
    \brief    decoder class (header)
*/

#pragma once

#include "PicListManager.h"
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
  DecLibParser             m_decLibParser{ *this, m_picListManager };
  std::list<DecLibRecon>   m_decLibRecon{ 2 };

  std::unique_ptr<ThreadPool> m_decodeThreadPool;

  unsigned int m_parseFrameDelay = 0;

  int         m_decodedPictureHashSEIEnabled   = 0;   ///< Checksum(3)/CRC(2)/MD5(1)/disable(0) acting on decoded picture hash SEI message
  uint32_t    m_numberOfChecksumErrorsDetected = 0;
  std::string m_sDecoderCapabilities;

  int  m_iMaxTemporalLayer  = -1;
  bool m_checkMissingOutput = false;

  std::vector<NalUnitType> m_pictureUnitNals;
  std::list<InputNALUnit>  m_pictureSeiNalus;

public:
  DecLib();
  ~DecLib() = default;

  void create( int numDecThreads, int parserFrameDelay, const UserAllocator& userAllocator, ErrHandlingFlags errHandlingFlags);
  void destroy();

  const char* getDecoderCapabilities() const { return m_sDecoderCapabilities.c_str(); }

  void     setMaxTemporalLayer( int layer ) { m_iMaxTemporalLayer = layer; }

  Picture* decode( InputNALUnit& nalu );
  Picture* flushPic();
  void     releasePicture   ( Picture* pcPic ) { m_picListManager.releasePicture( pcPic ); }   // notify the decoder-lib that the picture buffers can be reused

  void     setDecodedSEIMessageOutputStream( std::ostream* pOpStream ) { m_decLibParser.setDecodedSEIMessageOutputStream( pOpStream ); }
  void     setDecodedPictureHashSEIEnabled( int enabled ) { m_decodedPictureHashSEIEnabled = enabled; }
  uint32_t getNumberOfChecksumErrorsDetected() const { return m_numberOfChecksumErrorsDetected; }
  void     checkPictureHashSEI( Picture* pcPic );

  void     checkNalUnitConstraints( uint32_t naluType );
  void     checkSeiInPictureUnit();
  void     resetPictureSeiNalus();
  void     checkAPSInPictureUnit();
  void     resetPictureUnitNals() { m_pictureUnitNals.clear(); }


  ThreadPool& getThreadPool() { return *m_decodeThreadPool; }

private:
  void     reconPicture( Picture* pcPic );
#if JVET_R0270
  int      finishPicture    ( Picture* pcPic, MsgLevel msgl = NOTICE, bool associatedWithNewClvs );
#else
  int      finishPicture    ( Picture* pcPic, MsgLevel msgl = NOTICE );
#endif
  Picture* getNextOutputPic ( bool bFlush = false );
  void     blockAndFinishPictures( Picture* pcPic = nullptr );   // iterate over DecLibRecon instances and wait to finish picture(s)
  void     xCheckNalUnitConstraintFlags( const ConstraintInfo *cInfo, uint32_t naluType );
};

}
