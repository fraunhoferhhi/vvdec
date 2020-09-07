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

/** \file     DecLib.h
    \brief    decoder class (header)
*/

#ifndef DECLIB_H
#define DECLIB_H

#include "DecLibParser.h"
#include "DecLibRecon.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/Picture.h"

#include "Utilities/NoMallocThreadPool.h"

//! \ingroup DecoderLib
//! \{
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

  std::unique_ptr<NoMallocThreadPool> m_decodeThreadPool;

  unsigned int m_parseFrameDelay = 0;

  int         m_decodedPictureHashSEIEnabled   = 0;   ///< Checksum(3)/CRC(2)/MD5(1)/disable(0) acting on decoded picture hash SEI message
  uint32_t    m_numberOfChecksumErrorsDetected = 0;
  std::string m_sDecoderCapabilities;

  int  m_iMaxTemporalLayer  = -1;
  bool m_checkMissingOutput = false;

  std::vector<NalUnitType> m_pictureUnitNals;
  std::list<InputNALUnit*> m_pictureSeiNalus;

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

  void create( int numDecThreads, int parserFrameDelay );
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

//! \}

#endif   // DECLIB_H
