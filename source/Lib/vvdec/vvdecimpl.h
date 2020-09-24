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

#pragma once

#include "../../../include/vvdec/vvdec.h"
#include "DecoderLib/DecLib.h"             // internal decoder

namespace vvdec {

/**
  \ingroup hhivvcdeclibExternalInterfaces
  The class HhiVvcDec provides the decoder user interface. The simplest way to use the decoder is to call init() to initialize an decoder instance with the
  the given VVCDecoderParameters. After initialization the decoding of the video is performed by using the decoder() method to hand over compressed packets (bitstream chunks) in decoding order
  and retrieve uncompressed pictures. The decoding can be end by calling flush() that causes the decoder to finish decoding of all pending packets.
  Finally calling uninit() releases all allocated resources held by the decoder internally.
*/
class VVDecImpl
{
public:
  /// Constructor
  VVDecImpl();

  /// Destructor
  virtual ~VVDecImpl();

public:

   int init( const VVDecParameter& rcVVDecParameter );
   int uninit();

   int decode( AccessUnit& rcAccessUnit, Frame** ppcFrame );

   int flush( Frame** ppcFrame );

   int objectUnref( Frame* pcFrame );

   int getNumberOfErrorsPictureHashSEI( );

   void clockStartTime();
   void clockEndTime();
   double clockGetTimeDiffMs();

   int setAndRetErrorMsg( int Ret );

   const char* getDecoderInfo();
//   const char* getDecoderCapabilities( );

   static const char* getErrorMsg( int nRet );
   static const char* getVersionNumber();

   static NalType getNalUnitType        ( AccessUnit& rcAccessUnit );
   static const char* getNalUnitTypeAsString( NalType t );

   static bool isNalUnitSlice               ( NalType t );
   static bool isNalUnitSideData            ( NalType t );

private:

   int xAddPicture                  ( Picture* pcPic );
   int xCreateFrame                 ( Frame& rcFrame, const CPelUnitBuf& rcPicBuf, uint32_t uiWidth, uint32_t uiHeight, const BitDepths& rcBitDepths );
   static int xRetrieveNalStartCode ( unsigned char *pB, int iZerosInStartcode );
   static int xConvertPayloadToRBSP ( std::vector<uint8_t>& nalUnitBuf, InputBitstream *bitstream, bool isVclNalUnit);
   static int xReadNalUnitHeader    ( InputNALUnit& nalu );

   int xHandleOutput( Picture* pcPic );

   static int copyComp( const unsigned char* pucSrc, unsigned char* pucDest, unsigned int uiWidth, unsigned int uiHeight, int iStrideSrc, int iStrideDest, int iBytesPerSample );

public:

   bool                                    m_bInitialized = false;

   DecLib                                  m_cDecLib;
   bool                                    m_bCreateNewPicBuf    = false;

   std::list<Frame>                        m_rcFrameList;
   std::list<Frame>::iterator              m_pcFrameNext = m_rcFrameList.begin();

   std::list<Picture*>                     m_pcLibPictureList; // internal picture list

   std::string                             m_sDecoderInfo;
   std::string                             m_sDecoderCapabilities;

   std::string                             m_cErrorString;
   std::string                             m_cAdditionalErrorString;

   uint64_t                                m_uiSeqNumber       = 0;
   uint64_t                                m_uiSeqNumOutput    = 0;
   uint64_t                                m_uiPicCount        = 0;

  static std::string                       m_cTmpErrorString;
  static std::string                       m_cNalType;

  std::chrono::steady_clock::time_point    m_cTPStart;
  std::chrono::steady_clock::time_point    m_cTPEnd;
};


} // namespace

