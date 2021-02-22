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

#pragma once

#include "../../../include/vvdec/vvdec.h"
#include "DecoderLib/DecLib.h"             // internal decoder

namespace vvdec {


static const char * const vvdecNalTypeNames[] = { "NAL_UNIT_CODED_SLICE_TRAIL", "NAL_UNIT_CODED_SLICE_STSA", "NAL_UNIT_CODED_SLICE_RADL", "NAL_UNIT_CODED_SLICE_RASL",
                                                  "NAL_UNIT_RESERVED_VCL_4", "NAL_UNIT_RESERVED_VCL_5", "NAL_UNIT_RESERVED_VCL_6",
                                                  "NAL_UNIT_CODED_SLICE_IDR_W_RADL","NAL_UNIT_CODED_SLICE_IDR_N_LP","NAL_UNIT_CODED_SLICE_CRA","NAL_UNIT_CODED_SLICE_GDR",
                                                  "NAL_UNIT_RESERVED_IRAP_VCL_11","NAL_UNIT_RESERVED_IRAP_VCL_12",
                                                  "NAL_UNIT_DCI","NAL_UNIT_VPS","NAL_UNIT_SPS","NAL_UNIT_PPS",
                                                  "NAL_UNIT_PREFIX_APS","NAL_UNIT_SUFFIX_APS","NAL_UNIT_PH","NAL_UNIT_ACCESS_UNIT_DELIMITER",
                                                  "NAL_UNIT_EOS","NAL_UNIT_EOB","NAL_UNIT_PREFIX_SEI","NAL_UNIT_SUFFIX_SEI","NAL_UNIT_FD",
                                                  "NAL_UNIT_RESERVED_NVCL_26","NAL_UNIT_RESERVED_NVCL_27",
                                                  "NAL_UNIT_UNSPECIFIED_28","NAL_UNIT_UNSPECIFIED_29","NAL_UNIT_UNSPECIFIED_30","NAL_UNIT_UNSPECIFIED_31",
                                                  "NAL_UNIT_INVALID", 0 };

static const char * const vvdecErrorMsg[] = { "expected behavior",
                                              "unspecified malfunction",
                                              "decoder not initialized or tried to initialize multiple times",
                                              "internal allocation error",
                                              "allocated memory to small to receive decoded data",
                                              "inconsistent or invalid parameters",
                                              "unsupported request",
                                              "decoder requires restart",
                                              "unsupported CPU - SSE 4.1 needed",
                                              "more bitstream data needed. try again",
                                              "end of stream",
                                              "unknown error code", 0 };

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

   int init( const vvdec_Params& rcVVDecParameter );
   int uninit();

   void setLoggingCallback(vvdec_loggingCallback callback, void *userData, LogLevel level);

   int decode( vvdec_AccessUnit& rcAccessUnit, vvdec_Frame** ppcFrame );

   int flush( vvdec_Frame** ppcFrame );

   int objectUnref( vvdec_Frame* pcFrame );

   int getNumberOfErrorsPictureHashSEI( );

   int setAndRetErrorMsg( int Ret );

   const char* getDecoderInfo();
//   const char* getDecoderCapabilities( );

   static const char* getErrorMsg( int nRet );
   static const char* getVersionNumber();

   static NalType getNalUnitType            ( vvdec_AccessUnit& rcAccessUnit );
   static const char* getNalUnitTypeAsString( NalType t );

   static bool isNalUnitSlice               ( NalType t );

private:

   int xAddPicture                  ( Picture* pcPic );
   int xCreateFrame                 ( vvdec_Frame& rcFrame, const CPelUnitBuf& rcPicBuf, uint32_t uiWidth, uint32_t uiHeight, const BitDepths& rcBitDepths );
   int xHandleSEIs                  ( vvdec_Frame& rcFrame, Picture* pcPic );

   static int xRetrieveNalStartCode ( unsigned char *pB, int iZerosInStartcode );
   static int xConvertPayloadToRBSP ( std::vector<uint8_t>& nalUnitBuf, InputBitstream *bitstream, bool isVclNalUnit);
   static int xReadNalUnitHeader    ( InputNALUnit& nalu );

   int xHandleOutput( Picture* pcPic );

   static int copyComp( const unsigned char* pucSrc, unsigned char* pucDest, unsigned int uiWidth, unsigned int uiHeight, int iStrideSrc, int iStrideDest, int iBytesPerSample );

public:

   bool                                    m_bInitialized = false;

   DecLib*                                 m_cDecLib;
   bool                                    m_bCreateNewPicBuf    = false;
#if RPR_YUV_OUTPUT
   unsigned int                            m_uiBitDepth = 8;
#endif

   std::list<vvdec_Frame>                  m_rcFrameList;
   std::list<vvdec_Frame>::iterator        m_pcFrameNext = m_rcFrameList.begin();

   std::list<Picture*>                     m_pcLibPictureList; // internal picture list

   std::string                             m_sDecoderInfo;
   std::string                             m_sDecoderCapabilities;

   std::string                             m_cErrorString;
   std::string                             m_cAdditionalErrorString;

   uint64_t                                m_uiSeqNumber       = 0;
   uint64_t                                m_uiSeqNumOutput    = 0;
   uint64_t                                m_uiPicCount        = 0;

  vvdec_loggingCallback                    loggingCallback {};
  void                                     *loggingUserData {};
};


} // namespace

