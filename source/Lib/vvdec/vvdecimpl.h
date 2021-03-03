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

#include "vvdec/vvdec.h"
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
                                              "decoder input data error",
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

  class FrameStorage
  {
  public:
    FrameStorage()  = default;
    ~FrameStorage() = default;

    int allocateStorage( size_t size )
    {
      if( size == 0 ){ return VVDEC_ERR_ALLOCATE; }
      m_ptr = new unsigned char [ size ];
      m_size = size;
      m_isAllocated = true;
      return 0;
    }

    int freeStorage()
    {
      if( !m_isAllocated) { return VVDEC_ERR_ALLOCATE; }
      delete [] m_ptr;
      m_size = 0;
      m_isAllocated = false;
      return 0;
    }

    unsigned char * getStorage()
    {
      if( !m_isAllocated) { return nullptr; }
      return m_ptr;
    }

    bool isAllocated(){ return m_isAllocated; }

  private:
    bool           m_isAllocated = false;
    unsigned char *m_ptr         = nullptr;     // pointer to plane buffer
    size_t         m_size        = 0;
  };

public:

   int init( const vvdecParams& params );
   int uninit();

   void setLoggingCallback( vvdecLoggingCallback callback );

   int decode( vvdecAccessUnit& accessUnit, vvdecFrame** ppframe );

   int flush( vvdecFrame** ppcFrame );

   vvdecSEI* findFrameSei( vvdecSEIPayloadType payloadType, vvdecFrame *frame );

   int objectUnref( vvdecFrame* pframe );

   int getNumberOfErrorsPictureHashSEI( );

   int setAndRetErrorMsg( int Ret, std::string errString = "" );

   const char* getDecoderInfo();
//   const char* getDecoderCapabilities( );

   static const char* getErrorMsg( int nRet );
   static const char* getVersionNumber();

   static vvdecNalType getNalUnitType       ( vvdecAccessUnit& accessUnit );
   static const char* getNalUnitTypeAsString( vvdecNalType t );
   static bool isNalUnitSlice               ( vvdecNalType t );

private:
   int xAddPicture                  ( Picture* pcPic );
   int xCreateFrame                 ( vvdecFrame& frame, const CPelUnitBuf& rcPicBuf, uint32_t uiWidth, uint32_t uiHeight, const BitDepths& rcBitDepths, bool bCreateStorage );

   static int xRetrieveNalStartCode ( unsigned char *pB, int iZerosInStartcode );
   static int xConvertPayloadToRBSP ( std::vector<uint8_t>& nalUnitBuf, InputBitstream *bitstream, bool isVclNalUnit);
   static int xReadNalUnitHeader    ( InputNALUnit& nalu );

   int xHandleOutput( Picture* pcPic );
   bool isFrameConverted( vvdecFrame* frame );

   static int copyComp( const unsigned char* pucSrc, unsigned char* pucDest, unsigned int uiWidth, unsigned int uiHeight, int iStrideSrc, int iStrideDest, int iBytesPerSample );

   void vvdec_picAttributes_default(vvdecPicAttributes *attributes);
   void vvdec_frame_default(vvdecFrame *frame);
   void vvdec_plane_default(vvdecPlane *plane);
   void vvdec_frame_reset(vvdecFrame *frame );

private:
  typedef std::map<uint64_t,FrameStorage>    frameStorageMap;
  typedef frameStorageMap::value_type        frameStorageMapType;
public:
   bool                                    m_bInitialized = false;

   DecLib*                                 m_cDecLib;

   std::list<vvdecFrame>                   m_rcFrameList;
   std::list<vvdecFrame>::iterator         m_pcFrameNext = m_rcFrameList.begin();

   std::list<Picture*>                     m_pcLibPictureList;  // internal picture list
   frameStorageMap                         m_cFrameStorageMap;  // map of frame storage class( converted frames)

   std::string                             m_sDecoderInfo;
   std::string                             m_sDecoderCapabilities;

   std::string                             m_cErrorString;
   std::string                             m_cAdditionalErrorString;

   uint64_t                                m_uiSeqNumber       = 0;
   uint64_t                                m_uiSeqNumOutput    = 0;
   uint64_t                                m_uiPicCount        = 0;

  vvdecLoggingCallback                     loggingCallback {};
};


} // namespace

