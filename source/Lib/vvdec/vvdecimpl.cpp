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

Copyright (c) 2018-2020, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. 
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

#include <string>

#ifdef _WIN32
    #include <intrin.h>
#endif

#if defined( __linux__ )
#include <malloc.h>
#endif

#include "vvdecimpl.h"
#include "vvdec/vvdec.h"
#include "vvdec/version.h"
#include "DecoderLib/NALread.h"

namespace vvdec {
  

std::string VVDecImpl::m_cTmpErrorString;
std::string VVDecImpl::m_cNalType;


VVDecImpl::VVDecImpl()
{

}

VVDecImpl::~VVDecImpl()
{

}

int VVDecImpl::init( const VVDecParameter& rcVVDecParameter )
{
  if( m_bInitialized ){ return VVDEC_ERR_INITIALIZE; }

  initROM();

  // create decoder class
  m_cDecLib.create( rcVVDecParameter.m_iThreads, rcVVDecParameter.m_iParseThreads );

  g_verbosity = MsgLevel( rcVVDecParameter.m_eLogLevel );

  // initialize decoder class
  m_cDecLib.setDecodedPictureHashSEIEnabled( (int) rcVVDecParameter.m_bDecodedPictureHashSEIEnabled );
//  if (!m_outputDecodedSEIMessagesFilename.empty())
//  {
//    std::ostream &os=m_seiMessageFileStream.is_open() ? m_seiMessageFileStream : std::cout;
//    m_cDecLib.setDecodedSEIMessageOutputStream(&os);
//  }

#ifdef TARGET_SIMD_X86
  switch( rcVVDecParameter.m_eSIMD_Extension )
  {
  case SIMD_SCALAR: read_x86_extension( "SCALAR" ); break;
  case SIMD_SSE41 : read_x86_extension( "SSE41"  ); break;
  case SIMD_SSE42 : read_x86_extension( "SSE42"  ); break;
  case SIMD_AVX   : read_x86_extension( "AVX"    ); break;
  case SIMD_AVX2  : read_x86_extension( "AVX2"   ); break;
  case SIMD_AVX512: read_x86_extension( "AVX512" ); break;

  case SIMD_DEFAULT:
  default: break;
  }
#endif

  m_sDecoderCapabilities = m_cDecLib.getDecoderCapabilities();

  m_uiSeqNumber    = 0;
  m_uiSeqNumOutput = 0;
  m_bInitialized   = true;

  return VVDEC_OK;
}

int VVDecImpl::uninit()
{
  if( !m_bInitialized ){ return VVDEC_ERR_INITIALIZE; }

  bool bFlushDecoder = true;
  while( bFlushDecoder)
  {
    Frame* pcFrame= NULL;

    // flush the decoder
    int iRet = flush( &pcFrame );
    if( iRet != 0 )  {  bFlushDecoder = false; }

    if( NULL != pcFrame  )
    {
      // free picture memory
      objectUnref( pcFrame );
    }
    else
    {
      bFlushDecoder = false;
      break;
    }
  };

  for( auto& pic : m_pcLibPictureList )
  {
    m_cDecLib.releasePicture( pic );
  }
  m_pcLibPictureList.clear();

  // destroy internal classes
  m_cDecLib.destroy();

  destroyROM();

  for( auto& pic : m_rcFrameList )
  {
     if( m_bCreateNewPicBuf )
     {
       for( unsigned int comp = 0; comp < pic.m_uiNumComponents; comp++ )
       {
         if ( comp == 0 && NULL != pic.m_cComponent[comp].m_pucBuffer )
         {
           delete [] pic.m_cComponent[comp].m_pucBuffer;
         }
         pic.m_cComponent[comp].m_pucBuffer = NULL;
       }
     }

     if( NULL != pic.m_pcPicExtendedAttributes )
     {
       delete pic.m_pcPicExtendedAttributes;
       pic.m_pcPicExtendedAttributes = NULL;
     }
  }

  m_rcFrameList.clear();
  m_pcFrameNext = m_rcFrameList.end();

#if defined( __linux__ )
  malloc_trim(0);
#endif

  m_bInitialized = false;
  return VVDEC_OK;
}


int VVDecImpl::decode( AccessUnit& rcAccessUnit, Frame** ppcFrame )
{
  if( !m_bInitialized ){ return VVDEC_ERR_INITIALIZE; }

  int iRet= VVDEC_OK;

  try
  {
    InputNALUnit nalu;
    Picture * pcPic = nullptr;

    int iComrpPacketCnt = 0;

    if( rcAccessUnit.m_iUsedSize )
    {
      bool bStartCodeFound = false;
      std::vector<size_t> iStartCodePosVec;
      std::vector<size_t> iAUEndPosVec;
      std::vector<size_t> iStartCodeSizeVec;

      int pos = 0;
      while( pos+3 < rcAccessUnit.m_iUsedSize )
      {
        // no start code found
        if( pos >= rcAccessUnit.m_iUsedSize ) { THROW( "could not find a startcode" ); }

        int iFound = xRetrieveNalStartCode(&rcAccessUnit.m_pucBuffer[pos], 3);
        if( iFound == 1 )
        {
          bStartCodeFound = true;

          iStartCodePosVec.push_back( pos+4 );
          iStartCodeSizeVec.push_back( 4 );

          if( pos > 0 )
          {
            iAUEndPosVec.push_back(pos);
          }

          pos+=3;
        }
        else
        {
          iFound = xRetrieveNalStartCode(&rcAccessUnit.m_pucBuffer[pos], 2);
          if( iFound == 1 )
          {
            bStartCodeFound = true;

            iStartCodePosVec.push_back( pos+3 );
            iStartCodeSizeVec.push_back( 3 );
            if( pos > 0 )
            {
              iAUEndPosVec.push_back(pos);
            }

            pos+=2;
          }
        }
        pos++;
      }

      if( bStartCodeFound )
      {
        int iLastPos = rcAccessUnit.m_iUsedSize;
        while( rcAccessUnit.m_pucBuffer[iLastPos-1] == 0 && iLastPos > 0 )
        {
          iLastPos--;
        }
        iAUEndPosVec.push_back( iLastPos );

        // iterate over all AU´s
        for( size_t iAU = 0; iAU < iStartCodePosVec.size(); iAU++ )
        {
          std::vector<uint8_t>& nalUnit = nalu.getBitstream().getFifo();
          uint32_t uiNaluBytes = (uint32_t)iStartCodeSizeVec[iAU];
          for( size_t pos = iStartCodePosVec[iAU]; pos < iAUEndPosVec[iAU]; pos++ )
          {
            nalUnit.push_back( rcAccessUnit.m_pucBuffer[pos]);
            uiNaluBytes++;
          }

          InputBitstream& rBitstream = nalu.getBitstream();
          // perform anti-emulation prevention
          if( 0 != xConvertPayloadToRBSP(nalUnit, &rBitstream, (nalUnit[0] & 64) == 0) )
          {
            return VVDEC_ERR_UNSPECIFIED;
          }

          rBitstream.resetToStart();

          if( 0 != xReadNalUnitHeader(nalu) )
          {
            return VVDEC_ERR_UNSPECIFIED;
          }


          if ( NALUnit::isVclNalUnitType( nalu.m_nalUnitType ) )
          {
            iComrpPacketCnt++;
          }

          if( rcAccessUnit.m_bCtsValid ){  nalu.m_cts = rcAccessUnit.m_uiCts; }
          if( rcAccessUnit.m_bDtsValid ){  nalu.m_dts = rcAccessUnit.m_uiDts; }
          nalu.m_rap = rcAccessUnit.m_bRAP;
          nalu.m_bits = uiNaluBytes*8;

          pcPic = m_cDecLib.decode( nalu );
          xHandleOutput( pcPic );

          if( iAU != iStartCodePosVec.size() - 1 )
          {
            // reset nalu only when not last nal
            nalu.getBitstream().resetToStart();
            nalu.getBitstream().getFifo().clear();
          }
        }
      }
      else
      {
        *ppcFrame = nullptr;
        return VVDEC_ERR_UNSPECIFIED;
      }
    }
    else
    {
      nalu.m_nalUnitType = NAL_UNIT_INVALID;

      // Flush decoder
      pcPic = m_cDecLib.flushPic();
      xHandleOutput( pcPic );

      iRet = VVDEC_EOF;
    }

    if ( iRet == VVDEC_EOF && !pcPic )
    {
      *ppcFrame = nullptr;
      return iRet;
    }

    if( !m_rcFrameList.empty())
    {
      if( m_pcFrameNext == m_rcFrameList.end()  )
      {
        if( iComrpPacketCnt )
        {
          iRet = VVDEC_TRY_AGAIN;
        }
        *ppcFrame = nullptr;
      }
      else
      {
        *ppcFrame = &( *m_pcFrameNext );
        m_uiSeqNumOutput = (*ppcFrame)->m_uiSequenceNumber;
        ++m_pcFrameNext;
      }
    }
    else
    {
      *ppcFrame = nullptr;
      if( iComrpPacketCnt )
      {
        iRet = VVDEC_TRY_AGAIN;
      }
    }
  }
  catch( std::overflow_error& e )
  {
    //assert( 0 );
    std::stringstream css;
    css << "caught overflow exception " << e.what();
    m_cAdditionalErrorString = css.str();
    return VVDEC_ERR_UNSPECIFIED;
  }
  catch( std::exception& e )
  {
    //assert( 0 );
    std::stringstream css;
    css << "caught unknown exception " << e.what();
    m_cAdditionalErrorString = css.str();
    return VVDEC_ERR_UNSPECIFIED;
  }

  return iRet;
}

int VVDecImpl::flush( Frame** ppcFrame )
{
  if( !m_bInitialized ){ return VVDEC_ERR_INITIALIZE; }
  int iRet= VVDEC_OK;

  // Flush decoder
  try
  {
    Picture * pcPic = nullptr;

    bool bContinue = true;
    while( bContinue )
    {
      pcPic = m_cDecLib.flushPic();
      xHandleOutput( pcPic );
      if( !pcPic || !m_rcFrameList.empty() )
      {
        bContinue = false;
      }
    }

    if( !m_rcFrameList.empty())
    {
      if( m_pcFrameNext == m_rcFrameList.end()  )
      {
        iRet = VVDEC_EOF;
        *ppcFrame = nullptr;
      }
      else
      {
        *ppcFrame = &( *m_pcFrameNext );
        m_uiSeqNumOutput = (*ppcFrame)->m_uiSequenceNumber;
        ++m_pcFrameNext;
      }
    }
    else
    {
      iRet = VVDEC_EOF;
      *ppcFrame = nullptr;
    }
  }
  catch( std::overflow_error& e )
  {
    std::stringstream css;
    css << "caught overflow exception " << e.what();
    m_cAdditionalErrorString = css.str();
    return VVDEC_ERR_UNSPECIFIED;
  }
  catch( std::exception& e )
  {
    std::stringstream css;
    css << "caught unknown exception " << e.what();
    m_cAdditionalErrorString = css.str();
    return VVDEC_ERR_UNSPECIFIED;
  }

  if( 0 != iRet )
  {
    return (int)VVDEC_EOF;
  }

  return iRet;
}

int VVDecImpl::objectUnref( Frame* pcFrame )
{
  if( !m_bInitialized ){ return VVDEC_ERR_INITIALIZE; }

  if ( NULL == pcFrame )
  {
    m_cErrorString = "objectUnref: cannot unref pictue with null pointer";
    return VVDEC_ERR_UNSPECIFIED;
  }

  bool bPicFound = false;
  for( auto& pic : m_rcFrameList )
  {
    if( &pic == pcFrame )
    {
      bPicFound = true;

      if ( m_bCreateNewPicBuf )
      {
        for( unsigned int comp = 0; comp < pic.m_uiNumComponents; comp++ )
        {
          if ( comp == 0 && NULL != pic.m_cComponent[comp].m_pucBuffer )
          {
            delete [] pic.m_cComponent[comp].m_pucBuffer;
            pic.m_cComponent[comp].m_pucBuffer = NULL;
          }
          pic.m_cComponent[comp].m_pucBuffer = NULL;

          pic.m_cComponent[comp].m_uiWidth      = 0;
          pic.m_cComponent[comp].m_uiHeight     = 0;
          pic.m_cComponent[comp].m_iStride      = 0;
          pic.m_cComponent[comp].m_uiByteOffset = 0;
        }
      }
      else
      {
        for( std::list<Picture*>::iterator itLibPic = m_pcLibPictureList.begin(); itLibPic != m_pcLibPictureList.end();  itLibPic++ )
        {
          if( (*itLibPic)->cts == pic.m_uiCts )
          {
            m_cDecLib.releasePicture( *itLibPic );
            m_pcLibPictureList.erase( itLibPic );
            break;
          }
        }
      }

      pic.m_uiNumComponents = 0;
      pic.m_uiWidth         = 0;
      pic.m_uiHeight        = 0;
      pic.m_uiBitDepth      = 0;
      pic.m_uiSequenceNumber= 0;
      pic.m_uiCts           = 0;
      pic.m_bCtsValid       = false;

      pic.m_eFrameFormat = VVC_FF_INVALID;
      pic.m_eColorFormat = VVC_CF_INVALID;

      if( NULL != pic.m_pcPicExtendedAttributes )
      {
         delete pic.m_pcPicExtendedAttributes;
         pic.m_pcPicExtendedAttributes = NULL;
      }

      break;
    }
  }

  if( bPicFound )
  {
    // remove picture from picture list
    std::list<Frame>::iterator itFrame = m_rcFrameList.end();
    for( std::list<Frame>::iterator it = m_rcFrameList.begin(); it != m_rcFrameList.end(); it++ )
    {
       if( &*it == pcFrame )
       {
           itFrame = it;
           break;
       }
    }
    if( itFrame != m_rcFrameList.end())
    {
      m_rcFrameList.erase(itFrame);
    }
    else
    {
      m_cErrorString = "objectUnref() cannot find picture in picture list";
      return VVDEC_ERR_UNSPECIFIED;
    }
  }
  else
  {
    return VVDEC_ERR_UNSPECIFIED;
  }

  return VVDEC_OK;
}

int VVDecImpl::getNumberOfErrorsPictureHashSEI()
{
  if( !m_bInitialized ){ return VVDEC_ERR_INITIALIZE; }

  uint32_t iErrors = m_cDecLib.getNumberOfChecksumErrorsDetected();
  return iErrors;
}

void VVDecImpl::clockStartTime()
{
  m_cTPStart = std::chrono::steady_clock::now();
}

void VVDecImpl::clockEndTime()
{
  m_cTPEnd = std::chrono::steady_clock::now();
}

double VVDecImpl::clockGetTimeDiffMs()
{
  return (double)(std::chrono::duration_cast<std::chrono::milliseconds>((m_cTPEnd)-(m_cTPStart)).count());
}

const char* VVDecImpl::getErrorMsg( int nRet )
{
  switch( nRet )
  {
  case VVDEC_OK :                  m_cTmpErrorString = "expected behavior"; break;
  case VVDEC_ERR_UNSPECIFIED:      m_cTmpErrorString = "unspecified malfunction"; break;
  case VVDEC_ERR_INITIALIZE:       m_cTmpErrorString = "decoder not initialized or tried to initialize multiple times"; break;
  case VVDEC_ERR_ALLOCATE:         m_cTmpErrorString = "internal allocation error"; break;
  case VVDEC_NOT_ENOUGH_MEM:       m_cTmpErrorString = "allocated memory to small to receive decoded data"; break;
  case VVDEC_ERR_PARAMETER:        m_cTmpErrorString = "inconsistent or invalid parameters"; break;
  case VVDEC_ERR_NOT_SUPPORTED:    m_cTmpErrorString = "unsupported request"; break;
  case VVDEC_ERR_RESTART_REQUIRED: m_cTmpErrorString = "decoder requires restart"; break;
  case VVDEC_ERR_CPU:              m_cTmpErrorString = "unsupported CPU - SSE 4.1 needed!"; break;
  case VVDEC_TRY_AGAIN:            m_cTmpErrorString = "more bitstream data needed. try again"; break;
  case VVDEC_EOF:                  m_cTmpErrorString = "end of stream"; break;
  default:                         m_cTmpErrorString = "unknown ret code"; break;
  }
  return m_cTmpErrorString.c_str();
}

int VVDecImpl::setAndRetErrorMsg( int iRet )
{
  m_cErrorString = getErrorMsg(iRet);
  return iRet;
}

const char* VVDecImpl::getVersionNumber()
{
  return VVDEC_VERSION;
}

const char* VVDecImpl::getDecoderInfo()
{
    m_sDecoderInfo  = "Fraunhofer Versatile Video Decoder ";
//     m_sDecoderInfo += "/";
    m_sDecoderInfo += " version ";
    m_sDecoderInfo += getVersionNumber();
    m_sDecoderInfo += "; ";
    m_sDecoderInfo += m_sDecoderCapabilities;
    //m_sDecoderInfo += ";";
    //m_sDecoderInfo += hevcdec_capabilities( &m_hevcdec );
    m_sDecoderInfo += ") ";
    return m_sDecoderInfo.c_str();
}

NalType VVDecImpl::getNalUnitType ( AccessUnit& rcAccessUnit )
{
  NalType eNalType = VVC_NAL_UNIT_INVALID;

  if( rcAccessUnit.m_pucBuffer == nullptr || rcAccessUnit.m_iBufSize == 0 || rcAccessUnit.m_iUsedSize == 0 || rcAccessUnit.m_iBufSize < 3 )
  {
    return eNalType;
  }

  unsigned char* pcBuf = rcAccessUnit.m_pucBuffer;
  int iOffset=0;

  int found = 1;
  int i=0;
  for ( i = 0; i < 3; i++)
  {
    if( pcBuf[i] != 0 )
    {
      found = 0;
    }
  }

  if( pcBuf[i] != 1 )
  {
    found = 0;
  }

  if( found )
  {
    iOffset=5;
  }
  else
  {
    found = 1;
    i=0;
    for ( i = 0; i < 2; i++)
    {
      if( pcBuf[i] != 0 )
      {
        found = 0;
      }
    }

    if( pcBuf[i] != 1 )
    {
      found = 0;
    }

    if( found )
    {
      iOffset=4;
    }
  }

  if( found )
  {
    unsigned char uc = pcBuf[iOffset];
    int nalUnitType   = ((uc >> 3) & 0x1F ); 
    eNalType = (NalType)nalUnitType;
  }

  return eNalType;
}

const char* VVDecImpl::getNalUnitTypeAsString( NalType t )
{
  m_cNalType = "NAL_UNIT_INVALID";

  GCC_EXTRA_WARNING_switch_enum
  switch ( t )
  {
  case NAL_UNIT_CODED_SLICE_TRAIL:           m_cNalType = "NAL_UNIT_CODED_SLICE_TRAIL"; break; // 0
  case NAL_UNIT_CODED_SLICE_STSA:            m_cNalType = "NAL_UNIT_CODED_SLICE_STSA"; break; // 1
  case NAL_UNIT_CODED_SLICE_RADL:            m_cNalType = "NAL_UNIT_CODED_SLICE_RADL"; break; // 2
  case NAL_UNIT_CODED_SLICE_RASL:            m_cNalType = "NAL_UNIT_CODED_SLICE_RASL"; break; // 3

  case NAL_UNIT_RESERVED_VCL_4:              m_cNalType = "NAL_UNIT_RESERVED_VCL_4"; break; // 4
  case NAL_UNIT_RESERVED_VCL_5:              m_cNalType = "NAL_UNIT_RESERVED_VCL_5"; break; // 5
  case NAL_UNIT_RESERVED_VCL_6:              m_cNalType = "NAL_UNIT_RESERVED_VCL_6"; break; // 6

  case NAL_UNIT_CODED_SLICE_IDR_W_RADL:      m_cNalType = "NAL_UNIT_CODED_SLICE_IDR_W_RADL"; break; // 7
  case NAL_UNIT_CODED_SLICE_IDR_N_LP:        m_cNalType = "NAL_UNIT_CODED_SLICE_IDR_N_LP"; break; // 8
  case NAL_UNIT_CODED_SLICE_CRA:             m_cNalType = "NAL_UNIT_CODED_SLICE_CRA"; break; // 9
  case NAL_UNIT_CODED_SLICE_GDR:             m_cNalType = "NAL_UNIT_CODED_SLICE_GDR"; break; // 10

  case NAL_UNIT_RESERVED_IRAP_VCL_11:        m_cNalType = "NAL_UNIT_RESERVED_IRAP_VCL_11"; break; // 11
  case NAL_UNIT_RESERVED_IRAP_VCL_12:        m_cNalType = "NAL_UNIT_RESERVED_IRAP_VCL_12"; break; // 12

  case NAL_UNIT_DCI:                         m_cNalType = "NAL_UNIT_DCI"; break; // 13
  case NAL_UNIT_VPS:                         m_cNalType = "NAL_UNIT_VPS"; break; // 14
  case NAL_UNIT_SPS:                         m_cNalType = "NAL_UNIT_SPS"; break; // 15
  case NAL_UNIT_PPS:                         m_cNalType = "NAL_UNIT_PPS"; break; // 16
  case NAL_UNIT_PREFIX_APS:                  m_cNalType = "NAL_UNIT_PREFIX_APS"; break; // 17
  case NAL_UNIT_SUFFIX_APS:                  m_cNalType = "NAL_UNIT_SUFFIX_APS"; break; // 18
  case NAL_UNIT_PH:                          m_cNalType = "NAL_UNIT_PH"; break; // 19
  case NAL_UNIT_ACCESS_UNIT_DELIMITER:       m_cNalType = "NAL_UNIT_ACCESS_UNIT_DELIMITER"; break; // 20
  case NAL_UNIT_EOS:                         m_cNalType = "NAL_UNIT_EOS"; break; // 21
  case NAL_UNIT_EOB:                         m_cNalType = "NAL_UNIT_EOB"; break; // 22
  case NAL_UNIT_PREFIX_SEI:                  m_cNalType = "NAL_UNIT_PREFIX_SEI"; break; // 23
  case NAL_UNIT_SUFFIX_SEI:                  m_cNalType = "NAL_UNIT_SUFFIX_SEI"; break; // 24
  case NAL_UNIT_FD:                          m_cNalType = "NAL_UNIT_FD"; break; // 25


  case NAL_UNIT_RESERVED_NVCL_26:            m_cNalType = "NAL_UNIT_RESERVED_NVCL_26"; break; // 26
  case NAL_UNIT_RESERVED_NVCL_27:            m_cNalType = "NAL_UNIT_RESERVED_NVCL_27"; break; // 27

  case NAL_UNIT_UNSPECIFIED_28:              m_cNalType = "NAL_UNIT_UNSPECIFIED_28"; break; // 28
  case NAL_UNIT_UNSPECIFIED_29:              m_cNalType = "NAL_UNIT_UNSPECIFIED_29"; break; // 29
  case NAL_UNIT_UNSPECIFIED_30:              m_cNalType = "NAL_UNIT_UNSPECIFIED_30"; break; // 30
  case NAL_UNIT_UNSPECIFIED_31:              m_cNalType = "NAL_UNIT_UNSPECIFIED_31"; break; // 31

  case NAL_UNIT_INVALID:
  default:                                   m_cNalType = "NAL_UNIT_INVALID"; break;
  }
  GCC_WARNING_RESET

  return m_cNalType.c_str();
}


bool VVDecImpl::isNalUnitSlice( NalType t )
{
  return t == VVC_NAL_UNIT_CODED_SLICE_TRAIL
      || t == VVC_NAL_UNIT_CODED_SLICE_STSA
      || t == VVC_NAL_UNIT_CODED_SLICE_RADL
      || t == VVC_NAL_UNIT_CODED_SLICE_RASL
      || t == VVC_NAL_UNIT_CODED_SLICE_IDR_W_RADL
      || t == VVC_NAL_UNIT_CODED_SLICE_IDR_N_LP
      || t == VVC_NAL_UNIT_CODED_SLICE_CRA
      || t == VVC_NAL_UNIT_CODED_SLICE_GDR;
}


bool VVDecImpl::isNalUnitSideData( NalType t )
{
  return ! isNalUnitSlice(t);
}


int VVDecImpl::copyComp( const unsigned char* pucSrc, unsigned char* pucDest, unsigned int uiWidth, unsigned int uiHeight, int iStrideSrc, int iStrideDest, int iBytesPerSample  )
{
  if( NULL != pucSrc && NULL != pucDest )
  {
    if( iStrideDest ==  iStrideSrc )
    {
      ::memcpy( pucDest, pucSrc, uiHeight * iStrideDest );
    }
    else
    {
      if (iBytesPerSample > 1)
      {
        uiWidth <<= 1;
        for( unsigned int uiH = 0; uiH < uiHeight; uiH++ )
        {
          ::memcpy( pucDest, pucSrc, uiWidth );
          pucSrc  += iStrideSrc;
          pucDest += iStrideDest;
        }
      }
      else
      {
        // shift short->char
        for( unsigned int y=0; y < uiHeight; y++ )
        {
          for( unsigned int x=0; x < uiWidth; x++ )
          {
            pucDest[x] = pucSrc[x<<1];
          }
          pucSrc   += iStrideSrc;
          pucDest  += iStrideDest;
        }
      }
    }
  }
  else
  {
    return -1;
  }

  return 0;
}

int VVDecImpl::xAddPicture( Picture* pcPic )
{
  // copy internal picture to external (Priority PPS > SPS)
  const Window &conf    = pcPic->cs->pps->getConformanceWindowPresentFlag()
                          ? pcPic->cs->pps->getConformanceWindow()
                          : pcPic->cs->sps->getConformanceWindow();
//          const Window  defDisp = (m_respectDefDispWindow && pcPic->cs->sps->getVuiParametersPresentFlag())
//                                  ? pcPic->cs->sps->getVuiParameters()->getDefaultDisplayWindow()
//                                  : Window();
  const Window  defDisp =  Window();

  int confLeft   = conf.getWindowLeftOffset()   * SPS::getWinUnitY(pcPic->cs->sps->getChromaFormatIdc())  + defDisp.getWindowLeftOffset();
  int confRight  = conf.getWindowRightOffset()  * SPS::getWinUnitY(pcPic->cs->sps->getChromaFormatIdc())  + defDisp.getWindowRightOffset();
  int confTop    = conf.getWindowTopOffset()    * SPS::getWinUnitY(pcPic->cs->sps->getChromaFormatIdc())  + defDisp.getWindowTopOffset();
  int confBottom = conf.getWindowBottomOffset() * SPS::getWinUnitY(pcPic->cs->sps->getChromaFormatIdc())  + defDisp.getWindowBottomOffset();

  const CPelUnitBuf& cPicBuf =  pcPic->getRecoBuf();

  const CPelBuf areaY     = cPicBuf.get(COMPONENT_Y);
  const uint32_t uiWidth  = areaY.width - confLeft - confRight;
  const uint32_t uiHeight = areaY.height -  confTop  - confBottom;

  const BitDepths &bitDepths= pcPic->cs->sps->getBitDepths(); // use bit depths of first reconstructed picture.

  if ((uiWidth == 0) || (uiHeight == 0))
  {
    msg( ERROR, "objectUnref: %dx%d luma sample output picture!\n", uiWidth, uiHeight );
    return VVDEC_ERR_UNSPECIFIED;
  }

  if( m_uiSeqNumber == 0 )
  {
    unsigned int uiBitDepth   = 8;
    for( uint32_t c = 0; c < MAX_NUM_CHANNEL_TYPE; c++ )
    {
      uiBitDepth = std::max( (uint32_t)bitDepths.recon[c], uiBitDepth );
    }

    m_bCreateNewPicBuf =  (uiBitDepth == 8) ? true : false; // for 8bit output we need to copy the lib picture from unsigned short into unsigned char buffer
  }

  // create a brand new picture object
  Frame cFrame;
  xCreateFrame ( cFrame, cPicBuf, uiWidth, uiHeight, bitDepths );

  const int maxComponent = getNumberValidComponents( cPicBuf.chromaFormat );

  if( m_bCreateNewPicBuf )
  {
    // copy picture into target memory
    for( uint32_t comp=0; comp < maxComponent; comp++ )
    {
      const ComponentID compID      = ComponentID(comp);
      const uint32_t    csx         = ::getComponentScaleX(compID, cPicBuf.chromaFormat);
      const uint32_t    csy         = ::getComponentScaleY(compID, cPicBuf.chromaFormat);
      const CPelBuf     area        = cPicBuf.get(compID);
      unsigned int uiBytesPerSample = bitDepths.recon[0] > 8 ? 2 : 1;

      const ptrdiff_t   planeOffset = (confLeft >> csx) + (confTop >> csy) * area.stride;
      //const unsigned char* pucOrigin   = (const unsigned char*)area.bufAt (0, 0).ptr;
      const unsigned char* pucOrigin   = (const unsigned char*)area.buf;

     copyComp(  pucOrigin + planeOffset,
         cFrame.m_cComponent[VVC_CT_Y].m_pucBuffer+cFrame.m_cComponent[comp].m_uiByteOffset,
         area.width, area.height,
         area.stride<<1, cFrame.m_cComponent[comp].m_iStride, uiBytesPerSample  );
    }
  }
  else
  {
    // use internal lib picture memory
    for( uint32_t comp=0; comp < maxComponent; comp++ )
    {
      const ComponentID compID      = ComponentID(comp);
      const uint32_t    csx         = ::getComponentScaleX(compID, cPicBuf.chromaFormat);
      const uint32_t    csy         = ::getComponentScaleY(compID, cPicBuf.chromaFormat);
      const CPelBuf     area        = cPicBuf.get(compID);
      //unsigned int wordSize         = bitDepths.recon[0] > 8 ? 2 : 1;
      const ptrdiff_t   planeOffset = (confLeft >> csx) + (confTop >> csy) * area.stride;

      //unsigned char* pucOrigin   = (unsigned char*)area.bufAt (0, 0).ptr;
      Pel* pucOrigin   = (Pel*)area.buf;

      cFrame.m_cComponent[comp].m_pucBuffer = (unsigned char*)(pucOrigin + planeOffset);
    }
    m_pcLibPictureList.push_back( pcPic );
  }

  // set picture attributes
  cFrame.m_uiSequenceNumber = m_uiSeqNumber;
  cFrame.m_uiCts     = pcPic->getCts();
  cFrame.m_bCtsValid = true;

  cFrame.m_pcPicExtendedAttributes = new PicExtendedAttributes();
  cFrame.m_pcPicExtendedAttributes->m_uiPOC          = pcPic->poc;
  cFrame.m_pcPicExtendedAttributes->m_iTemporalLayer = pcPic->getTLayer();
  cFrame.m_pcPicExtendedAttributes->m_uiBits         = pcPic->getNaluBits();

  cFrame.m_pcPicExtendedAttributes->m_eNalType       = (NalType)pcPic->eNalUnitType;

  cFrame.m_pcPicExtendedAttributes->m_bRefPic = pcPic->referenced;

  if ( !pcPic->slices.empty() )
  {
    switch( pcPic->slices.front()->getSliceType() )
    {
      case I_SLICE: cFrame.m_pcPicExtendedAttributes->m_eSliceType = VVC_SLICETYPE_I; break;
      case P_SLICE: cFrame.m_pcPicExtendedAttributes->m_eSliceType = VVC_SLICETYPE_P; break;
      case B_SLICE: cFrame.m_pcPicExtendedAttributes->m_eSliceType = VVC_SLICETYPE_B; break;
      default:      cFrame.m_pcPicExtendedAttributes->m_eSliceType = VVC_SLICETYPE_UNKNOWN; break;
    }
  }

  m_rcFrameList.push_back( cFrame );

  if( m_pcFrameNext == m_rcFrameList.end() )
  {
    if( m_uiSeqNumber == 0 )
    {
      m_pcFrameNext = m_rcFrameList.begin();
    }
    else
    {
      for( std::list<Frame>::iterator it = m_rcFrameList.begin(); it != m_rcFrameList.end(); it++ )
      {
        if(  (*it).m_uiSequenceNumber > m_uiSeqNumOutput )
        {
          m_pcFrameNext = it;
          break;
        }
      }
    }
  }

  m_uiSeqNumber++;

  return 0;
}



int VVDecImpl::xCreateFrame( Frame& rcFrame, const CPelUnitBuf& rcPicBuf, uint32_t uiWidth, uint32_t uiHeight, const BitDepths& rcBitDepths )
{
  size_t nBufSize = 0;

  rcFrame.m_uiWidth      = uiWidth;
  rcFrame.m_uiHeight     = uiHeight;
  rcFrame.m_uiBitDepth   = 8;
  rcFrame.m_eFrameFormat = VVC_FF_PROGRESSIVE;

  for( uint32_t c = 0; c < MAX_NUM_CHANNEL_TYPE; c++ )
  {
    rcFrame.m_uiBitDepth = std::max( (uint32_t)rcBitDepths.recon[c], rcFrame.m_uiBitDepth );
  }

  rcFrame.m_cComponent[VVC_CT_Y].m_uiWidth          = uiWidth;
  rcFrame.m_cComponent[VVC_CT_Y].m_uiHeight         = uiHeight;
  rcFrame.m_cComponent[VVC_CT_Y].m_uiBytesPerSample = rcBitDepths.recon[CHANNEL_TYPE_LUMA] > 8 ? 2 : 1;
  rcFrame.m_cComponent[VVC_CT_Y].m_iStride          = m_bCreateNewPicBuf  ? uiWidth*rcFrame.m_cComponent[VVC_CT_Y].m_uiBytesPerSample
                                                                            : rcPicBuf.get(COMPONENT_Y).stride * rcFrame.m_cComponent[VVC_CT_Y].m_uiBytesPerSample;
  rcFrame.m_cComponent[VVC_CT_Y].m_uiByteOffset     = 0;
  rcFrame.m_cComponent[VVC_CT_Y].m_uiBitDepth       = rcBitDepths.recon[CHANNEL_TYPE_LUMA];


  switch( rcPicBuf.chromaFormat )
  {
    case CHROMA_400:
      {
        rcFrame.m_eColorFormat = VVC_CF_YUV400_PLANAR;
        rcFrame.m_uiNumComponents = 1;

        rcFrame.m_cComponent[VVC_CT_U].m_uiWidth          = 0;
        rcFrame.m_cComponent[VVC_CT_U].m_uiHeight         = 0;
        rcFrame.m_cComponent[VVC_CT_U].m_iStride          = 0;
        rcFrame.m_cComponent[VVC_CT_U].m_uiByteOffset     = 0;
        rcFrame.m_cComponent[VVC_CT_U].m_uiBytesPerSample = 0;
        rcFrame.m_cComponent[VVC_CT_U].m_uiBitDepth       = 0;

        rcFrame.m_cComponent[VVC_CT_V].m_uiWidth          = 0;
        rcFrame.m_cComponent[VVC_CT_V].m_uiHeight         = 0;
        rcFrame.m_cComponent[VVC_CT_V].m_iStride          = 0;
        rcFrame.m_cComponent[VVC_CT_V].m_uiByteOffset     = 0;
        rcFrame.m_cComponent[VVC_CT_V].m_uiBytesPerSample = 0;
        rcFrame.m_cComponent[VVC_CT_V].m_uiBitDepth       = 0;

        if( m_bCreateNewPicBuf )
        {
          // we have to copy the packet into 8bit, because internal bitdepth is always Pel (unsigned short)
          nBufSize = rcFrame.m_cComponent[VVC_CT_Y].m_iStride * uiHeight;
        }
        break;
      }
    case CHROMA_420:
      {
        rcFrame.m_eColorFormat = VVC_CF_YUV420_PLANAR;
        rcFrame.m_uiNumComponents = 3;
        const unsigned int uiCWidth       = uiWidth>>1;
        const unsigned int uiCHeight      = uiHeight>>1;

        rcFrame.m_cComponent[VVC_CT_U].m_uiWidth          = uiCWidth;
        rcFrame.m_cComponent[VVC_CT_U].m_uiHeight         = uiCHeight;
        rcFrame.m_cComponent[VVC_CT_U].m_uiBytesPerSample = rcBitDepths.recon[CHANNEL_TYPE_CHROMA] > 8 ? 2 : 1;
        rcFrame.m_cComponent[VVC_CT_U].m_iStride          = m_bCreateNewPicBuf ? uiCWidth * rcFrame.m_cComponent[CHANNEL_TYPE_CHROMA].m_uiBytesPerSample
                                                                                  : rcPicBuf.get(COMPONENT_Cb).stride * rcFrame.m_cComponent[VVC_CT_Y].m_uiBytesPerSample;
        rcFrame.m_cComponent[VVC_CT_U].m_uiByteOffset     = rcFrame.m_cComponent[VVC_CT_Y].m_iStride * rcFrame.m_cComponent[VVC_CT_Y].m_uiHeight;
        rcFrame.m_cComponent[VVC_CT_U].m_uiBitDepth       = rcBitDepths.recon[CHANNEL_TYPE_CHROMA];

        uint32_t nCSize = rcFrame.m_cComponent[VVC_CT_U].m_iStride*uiCHeight;

        rcFrame.m_cComponent[VVC_CT_V].m_uiWidth          = uiCWidth;
        rcFrame.m_cComponent[VVC_CT_V].m_uiHeight         = uiCHeight;
        rcFrame.m_cComponent[VVC_CT_V].m_uiBytesPerSample = rcBitDepths.recon[CHANNEL_TYPE_CHROMA] > 8 ? 2 : 1;
        rcFrame.m_cComponent[VVC_CT_V].m_iStride          = uiCWidth * rcFrame.m_cComponent[CHANNEL_TYPE_CHROMA].m_uiBytesPerSample;
        rcFrame.m_cComponent[VVC_CT_V].m_iStride          = m_bCreateNewPicBuf ? uiCWidth * rcFrame.m_cComponent[CHANNEL_TYPE_CHROMA].m_uiBytesPerSample
                                                                                 : rcPicBuf.get(COMPONENT_Cr).stride * rcFrame.m_cComponent[VVC_CT_Y].m_uiBytesPerSample;
        rcFrame.m_cComponent[VVC_CT_V].m_uiByteOffset     = rcFrame.m_cComponent[VVC_CT_U].m_uiByteOffset + nCSize;
        rcFrame.m_cComponent[VVC_CT_V].m_uiBitDepth       = rcBitDepths.recon[CHANNEL_TYPE_CHROMA];
        if( m_bCreateNewPicBuf )    nBufSize = (rcFrame.m_cComponent[VVC_CT_Y].m_iStride * uiHeight) + (nCSize<<1);
        break;
      }
    case CHROMA_422:
      {
        rcFrame.m_eColorFormat = VVC_CF_YUV422_PLANAR;
        rcFrame.m_uiNumComponents = 3;

        const unsigned int uiCWidth       = uiWidth>>1;
        const unsigned int uiCHeight      = uiHeight;

        rcFrame.m_cComponent[VVC_CT_U].m_uiWidth          = uiCWidth;
        rcFrame.m_cComponent[VVC_CT_U].m_uiHeight         = uiCHeight;
        rcFrame.m_cComponent[VVC_CT_U].m_uiBytesPerSample = rcBitDepths.recon[CHANNEL_TYPE_CHROMA] > 8 ? 2 : 1;
        rcFrame.m_cComponent[VVC_CT_U].m_iStride          = m_bCreateNewPicBuf ? uiCWidth * rcFrame.m_cComponent[CHANNEL_TYPE_CHROMA].m_uiBytesPerSample :
                                                                                   rcPicBuf.get(COMPONENT_Cb).stride * rcFrame.m_cComponent[VVC_CT_Y].m_uiBytesPerSample;
        rcFrame.m_cComponent[VVC_CT_U].m_uiByteOffset     = rcFrame.m_cComponent[VVC_CT_Y].m_iStride * rcFrame.m_cComponent[VVC_CT_Y].m_uiHeight;
        rcFrame.m_cComponent[VVC_CT_U].m_uiBitDepth       = rcBitDepths.recon[CHANNEL_TYPE_CHROMA];

        uint32_t nCSize = rcFrame.m_cComponent[VVC_CT_U].m_iStride*uiCHeight;

        rcFrame.m_cComponent[VVC_CT_V].m_uiWidth          = uiCWidth;
        rcFrame.m_cComponent[VVC_CT_V].m_uiHeight         = uiCHeight;
        rcFrame.m_cComponent[VVC_CT_V].m_uiBytesPerSample = rcBitDepths.recon[CHANNEL_TYPE_CHROMA] > 8 ? 2 : 1;
        rcFrame.m_cComponent[VVC_CT_V].m_iStride          = uiCWidth * rcFrame.m_cComponent[CHANNEL_TYPE_CHROMA].m_uiBytesPerSample;
        rcFrame.m_cComponent[VVC_CT_V].m_iStride          = m_bCreateNewPicBuf ? uiCWidth * rcFrame.m_cComponent[CHANNEL_TYPE_CHROMA].m_uiBytesPerSample
                                                                                 : rcPicBuf.get(COMPONENT_Cr).stride * rcFrame.m_cComponent[VVC_CT_Y].m_uiBytesPerSample;
        rcFrame.m_cComponent[VVC_CT_V].m_uiByteOffset     = rcFrame.m_cComponent[VVC_CT_U].m_uiByteOffset + nCSize;
        rcFrame.m_cComponent[VVC_CT_V].m_uiBitDepth       = rcBitDepths.recon[CHANNEL_TYPE_CHROMA];

        if( m_bCreateNewPicBuf )  nBufSize = (rcFrame.m_cComponent[VVC_CT_Y].m_iStride * uiHeight) + (nCSize<<1);
        break;
      }
    case CHROMA_444:
      {
        rcFrame.m_eColorFormat = VVC_CF_YUV444_PLANAR;
        rcFrame.m_uiNumComponents = 3;

        rcFrame.m_cComponent[VVC_CT_U].m_uiWidth          = uiWidth;
        rcFrame.m_cComponent[VVC_CT_U].m_uiHeight         = uiHeight;
        rcFrame.m_cComponent[VVC_CT_U].m_uiBytesPerSample = rcBitDepths.recon[CHANNEL_TYPE_CHROMA] > 8 ? 2 : 1;
        rcFrame.m_cComponent[VVC_CT_U].m_iStride          = m_bCreateNewPicBuf ? uiWidth * rcFrame.m_cComponent[CHANNEL_TYPE_CHROMA].m_uiBytesPerSample
                                                                                 : rcPicBuf.get(COMPONENT_Cb).stride * rcFrame.m_cComponent[VVC_CT_Y].m_uiBytesPerSample;
        rcFrame.m_cComponent[VVC_CT_U].m_uiByteOffset     = rcFrame.m_cComponent[VVC_CT_Y].m_iStride * rcFrame.m_cComponent[VVC_CT_Y].m_uiHeight;
        rcFrame.m_cComponent[VVC_CT_U].m_uiBitDepth       = rcBitDepths.recon[CHANNEL_TYPE_CHROMA];

        rcFrame.m_cComponent[VVC_CT_V].m_uiWidth          = uiWidth;
        rcFrame.m_cComponent[VVC_CT_V].m_uiHeight         = uiHeight;
        rcFrame.m_cComponent[VVC_CT_V].m_uiBytesPerSample = rcBitDepths.recon[CHANNEL_TYPE_CHROMA] > 8 ? 2 : 1;
        rcFrame.m_cComponent[VVC_CT_V].m_iStride          = m_bCreateNewPicBuf ? uiWidth * rcFrame.m_cComponent[CHANNEL_TYPE_CHROMA].m_uiBytesPerSample
                                                                                 : rcPicBuf.get(COMPONENT_Cr).stride * rcFrame.m_cComponent[VVC_CT_Y].m_uiBytesPerSample;
        rcFrame.m_cComponent[VVC_CT_V].m_uiByteOffset     = rcFrame.m_cComponent[VVC_CT_U].m_uiByteOffset<<1;
        rcFrame.m_cComponent[VVC_CT_V].m_uiBitDepth       = rcBitDepths.recon[CHANNEL_TYPE_CHROMA];

        if( m_bCreateNewPicBuf ) nBufSize = (rcFrame.m_cComponent[VVC_CT_Y].m_iStride * uiHeight)*3;
        break;
      }
    default: break;
  }


  if( m_bCreateNewPicBuf )
  {
    if( nBufSize == 0 ){ return VVDEC_ERR_ALLOCATE; }
    rcFrame.m_cComponent[VVC_CT_Y].m_pucBuffer = new unsigned char [ nBufSize ];

    switch( rcPicBuf.chromaFormat )
    {
      case CHROMA_400:
          break;
      case CHROMA_420:
      case CHROMA_422:
      case CHROMA_444:
          rcFrame.m_cComponent[VVC_CT_U].m_pucBuffer  = rcFrame.m_cComponent[VVC_CT_Y].m_pucBuffer + rcFrame.m_cComponent[VVC_CT_U].m_uiByteOffset;
          rcFrame.m_cComponent[VVC_CT_V].m_pucBuffer  = rcFrame.m_cComponent[VVC_CT_Y].m_pucBuffer + rcFrame.m_cComponent[VVC_CT_V].m_uiByteOffset;
          break;
      default: break;
    }
  }


  return 0;
}

int VVDecImpl::xRetrieveNalStartCode( unsigned char *pB, int iZerosInStartcode )
{
  int found = 1;
  int i=0;
  for ( i = 0; i < iZerosInStartcode; i++)
  {
    if( pB[i] != 0 )
    {
      found = 0;
    }
  }

  if( pB[i] != 1 )
  {
    found = 0;
  }

  return found;
}


int VVDecImpl::xConvertPayloadToRBSP( std::vector<uint8_t>& nalUnitBuf, InputBitstream *bitstream, bool isVclNalUnit)
{
  uint32_t zeroCount = 0;
  std::vector<uint8_t>::iterator it_read, it_write;

  uint32_t pos = 0;
  bitstream->clearEmulationPreventionByteLocation();
  for (it_read = it_write = nalUnitBuf.begin(); it_read != nalUnitBuf.end(); it_read++, it_write++, pos++)
  {
    if(zeroCount >= 2 && *it_read < 0x03 )
    {
      msg( ERROR, "Zero count is '2' and read value is small than '3'\n");
      return -1;
    }
    if (zeroCount == 2 && *it_read == 0x03)
    {
      bitstream->pushEmulationPreventionByteLocation( pos );
      pos++;
      it_read++;
      zeroCount = 0;
      if (it_read == nalUnitBuf.end())
      {
        break;
      }

      if( *it_read > 0x03 )
      {
        msg( ERROR, "Read a value bigger than '3'\n");
        return -1;
      }

    }
    zeroCount = (*it_read == 0x00) ? zeroCount+1 : 0;
    *it_write = *it_read;
  }

  if( zeroCount != 0 )
  {
    msg( ERROR, "Zero count is  not '0'\n");
    return -1;
  }

  if (isVclNalUnit)
  {
    // Remove cabac_zero_word from payload if present
    int n = 0;

    while (it_write[-1] == 0x00)
    {
      it_write--;
      n++;
    }

    if (n > 0)
    {
      msg( NOTICE, "\nDetected %d instances of cabac_zero_word\n", n/2);
    }
  }

  nalUnitBuf.resize(it_write - nalUnitBuf.begin());

  return 0;
}

int VVDecImpl::xReadNalUnitHeader(InputNALUnit& nalu)
{
  InputBitstream& bs = nalu.getBitstream();

  nalu.m_forbiddenZeroBit   = bs.read(1);                 // forbidden zero bit
  nalu.m_nuhReservedZeroBit = bs.read(1);                 // nuh_reserved_zero_bit
  nalu.m_nuhLayerId         = bs.read(6);                 // nuh_layer_id

  if( nalu.m_nuhLayerId < 0)
  {
    msg( ERROR, "this needs to be adjusted for the reco yuv output\n");
    return -1;
  }
  if( nalu.m_nuhLayerId > 55 )
  {
    msg( ERROR, "The value of nuh_layer_id shall be in the range of 0 to 55, inclusive\n");
    return -1;
  }

  nalu.m_nalUnitType        = (NalUnitType) bs.read(5);   // nal_unit_type
  nalu.m_temporalId         = bs.read(3) - 1;             // nuh_temporal_id_plus1

  // only check these rules for base layer
  if (nalu.m_nuhLayerId == 0)
  {
    if ( nalu.m_temporalId )
    {
    }
    else
    {
      if( nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_STSA )
      {
        msg( ERROR, "hen NAL unit type is equal to STSA_NUT, TemporalId shall not be equal to 0\n" );
        return -1;
      }
    }
  }

  return 0;
}

int VVDecImpl::xHandleOutput( Picture* pcPic )
{
  if( pcPic )
  {
    // copy internal picture to external
    xAddPicture( pcPic );

    if ( m_bCreateNewPicBuf )
    {
      m_cDecLib.releasePicture( pcPic );
    }
  }

  return 0;
}



} // namespace
