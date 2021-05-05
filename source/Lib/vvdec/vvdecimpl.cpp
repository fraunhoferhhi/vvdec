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

#include <string>

#ifdef _WIN32
    #include <intrin.h>
#endif

#if defined( __linux__ )
#include <malloc.h>
#endif

#include "vvdecimpl.h"
#include "vvdec/version.h"
#include "DecoderLib/NALread.h"
#include "CommonLib/CommonDef.h"


namespace vvdec {

VVDecImpl::VVDecImpl()
{

}

VVDecImpl::~VVDecImpl()
{

}

int VVDecImpl::init( const vvdecParams& params )
{
  if( m_bInitialized ){ return VVDEC_ERR_INITIALIZE; }

#if ENABLE_TRACING
  if( !g_trace_ctx )
  {
    g_trace_ctx = tracing_init( sTracingFile, sTracingRule );
  }
  if( bTracingChannelsList && g_trace_ctx )
  {
    std::string sChannelsList;
    g_trace_ctx->getChannelsList( sChannelsList );
    msg( INFO, "\nAvailable tracing channels:\n\n%s\n", sChannelsList.c_str() );
  }
#endif
  
#ifdef TARGET_SIMD_X86
  switch( params.simd )
  {
  case VVDEC_SIMD_SCALAR: read_x86_extension( "SCALAR" ); break;
  case VVDEC_SIMD_SSE41 : read_x86_extension( "SSE41"  ); break;
  case VVDEC_SIMD_SSE42 : read_x86_extension( "SSE42"  ); break;
  case VVDEC_SIMD_AVX   : read_x86_extension( "AVX"    ); break;
  case VVDEC_SIMD_AVX2  : read_x86_extension( "AVX2"   ); break;
  case VVDEC_SIMD_AVX512: read_x86_extension( "AVX512" ); break;
  case VVDEC_SIMD_DEFAULT:
  default: break;
  }
#endif

  m_cDecLib = new DecLib();

  initROM();

  // create decoder class
#if RPR_YUV_OUTPUT
  m_cDecLib->create( params.threads, params.parseThreads, params.upscaleOutput );
#else
  m_cDecLib->create( params.threads, params.parseThreads );
#endif

  g_verbosity = MsgLevel( params.logLevel );
  g_context = this;

  // initialize decoder class
  m_cDecLib->setDecodedPictureHashSEIEnabled( (int) params.verifyPictureHash );
//  if (!m_outputDecodedSEIMessagesFilename.empty())
//  {
//    std::ostream &os=m_seiMessageFileStream.is_open() ? m_seiMessageFileStream : std::cout;
//    m_cDecLib.setDecodedSEIMessageOutputStream(&os);
//  }

  m_sDecoderCapabilities = m_cDecLib->getDecoderCapabilities();

  m_uiSeqNumber    = 0;
  m_uiSeqNumOutput = 0;
  m_bInitialized   = true;
  m_eState         = INTERNAL_STATE_INITIALIZED;

  return VVDEC_OK;
}

int VVDecImpl::uninit()
{
  if( !m_bInitialized ){ return VVDEC_ERR_INITIALIZE; }

  bool bFlushDecoder = true;
  while( bFlushDecoder)
  {
    vvdecFrame* frame= NULL;

    // flush the decoder
    int iRet = flush( &frame );
    if( iRet != 0 )  {  bFlushDecoder = false; }

    if( frame  )
    {
      // free picture memory
      objectUnref( frame );
    }
    else
    {
      bFlushDecoder = false;
      break;
    }
  };

  for( auto& frame : m_rcFrameList )
  {
    vvdec_frame_reset( &frame );
  }
  m_rcFrameList.clear();
  m_pcFrameNext = m_rcFrameList.end();


  for( auto& pic : m_pcLibPictureList )
  {
    m_cDecLib->releasePicture( pic );
  }
  m_pcLibPictureList.clear();


  for( auto& storage : m_cFrameStorageMap )
  {
    if( storage.second.isAllocated())
    {
      storage.second.freeStorage();
    }
  }
  m_cFrameStorageMap.clear();

  // destroy internal classes
  m_cDecLib->destroy();
  delete m_cDecLib;
  destroyROM();

#if defined( __linux__ )
  malloc_trim(0);
#endif

  m_bInitialized = false;
  m_eState       = INTERNAL_STATE_UNINITIALIZED;

  return VVDEC_OK;
}

void VVDecImpl::setLoggingCallback( vvdecLoggingCallback callback )
{
  g_msgFnc      = callback;
}

int VVDecImpl::decode( vvdecAccessUnit& rcAccessUnit, vvdecFrame** ppcFrame )
{
  if( !m_bInitialized )      { return VVDEC_ERR_INITIALIZE; }
  if( m_eState == INTERNAL_STATE_FINALIZED )        { m_cErrorString = "decoder already flushed, please reinit."; return VVDEC_ERR_RESTART_REQUIRED; }
  if( m_eState == INTERNAL_STATE_RESTART_REQUIRED ) { m_cErrorString = "restart required, please reinit."; return VVDEC_ERR_RESTART_REQUIRED; }
  if( m_eState == INTERNAL_STATE_FLUSHING )         { m_cErrorString = "decoder already received flush indication, please reinit."; return VVDEC_ERR_RESTART_REQUIRED; }

  if( m_eState == INTERNAL_STATE_INITIALIZED ){ m_eState = INTERNAL_STATE_DECODING; }

  if( !rcAccessUnit.payload )
  {
    return setAndRetErrorMsg( VVDEC_ERR_DEC_INPUT, "payload is null" );
  }

  if( rcAccessUnit.payloadSize <= 0  )
  {
    return setAndRetErrorMsg( VVDEC_ERR_DEC_INPUT, "payloadSize must be > 0" );
  }

  if( rcAccessUnit.payloadUsedSize > rcAccessUnit.payloadSize )
  {
    return setAndRetErrorMsg( VVDEC_ERR_DEC_INPUT, "payloadUsedSize must be <= payloadSize" );
  }

  int iRet= VVDEC_OK;

  try
  {
    InputNALUnit nalu;
    Picture * pcPic = nullptr;

    int iComrpPacketCnt = 0;

    if( rcAccessUnit.payloadUsedSize )
    {
      bool bStartCodeFound = false;
      std::vector<size_t> iStartCodePosVec;
      std::vector<size_t> iAUEndPosVec;
      std::vector<size_t> iStartCodeSizeVec;

      int pos = 0;
      while( pos+3 < rcAccessUnit.payloadUsedSize )
      {
        // no start code found
        if( pos >= rcAccessUnit.payloadUsedSize ) { THROW( "could not find a startcode" ); }

        int iFound = xRetrieveNalStartCode(&rcAccessUnit.payload[pos], 3);
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
          iFound = xRetrieveNalStartCode(&rcAccessUnit.payload[pos], 2);
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
        int iLastPos = rcAccessUnit.payloadUsedSize;
        while( rcAccessUnit.payload[iLastPos-1] == 0 && iLastPos > 0 )
        {
          iLastPos--;
        }
        iAUEndPosVec.push_back( iLastPos );

        // check if first AU begins on begin of payload (otherwise wrong input)
        if(!iStartCodePosVec.empty() && (iStartCodePosVec[0] != 3 && iStartCodePosVec[0] != 4 ))
        {
          m_cErrorString = "vvdecAccessUnit does not start with valid start code.";
          return VVDEC_ERR_DEC_INPUT;
        }

        // iterate over all AU´s
        for( size_t iAU = 0; iAU < iStartCodePosVec.size(); iAU++ )
        {
          std::vector<uint8_t>& nalUnit = nalu.getBitstream().getFifo();
          uint32_t uiNaluBytes = (uint32_t)iStartCodeSizeVec[iAU];
          for( size_t pos = iStartCodePosVec[iAU]; pos < iAUEndPosVec[iAU]; pos++ )
          {
            nalUnit.push_back( rcAccessUnit.payload[pos]);
            uiNaluBytes++;
          }

          if( uiNaluBytes )
          {
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

            if( rcAccessUnit.ctsValid ){  nalu.m_cts = rcAccessUnit.cts; }
            if( rcAccessUnit.dtsValid ){  nalu.m_dts = rcAccessUnit.dts; }
            nalu.m_rap = rcAccessUnit.rap;
            nalu.m_bits = uiNaluBytes*8;

            pcPic = m_cDecLib->decode( nalu );
            if( 0 != xHandleOutput( pcPic ))
            {
              iRet = VVDEC_ERR_UNSPECIFIED;
            }
          }

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
      pcPic = m_cDecLib->flushPic();
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
        m_uiSeqNumOutput = (*ppcFrame)->sequenceNumber;
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
    m_eState = INTERNAL_STATE_RESTART_REQUIRED;
    return VVDEC_ERR_RESTART_REQUIRED;
  }
  catch( std::exception& e )
  {
    //assert( 0 );
    std::stringstream css;
    css << "caught unknown exception " << e.what();
    m_cAdditionalErrorString = css.str();
    m_eState = INTERNAL_STATE_RESTART_REQUIRED;
    return VVDEC_ERR_RESTART_REQUIRED;
  }

  return iRet;
}

int VVDecImpl::flush( vvdecFrame** ppframe )
{
  if( !m_bInitialized ){ return VVDEC_ERR_INITIALIZE; }

  if( m_eState == INTERNAL_STATE_INITIALIZED )      { m_cErrorString = "decoder did not receive any data to decode, cannot flush"; return VVDEC_ERR_RESTART_REQUIRED; }
  if( m_eState == INTERNAL_STATE_FINALIZED )        { m_cErrorString = "decoder already flushed, please reinit."; return VVDEC_ERR_RESTART_REQUIRED; }
  if( m_eState == INTERNAL_STATE_RESTART_REQUIRED ) { m_cErrorString = "restart required, please reinit."; return VVDEC_ERR_RESTART_REQUIRED; }

  if( m_eState == INTERNAL_STATE_DECODING ) { m_eState = INTERNAL_STATE_FLUSHING; }

  int iRet= VVDEC_OK;

  // Flush decoder
  try
  {
    Picture * pcPic = nullptr;

    bool bContinue = true;
    while( bContinue )
    {
      pcPic = m_cDecLib->flushPic();
      if( 0 != xHandleOutput( pcPic ))
      {
        iRet = VVDEC_ERR_UNSPECIFIED;
      }

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
        *ppframe = nullptr;
         m_eState = INTERNAL_STATE_FINALIZED;
      }
      else
      {
        *ppframe = &( *m_pcFrameNext );
        m_uiSeqNumOutput = (*ppframe)->sequenceNumber;
        ++m_pcFrameNext;
      }
    }
    else
    {
      iRet = VVDEC_EOF;
      *ppframe = nullptr;
      m_eState = INTERNAL_STATE_FINALIZED;
    }
  }
  catch( std::overflow_error& e )
  {
    std::stringstream css;
    css << "caught overflow exception " << e.what();
    m_cAdditionalErrorString = css.str();
    m_eState = INTERNAL_STATE_RESTART_REQUIRED;
    return VVDEC_ERR_RESTART_REQUIRED;
  }
  catch( std::exception& e )
  {
    std::stringstream css;
    css << "caught unknown exception " << e.what();
    m_cAdditionalErrorString = css.str();
    m_eState = INTERNAL_STATE_RESTART_REQUIRED;
    return VVDEC_ERR_RESTART_REQUIRED;
  }

  if( 0 != iRet )
  {
    m_eState = INTERNAL_STATE_FINALIZED;
    return (int)VVDEC_EOF;
  }

  return iRet;
}

vvdecSEI* VVDecImpl::findFrameSei( vvdecSEIPayloadType payloadType, vvdecFrame *frame )
{
  if( !m_bInitialized ){ return nullptr; }

  if( nullptr == frame )
  {
    m_cErrorString = "findFrameSei: frame is null\n";
    return nullptr;
  }

  Picture* picture = nullptr;
  for ( auto& pic : m_pcLibPictureList )
  {
    if( frame->picAttributes != NULL )
    {
      if( frame->picAttributes->poc == (uint64_t)pic->poc )
      {
        picture = pic;
        break;
      }
    }
    else
    {
      if( frame->ctsValid && frame ->cts == pic->cts )
      {
        picture = pic;
        break;
      }
    }
  }

  if( picture == nullptr)
  {
    msg(VERBOSE, "findFrameSei: cannot find pictue in internal list.\n");
    return nullptr;
  }

  vvdecSEI *sei = nullptr;
  for( auto& s : picture->seiMessageList )
  {
    if( s->payloadType == payloadType )
    {
      sei = s;
    }
  }

  return sei;
}


int VVDecImpl::objectUnref( vvdecFrame* pcFrame )
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
      vvdec_frame_reset( &pic );
      break;
    }
  }

  if( bPicFound )
  {
    std::list<vvdecFrame>::iterator itFrame = m_rcFrameList.end();
    for( std::list<vvdecFrame>::iterator it = m_rcFrameList.begin(); it != m_rcFrameList.end(); it++ )
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

  uint32_t iErrors = m_cDecLib->getNumberOfChecksumErrorsDetected();
  return iErrors;
}

const char* VVDecImpl::getErrorMsg( int nRet )
{
  switch( nRet )
  {
  case VVDEC_OK :                  return vvdecErrorMsg[0]; break;
  case VVDEC_ERR_UNSPECIFIED:      return vvdecErrorMsg[1]; break;
  case VVDEC_ERR_INITIALIZE:       return vvdecErrorMsg[2]; break;
  case VVDEC_ERR_ALLOCATE:         return vvdecErrorMsg[3]; break;
  case VVDEC_ERR_DEC_INPUT:        return vvdecErrorMsg[4]; break;
  case VVDEC_NOT_ENOUGH_MEM:       return vvdecErrorMsg[5]; break;
  case VVDEC_ERR_PARAMETER:        return vvdecErrorMsg[6]; break;
  case VVDEC_ERR_NOT_SUPPORTED:    return vvdecErrorMsg[7]; break;
  case VVDEC_ERR_RESTART_REQUIRED: return vvdecErrorMsg[8]; break;
  case VVDEC_ERR_CPU:              return vvdecErrorMsg[9]; break;
  case VVDEC_TRY_AGAIN:            return vvdecErrorMsg[10]; break;
  case VVDEC_EOF:                  return vvdecErrorMsg[11]; break;
  default:                         return vvdecErrorMsg[12]; break;
  }
  return vvdecErrorMsg[12];
}

int VVDecImpl::setAndRetErrorMsg( int iRet, std::string errString  )
{
  if( !errString.empty() )
  {
    m_cErrorString = errString;
  }
  else
  {
    m_cErrorString = getErrorMsg(iRet);
  }
  return iRet;
}

const char* VVDecImpl::getVersionNumber()
{
  return VVDEC_VERSION;
}

const char* VVDecImpl::getDecoderInfo()
{
    m_sDecoderInfo  = "Fraunhofer VVC/H.266 Decoder VVdeC";
    m_sDecoderInfo += ", version ";
    m_sDecoderInfo += getVersionNumber();
    m_sDecoderInfo += " [";
    m_sDecoderInfo += m_sDecoderCapabilities;
    m_sDecoderInfo += "]";
    return m_sDecoderInfo.c_str();
}

vvdecNalType VVDecImpl::getNalUnitType ( vvdecAccessUnit& rcAccessUnit )
{
  vvdecNalType eNalType = VVC_NAL_UNIT_INVALID;

  if( rcAccessUnit.payload == nullptr ||
      rcAccessUnit.payloadSize < 3 ||
      rcAccessUnit.payloadUsedSize == 0 )
  {
    return eNalType;
  }

  unsigned char* pcBuf = rcAccessUnit.payload;
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
    eNalType = (vvdecNalType)nalUnitType;
  }

  return eNalType;
}

const char* VVDecImpl::getNalUnitTypeAsString( vvdecNalType t )
{
  if( (int) t >= (int)NAL_UNIT_INVALID || (int)t < 0)
  {
    return vvdecNalTypeNames[NAL_UNIT_INVALID];
  }

  return vvdecNalTypeNames[t];
}


bool VVDecImpl::isNalUnitSlice( vvdecNalType t )
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

#if RPR_YUV_OUTPUT
  const uint32_t orgWidth  = pcPic->cs->sps->getMaxPicWidthInLumaSamples() - confLeft - confRight;
  const uint32_t orgHeight = pcPic->cs->sps->getMaxPicHeightInLumaSamples() -  confTop  - confBottom;
#endif

  const BitDepths &bitDepths= pcPic->cs->sps->getBitDepths(); // use bit depths of first reconstructed picture.

  if ((uiWidth == 0) || (uiHeight == 0))
  {
    msg( ERROR, "objectUnref: %dx%d luma sample output picture!\n", uiWidth, uiHeight );
    return VVDEC_ERR_UNSPECIFIED;
  }

  bool bCreateStorage = (bitDepths.recon[0] == 8) ? true : false; // for 8bit output we need to copy the lib picture from unsigned short into unsigned char buffer

  // create a brand new picture object
  vvdecFrame cFrame;
  vvdec_frame_default( &cFrame );

  cFrame.sequenceNumber = m_uiSeqNumber;
  cFrame.cts      = pcPic->getCts();
  cFrame.ctsValid = true;

#if RPR_YUV_OUTPUT
  if( m_cDecLib->getUpscaledOutput() && ( uiWidth != orgWidth || uiHeight != orgHeight ) )
  {
    bCreateStorage = true;
    xCreateFrame ( cFrame, cPicBuf, orgWidth, orgHeight, bitDepths, bCreateStorage );
  }
  else
  {
    xCreateFrame ( cFrame, cPicBuf, uiWidth, uiHeight, bitDepths, bCreateStorage );
  }
#else
  xCreateFrame ( cFrame, cPicBuf, uiWidth, uiHeight, bitDepths, bCreateStorage );
#endif

  const int maxComponent = getNumberValidComponents( cPicBuf.chromaFormat );

  if( bCreateStorage )
  {
#if RPR_YUV_OUTPUT
    if( m_cDecLib->getUpscaledOutput() == (int)VVDEC_UPSCALING_RESCALE )
    {
      PelStorage upscaledPic;
      upscaledPic.create( cPicBuf.chromaFormat, Size( orgWidth, orgHeight ) );
      
      int xScale = ( ( uiWidth  << SCALE_RATIO_BITS ) + ( orgWidth  >> 1 ) ) / orgWidth;
      int yScale = ( ( uiHeight << SCALE_RATIO_BITS ) + ( orgHeight >> 1 ) ) / orgHeight;

      upscaledPic.rescaleBuf( cPicBuf, std::pair<int, int>( xScale, yScale ), conf, defDisp, bitDepths, pcPic->cs->sps->getHorCollocatedChromaFlag(), pcPic->cs->sps->getVerCollocatedChromaFlag() );
      
      // copy picture into target memory
      for( int comp=0; comp < maxComponent; comp++ )
      {
        const ComponentID compID      = ComponentID(comp);
        const uint32_t    csx         = getComponentScaleX(compID, cPicBuf.chromaFormat);
        const uint32_t    csy         = getComponentScaleY(compID, cPicBuf.chromaFormat);
        const CPelBuf     area        = upscaledPic.get(compID);
        unsigned int uiBytesPerSample = bitDepths.recon[0] > 8 ? 2 : 1;
        
        const ptrdiff_t   planeOffset = (confLeft >> csx) + (confTop >> csy) * area.stride;
        const unsigned char* pucOrigin   = (const unsigned char*)area.buf;
        const unsigned int width = std::min( area.width, cFrame.planes[comp].width );
        const unsigned int height = std::min( area.height, cFrame.planes[comp].height );
        
        copyComp(  pucOrigin + planeOffset,
                 cFrame.planes[comp].ptr, width, height,
                 area.stride<<1, cFrame.planes[comp].stride, uiBytesPerSample  );
      }

      upscaledPic.destroy();
    }
    else
#endif
    {
      // init memory
      for( int comp=0; comp < maxComponent; comp++ )
      {
        ::memset( cFrame.planes[comp].ptr,0, cFrame.planes[comp].width*cFrame.planes[comp].height*cFrame.planes[comp].bytesPerSample);
      }

      // copy picture into target memory
      for( int comp=0; comp < maxComponent; comp++ )
      {
        const ComponentID compID      = ComponentID(comp);
        const uint32_t    csx         = getComponentScaleX(compID, cPicBuf.chromaFormat);
        const uint32_t    csy         = getComponentScaleY(compID, cPicBuf.chromaFormat);
        const CPelBuf     area        = cPicBuf.get(compID);
        unsigned int uiBytesPerSample = bitDepths.recon[0] > 8 ? 2 : 1;
  
        const ptrdiff_t   planeOffset = (confLeft >> csx) + (confTop >> csy) * area.stride;
        const unsigned char* pucOrigin   = (const unsigned char*)area.buf;
        const unsigned int width = std::min( area.width, cFrame.planes[comp].width );
        const unsigned int height = std::min( area.height, cFrame.planes[comp].height );

       copyComp(  pucOrigin + planeOffset,
           cFrame.planes[comp].ptr, width, height,
           area.stride<<1, cFrame.planes[comp].stride, uiBytesPerSample  );
      }
    }
  }
  else
  {
    // use internal lib picture memory
    for( int comp=0; comp < maxComponent; comp++ )
    {
      const ComponentID compID      = ComponentID(comp);
      const uint32_t    csx         = getComponentScaleX(compID, cPicBuf.chromaFormat);
      const uint32_t    csy         = getComponentScaleY(compID, cPicBuf.chromaFormat);
      const CPelBuf     area        = cPicBuf.get(compID);
      const ptrdiff_t   planeOffset = (confLeft >> csx) + (confTop >> csy) * area.stride;

      //unsigned char* pucOrigin   = (unsigned char*)area.bufAt (0, 0).ptr;
      Pel* pucOrigin   = (Pel*)area.buf;

      cFrame.planes[comp].ptr = (unsigned char*)(pucOrigin + planeOffset);
    }
    m_pcLibPictureList.push_back( pcPic );
  }

  // set picture attributes

  cFrame.picAttributes = new vvdecPicAttributes();
  vvdec_picAttributes_default( cFrame.picAttributes );
  cFrame.picAttributes->poc           = pcPic->poc;
  cFrame.picAttributes->temporalLayer = pcPic->getTLayer();
  cFrame.picAttributes->bits          = pcPic->getNaluBits();

  cFrame.picAttributes->nalType       = (vvdecNalType)pcPic->eNalUnitType;

  cFrame.picAttributes->isRefPic = pcPic->referenced;

  if( pcPic->fieldPic )
  {
    cFrame.frameFormat = pcPic->topField ? VVDEC_FF_TOP_FIELD : VVDEC_FF_BOT_FIELD;
  }

  if ( !pcPic->slices.empty() )
  {
    switch( pcPic->slices.front()->getSliceType() )
    {
      case I_SLICE: cFrame.picAttributes->sliceType = VVDEC_SLICETYPE_I; break;
      case P_SLICE: cFrame.picAttributes->sliceType = VVDEC_SLICETYPE_P; break;
      case B_SLICE: cFrame.picAttributes->sliceType = VVDEC_SLICETYPE_B; break;
      default:      cFrame.picAttributes->sliceType = VVDEC_SLICETYPE_UNKNOWN; break;
    }

    if( pcPic->slices.front()->getSPS()->getVuiParametersPresentFlag() )
    {
      const VUI* vui = pcPic->slices.front()->getSPS()->getVuiParameters();
      if( vui != NULL )
      {
        cFrame.picAttributes->vui = new vvdecVui;
        cFrame.picAttributes->vui->aspectRatioInfoPresentFlag    = vui->getAspectRatioInfoPresentFlag();
        cFrame.picAttributes->vui->aspectRatioConstantFlag       = vui->getAspectRatioConstantFlag();
        cFrame.picAttributes->vui->nonPackedFlag                 = vui->getNonPackedFlag();
        cFrame.picAttributes->vui->nonProjectedFlag              = vui->getNonProjectedFlag();
        cFrame.picAttributes->vui->aspectRatioIdc                = vui->getAspectRatioIdc();
        cFrame.picAttributes->vui->sarWidth                      = vui->getSarWidth();
        cFrame.picAttributes->vui->sarHeight                     = vui->getSarHeight();
        cFrame.picAttributes->vui->colourDescriptionPresentFlag  = vui->getColourDescriptionPresentFlag();
        cFrame.picAttributes->vui->colourPrimaries               = vui->getColourPrimaries();
        cFrame.picAttributes->vui->transferCharacteristics       = vui->getTransferCharacteristics();
        cFrame.picAttributes->vui->matrixCoefficients            = vui->getMatrixCoefficients();
        cFrame.picAttributes->vui->progressiveSourceFlag         = vui->getProgressiveSourceFlag();
        cFrame.picAttributes->vui->interlacedSourceFlag          = vui->getInterlacedSourceFlag();
        cFrame.picAttributes->vui->chromaLocInfoPresentFlag      = vui->getChromaLocInfoPresentFlag();
        cFrame.picAttributes->vui->chromaSampleLocTypeTopField   = vui->getChromaSampleLocTypeTopField();
        cFrame.picAttributes->vui->chromaSampleLocTypeBottomField= vui->getChromaSampleLocTypeBottomField();
        cFrame.picAttributes->vui->chromaSampleLocType           = vui->getChromaSampleLocType();
        cFrame.picAttributes->vui->overscanInfoPresentFlag       = vui->getOverscanInfoPresentFlag();
        cFrame.picAttributes->vui->overscanAppropriateFlag       = vui->getOverscanAppropriateFlag();
        cFrame.picAttributes->vui->videoSignalTypePresentFlag    = vui->getVideoSignalTypePresentFlag();
        cFrame.picAttributes->vui->videoFullRangeFlag            = vui->getVideoFullRangeFlag();
      }
    }

    if( pcPic->slices.front()->getSPS()->getGeneralHrdParameters() )
    {
      const GeneralHrdParams* hrd = pcPic->slices.front()->getSPS()->getGeneralHrdParameters();
      if( hrd != NULL )
      {
        cFrame.picAttributes->hrd = new vvdecHrd;
        cFrame.picAttributes->hrd->numUnitsInTick                   = hrd->getNumUnitsInTick();
        cFrame.picAttributes->hrd->timeScale                        = hrd->getTimeScale();
        cFrame.picAttributes->hrd->generalNalHrdParamsPresentFlag   = hrd->getGeneralNalHrdParametersPresentFlag();
        cFrame.picAttributes->hrd->generalVclHrdParamsPresentFlag   = hrd->getGeneralVclHrdParametersPresentFlag();
        cFrame.picAttributes->hrd->generalSamePicTimingInAllOlsFlag = hrd->getGeneralSamePicTimingInAllOlsFlag();
        cFrame.picAttributes->hrd->tickDivisor                      = hrd->getTickDivisorMinus2()+2;
        cFrame.picAttributes->hrd->generalDecodingUnitHrdParamsPresentFlag = hrd->getGeneralDecodingUnitHrdParamsPresentFlag();
        cFrame.picAttributes->hrd->bitRateScale                     = hrd->getBitRateScale();
        cFrame.picAttributes->hrd->cpbSizeScale                     = hrd->getCpbSizeScale();
        cFrame.picAttributes->hrd->cpbSizeDuScale                   = hrd->getCpbSizeDuScale();
        cFrame.picAttributes->hrd->hrdCpbCnt                        = hrd->getHrdCpbCntMinus1()+1;
      }
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
      for( std::list<vvdecFrame>::iterator it = m_rcFrameList.begin(); it != m_rcFrameList.end(); it++ )
      {
        if(  (*it).sequenceNumber > m_uiSeqNumOutput )
        {
          m_pcFrameNext = it;
          break;
        }
      }
    }
  }

  m_uiSeqNumber++;

  if ( bCreateStorage   )
  {
    // release library picture storage, because picture has been copied into new storage class
    m_cDecLib->releasePicture( pcPic );
  }

  return 0;
}



int VVDecImpl::xCreateFrame( vvdecFrame& rcFrame, const CPelUnitBuf& rcPicBuf, uint32_t uiWidth, uint32_t uiHeight, const BitDepths& rcBitDepths, bool bCreateStorage )
{
  rcFrame.width       = uiWidth;
  rcFrame.height      = uiHeight;
  rcFrame.bitDepth    = 8;
  rcFrame.frameFormat = VVDEC_FF_PROGRESSIVE;
  rcFrame.bitDepth    = std::max( (uint32_t)rcBitDepths.recon[CHANNEL_TYPE_LUMA], rcFrame.bitDepth );

  rcFrame.planes[VVDEC_CT_Y].width          = uiWidth;
  rcFrame.planes[VVDEC_CT_Y].height         = uiHeight;
  rcFrame.planes[VVDEC_CT_Y].bytesPerSample = rcBitDepths.recon[CHANNEL_TYPE_LUMA] > 8 ? 2 : 1;
  rcFrame.planes[VVDEC_CT_Y].stride         = bCreateStorage  ? uiWidth*rcFrame.planes[VVDEC_CT_Y].bytesPerSample
                                                              : rcPicBuf.get(COMPONENT_Y).stride * rcFrame.planes[VVDEC_CT_Y].bytesPerSample;

  size_t nBufSize = 0;
  size_t nLSize   = rcFrame.planes[VVDEC_CT_Y].stride * uiHeight;
  size_t nCSize   = 0;

  switch( rcPicBuf.chromaFormat )
  {
    case CHROMA_400:
      {
        rcFrame.colorFormat = VVDEC_CF_YUV400_PLANAR;
        rcFrame.numPlanes = 1;

        rcFrame.planes[VVDEC_CT_U].width          = 0;
        rcFrame.planes[VVDEC_CT_U].height         = 0;
        rcFrame.planes[VVDEC_CT_U].stride         = 0;
        rcFrame.planes[VVDEC_CT_U].bytesPerSample = 0;

        rcFrame.planes[VVDEC_CT_V].width          = 0;
        rcFrame.planes[VVDEC_CT_V].height         = 0;
        rcFrame.planes[VVDEC_CT_V].stride         = 0;
        rcFrame.planes[VVDEC_CT_V].bytesPerSample = 0;

        nCSize = 0;
        nBufSize = nLSize; // we have to copy the packet into 8bit, because internal bitdepth is always Pel (unsigned short)
        break;
      }
    case CHROMA_420:
      {
        rcFrame.colorFormat    = VVDEC_CF_YUV420_PLANAR;
        rcFrame.numPlanes = 3;
        const unsigned int uiCWidth       = uiWidth>>1;
        const unsigned int uiCHeight      = uiHeight>>1;

        rcFrame.planes[VVDEC_CT_U].width          = uiCWidth;
        rcFrame.planes[VVDEC_CT_U].height         = uiCHeight;
        rcFrame.planes[VVDEC_CT_U].bytesPerSample = rcBitDepths.recon[CHANNEL_TYPE_CHROMA] > 8 ? 2 : 1;

        rcFrame.planes[VVDEC_CT_V].width          = uiCWidth;
        rcFrame.planes[VVDEC_CT_V].height         = uiCHeight;
        rcFrame.planes[VVDEC_CT_V].bytesPerSample = rcBitDepths.recon[CHANNEL_TYPE_CHROMA] > 8 ? 2 : 1;

        if( bCreateStorage )
        {
          rcFrame.planes[VVDEC_CT_U].stride         = uiCWidth * rcFrame.planes[CHANNEL_TYPE_CHROMA].bytesPerSample;
          rcFrame.planes[VVDEC_CT_V].stride         = uiCWidth * rcFrame.planes[CHANNEL_TYPE_CHROMA].bytesPerSample;
        }
        else
        {
          rcFrame.planes[VVDEC_CT_U].stride         = rcPicBuf.get(COMPONENT_Cb).stride * rcFrame.planes[CHANNEL_TYPE_CHROMA].bytesPerSample;
          rcFrame.planes[VVDEC_CT_V].stride         = rcPicBuf.get(COMPONENT_Cr).stride * rcFrame.planes[CHANNEL_TYPE_CHROMA].bytesPerSample;
        }

        nCSize = rcFrame.planes[VVDEC_CT_U].stride*uiCHeight;
        nBufSize = nLSize + (nCSize<<1);
        break;
      }
    case CHROMA_422:
      {
        rcFrame.colorFormat = VVDEC_CF_YUV422_PLANAR;
        rcFrame.numPlanes = 3;

        const unsigned int uiCWidth       = uiWidth>>1;
        const unsigned int uiCHeight      = uiHeight;

        rcFrame.planes[VVDEC_CT_U].width          = uiCWidth;
        rcFrame.planes[VVDEC_CT_U].height         = uiCHeight;
        rcFrame.planes[VVDEC_CT_U].bytesPerSample = rcBitDepths.recon[CHANNEL_TYPE_CHROMA] > 8 ? 2 : 1;

        rcFrame.planes[VVDEC_CT_V].width          = uiCWidth;
        rcFrame.planes[VVDEC_CT_V].height         = uiCHeight;
        rcFrame.planes[VVDEC_CT_V].bytesPerSample = rcBitDepths.recon[CHANNEL_TYPE_CHROMA] > 8 ? 2 : 1;

        if( bCreateStorage )
        {
          rcFrame.planes[VVDEC_CT_U].stride         = uiCWidth * rcFrame.planes[CHANNEL_TYPE_CHROMA].bytesPerSample;
          rcFrame.planes[VVDEC_CT_V].stride         = uiCWidth * rcFrame.planes[CHANNEL_TYPE_CHROMA].bytesPerSample;
        }
        else
        {
          rcFrame.planes[VVDEC_CT_U].stride         = rcPicBuf.get(COMPONENT_Cb).stride * rcFrame.planes[CHANNEL_TYPE_CHROMA].bytesPerSample;
          rcFrame.planes[VVDEC_CT_V].stride         = rcPicBuf.get(COMPONENT_Cr).stride * rcFrame.planes[CHANNEL_TYPE_CHROMA].bytesPerSample;
        }

        uint32_t nCSize = rcFrame.planes[VVDEC_CT_U].stride*uiCHeight;
        nBufSize = nLSize + (nCSize<<1);
        break;
      }
    case CHROMA_444:
      {
        rcFrame.colorFormat = VVDEC_CF_YUV444_PLANAR;
        rcFrame.numPlanes = 3;

        rcFrame.planes[VVDEC_CT_U].width          = uiWidth;
        rcFrame.planes[VVDEC_CT_U].height         = uiHeight;
        rcFrame.planes[VVDEC_CT_U].bytesPerSample = rcBitDepths.recon[CHANNEL_TYPE_CHROMA] > 8 ? 2 : 1;
        rcFrame.planes[VVDEC_CT_V].width          = uiWidth;
        rcFrame.planes[VVDEC_CT_V].height         = uiHeight;
        rcFrame.planes[VVDEC_CT_V].bytesPerSample = rcBitDepths.recon[CHANNEL_TYPE_CHROMA] > 8 ? 2 : 1;

        if( bCreateStorage )
        {
          rcFrame.planes[VVDEC_CT_U].stride         = uiWidth * rcFrame.planes[CHANNEL_TYPE_CHROMA].bytesPerSample;
          rcFrame.planes[VVDEC_CT_V].stride         = uiWidth * rcFrame.planes[CHANNEL_TYPE_CHROMA].bytesPerSample;
        }
        else
        {
          rcFrame.planes[VVDEC_CT_U].stride         = rcPicBuf.get(COMPONENT_Cb).stride * rcFrame.planes[CHANNEL_TYPE_CHROMA].bytesPerSample;
          rcFrame.planes[VVDEC_CT_V].stride         = rcPicBuf.get(COMPONENT_Cr).stride * rcFrame.planes[CHANNEL_TYPE_CHROMA].bytesPerSample;
        }

        nBufSize = nLSize*3;
        break;
      }
    default: break;
  }


  if( bCreateStorage )
  {
    if( nBufSize == 0 ){ return VVDEC_ERR_ALLOCATE; }

    FrameStorage frameStorage;
    frameStorage.allocateStorage( nBufSize );
    rcFrame.planes[VVDEC_CT_Y].ptr = frameStorage.getStorage();

    m_cFrameStorageMap.insert( frameStorageMapType( rcFrame.sequenceNumber, frameStorage));

    switch( rcPicBuf.chromaFormat )
    {
      case CHROMA_400:
          break;
      case CHROMA_420:
      case CHROMA_422:
      case CHROMA_444:
          rcFrame.planes[VVDEC_CT_U].ptr  = rcFrame.planes[VVDEC_CT_Y].ptr + nLSize;
          rcFrame.planes[VVDEC_CT_V].ptr  = rcFrame.planes[VVDEC_CT_Y].ptr + (nLSize + nCSize);
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
  int ret = 0;
  if( pcPic )
  {
    // copy internal picture to external
    if( 0 != xAddPicture( pcPic ))
    {
      ret = -1;
    }
  }

  return ret;
}

/*
 * return true if the frame is allocated in vvdecimpl, because it had to be converted
 * e.g. to convert internal 16->8bit samples
 * of   to scale RPR picture
*/
bool VVDecImpl::isFrameConverted( vvdecFrame* frame )
{
  frameStorageMap::iterator storageIter =  m_cFrameStorageMap.find( frame->sequenceNumber );
  if( storageIter != m_cFrameStorageMap.end() )
  {
    if( storageIter->second.isAllocated() )
    {
      return true;;
    }
  }

  return false;
}

void VVDecImpl::vvdec_picAttributes_default(vvdecPicAttributes *attributes)
{
  attributes->nalType         = VVC_NAL_UNIT_INVALID;     ///< nal unit type
  attributes->sliceType       = VVDEC_SLICETYPE_UNKNOWN;  ///< slice type (I/P/B) */
  attributes->isRefPic        = false;                    ///< reference picture
  attributes->temporalLayer   = 0;                        ///< temporal layer
  attributes->poc             = 0;                        ///< picture order count
  attributes->bits            = 0;                        ///< bits of the compr. image packet
  attributes->vui             = NULL;                     ///< if available, pointer to VUI (Video Usability Information)
  attributes->hrd             = NULL;                     ///< if available, pointer to HRD (Hypothetical Reference Decoder)
}

void VVDecImpl::vvdec_frame_default(vvdecFrame *frame)
{
  for( auto & p : frame->planes )
  {
    vvdec_plane_default( &p );
  }
  frame->numPlanes       = 0;                 ///< number of color components
  frame->width           = 0;                 ///< width of the luminance plane
  frame->height          = 0;                 ///< height of the luminance plane
  frame->bitDepth        = 0;                 ///< bit depth of input signal (8: depth 8 bit, 10: depth 10 bit  )
  frame->frameFormat     = VVDEC_FF_INVALID;  ///< interlace format (VVC_FF_PROGRESSIVE)
  frame->colorFormat     = VVDEC_CF_INVALID;  ///< color format     (VVC_CF_YUV420_PLANAR)
  frame->sequenceNumber  = 0;                 ///< sequence number of the picture
  frame->cts             = 0;                 ///< composition time stamp in TicksPerSecond (see HEVCEncoderParameter)
  frame->ctsValid        = false;             ///< composition time stamp valid flag (true: valid, false: CTS not set)
  frame->picAttributes   = NULL;              ///< pointer to PicAttribute that might be NULL, containing decoder side information
}

void VVDecImpl::vvdec_plane_default(vvdecPlane *plane)
{
  plane->ptr     = nullptr;      ///< pointer to plane buffer
  plane->width   = 0;            ///< width of the plane
  plane->height  = 0;            ///< height of the plane
  plane->stride  = 0;            ///< stride (width + left margin + right margins) of plane in samples
  plane->bytesPerSample = 1;     ///< offset to first sample in bytes
}

void VVDecImpl::vvdec_frame_reset(vvdecFrame *frame )
{
  bool bIsInternalLibStorage = true;
  frameStorageMap::iterator storageIter =  m_cFrameStorageMap.find( frame->sequenceNumber );
  if( storageIter != m_cFrameStorageMap.end() )
  {
    if( storageIter->second.isAllocated() )
    {
      storageIter->second.freeStorage();
      bIsInternalLibStorage = false;
    }

    m_cFrameStorageMap.erase (storageIter);
  }

  if( bIsInternalLibStorage )
  {
    // release internal picture memory
    for( std::list<Picture*>::iterator itLibPic = m_pcLibPictureList.begin(); itLibPic != m_pcLibPictureList.end();  itLibPic++ )
    {
      if( (*itLibPic)->cts == frame->cts )
      {
        m_cDecLib->releasePicture( *itLibPic );
        m_pcLibPictureList.erase( itLibPic );
        break;
      }
    }
  }

  if( frame->picAttributes )
  {
    if( frame->picAttributes->vui )
    {
      delete frame->picAttributes->vui;
    }

    if( frame->picAttributes->hrd )
    {
      delete frame->picAttributes->hrd;
    }

    delete frame->picAttributes;
    frame->picAttributes = NULL;
  }

  vvdec_frame_default( frame );
}

} // namespace
