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

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <string.h>
#include <chrono>

#include "vvdec/vvdec.h"
#include "vvdec/version.h"

#include "CmdLineParser.h"

#include "vvdecHelper.h"

/*! Prototypes */
int writeYUVToFile( std::ostream *f, vvdec_Frame *frame );

//LogLevel g_verbosity = VVDEC_VERBOSE;

void msgFnc( void *, int level, const char* fmt, va_list args )
{
  vfprintf( level == 1 ? stderr : stdout, fmt, args );
}

int main( int argc, char* argv[] )
{
  std::string cAppname = argv[0];
  std::size_t iPos = (int)cAppname.find_last_of("/");
  if( std::string::npos != iPos )
  {
    cAppname = cAppname.substr(iPos+1 );
  }
  else
  {
    iPos = (int)cAppname.find_last_of("\\");
    if( std::string::npos != iPos )
    {
      cAppname = cAppname.substr(iPos+1 );
    }
  }

  std::cout << "Fraunhofer VVC Decoder " << cAppname  << " version: " << VVDEC_VERSION << std::endl;

  std::string cBitstreamFile  = "";
  std::string cOutputFile = "";
  int iMaxFrames=-1;
  int iLoopCount=1;
  vvdec_Params cVVDecParameter;

  cVVDecParameter.m_eLogLevel = VVDEC_INFO;

  if(  argc > 1 && (!strcmp( (const char*) argv[1], "--help" ) || !strcmp( (const char*) argv[1], "-help" )) )
  {
    vvdecoderapp::CmdLineParser::print_usage( cAppname, cVVDecParameter );
    return 0;
  }

  int iRet = vvdecoderapp::CmdLineParser::parse_command_line(  argc, argv, cVVDecParameter, cBitstreamFile, cOutputFile, iMaxFrames, iLoopCount );
  if( iRet != 0 )
  {
    if( iRet == 2 )
    {
      vvdecoderapp::CmdLineParser::print_usage( cAppname, cVVDecParameter);
      return 0;
    }

    std::cerr << "vvdecapp [error]: cannot parse command line. run vvdecapp --help to see available options" << std::endl;
    return -1;
  }

  if( cBitstreamFile.empty() )
  {
    std::cerr << "vvdecapp [error]: no bitstream file given. run vvdecapp --help to see available options" << std::endl;
    return -1;
  }

  // open input file
  std::ifstream cInFile( cBitstreamFile.c_str(), std::fstream::binary );
  if( !cInFile )
  {
    std::cout << "vvdecapp [error]: failed to open bitstream file " << cBitstreamFile << std::endl;
    return -1;
  }

  // open output file
  std::ios * pStream{nullptr};
  std::fstream cRecFile;
  if( !cOutputFile.empty() )
  {
    cRecFile.open( cOutputFile.c_str(), std::fstream::binary | std::fstream::out );
    if( !cRecFile )
    {
      std::cout << "vvdecapp [error]: failed to open ouptut file " << cOutputFile << std::endl;
      return -1;
    }
    pStream = &cRecFile;
  }
  std::ostream * outStream = dynamic_cast<std::ostream*>(pStream);
  if( !cOutputFile.empty() && (nullptr == outStream || !outStream) )
  {
    std::cout << "vvdecapp [error]: failed to open ouptut file " << cOutputFile << std::endl;
    return -1;
  }

  vvdec_decoder_t *dec = nullptr;

  //> decoding loop
  vvdec_AccessUnit cAccessUnit;
  cAccessUnit.m_pucBuffer = new unsigned char [ MAX_CODED_PICTURE_SIZE ];
  cAccessUnit.m_iBufSize  = MAX_CODED_PICTURE_SIZE;

  bool bOutputInfoWritten = false;

  std::chrono::steady_clock::time_point cTPStartRun;
  std::chrono::steady_clock::time_point cTPEndRun;

  std::chrono::steady_clock::time_point cTPStart;
  std::chrono::steady_clock::time_point cTPEnd;

  std::vector<double > dFpsPerLoopVec;

  int iSEIHashErrCount = 0;

  int iLoop = 0;
  bool bContinue = true;
  while ( bContinue )
  {
    vvdec_Frame* pcFrame = NULL;

    if( iLoopCount > 1 )
    {
      std::cout << "-------------------------------------" << std::endl;
      std::cout << "begin decoder loop #" << iLoop << std::endl;
      std::cout << "-------------------------------------" << std::endl;
    }

    // initialize the decoder
    dec = vvdec_decoder_open( &cVVDecParameter );
    if( nullptr == dec )
    {
      std::cout << "cannot init decoder" << std::endl;
      return -1;
    }

    vvdec_set_logging_callback( dec, msgFnc, nullptr, cVVDecParameter.m_eLogLevel );

    if( iLoop == 0 && cVVDecParameter.m_eLogLevel >= VVDEC_VERBOSE )
    {
      std::cout << vvdec_getDecoderInfo( dec ) << std::endl;
    }

    bool bFlushDecoder = false;

    cTPStartRun = std::chrono::steady_clock::now();
    cTPStart = std::chrono::steady_clock::now();

    cAccessUnit.m_uiCts = 0; cAccessUnit.m_bCtsValid = true;
    cAccessUnit.m_uiDts = 0; cAccessUnit.m_bDtsValid = true;

    int iComprPics = 0;
    unsigned int uiFrames = 0;
    unsigned int uiFramesTmp = 0;
    unsigned int uiBitrate = 0;

    bool bTunedIn = false;
    unsigned int uiNoFrameAfterTuneInCount = 0;
    NalType eNalTypeSlice = VVC_NAL_UNIT_INVALID;
    bool bMultipleSlices = false;

    int iRead = 0;
    do
    {
      iRead = readBitstreamFromFile( &cInFile, &cAccessUnit, false );
      //if( iRead > 0 )
      {
        NalType eNalType = vvdec_getNalUnitType( &cAccessUnit );
        if( cVVDecParameter.m_eLogLevel == VVDEC_DETAILS )
        {
          std::string cNal = getNalUnitTypeAsString( eNalType );
          std::cout << "  read nal " <<  cNal << " size " << cAccessUnit.m_iUsedSize << std::endl;
        }

        if( eNalType == VVC_NAL_UNIT_PH )
        {
          // picture header indicates multiple slices
          bMultipleSlices = true;
        }

        bool bIsSlice  = vvdec_isNalUnitSlice( eNalType );
        if( bIsSlice )
        {
          if( bMultipleSlices )
          {
            if( eNalTypeSlice == VVC_NAL_UNIT_INVALID )
            {
              // set current slice type and increment pic count
              iComprPics++;
              eNalTypeSlice = eNalType;
            }
            else
            {
              bIsSlice = false; // prevent cts/dts increase if not first slice
            }
          }
          else
          {
            iComprPics++;
          }
        }

        if( eNalTypeSlice != VVC_NAL_UNIT_INVALID &&
            eNalType != eNalTypeSlice )
        {
          eNalTypeSlice = VVC_NAL_UNIT_INVALID; // reset slice type
        }


        if( iMaxFrames > 0 && iComprPics >= iMaxFrames )
        {
          iRead = -1;
        }

        // call decode
        iRet = vvdec_decode( dec, &cAccessUnit, &pcFrame );
        if( bIsSlice )
        {
          cAccessUnit.m_uiCts++;
          cAccessUnit.m_uiDts++;
        }

        // check success
        if( iRet == VVDEC_EOF )
        {
          //std::cout << "flush" << std::endl;
          bFlushDecoder = true;
        }
        else if( iRet == VVDEC_TRY_AGAIN )
        {
          if( !bMultipleSlices )
          {
            if( cVVDecParameter.m_eLogLevel >= VVDEC_VERBOSE ) std::cout << "more data needed to tune in" << std::endl;
            if( bTunedIn )
            {
              // after the first frame is returned, the decoder must always return a frame
              if( bIsSlice)
              {
                std::cout << "vvdecapp [error]: missing output picture!" << std::endl;
                uiNoFrameAfterTuneInCount++;
              }
            }
          }
        }
        else if( iRet != VVDEC_OK )
        {
          std::string cAdditionalErr = vvdec_getLastAdditionalError(dec);
          if( !cAdditionalErr.empty() )
          {
            std::cout << "vvdecapp [error]: decoding failed: " << vvdec_getErrorMsg( iRet )
                      << " detail: " << vvdec_getLastAdditionalError(dec) << std::endl;
          }
          else
          {
            std::cout << "vvdecapp [error]: decoding failed: " << vvdec_getErrorMsg( iRet ) << std::endl;
          }
          return iRet;
        }

        if( NULL != pcFrame && pcFrame->m_bCtsValid)
        {
          if (!bOutputInfoWritten)
          {
            if( cVVDecParameter.m_eLogLevel >= VVDEC_INFO )
            {
              std::cout << "vvdecapp [info]: SizeInfo: " << pcFrame->m_uiWidth << "x" << pcFrame->m_uiHeight << " (" << pcFrame->m_uiBitDepth << "b)" << std::endl;
            }
            bOutputInfoWritten = true;
          }

          if( !bTunedIn )
          {
            bTunedIn = true;
          }

          uiFrames++;
          uiFramesTmp++;

          if( pcFrame->m_pcPicExtendedAttributes )
          {
            uiBitrate += pcFrame->m_pcPicExtendedAttributes->m_uiBits;
          }

#if 0
          vvdec_sei_message_t *sei = vvdec_find_frame_sei( dec, VVDEC_CONTENT_LIGHT_LEVEL_INFO, pcFrame );
          if( sei )
          {
            vvdec_sei_content_light_level_info_t* p = reinterpret_cast<vvdec_sei_content_light_level_info_t *>(sei->payload);
            std::cout << "vvdecapp [info]: CONTENT_LIGHT_LEVEL_INFO: " << p->m_maxContentLightLevel << "," << p->m_maxPicAverageLightLevel << std::endl;
          }
#endif

          if( cRecFile.is_open() )
          {
            if( 0 != writeYUVToFile( outStream, pcFrame ) )
            {
              std::cout << "vvdecapp [error]: write of rec. yuv failed for picture seq. " <<  pcFrame->m_uiSequenceNumber << std::endl;
              return iRet;
            }
          }

          // free picture memory
          vvdec_frame_unref( dec, pcFrame );
        }

        if( uiFrames && cVVDecParameter.m_eLogLevel >= VVDEC_INFO )
        {
          cTPEnd = std::chrono::steady_clock::now();
          double dTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>((cTPEnd)-(cTPStart)).count();
          if( dTimeMs > 1000.0 )
          {
            std::cout << "vvdecapp [info]: decoded Frames: " << uiFrames << " Fps: " << uiFramesTmp << std::endl;
            cTPStart = std::chrono::steady_clock::now();
            uiFramesTmp = 0;
          }
        }
      }
    }
    while( iRead > 0 && !bFlushDecoder ); // end for frames

    //std::cout << "flush" << std::endl;

    // flush the encoder
    bFlushDecoder = true;
    while( bFlushDecoder)
    {
      vvdec_Frame* pcFrame = NULL;

      // flush the decoder
      iRet = vvdec_flush( dec, &pcFrame );
      if( iRet != VVDEC_OK && iRet != VVDEC_EOF )
      {
        std::cout << "vvdecapp [error]: decoding failed (" << iRet << ")" << std::endl;  return iRet;
      }

      if( NULL != pcFrame  )
      {
        uiFrames++;

        if( cRecFile.is_open() )
        {
          if( 0 != writeYUVToFile( outStream, pcFrame ) )
          {
            std::cout << "vvdecapp [error]: write of rec. yuv failed for picture seq. " <<  pcFrame->m_uiSequenceNumber << std::endl;
            return iRet;
          }
        }

        // free picture memory
        vvdec_frame_unref( dec, pcFrame );

        if( cVVDecParameter.m_eLogLevel >= VVDEC_INFO )
        {
          cTPEnd = std::chrono::steady_clock::now();
          double dTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>((cTPEnd)-(cTPStart)).count();
          if( dTimeMs > 1000.0 )
          {
            std::cout << "vvdecapp [info]: decoded Frames: " << uiFrames << " Fps: " << uiFramesTmp << std::endl;
            cTPStart = std::chrono::steady_clock::now();
            uiFramesTmp = 0;
          }
        }
      }
      else
      {
        break;
      }
    };

    if( uiNoFrameAfterTuneInCount )
    {
      std::cout << "vvdecapp [error]: Decoder did not return " << uiNoFrameAfterTuneInCount << " pictures during decoding - came out too late" << std::endl;
    }

    cTPEndRun = std::chrono::steady_clock::now();
    double dTimeSec = std::chrono::duration_cast<std::chrono::milliseconds>((cTPEndRun)-(cTPStartRun)).count() / 1000.0;

    double dFps = dTimeSec ? ((double)uiFrames / dTimeSec) : uiFrames;
    dFpsPerLoopVec.push_back( dFps );
    if( cVVDecParameter.m_eLogLevel >= VVDEC_INFO )
      std::cout << "vvdecapp [info]: " << getTimePointAsString() << ": " << uiFrames << " frames decoded @ " << dFps << " fps (" << dTimeSec << " sec)\n" << std::endl;

    iSEIHashErrCount = vvdec_get_number_of_errors_PicHashSEI(dec);
    if (iSEIHashErrCount )
    {
      std::cout << "vvdecapp [error]: MD5 checksum error ( " << iSEIHashErrCount << " errors )" << std::endl;
      return iSEIHashErrCount;
    }

    if( iComprPics && !uiFrames )
    {
      std::cout << "vvdecapp [error]: read some input pictures (" << iComprPics << "), but no output was generated." << std::endl;
      return -1;
    }

    // un-initialize the decoder
    iRet = vvdec_decoder_close(dec);
    if( 0 != iRet )  { std::cout << "vvdecapp [error]: cannot uninit decoder (" << iRet << ")" << std::endl;  return iRet;  }

    if( iLoopCount < 0 || iLoop < iLoopCount-1 )
    {
      cInFile.clear();
      cInFile.seekg( 0, cInFile.beg );
      if(cInFile.bad() || cInFile.fail())
      {
        std::cout << "vvdecapp [error]: cannot seek to bitstream begin" << std::endl;  return -1;
      }
    }

    iLoop++;
    if( iLoopCount >= 0 && iLoop >= iLoopCount ){ bContinue = false;}

  }
  //< decoding loop finished

  // free memory of access unit
  delete [] cAccessUnit.m_pucBuffer;

  // close yuv output file
  if( !cOutputFile.empty() )
  {
    cRecFile.close();
  }

  cInFile.close();

  if( iLoopCount > 1 )
  {
    // print average fps over all decoder loops
    double dFpsSum  = .0;
    for( auto f : dFpsPerLoopVec )
    {
      dFpsSum += f;
    }
    if( cVVDecParameter.m_eLogLevel > VVDEC_SILENT )
      std::cout <<"vvdecapp [info]: avg. fps for " << dFpsPerLoopVec.size() << " loops: " << dFpsSum/dFpsPerLoopVec.size() << " Hz " << std::endl;
  }

  return 0;
}
