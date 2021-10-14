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
#include <thread>

#include "vvdec/vvdec.h"
#include "vvdec/version.h"

#include "CmdLineParser.h"

#include "vvdecHelper.h"
#include "MD5StreamBuf.h"

#if defined (_WIN32) || defined (WIN32) || defined (_WIN64) || defined (WIN64)
#include <io.h>
#include <fcntl.h>
#endif

/*! Prototypes */
int writeYUVToFile( std::ostream *f, vvdecFrame *frame, bool y4mFormat = false );
int writeYUVToFileInterlaced( std::ostream *f, vvdecFrame *topField, vvdecFrame *botField = nullptr, bool y4mFormat = false );

void msgFnc( void *, int level, const char* fmt, va_list args )
{
  vfprintf( level == 1 ? stderr : stdout, fmt, args );
}

void msgFncErr( void *, int, const char* fmt, va_list args )
{
  vfprintf( stderr, fmt, args );
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

  std::string cBitstreamFile = "";
  std::string cOutputFile    = "";
  int         iMaxFrames     = -1;
  int         iLoopCount     = 1;
  bool        y4mOutput      = false;
  std::string cExpectedYuvMD5;
  vvdecParams params;
  vvdec_params_default(&params);

  params.logLevel = VVDEC_INFO;

  if(  argc > 1 && (!strcmp( (const char*) argv[1], "--help" ) || !strcmp( (const char*) argv[1], "-help" )) )
  {
    vvdecoderapp::CmdLineParser::print_usage( cAppname, params );
    return 0;
  }

  int iRet = vvdecoderapp::CmdLineParser::parse_command_line(  argc, argv, params, cBitstreamFile, cOutputFile, iMaxFrames, iLoopCount, cExpectedYuvMD5, y4mOutput);
  if( iRet != 0 )
  {
    if( iRet == 2 )
    {
      vvdecoderapp::CmdLineParser::print_usage( cAppname, params);
      return 0;
    }
    else if( iRet == 3 )
    {
      std::cout << cAppname  << " version " << vvdec_get_version()<< std::endl;
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
    std::cerr << "vvdecapp [error]: failed to open bitstream file " << cBitstreamFile << std::endl;
    return -1;
  }

  bool writeStdout = false;
  
  // open output file
  std::ios * pStream{nullptr};
  std::fstream cRecFile;
  if( !cOutputFile.empty() )
  {
    if( !strcmp( cOutputFile.c_str(), "-" ) )
    {
      writeStdout = true;
#if defined (_WIN32) || defined (WIN32) || defined (_WIN64) || defined (WIN64)
      if( _setmode( _fileno( stdout ), _O_BINARY ) == -1 )
      {
        std::cerr << "vvdecapp [error]: failed to set stdout to binary mode" << std::endl;
        return -1;
      }
#endif
    }
    else
    {
      const char * s = strrchr(cOutputFile.c_str(), '.');
      if (s && !strcmp(s, ".y4m"))
      {
        y4mOutput = true;
      }

      cRecFile.open( cOutputFile.c_str(), std::fstream::binary | std::fstream::out );
      if( !cRecFile )
      {
        std::cerr << "vvdecapp [error]: failed to open ouptut file " << cOutputFile << std::endl;
        return -1;
      }
      pStream = &cRecFile;
    }
  }
  
  std::ostream * outStream = writeStdout ? &std::cout : dynamic_cast<std::ostream*>(pStream);
  std::ostream * logStream = writeStdout ? &std::cerr : &std::cout;

  if( !cOutputFile.empty() && (nullptr == outStream || !outStream) )
  {
    *logStream << "vvdecapp [error]: failed to open ouptut file " << cOutputFile << std::endl;
    return -1;
  }

  // stream buffer to directly calculate MD5 hash over the YUV output
  vvdecoderapp::MD5StreamBuf md5Buf( 1024 * 1024 );
  std::ostream md5Stream( &md5Buf );

  vvdecDecoder *dec = nullptr;

  //> decoding loop
  vvdecAccessUnit* accessUnit = vvdec_accessUnit_alloc();
  vvdec_accessUnit_alloc_payload( accessUnit, MAX_CODED_PICTURE_SIZE );

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
    vvdecFrame* pcFrame     = NULL;
    vvdecFrame* pcPrevField = NULL;

    if( iLoopCount > 1 )
    {
      *logStream << "-------------------------------------" << std::endl;
      *logStream << "begin decoder loop #" << iLoop << std::endl;
      *logStream << "-------------------------------------" << std::endl;
    }

    // initialize the decoder
    dec = vvdec_decoder_open( &params );
    if( nullptr == dec )
    {
      *logStream << "cannot init decoder" << std::endl;
      vvdec_accessUnit_free( accessUnit );
      return -1;
    }

    vvdec_set_logging_callback( dec, writeStdout ? msgFncErr : msgFnc );

    if( iLoop == 0 )
    {
      *logStream << vvdec_get_dec_information( dec ) << std::endl;
    }

    bool bFlushDecoder = false;

    cTPStartRun = std::chrono::steady_clock::now();
    cTPStart    = std::chrono::steady_clock::now();

    accessUnit->cts = 0; accessUnit->ctsValid = true;
    accessUnit->dts = 0; accessUnit->dtsValid = true;

    int iComprPics = 0;
    unsigned int uiFrames = 0;
    unsigned int uiFramesTmp = 0;
    unsigned int uiBitrate = 0;

    bool bTunedIn = false;
    unsigned int uiNoFrameAfterTuneInCount = 0;
    vvdecNalType eNalTypeSlice = VVC_NAL_UNIT_INVALID;
    bool bMultipleSlices = false;

    int iRead = 0;
    do
    {
      iRead = readBitstreamFromFile( &cInFile, accessUnit, false );
      //if( iRead > 0 )
      {
        vvdecNalType eNalType = vvdec_get_nal_unit_type( accessUnit );
        if( params.logLevel == VVDEC_DETAILS )
        {
          std::string cNal = getNalUnitTypeAsString( eNalType );
          *logStream << "  read nal " <<  cNal << " size " << accessUnit->payloadUsedSize << std::endl;
        }

        if( eNalType == VVC_NAL_UNIT_PH )
        {
          // picture header indicates multiple slices
          bMultipleSlices = true;
        }

        bool bIsSlice  = vvdec_is_nal_unit_slice( eNalType );
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

        iRet = vvdec_decode( dec, accessUnit, &pcFrame );
        if( bIsSlice )
        {
          accessUnit->cts++;
          accessUnit->dts++;
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
            if( params.logLevel >= VVDEC_VERBOSE ) *logStream << "more data needed to tune in" << std::endl;
            if( bTunedIn )
            {
              // after the first frame is returned, the decoder must always return a frame
              if( bIsSlice)
              {
                *logStream << "vvdecapp [error]: missing output picture!" << std::endl;
                uiNoFrameAfterTuneInCount++;
              }
            }
          }
        }
        else if( iRet != VVDEC_OK )
        {
          std::string cErr = vvdec_get_last_error(dec);
          std::string cAdditionalErr = vvdec_get_last_additional_error(dec);
          if( !cAdditionalErr.empty() )
          {
            *logStream << "vvdecapp [error]: decoding failed: " << cErr << " (" <<vvdec_get_error_msg( iRet ) << ")"
                       << " detail: " << vvdec_get_last_additional_error(dec) << std::endl;
          }
          else
          {
            *logStream << "vvdecapp [error]: decoding failed: " << cErr << " (" <<vvdec_get_error_msg( iRet ) << ")" << std::endl;
          }
          vvdec_accessUnit_free( accessUnit );
          return iRet;
        }

        if( NULL != pcFrame && pcFrame->ctsValid )
        {
          if (!bOutputInfoWritten)
          {
            if( params.logLevel >= VVDEC_INFO )
            {
              *logStream << "vvdecapp [info]: SizeInfo: " << pcFrame->width << "x" << pcFrame->height << " (" << pcFrame->bitDepth << "b)" << std::endl;
            }
            bOutputInfoWritten = true;
          }

          if( !bTunedIn )
          {
            bTunedIn = true;
          }

          uiFrames++;
          uiFramesTmp++;

          if( pcFrame->picAttributes )
          {
            uiBitrate += pcFrame->picAttributes->bits;
            (void)uiBitrate;
          }

#if 0 // just sample code to retrieve sei messages
          vvdecSEI *sei = vvdec_find_frame_sei( dec, VVDEC_CONTENT_LIGHT_LEVEL_INFO, pcFrame );
          if( sei )
          {
            vvdecSEIContentLightLevelInfo* p = reinterpret_cast<vvdecSEIContentLightLevelInfo *>(sei->payload);
            *logStream << "vvdecapp [info]: CONTENT_LIGHT_LEVEL_INFO: " << p->maxContentLightLevel << "," << p->maxPicAverageLightLevel << std::endl;
          }
#endif

          if( pcFrame->frameFormat == VVDEC_FF_PROGRESSIVE )
          {
            if( !cExpectedYuvMD5.empty() )
            {
              writeYUVToFile( &md5Stream, pcFrame );
            }
            if( cRecFile.is_open() || writeStdout )
            {
              if( 0 != writeYUVToFile( outStream, pcFrame, y4mOutput ) )
              {
                *logStream << "vvdecapp [error]: write of rec. yuv failed for picture seq. " <<  pcFrame->sequenceNumber << std::endl;
                vvdec_accessUnit_free( accessUnit );
                return iRet;
              }
            }
          }
          else if( pcFrame->frameFormat == VVDEC_FF_TOP_FIELD ||
                   pcFrame->frameFormat == VVDEC_FF_BOT_FIELD )
          {
            if( !pcPrevField )
            {
              pcPrevField = pcFrame;
            }
            else
            {
              if( !cExpectedYuvMD5.empty() )
              {
                writeYUVToFileInterlaced( &md5Stream, pcPrevField, pcFrame );
              }
              if( cRecFile.is_open() || writeStdout )
              {
                if( 0 != writeYUVToFileInterlaced( outStream, pcPrevField, pcFrame, y4mOutput ) )
                {
                  *logStream << "vvdecapp [error]: write of rec. yuv failed for picture seq. " <<  pcFrame->sequenceNumber << std::endl;
                  vvdec_accessUnit_free( accessUnit );
                  return iRet;
                }
              }
              vvdec_frame_unref( dec, pcPrevField );
              pcPrevField = nullptr;
            }
          }
          else
          {
            *logStream << "vvdecapp [error]: unsupported FrameFormat " << pcFrame->frameFormat << " for picture seq. " <<  pcFrame->sequenceNumber << std::endl;
            vvdec_accessUnit_free( accessUnit );
            return -1;
          }

          // free picture memory
          if( !pcPrevField || pcPrevField != pcFrame)
          {
            vvdec_frame_unref( dec, pcFrame );
          }
        }

        if( uiFrames && params.logLevel >= VVDEC_INFO )
        {
          cTPEnd = std::chrono::steady_clock::now();
          double dTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>((cTPEnd)-(cTPStart)).count();
          if( dTimeMs > 1000.0 )
          {
            *logStream << "vvdecapp [info]: decoded Frames: " << uiFrames << " Fps: " << uiFramesTmp << std::endl;
            cTPStart = std::chrono::steady_clock::now();
            uiFramesTmp = 0;
          }
        }
      }
    }
    while( iRead > 0 && !bFlushDecoder ); // end for frames

    //std::cout << "flush" << std::endl;

    // flush the decoder
    bFlushDecoder = true;
    while( bFlushDecoder)
    {
      vvdecFrame* pcFrame = NULL;

      // flush the decoder
      iRet = vvdec_flush( dec, &pcFrame );
      if( iRet != VVDEC_OK && iRet != VVDEC_EOF )
      {
        *logStream << "vvdecapp [error]: decoding failed (" << iRet << ")" << std::endl;  return iRet;
      }

      if( NULL != pcFrame  )
      {
        uiFrames++;

        if( pcFrame->frameFormat == VVDEC_FF_PROGRESSIVE )
        {
          if( !cExpectedYuvMD5.empty() )
          {
            writeYUVToFile( &md5Stream, pcFrame );
          }
          if( cRecFile.is_open() || writeStdout )
          {
            if( 0 != writeYUVToFile( outStream, pcFrame, y4mOutput ) )
            {
              *logStream << "vvdecapp [error]: write of rec. yuv failed for picture seq. " << pcFrame->sequenceNumber << std::endl;
              vvdec_accessUnit_free( accessUnit );
              return iRet;
            }
          }
        }
        else if( pcFrame->frameFormat == VVDEC_FF_TOP_FIELD ||
                 pcFrame->frameFormat == VVDEC_FF_BOT_FIELD )
        {
          if( !pcPrevField )
          {
            pcPrevField = pcFrame;
          }
          else
          {
            if( !cExpectedYuvMD5.empty() )
            {
              writeYUVToFileInterlaced( &md5Stream, pcPrevField, pcFrame );
            }
            if( cRecFile.is_open() || writeStdout )
            {
              if( 0 != writeYUVToFileInterlaced( outStream, pcPrevField, pcFrame, y4mOutput ) )
              {
                *logStream << "vvdecapp [error]: write of rec. yuv failed for picture seq. " << pcFrame->sequenceNumber << std::endl;
                vvdec_accessUnit_free( accessUnit );
                return iRet;
              }
            }
            vvdec_frame_unref( dec, pcPrevField );
            pcPrevField = nullptr;
          }
        }
        else
        {
          *logStream << "vvdecapp [error]: unsupported FrameFormat " << pcFrame->frameFormat << " for picture seq. " <<  pcFrame->sequenceNumber << std::endl;
          vvdec_accessUnit_free( accessUnit );
          return -1;
        }

        // free picture memory
        if( !pcPrevField || pcPrevField != pcFrame)
        {
          vvdec_frame_unref( dec, pcFrame );
        }

        if( params.logLevel >= VVDEC_INFO )
        {
          cTPEnd = std::chrono::steady_clock::now();
          double dTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>((cTPEnd)-(cTPStart)).count();
          if( dTimeMs > 1000.0 )
          {
            *logStream << "vvdecapp [info]: decoded Frames: " << uiFrames << " Fps: " << uiFramesTmp << std::endl;
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
      *logStream << "vvdecapp [error]: Decoder did not return " << uiNoFrameAfterTuneInCount << " pictures during decoding - came out too late" << std::endl;
    }

    cTPEndRun = std::chrono::steady_clock::now();
    double dTimeSec = std::chrono::duration_cast<std::chrono::milliseconds>((cTPEndRun)-(cTPStartRun)).count() / 1000.0;

    double dFps = dTimeSec ? ((double)uiFrames / dTimeSec) : uiFrames;
    dFpsPerLoopVec.push_back( dFps );
    if( params.logLevel >= VVDEC_INFO )
    {
      *logStream << "vvdecapp [info]: " << getTimePointAsString() << ": " << uiFrames << " frames decoded @ " << dFps << " fps (" << dTimeSec << " sec)\n" << std::endl;
    }

    iSEIHashErrCount = vvdec_get_hash_error_count(dec);
    if (iSEIHashErrCount )
    {
      *logStream << "vvdecapp [error]: MD5 checksum error ( " << iSEIHashErrCount << " errors )" << std::endl;
      vvdec_accessUnit_free( accessUnit );
      return iSEIHashErrCount;
    }

    if( iComprPics && !uiFrames )
    {
      *logStream << "vvdecapp [error]: read some input pictures (" << iComprPics << "), but no output was generated." << std::endl;
      vvdec_accessUnit_free( accessUnit );
      return -1;
    }

    if( !cExpectedYuvMD5.empty() )
    {
      std::transform( cExpectedYuvMD5.begin(), cExpectedYuvMD5.end(), cExpectedYuvMD5.begin(), (int ( * )( int ))std::tolower );

      const std::string yuvMD5 = md5Buf.finalizeHex();
      if( cExpectedYuvMD5 != yuvMD5 )
      {
        *logStream << "vvdecapp [error] full YUV output MD5 mismatch: " << cExpectedYuvMD5 << " != " << yuvMD5 << std::endl;
        vvdec_accessUnit_free( accessUnit );
        return -1;
      }
    }

    // un-initialize the decoder
    iRet = vvdec_decoder_close(dec);
    if( 0 != iRet )
    {
      *logStream << "vvdecapp [error]: cannot uninit decoder (" << iRet << ")" << std::endl;
      vvdec_accessUnit_free( accessUnit );
      return iRet;
    }

    if( iLoopCount < 0 || iLoop < iLoopCount-1 )
    {
      cInFile.clear();
      cInFile.seekg( 0, cInFile.beg );
      if(cInFile.bad() || cInFile.fail())
      {
        *logStream << "vvdecapp [error]: cannot seek to bitstream begin" << std::endl;
        vvdec_accessUnit_free( accessUnit );
        return -1;
      }
    }

    iLoop++;
    if( iLoopCount >= 0 && iLoop >= iLoopCount ){ bContinue = false;}
  }
  //< decoding loop finished

  // free memory of access unit
  vvdec_accessUnit_free( accessUnit );

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
    if( params.logLevel > VVDEC_SILENT )
    {
      *logStream <<"vvdecapp [info]: avg. fps for " << dFpsPerLoopVec.size() << " loops: " << dFpsSum/dFpsPerLoopVec.size() << " Hz " << std::endl;
    }
  }

  return 0;
}
