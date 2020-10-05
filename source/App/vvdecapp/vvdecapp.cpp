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
int writeYUVToFile( std::ostream *f, vvdec::Frame *frame );

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
  vvdec::VVDecParameter cVVDecParameter;

  cVVDecParameter.m_eLogLevel = vvdec::LogLevel::LL_INFO;

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

  vvdec::VVDec cVVDec;

  //> decoding loop
  vvdec::AccessUnit cAccessUnit;
  cAccessUnit.m_pucBuffer = new unsigned char [ MAX_CODED_PICTURE_SIZE ];
  cAccessUnit.m_iBufSize = MAX_CODED_PICTURE_SIZE;

  bool bOutputInfoWritten = false;

  std::chrono::steady_clock::time_point cTPStart;
  std::chrono::steady_clock::time_point cTPEnd;

  std::vector<double > dFpsPerLoopVec;

  int iSEIHashErrCount = 0;

  int iLoop = 0;
  bool bContinue = true;
  while ( bContinue )
  {
    vvdec::Frame* pcFrame = NULL;

    if( iLoopCount > 1 )
    {
      std::cout << "-------------------------------------" << std::endl;
      std::cout << "begin decoder loop #" << iLoop << std::endl;
      std::cout << "-------------------------------------" << std::endl;
    }

    // initialize the decoder
    int iRet = cVVDec.init( cVVDecParameter );
    if( 0 != iRet )
    {
      std::cout << "cannot init decoder, ";
      switch( iRet )
      {
        case vvdec::VVDEC_ERR_CPU :       std::cout << "SSE 4.1 cpu support required."; break;
        case vvdec::VVDEC_ERR_PARAMETER : std::cout << "invalid parameter."; break;
        default : std::cout << "error " << iRet; break;
      };
      std::cout << std::endl;
      return -1;
    }

    if( iLoop == 0 && cVVDecParameter.m_eLogLevel >= vvdec::LL_VERBOSE )
    {
      std::cout << cVVDec.getDecoderInfo() << std::endl;
    }

    bool bFlushDecoder = false;

    cVVDec.clockStartTime();
    cTPStart = std::chrono::steady_clock::now();

    cAccessUnit.m_uiCts = 0; cAccessUnit.m_bCtsValid = true;
    cAccessUnit.m_uiDts = 0; cAccessUnit.m_bDtsValid = true;

    int iComprPics = 0;
    unsigned int uiFrames = 0;
    unsigned int uiFramesTmp = 0;
    unsigned int uiBitrate = 0;

    bool bTunedIn = false;
    unsigned int uiNoFrameAfterTuneInCount = 0;
    vvdec::NalType eNalTypeSlice = vvdec::VVC_NAL_UNIT_INVALID;
    bool bMultipleSlices = false;

    int iRead = 0;
    do
    {
      iRead = readBitstreamFromFile( &cInFile, &cAccessUnit, false );
      //if( iRead > 0 )
      {
        vvdec::NalType eNalType = vvdec::VVDec::getNalUnitType( cAccessUnit );
        if( cVVDecParameter.m_eLogLevel == vvdec::LL_DETAILS )
        {
          std::string cNal = getNalUnitTypeAsString( eNalType );
          std::cout << "  read nal " <<  cNal << " size " << cAccessUnit.m_iUsedSize << std::endl;
        }

        if( eNalType == vvdec::VVC_NAL_UNIT_PH )
        {
          // picture header indicates multiple slices
          bMultipleSlices = true;
        }

        bool bIsSlice  = vvdec::VVDec::isNalUnitSlice( eNalType );
        if( bIsSlice )
        {
          if( bMultipleSlices )
          {
            if( eNalTypeSlice == vvdec::VVC_NAL_UNIT_INVALID )
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

        if( eNalTypeSlice != vvdec::VVC_NAL_UNIT_INVALID &&
            eNalType != eNalTypeSlice )
        {
          eNalTypeSlice = vvdec::VVC_NAL_UNIT_INVALID; // reset slice type
        }


        if( iMaxFrames > 0 && iComprPics >= iMaxFrames )
        {
          iRead = -1;
        }

        // call decode
        iRet = cVVDec.decode( cAccessUnit, &pcFrame );

        if( bIsSlice )
        {
          cAccessUnit.m_uiCts++;
          cAccessUnit.m_uiDts++;
        }

        // check success
        if( iRet == vvdec::VVDEC_EOF )
        {
          //std::cout << "flush" << std::endl;
          bFlushDecoder = true;
        }
        else if( iRet == vvdec::VVDEC_TRY_AGAIN )
        {
          if( !bMultipleSlices )
          {
            if( cVVDecParameter.m_eLogLevel >= vvdec::LL_VERBOSE ) std::cout << "more data needed to tune in" << std::endl;
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
        else if( iRet != vvdec::VVDEC_OK )
        {
          std::string cAdditionalErr = cVVDec.getLastAdditionalError();
          if( !cAdditionalErr.empty() )
          {
            std::cout << "vvdecapp [error]: decoding failed: " << cVVDec.getErrorMsg( iRet )
                      << " detail: " << cVVDec.getLastAdditionalError() << std::endl;
          }
          else
          {
            std::cout << "vvdecapp [error]: decoding failed: " << cVVDec.getErrorMsg( iRet ) << std::endl;
          }
          return iRet;
        }

        if( NULL != pcFrame && pcFrame->m_bCtsValid)
        {
          if (!bOutputInfoWritten)
          {
            if( cVVDecParameter.m_eLogLevel >= vvdec::LL_INFO )
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

          if( cRecFile.is_open() )
          {
            if( 0 != writeYUVToFile( outStream, pcFrame ) )
            {
              std::cout << "vvdecapp [error]: write of rec. yuv failed for picture seq. " <<  pcFrame->m_uiSequenceNumber << std::endl;
              return iRet;
            }
          }

          // freevvdecmemory
          cVVDec.objectUnref( pcFrame );
        }

        if( uiFrames && cVVDecParameter.m_eLogLevel >= vvdec::LL_INFO )
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
      vvdec::Frame* pcFrame = NULL;

      // flush the decoder
      iRet = cVVDec.flush( &pcFrame );
      if( iRet != vvdec::VVDEC_OK && iRet != vvdec::VVDEC_EOF )
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
        cVVDec.objectUnref( pcFrame );

        if( cVVDecParameter.m_eLogLevel >= vvdec::LL_INFO )
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

    cVVDec.clockEndTime();
    double dTimeSec = cVVDec.clockGetTimeDiffMs() / 1000;
    double dFps = (double)uiFrames / dTimeSec;
    dFpsPerLoopVec.push_back( dFps );
    if( cVVDecParameter.m_eLogLevel >= vvdec::LL_INFO )
      std::cout << "vvdecapp [info]: " << getTimePointAsString() << ": " << uiFrames << " frames decoded @ " << dFps << " fps (" << dTimeSec << " sec)\n" << std::endl;

    iSEIHashErrCount = cVVDec.getNumberOfErrorsPictureHashSEI();
    if (iSEIHashErrCount )
    {
      std::cout << "vvdecapp [error]: MD5 checksum error ( " << iSEIHashErrCount << " errors )" << std::endl;
      return iSEIHashErrCount;
    }

    // un-initialize the decoder
    iRet = cVVDec.uninit();
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
    if( cVVDecParameter.m_eLogLevel > vvdec::LL_SILENT )
      std::cout <<"vvdecapp [info]: avg. fps for " << dFpsPerLoopVec.size() << " loops: " << dFpsSum/dFpsPerLoopVec.size() << " Hz " << std::endl;
  }

  return 0;
}
