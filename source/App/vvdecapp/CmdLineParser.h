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

#include <string>
#include <iostream>
#include <stdio.h>
#include <string.h>

#include "vvdec/vvdec.h"

namespace vvdecoderapp {

class CmdLineParser
{
public:
  /// Constructor
  CmdLineParser(){}

  /// Destructor
  virtual ~CmdLineParser() {}

  static void print_usage( std::string cApp, vvdec::VVDecParameter& rcParams )
  {
      printf( "\n Usage:  %s  [param1] [pararm2] [...] \n", cApp.c_str() );
      std::cout << "\n"
          "\t File input Options\n"
          "\t\t [--bitstream,-b <str>      ] : bitstream input file\n"
          "\t\t [--frames,-f  <int>        ] : max. frames to decode (default: -1 all frames) \n"
          "\n"
          "\t YUV output options\n"
          "\n"
          "\t\t [--output,-o  <str>        ] : yuv output file (default: not set)\n"
          "\n"
          "\t Decoder Options\n"
          "\n"
          "\t\t [--threads,-t  <int>       ] : number of threads (default: <= 0 auto detection )\n"
          "\t\t [--parsedelay,-p  <int>    ] : maximal number of frames to read before decoding (default: <= 0 auto detection )\n"
          "\n"
          "\t\t [--SEIDecodedPictureHash,-dph ] : enable handling of decoded picture hash SEI messages"
          "\n"
          "\t General Options\n"
          "\n"
          "\t\t [--loops,-L  <int>         ] : number of decoder loops (default: 0, -1 endless)\n"
          "\t\t [--verbosity,-v  <int>     ] : verbosity level (0: silent, 1: error, 2: warning, 3: info, 4: notice: 5, verbose, 6: debug) (default: " << (int)rcParams.m_eLogLevel << ")\n"
          "\t\t [--help,-h                 ] : show help\n"
          "\n" ;
      std::cout << std::endl;
  }


  static int parse_command_line( int argc, char* argv[] , vvdec::VVDecParameter& rcParams, std::string& rcBitstreamFile, std::string& rcOutputFile,
                                 int& riFrames, int& riLoops )
  {
    int iRet = 0;
    /* Check command line parameters */
    int32_t  i_arg = 1;

    /* Check general options firs*/
    while( i_arg < argc )
    {
      if( (!strcmp( (const char*)argv[i_arg], "-v" )) || !strcmp( (const char*)argv[i_arg], "--verbosity" ) )
      {
        i_arg++;
        int iLogLevel = atoi( argv[i_arg++] );
        if( iLogLevel < 0 ) iLogLevel = 0;
        if( iLogLevel > (int)vvdec::LogLevel::LL_DETAILS ) iLogLevel = (int)vvdec::LogLevel::LL_DETAILS ;
        rcParams.m_eLogLevel = (vvdec::LogLevel)iLogLevel;

        if( rcParams.m_eLogLevel > vvdec::LL_VERBOSE )
        {
          std::string cll;
          switch (rcParams.m_eLogLevel)
          {
            case vvdec::LL_SILENT : cll = "SILENT"; break;
            case vvdec::LL_ERROR  : cll = "ERROR"; break;
            case vvdec::LL_WARNING: cll = "WARNING"; break;
            case vvdec::LL_INFO   : cll = "INFO"; break;
            case vvdec::LL_NOTICE : cll = "NOTICE"; break;
            case vvdec::LL_VERBOSE: cll = "VERBOSE"; break;
            case vvdec::LL_DETAILS: cll = "DETAILS"; break;
            default: cll = "UNKNOWN"; break;
          };
          fprintf( stdout, "[verbosity] : %d - %s\n", (int)rcParams.m_eLogLevel, cll.c_str() );
        }
      }
      else if( (!strcmp( (const char*)argv[i_arg], "-h" )) || !strcmp( (const char*)argv[i_arg], "--help" ) )
      {
        i_arg++;
        iRet = 2;
        return iRet;
      }
      else
      {
        i_arg++;
      }
    }


    i_arg = 1;
    while( i_arg < argc )
    {
      if( (!strcmp( (const char*)argv[i_arg], "-b" )) || !strcmp( (const char*)argv[i_arg], "--bitstream" ) ) /* In: input-file */
      {
        i_arg++;
        if( rcParams.m_eLogLevel > vvdec::LL_VERBOSE )
          fprintf( stdout, "[bitstream] input-file:    %s\n", argv[i_arg] );
        rcBitstreamFile = argv[i_arg++];
      }
      else if( (!strcmp( (const char*)argv[i_arg], "-o" )) || !strcmp( (const char*)argv[i_arg], "--output" ) ) /* Out: bitstream-file */
      {
        i_arg++;
        if( i_arg < argc && strlen( argv[i_arg] ) > 0 )
        {
          if( rcParams.m_eLogLevel > vvdec::LL_VERBOSE )
            fprintf( stdout, "[output] yuv-file:    %s\n", argv[i_arg] );
          rcOutputFile = argv[i_arg++];
        }
      }
      else if( (!strcmp( (const char*)argv[i_arg], "-f" )) || !strcmp( (const char*)argv[i_arg], "--frames" ) )
      {
        i_arg++;
        riFrames = atoi( argv[i_arg++] );
        if( rcParams.m_eLogLevel > vvdec::LL_VERBOSE )
          fprintf( stdout, "[frames] : %d\n", riFrames );
      }
      else if( (!strcmp( (const char*)argv[i_arg], "-t" )) || !strcmp( (const char*)argv[i_arg], "--threads" ) )
      {
        i_arg++;
        int iThreads = atoi( argv[i_arg++] );
        if( rcParams.m_eLogLevel > vvdec::LL_VERBOSE )
          fprintf( stdout, "[threads] : %d\n", iThreads );
        rcParams.m_iThreads = iThreads;
      }
      else if( (!strcmp( (const char*)argv[i_arg], "-p" )) || !strcmp( (const char*)argv[i_arg], "--parsedelay" ) )
      {
        i_arg++;
        int iThreads = atoi( argv[i_arg++] );
        if( rcParams.m_eLogLevel > vvdec::LL_VERBOSE )
          fprintf( stdout, "[parsedelay] : %d\n", iThreads );
        rcParams.m_iParseThreads = iThreads;
      }
      else if( (!strcmp( (const char*)argv[i_arg], "-dph" )) || !strcmp( (const char*)argv[i_arg], "--SEIDecodedPictureHash" ) )
      {
        i_arg++;
        if( rcParams.m_eLogLevel > vvdec::LL_VERBOSE )
          fprintf( stdout, "[SEIDecodedPictureHash] : true\n" );
        rcParams.m_bDecodedPictureHashSEIEnabled = true;
      }
      else if( (!strcmp( (const char*)argv[i_arg], "-L" )) || !strcmp( (const char*)argv[i_arg], "--loops" ) )
      {
        i_arg++;
        riLoops = atoi( argv[i_arg++] );
        if( rcParams.m_eLogLevel > vvdec::LL_VERBOSE )
          fprintf( stdout, "[loops] : %d\n", riLoops );
      }
      else if( (!strcmp( (const char*)argv[i_arg], "-v" )) || !strcmp( (const char*)argv[i_arg], "--verbosity" ) )
      {
        // already processed
        i_arg++;
        i_arg++;
      }
      else if( (!strcmp( (const char*)argv[i_arg], "-h" )) || !strcmp( (const char*)argv[i_arg], "--help" ) )
      {
        // already processed
        i_arg++;
      }
      else
      {
        fprintf( stderr, " - IGNORED: %s \n", argv[i_arg++] );
      }
    }

    return iRet;
  }

private:
  std::ofstream m_cOS;
};



} // namespace

