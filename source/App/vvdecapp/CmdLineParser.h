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
#if 1 //RPR_YUV_OUTPUT
          "\t\t [--UpscaledOutput,-uo  <int> ] : yuv output file (default: not set)\n"
#endif
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
#if 1 //RPR_YUV_OUTPUT
      else if( (!strcmp( (const char*)argv[i_arg], "-uo" )) || !strcmp( (const char*)argv[i_arg], "--UpscaledOutput" ) ) /* Out: bitstream-file */
      {
        i_arg++;
        int iUpscaledOutput = atoi( argv[i_arg++] );
        if( rcParams.m_eLogLevel > vvdec::LL_VERBOSE )
          fprintf( stdout, "[UpscaledOutput] : %d\n", iUpscaledOutput );
        rcParams.m_iUpscaledOutput = iUpscaledOutput;
      }
#endif
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

