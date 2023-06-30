/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2018-2023, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVdeC Authors.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

     * Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

     * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.

     * Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from this
     software without specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.


------------------------------------------------------------------------------------------- */

#pragma once

#include <string>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <algorithm>
#include <cctype>

#include "vvdec/vvdec.h"

namespace vvdecoderapp {

class CmdLineParser
{
public:
  /// Constructor
  CmdLineParser(){}

  /// Destructor
  virtual ~CmdLineParser() {}

  static void print_usage( std::string cApp, vvdecParams& rcParams, bool fullHelp )
  {
    std::cout <<   std::endl;
    std::cout <<   " Usage:  " << cApp << "  [param1] [pararm2] [...]" << std::endl;
    std::cout <<   std::endl;
    std::cout <<   "\t File input Options" << std::endl;
    std::cout <<   std::endl;
    std::cout <<   "\t\t [--bitstream,-b <str>      ] : bitstream input file" << std::endl;
    std::cout <<   "\t\t [--frames,-f  <int>        ] : max. frames to decode (default: -1 all frames)" << std::endl;
    std::cout <<   std::endl;
    std::cout <<   "\t YUV output options" << std::endl;
    std::cout <<   std::endl;
    std::cout <<   "\t\t [--output,-o  <str>        ] : yuv output file (default: not set)" << std::endl;
    if( fullHelp )
    {
      std::cout << "\t\t [--upscale,-uo             ] : set upscaling mode for RPR pictures(default: 0: off, 1: copy without rescaling, 2: rescale to target resolution)" << std::endl;
    }
    std::cout <<   "\t\t [--y4m                     ] : force y4m output (for pipe output; auto enable for .y4m output file extension)" << std::endl;
    std::cout <<   std::endl;
    std::cout <<   "\t Decoder Options" << std::endl;
    std::cout <<   std::endl;
    std::cout <<   "\t\t [--threads,-t  <int>       ] : number of threads (default: <= 0 auto detection )" << std::endl;
    if( fullHelp )
    {
      std::cout << "\t\t [--parsedelay,-p  <int>    ] : maximal number of frames to read before decoding (default: <= 0 auto detection )" << std::endl;
#if VVDEC_ARCH_X86
      std::cout << "\t\t [--simd <int>              ] : used simd extension (-1: auto, 0: scalar, 1: sse41, 2: sse42, 3: avx, 4: avx2) (default: -1)" << std::endl;
#elif VVDEC_ARCH_ARM
      std::cout << "\t\t [--simd <int>              ] : used simd extension (-1: auto, 0: scalar, 1: neon) (default: -1)" << std::endl;
#elif VVDEC_ARCH_WASM
      std::cout << "\t\t [--simd <int>              ] : used simd extension (-1: auto, 0: scalar, 1: wasm-simd) (default: -1)" << std::endl;
#endif
      std::cout << "\t\t [--errHandling,-eh <int>   ] : error handling flags ( 0: off, 1: try continue ) (default: " << rcParams.errHandlingFlags << ")" << std::endl;
    }
    std::cout <<   std::endl;
    if( fullHelp )
    {
      std::cout << "\t\t [--SEIDecodedPictureHash,-dph ] : enable handling of decoded picture hash SEI messages" << std::endl;
      std::cout << "\t\t [--CheckYuvMD5,-md5 <md5str>  ] : enable calculation of md5 hash over the full YUV output and check against the provided value." << std::endl;
      std::cout << std::endl;
#ifdef ENABLE_TRACING
      std::cout << "\t Debug Options" << std::endl;
      std::cout << std::endl;
      std::cout << "\t\t [--TraceFile,-tf <str>     ] : Tracing filename if ENABLE_TRACING: on, e.g. --TraceFile=trace.txt" << std::endl;
      std::cout << "\t\t [--TraceRule,-tr <str>     ] : Tracing rule if ENABLE_TRACING: on, e.g. --TraceRule=D_SYNTAX,D_HEADER:poc<=1" << std::endl;
      std::cout << std::endl;
#endif   // ENABLE_TRACING
    }
    std::cout << "\t General Options" << std::endl;
    std::cout << std::endl;
    if( fullHelp )
    {
      std::cout << "\t\t [--loops,-L  <int>         ] : number of decoder loops (default: 0, -1 endless)" << std::endl;
      std::cout << "\t\t [--extern                  ] : use extern picture buffer allocator, instead internal buffer manager" << std::endl;
    }
    std::cout <<   "\t\t [--verbosity,-v  <int>     ] : verbosity level (0: silent, 1: error, 2: warning, 3: info, 4: notice, 5: verbose, 6: debug) (default: " << ( int ) rcParams.logLevel << ")" << std::endl;
    std::cout <<   "\t\t [--version                 ] : show version" << std::endl;
    std::cout <<   "\t\t [--help,-h                 ] : show help" << std::endl;
    std::cout <<   "\t\t [--fullhelp                ] : show full help including expert options" << std::endl;
    std::cout <<   std::endl;
    std::cout <<   std::endl;
  }


  static int parse_command_line( int argc, char* argv[] , vvdecParams& rcParams, std::string& rcBitstreamFile, std::string& rcOutputFile,
                                 int& riFrames, int& riLoops, std::string& rcExpectYuvMD5, bool& useY4mFormat, bool &useExternAllocator,
                                 std::string& sTracingFile, std::string& sTracingRule, int& riPrintPicHash )
  {
#ifndef ENABLE_TRACING
    // ignore unused variables
    (void) sTracingFile;
    (void) sTracingRule;
#endif   // !ENABLE_TRACING

    int iRet = 0;
    /* Check command line parameters */
    int32_t  i_arg = 1;

    /* Check general options first */
    while( i_arg < argc )
    {
      if( (!strcmp( (const char*)argv[i_arg], "-v" )) || !strcmp( (const char*)argv[i_arg], "--verbosity" ) )
      {
        if( i_arg == argc-1 ){ fprintf( stderr, " - missing argument for: %s \n", argv[i_arg] ); return -1; }
        i_arg++;
        int iLogLevel = atoi( argv[i_arg++] );
        if( iLogLevel < 0 ) iLogLevel = 0;
        if( iLogLevel > (int)vvdecLogLevel::VVDEC_DETAILS ) iLogLevel = (int)vvdecLogLevel::VVDEC_DETAILS ;
        rcParams.logLevel = (vvdecLogLevel)iLogLevel;

        if( rcParams.logLevel > VVDEC_VERBOSE )
        {
          std::string cll;
          switch (rcParams.logLevel)
          {
            case VVDEC_SILENT : cll = "SILENT"; break;
            case VVDEC_ERROR  : cll = "ERROR"; break;
            case VVDEC_WARNING: cll = "WARNING"; break;
            case VVDEC_INFO   : cll = "INFO"; break;
            case VVDEC_NOTICE : cll = "NOTICE"; break;
            case VVDEC_VERBOSE: cll = "VERBOSE"; break;
            case VVDEC_DETAILS: cll = "DETAILS"; break;
            default: cll = "UNKNOWN"; break;
          };
          fprintf( stdout, "[verbosity] : %d - %s\n", (int)rcParams.logLevel, cll.c_str() );
        }
      }
      else if( (!strcmp( (const char*)argv[i_arg], "-h" )) || !strcmp( (const char*)argv[i_arg], "--help" ) )
      {
        i_arg++;
        iRet = 2;
        return iRet;
      }
      else if( !strcmp( ( const char* ) argv[i_arg], "--fullhelp" ) )
      {
        i_arg++;
        iRet = 3;
        return iRet;
      }
      else if( !strcmp( (const char*)argv[i_arg], "--version" ) )
      {
        i_arg++;
        iRet = 4;
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
        if( i_arg == argc-1 ){ fprintf( stderr, " - missing argument for: %s \n", argv[i_arg] ); return -1; }
        i_arg++;
        if( rcParams.logLevel > VVDEC_VERBOSE )
          fprintf( stdout, "[bitstream] input-file:    %s\n", argv[i_arg] );
        rcBitstreamFile = argv[i_arg++];
      }
      else if( (!strcmp( (const char*)argv[i_arg], "-o" )) || !strcmp( (const char*)argv[i_arg], "--output" ) ) /* Out: bitstream-file */
      {
        if( i_arg == argc-1 ){ fprintf( stderr, " - missing argument for: %s \n", argv[i_arg] ); return -1; }
        i_arg++;
        if( i_arg < argc && strlen( argv[i_arg] ) > 0 )
        {
          if( rcParams.logLevel > VVDEC_VERBOSE )
            fprintf( stdout, "[output] yuv-file:    %s\n", argv[i_arg] );
          rcOutputFile = argv[i_arg++];
        }
      }
      else if( (!strcmp( (const char*)argv[i_arg], "-uo" )) || !strcmp( (const char*)argv[i_arg], "--upscale" ) ) /* In: upscale */
      {
        i_arg++;

        rcParams.upscaleOutput = (vvdecRPRUpscaling) atoi( argv[i_arg++]);

        if( rcParams.logLevel > VVDEC_VERBOSE )
        {
          std::string scale;
          switch( rcParams.upscaleOutput )
          {
          case VVDEC_UPSCALING_OFF      : scale = "OFF"; break;
          case VVDEC_UPSCALING_COPY_ONLY: scale = "COPY_ONLY"; break;
          case VVDEC_UPSCALING_RESCALE  : scale = "RESCALE"; break;
          default: scale = "UNKNOWN"; break;
          };
          fprintf( stdout, "[upscale] : %s\n", scale.c_str() );
        }
      }
      else if( !strcmp( (const char*)argv[i_arg], "--y4m" ) )
      {
        i_arg++;
        useY4mFormat = true;

        if( i_arg < argc )
        {
          if( std::isdigit(argv[i_arg][0]))
          {
            i_arg++;
          }
        }
      }
      else if( !strcmp( (const char*)argv[i_arg], "--extern" ) )
      {
        i_arg++;
        useExternAllocator = true;

        if( i_arg < argc )
        {
          if( std::isdigit(argv[i_arg][0]))
          {
            i_arg++;
          }
        }
      }
      else if( (!strcmp( (const char*)argv[i_arg], "-f" )) || !strcmp( (const char*)argv[i_arg], "--frames" ) )
      {
        if( i_arg == argc-1 ){ fprintf( stderr, " - missing argument for: %s \n", argv[i_arg] ); return -1; }
        i_arg++;
        riFrames = atoi( argv[i_arg++] );
        if( rcParams.logLevel > VVDEC_VERBOSE )
          fprintf( stdout, "[frames] : %d\n", riFrames );
      }
      else if( (!strcmp( (const char*)argv[i_arg], "-t" )) || !strcmp( (const char*)argv[i_arg], "--threads" ) )
      {
        if( i_arg == argc-1 ){ fprintf( stderr, " - missing argument for: %s \n", argv[i_arg] ); return -1; }
        i_arg++;
        int iThreads = atoi( argv[i_arg++] );
        if( rcParams.logLevel > VVDEC_VERBOSE )
          fprintf( stdout, "[threads] : %d\n", iThreads );
        rcParams.threads = iThreads;
      }
      else if( (!strcmp( (const char*)argv[i_arg], "-p" )) || !strcmp( (const char*)argv[i_arg], "--parsedelay" ) )
      {
        if( i_arg == argc-1 ){ fprintf( stderr, " - missing argument for: %s \n", argv[i_arg] ); return -1; }
        i_arg++;
        int iThreads = atoi( argv[i_arg++] );
        if( rcParams.logLevel > VVDEC_VERBOSE )
          fprintf( stdout, "[parsedelay] : %d\n", iThreads );
        rcParams.parseThreads = iThreads;
      }
      else if( (!strcmp( (const char*)argv[i_arg], "-dph" )) || !strcmp( (const char*)argv[i_arg], "--SEIDecodedPictureHash" ) )
      {
        i_arg++;
        if( i_arg < argc && std::isdigit( argv[i_arg][0] ) )
        {
          riPrintPicHash = atoi( argv[i_arg] );
          i_arg++;
        }
        else
        {
          riPrintPicHash = 1;
        }

        if( riPrintPicHash <= 1 )
        {
          if( rcParams.logLevel > VVDEC_VERBOSE )
            fprintf( stdout, "[SEIDecodedPictureHash] : true\n" );
          rcParams.verifyPictureHash = true;
        }
      }
      else if( ( !strcmp( (const char*)argv[i_arg], "-md5" ) ) || !strcmp( (const char*)argv[i_arg], "--CheckYuvMD5" ) )
      {
        if( i_arg >= argc - 1 ) { fprintf( stderr, " - missing argument for: %s \n", argv[i_arg] ); return -1; }
        i_arg++;
        if( strlen( argv[i_arg] ) != 32 )
        {
          fprintf( stderr, " - the provided md5 hash to %s should be exactly 32 characters long\n", argv[i_arg - 1] );
          return -1;
        }

        rcExpectYuvMD5 = std::string( argv[i_arg++] );

        if( rcParams.logLevel > VVDEC_VERBOSE )
          fprintf( stdout, "[CheckYuvMD5] : %s\n", rcExpectYuvMD5.c_str() );
      }
      else if( (!strcmp( (const char*)argv[i_arg], "-L" )) || !strcmp( (const char*)argv[i_arg], "--loops" ) )
      {
        if( i_arg == argc-1 ){ fprintf( stderr, " - missing argument for: %s \n", argv[i_arg] ); return -1; }
        i_arg++;
        riLoops = atoi( argv[i_arg++] );
        if( rcParams.logLevel > VVDEC_VERBOSE )
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
      else if( !strcmp( (const char*)argv[i_arg], "--version" ) )
      {
        // already processed
        i_arg++;
      }
      else if( !strcmp( ( const char* ) argv[i_arg], "--simd" ) )
      {
        if( i_arg == argc-1 ){ fprintf( stderr, " - missing argument for: %s \n", argv[i_arg] ); return -1; }
        i_arg++;
        const int simd_arg = atoi( argv[i_arg++] );
        if( simd_arg < -1 || simd_arg > VVDEC_SIMD_MAX - 1 )
        {
          fprintf( stderr, " - unsupported simd mode. Should be between -1 and %i inclusive.\n", VVDEC_SIMD_MAX - 1 );
          return -1;
        }
        rcParams.simd = vvdecSIMD_Extension( simd_arg + 1 );

        if( rcParams.logLevel > VVDEC_VERBOSE )
        {
          const char* cll;
          switch( rcParams.simd )
          {
          case VVDEC_SIMD_DEFAULT: cll = "DEFAULT";   break;
          case VVDEC_SIMD_SCALAR:  cll = "SCALAR";    break;
#if VVDEC_ARCH_X86
          case VVDEC_SIMD_SSE41:   cll = "SSE41";     break;
          case VVDEC_SIMD_SSE42:   cll = "SSE42";     break;
          case VVDEC_SIMD_AVX:     cll = "AVX";       break;
          case VVDEC_SIMD_AVX2:    cll = "AVX2";      break;
#elif VVDEC_ARCH_ARM
          case VVDEC_SIMD_NEON:    cll = "NEON";      break;
#elif VVDEC_ARCH_WASM
          case VVDEC_SIMD_WASM:    cll = "WASM-SIMD"; break;
#else
          case VVDEC_SIMD_SIMDE_ANY:cll = "SIMDE-ANY"; break;
#endif
          default:                 return -1;
          };
          fprintf( stdout, "[simd] : %s\n", cll );
        }
      }
      else if( (!strcmp( argv[i_arg], "-eh" )) || !strcmp( argv[i_arg], "--errHandling" ) )
      {
        if( i_arg == argc-1 ){ fprintf( stderr, " - missing argument for: %s \n", argv[i_arg] ); return -1; }
        i_arg++;

        const int err_handle_flags = atoi( argv[i_arg++] );
        if( err_handle_flags < 0 || err_handle_flags > VVDEC_ERR_HANDLING_TRY_CONTINUE )
        {
          fprintf( stderr, " - unsupported error handling flags. Should be between 0 and %i.\n", VVDEC_ERR_HANDLING_TRY_CONTINUE );
          return -1;
        }

        rcParams.errHandlingFlags = vvdecErrHandlingFlags( err_handle_flags );
      }
#ifdef ENABLE_TRACING
      else if( !strcmp( (const char*)argv[i_arg], "--TraceFile" ) || !strcmp( (const char*)argv[i_arg], "-tf" ) )
      {
        sTracingFile = argv[++i_arg];
        i_arg++;
      }
      else if( !strcmp( (const char*)argv[i_arg], "--TraceRule" ) || !strcmp( (const char*)argv[i_arg], "-tr" ) )
      {
        sTracingRule = argv[++i_arg];
        i_arg++;
      }
#endif   // ENABLE_TRACING
      else
      {
        fprintf( stderr, " - unknown argument: %s \n", argv[i_arg++] );
        iRet = -1;
      }
    }

    return iRet;
  }

private:
  std::ofstream m_cOS;
};



} // namespace

