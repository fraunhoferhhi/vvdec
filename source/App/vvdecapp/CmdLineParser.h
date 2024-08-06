/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2018-2024, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVdeC Authors.
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

#include <cctype>
#include <cstdio>
#include <cstring>
#include <cassert>
#include <string>
#include <iostream>
#include <algorithm>

#include "vvdec/vvdec.h"

namespace vvdecoderapp
{

enum RPRUpscaling
{
  UPSCALING_OFF       = 0,   // no RPR scaling
  UPSCALING_COPY_ONLY = 1,   // copy picture into target resolution only
  UPSCALING_RESCALE   = 2    // auto rescale RPR pictures into target resolution
};

struct AppOutputParams
{
  RPRUpscaling upscaleOutput = UPSCALING_OFF;
  bool         y4mOutput     = false;
};

class CmdLineParser
{
  int32_t m_iArg = 0;
  int     m_argc = 0;
  char**  m_argv = nullptr;

  // parse a parameter with the corresponding argument
  template<class TOut>
  bool parse_param( std::initializer_list<const char*> paramNames, TOut& outputVar, bool argOptional = false, const TOut optionalDefault = {} )
  {
    if( m_iArg >= m_argc )
    {
      return false;
    }

    const std::string currArg( m_argv[m_iArg] );
    if( std::any_of( paramNames.begin(), paramNames.end(),
                     [&]( const char* name ) { return currArg == name; } ) )
    {
      ++m_iArg;

      if( m_iArg < m_argc && parse_param_arg( outputVar ) )
      {
        return true;
      }
      if( argOptional )
      {
        outputVar = optionalDefault;
        return true;
      }
      if( std::is_same<bool, TOut>::value )
      {
        outputVar = true;   // default value for bool always true, if present
        return true;
      }

      fprintf( stderr, " - missing argument for: %s \n", m_argv[m_iArg - 1] );
      throw MissingArgumentException();
    }
    return false;
  }

  // parse boolean arguments
  bool parse_param_arg( bool& outputVar )
  {
    outputVar = true;   // boolean always defaults to true
    if( strlen( m_argv[m_iArg] ) >= 1 && std::isdigit( m_argv[m_iArg][0] ) )
    {
      outputVar = !!atoi( m_argv[m_iArg] );
      ++m_iArg;
      return true;
    }
    return true;
  }

  // parse string arguments
  bool parse_param_arg( std::string& outputVar )
  {
    outputVar = std::string( m_argv[m_iArg] );
    ++m_iArg;
    return true;
  }

  // parse signed int arguments
  bool parse_param_arg( int& outputVar )
  {
    const size_t argStrLen = strlen( m_argv[m_iArg] );
    if( ( argStrLen >= 1 && std::isdigit( m_argv[m_iArg][0] ) )                                    // positive number
        || ( argStrLen >= 2 && m_argv[m_iArg][0] == '-' && std::isdigit( m_argv[m_iArg][1] ) ) )   // negative number
    {
      outputVar = atoi( m_argv[m_iArg] );
      ++m_iArg;
      return true;
    }

    return false;
  }

  // parse unsigned int arguments
  bool parse_param_arg( unsigned int& outputVar )
  {
    if( strlen( m_argv[m_iArg] ) >= 1 && std::isdigit( m_argv[m_iArg][0] ) )
    {
      outputVar = atoi( m_argv[m_iArg] );
      ++m_iArg;
      return true;
    }
    return false;
  }

public:
  /// Constructor
  CmdLineParser() = default;
  /// Destructor
  ~CmdLineParser() = default;

  static void print_usage( std::string cApp, vvdecParams& rcParams, bool fullHelp )
  {
    // clang-format off
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
      std::cout << "\t\t [--filmGrain,-fg <int>     ] : set film grain synthesis using Film Grain Charactersitics SEI (default: 1, off: 0, on: 1)" << std::endl;
    }
    std::cout <<   "\t\t [--y4m                     ] : force y4m output (for pipe output; auto enable for .y4m output file extension)" << std::endl;
    std::cout <<   std::endl;
    std::cout <<   "\t Decoder Options" << std::endl;
    std::cout <<   std::endl;
    std::cout <<   "\t\t [--threads,-t  <int>       ] : number of threads (default: <= 0 auto detection )" << std::endl;
    if( fullHelp )
    {
      std::cout << "\t\t [--parsedelay,-p  <int>    ] : maximal number of frames to read before decoding (default: <= 0 auto detection )" << std::endl;
#if defined( VVDEC_ARCH_X86 )
      std::cout << "\t\t [--simd <int>              ] : used simd extension (-1: auto, 0: scalar, 1: sse41, 2: sse42, 3: avx, 4: avx2) (default: -1)" << std::endl;
#elif defined( VVDEC_ARCH_ARM)
      std::cout << "\t\t [--simd <int>              ] : used simd extension (-1: auto, 0: scalar, 1: neon) (default: -1)" << std::endl;
#elif defined(VVDEC_ARCH_WASM)
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
    // clang-format on
  }

  int parse_command_line( int              argc,
                          char*            argv[],
                          vvdecParams&     rcParams,
                          std::string&     rcBitstreamFile,
                          std::string&     rcOutputFile,
                          int&             riFrames,
                          int&             riLoops,
                          std::string&     rcExpectYuvMD5,
                          AppOutputParams& appParams,
                          bool&            useExternAllocator,
                          std::string&     sTracingFile,
                          std::string&     sTracingRule,
                          int&             riPrintPicHash )
  {
#ifndef ENABLE_TRACING
    // ignore unused variables
    (void) sTracingFile;
    (void) sTracingRule;
#endif   // !ENABLE_TRACING

    /* Check command line parameters */
    m_iArg = 1;
    m_argc = argc;
    m_argv = argv;

    /* Check general options first */
    while( m_iArg < argc )
    {
      bool     _dummy   = false;
      unsigned logLevel = 0;
      if( parse_param( { "-v", "--verbosity" }, logLevel ) )
      {
        rcParams.logLevel = std::min( (vvdecLogLevel) logLevel, VVDEC_DETAILS );

        if( rcParams.logLevel > VVDEC_VERBOSE )
        {
          const char* cll;
          switch( rcParams.logLevel )
          {
            // clang-format off
            case VVDEC_SILENT : cll = "SILENT";  break;
            case VVDEC_ERROR  : cll = "ERROR";   break;
            case VVDEC_WARNING: cll = "WARNING"; break;
            case VVDEC_INFO   : cll = "INFO";    break;
            case VVDEC_NOTICE : cll = "NOTICE";  break;
            case VVDEC_VERBOSE: cll = "VERBOSE"; break;
            case VVDEC_DETAILS: cll = "DETAILS"; break;
            default:            cll = "UNKNOWN"; break;
            // clang-format on
          };
          fprintf( stdout, "[verbosity] : %d - %s\n", (int) rcParams.logLevel, cll );
        }
      }
      else if( parse_param( { "-h", "--help" }, _dummy ) )
      {
        return 2;
      }
      else if( parse_param( { "--fullhelp", "--full-help" }, _dummy ) )
      {
        return 3;
      }
      else if( parse_param( { "--version" }, _dummy ) )
      {
        return 4;
      }
      else
      {
        m_iArg++;
      }
    }

    // restart from the beginning to parse the remainig options
    m_iArg = 1;
    while( m_iArg < argc )
    {
      int      simd_arg         = 0;
      int      err_handle_flags = 0;
      int      upscale_output   = 0;
      unsigned logLevel         = 0;
      if( parse_param( { "-b", "--bitstream" }, rcBitstreamFile ) ) /* In: input-file */
      {
        if( rcParams.logLevel > VVDEC_VERBOSE )
          fprintf( stdout, "[bitstream] input-file:    %s\n", argv[m_iArg] );
      }
      else if( parse_param( { "-o", "--output" }, rcOutputFile ) ) /* Out: bitstream-file */
      {
        if( rcParams.logLevel > VVDEC_VERBOSE )
          fprintf( stdout, "[output] yuv-file:    %s\n", argv[m_iArg] );
      }
      else if( parse_param( { "-uo", "--upscale" }, upscale_output ) ) /* In: upscale */
      {
        appParams.upscaleOutput = RPRUpscaling( upscale_output );
        if( rcParams.logLevel > VVDEC_VERBOSE )
        {
          const char* scale;
          switch( appParams.upscaleOutput )
          {
            // clang-format off
            case UPSCALING_OFF      : scale = "OFF";       break;
            case UPSCALING_COPY_ONLY: scale = "COPY_ONLY"; break;
            case UPSCALING_RESCALE  : scale = "RESCALE";   break;
            default                 : scale = "UNKNOWN";   break;
            // clang-format on
          };
          fprintf( stdout, "[upscale] : %s\n", scale );
        }
      }
      else if( parse_param( { "-fg", "--filmGrain" }, rcParams.filmGrainSynthesis ) ) {}
      else if( parse_param( { "--y4m" }, appParams.y4mOutput ) ) {}
      else if( parse_param( { "--extern" }, useExternAllocator ) ) {}
      else if( parse_param( { "-f", "--frames" }, riFrames ) )
      {
        if( rcParams.logLevel > VVDEC_VERBOSE )
          fprintf( stdout, "[frames] : %d\n", riFrames );
      }
      else if( parse_param( { "-t", "--threads" }, rcParams.threads ) )
      {
        if( rcParams.logLevel > VVDEC_VERBOSE )
          fprintf( stdout, "[threads] : %d\n", rcParams.threads );
      }
      else if( parse_param( { "-p", "--parsedelay" }, rcParams.parseDelay ) )
      {
        if( rcParams.logLevel > VVDEC_VERBOSE )
          fprintf( stdout, "[parsedelay] : %d\n", rcParams.parseDelay );
      }
      else if( parse_param( { "-dph", "--SEIDecodedPictureHash" }, riPrintPicHash, true, 1 ) )
      {
        if( riPrintPicHash == 1 )   // dph levels > 11 print the DPH, but don't verify it (only 1 actually verifies)
        {
          rcParams.verifyPictureHash = true;
          if( rcParams.logLevel > VVDEC_VERBOSE )
            fprintf( stdout, "[SEIDecodedPictureHash] : true\n" );
        }
      }
      else if( parse_param( { "-md5", "--CheckYuvMD5" }, rcExpectYuvMD5 ) )
      {
        if( rcParams.logLevel > VVDEC_VERBOSE )
          fprintf( stdout, "[CheckYuvMD5] : %s\n", rcExpectYuvMD5.c_str() );
      }
      else if( parse_param( { "-L", "--loops" }, riLoops ) )
      {
        if( rcParams.logLevel > VVDEC_VERBOSE )
          fprintf( stdout, "[loops] : %d\n", riLoops );
      }
      else if( parse_param( { "--simd" }, simd_arg ) )
      {
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
            // clang-format off
            case VVDEC_SIMD_DEFAULT:   cll = "DEFAULT";   break;
            case VVDEC_SIMD_SCALAR:    cll = "SCALAR";    break;
#if defined (VVDEC_ARCH_X86)
            case VVDEC_SIMD_SSE41:     cll = "SSE41";     break;
            case VVDEC_SIMD_SSE42:     cll = "SSE42";     break;
            case VVDEC_SIMD_AVX:       cll = "AVX";       break;
            case VVDEC_SIMD_AVX2:      cll = "AVX2";      break;
#elif defined (VVDEC_ARCH_ARM)
            case VVDEC_SIMD_NEON:      cll = "NEON";      break;
#elif defined (VVDEC_ARCH_WASM)
            case VVDEC_SIMD_WASM:      cll = "WASM-SIMD"; break;
#else
            case VVDEC_SIMD_SIMDE_ANY: cll = "SIMDE-ANY"; break;
#endif
            default:                   return -1;
            // clang-format on
          };
          fprintf( stdout, "[simd] : %s\n", cll );
        }
      }
      else if( parse_param( { "-eh", "--errHandling" }, err_handle_flags ) )
      {
        if( err_handle_flags < 0 || err_handle_flags > VVDEC_ERR_HANDLING_TRY_CONTINUE )
        {
          fprintf( stderr, " - unsupported error handling flags. Should be between 0 and %i.\n", VVDEC_ERR_HANDLING_TRY_CONTINUE );
          return -1;
        }

        rcParams.errHandlingFlags = vvdecErrHandlingFlags( err_handle_flags );
      }
#ifdef ENABLE_TRACING
      else if( parse_param( { "-tf", "--TraceFile" }, sTracingFile ) ) {}
      else if( parse_param( { "-tr", "--TraceRule" }, sTracingRule ) ) {}
#endif
      else if( parse_param( { "-v", "--verbosity" }, logLevel ) )   // already processed. Parse again so we don't detect an unknown argument
      {
        assert( logLevel == rcParams.logLevel );
      }
      else
      {
        fprintf( stderr, " - unknown argument: %s \n", argv[m_iArg++] );
        return -1;
      }
    }

    return 0;
  }

  struct MissingArgumentException : std::exception
  {
  };
};

}   // namespace vvdecoderapp
