/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2018-2026, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVdeC Authors.
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

#include <cstdint>
#include <string>
#include <vector>
#include <thread>
#include <algorithm>
#include <iostream>

#include <vvdec/vvdec.h>

static vvdecParams g_params;

#define COMPARE_ARG( ARG_STR ) ( currArg.compare( 0, sizeof( ARG_STR ) - 1, ARG_STR ) == 0 && currArg[sizeof( ARG_STR ) - 1] == '=' )

extern "C" int LLVMFuzzerInitialize( int* argc, char*** argv )
{
  vvdec_params_default( &g_params );
  g_params.threads  = std::min( 8u, std::thread::hardware_concurrency() );   // limit default to 8 threads
  g_params.logLevel = VVDEC_SILENT;

  if( argc && argv )
  {
    for( int i = 1; i < *argc; ++i )
    {
      std::string currArg( ( *argv )[i] );

      if( COMPARE_ARG( "--t" ) )
      {
        g_params.threads = std::atoi( currArg.c_str() + 4 );
      }
      else if( COMPARE_ARG( "--p" ) )
      {
        g_params.parseDelay = std::atoi( currArg.c_str() + 4 );
      }
      else if( COMPARE_ARG( "--simd" ) )
      {
        g_params.simd = static_cast<vvdecSIMD_Extension>( std::atoi( currArg.c_str() + 7 ) + 1 );
      }
      else if( COMPARE_ARG( "--v" ) )
      {
        g_params.logLevel = static_cast<vvdecLogLevel>( std::atoi( currArg.c_str() + 4 ) );
      }
      else if( COMPARE_ARG( "--eh" ) )
      {
        g_params.errHandlingFlags = static_cast<vvdecErrHandlingFlags>( std::atoi( currArg.c_str() + 5 ) );
      }
      else if( currArg.compare( 0, 2, "--" ) == 0 )
      {
        std::cerr << "Unknown VVdeC argument: " << currArg << std::endl;
        return -1;
      }
    }
  }
  return 0;
}

extern "C" int LLVMFuzzerTestOneInput( const uint8_t* data, size_t size )
{
  auto* dec = vvdec_decoder_open( &g_params );
  if( !dec )
  {
    return -1;
  }
  if( g_params.logLevel >= VVDEC_INFO )
  {
    std::cerr << vvdec_get_dec_information( dec ) << std::endl;
  }

  vvdecAccessUnit au;
  vvdec_accessUnit_default( &au );

  // we can pass the whole bitstream to the decoder, since the decoder will process the NAL units individually anyways
  std::vector<uint8_t> payload( data, data + size );
  au.payload         = payload.data();
  au.payloadSize     = static_cast<int>( size );
  au.payloadUsedSize = static_cast<int>( size );

  vvdecFrame* frame     = nullptr;
  int         numFailed = 0;
  int         ret       = vvdec_decode( dec, &au, &frame );
  while( ret == VVDEC_OK || ret == VVDEC_TRY_AGAIN || ret == VVDEC_ERR_DEC_INPUT )
  {
    if( frame )
    {
      vvdec_frame_unref( dec, frame );
    }

    frame = nullptr;
    ret   = vvdec_flush( dec, &frame );
    if( ret == VVDEC_OK || frame )
    {
      numFailed = 0;
    }
    else
    {
      ++numFailed;
      if( numFailed > 32 + std::max<int>( g_params.threads * 1.5, g_params.parseDelay ) )
      {
        std::cerr << "ERROR: probably infinite loop of vvdec_flush() calls. " << numFailed << " successive flush calles returned an error." << std::endl;
        abort();
      }
    }
  }
  if( frame )
  {
    vvdec_frame_unref( dec, frame );
  }

  vvdec_decoder_close( dec );

  return 0;   // Values other than 0 and -1 are reserved for future use.
}
