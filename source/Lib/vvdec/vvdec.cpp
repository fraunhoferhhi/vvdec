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
#include "vvdec/vvdec.h"
#include "vvdecimpl.h"

#include <stdio.h>

#include "vvdec/version.h"

#ifdef __cplusplus
extern "C" {
#endif

VVDEC_NAMESPACE_BEGIN

VVDEC_DECL void vvdec_params_default(vvdecParams *params)
{
  if( nullptr == params )
  {
    vvdec::msg( vvdec::ERROR, "vvdec_params_default: vvdecParams is invalid\n" );
    return;
  }

  // ensure the padding parameters are cleared also, so we don't read undefined values,
  // when new parameters are introduced and the library is used with an old executable
  memset( params, 0, sizeof( vvdecParams ) );

  params->threads            = -1;                      // thread count                          ( default: -1 )
  params->parseDelay         = -1;                      // number of frames to parse in parallel ( default: -1 )
  params->logLevel           = VVDEC_WARNING;           // verbosity level
  params->verifyPictureHash  = false;                   // verify picture, if digest is available, true: check hash in SEI messages if available, false: ignore SEI message
  params->opaque             = nullptr;                 // opaque pointer for private user data ( can be used to carry caller specific data or contexts )
  params->simd               = VVDEC_SIMD_DEFAULT;      // set specific simd optimization (default: max. availalbe)
  params->errHandlingFlags   = VVDEC_ERR_HANDLING_OFF;  // no special error handling
  params->filmGrainSynthesis = true;                    // enable film grain synthesis using Film Grain Charactersitics SEI ( default: true )
}

VVDEC_DECL vvdecParams* vvdec_params_alloc()
{
  vvdecParams* params = (vvdecParams*)malloc(sizeof(vvdecParams));
  if( nullptr == params )
  {
    vvdec::msg( vvdec::ERROR, "vvdec_params_alloc: memory allocation failed\n" );
    return nullptr;
  }
  vvdec_params_default( params );
  return params;
}

VVDEC_DECL void vvdec_params_free(vvdecParams *params )
{
  if( params )
  {
    free(params);
  }
}


VVDEC_DECL void vvdec_accessUnit_default(vvdecAccessUnit *accessUnit )
{
  if( nullptr == accessUnit )
  {
    vvdec::msg( vvdec::ERROR, "vvdec_accessUnit_default: vvdecAccessUnit is null\n" );
    return;
  }

  accessUnit->payload         = NULL;         ///< pointer to buffer that retrieves the coded data,
  accessUnit->payloadSize     = 0;            ///< size of the allocated buffer in bytes
  accessUnit->payloadUsedSize = 0;            ///< length of the coded data in bytes
  accessUnit->cts             = 0;            ///< composition time stamp in TicksPerSecond (see VVCDecoderParameter)
  accessUnit->dts             = 0;            ///< decoding time stamp in TicksPerSecond (see VVCDecoderParameter)
  accessUnit->ctsValid        = false;        ///< composition time stamp valid flag (true: valid, false: CTS not set)
  accessUnit->dtsValid        = false;        ///< decoding time stamp valid flag (true: valid, false: DTS not set)
  accessUnit->rap             = false;        ///< random access point flag (true: AU is random access point, false: sequential access)
}

VVDEC_DECL vvdecAccessUnit* vvdec_accessUnit_alloc()
{
  vvdecAccessUnit* accessUnit = (vvdecAccessUnit*)malloc(sizeof(vvdecAccessUnit));
  if( nullptr == accessUnit )
  {
    vvdec::msg( vvdec::ERROR, "vvdec_accessUnit_alloc: memory allocation failed\n" );
    return nullptr;
  }
  vvdec_accessUnit_default( accessUnit );
  return accessUnit;
}

VVDEC_DECL void vvdec_accessUnit_free(vvdecAccessUnit *accessUnit )
{
  if( accessUnit )
  {
    if( accessUnit->payload )
    {
      vvdec_accessUnit_free_payload ( accessUnit );
    }
    free(accessUnit);
  }
}

VVDEC_DECL void vvdec_accessUnit_alloc_payload(vvdecAccessUnit *accessUnit, int payload_size )
{
  accessUnit->payload = (unsigned char*)malloc(sizeof(unsigned char) * payload_size );
  if( nullptr == accessUnit->payload )
  {
    vvdec::msg( vvdec::ERROR, "vvdec_accessUnit_alloc_payload: memory allocation failed\n" );
    return;
  }
  accessUnit->payloadSize = payload_size;
}

VVDEC_DECL void vvdec_accessUnit_free_payload(vvdecAccessUnit *accessUnit )
{
  if( accessUnit->payload )
  {
    free(accessUnit->payload);
    accessUnit->payloadSize = 0;
    accessUnit->payloadUsedSize = 0;
  }
}

VVDEC_DECL const char* vvdec_get_version()
{
  return VVDEC_VERSION;
}

static int paramCheck( vvdecParams *params )
{
  if (nullptr == params)
  {
    vvdec::msg( vvdec::ERROR, "vvdecParams is null\n" );
    return -1;
  }

  int ret = 0;
  if( (int)params->simd > (int)VVDEC_SIMD_MAX || (int)params->simd < (int)VVDEC_SIMD_DEFAULT)
  {
    vvdec::msg( vvdec::ERROR, "unsupported simd mode. simd must be 0 <= simd <= %i\n", (int)VVDEC_SIMD_MAX );
    ret = -1;
  }

  return ret;
}

VVDEC_DECL vvdecDecoder* vvdec_decoder_open( vvdecParams *params)
{
  if (nullptr == params)
  {
    vvdec::msg( vvdec::ERROR, "vvdec_decoder_open() vvdecParams is null\n" );
    return nullptr;
  }

  if ( 0 != paramCheck( params ))
  {
    return nullptr;
  }

  vvdec::VVDecImpl* decCtx = new vvdec::VVDecImpl();
  if (!decCtx)
  {
    vvdec::msg( vvdec::ERROR, "cannot allocate memory for VVdeC decoder\n" );
    return nullptr;
  }

  int ret = decCtx->catchExceptions( &vvdec::VVDecImpl::init, *params, nullptr, nullptr );
  if (ret != 0)
  {
    const std::string initErr( std::move( decCtx->m_cAdditionalErrorString ) );

    // Error initializing the decoder
    delete decCtx;

    vvdec::msg( vvdec::ERROR, "cannot init the VVdeC decoder:\n%s\n", initErr.c_str() );
    return nullptr;
  }

  return (vvdecDecoder*)decCtx;
}

VVDEC_DECL vvdecDecoder* vvdec_decoder_open_with_allocator( vvdecParams *params,
                                              vvdecCreateBufferCallback  callbackBufAlloc,
                                              vvdecUnrefBufferCallback   callbackBufUnref )
{
  if ( 0 != paramCheck( params ))
  {
    return nullptr;
  }

  vvdec::VVDecImpl* decCtx = new vvdec::VVDecImpl();
  if (!decCtx)
  {
    vvdec::msg( vvdec::ERROR, "cannot allocate memory for VVdeC decoder\n" );
    return nullptr;
  }

  int ret = decCtx->catchExceptions( &vvdec::VVDecImpl::init, *params, callbackBufAlloc, callbackBufUnref );
  if (ret != 0)
  {
    const std::string initErr( std::move( decCtx->m_cAdditionalErrorString ) );

    // Error initializing the decoder
    delete decCtx;

    vvdec::msg( vvdec::ERROR, "cannot init the VVdeC decoder:\n%s\n", initErr.c_str() );
    return nullptr;
  }

  return (vvdecDecoder*)decCtx;
}


VVDEC_DECL int vvdec_decoder_close(vvdecDecoder *dec)
{
  auto d = (vvdec::VVDecImpl*)dec;
  if (!d)
  {
    return VVDEC_ERR_INITIALIZE;
  }

  const int ret = d->catchExceptions( &vvdec::VVDecImpl::uninit );

  delete d;

  return ret;
}

VVDEC_DECL int vvdec_set_logging_callback(vvdecDecoder* dec, vvdecLoggingCallback callback )
{
  auto d = (vvdec::VVDecImpl*)dec;
  if (!d)
  {
    return VVDEC_ERR_UNSPECIFIED;
  }

  d->setLoggingCallback(callback );
  return VVDEC_OK;
}


VVDEC_DECL int vvdec_decode( vvdecDecoder *dec, vvdecAccessUnit* accessUnit, vvdecFrame** frame )
{
  *frame = nullptr;

  auto d = (vvdec::VVDecImpl*)dec;
  if (!d)
  {
    return VVDEC_ERR_INITIALIZE;
  }

  if( nullptr == accessUnit )
  {
    return d->setAndRetErrorMsg( VVDEC_ERR_DEC_INPUT, "no access unit provided (null)" );
  }

  return d->catchExceptions( &vvdec::VVDecImpl::decode, *accessUnit, frame );
}


VVDEC_DECL int vvdec_flush( vvdecDecoder *dec, vvdecFrame **frame )
{
  *frame = nullptr;

  auto d = (vvdec::VVDecImpl*)dec;
  if (!d)
  {
    return VVDEC_ERR_INITIALIZE;
  }

  return d->catchExceptions( &vvdec::VVDecImpl::flush, frame );
}

VVDEC_DECL vvdecSEI* vvdec_find_frame_sei( vvdecDecoder *dec, vvdecSEIPayloadType seiPayloadType, vvdecFrame *frame )
{
  auto d = (vvdec::VVDecImpl*)dec;
  if (!d)
  {
    return nullptr;
  }

  return d->catchExceptions( &vvdec::VVDecImpl::findFrameSei, seiPayloadType, frame );
}

VVDEC_DECL int vvdec_frame_unref( vvdecDecoder *dec, vvdecFrame *frame )
{
  auto d = (vvdec::VVDecImpl*)dec;
  if (!d)
  {
    return VVDEC_ERR_INITIALIZE;
  }

  return d->catchExceptions( &vvdec::VVDecImpl::objectUnref, frame );
}

VVDEC_DECL int vvdec_get_hash_error_count( vvdecDecoder *dec )
{
  auto d = (vvdec::VVDecImpl*)dec;
  if (!d)
  {
    return VVDEC_ERR_INITIALIZE;
  }

  return d->catchExceptions( &vvdec::VVDecImpl::getNumberOfErrorsPictureHashSEI );
}


VVDEC_DECL const char* vvdec_get_dec_information( vvdecDecoder *dec )
{
  auto d = (vvdec::VVDecImpl*)dec;
  if (!d)
  {
    return nullptr;
  }

  return d->catchExceptions( &vvdec::VVDecImpl::getDecoderInfo );
}

VVDEC_DECL const char* vvdec_get_last_error( vvdecDecoder *dec )
{
  auto d = (vvdec::VVDecImpl*)dec;
  if (!d)
  {
    return nullptr;
  }

  return d->m_cErrorString.c_str();
}

VVDEC_DECL const char* vvdec_get_last_additional_error( vvdecDecoder *dec )
{
  auto d = (vvdec::VVDecImpl*)dec;
  if (!d)
  {
    return nullptr;
  }

  return d->m_cAdditionalErrorString.c_str();
}

VVDEC_DECL const char* vvdec_get_error_msg( int nRet )
{
  return vvdec::VVDecImpl::getErrorMsg( nRet );
}

VVDEC_DECL vvdecNalType vvdec_get_nal_unit_type( vvdecAccessUnit *accessUnit )
{
  if( nullptr == accessUnit )
  {
    return VVC_NAL_UNIT_INVALID;
  }
  return vvdec::VVDecImpl::getNalUnitType(*accessUnit);
}

VVDEC_DECL const char* vvdec_get_nal_unit_type_name( vvdecNalType t )
{
  return vvdec::VVDecImpl::getNalUnitTypeAsString(t);
}

VVDEC_DECL bool vvdec_is_nal_unit_slice( vvdecNalType t )
{
  return vvdec::VVDecImpl::isNalUnitSlice(t);
}

VVDEC_DECL int vvdec_set_tracing( const char *file, const char *rule )
{
  std::string sTracingFile = file;
  std::string sTracingRule = rule;

#if ENABLE_TRACING
  if( !vvdec::g_trace_ctx )
  {
    vvdec::g_trace_ctx = vvdec::tracing_init( sTracingFile, sTracingRule );
  }

  bool bPrint = !( sTracingFile.empty() && sTracingRule.empty() );
  if( vvdec::g_trace_ctx && ( bPrint || vvdec::g_trace_ctx->getLastError() ) )
  {
    std::string sChannelsList;
    vvdec::g_trace_ctx->getChannelsList( sChannelsList );
    vvdec::msg( vvdec::INFO, "\nAvailable tracing channels:\n\n%s\n", sChannelsList.c_str() );

    if( vvdec::g_trace_ctx->getLastError() )
    {
      vvdec::tracing_uninit( vvdec::g_trace_ctx );
      vvdec::g_trace_ctx = nullptr;
      return VVDEC_ERR_INITIALIZE;
    }
  }

  return VVDEC_OK;
#else
  if( sTracingFile.empty() && sTracingRule.empty() )
    return VVDEC_OK;
  else
    return VVDEC_ERR_INITIALIZE;
#endif
}

VVDEC_NAMESPACE_END

#ifdef __cplusplus
}
#endif
