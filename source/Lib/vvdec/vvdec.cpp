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
#include "vvdec/vvdec.h"
#include "vvdecimpl.h"

#include "vvdec/version.h"

#ifdef __cplusplus
extern "C" {
#endif

VVDEC_NAMESPACE_BEGIN

const char *sMsg = "Not initialized";

VVDEC_DECL void vvdec_params_default(vvdec_params *params)
{
  params->threads                      = -1;                   ///< thread count        ( default: -1 )
  params->parseThreads                 = -1;                   ///< parser thread count ( default: -1 )
  params->upscaleOutput                =  0;                   ///< do internal upscaling of rpl pictures to dest. resolution ( default: 0 )
  params->logLevel                     = VVDEC_WARNING;        ///< verbosity level
  params->decodedPictureHashSEIEnabled = false;                ///<  Control handling of decoded picture hash SEI messages, true: check hash in SEI messages if available in the bitstream, false: ignore SEI message
  params->simd                         = VVDEC_SIMD_DEFAULT;   ///< set specific simd optimization (default: max. availalbe)
}

VVDEC_DECL vvdec_params* vvdec_params_alloc()
{
  vvdec_params* params = (vvdec_params*)malloc(sizeof(vvdec_params));
  vvdec_params_default( params );
  return params;
}

VVDEC_DECL void vvdec_params_free(vvdec_params *params )
{
  if( params )
  {
    free(params);
  }
}


VVDEC_DECL void vvdec_accessUnit_default(vvdec_accessUnit *accessUnit )
{
  accessUnit->payload         = NULL;         ///< pointer to buffer that retrieves the coded data,
  accessUnit->payloadSize     = 0;            ///< size of the allocated buffer in bytes
  accessUnit->payloadUsedSize = 0;            ///< length of the coded data in bytes
  accessUnit->cts             = 0;            ///< composition time stamp in TicksPerSecond (see VVCDecoderParameter)
  accessUnit->dts             = 0;            ///< decoding time stamp in TicksPerSecond (see VVCDecoderParameter)
  accessUnit->ctsValid        = false;        ///< composition time stamp valid flag (true: valid, false: CTS not set)
  accessUnit->dtsValid        = false;        ///< decoding time stamp valid flag (true: valid, false: DTS not set)
  accessUnit->rap             = false;        ///< random access point flag (true: AU is random access point, false: sequential access)
}

VVDEC_DECL vvdec_accessUnit* vvdec_accessUnit_alloc()
{
  vvdec_accessUnit* accessUnit = (vvdec_accessUnit*)malloc(sizeof(vvdec_accessUnit));
  vvdec_accessUnit_default( accessUnit );
  return accessUnit;
}

VVDEC_DECL void vvdec_accessUnit_free(vvdec_accessUnit *accessUnit )
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

VVDEC_DECL void vvdec_accessUnit_alloc_payload(vvdec_accessUnit *accessUnit, int payload_size )
{
  accessUnit->payload = (unsigned char*)malloc(sizeof(unsigned char) * payload_size );
  accessUnit->payloadSize = payload_size;
}

VVDEC_DECL void vvdec_accessUnit_free_payload(vvdec_accessUnit *accessUnit )
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

VVDEC_DECL vvdec_decoder_t* vvdec_decoder_open( vvdec_params *params)
{
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

  if (nullptr == params)
  {
    msg( ERROR, "vvdec_Params_t is null\n" );
    return nullptr;
  }

  if( params->threads > 64 )
  {
    msg( ERROR, "threads must be <= 64\n" );
    return nullptr;
  }

  vvdec::VVDecImpl* decCtx = new vvdec::VVDecImpl();
  if (!decCtx)
  {
    msg( ERROR, "cannot allocate memory for VVdeC decoder\n" );
    return nullptr;
  }

  int ret = decCtx->init(*params);
  if (ret != 0)
  {
    // Error initializing the decoder
    delete decCtx;

    msg( ERROR, "cannot init the VVdeC decoder\n" );
    return nullptr;
  }

  return (vvdec_decoder_t*)decCtx;
}

VVDEC_DECL int vvdec_decoder_close(vvdec_decoder_t *dec)
{
  auto d = (vvdec::VVDecImpl*)dec;
  if (!d)
  {
    return VVDEC_ERR_INITIALIZE;
  }

  d->uninit();

  delete d;

  return VVDEC_OK;
}

VVDEC_DECL int vvdec_set_logging_callback(vvdec_decoder_t* dec, vvdec_loggingCallback callback, void *userData, LogLevel loglevel )
{
  auto d = (vvdec::VVDecImpl*)dec;
  if (!d || !callback)
  {
    return VVDEC_ERR_UNSPECIFIED;
  }

  d->setLoggingCallback(callback, userData, loglevel);
  return VVDEC_OK;
}


VVDEC_DECL int vvdec_decode( vvdec_decoder_t *dec, vvdec_accessUnit_t* accessUnit, vvdec_frame_t** frame )
{
  auto d = (vvdec::VVDecImpl*)dec;
  if (!d)
  {
    return VVDEC_ERR_INITIALIZE;
  }

  if( nullptr == accessUnit )
  {
     return d->setAndRetErrorMsg(VVDEC_ERR_UNSPECIFIED);
  }

  return d->setAndRetErrorMsg( d->decode( *accessUnit, frame ) );
}


VVDEC_DECL int vvdec_flush( vvdec_decoder_t *dec, vvdec_frame_t **frame )
{
  auto d = (vvdec::VVDecImpl*)dec;
  if (!d)
  {
    return VVDEC_ERR_INITIALIZE;
  }

  return d->setAndRetErrorMsg( d->flush( frame ) );
}

VVDEC_DECL vvdec_sei_message_t* vvdec_find_frame_sei( vvdec_decoder_t *dec, SEIPayloadType seiPayloadType, vvdec_frame_t *frame )
{
  auto d = (vvdec::VVDecImpl*)dec;
  if (!d)
  {
    return nullptr;
  }

  return  d->findFrameSei( seiPayloadType, frame );
}

VVDEC_DECL int vvdec_frame_unref( vvdec_decoder_t *dec, vvdec_frame_t *frame )
{
  auto d = (vvdec::VVDecImpl*)dec;
  if (!d)
  {
    return VVDEC_ERR_INITIALIZE;
  }

  return d->setAndRetErrorMsg( d->objectUnref( frame ) );
}

VVDEC_DECL int vvdec_get_hash_error_count( vvdec_decoder_t *dec )
{
  auto d = (vvdec::VVDecImpl*)dec;
  if (!d)
  {
    return VVDEC_ERR_INITIALIZE;
  }

  return d->setAndRetErrorMsg( d->getNumberOfErrorsPictureHashSEI() );
}


VVDEC_DECL const char* vvdec_get_dec_information( vvdec_decoder_t *dec )
{
  auto d = (vvdec::VVDecImpl*)dec;
  if (!d)
  {
    return nullptr;
  }

  return d->getDecoderInfo();
}

VVDEC_DECL const char* vvdec_get_last_error( vvdec_decoder_t *dec )
{
  auto d = (vvdec::VVDecImpl*)dec;
  if (!d)
  {
    return nullptr;
  }

  return d->m_cErrorString.c_str();
}

VVDEC_DECL const char* vvdec_get_last_additional_error( vvdec_decoder_t *dec )
{
  auto d = (vvdec::VVDecImpl*)dec;
  if (!d)
  {
    return nullptr;
  }

  return d->m_cAdditionalErrorString.c_str();
}

const char* vvdec_get_error_msg( int nRet )
{
  return vvdec::VVDecImpl::getErrorMsg( nRet );
}

NalType vvdec_get_nal_unit_type ( vvdec_accessUnit *accessUnit )
{
  if( nullptr == accessUnit )
  {
    return VVC_NAL_UNIT_INVALID;
  }
  return vvdec::VVDecImpl::getNalUnitType(*accessUnit);
}

const char* vvdec_get_nal_unit_type_name( NalType t )
{
  return vvdec::VVDecImpl::getNalUnitTypeAsString(t);
}

bool vvdec_is_nal_unit_slice               ( NalType t )
{
  return vvdec::VVDecImpl::isNalUnitSlice(t);
}

#ifdef __cplusplus
};
#endif

VVDEC_NAMESPACE_END
