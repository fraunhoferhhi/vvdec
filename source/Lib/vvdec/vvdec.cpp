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

VVDEC_DECL const char* vvdec_get_version()
{
  return VVDEC_VERSION;
}

VVDEC_DECL vvdec_decoder_t* vvdec_decoder_open( vvdec_Params *params)
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
    return nullptr;
  }

  if( params->m_iThreads > 64 )
  {
    return nullptr;
  }

  vvdec::VVDecImpl* decCtx = new vvdec::VVDecImpl();
  if (!decCtx)
  {
    return nullptr;
  }

  int ret = decCtx->init(*params);
  if (ret != 0)
  {
    // Error initializing the decoder
    throw( Exception( "\nERROR: In function \"" ) << __FUNCTION__ << "\" in " << __FILE__ << ":" << __LINE__ << ": cannot init VVDecimpl" );
    delete decCtx;
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

  delete d;

  return VVDEC_OK;
}

VVDEC_DECL ErrorCodes vvdec_set_logging_callback(vvdec_decoder_t* dec, vvdec_logging_callback callback, void *userData, LogLevel loglevel )
{
  auto d = (vvdec::VVDecImpl*)dec;
  if (!d || !callback)
  {
    return VVDEC_ERR_UNSPECIFIED;
  }

  d->setLogging(callback, userData, loglevel);
  return VVDEC_OK;
}


VVDEC_DECL int vvdec_decode( vvdec_decoder_t *dec, vvdec_AccessUnit* accessUnit, vvdec_Frame** frame )
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


VVDEC_DECL int vvdec_flush( vvdec_decoder_t *dec, vvdec_Frame **frame )
{
  auto d = (vvdec::VVDecImpl*)dec;
  if (!d)
  {
    return VVDEC_ERR_INITIALIZE;
  }

  return d->setAndRetErrorMsg( d->flush( frame ) );
}

VVDEC_DECL int vvdec_objectUnref( vvdec_decoder_t *dec, vvdec_Frame *frame )
{
  auto d = (vvdec::VVDecImpl*)dec;
  if (!d)
  {
    return VVDEC_ERR_INITIALIZE;
  }

  return d->setAndRetErrorMsg( d->objectUnref( frame ) );
}

VVDEC_DECL int vvdec_getNumberOfErrorsPictureHashSEI( vvdec_decoder_t *dec )
{
  auto d = (vvdec::VVDecImpl*)dec;
  if (!d)
  {
    return VVDEC_ERR_INITIALIZE;
  }

  return d->setAndRetErrorMsg( d->getNumberOfErrorsPictureHashSEI() );
}


VVDEC_DECL const char* vvdec_getDecoderInfo( vvdec_decoder_t *dec )
{
  auto d = (vvdec::VVDecImpl*)dec;
  if (!d)
  {
    return nullptr;
  }

  return d->getDecoderInfo();
}

VVDEC_DECL const char* vvdec_getLastError( vvdec_decoder_t *dec )
{
  auto d = (vvdec::VVDecImpl*)dec;
  if (!d)
  {
    return nullptr;
  }

  return d->m_cErrorString.c_str();
}

VVDEC_DECL const char* vvdec_getLastAdditionalError( vvdec_decoder_t *dec )
{
  auto d = (vvdec::VVDecImpl*)dec;
  if (!d)
  {
    return nullptr;
  }

  return d->m_cAdditionalErrorString.c_str();
}

const char* vvdec_getErrorMsg( int nRet )
{
  return vvdec::VVDecImpl::getErrorMsg( nRet );
}

NalType vvdec_getNalUnitType ( vvdec_AccessUnit *accessUnit )
{
  if( nullptr == accessUnit )
  {
    return VVC_NAL_UNIT_INVALID;
  }
  return vvdec::VVDecImpl::getNalUnitType(*accessUnit);
}

const char* vvdec_getNalUnitTypeAsString( NalType t )
{
  return vvdec::VVDecImpl::getNalUnitTypeAsString(t);
}

bool vvdec_isNalUnitSideData            ( NalType t )
{
  return vvdec::VVDecImpl::isNalUnitSideData(t);
}

bool vvdec_isNalUnitSlice               ( NalType t )
{
  return vvdec::VVDecImpl::isNalUnitSlice(t);
}

#ifdef __cplusplus
};
#endif

VVDEC_NAMESPACE_END
