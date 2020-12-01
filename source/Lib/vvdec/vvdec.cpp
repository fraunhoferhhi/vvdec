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
#include "../../../include/vvdec/vvdec.h"

#include "vvdecimpl.h"

#define THROW_EXC(x)            throw( Exception( "\nERROR: In function \"" ) << __FUNCTION__ << "\" in " << __FILE__ << ":" << __LINE__ << ": " << x )

namespace vvdec {


const char *sMsg = "Not initialized";

VVDec::VVDec()
{
  m_pcVVDecImpl = new VVDecImpl;

  if( NULL == m_pcVVDecImpl ) {  throw( Exception( "\nERROR: In function \"" ) << __FUNCTION__ << "\" in " << __FILE__ << ":" << __LINE__ << ": cannot init VVDecimpl" ); }
}

/// Destructor
VVDec::~VVDec()
{
  if( NULL != m_pcVVDecImpl )
  {
    if( m_pcVVDecImpl->m_bInitialized )
    {
      uninit();
    }
    delete m_pcVVDecImpl;
    m_pcVVDecImpl = NULL;
  }
}

int VVDec::init( const VVDecParameter& rcVVDecParameter )
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
  
  if( m_pcVVDecImpl->m_bInitialized )
  {
    return m_pcVVDecImpl->setAndRetErrorMsg( VVDEC_ERR_INITIALIZE );
  }

  if( rcVVDecParameter.m_iThreads > 64 )
  {
    return m_pcVVDecImpl->setAndRetErrorMsg( VVDEC_ERR_NOT_SUPPORTED );
  }

  if ( 0 != m_pcVVDecImpl->init( rcVVDecParameter ) )
  {
    return VVDEC_ERR_INITIALIZE;
  }

  return VVDEC_OK;
}

int VVDec::uninit()
{
  if( !m_pcVVDecImpl->m_bInitialized )
  {
    return m_pcVVDecImpl->setAndRetErrorMsg( VVDEC_ERR_INITIALIZE );
  }

  if ( 0 != m_pcVVDecImpl->uninit( ) )
  {
    return VVDEC_ERR_INITIALIZE;
  }

  m_pcVVDecImpl->m_bInitialized = false;

  return VVDEC_OK;
}

bool VVDec::isInitialized()
{
  return m_pcVVDecImpl->m_bInitialized;
}

int VVDec::decode( AccessUnit& rcAccessUnit, Frame** ppcFrame )
{
  if( !m_pcVVDecImpl->m_bInitialized )
  { return m_pcVVDecImpl->setAndRetErrorMsg(VVDEC_ERR_INITIALIZE); }

  return m_pcVVDecImpl->setAndRetErrorMsg( m_pcVVDecImpl->decode( rcAccessUnit, ppcFrame ) );
}

int VVDec::flush(  Frame** ppcFrame )
{
  if( !m_pcVVDecImpl->m_bInitialized )
  {  return m_pcVVDecImpl->setAndRetErrorMsg(VVDEC_ERR_INITIALIZE); }

  return m_pcVVDecImpl->setAndRetErrorMsg( m_pcVVDecImpl->flush( ppcFrame ) );
}

int VVDec::objectUnref( Frame* pcFrame )
{
  if( !m_pcVVDecImpl->m_bInitialized )
  {  return m_pcVVDecImpl->setAndRetErrorMsg(VVDEC_ERR_INITIALIZE); }

  return m_pcVVDecImpl->setAndRetErrorMsg( m_pcVVDecImpl->objectUnref( pcFrame ) );
}


int VVDec::getNumberOfErrorsPictureHashSEI( )
{
  if( !m_pcVVDecImpl->m_bInitialized )
  {  return m_pcVVDecImpl->setAndRetErrorMsg(VVDEC_ERR_INITIALIZE); }

  return m_pcVVDecImpl->setAndRetErrorMsg( m_pcVVDecImpl->getNumberOfErrorsPictureHashSEI( ) );
}

int VVDec::clockStartTime()
{
  if( !m_pcVVDecImpl->m_bInitialized )
  {  return m_pcVVDecImpl->setAndRetErrorMsg(VVDEC_ERR_INITIALIZE); }

  m_pcVVDecImpl->clockStartTime();
  return 0;
}

int VVDec::clockEndTime()
{
  if( !m_pcVVDecImpl->m_bInitialized )
  {  return m_pcVVDecImpl->setAndRetErrorMsg(VVDEC_ERR_INITIALIZE); }

  m_pcVVDecImpl->clockEndTime();
  return 0;
}

double VVDec::clockGetTimeDiffMs( )
{
  if( !m_pcVVDecImpl->m_bInitialized )
  {  return m_pcVVDecImpl->setAndRetErrorMsg(VVDEC_ERR_INITIALIZE); }

  return m_pcVVDecImpl->clockGetTimeDiffMs();
}

const char* VVDec::getDecoderInfo()
{
  return m_pcVVDecImpl->getDecoderInfo();
}

const char* VVDec::getLastError() const
{
  return m_pcVVDecImpl->m_cErrorString.c_str();
}

const char* VVDec::getLastAdditionalError() const
{
  return m_pcVVDecImpl->m_cAdditionalErrorString.c_str();
}

const char* VVDec::getVersionNumber()
{
  return VVDecImpl::getVersionNumber();
}

const char* VVDec::getErrorMsg( int nRet )
{
  return VVDecImpl::getErrorMsg(nRet);
}


NalType VVDec::getNalUnitType ( AccessUnit& rcAccessUnit )
{
  return VVDecImpl::getNalUnitType(rcAccessUnit);
}

const char* VVDec::getNalUnitTypeAsString( NalType t )
{
  return VVDecImpl::getNalUnitTypeAsString(t);
}

bool VVDec::isNalUnitSlice( NalType t )
{
  return VVDecImpl::isNalUnitSlice(t);
}

bool VVDec::isNalUnitSideData( NalType t )
{
  return VVDecImpl::isNalUnitSideData(t);
}


} // namespace

