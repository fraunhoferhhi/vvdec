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

/** \file     VLCReader.cpp
 *  \brief    Reader for high level syntax
 */

//! \ingroup DecoderLib
//! \{

#include "VLCReader.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/dtrace_next.h"

namespace vvdec
{

// ====================================================================================================================
//  read functions taking a reference for the result value
// ====================================================================================================================

void VLCReader::xReadFlag( uint32_t& ruiCode )
{
  ruiCode = m_pcBitstream->read( 1 );
}

void VLCReader::xReadUvlc( uint32_t& ruiVal )
{
  uint32_t uiVal = 0;
  uint32_t uiCode = 0;
  uint32_t uiLength = 0;

  uiCode = m_pcBitstream->read( 1 );
  if( 0 == uiCode )
  {
    uiLength = 0;

    while( ! ( uiCode & 1 ) )
    {
      uiCode = m_pcBitstream->read( 1 );
      uiLength++;
    }

    uiVal = m_pcBitstream->read( uiLength );

    uiVal += ( 1 << uiLength ) - 1;
  }

  ruiVal = uiVal;
}

void VLCReader::xReadSvlc( int32_t& riVal )
{
  uint32_t uiBits = m_pcBitstream->read( 1 );
  if( 0 == uiBits )
  {
    uint32_t uiLength = 0;

    while( ! ( uiBits & 1 ) )
    {
      uiBits = m_pcBitstream->read( 1 );
      uiLength++;
    }

    uiBits = m_pcBitstream->read( uiLength );

    uiBits += ( 1 << uiLength );
    riVal = ( uiBits & 1 ) ? -(int32_t)( uiBits>>1 ) : (int32_t)( uiBits>>1 );
  }
  else
  {
    riVal = 0;
  }
}

void VLCReader::xReadCode( uint32_t uiLength, uint32_t& ruiCode )
{
  CHECK( uiLength == 0, "Reading a code of lenght '0'" );
  ruiCode = m_pcBitstream->read( uiLength );
}

void VLCReader::xReadSCode( uint32_t length, int32_t& value )
{
  CHECK( length == 0 || length > 31, "wrong" );
  uint32_t val = m_pcBitstream->read( length );
  value = length >= 32 ? int32_t( val ) : ( ( -int32_t( val & ( uint32_t( 1 ) << ( length - 1 ) ) ) ) | int32_t( val ) );
}

void VLCReader::xReadFlag( uint32_t& rValue, const char *pSymbolName )
{
  xReadFlag( rValue );
  DTRACE( g_trace_ctx, D_HEADER, "%-50s u(1)  : %d\n", pSymbolName, rValue );
}

void VLCReader::xReadUvlc( uint32_t& rValue, const char *pSymbolName )
{
  xReadUvlc( rValue );
  DTRACE( g_trace_ctx, D_HEADER, "%-50s ue(v) : %u\n", pSymbolName, rValue );
}

void VLCReader::xReadSvlc( int32_t& rValue, const char *pSymbolName )
{
  xReadSvlc( rValue );
  DTRACE( g_trace_ctx, D_HEADER, "%-50s se(v) : %d\n", pSymbolName, rValue );
}

void VLCReader::xReadCode( uint32_t length, uint32_t& rValue, const char *pSymbolName )
{
  xReadCode( length, rValue );
  if( length < 10 )
  {
    DTRACE( g_trace_ctx, D_HEADER, "%-50s u(%d)  : %u\n", pSymbolName, length, rValue );
  }
  else
  {
    DTRACE( g_trace_ctx, D_HEADER, "%-50s u(%d) : %u\n", pSymbolName, length, rValue );
  }
}

void VLCReader::xReadSCode( uint32_t length, int32_t& value, const char *pSymbolName )
{
  xReadSCode( length, value );

  if( length < 10 )
  {
    DTRACE( g_trace_ctx, D_HEADER, "%-50s i(%d)  : %d\n", pSymbolName, length, value );
  }
  else
  {
    DTRACE( g_trace_ctx, D_HEADER, "%-50s i(%d) : %d\n", pSymbolName, length, value );
  }
}

// ====================================================================================================================
//  read functions returning the result value
// ====================================================================================================================

bool VLCReader::xReadFlag()
{
  return m_pcBitstream->read( 1 ) & 0x1;
}

uint32_t VLCReader::xReadUvlc()
{
  uint32_t uiVal = 0;
  uint32_t uiCode = 0;
  uint32_t uiLength = 0;

  uiCode = m_pcBitstream->read( 1 );
  if( 0 == uiCode )
  {
    uiLength = 0;

    while( ! ( uiCode & 1 ) )
    {
      uiCode = m_pcBitstream->read( 1 );
      uiLength++;
    }

    uiVal = m_pcBitstream->read( uiLength );

    uiVal += ( 1 << uiLength ) - 1;
  }

  return uiVal;
}

int32_t VLCReader::xReadSvlc()
{
  uint32_t uiBits = m_pcBitstream->read( 1 );
  if( 0 == uiBits )
  {
    uint32_t uiLength = 0;

    while( ! ( uiBits & 1 ) )
    {
      uiBits = m_pcBitstream->read( 1 );
      uiLength++;
    }

    uiBits = m_pcBitstream->read( uiLength );

    uiBits += ( 1 << uiLength );
    return ( uiBits & 1 ) ? -(int32_t)( uiBits>>1 ) : (int32_t)( uiBits>>1 );
  }
  else
  {
    return  0;
  }
}

uint32_t VLCReader::xReadCode( uint32_t uiLength )
{
  CHECK( uiLength == 0, "Reading a code of lenght '0'" );
  return m_pcBitstream->read( uiLength );
}

int32_t VLCReader::xReadSCode( uint32_t length )
{
  CHECK( length == 0 || length > 31, "wrong" );
  uint32_t val = m_pcBitstream->read( length );
  return length >= 32 ? int32_t( val ) : ( ( -int32_t( val & ( uint32_t( 1 ) << ( length - 1 ) ) ) ) | int32_t( val ) );
}

bool VLCReader::xReadFlag( const char* pSymbolName )
{
  bool value = xReadFlag();
  DTRACE( g_trace_ctx, D_HEADER, "%-50s u(1)  : %d\n", pSymbolName, value );
  return value;
}

uint32_t VLCReader::xReadUvlc( const char* pSymbolName )
{
  uint32_t value = xReadUvlc();
  DTRACE( g_trace_ctx, D_HEADER, "%-50s ue(v) : %u\n", pSymbolName, value );
  return value;
}

int32_t VLCReader::xReadSvlc( const char* pSymbolName )
{
  int32_t value = xReadSvlc();
  DTRACE( g_trace_ctx, D_HEADER, "%-50s se(v) : %d\n", pSymbolName, value );
  return value;
}

uint32_t VLCReader::xReadCode( uint32_t length, const char* pSymbolName )
{
  uint32_t value = xReadCode( length );
  if( length < 10 )
  {
    DTRACE( g_trace_ctx, D_HEADER, "%-50s u(%d)  : %u\n", pSymbolName, length, value );
  }
  else
  {
    DTRACE( g_trace_ctx, D_HEADER, "%-50s u(%d) : %u\n", pSymbolName, length, value );
  }
  return value;
}

int32_t VLCReader::xReadSCode( uint32_t length, const char* pSymbolName )
{
  int32_t value = xReadSCode( length );

  if( length < 10 )
  {
    DTRACE( g_trace_ctx, D_HEADER, "%-50s i(%d)  : %d\n", pSymbolName, length, value );
  }
  else
  {
    DTRACE( g_trace_ctx, D_HEADER, "%-50s i(%d) : %d\n", pSymbolName, length, value );
  }
  return value;
}

void VLCReader::xReadRbspTrailingBits()
{
  X_READ_FLAG( rbsp_stop_one_bit );
  CHECK( rbsp_stop_one_bit != 1, "Trailing bit not '1'" );
  int cnt = 0;
  while( m_pcBitstream->getNumBitsUntilByteAligned() )
  {
    X_READ_FLAG( rbsp_alignment_zero_bit );
    CHECK( rbsp_alignment_zero_bit != 0, "Alignment bit is not '0'" );
    cnt++;
  }
  CHECK( cnt >= 8, "Read more than '8' trailing bits" );
}

}   // namespace vvdec
