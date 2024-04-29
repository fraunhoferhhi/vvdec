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

/** \file     VLCReader.h
 *  \brief    Reader for high level syntax
 */

#pragma once

#include "CommonLib/BitStream.h"

#if ENABLE_TRACING

#define READ_CODE( length, code, name)    xReadCode ( length, code, name )
#define READ_UVLC(         code, name)    xReadUvlc (         code, name )
#define READ_SVLC(         code, name)    xReadSvlc (         code, name )
#define READ_FLAG(         code, name)    xReadFlag (         code, name )
#define READ_SCODE(length, code, name)    xReadSCode( length, code, name )

#else

#define READ_CODE( length, code, name)    xReadCode  ( length, code )
#define READ_UVLC(         code, name)    xReadUvlc  (         code )
#define READ_SVLC(         code, name)    xReadSvlc  (         code )
#define READ_FLAG(         code, name)    xReadFlag  (         code )
#define READ_SCODE(length, code, name)    xReadSCode ( length, code )

#endif


#define CHECK_READ( cond, msg, val )            CHECK( cond, msg << " (read:" << val << ")" )
#define CHECK_READ_RANGE( val, min, max, name ) CHECK( (val) < (min) || (val) > (max), name << " out of bounds (read:" << (val) << ")." )

#if ENABLE_TRACING
#  define X_READ_FLAG(  name )                             const bool     name = xReadFlag ( #name )
#  define X_READ_FLAG_idx( name, idx )                     const bool     name = xReadFlag ( #name idx )

#  define  X_READ_UVLC_NO_RANGE(  name         )           const uint32_t name = xReadUvlc (         #name )
#  define  X_READ_SVLC_NO_RANGE(  name         )           const int32_t  name = xReadSvlc (         #name )
#  define  X_READ_CODE_NO_RANGE(  name, length )           const uint32_t name = xReadCode ( length, #name )
#  define  X_READ_SCODE_NO_RANGE( name, length )           const int32_t  name = xReadSCode( length, #name )

#  define  X_READ_UVLC_NO_RANGE_idx(  name, idx         )  const uint32_t name = xReadUvlc (         #name idx )
#  define  X_READ_SVLC_NO_RANGE_idx(  name, idx         )  const int32_t  name = xReadSvlc (         #name idx )
#  define  X_READ_CODE_NO_RANGE_idx(  name, idx, length )  const uint32_t name = xReadCode ( length, #name idx )
#  define  X_READ_SCODE_NO_RANGE_idx( name, idx, length )  const int32_t  name = xReadSCode( length, #name idx )

#else

#  define X_READ_FLAG( name )                              const bool     name = xReadFlag ()
#  define X_READ_FLAG_idx( name, idx )                     const bool     name = xReadFlag ()

#  define X_READ_UVLC_NO_RANGE(  name         )            const uint32_t name = xReadUvlc (        )
#  define X_READ_SVLC_NO_RANGE(  name         )            const int32_t  name = xReadSvlc (        )
#  define X_READ_CODE_NO_RANGE(  name, length )            const uint32_t name = xReadCode ( length )
#  define X_READ_SCODE_NO_RANGE( name, length )            const int32_t  name = xReadSCode( length )

#  define X_READ_UVLC_NO_RANGE_idx(  name, idx         )   const uint32_t name = xReadUvlc (        )
#  define X_READ_SVLC_NO_RANGE_idx(  name, idx         )   const int32_t  name = xReadSvlc (        )
#  define X_READ_CODE_NO_RANGE_idx(  name, idx, length )   const uint32_t name = xReadCode ( length )
#  define X_READ_SCODE_NO_RANGE_idx( name, idx, length )   const int32_t  name = xReadSCode( length )

#endif


#define X_READ_FLAG_CHECK( name,          chk_cond, chk_msg ) const bool name = [&]{ X_READ_FLAG     ( name      ); CHECK_READ( chk_cond, chk_msg, name ); return name; }()
#define X_READ_FLAG_CHECK_idx( name, idx, chk_cond, chk_msg ) const bool name = [&]{ X_READ_FLAG_idx ( name, idx ); CHECK_READ( chk_cond, chk_msg, name ); return name; }()

// use immediately invoked lambdas, to ensure we get a warning, when the result is unused
#define X_READ_UVLC( name,         min, max )                 const uint32_t name = [&]{ X_READ_UVLC_NO_RANGE( name         ); CHECK_READ_RANGE( name, min, max, #name ); return name; }()
#define X_READ_SVLC( name,         min, max )                 const int32_t  name = [&]{ X_READ_SVLC_NO_RANGE( name         ); CHECK_READ_RANGE( name, min, max, #name ); return name; }()
#define X_READ_CODE( name, length, min, max )                 const uint32_t name = [&]{ X_READ_CODE_NO_RANGE( name, length ); CHECK_READ_RANGE( name, min, max, #name ); return name; }()
#define X_READ_SCODE(name, length, min, max )                 const int32_t  name = [&]{ X_READ_SCODE_NO_RANGE(name, length ); CHECK_READ_RANGE( name, min, max, #name ); return name; }()

#define X_READ_UVLC_idx( name, idx,         min, max )        const uint32_t name = [&]{ X_READ_UVLC_NO_RANGE_idx( name, idx         ); CHECK_READ_RANGE( name, min, max, (#name idx) ); return name; }()
#define X_READ_SVLC_idx( name, idx,         min, max )        const int32_t  name = [&]{ X_READ_SVLC_NO_RANGE_idx( name, idx         ); CHECK_READ_RANGE( name, min, max, (#name idx) ); return name; }()
#define X_READ_CODE_idx( name, idx, length, min, max )        const uint32_t name = [&]{ X_READ_CODE_NO_RANGE_idx( name, idx, length ); CHECK_READ_RANGE( name, min, max, (#name idx) ); return name; }()
#define X_READ_SCODE_idx(name, idx, length, min, max )        const int32_t  name = [&]{ X_READ_SCODE_NO_RANGE_idx(name, idx, length ); CHECK_READ_RANGE( name, min, max, (#name idx) ); return name; }()


namespace vvdec
{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

class VLCReader
{
protected:
  InputBitstream* m_pcBitstream = nullptr;   // we never own this object

  VLCReader()          = default;
  virtual ~VLCReader() = default;

  // read functions taking a reference for the result
  void xReadFlag (                  uint32_t& val );
  void xReadUvlc (                  uint32_t& val );
  void xReadSvlc (                  int32_t&  val );
  void xReadCode ( uint32_t length, uint32_t& val );
  void xReadSCode( uint32_t length, int32_t&  val );

  // read functions taking a reference for the result - tracing overloads
  void xReadFlag (                  uint32_t& rValue, const char* pSymbolName );
  void xReadUvlc (                  uint32_t& rValue, const char* pSymbolName );
  void xReadSvlc (                  int32_t&  rValue, const char* pSymbolName );
  void xReadCode ( uint32_t length, uint32_t& rValue, const char* pSymbolName );
  void xReadSCode( uint32_t length, int32_t&  rValue, const char* pSymbolName );

  // read functions returning the result value
  bool     xReadFlag();
  uint32_t xReadUvlc();
  int32_t  xReadSvlc();
  uint32_t xReadCode( uint32_t length );
  int32_t  xReadSCode( uint32_t length );

  // read functions returning the result value - tracing overloads
  bool     xReadFlag (                  const char* pSymbolName );
  uint32_t xReadUvlc (                  const char* pSymbolName );
  int32_t  xReadSvlc (                  const char* pSymbolName );
  uint32_t xReadCode ( uint32_t length, const char* pSymbolName );
  int32_t  xReadSCode( uint32_t length, const char* pSymbolName );

public:
  void            setBitstream( InputBitstream* p ) { m_pcBitstream = p; }
  InputBitstream* getBitstream() { return m_pcBitstream; }

protected:
  void xReadRbspTrailingBits();
  bool isByteAligned() { return ( m_pcBitstream->getNumBitsUntilByteAligned() == 0 ); }
};

}   // namespace vvdec
