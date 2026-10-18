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

/** \file     BinDecoder.cpp
 *  \brief    Low level binary symbol writer
 */


#include "BinDecoder.h"
#include "CommonLib/Rom.h"
#include "CommonLib/BitStream.h"

#include "CommonLib/dtrace_next.h"

#include <limits>

#define CNT_OFFSET 0

namespace vvdec
{

// BinDecoder uses the `signed >> 31` sign-extract idiom, which is implementation-defined
// for negative operands before C++20. Require two's-complement representation so the idiom
// behaves as intended (mandatory anyway in C++20).
static_assert( std::numeric_limits<int>::min() == -std::numeric_limits<int>::max() - 1,
               "two's complement signed integer representation required" );
static_assert( ( -1 >> 1 ) == -1,
               "right shift of negative number needs to be implemented as arithmetic shift." );

void BinDecoder::init( InputBitstream* bitstream )
{
  m_Bitstream = bitstream;
}


void BinDecoder::uninit()
{
  m_Bitstream = 0;
}


void BinDecoder::start()
{
  CHECK( m_Bitstream->getNumBitsUntilByteAligned(), "Bitstream is not byte aligned." );
  m_Range       = 510;
  m_Value       = ( uint32_t( m_Bitstream->readByte() ) << 8 ) + m_Bitstream->readByte();
  m_bitsNeeded  = -8;
}


void BinDecoder::finish()
{
  unsigned lastByte = m_Bitstream->peekPreviousByte();
  CHECK( ( ( lastByte << ( 8 + m_bitsNeeded ) ) & 0xff ) != 0x80,
        "No proper stop/alignment pattern at end of CABAC stream." );
}


void BinDecoder::reset( int qp, int initId )
{
  m_Ctx.init( qp, initId );
  start();
}


// decodeBinEP() is defined ALWAYS_INLINE in BinDecoder.h (moved for inlining into the CABAC hot loops).


unsigned BinDecoder::decodeBinsEP( unsigned numBins )
{
#if ENABLE_TRACING
  int numBinsOrig = numBins;
#endif

  if( m_Range == 256 )
  {
    return decodeAlignedBinsEP( numBins );
  }
  unsigned remBins = numBins;
  unsigned bins    = 0;
  unsigned value   = m_Value;
  unsigned range = m_Range;
  int bitsNeeded = m_bitsNeeded;
  while( remBins > 8 )
  {
    value       = ( value << 8 ) + ( m_Bitstream->readByte() << ( 8 + bitsNeeded ) );
    unsigned SR =   range << 15;
    for( int i = 0; i < 8; i++ )
    {
      bins += bins;
      SR  >>= 1;
      if( value >= SR )
      {
        bins    ++;
        value -= SR;
      }
    }
    remBins -= 8;
  }
  bitsNeeded   += (int)remBins;
  value       <<= remBins;
  if( bitsNeeded >= 0 )
  {
    value      += m_Bitstream->readByte() << bitsNeeded;
    bitsNeeded -= 8;
  }
  unsigned SR = range << ( remBins + 7 );
  for ( unsigned i = 0; i < remBins; i++ )
  {
    bins += bins;
    SR  >>= 1;
    if( value >= SR )
    {
      bins    ++;
      value -= SR;
    }
  }
  m_Value = value;
  m_Range = range;
  m_bitsNeeded = bitsNeeded;
#if ENABLE_TRACING
  for( int i = 0; i < numBinsOrig; i++ )
  {
    DTRACE( g_trace_ctx, D_CABAC, "%d" "  " "%d" "  EP=%d \n", DTRACE_GET_COUNTER( g_trace_ctx, D_CABAC ), m_Range, ( bins >> ( numBinsOrig - 1 - i ) ) & 1 );
  }
#endif
  return bins;
}

unsigned BinDecoder::decodeRemAbsEP(unsigned goRicePar, unsigned cutoff, int maxLog2TrDynamicRange)
{
  unsigned prefix = 0;
  {
    const unsigned  maxPrefix = 32 - maxLog2TrDynamicRange;
    unsigned        codeWord = 0;
    do
    {
      prefix++;
      codeWord = decodeBinEP();
    } while (codeWord && prefix < maxPrefix);
    prefix -= 1 - (int)codeWord;
  }

  unsigned length = goRicePar, offset;
  if (prefix < cutoff)
  {
    offset = prefix << goRicePar;
  }
  else
  {
    offset = (((1u << (prefix - cutoff)) + cutoff - 1) << goRicePar);
    {
      length += (prefix == (32u - maxLog2TrDynamicRange) ? maxLog2TrDynamicRange - goRicePar : prefix - cutoff);
    }
  }
  return offset + decodeBinsEP(length);
}

// decodeBinTrm() is defined ALWAYS_INLINE in BinDecoder.h (moved for inlining into the CABAC hot loops).


void BinDecoder::align()
{
  m_Range = 256;
}

#if ENABLE_TRACING
unsigned int BinDecoder::getNumBitsRead() const
{
  return m_Bitstream->getNumBitsRead() + m_bitsNeeded;
}
#endif   // ENABLE_TRACING

unsigned BinDecoder::decodeAlignedBinsEP( unsigned numBins )
{
#if ENABLE_TRACING
  int numBinsOrig = numBins;
#endif
  unsigned remBins = numBins;
  unsigned bins    = 0;
  while( remBins > 0 )
  {
    // The MSB of m_Value is known to be 0 because range is 256. Therefore:
    //   > The comparison against the symbol range of 128 is simply a test on the next-most-significant bit
    //   > "Subtracting" the symbol range if the decoded bin is 1 simply involves clearing that bit.
    //  As a result, the required bins are simply the <binsToRead> next-most-significant bits of m_Value
    //  (m_Value is stored MSB-aligned in a 16-bit buffer - hence the shift of 15)
    //
    //    m_Value = |0|V|V|V|V|V|V|V|V|B|B|B|B|B|B|B|        (V = usable bit, B = potential buffered bit (buffer refills when m_bitsNeeded >= 0))
    //
    unsigned binsToRead = std::min<unsigned>( remBins, 8 ); //read bytes if able to take advantage of the system's byte-read function
    unsigned binMask    = ( 1 << binsToRead ) - 1;
    unsigned newBins    = ( m_Value >> (15 - binsToRead) ) & binMask;
    bins                = ( bins    << binsToRead) | newBins;
    m_Value             = ( m_Value << binsToRead) & 0x7FFF;
    remBins            -= binsToRead;
    m_bitsNeeded       += (int)binsToRead;
    if( m_bitsNeeded >= 0 )
    {
      m_Value          |= m_Bitstream->readByte() << m_bitsNeeded;
      m_bitsNeeded     -= 8;
    }
  }
#if ENABLE_TRACING
  for( int i = 0; i < numBinsOrig; i++ )
  {
    DTRACE( g_trace_ctx, D_CABAC, "%d" "  " "%d" "  " "EP=%d \n", DTRACE_GET_COUNTER( g_trace_ctx, D_CABAC ), m_Range, ( bins >> ( numBinsOrig - 1 - i ) ) & 1 );
  }
#endif
  return bins;
}


// decodeBin() is defined ALWAYS_INLINE in BinDecoder.h (moved for inlining into the CABAC hot loops).

}
