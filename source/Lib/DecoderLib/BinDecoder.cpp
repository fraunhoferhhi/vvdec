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

/** \file     BinDecoder.cpp
 *  \brief    Low level binary symbol writer
 */


#include "BinDecoder.h"
#include "CommonLib/Rom.h"
#include "CommonLib/BitStream.h"

#include "CommonLib/dtrace_next.h"

#define CNT_OFFSET 0

namespace vvdec
{

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
  m_Value       = ( m_Bitstream->readByte() << 8 ) + m_Bitstream->readByte();
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


unsigned BinDecoder::decodeBinEP()
{
  int value = m_Value << 1;
  if (++m_bitsNeeded >= 0) {
    value += m_Bitstream->readByte();
    m_bitsNeeded = -8;
  }

  unsigned bin = 0;
  unsigned SR = m_Range << 7;
  if (value >= SR) {
    value -= SR;
    bin = 1;
  }
  m_Value = value;
  DTRACE(g_trace_ctx, D_CABAC,
         "%d"
         "  "
         "%d"
         "  EP=%d \n",
         DTRACE_GET_COUNTER(g_trace_ctx, D_CABAC), m_Range, bin);
  return bin;
}


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
  int value = m_Value;
  int range = m_Range;
  int bitsNeeded = m_bitsNeeded;
  while(   remBins > 8 )
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
  bitsNeeded   += remBins;
  value       <<= remBins;
  if( bitsNeeded >= 0 )
  {
    value      += m_Bitstream->readByte() << bitsNeeded;
    bitsNeeded -= 8;
  }
  unsigned SR = range << ( remBins + 7 );
  for ( int i = 0; i < remBins; i++ )
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
    prefix -= 1 - codeWord;
  }

  unsigned length = goRicePar, offset;
  if (prefix < cutoff)
  {
    offset = prefix << goRicePar;
  }
  else
  {
    offset = (((1 << (prefix - cutoff)) + cutoff - 1) << goRicePar);
    {
      length += (prefix == (32 - maxLog2TrDynamicRange) ? maxLog2TrDynamicRange - goRicePar : prefix - cutoff);
    }
  }
  return offset + decodeBinsEP(length);
}

unsigned BinDecoder::decodeBinTrm()
{
  m_Range    -= 2;
  unsigned SR = m_Range << 7;
  
  DTRACE( g_trace_ctx, D_CABAC, "%d" " binTrm  range=" "%d" "  bin=%d\n" , DTRACE_GET_COUNTER( g_trace_ctx, D_CABAC ), m_Range, m_Value >= SR );

  if( m_Value >= SR )
  {
    return 1;
  }
  else
  {
    if( m_Range < 256 )
    {
      m_Range += m_Range;
      m_Value += m_Value;
      if( ++m_bitsNeeded == 0 )
      {
        m_Value      += m_Bitstream->readByte();
        m_bitsNeeded  = -8;
      }
    }
    return 0;
  }
}


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
  while(   remBins > 0 )
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
    m_bitsNeeded       += binsToRead;
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


unsigned BinDecoder::decodeBin( unsigned ctxId )
{
  BinProbModel& rcProbModel = m_Ctx[ctxId];

  unsigned bin, LPS;
  uint32_t Range      = m_Range;
  uint32_t Value      = m_Value;
  int32_t  bitsNeeded = m_bitsNeeded;

  rcProbModel.lpsmps( Range, LPS, bin );

//  DTRACE( g_trace_ctx, D_CABAC, "%d" " xxx " "%d" "  " "[%d:%d]" "  " "%2d(MPS=%d)"  "  " , DTRACE_GET_COUNTER( g_trace_ctx, D_CABAC ), m_Range, m_Range-LPS, LPS, ( unsigned int )( rcProbModel.state() ), m_Value < ( ( m_Range - LPS ) << 7 ) );
  DTRACE( g_trace_ctx, D_CABAC, "%d" " %d " "%d" "  " "[%d:%d]" "  " "%2d(MPS=%d)"  "  " , DTRACE_GET_COUNTER( g_trace_ctx, D_CABAC ), 666, Range, Range-LPS, LPS, ( unsigned int )( rcProbModel.state() ), Value < ( ( Range - LPS ) << 7 ) );
  //DTRACE( g_trace_ctx, D_CABAC, " %d " "%d" "  " "[%d:%d]" "  " "%2d(MPS=%d)"  "  ", DTRACE_GET_COUNTER( g_trace_ctx, D_CABAC ), m_Range, m_Range - LPS, LPS, (unsigned int)( rcProbModel.state() ), m_Value < ( ( m_Range - LPS ) << 7 ) );

  Range     -=  LPS;
  uint32_t      SR          = Range << 7;

  int b = ~( ( int( Value ) - int( SR ) ) >> 31 );
  int a = ~b & ( ( int( Range ) - 256 ) >> 31 );
  //int b = -( Value >= SR );
  
  int numBits  = ( a & rcProbModel.getRenormBitsRange( Range ) ) | ( b & rcProbModel.getRenormBitsLPS( LPS ) );
  
  Value       -= b & SR;
  Value      <<= numBits;
  
  Range       &=  ~b;
  Range       |= ( b & LPS );
  Range      <<= numBits;
  
  // b    0 0 1 1
  // bin  0 1 0 1
  // res  0 1 1 0

  //bin          = ( ~b & bin ) | ( b & !bin );
  bin         ^= b;
  bin         &= 1;
  
  bitsNeeded  += numBits & ( a | b );
  
  const int c = ~(bitsNeeded >> 31);
  Value      += m_Bitstream->readByteFlag( c ) << ( bitsNeeded & 31 );
  bitsNeeded -= c & 8;
  
  //if( Value < SR )
  //{
  //  // MPS path
  //  if( Range < 256 )
  //  {
  //    int numBits   = rcProbModel.getRenormBitsRange( Range );
  //    Range       <<= numBits;
  //    Value       <<= numBits;
  //    bitsNeeded   += numBits;
  //    if( bitsNeeded >= 0 )
  //    {
  //      Value      += m_Bitstream->readByte() << bitsNeeded;
  //      bitsNeeded -= 8;
  //    }
  //  }
  //}
  //else
  //{
  //  bin = !bin;
  //  // LPS path
  //  int numBits   = rcProbModel.getRenormBitsLPS( LPS );
  //  Value        -= SR;
  //  Value       <<= numBits;
  //  Range         = LPS     << numBits;
  //  bitsNeeded   += numBits;
  //  if( bitsNeeded >= 0 )
  //  {
  //    Value      += m_Bitstream->readByte() << bitsNeeded;
  //    bitsNeeded -= 8;
  //  }
  //}

  m_Range      = Range;
  m_Value      = Value;
  m_bitsNeeded = bitsNeeded;

  rcProbModel.update( bin );
  DTRACE_WITHOUT_COUNT( g_trace_ctx, D_CABAC, "  -  " "%d" "\n", bin );
  return  bin;
}

}
