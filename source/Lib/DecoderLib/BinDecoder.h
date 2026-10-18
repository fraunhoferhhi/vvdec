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

/** \file     BinDecoder.h
 *  \brief    Low level binary symbol writer
 */

#pragma once

#include "CommonLib/Contexts.h"
#include "CommonLib/BitStream.h"      // complete type needed to inline readByte/readByteFlag below
#include "CommonLib/dtrace_next.h"    // DTRACE macros (no-ops in release) used by the inlined bodies

namespace vvdec
{
class InputBitstream;

class BinDecoder
{
public:
  BinDecoder()  = default;
  ~BinDecoder() = default;

  void      init    ( InputBitstream* bitstream );
  void      uninit  ();
  void      start   ();
  void      finish  ();
  void      reset   ( int qp, int initId );

  const Ctx& getCtx() const           { return m_Ctx; }
  void       setCtx( const Ctx& ctx ) { m_Ctx = ctx; }

  unsigned          decodeBin           ( unsigned ctxId );
  unsigned          decodeBinEP         ();
  unsigned          decodeBinsEP        ( unsigned numBins  );
  unsigned          decodeRemAbsEP      ( unsigned goRicePar, unsigned cutoff, int maxLog2TrDynamicRange );
  unsigned          decodeBinTrm        ();
  void              align               ();
#if ENABLE_TRACING
  unsigned          getNumBitsRead() const;
#endif

private:
  unsigned          decodeAlignedBinsEP ( unsigned numBins  );

  Ctx m_Ctx;

protected:
  InputBitstream* m_Bitstream  = nullptr;
  uint32_t        m_Range      = 0;
  uint32_t        m_Value      = 0;
  int32_t         m_bitsNeeded = 0;
};

// ---------------------------------------------------------------------------
// Hot-path bin decoders are defined here (not in BinDecoder.cpp) and forced
// ALWAYS_INLINE so they inline into the CABAC parser hot loops (residual_coding,
// coding_tree, ...) which live in a different translation unit. This keeps
// m_Range/m_Value/m_bitsNeeded in registers across consecutive bins instead of
// reloading/storing them through `this` on every bin. Pure relocation: bodies
// are identical to the previous out-of-line definitions -> bit-exact output.
// ---------------------------------------------------------------------------

ALWAYS_INLINE unsigned BinDecoder::decodeBinEP()
{
  unsigned value = m_Value << 1;
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

ALWAYS_INLINE unsigned BinDecoder::decodeBinTrm()
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

ALWAYS_INLINE unsigned BinDecoder::decodeBin( unsigned ctxId )
{
  BinProbModel& rcProbModel = m_Ctx[ctxId];

  unsigned bin, LPS, s0, s1;
  uint32_t range      = m_Range;
  uint32_t value      = m_Value;
  int32_t  bitsNeeded = m_bitsNeeded;

  rcProbModel.lpsmps( range, LPS, bin, s0, s1 );

  DTRACE( g_trace_ctx, D_CABAC, "%d" " %d " "%d" "  " "[%d:%d]" "  " "%2d(MPS=%d)"  "  " , DTRACE_GET_COUNTER( g_trace_ctx, D_CABAC ), 666, range, range-LPS, LPS, ( unsigned int )( rcProbModel.state() ), value < ( ( range - LPS ) << 7 ) );

  range      -= LPS;
  uint32_t SR = range << 7;

  int b = ~( ( int( value ) - int( SR ) ) >> 31 );
  int a = ~b & ( ( int( range ) - 256 ) >> 31 );

  int numBits  = ( a & rcProbModel.getRenormBitsRange( range ) ) | ( b & rcProbModel.getRenormBitsLPS( LPS ) );

  value       -= b & SR;
  value      <<= numBits;

  range       &=  ~b;
  range       |= ( b & LPS );
  range      <<= numBits;

  bin         ^= b;
  bin         &= 1;

  bitsNeeded  += numBits & ( a | b );

  const int c = ~(bitsNeeded >> 31);
  value      += uint32_t( m_Bitstream->readByteFlag( c ) ) << ( bitsNeeded & 31 );
  bitsNeeded -= c & 8;

  m_Range      = range;
  m_Value      = value;
  m_bitsNeeded = bitsNeeded;

  rcProbModel.update( bin, s0, s1 );
  DTRACE_WITHOUT_COUNT( g_trace_ctx, D_CABAC, "  -  " "%d" "\n", bin );
  return  bin;
}

}
