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

/** \file     BitStream.cpp
    \brief    class for handling bitstream
*/

#include "BitStream.h"

#include <stdint.h>
#include <vector>
#include <string.h>
#include <memory.h>


namespace vvdec
{

void InputBitstream::resetToStart()
{
  m_fifo_idx      = 0;
  m_num_held_bits = 0;
  m_held_bits     = 0;
#if ENABLE_TRACING
  m_numBitsRead   = 0;
#endif
}

/**
 * read uiNumberOfBits from bitstream without updating the bitstream
 * state, storing the result in ruiBits.
 *
 * If reading uiNumberOfBits would overrun the bitstream buffer,
 * the bitstream is effectively padded with sufficient zero-bits to
 * avoid the overrun.
 */
uint32_t InputBitstream::peekBits( uint32_t uiNumberOfBits )
{
  auto saved_fifo_idx      = m_fifo_idx;
  auto saved_num_held_bits = m_num_held_bits;
  auto saved_held_bits     = m_held_bits;
#if ENABLE_TRACING
  auto saved_numBitsRead   = m_numBitsRead;
#endif

  uint32_t num_bits_to_read = std::min(uiNumberOfBits, getNumBitsLeft());
  uint32_t uiBits           = read( num_bits_to_read );
  uiBits <<= (uiNumberOfBits - num_bits_to_read);

  m_fifo_idx      = saved_fifo_idx;
  m_num_held_bits = saved_num_held_bits;
  m_held_bits     = saved_held_bits;
#if ENABLE_TRACING
  m_numBitsRead   = saved_numBitsRead;
#endif
  return uiBits;
}


uint32_t InputBitstream::read (uint32_t uiNumberOfBits)
{
#if ENABLE_TRACING
  m_numBitsRead += uiNumberOfBits;
#endif

  constexpr static uint64_t ONES = ~static_cast<uint64_t>( 0 );   // need to ensure 64 bits for the mask, because shift by 32 is UB for uin32_t

  /* NB, bits are extracted from the MSB of each byte. */
  uint32_t retval = 0;
  if (uiNumberOfBits <= m_num_held_bits)
  {
    /* n=1, len(H)=7:   -VHH HHHH, shift_down=6, mask=0xfe
     * n=3, len(H)=7:   -VVV HHHH, shift_down=4, mask=0xf8
     */
    retval = static_cast<uint32_t>( m_held_bits >> ( m_num_held_bits - uiNumberOfBits ) );
    retval &= ~( ONES << uiNumberOfBits );
    m_num_held_bits -= uiNumberOfBits;

    return retval;
  }

  CHECK( uiNumberOfBits > 32, "Too many bits read" );

  if( m_num_held_bits )
  {
    /* all num_held_bits will go into retval
     *   => need to mask leftover bits from previous extractions
     *   => align retval with top of extracted word */
    /* n=5, len(H)=3: ---- -VVV, mask=0x07, shift_up=5-3=2,
     * n=9, len(H)=3: ---- -VVV, mask=0x07, shift_up=9-3=6 */
    uiNumberOfBits -= m_num_held_bits;
    retval = static_cast<uint32_t>( m_held_bits ) & ~( ONES << m_num_held_bits );   // we can cast to 32 bits, because the held bits are the rightmost bits
    retval <<= uiNumberOfBits;
  }

  /* number of whole bytes that need to be loaded to form retval */
  /* n=32, len(H)=0, load 4bytes, shift_down=0
   * n=32, len(H)=1, load 4bytes, shift_down=1
   * n=31, len(H)=1, load 4bytes, shift_down=1+1
   * n=8,  len(H)=0, load 1byte,  shift_down=0
   * n=8,  len(H)=3, load 1byte,  shift_down=3
   * n=5,  len(H)=1, load 1byte,  shift_down=1+3
   */
  load_next_bits( uiNumberOfBits );

  /* resolve remainder bits */
  m_num_held_bits -= uiNumberOfBits;

  /* copy required part of m_held_bits into retval */
  retval |= static_cast<uint32_t>( m_held_bits >> m_num_held_bits );

  return retval;
}

uint32_t InputBitstream::readOutTrailingBits ()
{
  uint32_t count = 0;
  while( ( getNumBitsLeft() > 0 ) && ( getNumBitsUntilByteAligned() != 0 ) )
  {
    count++;
    read( 1 );
  }
  return count;
}

/**
 Extract substream from the current bitstream.

 \param  uiNumBits    number of bits to transfer
 */
std::unique_ptr<InputBitstream> InputBitstream::extractSubstream( uint32_t uiNumBits )
{
  std::unique_ptr<InputBitstream> substream( new InputBitstream );

  AlignedByteVec& buf = substream->getFifo();
  buf.reserve( ( uiNumBits + 7 ) / 8 + 1 );    // +1 because a zero byte might be added later

  const uint32_t uiNumBytes = uiNumBits / 8;   // don't round up here, because the remaing bits will be copied later
  if( m_num_held_bits == 0 )
  {
    CHECK( m_fifo_idx + uiNumBytes > m_fifo.size(), "Exceeded FIFO size" );

    std::size_t currentOutputBufferSize = buf.size();
    buf.resize( currentOutputBufferSize + uiNumBytes );

    const uint32_t uiNumBytesToReadFromFifo = std::min<uint32_t>( uiNumBytes, (uint32_t) m_fifo.size() - m_fifo_idx );
    memcpy( &( buf[currentOutputBufferSize] ), &( m_fifo[m_fifo_idx] ), uiNumBytesToReadFromFifo );
    m_fifo_idx += uiNumBytesToReadFromFifo;

    if( uiNumBytesToReadFromFifo != uiNumBytes )
    {
      memset( &( buf[currentOutputBufferSize + uiNumBytesToReadFromFifo] ), 0, uiNumBytes - uiNumBytesToReadFromFifo );
    }
  }
  else
  {
    for( uint32_t ui = 0; ui < uiNumBytes; ui++ )
    {
      uint32_t uiByte = read( 8 );
      buf.push_back( uiByte );
    }
  }

  if( uiNumBits & 0x7 )
  {
    uint32_t uiByte = read( uiNumBits & 0x7 );
    uiByte <<= 8 - ( uiNumBits & 0x7 );
    buf.push_back( uiByte );
  }
  return substream;
}

uint32_t InputBitstream::readByteAlignment()
{
  uint32_t code = read( 1 );
  CHECK( code != 1, "Code is not '1'" );

  uint32_t numBits = getNumBitsUntilByteAligned();
  if(numBits)
  {
    CHECK( numBits > getNumBitsLeft(), "More bits available than left" );
    code = read( numBits );
    CHECK( code != 0, "Code not '0'" );
  }
  return numBits+1;
}

}
