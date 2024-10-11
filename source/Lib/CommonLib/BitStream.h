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

/** \file     BitStream.h
    \brief    class for handling bitstream (header)
*/

#pragma once

#include <stdint.h>
#include <vector>
#include <memory>
#include "CommonDef.h"

#ifdef TARGET_SIMD_X86
#  include "CommonDefX86.h"   // needed for simde_bswap64, but don't just include simde-common.h, because it breaks other files
#else
#  include "simde/simde-common.h"
#endif

namespace vvdec
{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/**
 * Model of an input bitstream that extracts bits from a predefined
 * bytestream.
 */
class InputBitstream
{
private:
  AlignedByteVec        m_fifo;   /// FIFO for storage of complete bytes
  std::vector<uint32_t> m_emulationPreventionByteLocation;

  uint32_t m_fifo_idx = 0;   /// Read index into m_fifo

  uint32_t m_num_held_bits = 0;
  uint64_t m_held_bits     = 0;

  bool m_zeroByteAdded = false;

#if ENABLE_TRACING
  uint32_t m_numBitsRead   = 0;
#endif

public:
  /**
   * Create a new bitstream reader object that reads from buf.
   */
  InputBitstream()  = default;
  ~InputBitstream() = default;
  CLASS_COPY_MOVE_DEFAULT( InputBitstream )

  void resetToStart();

  void inputZeroByte() { if( !m_zeroByteAdded ) m_fifo.push_back(0); };

  // interface for decoding
  uint32_t       peekBits( uint32_t uiNumberOfBits );
  uint32_t       read    ( uint32_t uiNumberOfBits );
  inline uint8_t readByte()
  {
#if ENABLE_TRACING
    m_numBitsRead += 8;
#endif

    if( m_num_held_bits )
    {
      CHECKD( m_num_held_bits & 7, "held bits should be byte-aligned" );
    }
    else
    {
      load_next_bits( 8 );
    }

    uint32_t uiBits = 0xff & ( m_held_bits >> ( m_num_held_bits - 8 ) );
    m_num_held_bits -= 8;
    return uiBits;
  }

  // In the function initBitstream, add a Byte at the end of m_fifo
  // In the past, readByteFlag judged whether to read data according to the flag, so it would not cross the boundary
  // Now, in order to speed up, readByteFlag will definitely read data, so it may cross the boundary
  inline uint8_t readByteFlag( uint8_t flag )
  {
#if ENABLE_TRACING
    m_numBitsRead += (flag & 1) * 8;
#endif

    if( m_num_held_bits )
    {
      CHECKD( m_num_held_bits & 7, "held bits should be byte-aligned" );
    }
    else
    {
      load_next_bits( 8 );
    }

    uint32_t uiBits = flag & ( m_held_bits >> ( m_num_held_bits - 8 ) );
    m_num_held_bits -= 8 * ( flag & 1 );
    return uiBits;
  }

  inline uint8_t peekPreviousByte()
  {
    CHECKD( m_num_held_bits & 7, "held bits should be byte-aligned" );

    // We don't know if m_heldBits actually contains the previous byte, because readByteFlag()
    // sometimes refills m_heldBits, but doesn't actually consume any of the loaded bits. So
    // we always need to look at the actual data buffer.
    const unsigned held_bytes = m_num_held_bits >> 3;
    CHECK( m_fifo_idx - held_bytes - 1 >= m_fifo.size(), "Exceeded FIFO size" );
    return m_fifo[m_fifo_idx - held_bytes - 1];
  }

  uint32_t         readOutTrailingBits();
  inline uint8_t   getHeldBits()     { return m_held_bits; }
  inline uint32_t  getByteLocation()
  {
    CHECKD( m_num_held_bits & 7, "held bits should be byte-aligned" );
    return m_fifo_idx - m_num_held_bits / 8;
  }

  inline uint8_t  getNumBitsUntilByteAligned() const { return m_num_held_bits & ( 0x7 ); }
  inline uint32_t getNumBitsLeft()             const { return ( m_fifo_idx < m_fifo.size() ? 8 * ( (uint32_t) m_fifo.size() - m_fifo_idx ) : 0 ) + m_num_held_bits; }
#if ENABLE_TRACING
  inline uint32_t getNumBitsRead()             const { return m_numBitsRead; }
#endif
  uint32_t        readByteAlignment();
  std::unique_ptr<InputBitstream> extractSubstream( uint32_t uiNumBits );   // Read the nominated number of bits, and return as a bitstream.

  void                         pushEmulationPreventionByteLocation( uint32_t pos )                    { m_emulationPreventionByteLocation.push_back( pos ); }
  uint32_t                     numEmulationPreventionBytesRead()                                      { return (uint32_t) m_emulationPreventionByteLocation.size(); }
  uint32_t                     getEmulationPreventionByteLocation( uint32_t idx )                     { return m_emulationPreventionByteLocation[idx]; }
  const std::vector<uint32_t>& getEmulationPreventionByteLocation() const                             { return m_emulationPreventionByteLocation; }
  void                         setEmulationPreventionByteLocation( const std::vector<uint32_t>& vec ) { m_emulationPreventionByteLocation = vec; }
  void                         clearEmulationPreventionByteLocation()                                 { m_emulationPreventionByteLocation.clear(); }

  const AlignedByteVec& getFifo() const { return m_fifo; }
        AlignedByteVec& getFifo()       { return m_fifo; }
  void                  clearFifo()     { m_fifo.clear(); m_zeroByteAdded = false; }

private:
  inline void load_next_bits( int requiredBits )
  {
    uint32_t num_bytes_to_load = 8;
    if UNLIKELY( m_fifo_idx + num_bytes_to_load > m_fifo.size()   // end of bitstream
                 || ( m_fifo_idx & 0x7 ) != 0 )                   // unaligned read position (m_fifo should be aligned)
    {
      const int required_bytes = ( requiredBits + 7 ) >> 3;
      CHECK( m_fifo_idx + required_bytes > m_fifo.size(), "Exceeded FIFO size" );

      num_bytes_to_load = (uint32_t)m_fifo.size() - m_fifo_idx;

      m_held_bits = 0;
      switch( num_bytes_to_load )
      {
      default: num_bytes_to_load = 8;   // in the unaligned case num_bytes_to_load could be >8
      case 8:  m_held_bits =  static_cast<uint64_t>( m_fifo[m_fifo_idx++] ) << ( 7 * 8 );
      case 7:  m_held_bits |= static_cast<uint64_t>( m_fifo[m_fifo_idx++] ) << ( 6 * 8 );
      case 6:  m_held_bits |= static_cast<uint64_t>( m_fifo[m_fifo_idx++] ) << ( 5 * 8 );
      case 5:  m_held_bits |= static_cast<uint64_t>( m_fifo[m_fifo_idx++] ) << ( 4 * 8 );
      case 4:  m_held_bits |= static_cast<uint64_t>( m_fifo[m_fifo_idx++] ) << ( 3 * 8 );
      case 3:  m_held_bits |= static_cast<uint64_t>( m_fifo[m_fifo_idx++] ) << ( 2 * 8 );
      case 2:  m_held_bits |= static_cast<uint64_t>( m_fifo[m_fifo_idx++] ) << ( 1 * 8 );
      case 1:  m_held_bits |= static_cast<uint64_t>( m_fifo[m_fifo_idx++] );
      }
    }
    else
    {
      CHECKD( reinterpret_cast<intptr_t>( &m_fifo[m_fifo_idx] ) & 0x7, "bistream read pos unaligned" );
      m_held_bits = simde_bswap64( *reinterpret_cast<uint64_t*>( &m_fifo[m_fifo_idx] ) );
      m_fifo_idx += num_bytes_to_load;
    }

    m_num_held_bits = num_bytes_to_load * 8;
  }
};

}
