/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2018-2023, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVdeC Authors.
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

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

OutputBitstream::OutputBitstream()
{
  clear();
}

OutputBitstream::~OutputBitstream()
{
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

uint8_t* OutputBitstream::getByteStream() const
{
  return (uint8_t*) &m_fifo.front();
}

uint32_t OutputBitstream::getByteStreamLength()
{
  return uint32_t(m_fifo.size());
}

void OutputBitstream::clear()
{
  m_fifo.clear();
  m_held_bits = 0;
  m_num_held_bits = 0;
}

void OutputBitstream::write   ( uint32_t uiBits, uint32_t uiNumberOfBits )
{
  CHECK_RECOVERABLE( uiNumberOfBits > 32, "Number of bits is exceeds '32'" );
  CHECK_RECOVERABLE( uiNumberOfBits != 32 && (uiBits & (~0u << uiNumberOfBits)) != 0, "Unsupported parameters" );

  /* any modulo 8 remainder of num_total_bits cannot be written this time,
   * and will be held until next time. */
  uint32_t num_total_bits = uiNumberOfBits + m_num_held_bits;
  uint32_t next_num_held_bits = num_total_bits % 8;

  /* form a byte aligned word (write_bits), by concatenating any held bits
   * with the new bits, discarding the bits that will form the next_held_bits.
   * eg: H = held bits, V = n new bits        /---- next_held_bits
   * len(H)=7, len(V)=1: ... ---- HHHH HHHV . 0000 0000, next_num_held_bits=0
   * len(H)=7, len(V)=2: ... ---- HHHH HHHV . V000 0000, next_num_held_bits=1
   * if total_bits < 8, the value of v_ is not used */
  uint8_t next_held_bits = uiBits << (8 - next_num_held_bits);

  if (!(num_total_bits >> 3))
  {
    /* insufficient bits accumulated to write out, append new_held_bits to
     * current held_bits */
    /* NB, this requires that v only contains 0 in bit positions {31..n} */
    m_held_bits |= next_held_bits;
    m_num_held_bits = next_num_held_bits;
    return;
  }

  /* topword serves to justify held_bits to align with the msb of uiBits */
  uint32_t topword = (uiNumberOfBits - next_num_held_bits) & ~((1 << 3) -1);
  uint32_t write_bits = (m_held_bits << topword) | (uiBits >> next_num_held_bits);

  switch (num_total_bits >> 3)
  {
  case 4: m_fifo.push_back(write_bits >> 24);
  case 3: m_fifo.push_back(write_bits >> 16);
  case 2: m_fifo.push_back(write_bits >> 8);
  case 1: m_fifo.push_back(write_bits);
  }

  m_held_bits = next_held_bits;
  m_num_held_bits = next_num_held_bits;
}

void OutputBitstream::writeAlignOne()
{
  uint32_t num_bits = getNumBitsUntilByteAligned();
  write((1 << num_bits) - 1, num_bits);
  return;
}

void OutputBitstream::writeAlignZero()
{
  if (0 == m_num_held_bits)
  {
    return;
  }
  m_fifo.push_back(m_held_bits);
  m_held_bits = 0;
  m_num_held_bits = 0;
}

/**
 - add substream to the end of the current bitstream
 .
 \param  pcSubstream  substream to be added
 */
void   OutputBitstream::addSubstream( OutputBitstream* pcSubstream )
{
  uint32_t uiNumBits = pcSubstream->getNumberOfWrittenBits();

  const std::vector<uint8_t>& rbsp = pcSubstream->getFIFO();
  for (std::vector<uint8_t>::const_iterator it = rbsp.begin(); it != rbsp.end();)
  {
    write(*it++, 8);
  }
  if (uiNumBits&0x7)
  {
    write(pcSubstream->getHeldBits()>>(8-(uiNumBits&0x7)), uiNumBits&0x7);
  }
}

void OutputBitstream::writeByteAlignment()
{
  write( 1, 1);
  writeAlignZero();
}

int OutputBitstream::countStartCodeEmulations()
{
  uint32_t cnt = 0;
  std::vector<uint8_t>& rbsp   = getFIFO();
  for (std::vector<uint8_t>::iterator it = rbsp.begin(); it != rbsp.end();)
  {
    std::vector<uint8_t>::iterator found = it;
    do
    {
      // find the next emulated 00 00 {00,01,02,03}
      // NB, end()-1, prevents finding a trailing two byte sequence
      found = std::search_n(found, rbsp.end()-1, 2, 0);
      found++;
      // if not found, found == end, otherwise found = second zero byte
      if (found == rbsp.end())
      {
        break;
      }
      if (*(++found) <= 3)
      {
        break;
      }
    } while (true);
    it = found;
    if (found != rbsp.end())
    {
      cnt++;
    }
  }
  return cnt;
}

void InputBitstream::resetToStart()
{
  m_fifo_idx=0;
  m_num_held_bits=0;
  m_held_bits=0;
  m_numBitsRead=0;
}

/**
 * read uiNumberOfBits from bitstream without updating the bitstream
 * state, storing the result in ruiBits.
 *
 * If reading uiNumberOfBits would overrun the bitstream buffer,
 * the bitstream is effectively padded with sufficient zero-bits to
 * avoid the overrun.
 */
void InputBitstream::pseudoRead ( uint32_t uiNumberOfBits, uint32_t& ruiBits )
{
  uint32_t saved_num_held_bits = m_num_held_bits;
  uint8_t saved_held_bits = m_held_bits;
  uint32_t saved_fifo_idx = m_fifo_idx;

  uint32_t num_bits_to_read = std::min(uiNumberOfBits, getNumBitsLeft());
  read(num_bits_to_read, ruiBits);
  ruiBits <<= (uiNumberOfBits - num_bits_to_read);

  m_fifo_idx = saved_fifo_idx;
  m_held_bits = saved_held_bits;
  m_num_held_bits = saved_num_held_bits;
}


void InputBitstream::read (uint32_t uiNumberOfBits, uint32_t& ruiBits)
{
  CHECK_RECOVERABLE( uiNumberOfBits > 32, "Too many bits read" );

  m_numBitsRead += uiNumberOfBits;

  /* NB, bits are extracted from the MSB of each byte. */
  uint32_t retval = 0;
  if (uiNumberOfBits <= m_num_held_bits)
  {
    /* n=1, len(H)=7:   -VHH HHHH, shift_down=6, mask=0xfe
     * n=3, len(H)=7:   -VVV HHHH, shift_down=4, mask=0xf8
     */
    retval = m_held_bits >> (m_num_held_bits - uiNumberOfBits);
    retval &= ~(0xff << uiNumberOfBits);
    m_num_held_bits -= uiNumberOfBits;
    ruiBits = retval;
    return;
  }

  /* all num_held_bits will go into retval
   *   => need to mask leftover bits from previous extractions
   *   => align retval with top of extracted word */
  /* n=5, len(H)=3: ---- -VVV, mask=0x07, shift_up=5-3=2,
   * n=9, len(H)=3: ---- -VVV, mask=0x07, shift_up=9-3=6 */
  uiNumberOfBits -= m_num_held_bits;
  retval = m_held_bits & ~(0xff << m_num_held_bits);
  retval = retval ? (retval << uiNumberOfBits) : 0;   // sanity error when retval=0 && uiNumberOfBits=32


  /* number of whole bytes that need to be loaded to form retval */
  /* n=32, len(H)=0, load 4bytes, shift_down=0
   * n=32, len(H)=1, load 4bytes, shift_down=1
   * n=31, len(H)=1, load 4bytes, shift_down=1+1
   * n=8,  len(H)=0, load 1byte,  shift_down=0
   * n=8,  len(H)=3, load 1byte,  shift_down=3
   * n=5,  len(H)=1, load 1byte,  shift_down=1+3
   */
  uint32_t aligned_word = 0;
  uint32_t num_bytes_to_load = (uiNumberOfBits - 1) >> 3;
  CHECK_RECOVERABLE(m_fifo_idx + num_bytes_to_load >= m_fifo.size(), "Exceeded FIFO size");

  switch (num_bytes_to_load)
  {
  case 3: aligned_word  = m_fifo[m_fifo_idx++] << 24;
  case 2: aligned_word |= m_fifo[m_fifo_idx++] << 16;
  case 1: aligned_word |= m_fifo[m_fifo_idx++] <<  8;
  case 0: aligned_word |= m_fifo[m_fifo_idx++];
  }

  /* resolve remainder bits */
  uint32_t next_num_held_bits = (32 - uiNumberOfBits) % 8;

  /* copy required part of aligned_word into retval */
  retval |= aligned_word >> next_num_held_bits;

  /* store held bits */
  m_num_held_bits = next_num_held_bits;
  m_held_bits = aligned_word;

  ruiBits = retval;
}

/**
 * insert the contents of the bytealigned (and flushed) bitstream src
 * into this at byte position pos.
 */
void OutputBitstream::insertAt(const OutputBitstream& src, uint32_t pos)
{
  CHECK_RECOVERABLE(0 != src.getNumberOfWrittenBits() % 8, "Number of written bits is not a multiple of 8");

  std::vector<uint8_t>::iterator at = m_fifo.begin() + pos;
  m_fifo.insert(at, src.m_fifo.begin(), src.m_fifo.end());
}

uint32_t InputBitstream::readOutTrailingBits ()
{
  uint32_t count=0;
  uint32_t uiBits = 0;

  while ( ( getNumBitsLeft() > 0 ) && (getNumBitsUntilByteAligned()!=0) )
  {
    count++;
    read ( 1, uiBits );
  }
  return count;
}
//
//OutputBitstream& OutputBitstream::operator= (const OutputBitstream& src)
//{
//  vector<uint8_t>::iterator at = m_fifo.begin();
//  m_fifo.insert(at, src.m_fifo.begin(), src.m_fifo.end());
//
//  m_num_held_bits             = src.m_num_held_bits;
//  m_held_bits                 = src.m_held_bits;
//
//  return *this;
//}

/**
 Extract substream from the current bitstream.

 \param  uiNumBits    number of bits to transfer
 */
InputBitstream *InputBitstream::extractSubstream( uint32_t uiNumBits )
{
  uint32_t uiNumBytes = uiNumBits/8;
  InputBitstream *pResult = new InputBitstream;

  std::vector<uint8_t> &buf = pResult->getFifo();
  buf.reserve((uiNumBits+7)>>3);

  if (m_num_held_bits == 0)
  {
    std::size_t currentOutputBufferSize=buf.size();
    const uint32_t uiNumBytesToReadFromFifo = std::min<uint32_t>(uiNumBytes, (uint32_t)m_fifo.size() - m_fifo_idx);
    buf.resize(currentOutputBufferSize+uiNumBytes);
    memcpy(&(buf[currentOutputBufferSize]), &(m_fifo[m_fifo_idx]), uiNumBytesToReadFromFifo); m_fifo_idx+=uiNumBytesToReadFromFifo;
    if (uiNumBytesToReadFromFifo != uiNumBytes)
    {
      memset(&(buf[currentOutputBufferSize+uiNumBytesToReadFromFifo]), 0, uiNumBytes - uiNumBytesToReadFromFifo);
    }
  }
  else
  {
    for (uint32_t ui = 0; ui < uiNumBytes; ui++)
    {
      uint32_t uiByte;
      read(8, uiByte);
      buf.push_back(uiByte);
    }
  }
  if (uiNumBits&0x7)
  {
    uint32_t uiByte = 0;
    read(uiNumBits&0x7, uiByte);
    uiByte <<= 8-(uiNumBits&0x7);
    buf.push_back(uiByte);
  }
  return pResult;
}

uint32_t InputBitstream::readByteAlignment()
{
  uint32_t code = 0;
  read( 1, code );
  CHECK_RECOVERABLE(code != 1, "Code is not '1'");

  uint32_t numBits = getNumBitsUntilByteAligned();
  if(numBits)
  {
    CHECK_RECOVERABLE(numBits > getNumBitsLeft(), "More bits available than left");
    read( numBits, code );
    CHECK_RECOVERABLE(code != 0, "Code not '0'");
  }
  return numBits+1;
}

}
