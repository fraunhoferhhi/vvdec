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

/**
 \file     NALread.cpp
 \brief    reading functionality for NAL units
 */


#include <vector>
#include <algorithm>
#include <ostream>

#include "NALread.h"

#include "CommonLib/NAL.h"
#include "CommonLib/BitStream.h"
#include "CommonLib/Rom.h"
#include "CommonLib/dtrace_next.h"

namespace vvdec
{

//! \ingroup DecoderLib
//! \{
static void convertPayloadToRBSP(std::vector<uint8_t>& nalUnitBuf, InputBitstream *bitstream, bool isVclNalUnit)
{
  uint32_t zeroCount = 0;
  std::vector<uint8_t>::iterator it_read, it_write;

  uint32_t pos = 0;
  bitstream->clearEmulationPreventionByteLocation();
  for (it_read = it_write = nalUnitBuf.begin(); it_read != nalUnitBuf.end(); it_read++, it_write++, pos++)
  {
    CHECK_RECOVERABLE(zeroCount >= 2 && *it_read < 0x03, "Zero count is '2' and read value is small than '3'");
    if (zeroCount == 2 && *it_read == 0x03)
    {
      bitstream->pushEmulationPreventionByteLocation( pos );
      pos++;
      it_read++;
      zeroCount = 0;
      if (it_read == nalUnitBuf.end())
      {
        break;
      }
      CHECK_RECOVERABLE(*it_read > 0x03, "Read a value bigger than '3'");
    }
    zeroCount = (*it_read == 0x00) ? zeroCount+1 : 0;
    *it_write = *it_read;
  }
  CHECK_RECOVERABLE(zeroCount != 0, "Zero count not '0'");

  if (isVclNalUnit)
  {
    // Remove cabac_zero_word from payload if present
    int n = 0;

    while (it_write[-1] == 0x00)
    {
      it_write--;
      n++;
    }

    if (n > 0)
    {
      msg( VERBOSE, "\nDetected %d instances of cabac_zero_word\n", n/2);
    }
  }

  nalUnitBuf.resize(it_write - nalUnitBuf.begin());
}

#if ENABLE_TRACING
static void xTraceNalUnitHeader(InputNALUnit& nalu)
{
  DTRACE( g_trace_ctx, D_NALUNITHEADER, "*********** NAL UNIT (%s) ***********\n", nalUnitTypeToString(nalu.m_nalUnitType) );
  bool zeroTidRequiredFlag = 0;
  if((nalu.m_nalUnitType >= 16) && (nalu.m_nalUnitType <= 31)) {
    zeroTidRequiredFlag = 1;
  }
  DTRACE( g_trace_ctx, D_NALUNITHEADER, "%-50s u(%d)  : %u\n", "zero_tid_required_flag", 1, zeroTidRequiredFlag );
  DTRACE( g_trace_ctx, D_NALUNITHEADER, "%-50s u(%d)  : %u\n", "nuh_temporal_id_plus1", 3, nalu.m_temporalId + 1 );
  DTRACE( g_trace_ctx, D_NALUNITHEADER, "%-50s u(%d)  : %u\n", "nal_unit_type_lsb", 4, (nalu.m_nalUnitType) - (zeroTidRequiredFlag << 4));
  DTRACE( g_trace_ctx, D_NALUNITHEADER, "%-50s u(%d)  : %u\n", "nuh_layer_id_plus1", 7, nalu.m_nuhLayerId+1);
  DTRACE( g_trace_ctx, D_NALUNITHEADER, "%-50s u(%d)  : %u\n", "nuh_reserved_zero_bit", 1, 0 );
}
#endif

void InputNALUnit::readNalUnitHeader()
{
  InputNALUnit&   nalu = *this;
  InputBitstream& bs   = nalu.getBitstream();

  nalu.m_forbiddenZeroBit   = bs.read(1);                 // forbidden zero bit
  nalu.m_nuhReservedZeroBit = bs.read(1);                 // nuh_reserved_zero_bit
  nalu.m_nuhLayerId         = bs.read(6);                 // nuh_layer_id
  CHECK_RECOVERABLE( nalu.m_nuhLayerId < 0, "this needs to be adjusted for the reco yuv output" );
  CHECK_RECOVERABLE(nalu.m_nuhLayerId > 55, "The value of nuh_layer_id shall be in the range of 0 to 55, inclusive");
  nalu.m_nalUnitType        = (NalUnitType) bs.read(5);   // nal_unit_type
  nalu.m_temporalId         = bs.read(3) - 1;             // nuh_temporal_id_plus1

#if ENABLE_TRACING
  xTraceNalUnitHeader(nalu);
#endif

  // only check these rules for base layer
  if (nalu.m_nuhLayerId == 0)
  {
    if ( nalu.m_temporalId )
    {
    }
    else
    {
      CHECK_RECOVERABLE( nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_STSA, "When NAL unit type is equal to STSA_NUT, TemporalId shall not be equal to 0" );
    }
  }
}
/**
 * create a NALunit structure with given header values and storage for
 * a bitstream
 */
void InputNALUnit::read()
{
  std::vector<uint8_t> & nalUnitBuf = m_Bitstream.getFifo();

  // perform anti-emulation prevention
  convertPayloadToRBSP( nalUnitBuf, &m_Bitstream, ( nalUnitBuf[0] & 64 ) == 0 );
  m_Bitstream.resetToStart();
  readNalUnitHeader();
}

bool InputNALUnit::checkPictureHeaderInSliceHeaderFlag( InputNALUnit& nalu )
{
  InputBitstream& bitstream = nalu.getBitstream();
  CHECK_RECOVERABLE(bitstream.getByteLocation() != 2, "The picture_header_in_slice_header_flag is the first bit after the NAL unit header");
  return (bool)bitstream.read(1);
}

}
