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

#pragma once

#include "CommonDef.h"

namespace vvdec
{

/**
 * Represents a single NALunit header and the associated RBSPayload
 */
struct NALUnit
{
  NalUnitType m_nalUnitType = NAL_UNIT_INVALID;   ///< nal_unit_type
  uint32_t    m_temporalId  = 0;                  ///< temporal_id
  uint32_t    m_nuhLayerId  = 0;                  ///< nuh_layer_id
  uint32_t    m_forbiddenZeroBit   = 0;
  uint32_t    m_nuhReservedZeroBit = 0;

  uint64_t    m_bits     = 0;            ///< original nal unit bits
  uint64_t    m_cts      = 0;            ///< composition time stamp in TicksPerSecond
  uint64_t    m_dts      = 0;            ///< decoding time stamp in TicksPerSecond
  bool        m_rap      = false;        ///< random access point flag

  NALUnit(const NALUnit &src)
    : m_nalUnitType (src.m_nalUnitType)
    , m_temporalId  (src.m_temporalId)
    , m_nuhLayerId  (src.m_nuhLayerId)
    , m_forbiddenZeroBit  (src.m_forbiddenZeroBit)
    , m_nuhReservedZeroBit(src.m_nuhReservedZeroBit)
  {}

  /** construct an NALunit structure with given header values. */
  NALUnit( NalUnitType nalUnitType,
           int         temporalId = 0 ,
           int         nuhLayerId = 0 )
    : m_nalUnitType (nalUnitType)
    , m_temporalId  (temporalId)
    , m_nuhLayerId  (nuhLayerId)
  {}

  /** default constructor - no initialization; must be performed by user */
  NALUnit() {}

  virtual ~NALUnit() { }

  /** returns true if the NALunit is a slice NALunit */
  bool isSlice()
  {
    return m_nalUnitType == NAL_UNIT_CODED_SLICE_TRAIL
        || m_nalUnitType == NAL_UNIT_CODED_SLICE_STSA
        || m_nalUnitType == NAL_UNIT_CODED_SLICE_RADL
        || m_nalUnitType == NAL_UNIT_CODED_SLICE_RASL
        || m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL
        || m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP
        || m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA
        || m_nalUnitType == NAL_UNIT_CODED_SLICE_GDR
        ;
  }
  bool isSei()
  {
    return m_nalUnitType == NAL_UNIT_PREFIX_SEI
        || m_nalUnitType == NAL_UNIT_SUFFIX_SEI;
  }

  bool isVcl()
  {
    return isVclNalUnitType(m_nalUnitType);
  }

  static bool isVclNalUnitType(NalUnitType t)
  {
    return t <= NAL_UNIT_RESERVED_IRAP_VCL_11;
  }
};

}
