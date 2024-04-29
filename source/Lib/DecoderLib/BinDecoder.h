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

/** \file     BinDecoder.h
 *  \brief    Low level binary symbol writer
 */

#pragma once

#include "CommonLib/Contexts.h"

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

}
