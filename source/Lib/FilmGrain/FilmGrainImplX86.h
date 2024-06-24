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

#include "FilmGrainImpl.h"

#include <CommonDef.h>
#include <CommonDefX86.h>

#include <memory>

namespace vvdec
{

template<X86_VEXT VEXT>
class FilmGrainImplX86 : public FilmGrainImpl
{
public:
  static std::unique_ptr<FilmGrainImpl> makeFilmGrainImpl();

protected:
  void make_grain_pattern( const void* I,
                           int         c,
                           int         x,
                           int         subx,
                           uint8_t     oc1,
                           uint8_t     oc2,
                           uint8_t     ox,
                           uint8_t     ox_up,
                           uint8_t     oy,
                           uint8_t     oy_up,
                           int         s,
                           int         s_up,
                           int16_t     grain[3][32],
                           uint8_t     scale[3][32] ) const override;
  void scale_and_output( void*   I,   //
                         int     c,
                         int     x,
                         int     subx,
                         int     width,
                         int16_t grain[3][32],
                         uint8_t scale[3][32] ) const override;
};

template<>
inline std::unique_ptr<FilmGrainImpl> FilmGrainImplX86<UNDEFINED>::makeFilmGrainImpl()
{
  switch( read_x86_extension_flags() )
  {
  case AVX512:
  case AVX2:
#if ENABLE_AVX2_IMPLEMENTATIONS
    return std::make_unique<FilmGrainImplX86<AVX2>>();
#endif
  case AVX:
  case SSE42:
  case SSE41:
    return std::make_unique<FilmGrainImplX86<SSE41>>();
  default:
    return std::make_unique<FilmGrainImpl>();
  }
}

}   // namespace vvdec
