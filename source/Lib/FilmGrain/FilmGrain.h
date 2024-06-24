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

/* This file is based on VFGS, available on
 * https://github.com/InterDigitalInc/VersatileFilmGrain
 *
 * VFGS implements film grain synthesis as a hardware model: it simulates the
 * output of a cost-effective hardware implementation in a video display
 * pipeline. Also, the C code is split into "fw" (firmware) and "hw" (hardware)
 * parts, and as self-explanatory as possible. See VFGS github repository for
 * more details.
 *
 * The VFGS github repository also contains other tools to experiment with film
 * grain synthesis (e.g. a graphical display and tuning tool for FGC SEI
 * message).
 */

#pragma once

#include "FilmGrainImpl.h"

#include <cstring>
#include <vector>
#include <memory>

#include "vvdec/sei.h"
#include "vvdec/vvdec.h"

#define USE_SIMD

namespace vvdec
{

#define SEI_MAX_MODEL_VALUES 6

struct fgs_sei
{
  fgs_sei() { memset( this, 0, sizeof( *this ) ); }

  uint8_t  model_id;
  uint8_t  log2_scale_factor;
  uint8_t  comp_model_present_flag[3];
  uint16_t num_intensity_intervals[3];
  uint8_t  num_model_values[3];
  uint8_t  intensity_interval_lower_bound[3][256];
  uint8_t  intensity_interval_upper_bound[3][256];
  int16_t  comp_model_value[3][256][SEI_MAX_MODEL_VALUES];
};

class FilmGrain
{
  std::unique_ptr<FilmGrainImpl> m_impl;

  uint32_t m_line_rnd               = 0xdeadbeef;
  uint32_t m_line_rnd_up            = 0xdeadbeef;
  uint32_t m_prev_frame_line_rnd_up = 0xdeadbeef;

  std::vector<uint32_t> m_line_seeds;
  fgs_sei               fgs;

public:
  FilmGrain();
  ~FilmGrain() = default;

  void updateFGC( vvdecSEIFilmGrainCharacteristics* fgc );
  void setDepth( int depth ) { m_impl->set_depth( depth ); }
  void setColorFormat( vvdecColorFormat fmt );
  void prepareBlockSeeds( int width, int height );

  void add_grain_line( void* Y, void* U, void* V, int y, int width );

private:
  void set_seed( uint32_t seed );
  void init_sei();
};

}   // namespace vvdec
