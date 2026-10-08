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

#include <cstdint>
#include <cstring>
#include <iostream>
#include <sstream>
#include <vector>

#include "FilmGrain/FilmGrain.h"

using namespace vvdec;

namespace
{

void fillFilmGrainMessage( vvdecSEIFilmGrainCharacteristics& fgc, int presentComponent )
{
  memset( &fgc, 0, sizeof( fgc ) );
  fgc.filmGrainCharacteristicsCancelFlag      = false;
  fgc.filmGrainModelId                        = 0;
  fgc.log2ScaleFactor                         = 2;
  fgc.filmGrainCharacteristicsPersistenceFlag = true;

  if( presentComponent >= 0 )
  {
    vvdecCompModel& cm        = fgc.compModel[presentComponent];
    cm.presentFlag            = true;
    cm.numModelValues         = 3;
    cm.numIntensityIntervals  = 1;
    vvdecCompModelIntensityValues& iv = cm.intensityValues[0];
    iv.intensityIntervalLowerBound    = 0;
    iv.intensityIntervalUpperBound    = 255;
    iv.compModelValue[0]              = 64;
    iv.compModelValue[1]              = 8;
    iv.compModelValue[2]              = 8;
  }
}

// Decoder-order application: updateFGC (SEI applied at parse time), then
// setDepth/setColorFormat/prepareBlockSeeds/add_grain_line (applied at
// picture output time), mirroring VVDecImpl::xUpdateFGC / xAddGrain.
void applyFilmGrain( FilmGrain& fg, vvdecSEIFilmGrainCharacteristics& fgc,
                     uint8_t* y, uint8_t* u, uint8_t* v, int width, int height )
{
  fg.updateFGC( &fgc );
  fg.setDepth( 8 );
  fg.setColorFormat( VVDEC_CF_YUV420_PLANAR );
  fg.prepareBlockSeeds( width, height );

  const int chromaWidth = width / 2;
  for( int row = 0; row < height; row++ )
  {
    uint8_t* yRow = y + (size_t)row * width;
    uint8_t* uRow = u + (size_t)( row / 2 ) * chromaWidth;
    uint8_t* vRow = v + (size_t)( row / 2 ) * chromaWidth;
    fg.add_grain_line( yRow, uRow, vRow, row, width );
  }
}

bool planesEqual( const std::string& context, const std::vector<uint8_t>& a, const std::vector<uint8_t>& b )
{
  if( a == b )
  {
    return true;
  }
  std::cout << "failed: " << context << " (plane differs from pristine reference)\n";
  return false;
}

}   // namespace

bool test_FilmGrain()
{
  bool passed = true;

  const int width   = 256;
  const int height  = 16;
  const int cWidth  = width / 2;
  const int cHeight = height / 2;

  const std::vector<uint8_t> pristineY( (size_t)width * height, 128 );
  const std::vector<uint8_t> pristineU( (size_t)cWidth * cHeight, 128 );
  const std::vector<uint8_t> pristineV( (size_t)cWidth * cHeight, 128 );

  for( int component = 0; component < 3; component++ )
  {
    std::ostringstream sstm;
    sstm << "FilmGrain component=" << component;

    std::vector<uint8_t> y = pristineY, u = pristineU, v = pristineV;

    FilmGrain fg;
    vvdecSEIFilmGrainCharacteristics fgcEnabled;
    fillFilmGrainMessage( fgcEnabled, component );
    applyFilmGrain( fg, fgcEnabled, y.data(), u.data(), v.data(), width, height );

    const std::vector<uint8_t>& changedPlane    = component == 0 ? y : ( component == 1 ? u : v );
    const std::vector<uint8_t>& changedPristine = component == 0 ? pristineY : ( component == 1 ? pristineU : pristineV );
    if( changedPlane == changedPristine )
    {
      std::cout << "failed: " << sstm.str() << " (enabled) did not change any samples; fixture does not exercise synthesis\n";
      passed = false;
      continue;
    }

    // Reset to pristine, then apply a second SEI message on the SAME FilmGrain
    // instance that no longer signals this component as present.
    y = pristineY;
    u = pristineU;
    v = pristineV;

    vvdecSEIFilmGrainCharacteristics fgcDisabled;
    fillFilmGrainMessage( fgcDisabled, -1 );
    applyFilmGrain( fg, fgcDisabled, y.data(), u.data(), v.data(), width, height );

    passed = planesEqual( sstm.str() + " (disabled, luma)",     y, pristineY ) && passed;
    passed = planesEqual( sstm.str() + " (disabled, chroma U)", u, pristineU ) && passed;
    passed = planesEqual( sstm.str() + " (disabled, chroma V)", v, pristineV ) && passed;
  }

  return passed;
}
