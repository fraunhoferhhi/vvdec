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

#include "Unit.h"
#include "Picture.h"

#include <unordered_set>

namespace vvdec
{

struct Picture;
class Slice;
class SPS;
class PPS;
class VPS;
class ReferencePictureList;

class PicListManager
{
private:
  PicList                            m_cPicList;   //  Dynamic buffer
  int                                m_parseFrameDelay = -1;
  int                                m_parallelDecInst = 1;
  UserAllocator                      m_userAllocator;
  int                                m_tuneInDelay    = 0;
  bool                               m_firstOutputPic = true;
  std::unordered_set<const Picture*> m_allRefPics;

public:
  PicListManager() = default;
  ~PicListManager() { deleteBuffers(); }

  void create( int frameDelay, int decInstances, const UserAllocator& userAllocator );
  void restart()
  {
    m_firstOutputPic = true;
    m_tuneInDelay    = 0;
  }

  const Picture* getFrontPic() const { return m_cPicList.empty() ? nullptr : m_cPicList.front(); }
  const Picture* getBackPic() const  { return m_cPicList.empty() ? nullptr : m_cPicList.back(); }
  Picture*       getNewPicBuffer( const SPS& sps, const PPS& pps, const uint32_t temporalLayer, const int layerId, const VPS* vps );
  void           deleteBuffers();
  void           markUnusedPicturesReusable();
  Picture*       findClosestPic( int iLostPoc );
  Picture*       getNextOutputPic( uint32_t numReorderPicsHighestTid, uint32_t maxDecPicBufferingHighestTid, bool bFlush );
  void           releasePicture( Picture* pic );
};
}
