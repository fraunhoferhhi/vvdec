/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

For any license concerning other Intellectual Property rights than the software, 
especially patent licenses, a separate Agreement needs to be closed. 
For more information please contact:

Fraunhofer Heinrich Hertz Institute
Einsteinufer 37
10587 Berlin, Germany
www.hhi.fraunhofer.de/vvc
vvc@hhi.fraunhofer.de

Copyright (c) 2018-2021, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. 
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of Fraunhofer nor the names of its contributors may
   be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
THE POSSIBILITY OF SUCH DAMAGE.


------------------------------------------------------------------------------------------- */

#ifndef PICLISTMANAGER_H
#define PICLISTMANAGER_H

#include "CommonDef.h"

#include <list>
#include <tuple>

struct Picture;
class SPS;
class PPS;
#if JVET_Q0814_DPB
class VPS;
#endif
class ReferencePictureList;

typedef std::list<Picture*> PicList;
#if 0
typedef const std::list<Picture*> PicListRange;
#else
typedef std::tuple<PicList::const_iterator, PicList::const_iterator> PicListRange;

static const PicList::const_iterator begin( const PicListRange & t ) { return std::get<0>( t ); }
static const PicList::const_iterator end  ( const PicListRange & t ) { return std::get<1>( t ); }
#endif

class PicListManager
{
private:
  PicList m_cPicList;   //  Dynamic buffer
  int     m_parseFrameDelay = -1;
  int     m_parallelDecInst = 1;
  int     m_tuneInDelay     = 0;
  bool    m_firstOutputPic  = true;

public:
  PicListManager() = default;
  ~PicListManager() { deleteBuffers(); }

  void create( int frameDelay, int decInstances );
  void restart()
  {
    m_firstOutputPic = true;
    m_tuneInDelay    = 0;
  }

  PicListRange   getPicListRange( const Picture* pic ) const;
  const Picture* getFrontPic() const { return m_cPicList.empty() ? nullptr : m_cPicList.front(); }
  const Picture* getBackPic() const  { return m_cPicList.empty() ? nullptr : m_cPicList.back(); }
#if JVET_Q0814_DPB
  Picture*       getNewPicBuffer( const SPS& sps,const PPS& pps, const uint32_t temporalLayer, const int layerId, const VPS* vps );
#else
  Picture*       getNewPicBuffer( const SPS& sps,const PPS& pps, const uint32_t temporalLayer, const int layerId );
#endif
  void           deleteBuffers();
  void           applyReferencePictureListBasedMarking( const Picture * currPic, const ReferencePictureList * rpl0, const ReferencePictureList * rpl1 );
  void           applyDoneReferencePictureMarking();
  Picture*       findClosestPic( int iLostPoc );
  Picture*       getNextOutputPic( uint32_t numReorderPicsHighestTid, uint32_t maxDecPicBufferingHighestTid, bool bFlush );
  void           releasePicture( Picture* pic );
};

#endif   // PICLISTMANAGER_H
