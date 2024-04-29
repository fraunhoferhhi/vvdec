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

//======================================================================================================================
//Chroma format utility functions  =====================================================================================
//======================================================================================================================

namespace vvdec
{

static inline ChannelType toChannelType             (const ComponentID id)                         { return (id==COMPONENT_Y)? CHANNEL_TYPE_LUMA : CHANNEL_TYPE_CHROMA; }
static inline bool        isLuma                    (const ComponentID id)                         { return (id==COMPONENT_Y);                                          }
static inline bool        isLuma                    (const ChannelType id)                         { return (id==CHANNEL_TYPE_LUMA);                                    }
static inline bool        isChroma                  (const ComponentID id)                         { return (id!=COMPONENT_Y);                                          }
static inline bool        isChroma                  (const ChannelType id)                         { return (id!=CHANNEL_TYPE_LUMA);                                    }
static inline uint32_t    getChannelTypeScaleX      (const ChannelType id, const ChromaFormat fmt) { return (isLuma(id) || (fmt==CHROMA_444)) ? 0 : 1;                  }
static inline uint32_t    getChannelTypeScaleY      (const ChannelType id, const ChromaFormat fmt) { return (isLuma(id) || (fmt!=CHROMA_420)) ? 0 : 1;                  }
static inline uint32_t    getComponentScaleX        (const ComponentID id, const ChromaFormat fmt) { return getChannelTypeScaleX(toChannelType(id), fmt);               }
static inline uint32_t    getComponentScaleY        (const ComponentID id, const ChromaFormat fmt) { return getChannelTypeScaleY(toChannelType(id), fmt);               }
static inline uint32_t    getNumberValidComponents  (const ChromaFormat fmt)                       { return (fmt==CHROMA_400) ? 1 : MAX_NUM_COMPONENT;                  }
static inline uint32_t    getNumberValidChannels    (const ChromaFormat fmt)                       { return (fmt==CHROMA_400) ? 1 : MAX_NUM_CHANNEL_TYPE;               }
static inline bool        isChromaEnabled           (const ChromaFormat fmt)                       { return !(fmt==CHROMA_400);                                         }
static inline ComponentID getFirstComponentOfChannel(const ChannelType id)                         { return (isLuma(id) ? COMPONENT_Y : COMPONENT_Cb);                  }

}
