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

#include <list>
#include <vector>
#include <cstdint>
#include <cstring>

#include "vvdec/sei.h"

namespace vvdec
{

typedef std::list<vvdecSEI*> seiMessages;

class SEI_internal
{
public:

  SEI_internal() {}
  virtual ~SEI_internal() {}

  static const char *getSEIMessageString( vvdecSEIPayloadType payloadType);

  /// output a selection of SEI messages by payload type. Ownership stays in original message list.
  static seiMessages getSeisByType(const seiMessages &seiList, vvdecSEIPayloadType seiType);

  /// remove a selection of SEI messages by payload type from the original list and return them in a new list.
  static seiMessages extractSeisByType(seiMessages &seiList, vvdecSEIPayloadType seiType);

  /// delete list of SEI messages (freeing the referenced objects)
  static void deleteSEIs (seiMessages &seiList);

  static vvdecSEI* allocSEI ( vvdecSEIPayloadType payloadType );
  static int allocSEIPayload( vvdecSEI* sei, int userDefSize = -1 );
  static int getPayloadSize ( vvdecSEIPayloadType payloadType);
};



struct PictureHash
{
  std::vector<uint8_t> hash = {};

  bool operator==(const PictureHash &other) const
  {
    if (other.hash.size() != hash.size())
    {
      return false;
    }
    for(uint32_t i=0; i<uint32_t(hash.size()); i++)
    {
      if (other.hash[i] != hash[i])
      {
        return false;
      }
    }
    return true;
  }

  bool operator!=(const PictureHash &other) const
  {
    return !(*this == other);
  }

  bool equal( vvdecSEIDecodedPictureHash digest ) const
  {
    if ((size_t)digest.digest_length != hash.size())
    {
      return false;
    }
    for(uint32_t i=0; i<uint32_t(hash.size()); i++)
    {
      if (digest.digest[i] != hash[i])
      {
        return false;
      }
    }
    return true;
  }
};

}
