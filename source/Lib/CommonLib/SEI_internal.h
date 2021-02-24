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

#pragma once

#include <list>
#include <vector>
#include <cstdint>
#include <cstring>

#include "vvdec/sei.h"

typedef std::list<vvdec_sei_message*> seiMessages;

class SEI_internal
{
public:

  SEI_internal() {}
  virtual ~SEI_internal() {}

  static const char *getSEIMessageString( SEIPayloadType payloadType);

  /// output a selection of SEI messages by payload type. Ownership stays in original message list.
  static seiMessages getSeisByType(const seiMessages &seiList, SEIPayloadType seiType);

  /// remove a selection of SEI messages by payload type from the original list and return them in a new list.
  static seiMessages extractSeisByType(seiMessages &seiList, SEIPayloadType seiType);

  /// delete list of SEI messages (freeing the referenced objects)
  static void deleteSEIs (seiMessages &seiList);

  static vvdec_sei_message_t* allocSEI( SEIPayloadType payloadType );
  static int allocSEIPayload( vvdec_sei_message_t* sei, int userDefSize = -1 );
  static int getPayloadSize(SEIPayloadType payloadType);
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

  bool equal( vvdec_sei_decoded_picture_hash_t digest ) const
  {
    if ((size_t)digest.digist_length != hash.size())
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

