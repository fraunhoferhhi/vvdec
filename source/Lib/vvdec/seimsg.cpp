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

Copyright (c) 2018-2020, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. 
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
#include "vvdec/seimsg.h"

namespace vvdec {

const char *seimsg::getSEIMsgString(seimsg::PayloadType payloadType)
{
  switch (payloadType)
  {
    case seimsg::BUFFERING_PERIOD:                     return "Buffering period";
    case seimsg::PICTURE_TIMING:                       return "Picture timing";
    case seimsg::FILLER_PAYLOAD:                       return "Filler payload";                       // not currently decoded
    case seimsg::USER_DATA_REGISTERED_ITU_T_T35:       return "User data registered";                 // not currently decoded
    case seimsg::USER_DATA_UNREGISTERED:               return "User data unregistered";
    case seimsg::FILM_GRAIN_CHARACTERISTICS:           return "Film grain characteristics";           // not currently decoded
    case seimsg::FRAME_PACKING:                        return "Frame packing arrangement";
    case seimsg::PARAMETER_SETS_INCLUSION_INDICATION:  return "Parameter sets inclusion indication";
    case seimsg::DECODING_UNIT_INFO:                   return "Decoding unit information";
    case seimsg::SCALABLE_NESTING:                     return "Scalable nesting";
    case seimsg::DECODED_PICTURE_HASH:                 return "Decoded picture hash";
    case seimsg::DEPENDENT_RAP_INDICATION:             return "Dependent RAP indication";
    case seimsg::MASTERING_DISPLAY_COLOUR_VOLUME:      return "Mastering display colour volume";
    case seimsg::ALTERNATIVE_TRANSFER_CHARACTERISTICS: return "Alternative transfer characteristics";
    case seimsg::CONTENT_LIGHT_LEVEL_INFO:             return "Content light level information";
    case seimsg::AMBIENT_VIEWING_ENVIRONMENT:          return "Ambient viewing environment";
    case seimsg::CONTENT_COLOUR_VOLUME:                return "Content colour volume";
    case seimsg::EQUIRECTANGULAR_PROJECTION:           return "Equirectangular projection";
    case seimsg::SPHERE_ROTATION:                      return "Sphere rotation";
    case seimsg::REGION_WISE_PACKING:                  return "Region wise packing information";
    case seimsg::OMNI_VIEWPORT:                        return "Omni viewport";
    case seimsg::GENERALIZED_CUBEMAP_PROJECTION:       return "Generalized cubemap projection";
    case seimsg::SAMPLE_ASPECT_RATIO_INFO:             return "Sample aspect ratio information";
    case seimsg::SUBPICTURE_LEVEL_INFO:                return "Subpicture level information";
    default:                                        return "Unknown";
  }
}

std::list<seimsg*> seimsg::getSeisByType(const std::list<seimsg*> &seiList, seimsg::PayloadType seiType)
{
  std::list<seimsg*> result;

  for (std::list<seimsg*>::const_iterator it=seiList.begin(); it!=seiList.end(); it++)
  {
    if ((*it)->payloadType() == seiType)
    {
      result.push_back(*it);
    }
  }
  return result;
}

std::list<seimsg*> seimsg::extractSeisByType(std::list<seimsg*> &seiList, seimsg::PayloadType seiType)
{
  std::list<seimsg*> result;
  std::list<seimsg*>::iterator it=seiList.begin();
  while ( it!=seiList.end() )
  {
    if ((*it)->payloadType() == seiType)
    {
      result.push_back(*it);
      it = seiList.erase(it);
    }
    else
    {
      it++;
    }
  }
  return result;
}


void seimsg::deleteSEIs (std::list<seimsg*> &seiList)
{
  for (std::list<seimsg*>::iterator it=seiList.begin(); it!=seiList.end(); it++)
  {
    delete (*it);
  }
  seiList.clear();
}

} // namespace

