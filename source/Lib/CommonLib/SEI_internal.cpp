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
#include "SEI_internal.h"
#include "vvdec/sei.h"
#include <stdlib.h>
#include "CommonDef.h"


const char *SEI_internal::getSEIMessageString(SEIPayloadType payloadType)
{
  switch (payloadType)
  {
    case VVDEC_BUFFERING_PERIOD:                     return "Buffering period";
    case VVDEC_PICTURE_TIMING:                       return "Picture timing";
    case VVDEC_FILLER_PAYLOAD:                       return "Filler payload";                       // not currently decoded
    case VVDEC_USER_DATA_REGISTERED_ITU_T_T35:       return "User data registered";                 // not currently decoded
    case VVDEC_USER_DATA_UNREGISTERED:               return "User data unregistered";
    case VVDEC_FILM_GRAIN_CHARACTERISTICS:           return "Film grain characteristics";           // not currently decoded
    case VVDEC_FRAME_PACKING:                        return "Frame packing arrangement";
    case VVDEC_PARAMETER_SETS_INCLUSION_INDICATION:  return "Parameter sets inclusion indication";
    case VVDEC_DECODING_UNIT_INFO:                   return "Decoding unit information";
    case VVDEC_SCALABLE_NESTING:                     return "Scalable nesting";
    case VVDEC_DECODED_PICTURE_HASH:                 return "Decoded picture hash";
    case VVDEC_DEPENDENT_RAP_INDICATION:             return "Dependent RAP indication";
    case VVDEC_MASTERING_DISPLAY_COLOUR_VOLUME:      return "Mastering display colour volume";
    case VVDEC_ALTERNATIVE_TRANSFER_CHARACTERISTICS: return "Alternative transfer characteristics";
    case VVDEC_CONTENT_LIGHT_LEVEL_INFO:             return "Content light level information";
    case VVDEC_AMBIENT_VIEWING_ENVIRONMENT:          return "Ambient viewing environment";
    case VVDEC_CONTENT_COLOUR_VOLUME:                return "Content colour volume";
    case VVDEC_EQUIRECTANGULAR_PROJECTION:           return "Equirectangular projection";
    case VVDEC_SPHERE_ROTATION:                      return "Sphere rotation";
    case VVDEC_REGION_WISE_PACKING:                  return "Region wise packing information";
    case VVDEC_OMNI_VIEWPORT:                        return "Omni viewport";
    case VVDEC_GENERALIZED_CUBEMAP_PROJECTION:       return "Generalized cubemap projection";
    case VVDEC_FRAME_FIELD_INFO:                     return "Frame field info";
    case VVDEC_SAMPLE_ASPECT_RATIO_INFO:             return "Sample aspect ratio information";
    case VVDEC_SUBPICTURE_LEVEL_INFO:                return "Subpicture level information";
    default:                                        return "Unknown";
  }
}

seiMessages SEI_internal::getSeisByType(const seiMessages &seiList, SEIPayloadType seiType)
{
  seiMessages result;

  for( auto& s : seiList )
  {
    if ( s->payloadType == seiType)
    {
      result.push_back(s);
    }
  }
  return result;
}

seiMessages SEI_internal::extractSeisByType(seiMessages &seiList, SEIPayloadType seiType)
{
  seiMessages result;
  seiMessages::iterator it=seiList.begin();
  while ( it!=seiList.end() )
  {
    if ((*it)->payloadType == seiType)
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

void SEI_internal::deleteSEIs ( seiMessages &seiList)
{
  for( auto &sei : seiList )
  {
    if( sei )
    {
      if( sei->payload )
          free( sei->payload );
      delete sei ;
    }
  }
  seiList.clear();
}

vvdec_sei_message_t* SEI_internal::allocSEI( SEIPayloadType payloadType )
{
  vvdec_sei_message_t* sei = new vvdec_sei_message_t;

  if( sei )
  {
    sei->payload     = NULL;
    sei->payloadType = (SEIPayloadType)payloadType;
    sei->size        = 0;
  }
  else
  {
    CHECK( !sei, "sei memory allocation error" );
    return nullptr;
  }

  if( 0 != allocSEIPayload( sei ))
  {
    CHECK( !sei, "sei payload allocation error" );
    delete sei ;
    return nullptr;
  }

  return sei;
}


int SEI_internal::allocSEIPayload( vvdec_sei_message_t* sei, int userDefSize )
{
  if( NULL == sei ){ return -1; }
  int size = userDefSize>0 ? userDefSize : getPayloadSize( sei->payloadType );
  if( size <= 0 ){ return -1;}

  sei->payload = malloc( size );
  if( sei->payload )
  {
    sei->size = size;
    memset(  sei->payload, 0, size );
  }

  return 0;
}

int SEI_internal::getPayloadSize(SEIPayloadType payloadType)
{
  switch (payloadType)
  {
    case VVDEC_BUFFERING_PERIOD:                     return sizeof( vvdec_sei_buffering_period_t );
    case VVDEC_PICTURE_TIMING:                       return sizeof( vvdec_sei_picture_timing_t );
    case VVDEC_FILLER_PAYLOAD:                       return 0;
    case VVDEC_USER_DATA_REGISTERED_ITU_T_T35:       return sizeof( vvdec_sei_user_data_registered_t );                 // not currently decoded
    case VVDEC_USER_DATA_UNREGISTERED:               return sizeof( vvdec_sei_user_data_unregistered_t );
    case VVDEC_FILM_GRAIN_CHARACTERISTICS:           return sizeof( vvdec_sei_film_grain_characteristics_t );
    case VVDEC_FRAME_PACKING:                        return sizeof( vvdec_sei_frame_packing_t );
    case VVDEC_PARAMETER_SETS_INCLUSION_INDICATION:  return sizeof( vvdec_sei_parameter_sets_inclusion_indication_t );
    case VVDEC_DECODING_UNIT_INFO:                   return sizeof( vvdec_sei_decoding_unit_info_t );
    case VVDEC_SCALABLE_NESTING:                     return sizeof( vvdec_sei_scalable_nesting_t );
    case VVDEC_DECODED_PICTURE_HASH:                 return sizeof( vvdec_sei_decoded_picture_hash_t );
    case VVDEC_DEPENDENT_RAP_INDICATION:             return sizeof( vvdec_sei_dependent_rap_indication_t );
    case VVDEC_MASTERING_DISPLAY_COLOUR_VOLUME:      return sizeof( vvdec_sei_mastering_display_colour_volume_t );
    case VVDEC_ALTERNATIVE_TRANSFER_CHARACTERISTICS: return sizeof( vvdec_sei_alternative_transfer_characteristics_t );
    case VVDEC_CONTENT_LIGHT_LEVEL_INFO:             return sizeof( vvdec_sei_content_light_level_info_t );
    case VVDEC_AMBIENT_VIEWING_ENVIRONMENT:          return sizeof( vvdec_sei_ambient_viewing_environment_t );
    case VVDEC_CONTENT_COLOUR_VOLUME:                return sizeof( vvdec_sei_ambient_viewing_environment_t );
    case VVDEC_EQUIRECTANGULAR_PROJECTION:           return sizeof( vvdec_sei_equirectangular_projection_t );
    case VVDEC_SPHERE_ROTATION:                      return sizeof( vvdec_sei_sphere_rotation_t );
    case VVDEC_REGION_WISE_PACKING:                  return sizeof( vvdec_sei_region_wise_packing_t );
    case VVDEC_OMNI_VIEWPORT:                        return sizeof( vvdec_sei_omni_viewport_t );
    case VVDEC_GENERALIZED_CUBEMAP_PROJECTION:       return sizeof( vvdec_sei_generalized_cubemap_projection_t );
    case VVDEC_FRAME_FIELD_INFO:                     return sizeof( vvdec_sei_frame_field_info_t );
    case VVDEC_SAMPLE_ASPECT_RATIO_INFO:             return sizeof( vvdec_sei_sample_aspect_ratio_info_t );
    case VVDEC_SUBPICTURE_LEVEL_INFO:                return sizeof( vvdec_sei_subpicture_level_info_t );
    default:                                         return -1;
  }

  return -1;
}



