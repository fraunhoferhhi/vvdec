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
#include "SEI_internal.h"
#include "vvdec/sei.h"
#include <stdlib.h>
#include "CommonDef.h"

namespace vvdec
{

const char *SEI_internal::getSEIMessageString( vvdecSEIPayloadType payloadType)
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

seiMessages SEI_internal::getSeisByType(const seiMessages &seiList, vvdecSEIPayloadType seiType)
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

seiMessages SEI_internal::extractSeisByType(seiMessages &seiList, vvdecSEIPayloadType seiType)
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
      if( sei->payloadType == VVDEC_SCALABLE_NESTING )
      {
        const vvdecSEIScalableNesting* nestingSei = ( vvdecSEIScalableNesting* ) sei->payload;

        if( !nestingSei->snSubpicFlag )
        {
          continue;
        }

        for( int i = 0; i < nestingSei->snNumSEIs; ++i )
        {
          auto& nestedSei = nestingSei->nestedSEIs[i];

          if( nestedSei->payload )
            free( nestedSei->payload );
          delete nestedSei;
        }
      }

      if( sei->payload )
          free( sei->payload );
      delete sei ;
    }
  }
  seiList.clear();
}

vvdecSEI* SEI_internal::allocSEI( vvdecSEIPayloadType payloadType )
{
  vvdecSEI* sei = new vvdecSEI;

  if( sei )
  {
    sei->payload     = NULL;
    sei->payloadType = (vvdecSEIPayloadType)payloadType;
    sei->size        = 0;
  }
  else
  {
    CHECK_FATAL( !sei, "sei memory allocation error" );
    return nullptr;
  }

  if( 0 != allocSEIPayload( sei ))
  {
    CHECK_FATAL( !sei, "sei payload allocation error" );
    delete sei ;
    return nullptr;
  }

  return sei;
}


int SEI_internal::allocSEIPayload( vvdecSEI* sei, int userDefSize )
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

int SEI_internal::getPayloadSize(vvdecSEIPayloadType payloadType)
{
  switch (payloadType)
  {
    case VVDEC_BUFFERING_PERIOD:                     return sizeof( vvdecSEIBufferingPeriod );
    case VVDEC_PICTURE_TIMING:                       return sizeof( vvdecSEIPictureTiming );
    case VVDEC_FILLER_PAYLOAD:                       return 0;
    case VVDEC_USER_DATA_REGISTERED_ITU_T_T35:       return sizeof( vvdecSEIUserDataRegistered );                 // not currently decoded
    case VVDEC_USER_DATA_UNREGISTERED:               return sizeof( vvdecSEIUserDataUnregistered );
    case VVDEC_FILM_GRAIN_CHARACTERISTICS:           return sizeof( vvdecSEIFilmGrainCharacteristics );
    case VVDEC_FRAME_PACKING:                        return sizeof( vvdecSEIFramePacking );
    case VVDEC_PARAMETER_SETS_INCLUSION_INDICATION:  return sizeof( vvdecSEIParameterSetsInclusionIndication );
    case VVDEC_DECODING_UNIT_INFO:                   return sizeof( vvdecSEIDecodingUnitInfo );
    case VVDEC_SCALABLE_NESTING:                     return sizeof( vvdecSEIScalableNesting );
    case VVDEC_DECODED_PICTURE_HASH:                 return sizeof( vvdecSEIDecodedPictureHash );
    case VVDEC_DEPENDENT_RAP_INDICATION:             return sizeof( vvdecSEIDependentRapIndication );
    case VVDEC_MASTERING_DISPLAY_COLOUR_VOLUME:      return sizeof( vvdecSEIMasteringDisplayColourVolume );
    case VVDEC_ALTERNATIVE_TRANSFER_CHARACTERISTICS: return sizeof( vvdecSEIAlternativeTransferCharacteristics );
    case VVDEC_CONTENT_LIGHT_LEVEL_INFO:             return sizeof( vvdecSEIContentLightLevelInfo );
    case VVDEC_AMBIENT_VIEWING_ENVIRONMENT:          return sizeof( vvdecSEIAmbientViewingEnvironment );
    case VVDEC_CONTENT_COLOUR_VOLUME:                return sizeof( vvdecSEIContentColourVolume );
    case VVDEC_EQUIRECTANGULAR_PROJECTION:           return sizeof( vvdecSEIEquirectangularProjection );
    case VVDEC_SPHERE_ROTATION:                      return sizeof( vvdecSEISphereRotation );
    case VVDEC_REGION_WISE_PACKING:                  return sizeof( vvdecSEIRegionWisePacking );
    case VVDEC_OMNI_VIEWPORT:                        return sizeof( vvdecSEIOmniViewport );
    case VVDEC_GENERALIZED_CUBEMAP_PROJECTION:       return sizeof( vvdecSEIGeneralizedCubemapProjection );
    case VVDEC_FRAME_FIELD_INFO:                     return sizeof( vvdecSEIFrameFieldInfo );
    case VVDEC_SAMPLE_ASPECT_RATIO_INFO:             return sizeof( vvdecSEISampleAspectRatioInfo );
    case VVDEC_SUBPICTURE_LEVEL_INFO:                return sizeof( vvdecSEISubpictureLevelInfo );
    default:                                         return -1;
  }

  return -1;
}

}
