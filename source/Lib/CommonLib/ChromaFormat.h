/* -----------------------------------------------------------------------------
Software Copyright License for the Fraunhofer Software Library VVdec

(c) Copyright (2018-2020) Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. 

1.    INTRODUCTION

The Fraunhofer Software Library VVdec (“Fraunhofer Versatile Video Decoding Library”) is software that implements (parts of) the Versatile Video Coding Standard - ITU-T H.266 | MPEG-I - Part 3 (ISO/IEC 23090-3) and related technology. 
The standard contains Fraunhofer patents as well as third-party patents. Patent licenses from third party standard patent right holders may be required for using the Fraunhofer Versatile Video Decoding Library. It is in your responsibility to obtain those if necessary. 

The Fraunhofer Versatile Video Decoding Library which mean any source code provided by Fraunhofer are made available under this software copyright license. 
It is based on the official ITU/ISO/IEC VVC Test Model (VTM) reference software whose copyright holders are indicated in the copyright notices of its source files. The VVC Test Model (VTM) reference software is licensed under the 3-Clause BSD License and therefore not subject of this software copyright license.

2.    COPYRIGHT LICENSE

Internal use of the Fraunhofer Versatile Video Decoding Library, in source and binary forms, with or without modification, is permitted without payment of copyright license fees for non-commercial purposes of evaluation, testing and academic research. 

No right or license, express or implied, is granted to any part of the Fraunhofer Versatile Video Decoding Library except and solely to the extent as expressly set forth herein. Any commercial use or exploitation of the Fraunhofer Versatile Video Decoding Library and/or any modifications thereto under this license are prohibited.

For any other use of the Fraunhofer Versatile Video Decoding Library than permitted by this software copyright license You need another license from Fraunhofer. In such case please contact Fraunhofer under the CONTACT INFORMATION below.

3.    LIMITED PATENT LICENSE

As mentioned under 1. Fraunhofer patents are implemented by the Fraunhofer Versatile Video Decoding Library. If You use the Fraunhofer Versatile Video Decoding Library in Germany, the use of those Fraunhofer patents for purposes of testing, evaluating and research and development is permitted within the statutory limitations of German patent law. However, if You use the Fraunhofer Versatile Video Decoding Library in a country where the use for research and development purposes is not permitted without a license, you must obtain an appropriate license from Fraunhofer. It is Your responsibility to check the legal requirements for any use of applicable patents.    

Fraunhofer provides no warranty of patent non-infringement with respect to the Fraunhofer Versatile Video Decoding Library.


4.    DISCLAIMER

The Fraunhofer Versatile Video Decoding Library is provided by Fraunhofer "AS IS" and WITHOUT ANY EXPRESS OR IMPLIED WARRANTIES, including but not limited to the implied warranties fitness for a particular purpose. IN NO EVENT SHALL FRAUNHOFER BE LIABLE for any direct, indirect, incidental, special, exemplary, or consequential damages, including but not limited to procurement of substitute goods or services; loss of use, data, or profits, or business interruption, however caused and on any theory of liability, whether in contract, strict liability, or tort (including negligence), arising in any way out of the use of the Fraunhofer Versatile Video Decoding Library, even if advised of the possibility of such damage.

5.    CONTACT INFORMATION

Fraunhofer Heinrich Hertz Institute
Attention: Video Coding & Analytics Department
Einsteinufer 37
10587 Berlin, Germany
www.hhi.fraunhofer.de/vvc
vvc@hhi.fraunhofer.de
------------------------------------------------------------------------------------------- */

#ifndef __CHROMAFORMAT__
#define __CHROMAFORMAT__

#include "Common.h"
#include "CommonDef.h"
#include "Rom.h"

#include <iostream>
#include <vector>

//======================================================================================================================
//Chroma format utility functions  =====================================================================================
//======================================================================================================================


static inline ChannelType toChannelType             (const ComponentID id)                         { return (id==COMPONENT_Y)? CHANNEL_TYPE_LUMA : CHANNEL_TYPE_CHROMA; }
static inline bool        isLuma                    (const ComponentID id)                         { return (id==COMPONENT_Y);                                          }
static inline bool        isLuma                    (const ChannelType id)                         { return (id==CHANNEL_TYPE_LUMA);                                    }
static inline bool        isChroma                  (const ComponentID id)                         { return (id!=COMPONENT_Y);                                          }
static inline bool        isChroma                  (const ChannelType id)                         { return (id!=CHANNEL_TYPE_LUMA);                                    }
static inline uint32_t        getChannelTypeScaleX      (const ChannelType id, const ChromaFormat fmt) { return (isLuma(id) || (fmt==CHROMA_444)) ? 0 : 1;                  }
static inline uint32_t        getChannelTypeScaleY      (const ChannelType id, const ChromaFormat fmt) { return (isLuma(id) || (fmt!=CHROMA_420)) ? 0 : 1;                  }
static inline uint32_t        getComponentScaleX        (const ComponentID id, const ChromaFormat fmt) { return getChannelTypeScaleX(toChannelType(id), fmt);               }
static inline uint32_t        getComponentScaleY        (const ComponentID id, const ChromaFormat fmt) { return getChannelTypeScaleY(toChannelType(id), fmt);               }
static inline uint32_t        getNumberValidComponents  (const ChromaFormat fmt)                       { return (fmt==CHROMA_400) ? 1 : MAX_NUM_COMPONENT;                  }
static inline uint32_t        getNumberValidChannels    (const ChromaFormat fmt)                       { return (fmt==CHROMA_400) ? 1 : MAX_NUM_CHANNEL_TYPE;               }
static inline bool        isChromaEnabled           (const ChromaFormat fmt)                       { return !(fmt==CHROMA_400);                                         }
static inline ComponentID getFirstComponentOfChannel(const ChannelType id)                         { return (isLuma(id) ? COMPONENT_Y : COMPONENT_Cb);                  }

InputColourSpaceConversion stringToInputColourSpaceConvert(const std::string &value, const bool bIsForward);
std::string getListOfColourSpaceConverts(const bool bIsForward);

//------------------------------------------------

static inline uint32_t getTotalSamples(const uint32_t width, const uint32_t height, const ChromaFormat format)
{
  const uint32_t samplesPerChannel = width * height;

  switch (format)
  {
    case CHROMA_400: return  samplesPerChannel;           break;
    case CHROMA_420: return (samplesPerChannel * 3) >> 1; break;
    case CHROMA_422: return  samplesPerChannel * 2;       break;
    case CHROMA_444: return  samplesPerChannel * 3;       break;
    default:
      EXIT( "ERROR: Unrecognised chroma format in getTotalSamples() " );
      break;
  }

  return MAX_UINT;
}

//------------------------------------------------

static inline uint64_t getTotalFracBits(const uint32_t width, const uint32_t height, const ChromaFormat format, const int bitDepths[MAX_NUM_CHANNEL_TYPE])
{
  unsigned bitsPerSampleTimes2 = MAX_UINT;
  switch (format)
  {
  case CHROMA_400: bitsPerSampleTimes2 =   2 *  bitDepths[CHANNEL_TYPE_LUMA];                                              break;
  case CHROMA_420: bitsPerSampleTimes2 = ( 2 * (bitDepths[CHANNEL_TYPE_LUMA]*2 +   bitDepths[CHANNEL_TYPE_CHROMA]) ) >> 1; break;
  case CHROMA_422: bitsPerSampleTimes2 =   2 * (bitDepths[CHANNEL_TYPE_LUMA]   +   bitDepths[CHANNEL_TYPE_CHROMA]);        break;
  case CHROMA_444: bitsPerSampleTimes2 =   2 * (bitDepths[CHANNEL_TYPE_LUMA]   + 2*bitDepths[CHANNEL_TYPE_CHROMA]);        break;
  default:
      EXIT( "ERROR: Unrecognised chroma format in getTotalFracBits() " );
    break;
  }
  return uint64_t( width * height * bitsPerSampleTimes2 ) << ( SCALE_BITS - 1 );
}


//======================================================================================================================
//Intra prediction  ====================================================================================================
//======================================================================================================================

static inline bool filterIntraReferenceSamples (const ChannelType chType, const ChromaFormat chFmt, const bool intraReferenceSmoothingDisabled)
{
  return (!intraReferenceSmoothingDisabled) && (isLuma(chType) || (chFmt == CHROMA_444));
}


//------------------------------------------------

static inline int getTransformShift(const int channelBitDepth, const Size size, const int maxLog2TrDynamicRange)
{
  return maxLog2TrDynamicRange - channelBitDepth - ( ( getLog2(size.width) + getLog2(size.height) ) >> 1 );
}


//------------------------------------------------


//======================================================================================================================
//Scaling lists  =======================================================================================================
//======================================================================================================================

static inline int getScalingListType(const PredMode predMode, const ComponentID compID)
{
  return ( ( predMode == MODE_INTRA ) ? 0 : MAX_NUM_COMPONENT ) + MAP_CHROMA( compID );
}

#endif
