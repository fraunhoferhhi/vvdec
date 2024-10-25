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

#include <assert.h>
#include <stdint.h>
#include <vector>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <ctime>
#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>

#include "vvdec/vvdec.h"

#include "MD5.h"

#define MAX_CODED_PICTURE_SIZE  800000

static int _writeComponentToFile( std::ostream *f, vvdecPlane *plane, vvdecPlane *planeField2, uint32_t uiBytesPerSample )
{
  uint32_t uiWidth  = plane->width;
  uint32_t uiHeight = plane->height;

  assert( f != NULL );

  if( plane->bytesPerSample == 2 )
  {
     unsigned short* p  = reinterpret_cast<unsigned short*>( plane->ptr );
     unsigned short* p2  = planeField2 ? reinterpret_cast<unsigned short*>( planeField2->ptr ) : nullptr;

     if( uiBytesPerSample == 1 )  // cut to 8bit output
     {
       // 8bit > 16bit conversion
       std::vector<unsigned char> tmp;
       std::vector<unsigned char> tmp2;
       tmp.resize(uiWidth);
       if( planeField2 )
       {
         tmp2.resize(uiWidth);
       }

       for( uint32_t y = 0; y < uiHeight; y++ )
       {
         for( uint32_t x = 0; x < uiWidth; x++ )
         {
           tmp[x] = (unsigned char)(p[x]>>2);
         }
         f->write( (char*)&tmp[0], sizeof(std::vector<unsigned char>::value_type)*tmp.size());
         p += plane->stride;

         if( p2 )
         {
           for( uint32_t x = 0; x < uiWidth; x++ )
           {
             tmp2[x] = (unsigned char)(p2[x]>>2);
           }
           f->write( (char*)&tmp2[0], sizeof(std::vector<unsigned char>::value_type)*tmp2.size());
           p2 += planeField2->stride;
         }
       }
     }
     else
     {
       uint8_t *p = plane->ptr;
       uint8_t *p2 = planeField2 ? planeField2->ptr : nullptr;

       for( uint32_t y = 0; y < uiHeight; y++ )
       {
         f->write( (char*)p, uiWidth*uiBytesPerSample );
         p  += plane->stride;

         if( p2 )
         {
           f->write( (char*)p2, uiWidth*uiBytesPerSample );
           p2 += planeField2->stride;
         }
       }
     }
  }
  else
  {
   uint8_t *p = plane->ptr;
   uint8_t *p2 = planeField2 ? planeField2->ptr : nullptr;

   if( uiBytesPerSample == 2 )
   {
     // 8bit > 16bit conversion
     std::vector<short> tmp;
     std::vector<short> tmp2;
     tmp.resize(uiWidth);

     if( p2 )
     {
       tmp2.resize(uiWidth);
     }

     for( uint32_t y = 0; y < uiHeight; y++ )
     {
       for( uint32_t x = 0; x < uiWidth; x++ )
       {
         tmp[x] = p[x];
       }
       f->write( (char*)&tmp[0], sizeof(std::vector<short>::value_type)*tmp.size());
       p += plane->stride;

       if( p2 )
       {
         for( uint32_t x = 0; x < uiWidth; x++ )
         {
           tmp2[x] = p2[x];
         }
         f->write( (char*)&tmp2[0], sizeof(std::vector<short>::value_type)*tmp2.size());
         p2 += planeField2->stride;
       }
     }
   }
   else
   {
     for( uint32_t y = 0; y < uiHeight; y++ )
     {
       f->write( (char*)p, uiBytesPerSample*uiWidth );
       p += plane->stride;

       if( p2 )
       {
         f->write( (char*)p2, uiBytesPerSample*uiWidth );
         p2 += planeField2->stride;
       }
     }
   }
  }
  return 0;
}


/**
 * \brief Retrieving of NAL unit start code
 */
static inline int retrieveNalStartCode( unsigned char *pB, int iZerosInStartcode )
{
  int info = 1;
  int i=0;
  for ( i = 0; i < iZerosInStartcode; i++)
  {
    if( pB[i] != 0 )
    {
      info = 0;
    }
  }

  if( pB[i] != 1 )
  {
    info = 0;
  }

  return info;
}

/**
 * \brief Reading of one Annex B NAL unit from file stream
 */
static int readBitstreamFromFile( std::ifstream *f, vvdecAccessUnit* pcAccessUnit, bool bLoop )
{
  int info2=0;
  int info3=0;
  int pos = 0;

  int iStartCodeFound =0;
  int iRewind=0;
  uint32_t len;
  unsigned char* pBuf = pcAccessUnit->payload;
  pcAccessUnit->payloadUsedSize = 0;

  auto curfilpos = f->tellg();
  if( curfilpos < 0 )
  {
    if( bLoop )
    {
      f->clear();
      f->seekg( 0, f->beg );
      if(f->bad() || f->fail())
      {
        return -1;
      }
    }
    else
    {
      return -1;
    }
  }

  //jump over possible start code
  f->read( (char*)pBuf, 5 );
  size_t extracted = f->gcount();
  if( extracted < 4 )
  {
    if( bLoop )
    {
      f->clear();
      f->seekg( 0, f->beg );
      if(f->bad() || f->fail())
      {
        return -1;
      }

      f->read( (char*)pBuf, 5 );
      size_t extracted = f->gcount();
      if( extracted < 4 )
      {
        return -1;
      }
    }
    else
    {
      return -1;
    }
  }

  pos +=5;
  info2 = 0;
  info3 = 0;
  iStartCodeFound = 0;

  while( !iStartCodeFound )
  {
    if( f->eof() )
    {
      if( pos > 5 )
      {
        len = pos - 1;
        pcAccessUnit->payloadUsedSize=len;
        return len;
      }
      else if( bLoop )
      {
        f->clear();
        f->seekg( 0, f->beg );
      }
      else
      {
        return -1;
      }
    }

    if( pos >= pcAccessUnit->payloadSize )
    {
      int iNewSize = pcAccessUnit->payloadSize*2;
      unsigned char* newbuf = (unsigned char*) malloc( sizeof( unsigned char ) * iNewSize );
      if( newbuf == NULL )
      {
        fprintf( stderr, "ERR: readBitstreamFromFile: memory re-allocation failed!\n" );
        return -1;
      }
      std::copy_n( pcAccessUnit->payload, std::min( pcAccessUnit->payloadSize , iNewSize), newbuf);
      pcAccessUnit->payloadSize = iNewSize;
      free( pcAccessUnit->payload );

      pcAccessUnit->payload = newbuf;
      pBuf = pcAccessUnit->payload;
    }
    unsigned char* p= pBuf + pos;
    f->read( (char*)p, 1 );
    pos++;

    info3 = retrieveNalStartCode(&pBuf[pos-4], 3);
    if(info3 != 1)
    {
      info2 = retrieveNalStartCode(&pBuf[pos-3], 2);
    }
    iStartCodeFound = (info2 == 1 || info3 == 1);
  }


  // Here, we have found another start code (and read length of startcode bytes more than we should
  // have.  Hence, go back in the file
  iRewind = 0;
  if(info3 == 1)
    iRewind = -4;
  else if (info2 == 1)
    iRewind = -3;
  else
    fprintf( stderr, "ERR: readBitstreamFromFile: Error in next start code search \n");

  f->seekg ( iRewind, f->cur);
  if(f->bad() || f->fail())
  {
    fprintf( stderr, "ERR: readBitstreamFromFile: Cannot seek %d in the bit stream file", iRewind );
    return -1;
  }

  // Here the Start code, the complete NALU, and the next start code is in the pBuf.
  // The size of pBuf is pos, pos+rewind are the number of bytes excluding the next
  // start code, and (pos+rewind)-startcodeprefix_len is the size of the NALU

  len = (pos+iRewind);
  pcAccessUnit->payloadUsedSize=len;
  return len;
}

/**
  This method writes the y4m header in front of every frame
*/
static int writeY4MHeader( std::ostream *f, vvdecFrame *frame )
{
  std::stringstream cssHeader;

  if ( f == NULL || frame == NULL )
  {
    return -1;
  }

  if ( frame->sequenceNumber == 0 )
  {
    const char *cf = (frame->colorFormat == VVDEC_CF_YUV444_PLANAR) ? "444"
                   : (frame->colorFormat == VVDEC_CF_YUV422_PLANAR) ? "422" : "420";

    const char *bdepth = (frame->bitDepth == 10) ? "p10" : "";

    int frameRate=50;
    int frameScale=1;
    if( frame->picAttributes && frame->picAttributes->hrd )
    {
      if( frame->picAttributes->hrd->numUnitsInTick && frame->picAttributes->hrd->timeScale )
      {
        frameRate  = frame->picAttributes->hrd->timeScale;
        frameScale = frame->picAttributes->hrd->numUnitsInTick;

        if( frame->picAttributes->olsHrd )
        {
          // if the OLS HRD is present and indicates a non-constant frame-rate, producing a Y4M is no possible
          //if( !frame->picAttributes->olsHrd->fixedPicRateGeneralFlag && !frame->picAttributes->olsHrd->fixedPicRateWithinCvsFlag ) return -1;
          frameScale *= frame->picAttributes->olsHrd->elementDurationInTc;
        }
      }
    }

    const char *interlacedMode = (frame->frameFormat == VVDEC_FF_TOP_FIELD || frame->frameFormat == VVDEC_FF_BOT_FIELD) ? " Im" : " Ip";
    cssHeader << "YUV4MPEG2 W" << frame->width << " H" << frame->height << " F" << frameRate << ":" << frameScale << interlacedMode << " C" << cf << bdepth << "\n";
  }

  cssHeader << "FRAME\n";

  f->write( cssHeader.str().c_str(),cssHeader.str().length() );

  return 0;
}

/**
  This method writes and yuv planar picture into a file sink.
  \param[in]  *f file sink pointer
  \param[in]  *frame decoded picture pointer
  \retval     int  if non-zero an error occurred (see ErrorCodes), otherwise the return value indicates success VVC_DEC_OK
  \pre        The decoder must not be initialized.
*/
static int writeYUVToFile( std::ostream *f, vvdecFrame *frame, bool y4mFormat )
{
  uint32_t c = 0;

  uint32_t uiBytesPerSample = 1;

  assert( f != NULL );

  if( y4mFormat )
  {
    writeY4MHeader( f, frame );
  }

  for( c = 0; c < frame->numPlanes; c++ )
  {
    uiBytesPerSample = std::max( (uint32_t) frame->planes[c].bytesPerSample, uiBytesPerSample );
  }

  for( c = 0; c < frame->numPlanes; c++ )
  {
    int ret;
    if( ( ret = _writeComponentToFile( f, &frame->planes[c], nullptr, uiBytesPerSample ) ) != 0 )
    {
      return ret;
    }
  }

  return 0;
}

 /**
   This method writes and yuv planar picture into a file sink.
   \param[in]  *f file sink pointer
   \param[in]  *frame decoded picture pointer top field
   \param[in]  *frame decoded picture pointer bottom field
   \retval     int  if non-zero an error occurred (see ErrorCodes), otherwise the return value indicates success VVC_DEC_OK
   \pre        The decoder must not be initialized.
 */
static int writeYUVToFileInterlaced( std::ostream *f, vvdecFrame *topField, vvdecFrame *botField, bool y4mFormat )
{
  int ret;
  uint32_t c = 0;

  uint32_t uiBytesPerSample = 1;

  assert( f != NULL );
  assert( topField != NULL );
  assert( botField != NULL );
  assert( topField->numPlanes == botField->numPlanes );

  if ( y4mFormat )
  {
    writeY4MHeader( f, topField );
  }

  for( c = 0; c < topField->numPlanes; c++ )
  {
    uiBytesPerSample = std::max( (uint32_t)topField->planes[c].bytesPerSample, uiBytesPerSample );
  }

  for( c = 0; c < topField->numPlanes; c++ )
  {
    if( ( ret = _writeComponentToFile( f, &topField->planes[c], &botField->planes[c], uiBytesPerSample ) ) != 0 )
    {
      return ret;
    }
  }
  return 0;
}

static inline std::string getTimePointAsString( )
{
  std::chrono::system_clock::time_point _time_now = std::chrono::system_clock::now();

  //first step: create a time string with second accuracy using time_t and the standard C runtime.
  time_t      time_t_now = std::chrono::system_clock::to_time_t( _time_now );
  std::string time_str;
  struct tm   tm;
  struct tm*  tm_ptr = nullptr;

  std::chrono::system_clock::time_point time_now_sec =  std::chrono::system_clock::from_time_t( time_t_now );
  if( time_now_sec > _time_now )
  {
    // The std::chrono implementation of to_time_t does a nearest integer rounding or similar (e.g. Visual C++ 2012.) However, the majority of
    // chrono implementations round towards zero.
    time_t_now--;
    time_now_sec -=  std::chrono::seconds( 1 );
  }

#if defined( _WIN32 ) && defined( _MSC_VER )
  // Visual C++ with support for the secure runtime.
  errno_t err;
  err = ::localtime_s( &tm, &time_t_now );
  if( 0 != err )
  {
    assert( 0 == err );
    return time_str;
  }
  tm_ptr = &tm;
#else

# if defined( __MINGW32__ )
  // 32 bit and 64 bit MinGW does not come with reentrant versions of the CRT time functions.
  tm_ptr = ::localtime( &time_t_now );
  if( nullptr == tm_ptr )
  {
    assert( nullptr != tm_ptr );
    return time_str;
  }
  // tm_ptr points hopefully to TLS or to some other global storage (very bad). The copy simplifies the implementation downwards
  // that may have to write to the memory tm_ptr points to.
  tm = *tm_ptr;
  tm_ptr = &tm;
# else
  // The reentrant version of localtime and gmtime are available.
  tm_ptr = ::localtime_r( &time_t_now, &tm );
  if( tm_ptr != &tm )
  {
    assert( tm_ptr == &tm );
    return time_str;
  }
# endif
#endif

  char   buffer[ 512 ];
  size_t len = ::strftime( buffer, sizeof( buffer ), "%Y-%b-%d %H:%M:%S.", tm_ptr );

  if( 0 == len )
  {
    assert( 0 );
    return time_str;
  }

  time_str.assign( buffer, len );

  // second step: add remaining microseconds
  std::ostringstream        tmp_strm;
  std::chrono::microseconds microsec =  std::chrono::duration_cast<  std::chrono::microseconds >( _time_now - time_now_sec );

  tmp_strm << std::setfill( '0' ) << std::setw( 6 ) << microsec.count();
  time_str += tmp_strm.str();

  return time_str;
}


static inline std::string getNalUnitTypeAsString( vvdecNalType eNalType )
{
  std::string cNalType = "VVC_NAL_UNIT_UNIT_INVALID";

  switch ( eNalType )
  {
  case VVC_NAL_UNIT_CODED_SLICE_TRAIL:           cNalType = "VVC_NAL_UNIT_CODED_SLICE_TRAIL"; break; // 0
  case VVC_NAL_UNIT_CODED_SLICE_STSA:            cNalType = "VVC_NAL_UNIT_CODED_SLICE_STSA"; break; // 1
  case VVC_NAL_UNIT_CODED_SLICE_RADL:            cNalType = "VVC_NAL_UNIT_CODED_SLICE_RADL"; break; // 2
  case VVC_NAL_UNIT_CODED_SLICE_RASL:            cNalType = "VVC_NAL_UNIT_CODED_SLICE_RASL"; break; // 3

  case VVC_NAL_UNIT_RESERVED_VCL_4:              cNalType = "VVC_NAL_UNIT_RESERVED_VCL_4"; break; // 4
  case VVC_NAL_UNIT_RESERVED_VCL_5:              cNalType = "VVC_NAL_UNIT_RESERVED_VCL_5"; break; // 5
  case VVC_NAL_UNIT_RESERVED_VCL_6:              cNalType = "VVC_NAL_UNIT_RESERVED_VCL_6"; break; // 6

  case VVC_NAL_UNIT_CODED_SLICE_IDR_W_RADL:      cNalType = "VVC_NAL_UNIT_CODED_SLICE_IDR_W_RADL"; break; // 7
  case VVC_NAL_UNIT_CODED_SLICE_IDR_N_LP:        cNalType = "VVC_NAL_UNIT_CODED_SLICE_IDR_N_LP"; break; // 8
  case VVC_NAL_UNIT_CODED_SLICE_CRA:             cNalType = "VVC_NAL_UNIT_CODED_SLICE_CRA"; break; // 9
  case VVC_NAL_UNIT_CODED_SLICE_GDR:             cNalType = "VVC_NAL_UNIT_CODED_SLICE_GDR"; break; // 10

  case VVC_NAL_UNIT_RESERVED_IRAP_VCL_11:        cNalType = "VVC_NAL_UNIT_RESERVED_IRAP_VCL_11"; break; // 11
  case VVC_NAL_UNIT_RESERVED_IRAP_VCL_12:        cNalType = "VVC_NAL_UNIT_RESERVED_IRAP_VCL_12"; break; // 12

  case VVC_NAL_UNIT_DCI:                         cNalType = "VVC_NAL_UNIT_DCI"; break; // 13
  case VVC_NAL_UNIT_VPS:                         cNalType = "VVC_NAL_UNIT_VPS"; break; // 14
  case VVC_NAL_UNIT_SPS:                         cNalType = "VVC_NAL_UNIT_SPS"; break; // 15
  case VVC_NAL_UNIT_PPS:                         cNalType = "VVC_NAL_UNIT_PPS"; break; // 16
  case VVC_NAL_UNIT_PREFIX_APS:                  cNalType = "VVC_NAL_UNIT_PREFIX_APS"; break; // 17
  case VVC_NAL_UNIT_SUFFIX_APS:                  cNalType = "VVC_NAL_UNIT_SUFFIX_APS"; break; // 18
  case VVC_NAL_UNIT_PH:                          cNalType = "VVC_NAL_UNIT_PH"; break; // 19
  case VVC_NAL_UNIT_ACCESS_UNIT_DELIMITER:       cNalType = "VVC_NAL_UNIT_ACCESS_UNIT_DELIMITER"; break; // 20
  case VVC_NAL_UNIT_EOS:                         cNalType = "VVC_NAL_UNIT_EOS"; break; // 21
  case VVC_NAL_UNIT_EOB:                         cNalType = "VVC_NAL_UNIT_EOB"; break; // 22
  case VVC_NAL_UNIT_PREFIX_SEI:                  cNalType = "VVC_NAL_UNIT_PREFIX_SEI"; break; // 23
  case VVC_NAL_UNIT_SUFFIX_SEI:                  cNalType = "VVC_NAL_UNIT_SUFFIX_SEI"; break; // 24
  case VVC_NAL_UNIT_FD:                          cNalType = "VVC_NAL_UNIT_FD"; break; // 25

  case VVC_NAL_UNIT_RESERVED_NVCL_26:            cNalType = "VVC_NAL_UNIT_RESERVED_NVCL_26"; break; // 26
  case VVC_NAL_UNIT_RESERVED_NVCL_27:            cNalType = "VVC_NAL_UNIT_RESERVED_NVCL_27"; break; // 27

  case VVC_NAL_UNIT_UNSPECIFIED_28:              cNalType = "VVC_NAL_UNIT_UNSPECIFIED_28"; break; // 28
  case VVC_NAL_UNIT_UNSPECIFIED_29:              cNalType = "VVC_NAL_UNIT_UNSPECIFIED_29"; break; // 29
  case VVC_NAL_UNIT_UNSPECIFIED_30:              cNalType = "VVC_NAL_UNIT_UNSPECIFIED_30"; break; // 30
  case VVC_NAL_UNIT_UNSPECIFIED_31:              cNalType = "VVC_NAL_UNIT_UNSPECIFIED_31"; break; // 31
  case VVC_NAL_UNIT_INVALID:
  default:                                              cNalType = "VVC_NAL_UNIT_INVALID"; break;
  }

  return cNalType;
}

std::string hashToString( const std::vector<uint8_t> hash, int numChar )
{
  static const char* hex = "0123456789abcdef";
  std::string result;

  assert( numChar > 0 );

  for( int pos=0; pos<int( hash.size() ); pos++ )
  {
    if( ( pos % numChar ) == 0 && pos!=0 )
    {
      result += ',';
    }
    result += hex[hash[pos] >> 4];
    result += hex[hash[pos] & 0xf];
  }

  return result;
}

void md5_plane( vvdec::libmd5::MD5& md5, const vvdecPlane *plane )
{
  uint32_t width  = plane->width;
  uint32_t height = plane->height;
  uint32_t stride = plane->stride;
  
  uint32_t N = 32;
  uint32_t width_modN = width % N;
  uint32_t width_less_modN = width - width_modN;
  
  if( plane->bytesPerSample == 2 )
  {    
    unsigned short* pc  = reinterpret_cast<unsigned short*>( plane->ptr );
    uint8_t buf[32][2];
    stride >>= 1;

    for( uint32_t y = 0; y < height; y++ )
    {
      uint16_t *p;
      for( uint32_t x = 0; x < width_less_modN; x += N )
      {
        p = &pc[y*stride + x];
        for( uint32_t i = 0; i < N; i++ )
        {
          uint16_t pel = p[i];
          for( uint32_t d = 0; d < 2; d++ )
          {
            buf[i][d] = pel >> (d*8);
          }
        }
        md5.update( (uint8_t*)buf, N*2 );
      }

      p = &pc[y*stride + width_less_modN];
      for( uint32_t i = 0; i < width_modN; i++ )
      {
        uint16_t pel = p[i];
        for( uint32_t d = 0; d < 2; d++ )
        {
          buf[i][d] = pel >> (d*8);
        }
      }
      md5.update((uint8_t*)buf, width_modN*2);
    }
  }
  else
  {
    uint8_t *pc = plane->ptr;
    uint8_t buf[64][1];

    for( uint32_t y = 0; y < height; y++ )
    {
      uint8_t *p;
      for( uint32_t x = 0; x < width_less_modN; x += N )
      {
        p = &pc[y*stride + x];
        for( uint32_t i = 0; i < N; i++ )
        {
          buf[i][0] = p[i];
        }
        md5.update( (uint8_t*)buf, N );
      }

      p = &pc[y*stride + width_less_modN];
      for( uint32_t i = 0; i < width_modN; i++ )
      {
        buf[i][0] = p[i];
      }
      md5.update( (uint8_t*)buf, width_modN );
    }
  }
}

int calc_md5( vvdecFrame *frame, std::vector<uint8_t> &hash )
{
  hash.clear();
  vvdec::libmd5::MD5 md5[3];

  for( uint32_t c = 0; c < frame->numPlanes; c++ )
  {
    uint8_t tmp_digest[vvdec::MD5_DIGEST_STRING_LENGTH];
    md5_plane( md5[c], &frame->planes[c] );
    md5[c].finalize( tmp_digest );
    for( uint32_t i = 0; i < vvdec::MD5_DIGEST_STRING_LENGTH; i++ )
    {
      hash.push_back( tmp_digest[i] );
    }
  }
  
  return 16;
}

void crc_plane( const vvdecPlane *plane, std::vector<uint8_t> &hash )
{
  uint32_t width  = plane->width;
  uint32_t height = plane->height;
  uint32_t stride = plane->stride;
  
  uint32_t crcMsb;
  uint32_t bitVal;
  uint32_t crcVal  = 0xffff;
  uint32_t bitIdx;

  if( plane->bytesPerSample == 2 )
  {
    unsigned short* pc  = reinterpret_cast<unsigned short*>( plane->ptr );

    for( uint32_t y = 0; y < height; y++ )
    {
      for( uint32_t x = 0; x < width; x++ )
      {
        uint16_t pel = pc[x];

        for( bitIdx = 0; bitIdx < 8; bitIdx++ )
        {
          crcMsb = (crcVal >> 15) & 1;
          bitVal = (pel >> (7 - bitIdx)) & 1;
          crcVal = (((crcVal << 1) + bitVal) & 0xffff) ^ (crcMsb * 0x1021);
        }
        for( bitIdx = 0; bitIdx < 8; bitIdx++ )
        {
          crcMsb = (crcVal >> 15) & 1;
          bitVal = (pel >> (15 - bitIdx)) & 1;
          crcVal = (((crcVal << 1) + bitVal) & 0xffff) ^ (crcMsb * 0x1021);
        }
      }
      pc += (stride >> 1);
    }
  }
  else
  {
    uint8_t *pc = plane->ptr;

    for( uint32_t y = 0; y < height; y++ )
    {
      for( uint32_t x = 0; x < width; x++ )
      {
        for( bitIdx = 0; bitIdx < 8; bitIdx++ )
        {
          crcMsb = (crcVal >> 15) & 1;
          bitVal = (pc[y*stride+x] >> (7 - bitIdx)) & 1;
          crcVal = (((crcVal << 1) + bitVal) & 0xffff) ^ (crcMsb * 0x1021);
        }
      }
    }
  }
  
  for( bitIdx = 0; bitIdx < 16; bitIdx++ )
  {
    crcMsb = (crcVal >> 15) & 1;
    crcVal = ((crcVal << 1) & 0xffff) ^ (crcMsb * 0x1021);
  }

  hash.push_back( (crcVal>>8)  & 0xff );
  hash.push_back(  crcVal      & 0xff );
}

int calc_crc( vvdecFrame *frame, std::vector<uint8_t> &hash )
{
  hash.clear();

  for( uint32_t c = 0; c < frame->numPlanes; c++ )
  {
    crc_plane( &frame->planes[c], hash );
  }
  
  return 2;
}

void checksum_plane( const vvdecPlane *plane, std::vector<uint8_t> &hash )
{
  uint32_t width  = plane->width;
  uint32_t height = plane->height;
  uint32_t stride = plane->stride;

  uint32_t checksum = 0;
  uint8_t xor_mask;

  if( plane->bytesPerSample == 2 )
  {
    unsigned short* pc  = reinterpret_cast<unsigned short*>( plane->ptr );

    for( uint32_t y = 0; y < height; y++ )
    {
      for( uint32_t x = 0; x < width; x++ )
      {
        uint16_t pel = pc[x];
        xor_mask = (x & 0xff) ^ (y & 0xff) ^ (x >> 8) ^ (y >> 8);
        checksum = (checksum + ((pel & 0xff) ^ xor_mask)) & 0xffffffff;
        checksum = (checksum + ((pel >> 8)   ^ xor_mask)) & 0xffffffff;
      }
      pc += (stride>>1);
    }
  }
  else
  {
    uint8_t *pc = plane->ptr;

    for( uint32_t y = 0; y < height; y++ )
    {
      for( uint32_t x = 0; x < width; x++ )
      {
        xor_mask = (x & 0xff) ^ (y & 0xff) ^ (x >> 8) ^ (y >> 8);
        checksum = (checksum + (pc[y*stride+x] ^ xor_mask)) & 0xffffffff;
      }
    }
  }

  hash.push_back( (checksum>>24) & 0xff );
  hash.push_back( (checksum>>16) & 0xff );
  hash.push_back( (checksum>>8)  & 0xff );
  hash.push_back(  checksum      & 0xff );
}

int calc_checksum( vvdecFrame *frame, std::vector<uint8_t> &hash )
{
  hash.clear();

  for( uint32_t c = 0; c < frame->numPlanes; c++ )
  {
    checksum_plane( &frame->planes[c], hash );
  }
  
  return 4;
}

void printPicHash( vvdecFrame *frame, std::ostream * logStream, unsigned int frames, int method = 0 )
{
  int numChar=0;
  const char* hashType = "\0";
  std::vector<uint8_t> picHash;
  
  switch( method )
  {
    case VVDEC_HASHTYPE_MD5:
    {
      hashType = "MD5";
      numChar = calc_md5( frame, picHash );
      break;
    }
    case VVDEC_HASHTYPE_CRC:
    {
      hashType = "CRC";
      numChar = calc_crc( frame, picHash );
      break;
    }
    case VVDEC_HASHTYPE_CHECKSUM:
    {
      hashType = "Checksum";
      numChar = calc_checksum( frame, picHash );
      break;
    }
    default:
    {
      assert( 0 );
      break;
    }
  }

  *logStream << "vvdecapp picHash frame " << frames <<  "  [" << hashType << ":" << hashToString( picHash, numChar ) << "]"  << std::endl;
}

namespace vvdec
{
// This is defined in the vvdec lib but not exported in the official header
VVDEC_DECL void rescalePlane( const vvdecPlane&      srcPlane,
                              vvdecPlane&            dstPlane,
                              int                    planeComponent,
                              const vvdecColorFormat colorFormat,
                              int                    bitDepth,
                              const bool             horCollocatedChromaFlag,
                              const bool             verCollocatedChromaFlag );
}   // namespace vvdec

static void upscaleFrame( const vvdecFrame* srcFrame, vvdecFrame* dstFrame )
{
  bool collChromaHor = true;   // for chroma format other than 4:2:0  hor- & ver- are always collocated and vui_chroma_sample_loc_type* shall be ignored
  bool collChromaVer = true;

  if( srcFrame->colorFormat == VVDEC_CF_YUV420_PLANAR )
  {
    // default for 4:2:0
    collChromaHor = true;
    collChromaVer = false;

    const auto* vui = srcFrame->picAttributes ? srcFrame->picAttributes->vui : nullptr;
    if( vui && vui->colourPrimaries == 9 )
    {
      collChromaVer = true;   // default for Bt.2020
    }
    if( vui && vui->chromaLocInfoPresentFlag )
    {
      switch( vui->chromaSampleLocType )
      {
        // clang-format off
      case 0: collChromaHor = true;  collChromaVer = false; break;
      case 1: collChromaHor = false; collChromaVer = false; break;
      case 2: collChromaHor = true;  collChromaVer = true;  break;
      case 3: collChromaHor = false; collChromaVer = true;  break;
      case 6: /* unspecified */;                            break;
        // clang-format on
      default:
        std::cerr << "chromaSampleLocType==" << vui->chromaSampleLocType << "not implemented" << std::endl;
        return;
      }
    }
  }

  for( unsigned i = 0; i < srcFrame->numPlanes; ++i )
  {
    vvdec::rescalePlane( srcFrame->planes[i], dstFrame->planes[i], i, srcFrame->colorFormat, srcFrame->bitDepth, collChromaHor, collChromaVer );
  }
}

static void copyIntoFrame( const vvdecFrame* srcFrame, vvdecFrame* dstFrame )
{
  for( unsigned i = 0; i < srcFrame->numPlanes; ++i )
  {
    memset( dstFrame->planes[i].ptr, 0, dstFrame->planes[i].stride * dstFrame->planes[i].height );
    for( unsigned j = 0; j < srcFrame->planes[i].height; ++j )
    {
      memcpy( dstFrame->planes[i].ptr + j * dstFrame->planes[i].stride,
              srcFrame->planes[i].ptr + j * srcFrame->planes[i].stride,
              srcFrame->planes[i].width * srcFrame->planes[i].bytesPerSample );
    }
  }
}
