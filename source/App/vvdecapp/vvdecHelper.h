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

#include "../../../include/vvdec/vvdec.h"

#define MAX_CODED_PICTURE_SIZE  800000

static int _writeComponentToFile( std::ostream *f, vvdec::Component *comp, uint32_t uiBytesPerSample, int8_t iScale = 0 )
{
  uint32_t uiWidth  = comp->m_uiWidth;
  uint32_t uiHeight = comp->m_uiHeight;

  assert( f != NULL );

  if( comp->m_uiBytesPerSample == 2 )
  {
     unsigned short* p  = reinterpret_cast<unsigned short*>( comp->m_pucBuffer );
     if( uiBytesPerSample == 1 )  // cut to 8bit output
     {
       // 8bit > 16bit conversion
       std::vector<unsigned char> tmp;
       tmp.resize(uiWidth);

       for( uint32_t y = 0; y < uiHeight; y++ )
       {
         for( uint32_t x = 0; x < uiWidth; x++ )
         {
           tmp[x] = (unsigned char)(p[x]>>2);
         }
         f->write( (char*)&tmp[0], sizeof(std::vector<unsigned char>::value_type)*tmp.size());
         p += comp->m_iStride;
       }
     }
     else
     {
       unsigned char *p = comp->m_pucBuffer;
       for( uint32_t y = 0; y < uiHeight; y++ )
       {
         f->write( (char*)p, uiWidth*uiBytesPerSample );
         p += comp->m_iStride;
       }
     }
  }
  else
  {
   uint8_t *p = comp->m_pucBuffer;
   if( uiBytesPerSample == 2 )
   {
     // 8bit > 16bit conversion
     std::vector<short> tmp;
     tmp.resize(uiWidth);

     for( uint32_t y = 0; y < uiHeight; y++ )
     {
       for( uint32_t x = 0; x < uiWidth; x++ )
       {
         tmp[x] = p[x] << iScale;
       }

       f->write( (char*)&tmp[0], sizeof(std::vector<short>::value_type)*tmp.size());
       p += comp->m_iStride;
     }
   }
   else
   {
     for( uint32_t y = 0; y < uiHeight; y++ )
     {
       f->write( (char*)p, uiBytesPerSample*uiWidth );
       p += comp->m_iStride;
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
static int readBitstreamFromFile( std::ifstream *f, vvdec::AccessUnit* pcAccessUnit, bool bLoop )
{
  int info2=0;
  int info3=0;
  int pos = 0;

  int iStartCodeFound =0;
  int iRewind=0;
  uint32_t len;
  unsigned char* pBuf = pcAccessUnit->m_pucBuffer;
  pcAccessUnit->m_iUsedSize = 0;

  int curfilpos = f->tellg();
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
        pcAccessUnit->m_iUsedSize=len;
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

    if( pos >= pcAccessUnit->m_iBufSize )
    {
      int iNewSize = pcAccessUnit->m_iBufSize*2;
      unsigned char* newbuf = new unsigned char[iNewSize];
      if( newbuf == NULL )
      {
        fprintf( stderr, "ERR: readBitstreamFromFile: memory re-allocation failed!\n" );
        return -1;
      }
      std::copy_n( pcAccessUnit->m_pucBuffer, std::min( pcAccessUnit->m_iBufSize , iNewSize), newbuf);
      pcAccessUnit->m_iBufSize = iNewSize;
      delete[] pcAccessUnit->m_pucBuffer;

      pcAccessUnit->m_pucBuffer = newbuf;
      pBuf = pcAccessUnit->m_pucBuffer;
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
      printf("readBitstreamFromFile: Error in next start code search \n");

  f->seekg ( iRewind, f->cur);
  if(f->bad() || f->fail())
  {
    printf("readBitstreamFromFile: Cannot seek %d in the bit stream file", iRewind );
    return -1;
  }

  // Here the Start code, the complete NALU, and the next start code is in the pBuf.
  // The size of pBuf is pos, pos+rewind are the number of bytes excluding the next
  // start code, and (pos+rewind)-startcodeprefix_len is the size of the NALU

  len = (pos+iRewind);
  pcAccessUnit->m_iUsedSize=len;
  return len;
}


 /**
   This method writes and yuv planar picture into a file sink.
   \param[in]  *f file sink pointer
   \param[in]  *frame decoded picture pointer
   \retval     int  if non-zero an error occurred (see ErrorCodes), otherwise the return value indicates success VVC_DEC_OK
   \pre        The decoder must not be initialized.
 */
static int writeYUVToFile( std::ostream *f, vvdec::Frame *frame )
{
  int ret;
  uint32_t c = 0;

  uint32_t uiBytesPerSample = 1;
  uint32_t uiBitDepth       = frame->m_uiBitDepth;

  assert( f != NULL );

  for( c = 0; c < frame->m_uiNumComponents; c++ )
  {
    uiBytesPerSample = std::max( (uint32_t)frame->m_cComponent[c].m_uiBytesPerSample, uiBytesPerSample );
  }

  for( c = 0; c < frame->m_uiNumComponents; c++ )
  {
    uint32_t iScale = uiBitDepth - frame->m_cComponent[c].m_uiBitDepth;

    if( ( ret = _writeComponentToFile( f, &frame->m_cComponent[c], uiBytesPerSample, iScale ) ) != 0 )
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
