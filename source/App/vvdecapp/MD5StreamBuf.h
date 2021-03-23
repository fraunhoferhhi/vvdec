/**
  \ingroup Utilities
  \file    MD5StreamBuf.h
  \brief   This class implements a std::streambuf and instead of writing the output to a file, it calculates the md5-sum
  \author  gabriel.hege@hhi.fraunhofer.de
  \date    07.12.2018

  Copyright:
  2018 Fraunhofer Institute for Telecommunications, Heinrich-Hertz-Institut (HHI)
  The copyright of this software source code is the property of HHI.
  This software may be used and/or copied only with the written permission
  of HHI and in accordance with the terms and conditions stipulated
  in the agreement/contract under which the software has been supplied.
  The software distributed under this license is distributed on an "AS IS" basis,
  WITHOUT WARRANTY OF ANY KIND, either expressed or implied.
*/

#ifndef MD5_STREAMBUF_H
#define MD5_STREAMBUF_H

#include "MD5.h"

#include <streambuf>
#include <array>
#include <vector>

const int MD5_DIGEST_LENGTH = 128/8;

static inline std::string MD5ToHexString(unsigned char hash[MD5_DIGEST_LENGTH])
{
  static const char* hex = "0123456789abcdef";
  std::string result;

  for(unsigned pos=0; pos < MD5_DIGEST_LENGTH; ++pos)
  {
    result += hex[hash[pos] >> 4];
    result += hex[hash[pos] & 0xf];
  }

  return result;
}

class MD5StreamBuf : public std::streambuf
{
public:
  explicit MD5StreamBuf( size_t buf_size = 64 )
    : m_buffer( buf_size )
  {
    std::streambuf::setp( &m_buffer.front(), &m_buffer.back() + 1 );
  }

  int_type overflow(int_type ch) override
  {
    m_md5_ctx.update( (unsigned char*)m_buffer.data(), (unsigned int)m_buffer.size() );

    // reset start & end-pointers
    std::streambuf::setp( &m_buffer.front(), &m_buffer.back() + 1 );
    return std::streambuf::sputc(ch);
  }

  int sync() override
  {
    const auto fillLen = std::streambuf::pptr() - std::streambuf::pbase();
    if( fillLen )
      m_md5_ctx.update( (unsigned char*)std::streambuf::pbase(), (unsigned int)fillLen );
    return 0;
  }

  void finalize(unsigned char digest[16])
  {
    sync();
    m_md5_ctx.finalize( digest);
  }

  std::string finalizeHex()
  {
    unsigned char digest[MD5_DIGEST_LENGTH];
    finalize(digest);
    return MD5ToHexString(digest);
  }

private:
  vvdec::libmd5::MD5     m_md5_ctx;
  std::vector<char_type> m_buffer;
};

#endif // MD5_STREAMBUF_H
