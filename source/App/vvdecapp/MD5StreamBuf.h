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

#include "MD5.h"

#include <streambuf>
#include <array>
#include <vector>

namespace vvdecoderapp
{

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

}   // namespace vvdecoderapp
