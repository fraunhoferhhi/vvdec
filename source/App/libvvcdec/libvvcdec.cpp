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

#include "libvvcdec.h"

#include "vvdec/vvdec.h"
#include "vvdec/version.h"

namespace
{

class vvcDecoderWrapper
{
public:
  vvcDecoderWrapper() = default;
  ~vvcDecoderWrapper() = default;
  int init()
  {
    vvdec::VVDecParameter cVVDecParameter;
    return this->cVVDec.init( cVVDecParameter );
  }
private:
  vvdec::VVDec cVVDec;
};

}

extern "C" {

  VVCDECAPI const char *libvvcdec_get_version(void)
  {
    return VVDEC_VERSION;
  }

  VVCDECAPI libvvcdec_context* libvvcdec_new_decoder(void)
  {
    auto decCtx = new vvcDecoderWrapper();
    if (!decCtx)
      return nullptr;

    auto ret = decCtx->init();
    if (ret != 0)
    {
      // Error initializing the decoder
      delete decCtx;
      return nullptr;
    }

    return (libvvcdec_context*)decCtx;
  }

  VVCDECAPI libvvcdec_error libvvcdec_free_decoder(libvvcdec_context* decCtx)
  {
    auto d = (vvcDecoderWrapper*)decCtx;
    if (!d)
      return LIBVVCDEC_ERROR;

    delete d;
    return LIBVVCDEC_OK;
  }

  VVCDECAPI libvvcdec_error libvvcdec_push_nal_unit(libvvcdec_context *decCtx, const void* data8, int length, bool eof, bool &bNewPicture, bool &checkOutputPictures)
  {
    auto d = (vvcDecoderWrapper*)decCtx;
    if (!d)
      return LIBVVCDEC_ERROR;

    // TODO
    (void)data8;
    (void)length;
    (void)eof;
    (void)bNewPicture;
    (void)checkOutputPictures;

    return LIBVVCDEC_OK;
  }

  VVCDECAPI libvvcdec_picture *libvvcdec_get_picture(libvvcdec_context* decCtx)
  {
    auto d = (vvcDecoderWrapper*)decCtx;
    if (!d)
      return nullptr;

    // TODO
    return nullptr;
  }

  VVCDECAPI int libHMDEC_get_POC(libvvcdec_picture *pic)
  {
    if (pic == nullptr)
      return -1;
    
    // TODO
    return 0;
  }

  VVCDECAPI int libHMDEC_get_picture_width(libvvcdec_picture *pic, libvvcdec_ColorComponent c)
  {
    if (pic == nullptr)
      return -1;
    
    // TODO
    (void)c;

    return -1;
  }

  VVCDECAPI int libHMDEC_get_picture_height(libvvcdec_picture *pic, libvvcdec_ColorComponent c)
  {
    if (pic == nullptr)
      return -1;
    
    // TODO
    (void)c;

    return -1;
  }

  VVCDECAPI int libHMDEC_get_picture_stride(libvvcdec_picture *pic, libvvcdec_ColorComponent c)
  {
    if (pic == nullptr)
      return -1;
    
    // TODO
    (void)c;

    return -1;
  }

  VVCDECAPI short* libHMDEC_get_image_plane(libvvcdec_picture *pic, libvvcdec_ColorComponent c)
  {
    if (pic == nullptr)
      return nullptr;
    
    // TODO
    (void)c;

    return nullptr;
  }

  VVCDECAPI libvvcdec_ChromaFormat libHMDEC_get_chroma_format(libvvcdec_picture *pic)
  {
    if (pic == nullptr)
      return LIBVVCDEC_CHROMA_UNKNOWN;
    
    // TODO
    return LIBVVCDEC_CHROMA_UNKNOWN;
  }

  VVCDECAPI int libHMDEC_get_internal_bit_depth(libvvcdec_picture *pic, libvvcdec_ColorComponent c)
  {
    if (pic == nullptr)
      return -1;

    // TODO
    (void)c;

    return -1;
  }

} // extern "C"
