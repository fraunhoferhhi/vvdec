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
  vvcDecoderWrapper()
  {
    this->cAccessUnit.m_pucBuffer = nullptr;
    this->cAccessUnit.m_iBufSize = 0;
    this->cAccessUnit.m_uiCts = 0;
    this->cAccessUnit.m_bCtsValid = true;
    this->cAccessUnit.m_uiDts = 0;
    this->cAccessUnit.m_bDtsValid = true;
  }
  ~vvcDecoderWrapper()
  {
    if (this->cAccessUnit.m_pucBuffer != nullptr)
    {
      delete [] this->cAccessUnit.m_pucBuffer;
    }
  }
  int init()
  {
    vvdec::VVDecParameter cVVDecParameter;
    return this->cVVDec.init( cVVDecParameter );
  }
  bool setAUData(const unsigned char* data8, int length)
  {
    if (length > this->cAccessUnit.m_iBufSize)
    {
      // Allocate a new big enough buffer
      if (this->cAccessUnit.m_pucBuffer != nullptr)
      {
        delete[] this->cAccessUnit.m_pucBuffer;
      }
      this->cAccessUnit.m_pucBuffer = new unsigned char[length];
      if (this->cAccessUnit.m_pucBuffer == nullptr)
      {
        this->cAccessUnit.m_iBufSize = 0;
        return false;
      }
      this->cAccessUnit.m_iBufSize = length;
    }
    std::copy_n( data8 , length, this->cAccessUnit.m_pucBuffer);
    this->cAccessUnit.m_iUsedSize = length;
    return true;
  }
  int decode()
  {
    return this->cVVDec.decode( this->cAccessUnit, &pcFrame );
  }
  int flush()
  {
    return this->cVVDec.flush( &this->pcFrame );
  }
  bool gotFrame() const
  {
    return this->pcFrame != nullptr && this->pcFrame->m_bCtsValid;
  }
  vvdec::Frame* getFrame() const
  {
    return this->pcFrame;
  }
private:
  vvdec::VVDec cVVDec;
  vvdec::AccessUnit cAccessUnit;
  vvdec::Frame* pcFrame {nullptr};
};

unsigned getComponentIndex(libvvcdec_ColorComponent c)
{
  if (c == LIBVVCDEC_CHROMA_U)
    return 1;
  if (c == LIBVVCDEC_CHROMA_V)
    return 2;
  return 0;
}

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

  VVCDECAPI libvvcdec_error libvvcdec_push_nal_unit(libvvcdec_context *decCtx, const unsigned char* data8, int length, bool eof, bool &bNewPicture, bool &checkOutputPictures)
  {
    auto d = (vvcDecoderWrapper*)decCtx;
    if (!d)
      return LIBVVCDEC_ERROR;

    if (eof)
    {
      auto iRet = d->flush();
      if( iRet != vvdec::VVDEC_OK && iRet != vvdec::VVDEC_EOF )
      {
        return LIBVVCDEC_ERROR;
      }
    }
    else
    {
      if (!d->setAUData(data8, length))
      return LIBVVCDEC_ERROR;

      auto iRet = d->decode();

      if (iRet != vvdec::VVDEC_OK)
      {
        return LIBVVCDEC_ERROR;
      }
    }

    checkOutputPictures = d->gotFrame();

    // TODO: Do we still need this? I think we don't.
    (void)bNewPicture;

    return LIBVVCDEC_OK;
  }

  VVCDECAPI uint64_t libHMDEC_get_picture_POC(libvvcdec_context *decCtx)
  {
    auto d = (vvcDecoderWrapper*)decCtx;
    if (!d || !d->gotFrame())
      return 0;
    
    return d->getFrame()->m_uiSequenceNumber;
  }

  VVCDECAPI uint32_t libHMDEC_get_picture_width(libvvcdec_context *decCtx, libvvcdec_ColorComponent c)
  {
    auto d = (vvcDecoderWrapper*)decCtx;
    if (!d || !d->gotFrame())
      return 0;
    
    auto f = d->getFrame();
    auto idx = getComponentIndex(c);
    if (idx >= f->m_uiNumComponents)
      return 0;

    return f->m_cComponent[idx].m_uiWidth;
  }

  VVCDECAPI uint32_t libHMDEC_get_picture_height(libvvcdec_context *decCtx, libvvcdec_ColorComponent c)
  {
    auto d = (vvcDecoderWrapper*)decCtx;
    if (!d || !d->gotFrame())
      return 0;
    
    auto f = d->getFrame();
    auto idx = getComponentIndex(c);
    if (idx >= f->m_uiNumComponents)
      return 0;

    return f->m_cComponent[idx].m_uiHeight;
  }

  VVCDECAPI int32_t libHMDEC_get_picture_stride(libvvcdec_context *decCtx, libvvcdec_ColorComponent c)
  {
    auto d = (vvcDecoderWrapper*)decCtx;
    if (!d || !d->gotFrame())
      return 0;
    
    auto f = d->getFrame();
    auto idx = getComponentIndex(c);
    if (idx >= f->m_uiNumComponents)
      return 0;

    return f->m_cComponent[idx].m_iStride;
  }

  VVCDECAPI unsigned char* libHMDEC_get_picture_plane(libvvcdec_context *decCtx, libvvcdec_ColorComponent c)
  {
    auto d = (vvcDecoderWrapper*)decCtx;
    if (!d || !d->gotFrame())
      return 0;
    
    auto f = d->getFrame();
    auto idx = getComponentIndex(c);
    if (idx >= f->m_uiNumComponents)
      return nullptr;

    return f->m_cComponent[idx].m_pucBuffer;
  }

  VVCDECAPI libvvcdec_ChromaFormat libHMDEC_get_picture_chroma_format(libvvcdec_context *decCtx)
  {
    auto d = (vvcDecoderWrapper*)decCtx;
    if (!d || !d->gotFrame())
      return LIBVVCDEC_CHROMA_UNKNOWN;

    auto f = d->getFrame();
    if (f->m_eColorFormat == vvdec::VVC_CF_YUV400_PLANAR)
      return LIBVVCDEC_CHROMA_400;
    if (f->m_eColorFormat == vvdec::VVC_CF_YUV420_PLANAR)
      return LIBVVCDEC_CHROMA_420;
    if (f->m_eColorFormat == vvdec::VVC_CF_YUV422_PLANAR)
      return LIBVVCDEC_CHROMA_420;
    if (f->m_eColorFormat == vvdec::VVC_CF_YUV444_PLANAR)
      return LIBVVCDEC_CHROMA_444;
    
    return LIBVVCDEC_CHROMA_UNKNOWN;
  }

  VVCDECAPI uint32_t libHMDEC_get_picture_bit_depth(libvvcdec_context *decCtx, libvvcdec_ColorComponent c)
  {
    auto d = (vvcDecoderWrapper*)decCtx;
    if (!d || !d->gotFrame())
      return 0;
    
    auto f = d->getFrame();
    auto idx = getComponentIndex(c);
    if (idx >= f->m_uiNumComponents)
      return 0;

    return f->m_cComponent[idx].m_uiBitDepth;
  }

} // extern "C"
