/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2018-2023, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVdeC Authors.
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

/** \file     VLCWReader.h
 *  \brief    Reader for high level syntax
 */

#pragma once

#include "CommonLib/Rom.h"
#include "CommonLib/BitStream.h"
#include "CommonLib/Slice.h"
#include "CommonLib/SampleAdaptiveOffset.h"
#include "CABACReader.h"

#if ENABLE_TRACING

#define READ_SCODE(length, code, name)    xReadSCode  ( length, code, name )
#define READ_CODE(length, code, name)     xReadCodeTr ( length, code, name )
#define READ_UVLC(        code, name)     xReadUvlcTr (         code, name )
#define READ_SVLC(        code, name)     xReadSvlcTr (         code, name )
#define READ_FLAG(        code, name)     xReadFlagTr (         code, name )

#else


#define READ_SCODE(length, code, name)    xReadSCode ( length, code )
#define READ_CODE(length, code, name)     xReadCode ( length, code )
#define READ_UVLC(        code, name)     xReadUvlc (         code )
#define READ_SVLC(        code, name)     xReadSvlc (         code )
#define READ_FLAG(        code, name)     xReadFlag (         code )


#endif

namespace vvdec
{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

class VLCReader
{
protected:
  InputBitstream*   m_pcBitstream;

  VLCReader() : m_pcBitstream (NULL) {};
  virtual ~VLCReader() {};

  void  xReadCode    ( uint32_t   length, uint32_t& val );
  void  xReadUvlc    ( uint32_t& val );
  void  xReadSvlc    ( int& val );
  void  xReadFlag    ( uint32_t& val );
#if ENABLE_TRACING
  void  xReadCodeTr  ( uint32_t  length, uint32_t& rValue, const char *pSymbolName );
  void  xReadUvlcTr  ( uint32_t& rValue, const char *pSymbolName );
  void  xReadSvlcTr  ( int& rValue, const char *pSymbolName );
  void  xReadFlagTr  ( uint32_t& rValue, const char *pSymbolName );
#endif
#if ENABLE_TRACING
  void  xReadSCode   ( uint32_t length, int& val, const char *pSymbolName );
#else
  void  xReadSCode   ( uint32_t length, int& val );
#endif

public:
  void  setBitstream ( InputBitstream* p )   { m_pcBitstream = p; }
  InputBitstream* getBitstream() { return m_pcBitstream; }

protected:
  void xReadRbspTrailingBits();
  bool isByteAligned() { return ( m_pcBitstream->getNumBitsUntilByteAligned() == 0 ); }
};



class AUDReader: public VLCReader
{
public:
  AUDReader() {};
  virtual ~AUDReader() {};
  void parseAccessUnitDelimiter( InputBitstream* bs, uint32_t &picType );
};



class FDReader: public VLCReader
{
public:
  FDReader() {};
  virtual ~FDReader() {};
  void parseFillerData(InputBitstream* bs, uint32_t &fdSize);
};



class HLSyntaxReader : public VLCReader
{
public:
  HLSyntaxReader()          = default;
  virtual ~HLSyntaxReader() = default;

  void  parseVPS                 ( VPS* pcVPS );
  void  parseDCI                 ( DCI* dci );
  void  parseSPS                 ( SPS* pcSPS, const ParameterSetManager* parameterSetManager );
  void  parsePPS                 ( PPS* pcPPS, const ParameterSetManager* parameterSetManager );
  void  parseAPS                 ( APS* pcAPS);
  void  parsePictureHeader       ( PicHeader* picHeader, const ParameterSetManager* parameterSetManager, bool readRbspTrailingBits );
  void  parseSliceHeader         ( Slice* pcSlice, std::shared_ptr<PicHeader>& picHeader, const ParameterSetManager* parameterSetManager, const int prevTid0POC, bool& firstSliceInPic );

private:
  void  parseAlfAps              ( APS* pcAPS );
  void  parseLmcsAps             ( APS* pcAPS );
  void  parseScalingListAps      ( APS* pcAPS );
  void  parseVUI                 ( VUI* pcVUI, SPS* pcSPS );
  void  parseConstraintInfo      ( ConstraintInfo *cinfo );
  void  parseProfileTierLevel    ( ProfileTierLevel *ptl, bool profileTierPresentFlag, int maxNumSubLayersMinus1 );
  void  parseOlsHrdParameters    ( GeneralHrdParams* generalHrd, OlsHrdParams *olsHrd, uint32_t firstSubLayer, uint32_t tempLevelHigh );
  void  parseGeneralHrdParameters( GeneralHrdParams *generalHrd );
  template<typename HeaderT>
  void  parsePicOrSliceHeaderRPL ( HeaderT* header, const SPS* sps, const PPS* pps );
  void  parseRefPicList          ( ReferencePictureList* rpl, int rplIdx, const SPS* pcSPS );
  void  copyRefPicList           ( const SPS* pcSPS, const ReferencePictureList* source_rpl, ReferencePictureList* dest_rpl );

  void  checkAlfNaluTidAndPicTid ( const Slice* pcSlice, const PicHeader* picHeader, const ParameterSetManager* parameterSetManager);

  void  parsePredWeightTable     ( Slice* pcSlice, const SPS *sps );
  void  parsePredWeightTable     ( PicHeader *picHeader, const SPS *sps );
  void  parseScalingList         ( ScalingList *scalingList, bool aps_chromaPresentFlag );
  void  decodeScalingList        ( ScalingList *scalingList, uint32_t scalingListId, bool isPredictor );
  void  alfFilter                ( AlfSliceParam& alfSliceParam, const bool isChroma, const int altIdx );
  void  dpb_parameters           ( int maxSubLayersMinus1, bool subLayerInfoFlag, SPS *pcSPS );
  void  parseExtraPHBitsStruct   ( SPS *sps, int numBytes );
  void  parseExtraSHBitsStruct   ( SPS *sps, int numBytes );

  bool  xMoreRbspData();
};

}
