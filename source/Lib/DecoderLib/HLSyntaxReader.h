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

/** \file     HLSyntaxReader.h
 *  \brief    Reader for high level syntax
 */

#pragma once

#include "DecoderLib/VLCReader.h"
#include "CommonLib/Slice.h"

namespace vvdec
{

class AUDReader: public VLCReader
{
public:
  AUDReader(){};
  virtual ~AUDReader(){};
  void parseAccessUnitDelimiter( InputBitstream* bs, uint32_t& picType );
};

class FDReader : public VLCReader
{
public:
  FDReader(){};
  virtual ~FDReader(){};
  void parseFillerData( InputBitstream* bs, uint32_t& fdSize );
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
  bool  parseAPS                 ( APS* pcAPS );
  void  parsePictureHeader       ( PicHeader* picHeader, const ParameterSetManager* parameterSetManager, bool readRbspTrailingBits );
  void  parseSliceHeader         ( Slice* pcSlice, std::shared_ptr<PicHeader>& picHeader, const ParameterSetManager* parameterSetManager, const int prevTid0POC, bool& firstSliceInPic );

private:
  void  parseAlfAps              ( APS* pcAPS );
  void  parseLmcsAps             ( APS* pcAPS );
  void  parseScalingListAps      ( APS* pcAPS );
  void  parseVUI                 ( VUI* pcVUI, unsigned vuiPayloadSize );
  void  parseConstraintInfo      ( ConstraintInfo *cinfo );
  void  parseProfileTierLevel    ( ProfileTierLevel *ptl, bool profileTierPresentFlag, int maxNumSubLayersMinus1 );
  void  parseOlsHrdParameters    ( GeneralHrdParams* generalHrd, std::vector<OlsHrdParams>& olsHrd, uint32_t firstSubLayer, uint32_t tempLevelHigh );
  void  parseGeneralHrdParameters( GeneralHrdParams *generalHrd );
  template<typename HeaderT>
  void  parsePicOrSliceHeaderRPL ( HeaderT* header, const SPS* sps, const PPS* pps );
  void  parseRefPicList          ( ReferencePictureList* rpl, int rplIdx, const SPS* pcSPS );
  void  copyRefPicList           ( const SPS* pcSPS, const ReferencePictureList* source_rpl, ReferencePictureList* dest_rpl );

  void  checkAlfNaluTidAndPicTid ( const Slice* pcSlice, const PicHeader* picHeader, const ParameterSetManager* parameterSetManager);

  template<typename HeaderT>
  void  parsePredWeightTable     ( HeaderT* sh_or_ph, const SPS* sps, const PPS* pps, std::array<int, 2> numRefIdxActive = { 0, 0 } );  // numRefIdxActive is only needed, when parsing the HeaderT = Slice
  void  parseScalingList         ( ScalingList *scalingList, bool aps_chromaPresentFlag );
  void  decodeScalingList        ( ScalingList* scalingList, uint32_t id, uint32_t scalingListPredIdDelta, bool scalingListCopyModeFlag, bool scalingListPredModeFlag );
  void  alfFilterCoeffs          ( AlfSliceParam& alfSliceParam, const bool isChroma, const int altIdx );
  void  dpb_parameters           ( int maxSubLayersMinus1, bool subLayerInfoFlag, SPS *pcSPS );
  void  parseExtraPHBitsStruct   ( SPS *sps, int numBytes );
  void  parseExtraSHBitsStruct   ( SPS *sps, int numBytes );

  bool  xMoreRbspData();
};

}
