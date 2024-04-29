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

#include "CommonLib/CommonDef.h"
#include "CommonLib/Picture.h"

#include "DecSlice.h"
#include "SEIread.h"
#include "NALread.h"
#include "HLSyntaxReader.h"

#include "CommonLib/ParameterSetManager.h"

#include <unordered_set>

namespace vvdec
{

class InputNALUnit;
class ThreadPool;
class DecLib;
class PicListManager;

class DecLibParser
{
private:
  NalUnitType m_associatedIRAPType[MAX_VPS_LAYERS];   ///< NAL unit type of the associated IRAP picture
  int         m_pocCRA            [MAX_VPS_LAYERS];   ///< POC number of the latest CRA picture
  int         m_pocRandomAccess      = MAX_INT;       ///< POC number of the random access point (the first IDR or CRA picture)
  int         m_decodingOrderCounter = 0;
  uint32_t    m_prevLayerID          = MAX_INT;

  int m_prevPOC                   = MAX_INT;
  int m_prevTid0POC               = 0;

  int  m_skippedPOC               = 0;
  bool m_warningMessageSkipPicture = false;
  bool m_prevSliceSkipped         = false;

  bool m_bFirstSliceInPicture     = true;
  bool m_bFirstSliceInSequence[MAX_VPS_LAYERS] = { true };
  bool m_bFirstSliceInBitstream   = true;

  int      m_lastPOCNoOutputPriorPics                       = -1;
  bool     m_isNoOutputPriorPics                            = false;
  bool     m_lastNoOutputBeforeRecoveryFlag[MAX_VPS_LAYERS] = { false };   // value of variable NoOutputBeforeRecoveryFlag of the assocated CRA/GDR pic
  int      m_gdrRecoveryPointPocVal        [MAX_VPS_LAYERS];
  bool     m_gdrRecovered                  [MAX_VPS_LAYERS] = { false };
  uint32_t m_uiSliceSegmentIdx    = 0;

  int m_nonVCLbits                = 0;

  Picture* m_pcParsePic           = nullptr;
  Slice*   m_apcSlicePilot        = nullptr;

  DCI*     m_dci                  = nullptr;

  struct AccessUnitPicInfo
  {
    NalUnitType m_nalUnitType; ///< nal_unit_type
    uint32_t    m_temporalId;  ///< temporal_id
    uint32_t    m_nuhLayerId;  ///< nuh_layer_id
    int         m_POC;
  };
  std::vector<AccessUnitPicInfo> m_accessUnitPicInfo;

  struct NalUnitInfo
  {
    NalUnitType m_nalUnitType; ///< nal_unit_type
    uint32_t    m_nuhLayerId;  ///< nuh_layer_id
    uint32_t    m_firstCTUinSlice; /// the first CTU in slice, specified with raster scan order ctu address
    int         m_POC;             /// the picture order
  };
  std::vector<NalUnitInfo>  m_nalUnitInfo[MAX_VPS_LAYERS];

  std::vector<NalUnitType>  m_pictureUnitNals;
  std::list<InputNALUnit>   m_pictureSeiNalus;
  std::list<InputNALUnit>   m_prefixSEINALUs;   /// Buffered up prefix SEI NAL Units.

  std::ostream*             m_pDecodedSEIOutputStream = nullptr;

  seiMessages               m_seiMessageList;       ///< List of SEI messages that have been received before the first slice and between slices,
                                                    ///< excluding prefix SEIs...

  HRD                       m_HRD;

  ThreadPool*               m_threadPool = nullptr;

  // functional classes
  HLSyntaxReader            m_HLSReader;
  SEIReader                 m_seiReader;
  DecSlice                  m_cSliceDecoder;

  DecLib&                   m_decLib;
  ParameterSetManager       m_parameterSetManager;   // storage for parameter sets
  PicListManager&           m_picListManager;

  std::shared_ptr<PicHeader> m_picHeader;            // picture header

  unsigned int              m_numDecThreads   = 0;
  unsigned int              m_parseFrameDelay = 0;
  PicList                   m_parseFrameList;
  PicList                   m_dpbReferencePics;   // this mirrors the reference pictures from the DPB but is only used for parsing
  std::unordered_set<int>   m_tmpSeenPocs;
  int                       m_maxPicReconSkip = 1;
  ErrHandlingFlags          m_errHandlingFlags = ERR_HANDLING_OFF;

  CUChunkCache              m_cuChunkCache;
  TUChunkCache              m_tuChunkCache;

public:
  DecLibParser( DecLib& decLib, PicListManager& picListManager ) : m_decLib( decLib ), m_picListManager( picListManager ) {}
  ~DecLibParser();

  void create                   ( ThreadPool* tp, int parserFrameDelay, int numReconInst, int numDecThreads, ErrHandlingFlags errHandlingFlags );
  void destroy                  ();

#if 0
  void fillMissingPicBuf        ( Picture* pcPic, bool copyClosest );
#endif

  bool     parse                ( InputNALUnit& nalu );
  Picture* getNextDecodablePicture();

  void checkNoOutputPriorPics   ();
  void setNoOutputPriorPicsFlag (bool val)              { m_isNoOutputPriorPics = val; }
  bool getNoOutputPriorPicsFlag () const                { return m_isNoOutputPriorPics; }

  void setDecodedSEIMessageOutputStream( std::ostream* pOpStream ) { m_pDecodedSEIOutputStream = pOpStream; }

  void checkAPSInPictureUnit    ();
  void resetPictureUnitNals     ()                      { m_pictureUnitNals.clear(); }

  const ParameterSetManager& getParameterSetManager() const { return m_parameterSetManager; }

private:
  bool xDecodeSliceHead( InputNALUnit& nalu );
  bool xDecodeSliceMain( InputNALUnit& nalu );

  void xActivateParameterSets( const int layerId );
  void prepareUnavailablePicture( bool isLost, const PPS* pps, int iUnavailablePoc, const int layerId, const bool longTermFlag, const int temporalId );
  void applyReferencePictureListBasedMarking( Slice* currSlice, const int layerId, const PPS& pps );

  void xParsePrefixSEImessages();
  void xParsePrefixSEIsForUnknownVCLNal();

  void xDecodePicHeader       ( InputNALUnit& nalu );
  void xDecodeVPS             ( InputNALUnit& nalu );
  void xDecodeDCI             ( InputNALUnit& nalu );
  void xDecodeSPS             ( InputNALUnit& nalu );
  void xDecodePPS             ( InputNALUnit& nalu );
  void xDecodeAPS             ( InputNALUnit& nalu );
  void xUpdatePreviousTid0POC ( Slice* pSlice );

  bool isRandomAccessSkipPicture();
  void xCheckMixedNalUnit     ( Slice* pcSlice, const InputNALUnit& nalu );

  void waitForPicsToFinishParsing( const std::vector<Picture*>& refPics );
};

}
