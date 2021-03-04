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

Copyright (c) 2018-2021, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. 
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

#ifndef DEC_LIB_PARSER_H
#define DEC_LIB_PARSER_H

#include "CommonLib/CommonDef.h"
#include "CommonLib/Picture.h"

#include "DecSlice.h"
#include "VLCReader.h"
#include "SEIread.h"
#include "NALread.h"

#include "CommonLib/ParameterSetManager.h"


class InputNALUnit;
class NoMallocThreadPool;
class DecLib;
class PicListManager;

class DecLibParser
{
private:
  NalUnitType m_associatedIRAPType = NAL_UNIT_INVALID;   ///< NAL unit type of the associated IRAP picture
  int         m_pocCRA            = 0;                  ///< POC number of the latest CRA picture
  int         m_pocRandomAccess   = MAX_INT;            ///< POC number of the random access point (the first IDR or CRA picture)
  int         m_lastRasPoc        = MAX_INT;
  int         m_associatedIRAPDecodingOrderNumber = 0; ///< Decoding order number of the associated IRAP picture
  int         m_decodingOrderCounter = 0;
  uint32_t    m_prevLayerID       = MAX_INT;

  bool        m_prevPicSkipped                = true;
  bool        m_gdrRecoveryPeriod             = false;
  int         m_prevGDRInSameLayerPOC         = 0;    ///< POC number of the latest GDR picture
  int         m_prevGDRInSameLayerRecoveryPOC = 0;    ///< Recovery POC number of the latest GDR picture
  
  int m_prevPOC                   = MAX_INT;
  int m_prevTid0POC               = 0;

  int  m_skippedPOC               = 0;
  bool m_warningMessageSkipPicture = false;
  bool m_prevSliceSkipped         = false;

  bool m_bFirstSliceInPicture     = true;
  bool m_bFirstSliceInSequence[MAX_VPS_LAYERS] = { true };
  bool m_bFirstSliceInBitstream   = true;
  bool m_parseNewPicture          = false;

  int  m_lastPOCNoOutputPriorPics = -1;
  bool m_isNoOutputPriorPics      = false;
  bool m_lastNoOutputBeforeRecoveryFlag[MAX_VPS_LAYERS] = { false };    //value of variable NoOutputBeforeRecoveryFlag of the assocated CRA/GDR pic
  uint32_t m_uiSliceSegmentIdx    = 0;

  int m_iTargetLayer              = -1;   ///< target stream layer to be decoded

  Picture* m_pcParsePic           = nullptr;
  Slice*   m_apcSlicePilot        = nullptr;

  DCI*     m_dci                  = nullptr;
  std::list<InputNALUnit>  m_prefixSEINALUs;   /// Buffered up prefix SEI NAL Units.

#if JVET_P0101_POC_MULTILAYER
  struct AccessUnitPicInfo
  {
    NalUnitType m_nalUnitType; ///< nal_unit_type
    uint32_t    m_temporalId;  ///< temporal_id
    uint32_t    m_nuhLayerId;  ///< nuh_layer_id
    int         m_POC;
  };
  std::vector<AccessUnitPicInfo> m_accessUnitPicInfo;
#endif
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

  std::ostream*             m_pDecodedSEIOutputStream = nullptr;

  seiMessages               m_seiMessageList;       ///< List of SEI messages that have been received before the first slice and between slices,
                                                    ///< excluding prefix SEIs...

  HRD                       m_HRD;
  
  NoMallocThreadPool*       m_threadPool = nullptr;

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
  std::deque<Picture*>      m_parseFrameList;
  int                       m_maxPicReconSkip = 1;


public:
  DecLibParser( DecLib& decLib, PicListManager& picListManager,  PicHeader* picHeader ) : m_decLib( decLib ), m_picListManager( picListManager ), m_picHeader( picHeader ) {}
  ~DecLibParser();

  void create                   (NoMallocThreadPool* tp, int parserFrameDelay, int numReconInst, int numDecThreads);
  void destroy                  ();

  void recreateLostPicture      (Picture* pcPic);

#if JVET_P0288_PIC_OUTPUT
  Picture* parse                (InputNALUnit& nalu, int* pSkipFrame, int iTargetLayer = -1);
#else
  Picture* parse                (InputNALUnit& nalu, int* pSkipFrame);
#endif
  Picture* getNextDecodablePicture();

  void setFirstSliceInPicture   (bool val)              { m_bFirstSliceInPicture = val; }
  bool getFirstSliceInSequence  (int layerId) const     { return m_bFirstSliceInSequence[layerId]; }
  void setFirstSliceInSequence  (bool val, int layerId) { m_bFirstSliceInSequence[layerId] = val; }
  void checkNoOutputPriorPics   ();
  void setNoOutputPriorPicsFlag (bool val)              { m_isNoOutputPriorPics = val; }
  bool getNoOutputPriorPicsFlag () const                { return m_isNoOutputPriorPics; }
  bool getParseNewPicture       () const                { return m_parseNewPicture; }

  void setDecodedSEIMessageOutputStream( std::ostream* pOpStream ) { m_pDecodedSEIOutputStream = pOpStream; }

  void setTargetDecLayer        (int val)               { m_iTargetLayer = val; }
  int  getTargetDecLayer        ()                      { return m_iTargetLayer; }

  void checkAPSInPictureUnit    ();
  void resetPictureUnitNals     ()                      { m_pictureUnitNals.clear(); }

  ParameterSetManager getParameterSetManager()          { return m_parameterSetManager; }
  
  bool getPrevPicSkipped()                              { return m_prevPicSkipped; }
  void setPrevPicSkipped( bool val )                    { m_prevPicSkipped = val; }
  bool getGdrRecoveryPeriod()                           { return m_gdrRecoveryPeriod; }
  void setGdrRecoveryPeriod( bool val )                 { m_gdrRecoveryPeriod = val; }
  
private:
  enum SliceHeadResult { SkipPicture, NewPicture, ContinueParsing };
  SliceHeadResult xDecodeSliceHead( InputNALUnit& nalu, int* pSkipFrame );
  Slice*          xDecodeSliceMain( InputNALUnit& nalu );

  Picture*        xActivateParameterSets   ( const int layerId );
  void            prepareLostPicture       ( int iLostPOC, const int layerId );
#if JVET_S0124_UNAVAILABLE_REFERENCE
  void            prepareUnavailablePicture( const PPS *pps, int iUnavailablePoc, const int layerId, const bool longTermFlag, const int temporalId );
#else
  Picture*        prepareUnavailablePicture( int iUnavailablePoc, const int layerId, const bool longTermFlag );
#endif
  
  void xParsePrefixSEImessages();
  void xParsePrefixSEIsForUnknownVCLNal();

  void xDecodePicHeader       ( InputNALUnit& nalu );
  void xDecodeVPS             ( InputNALUnit& nalu );
  void xDecodeDCI             ( InputNALUnit& nalu );
  void xDecodeSPS             ( InputNALUnit& nalu );
  void xDecodePPS             ( InputNALUnit& nalu );
  void xDecodeAPS             ( InputNALUnit& nalu );
  void xUpdatePreviousTid0POC ( Slice* pSlice );
  void xUpdateRasInit         ( Slice* slice );

  bool isRandomAccessSkipPicture();
  void xCheckMixedNalUnit     ( Slice* pcSlice, InputNALUnit &nalu );
};

#endif   // !DEC_LIB_PARSER_H
