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

#ifndef DEC_LIB_PARSER_H
#define DEC_LIB_PARSER_H

#include "CommonLib/CommonDef.h"
#include "CommonLib/Picture.h"

#include "DecSlice.h"
#include "VLCReader.h"
#include "SEIread.h"

#include "CommonLib/ParameterSetManager.h"


class InputNALUnit;
class NoMallocThreadPool;
class DecLib;
class PicListManager;

class DecLibParser
{
private:
  NalUnitType m_associatedIRAPType = NAL_UNIT_INVALID;   ///< NAL unit type of the associated IRAP picture
  int         m_pocCRA             = 0;                  ///< POC number of the latest CRA picture
  int         m_pocRandomAccess    = MAX_INT;            ///< POC number of the random access point (the first IDR or CRA picture)
  int         m_lastRasPoc         = MAX_INT;
  int         m_associatedIRAPDecodingOrderNumber = 0; ///< Decoding order number of the associated IRAP picture
  int         m_decodingOrderCounter = 0;
  uint32_t    m_prevLayerID        = MAX_INT;

  int m_prevPOC     = MAX_INT;
  int m_prevTid0POC = 0;

  int  m_skippedPOC                = 0;
  bool m_warningMessageSkipPicture = false;
  bool m_prevSliceSkipped          = false;

  bool m_bFirstSliceInPicture   = true;
  bool m_bFirstSliceInSequence[MAX_VPS_LAYERS] = { true };
  bool m_bFirstSliceInBitstream = true;

  int  m_lastPOCNoOutputPriorPics = -1;
  bool m_isNoOutputPriorPics      = false;
  bool m_lastNoOutputBeforeRecoveryFlag[MAX_VPS_LAYERS] = { false };    //value of variable NoOutputBeforeRecoveryFlag of the assocated CRA/GDR pic
  uint32_t m_uiSliceSegmentIdx    = 0;

  int m_iTargetLayer = -1;   ///< target stream layer to be decoded

  Picture* m_pcParsePic    = nullptr;
  Slice*   m_apcSlicePilot = nullptr;

  DCI*     m_dci           = nullptr;
  std::list<InputNALUnit*> m_prefixSEINALUs;   /// Buffered up prefix SEI NAL Units.
#if JVET_P0101_POC_MULTILAYER
  struct AccessUnitPicInfo
  {
    NalUnitType     m_nalUnitType; ///< nal_unit_type
    uint32_t        m_temporalId;  ///< temporal_id
    uint32_t        m_nuhLayerId;  ///< nuh_layer_id
    int             m_POC;
  };
  std::vector<AccessUnitPicInfo> m_accessUnitPicInfo;
#endif
  struct NalUnitInfo
  {
    NalUnitType     m_nalUnitType; ///< nal_unit_type
    uint32_t        m_nuhLayerId;  ///< nuh_layer_id
    uint32_t        m_firstCTUinSlice; /// the first CTU in slice, specified with raster scan order ctu address
    int             m_POC;             /// the picture order
  };
  std::vector<NalUnitInfo> m_nalUnitInfo[MAX_VPS_LAYERS];
  
  std::vector<NalUnitType> m_pictureUnitNals;
  std::list<InputNALUnit*> m_pictureSeiNalus;

  std::ostream* m_pDecodedSEIOutputStream = nullptr;

  SEIMessages m_SEIs;   ///< List of SEI messages that have been received before the first slice and between slices,
                        ///< excluding prefix SEIs...
  HRD                     m_HRD;
  
  NoMallocThreadPool* m_threadPool = nullptr;

  // functional classes
  HLSyntaxReader m_HLSReader;
  SEIReader      m_seiReader;
  DecSlice       m_cSliceDecoder;

  DecLib&             m_decLib;
  ParameterSetManager m_parameterSetManager;   // storage for parameter sets
  PicListManager&     m_picListManager;

  PicHeader*          m_picHeader = nullptr;            // picture header

  unsigned int         m_parseFrameDelay = 0;
  std::deque<Picture*> m_parseFrameList;
  int                  m_maxPicReconSkip = 1;


public:
  DecLibParser( DecLib& decLib, PicListManager& picListManager,  PicHeader* picHeader ) : m_decLib( decLib ), m_picListManager( picListManager ), m_picHeader( picHeader ) {}
  virtual ~DecLibParser();

  void create( NoMallocThreadPool* tp, int parserFrameDelay, int numReconInst, int numDecThreads );
  void destroy();

  void recreateLostPicture( Picture* pcPic );

#if JVET_P0288_PIC_OUTPUT
  Picture* parse( InputNALUnit& nalu, int* pSkipFrame, int iTargetLayer = -1 );
#else
  Picture* parse( InputNALUnit& nalu, int* pSkipFrame );
#endif
  Picture* getNextDecodablePicture();

  void setFirstSliceInPicture( bool val )   { m_bFirstSliceInPicture = val; }
  bool  getFirstSliceInSequence(int layerId) const { return m_bFirstSliceInSequence[layerId]; }
  void  setFirstSliceInSequence(bool val, int layerId) { m_bFirstSliceInSequence[layerId] = val; }
  void checkNoOutputPriorPics();
  void setNoOutputPriorPicsFlag( bool val ) { m_isNoOutputPriorPics = val; }
  bool getNoOutputPriorPicsFlag() const     { return m_isNoOutputPriorPics; }

  void setDecodedSEIMessageOutputStream( std::ostream* pOpStream ) { m_pDecodedSEIOutputStream = pOpStream; }

  void setTargetDecLayer( int val ) { m_iTargetLayer = val; }
  int  getTargetDecLayer()          { return m_iTargetLayer; }

  void checkAPSInPictureUnit();
  void resetPictureUnitNals() { m_pictureUnitNals.clear(); }

  ParameterSetManager getParameterSetManager() { return m_parameterSetManager; }
  
private:
  enum SliceHeadResult { SkipPicture, NewPicture, ContinueParsing };
  SliceHeadResult xDecodeSliceHead( InputNALUnit& nalu, int* pSkipFrame );
  Slice*          xDecodeSliceMain( InputNALUnit& nalu );

  Picture*        xActivateParameterSets( const int layerId );
  Picture*        prepareLostPicture( int iLostPOC, const int layerId );
#if JVET_S0124_UNAVAILABLE_REFERENCE
  Picture*        prepareUnavailablePicture( const PPS *pps, int iUnavailablePoc, const int layerId, const bool longTermFlag, const int temporalId );
#else
  Picture*        prepareUnavailablePicture( int iUnavailablePoc, const int layerId, const bool longTermFlag );
#endif
  
  void xParsePrefixSEImessages();
  void xParsePrefixSEIsForUnknownVCLNal();

  void xDecodePicHeader( InputNALUnit& nalu );
  void xDecodeVPS( InputNALUnit& nalu );
  void xDecodeDCI( InputNALUnit& nalu );
  void xDecodeSPS( InputNALUnit& nalu );
  void xDecodePPS( InputNALUnit& nalu );
  void xDecodeAPS( InputNALUnit& nalu );
  void xUpdatePreviousTid0POC( Slice* pSlice );
  void xUpdateRasInit( Slice* slice );

  bool isRandomAccessSkipPicture();
  void xCheckMixedNalUnit( Slice* pcSlice, InputNALUnit &nalu );
};

#endif   // !DEC_LIB_PARSER_H
