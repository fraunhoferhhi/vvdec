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

#pragma once

#include <string>
#include <list>
#include <vector>
#include <cstring>

#include "vvdec/vvdecDecl.h"

namespace vvdec {

/**
  \ingroup VVDecExternalInterfaces
  \enum Status
  The enum Status enumerates picture extra status information. The information is delivered within the AccessUnit struct and is
  related to the according picture.
*/
enum Status
{
  STATUS_NORMAL = 0,                      ///< normal
};

/**
  \ingroup VVDecExternalInterfaces
  \enum ErrorCodes
  The enum ErrorCodes enumerates error codes returned by the decoder.
*/
enum ErrorCodes
{
  VVDEC_OK                      = 0,        ///< success
  VVDEC_ERR_UNSPECIFIED         = -1,       ///< unspecified malfunction
  VVDEC_ERR_INITIALIZE          = -2,       ///< decoder not initialized or tried to initialize multiple times
  VVDEC_ERR_ALLOCATE            = -3,       ///< internal allocation error
  VVDEC_NOT_ENOUGH_MEM          = -5,       ///< allocated memory to small to receive decoded data. After allocating sufficient memory the failed call can be repeated.
  VVDEC_ERR_PARAMETER           = -7,       ///< inconsistent or invalid parameters
  VVDEC_ERR_NOT_SUPPORTED       = -10,      ///< unsupported request
  VVDEC_ERR_RESTART_REQUIRED    = -11,      ///< decoder requires restart
  VVDEC_ERR_CPU                 = -30,      ///< unsupported CPU SSE 4.1 needed
  VVDEC_TRY_AGAIN               = -40,      ///< decoder needs more input and cannot return a picture
  VVDEC_EOF                     = -50       ///< end of file
};

/**
  \ingroup VVDecExternalInterfaces
  \enum LogLevel
  The enum LogLevel enumerates supported log levels/verbosity.
*/
enum LogLevel
{
  LL_SILENT  = 0,
  LL_ERROR   = 1,
  LL_WARNING = 2,
  LL_INFO    = 3,
  LL_NOTICE  = 4,
  LL_VERBOSE = 5,
  LL_DETAILS = 6
};

/**
  \ingroup VVDecExternalInterfaces
  \enum LogLevel
  The enum LogLevel enumerates supported log levels/verbosity.
*/
enum SIMD_Extension
{
  SIMD_DEFAULT  = 0,
  SIMD_SCALAR   = 1,
  SIMD_SSE41    = 2,
  SIMD_SSE42    = 3,
  SIMD_AVX      = 4,
  SIMD_AVX2     = 5,
  SIMD_AVX512   = 6
};


/**
  \ingroup VVDecExternalInterfaces
  \enum ColorFormat
  The enum ColorFormat enumerates supported input color formats.
*/
enum ColorFormat
{
  VVC_CF_INVALID = -1,                   ///< invalid color format
  VVC_CF_YUV400_PLANAR = 0,              ///< YUV400 planar color format
  VVC_CF_YUV420_PLANAR = 1,              ///< YUV420 planar color format
  VVC_CF_YUV422_PLANAR = 2,              ///< YUV422 planar color format
  VVC_CF_YUV444_PLANAR = 3               ///< YUV444 planar color format
};

/**
  \ingroup VVDecExternalInterfaces
  The class InterlaceFormat enumerates several supported picture formats.
  The enumeration InterlaceFormat is following the definition of the syntax element pic_struct defined in HEVC standard.
*/
enum FrameFormat
{
  VVC_FF_INVALID     = -1,               ///< invalid interlace format
  VVC_FF_PROGRESSIVE = 0,                ///< progressive coding picture format
  VVC_FF_TOP_FIELD   = 1,                ///< top field picture
  VVC_FF_BOT_FIELD   = 2,                ///< bottom field picture
  VVC_FF_TOP_BOT     = 3,                ///< interlaced frame (top field first in display order)
  VVC_FF_BOT_TOP     = 4,                ///< interlaced frame (bottom field first in display order)
  VVC_FF_TOP_BOT_TOP = 5,                ///< NOT SUPPORTED (top field, bottom field, top field repeated)
  VVC_FF_BOT_TOP_BOT = 6,                ///< NOT SUPPORTED (bottom field, top field, bottom field repeated)
  VVC_FF_FRAME_DOUB  = 7,                ///< NOT SUPPORTED (frame doubling)
  VVC_FF_FRAME_TRIP  = 8,                ///< NOT SUPPORTED (frame tripling)
  VVC_FF_TOP_PW_PREV = 9,                ///< top field    (is paired with previous bottom field)
  VVC_FF_BOT_PW_PREV = 10,               ///< bottom field (is paired with previous top field)
  VVC_FF_TOP_PW_NEXT = 11,               ///< top field    (is paired with next bottom field)
  VVC_FF_BOT_PW_NEXT = 12,               ///< bottom field (is paired with next top field)
};

/**
  \ingroup VVDecExternalInterfaces
  The class SliceType enumerates several supported slice types.
*/
enum SliceType
{
  VVC_SLICETYPE_I = 0,
  VVC_SLICETYPE_P,
  VVC_SLICETYPE_B,
  VVC_SLICETYPE_UNKNOWN
};

enum NalType
{
  VVC_NAL_UNIT_CODED_SLICE_TRAIL = 0,   // 0
  VVC_NAL_UNIT_CODED_SLICE_STSA,        // 1
  VVC_NAL_UNIT_CODED_SLICE_RADL,        // 2
  VVC_NAL_UNIT_CODED_SLICE_RASL,        // 3

  VVC_NAL_UNIT_RESERVED_VCL_4,
  VVC_NAL_UNIT_RESERVED_VCL_5,
  VVC_NAL_UNIT_RESERVED_VCL_6,

  VVC_NAL_UNIT_CODED_SLICE_IDR_W_RADL,  // 7
  VVC_NAL_UNIT_CODED_SLICE_IDR_N_LP,    // 8
  VVC_NAL_UNIT_CODED_SLICE_CRA,         // 9
  VVC_NAL_UNIT_CODED_SLICE_GDR,         // 10

  VVC_NAL_UNIT_RESERVED_IRAP_VCL_11,
  VVC_NAL_UNIT_RESERVED_IRAP_VCL_12,

  VVC_NAL_UNIT_DCI,                     // 13
  VVC_NAL_UNIT_VPS,                     // 14
  VVC_NAL_UNIT_SPS,                     // 15
  VVC_NAL_UNIT_PPS,                     // 16
  VVC_NAL_UNIT_PREFIX_APS,              // 17
  VVC_NAL_UNIT_SUFFIX_APS,              // 18
  VVC_NAL_UNIT_PH,                      // 19
  VVC_NAL_UNIT_ACCESS_UNIT_DELIMITER,   // 20
  VVC_NAL_UNIT_EOS,                     // 21
  VVC_NAL_UNIT_EOB,                     // 22
  VVC_NAL_UNIT_PREFIX_SEI,              // 23
  VVC_NAL_UNIT_SUFFIX_SEI,              // 24
  VVC_NAL_UNIT_FD,                      // 25

  VVC_NAL_UNIT_RESERVED_NVCL_26,
  VVC_NAL_UNIT_RESERVED_NVCL_27,

  VVC_NAL_UNIT_UNSPECIFIED_28,
  VVC_NAL_UNIT_UNSPECIFIED_29,
  VVC_NAL_UNIT_UNSPECIFIED_30,
  VVC_NAL_UNIT_UNSPECIFIED_31,
  VVC_NAL_UNIT_INVALID
};


enum VvcTier
{
  VVC_TIER_MAIN = 0,
  VVC_TIER_HIGH = 1,
};

enum VvcLevel
{
  VVC_LEVEL_NONE = 0,
  VVC_LEVEL_1   = 16,
  VVC_LEVEL_2   = 32,
  VVC_LEVEL_2_1 = 35,
  VVC_LEVEL_3   = 48,
  VVC_LEVEL_3_1 = 51,
  VVC_LEVEL_4   = 64,
  VVC_LEVEL_4_1 = 67,
  VVC_LEVEL_5   = 80,
  VVC_LEVEL_5_1 = 83,
  VVC_LEVEL_5_2 = 86,
  VVC_LEVEL_6   = 96,
  VVC_LEVEL_6_1 = 99,
  VVC_LEVEL_6_2 = 102,
  VVC_LEVEL_15_5 = 255,
};

/**
  \ingroup VVDecExternalInterfaces
  The struct AccessUnit contains attributes that are assigned to the compressed output of the decoder for a specific input picture.
  The structure contains buffer and size information of the compressed payload as well as timing, access and debug information.
  The smallest output unit of HEVC decoders are NalUnits. A set of NalUnits that belong to the same access unit are delivered in a continuous bitstream,
  where the NalUnits are separated by three byte start codes.
  The Buffer to retrieve the compressed video chunks has to be allocated by the caller. The related attribute BufSize
*/

typedef struct VVDEC_DECL AccessUnit
{
  AccessUnit()                                 ///< Default constructor, sets member attributes to default values
  {}

  unsigned char*  m_pucBuffer  = NULL;         ///< pointer to buffer that retrieves the coded data,
  int             m_iBufSize   = 0;            ///< size of the allocated buffer in bytes
  int             m_iUsedSize  = 0;            ///< length of the coded data in bytes
  uint64_t        m_uiCts      = 0;            ///< composition time stamp in TicksPerSecond (see VVCDecoderParameter)
  uint64_t        m_uiDts      = 0;            ///< decoding time stamp in TicksPerSecond (see VVCDecoderParameter)
  bool            m_bCtsValid  = false;        ///< composition time stamp valid flag (true: valid, false: CTS not set)
  bool            m_bDtsValid  = false;        ///< decoding time stamp valid flag (true: valid, false: DTS not set)
  bool            m_bRAP       = false;        ///< random access point flag (true: AU is random access point, false: sequential access)
} AccessUnit_t;


enum ComponentType
{
  VVC_CT_Y = 0,              ///< Y component
  VVC_CT_U = 1,              ///< U component
  VVC_CT_V = 2,              ///< V component
  VVC_CT_MAX_COMPONET = 3
};

typedef struct VVDEC_DECL Component
{
  Component()                             ///< default constructor, sets member attributes to default values
  {}
  unsigned char*  m_pucBuffer        = NULL;     ///< base pointer, allocated picture buffer */
  uint32_t        m_uiByteOffset     = 0;        ///< offset to first sample in bytes
  uint32_t        m_uiBytesPerSample = 1;        ///< offset to first sample in bytes
  uint32_t        m_uiBitDepth       = 8;        ///< bit depth of component
  uint32_t        m_uiWidth          = 0;        ///< width of the plane
  uint32_t        m_uiHeight         = 0;        ///< height of the plane
  int32_t         m_iStride          = 0;        ///< stride (width + left margin + right margins) of plane in bytes

} Component_t;


class SEIMsg;
typedef std::list<SEIMsg*> SEIMsgLst;


/**
  \ingroup VVDecExternalInterfaces
  The struct PicAttributes - contains decoder side information
*/
typedef struct VVDEC_DECL PicExtendedAttributes
{
  PicExtendedAttributes()
  {}

  NalType        m_eNalType                             = VVC_NAL_UNIT_INVALID;   ///< nal unit type
  SliceType      m_eSliceType                           = VVC_SLICETYPE_UNKNOWN;  ///< slice type (I/P/B) */
  bool           m_bRefPic                              = false;              // reference picture
  int            m_iTemporalLayer                       = 0;                  ///< temporal layer
  uint64_t       m_uiPOC                                = 0;                  ///< picture order count

  unsigned int   m_uiBits                               = 0;                  ///< bits of the compr. image packet

  bool           m_bCbr                                = false;               ///< Cbr mode flag (true: cbr, false: vbr)
  unsigned int   m_uiCpbRemovalDelay                   = 0;                   ///< Coded picture buffer removal delay
  unsigned int   m_uiCpbRemovalOffset                  = 0;                   ///< Coded picture buffer removal offset
  unsigned int   m_uiIndicatedBitrate                  = 0;                   ///< indicated bitrate for cbr
  unsigned int   m_uiMaxProfileLevelBitrate            = 0;                   ///< max. possible bitrate for profile and leven

  int            m_iAlternativeTransferCharacteristics = 0;
  int            m_iColourPrimaries                    = 0;
  int            m_iTransferCharacteristics            = 0;
  int            m_iMatrixCoefficients                 = 0;
  //boost::optional< dpl::ImagePacket::HdrMasteringDisplayColorVolume > m_cHdrMasteringDisplayColorVolume;

  SEIMsgLst      m_cSEIMsgLst;

} PicExtendedAttributes_t;


/**
  \ingroup VVDecExternalInterfaces
  The struct InputPicture combines the struct PicBuffer and the optional PicAttributes class.
*/
typedef struct VVDEC_DECL Frame
{
  Frame()
  {}
  Component       m_cComponent[3];                      ///< component plane for yuv
  uint32_t        m_uiNumComponents  = 0;               ///< number of color components
  uint32_t        m_uiWidth          = 0;               ///< width of the luminance plane
  uint32_t        m_uiHeight         = 0;               ///< height of the luminance plane
  uint32_t        m_uiBitDepth       = 0;               ///< bit depth of input signal (8: depth 8 bit, 10: depth 10 bit  )
  FrameFormat     m_eFrameFormat     = VVC_FF_INVALID;  ///< interlace format (VVC_FF_PROGRESSIVE)
  ColorFormat     m_eColorFormat     = VVC_CF_INVALID;  ///< color format     (VVC_CF_YUV420_PLANAR)
  uint64_t        m_uiSequenceNumber = 0;               ///< sequence number of the picture
  uint64_t        m_uiCts            = 0;               ///< composition time stamp in TicksPerSecond (see HEVCDecoderParameter)
  bool            m_bCtsValid        = false;           ///< composition time stamp valid flag (true: valid, false: CTS not set)

  PicExtendedAttributes*  m_pcPicExtendedAttributes = NULL;    ///< pointer to PicAttribute that might be NULL, containing decoder side information
} Frame_t;

/**
  \ingroup VVDecExternalInterfaces
  The struct VVCDecoderParameter is a container for decoder configuration parameters. This struct is used for initialization of an blank decoder
  as well as for reconfiguration of an already initialized decoder. The struct is equipped with an default constructor that initializes all parameters
  to default values for ease of use and best performance. However, some of the parameters has to be set by the caller, which can not be guessed by the decoder.
*/
typedef struct VVDEC_DECL VVDecParameter
{
  VVDecParameter()           ///< default constructor, sets member attributes to default values
  {}
  int m_iThreads                       = -1;             ///< thread count        ( default: -1 )
  int m_iParseThreads                  = -1;             ///< parser thread count ( default: -1 )
  LogLevel m_eLogLevel                 = LL_WARNING;     ///< verbosity level
  bool m_bDecodedPictureHashSEIEnabled = false;          ///<  Control handling of decoded picture hash SEI messages, true: check hash in SEI messages if available in the bitstream, false: ignore SEI message
  SIMD_Extension m_eSIMD_Extension     = SIMD_DEFAULT;   ///< set specific simd optimization (default: max. availalbe)
} VVCDecoderParameter_t;


class VVDecImpl;

/**
  \ingroup VVDecExternalInterfaces
  The class VvDec provides the decoder user interface. The simplest way to use the decoder is to call init() to initialize an decoder instance with the
  the given VVCDecoderParameters. After initialization the decoding of the video is performed by using the decoder() method to hand over compressed packets (bitstream chunks) in decoding order
  and retrieve uncompressed pictures. The decoding can be end by calling flush() that causes the decoder to finish decoding of all pending packets.
  Finally calling uninit() releases all allocated resources held by the decoder internally.
*/
class VVDEC_DECL VVDec
{
public:
  /// Constructor
  VVDec();

  /// Destructor
  virtual ~VVDec();

public:
  /**
    This method initializes the decoder instance.
    This method is used to initially set up the decoder with the assigned decoder parameter struct.
    The method fails if the decoder is already initialized or if the assigned parameter struct
    does not pass the consistency check. Other possibilities for an unsuccessful call are missing decoder license, or an machine with
    insufficient CPU-capabilities.
    \param[in]  rcVVDecParameter const reference of VVDecParameter struct that holds initial decoder parameters.
    \retval     int  if non-zero an error occurred (see ErrorCodes), otherwise the return value indicates success VVDEC_OK
    \pre        The decoder must not be initialized.
  */
   int init( const VVDecParameter& rcVVDecParameter );

   /**
    This method resets the decoder instance.
    This method clears the decoder and releases all internally allocated memory.
    Calling uninit cancels all pending decoding calls. In order to finish pending pictures use the flush method.
    \param[in]  None
    \retval     int if non-zero an error occurred (see ErrorCodes), otherwise VVDEC_OK indicates success.
    \pre        None
  */
   int uninit();

   /**
    * \brief Returns initialization status
    */
   bool isInitialized();

  /**
    This method decodes a compressed image packet (bitstream).
    Compressed image packet are passed to the decoder in decoder order. A picture is returned by filling the assigned Picture struct.
    A picture is valid if the decoder call returns success and the Picture is not null.
    If the AccessUnit  m_iBufSize = 0, the decoder just returns a pending pictures chunk if available.
    \param[in]   rcAccessUnit reference to AccessUnit that retrieves compressed access units and side information, data are valid if UsedSize attribute is non-zero and the call was successful.
    \param[out]  ppcPicture pointer to pointer of Picture structure containing a uncompressed picture and meta information.
    \retval      int if non-zero an error occurred or more data is needed, otherwise the retval indicates success VVDEC_OK
    \pre         The decoder has to be initialized successfully.
  */
   int decode( AccessUnit& rcAccessUnit, Frame** ppcFrame );


   /**
     This method flushes the decoder.
     This call is used to get outstanding pictures after all compressed packets have been passed over into the decoder using the decode call.
     Using the flush method the decoder is signaled that there are no further compressed packets to decode.
     The caller should repeat the flush call until all pending pictures has been delivered to the caller, which is when the the function returns VVDEC_EOF or no picture.
     \param[out]  ppcPicture pointer to pointer of Picture structure containing a uncompressed picture and meta information.
     \retval     int if non-zero an error occurred, otherwise the retval indicates success VVDEC_OK
     \pre        The decoder has to be initialized.
   */
   int flush(  Frame** ppcFrame );

   /**
     This method unreference an picture and frees the memory.
     This call is used to free the memory of an picture which is not used anymore.
     \param[out] rcAccessUnit reference to AccessUnit
     \retval     int if non-zero an error occurred, otherwise the retval indicates success VVDEC_OK
     \pre        The decoder has to be initialized.
   */
   int objectUnref( Frame* pcFrame );

   /**
    This method returns the number of found errors if PictureHashSEI is enabled.
    \param[in]  None
    \retval     int if non-zero an error occurred, otherwise 0 indicates success.
    \pre        None
  */
   int getNumberOfErrorsPictureHashSEI();

   /**
    * \brief Provide clock timer functions
    */
   int clockStartTime();
   int clockEndTime();
   double clockGetTimeDiffMs( );


   /**
    This method returns decoder information
    \param      None
    \retval     std::string decoder information
  */
   const char* getDecoderInfo();


    /**
     This method returns the last occurred error as a string.
     \param      None
     \retval     std::string empty string for no error assigned
   */
   const char* getLastError() const;

   /**
    This method returns additional information about the last occurred error as a string (if availalbe).
    \param      None
    \retval     std::string empty string for no error assigned
  */
   const char* getLastAdditionalError() const;

   /**
     This method returns the decoder version number as a string.
     \param      None
     \retval     std::string returns the version number
   */
   static const char* getVersionNumber();

   /**
     This static function returns a string according to the passed parameter nRet.
     \param[in]  nRet return value code to translate
     \retval[ ]  std::string empty string for no error
   */
   static const char* getErrorMsg( int nRet );

   static NalType getNalUnitType        ( AccessUnit& rcAccessUnit );
   static const char* getNalUnitTypeAsString( NalType t );
   
   static bool isNalUnitSideData            ( NalType t );
   static bool isNalUnitSlice               ( NalType t );

private:
   VVDecImpl*  m_pcVVDecImpl = NULL;
};


class VVDEC_DECL SEIMsg
{
public:
  enum PayloadType
  {
    BUFFERING_PERIOD                     = 0,
    PICTURE_TIMING                       = 1,
    FILLER_PAYLOAD                       = 3,
    USER_DATA_REGISTERED_ITU_T_T35       = 4,
    USER_DATA_UNREGISTERED               = 5,
    FILM_GRAIN_CHARACTERISTICS           = 19,
    FRAME_PACKING                        = 45,
    PARAMETER_SETS_INCLUSION_INDICATION  = 129,
    DECODING_UNIT_INFO                   = 130,
    DECODED_PICTURE_HASH                 = 132,
    SCALABLE_NESTING                     = 133,
    MASTERING_DISPLAY_COLOUR_VOLUME      = 137,
    DEPENDENT_RAP_INDICATION             = 145,
    EQUIRECTANGULAR_PROJECTION           = 150,
    SPHERE_ROTATION                      = 154,
    REGION_WISE_PACKING                  = 155,
    OMNI_VIEWPORT                        = 156,
    GENERALIZED_CUBEMAP_PROJECTION       = 153,
    FRAME_FIELD_INFO                     = 168,
    SUBPICTURE_LEVEL_INFO                = 203,
    SAMPLE_ASPECT_RATIO_INFO             = 204,
    CONTENT_LIGHT_LEVEL_INFO             = 144,
    ALTERNATIVE_TRANSFER_CHARACTERISTICS = 147,
    AMBIENT_VIEWING_ENVIRONMENT          = 148,
    CONTENT_COLOUR_VOLUME                = 149
  };

  SEIMsg() {}
  virtual ~SEIMsg() {}

  static const char *getSEIMsgString(SEIMsg::PayloadType payloadType);

  //virtual PayloadType payloadType() const = 0;
};


class SEIEquirectangularProjection : public SEIMsg
{
public:
  PayloadType payloadType() const { return EQUIRECTANGULAR_PROJECTION; }

  SEIEquirectangularProjection()  {}
  virtual ~SEIEquirectangularProjection() {}

  bool    m_erpCancelFlag          = false;
  bool    m_erpPersistenceFlag     = false;
  bool    m_erpGuardBandFlag       = false;
  uint8_t m_erpGuardBandType       = 0;
  uint8_t m_erpLeftGuardBandWidth  = 0;
  uint8_t m_erpRightGuardBandWidth = 0;
};

class SEISphereRotation : public SEIMsg
{
public:
  PayloadType payloadType() const { return SPHERE_ROTATION; }

  SEISphereRotation()  {}
  virtual ~SEISphereRotation() {}

  bool  m_sphereRotationCancelFlag      = false;
  bool  m_sphereRotationPersistenceFlag = false;
  int   m_sphereRotationYaw             = 0;
  int   m_sphereRotationPitch           = 0;
  int   m_sphereRotationRoll            = 0;
};

class SEIOmniViewport : public SEIMsg
{
public:
  PayloadType payloadType() const { return OMNI_VIEWPORT; }

  SEIOmniViewport() {}
  virtual ~SEIOmniViewport() {}

  struct OmniViewport
  {
    int      azimuthCentre   = 0;
    int      elevationCentre = 0;
    int      tiltCentre      = 0;
    uint32_t horRange        = 0;
    uint32_t verRange        = 0;
  };

  uint32_t m_omniViewportId              = 0;
  bool     m_omniViewportCancelFlag      = false;
  bool     m_omniViewportPersistenceFlag = false;
  uint8_t  m_omniViewportCntMinus1       = 0;
  std::vector<OmniViewport> m_omniViewportRegions;
};

class SEIRegionWisePacking : public SEIMsg
{
public:
  PayloadType payloadType() const { return REGION_WISE_PACKING; }
  SEIRegionWisePacking() {}
  virtual ~SEIRegionWisePacking() {}
  bool                  m_rwpCancelFlag                  = false;
  bool                  m_rwpPersistenceFlag             = false;
  bool                  m_constituentPictureMatchingFlag = false;
  int                   m_numPackedRegions               = 0;
  int                   m_projPictureWidth               = 0;
  int                   m_projPictureHeight              = 0;
  int                   m_packedPictureWidth             = 0;
  int                   m_packedPictureHeight            = 0;
  std::vector<uint8_t>  m_rwpTransformType;
  std::vector<bool>     m_rwpGuardBandFlag;
  std::vector<uint32_t> m_projRegionWidth;
  std::vector<uint32_t> m_projRegionHeight;
  std::vector<uint32_t> m_rwpProjRegionTop;
  std::vector<uint32_t> m_projRegionLeft;
  std::vector<uint16_t> m_packedRegionWidth;
  std::vector<uint16_t> m_packedRegionHeight;
  std::vector<uint16_t> m_packedRegionTop;
  std::vector<uint16_t> m_packedRegionLeft;
  std::vector<uint8_t>  m_rwpLeftGuardBandWidth;
  std::vector<uint8_t>  m_rwpRightGuardBandWidth;
  std::vector<uint8_t>  m_rwpTopGuardBandHeight;
  std::vector<uint8_t>  m_rwpBottomGuardBandHeight;
  std::vector<bool>     m_rwpGuardBandNotUsedForPredFlag;
  std::vector<uint8_t>  m_rwpGuardBandType;
};

class SEIGeneralizedCubemapProjection : public SEIMsg
{
public:
  PayloadType payloadType() const { return GENERALIZED_CUBEMAP_PROJECTION; }

  SEIGeneralizedCubemapProjection()  {}
  virtual ~SEIGeneralizedCubemapProjection() {}

  bool                 m_gcmpCancelFlag                     = false;
  bool                 m_gcmpPersistenceFlag                = false;
  uint8_t              m_gcmpPackingType                    = 0;
  uint8_t              m_gcmpMappingFunctionType            = 0;
  std::vector<uint8_t> m_gcmpFaceIndex;
  std::vector<uint8_t> m_gcmpFaceRotation;
  std::vector<uint8_t> m_gcmpFunctionCoeffU;
  std::vector<bool>    m_gcmpFunctionUAffectedByVFlag;
  std::vector<uint8_t> m_gcmpFunctionCoeffV;
  std::vector<bool>    m_gcmpFunctionVAffectedByUFlag;
  bool                 m_gcmpGuardBandFlag                 = false;
  uint8_t              m_gcmpGuardBandType                 = 0;
  bool                 m_gcmpGuardBandBoundaryExteriorFlag = false;
  uint8_t              m_gcmpGuardBandSamplesMinus1        = 0;
};

class SEISampleAspectRatioInfo : public SEIMsg
{
public:
  PayloadType payloadType() const { return SAMPLE_ASPECT_RATIO_INFO; }
  SEISampleAspectRatioInfo() {}
  virtual ~SEISampleAspectRatioInfo() {}
  bool                  m_sariCancelFlag      = false;
  bool                  m_sariPersistenceFlag = false;
  int                   m_sariAspectRatioIdc  = 0;
  int                   m_sariSarWidth        = 0;
  int                   m_sariSarHeight       = 0;
};

static const uint32_t ISO_IEC_11578_LEN=16;

class SEIuserDataUnregistered : public SEIMsg
{
public:
  PayloadType payloadType() const { return USER_DATA_UNREGISTERED; }

  SEIuserDataUnregistered()
    {}

  virtual ~SEIuserDataUnregistered()
  {
    delete userData;
  }

  uint8_t  uuid_iso_iec_11578[ISO_IEC_11578_LEN];
  uint32_t userDataLength = 0;
  uint8_t *userData;
};

class SEIDecodedPictureHash : public SEIMsg
{

public:

  struct PictureHash
  {
    std::vector<uint8_t> hash;

    bool operator==(const PictureHash &other) const
    {
      if (other.hash.size() != hash.size())
      {
        return false;
      }
      for(uint32_t i=0; i<uint32_t(hash.size()); i++)
      {
        if (other.hash[i] != hash[i])
        {
          return false;
        }
      }
      return true;
    }

    bool operator!=(const PictureHash &other) const
    {
      return !(*this == other);
    }
  };

  enum HashType
  {
    HASHTYPE_MD5             = 0,
    HASHTYPE_CRC             = 1,
    HASHTYPE_CHECKSUM        = 2,
    HASHTYPE_NONE            = 3,
    NUMBER_OF_HASHTYPES      = 4
  };

public:
  PayloadType payloadType() const { return DECODED_PICTURE_HASH; }

  SEIDecodedPictureHash() {}
  virtual ~SEIDecodedPictureHash() {}

  HashType    method         = HASHTYPE_MD5;
  bool        singleCompFlag = false;
  PictureHash m_pictureHash;
};

class SEIDependentRAPIndication : public SEIMsg
{
public:
  PayloadType payloadType() const { return DEPENDENT_RAP_INDICATION; }
  SEIDependentRAPIndication() { }

  virtual ~SEIDependentRAPIndication() { }
};

class SEIBufferingPeriod : public SEIMsg
{
public:
  PayloadType payloadType() const { return BUFFERING_PERIOD; }
  void copyTo (SEIBufferingPeriod& target) const;

  SEIBufferingPeriod()
  {
    ::memset(m_initialCpbRemovalDelay , 0, sizeof(m_initialCpbRemovalDelay));
    ::memset(m_initialCpbRemovalOffset, 0, sizeof(m_initialCpbRemovalOffset));
    ::memset(m_cpbRemovalDelayDelta   , 0, sizeof(m_cpbRemovalDelayDelta));
    ::memset(m_dpbOutputTidOffset     , 0, sizeof(m_dpbOutputTidOffset));
  }
  virtual ~SEIBufferingPeriod() {}

  void      setDuCpbRemovalDelayIncrementLength( uint32_t value )        { m_duCpbRemovalDelayIncrementLength = value;        }
  uint32_t  getDuCpbRemovalDelayIncrementLength( ) const                 { return m_duCpbRemovalDelayIncrementLength;         }
  void      setDpbOutputDelayDuLength( uint32_t value )                  { m_dpbOutputDelayDuLength = value;                  }
  uint32_t  getDpbOutputDelayDuLength( ) const                           { return m_dpbOutputDelayDuLength;                   }

  bool     m_bpNalCpbParamsPresentFlag                 = false;
  bool     m_bpVclCpbParamsPresentFlag                 = false;
  uint32_t m_initialCpbRemovalDelayLength              = 0;
  uint32_t m_cpbRemovalDelayLength                     = 0;
  uint32_t m_dpbOutputDelayLength                      = 0;
  int      m_bpCpbCnt                                  = 0;
  uint32_t m_duCpbRemovalDelayIncrementLength          = 0;
  uint32_t m_dpbOutputDelayDuLength                    = 0;
  uint32_t m_initialCpbRemovalDelay         [7][32][2];
  uint32_t m_initialCpbRemovalOffset        [7][32][2];
  bool     m_concatenationFlag                         = false;
  uint32_t m_auCpbRemovalDelayDelta                    = 0;
  bool     m_cpbRemovalDelayDeltasPresentFlag          = false;
  int      m_numCpbRemovalDelayDeltas                  = 0;
  int      m_bpMaxSubLayers                            = 0;
  uint32_t m_cpbRemovalDelayDelta           [15];
  bool     m_bpDecodingUnitHrdParamsPresentFlag        = false;
  bool     m_decodingUnitCpbParamsInPicTimingSeiFlag   = false;
  bool     m_decodingUnitDpbDuParamsInPicTimingSeiFlag = false;
  bool     m_sublayerInitialCpbRemovalDelayPresentFlag = false;
  bool     m_additionalConcatenationInfoPresentFlag    = false;
  uint32_t m_maxInitialRemovalDelayForConcatenation    = 0;
  bool     m_sublayerDpbOutputOffsetsPresentFlag       = false;
  uint32_t m_dpbOutputTidOffset              [7];
  bool     m_altCpbParamsPresentFlag                   = false;
  bool     m_useAltCpbParamsFlag                       = false;
};

class SEIPictureTiming : public SEIMsg
{
public:
  PayloadType payloadType() const { return PICTURE_TIMING; }
  void copyTo (SEIPictureTiming& target) const;

  SEIPictureTiming()
  {
    ::memset(m_ptSubLayerDelaysPresentFlag, 0, sizeof(m_ptSubLayerDelaysPresentFlag));
    ::memset(m_duCommonCpbRemovalDelayMinus1, 0, sizeof(m_duCommonCpbRemovalDelayMinus1));
    ::memset(m_cpbRemovalDelayDeltaEnabledFlag, 0, sizeof(m_cpbRemovalDelayDeltaEnabledFlag));
    ::memset(m_cpbRemovalDelayDeltaIdx, 0, sizeof(m_cpbRemovalDelayDeltaIdx));
    ::memset(m_auCpbRemovalDelay, 0, sizeof(m_auCpbRemovalDelay));
  }
  virtual ~SEIPictureTiming()
  {
  }

  bool      m_ptSubLayerDelaysPresentFlag[7];
  bool      m_cpbRemovalDelayDeltaEnabledFlag[7];
  uint32_t  m_cpbRemovalDelayDeltaIdx[7];
  uint32_t  m_auCpbRemovalDelay[7];
  uint32_t  m_picDpbOutputDelay           = 0;
  uint32_t  m_picDpbOutputDuDelay         = 0;
  uint32_t  m_numDecodingUnitsMinus1      = 0;
  bool      m_duCommonCpbRemovalDelayFlag = false;
  uint32_t  m_duCommonCpbRemovalDelayMinus1[7];
  std::vector<uint32_t> m_numNalusInDuMinus1;
  std::vector<uint32_t> m_duCpbRemovalDelayMinus1;
  bool      m_cpbAltTimingInfoPresentFlag = false;
  std::vector<std::vector<uint32_t>> m_nalCpbAltInitialRemovalDelayDelta;
  std::vector<std::vector<uint32_t>> m_nalCpbAltInitialRemovalOffsetDelta;
  std::vector<uint32_t>              m_nalCpbDelayOffset;
  std::vector<uint32_t>              m_nalDpbDelayOffset;
  std::vector<std::vector<uint32_t>> m_vclCpbAltInitialRemovalDelayDelta;
  std::vector<std::vector<uint32_t>> m_vclCpbAltInitialRemovalOffsetDelta;
  std::vector<uint32_t>              m_vclCpbDelayOffset;
  std::vector<uint32_t>              m_vclDpbDelayOffset;
  int m_ptDisplayElementalPeriodsMinus1 = 0;
};

class SEIDecodingUnitInfo : public SEIMsg
{
public:
  PayloadType payloadType() const { return DECODING_UNIT_INFO; }

  SEIDecodingUnitInfo()
  {
    ::memset(m_duiSubLayerDelaysPresentFlag, 0, sizeof(m_duiSubLayerDelaysPresentFlag));
    ::memset(m_duSptCpbRemovalDelayIncrement, 0, sizeof(m_duSptCpbRemovalDelayIncrement));
  }
  virtual ~SEIDecodingUnitInfo() {}
  int  m_decodingUnitIdx = 0;
  bool m_duiSubLayerDelaysPresentFlag[7];
  int  m_duSptCpbRemovalDelayIncrement[7];
  bool m_dpbOutputDuDelayPresentFlag = false;
  int  m_picSptDpbOutputDuDelay = 0;
};

class SEIFrameFieldInfo : public SEIMsg
{
public:
  PayloadType payloadType() const { return FRAME_FIELD_INFO; }

  SEIFrameFieldInfo()
  {}
  virtual ~SEIFrameFieldInfo() {}

  bool m_fieldPicFlag                  = false;
  bool m_bottomFieldFlag               = false;
  bool m_pairingIndicatedFlag          = false;
  bool m_pairedWithNextFieldFlag       = false;
  bool m_displayFieldsFromFrameFlag    = false;
  bool m_topFieldFirstFlag             = false;
  int  m_displayElementalPeriodsMinus1 = 0;
  int  m_sourceScanType                = 0;
  bool m_duplicateFlag                 = false;
};

class SEIFramePacking : public SEIMsg
{
public:
  PayloadType payloadType() const { return FRAME_PACKING; }

  SEIFramePacking() {}
  virtual ~SEIFramePacking() {}

  int  m_arrangementId               = 0;
  bool m_arrangementCancelFlag       = false;
  int  m_arrangementType             = 0;
  bool m_quincunxSamplingFlag        = false;
  int  m_contentInterpretationType   = 0;
  bool m_spatialFlippingFlag         = false;
  bool m_frame0FlippedFlag           = false;
  bool m_fieldViewsFlag              = false;
  bool m_currentFrameIsFrame0Flag    = false;
  bool m_frame0SelfContainedFlag     = false;
  bool m_frame1SelfContainedFlag     = false;
  int  m_frame0GridPositionX         = 0;
  int  m_frame0GridPositionY         = 0;
  int  m_frame1GridPositionX         = 0;
  int  m_frame1GridPositionY         = 0;
  int  m_arrangementReservedByte     = 0;
  bool m_arrangementPersistenceFlag  = false;
  bool m_upsampledAspectRatio        = false;
};

class SEIParameterSetsInclusionIndication : public SEIMsg
{
public:
  PayloadType payloadType() const { return PARAMETER_SETS_INCLUSION_INDICATION; }
  SEIParameterSetsInclusionIndication() {}
  virtual ~SEIParameterSetsInclusionIndication() {}

  int m_selfContainedClvsFlag;
};

class SEIMasteringDisplayColourVolume : public SEIMsg
{
public:
    PayloadType payloadType() const { return MASTERING_DISPLAY_COLOUR_VOLUME; }
    SEIMasteringDisplayColourVolume()
    {
      ::memset(primaries  , 0, sizeof(primaries));
      ::memset(whitePoint , 0, sizeof(whitePoint));
    }
    virtual ~SEIMasteringDisplayColourVolume(){}

    bool     colourVolumeSEIEnabled = false;
    uint32_t maxLuminance           = 0;
    uint32_t minLuminance           = 0;
    uint16_t primaries[3][2];
    uint16_t whitePoint[2];
};


/// output a selection of SEI messages by payload type. Ownership stays in original message list.
SEIMsgLst getSeisByType(const SEIMsgLst &seiList, SEIMsg::PayloadType seiType);

/// remove a selection of SEI messages by payload type from the original list and return them in a new list.
SEIMsgLst extractSeisByType(SEIMsgLst &seiList, SEIMsg::PayloadType seiType);

/// delete list of SEI messages (freeing the referenced objects)
void deleteSEIs (SEIMsgLst &seiList);

class SEIScalableNesting : public SEIMsg
{
public:
  PayloadType payloadType() const { return SCALABLE_NESTING; }

  SEIScalableNesting()
  {
    ::memset(m_snOlsIdxDeltaMinus1 , 0, sizeof(m_snOlsIdxDeltaMinus1));
    ::memset(m_snOlsIdx            , 0, sizeof(m_snOlsIdx));
    ::memset(m_snLayerId           , 0, sizeof(m_snLayerId));
  }

  virtual ~SEIScalableNesting()
  {
    deleteSEIs(m_nestedSEIs);
  }

  bool      m_snOlsFlag         = false;
  bool      m_snSubpicFlag      = false;
  uint32_t  m_snNumOlssMinus1   = 0;
  uint32_t  m_snOlsIdxDeltaMinus1[64];
  uint32_t  m_snOlsIdx[64];
  bool      m_snAllLayersFlag   = false;  //value valid if m_nestingOlsFlag == 0
  uint32_t  m_snNumLayersMinus1 = 0;      //value valid if m_nestingOlsFlag == 0 and m_nestingAllLayersFlag == 0
  uint8_t   m_snLayerId[64];              //value valid if m_nestingOlsFlag == 0 and m_nestingAllLayersFlag == 0. This can e.g. be a static array of 64 uint8_t values
  uint32_t  m_snNumSubpics      = 0;
  uint8_t   m_snSubpicIdLen     = 0;
  std::vector<uint16_t> m_snSubpicId;
  uint32_t  m_snNumSEIs         = 0;

  SEIMsgLst m_nestedSEIs;
};


class SEIAlternativeTransferCharacteristics : public SEIMsg
{
public:
  PayloadType payloadType() const { return ALTERNATIVE_TRANSFER_CHARACTERISTICS; }

  SEIAlternativeTransferCharacteristics() : m_preferredTransferCharacteristics(18)
  { }

  virtual ~SEIAlternativeTransferCharacteristics() {}

  uint32_t m_preferredTransferCharacteristics = 0;
};

class SEIUserDataRegistered : public SEIMsg
{
public:
  PayloadType payloadType() const { return USER_DATA_REGISTERED_ITU_T_T35; }

  SEIUserDataRegistered() {}
  virtual ~SEIUserDataRegistered() {}

  uint16_t             m_ituCountryCode = 0;
  std::vector<uint8_t> m_userData;
};

class SEIFilmGrainCharacteristics : public SEIMsg
{
public:
  PayloadType payloadType() const { return FILM_GRAIN_CHARACTERISTICS; }

  SEIFilmGrainCharacteristics() {}
  virtual ~SEIFilmGrainCharacteristics() {}

  bool        m_filmGrainCharacteristicsCancelFlag   = false;
  uint8_t     m_filmGrainModelId                     = 0;
  bool        m_separateColourDescriptionPresentFlag = false;
  uint8_t     m_filmGrainBitDepthLumaMinus8          = 0;
  uint8_t     m_filmGrainBitDepthChromaMinus8        = 0;
  bool        m_filmGrainFullRangeFlag               = false;
  uint8_t     m_filmGrainColourPrimaries             = 0;
  uint8_t     m_filmGrainTransferCharacteristics     = 0;
  uint8_t     m_filmGrainMatrixCoeffs                = 0;
  uint8_t     m_blendingModeId                       = 0;
  uint8_t     m_log2ScaleFactor                      = 0;

  struct CompModelIntensityValues
  {
    uint8_t intensityIntervalLowerBound = 0;
    uint8_t intensityIntervalUpperBound = 0;
    std::vector<int> compModelValue;
  };

  struct CompModel
  {
    bool    presentFlag;
    uint8_t numModelValues;
    std::vector<CompModelIntensityValues> intensityValues;
  };

  CompModel m_compModel[VVC_CT_MAX_COMPONET];
  bool      m_filmGrainCharacteristicsPersistenceFlag = false;
};

class SEIContentLightLevelInfo : public SEIMsg
{
public:
  PayloadType payloadType() const { return CONTENT_LIGHT_LEVEL_INFO; }
  SEIContentLightLevelInfo() { }

  virtual ~SEIContentLightLevelInfo() { }

  uint32_t m_maxContentLightLevel    = 0;
  uint32_t m_maxPicAverageLightLevel = 0;
};

class SEIAmbientViewingEnvironment : public SEIMsg
{
public:
  PayloadType payloadType() const { return AMBIENT_VIEWING_ENVIRONMENT; }
  SEIAmbientViewingEnvironment() { }

  virtual ~SEIAmbientViewingEnvironment() { }

  uint32_t m_ambientIlluminance = 0;
  uint16_t m_ambientLightX      = 0;
  uint16_t m_ambientLightY      = 0;
};

class SEIContentColourVolume : public SEIMsg
{
public:
  PayloadType payloadType() const { return CONTENT_COLOUR_VOLUME; }
  SEIContentColourVolume()
  {
    ::memset(m_ccvPrimariesX , 0, sizeof(m_ccvPrimariesX));
    ::memset(m_ccvPrimariesY , 0, sizeof(m_ccvPrimariesY));
  }
  virtual ~SEIContentColourVolume() {}

  bool      m_ccvCancelFlag                   = false;
  bool      m_ccvPersistenceFlag              = false;
  bool      m_ccvPrimariesPresentFlag         = false;
  bool      m_ccvMinLuminanceValuePresentFlag = false;
  bool      m_ccvMaxLuminanceValuePresentFlag = false;
  bool      m_ccvAvgLuminanceValuePresentFlag = false;
  int       m_ccvPrimariesX[VVC_CT_MAX_COMPONET];
  int       m_ccvPrimariesY[VVC_CT_MAX_COMPONET];
  uint32_t  m_ccvMinLuminanceValue            = 0;
  uint32_t  m_ccvMaxLuminanceValue            = 0;
  uint32_t  m_ccvAvgLuminanceValue            = 0;
};


class SEISubpicureLevelInfo : public SEIMsg
{
public:
  PayloadType payloadType() const { return SUBPICTURE_LEVEL_INFO; }
  SEISubpicureLevelInfo()
  {}
  virtual ~SEISubpicureLevelInfo() {}

  int       m_numRefLevels                = 0;
  bool      m_explicitFractionPresentFlag = false;
  bool      m_cbrConstraintFlag           = false;
  int       m_numSubpics                  = 0;
  int       m_sliMaxSublayers             = 0;
  bool      m_sliSublayerInfoPresentFlag  = false;
  std::vector<std::vector<int>>              m_nonSubpicLayersFraction;
  std::vector<std::vector<VvcLevel>>         m_refLevelIdc;
  std::vector<std::vector<std::vector<int>>> m_refLevelFraction;
};

} // namespace

