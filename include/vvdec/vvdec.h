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

#include "vvdec/vvdecDecl.h"

#include <list>
#include "vvdec/sei.h"

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
  VVC_CT_V = 2              ///< V component
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

typedef struct VVDEC_DECL Vui
{
  bool       m_aspectRatioInfoPresentFlag     = false;
  bool       m_aspectRatioConstantFlag        = false;
  bool       m_nonPackedFlag                  = false;
  bool       m_nonProjectedFlag               = false;
  int        m_aspectRatioIdc                 = 0;
  int        m_sarWidth                       = 0;
  int        m_sarHeight                      = 0;
  bool       m_colourDescriptionPresentFlag   = false;
  int        m_colourPrimaries                = 2;
  int        m_transferCharacteristics        = 2;
  int        m_matrixCoefficients             = 2;
  bool       m_progressiveSourceFlag          = false;
  bool       m_interlacedSourceFlag           = false;
  bool       m_chromaLocInfoPresentFlag       = false;
  int        m_chromaSampleLocTypeTopField    = 6;
  int        m_chromaSampleLocTypeBottomField = 6;
  int        m_chromaSampleLocType            = 6;
  bool       m_overscanInfoPresentFlag        = false;
  bool       m_overscanAppropriateFlag        = false;
  bool       m_videoSignalTypePresentFlag     = false;
  bool       m_videoFullRangeFlag             = false;
}Vui_t;


typedef struct VVDEC_DECL Hrd
{
  uint32_t m_numUnitsInTick                          = 0;
  uint32_t m_timeScale                               = 0;
  bool     m_generalNalHrdParamsPresentFlag          = 0;
  bool     m_generalVclHrdParamsPresentFlag          = 0;
  bool     m_generalSamePicTimingInAllOlsFlag        = false;
  uint32_t m_tickDivisor                             = 0;
  bool     m_generalDecodingUnitHrdParamsPresentFlag = false;
  uint32_t m_bitRateScale                            = 0;
  uint32_t m_cpbSizeScale                            = 0;
  uint32_t m_cpbSizeDuScale                          = 0;
  uint32_t m_hrdCpbCnt                               = 0;
}Hrd_t;

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

  Vui            *m_pcVui                               = NULL;                ///< if available, pointer to VUI (Video Usability Information)
  Hrd            *m_pcHrd                               = NULL;                ///< if available, pointer to HRD (Hypothetical Reference Decoder)
  std::list<sei*> m_cSeiMsgLst;

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
#if 1 //RPR_YUV_OUTPUT
  int m_iUpscaledOutput                =  0;
#endif
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
    \retval     const char* decoder information
  */
   const char* getDecoderInfo();


    /**
     This method returns the last occurred error as a string.
     \param      None
     \retval     const char* empty string for no error assigned
   */
   const char* getLastError() const;

   /**
    This method returns additional information about the last occurred error as a string (if availalbe).
    \param      None
    \retval     const char* empty string for no error assigned
  */
   const char* getLastAdditionalError() const;

   /**
     This method returns the decoder version number as a string.
     \param      None
     \retval     const char* returns the version number
   */
   static const char* getVersionNumber();

   /**
     This static function returns a string according to the passed parameter nRet.
     \param[in]  nRet return value code to translate
     \retval[ ]  const char* empty string for no error
   */
   static const char* getErrorMsg( int nRet );

   static NalType getNalUnitType            ( AccessUnit& rcAccessUnit );
   static const char* getNalUnitTypeAsString( NalType t );
   
   static bool isNalUnitSideData            ( NalType t );
   static bool isNalUnitSlice               ( NalType t );

private:
   VVDecImpl*  m_pcVVDecImpl = NULL;
};


} // namespace

