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

#ifndef _VVDEC_H_
#define _VVDEC_H_

#include "vvdec/vvdecDecl.h"

#include <stdio.h>
#include <stdint.h>

#include "vvdec/sei.h"

#define VVDEC_NAMESPACE_BEGIN
#define VVDEC_NAMESPACE_END

#ifdef __cplusplus
extern "C" {
#endif

VVDEC_NAMESPACE_BEGIN


/* vvdec_decoder:
 *      opaque handler for the decoder */
typedef struct vvdec_decoder vvdec_decoder_t;


/**
  \ingroup VVDecExternalInterfaces
  \enum ErrorCodes
  The enum ErrorCodes enumerates error codes returned by the decoder.
*/
typedef enum
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
}ErrorCodes;

/**
  \ingroup VVDecExternalInterfaces
  \enum LogLevel
  The enum LogLevel enumerates supported log levels/verbosity.
*/
typedef enum
{
  VVDEC_SILENT  = 0,
  VVDEC_ERROR   = 1,
  VVDEC_WARNING = 2,
  VVDEC_INFO    = 3,
  VVDEC_NOTICE  = 4,
  VVDEC_VERBOSE = 5,
  VVDEC_DETAILS = 6
}LogLevel;

/**
  \ingroup VVDecExternalInterfaces
  \enum LogLevel
  The enum LogLevel enumerates supported log levels/verbosity.
*/
typedef enum
{
  VVDEC_SIMD_DEFAULT  = 0,
  VVDEC_SIMD_SCALAR   = 1,
  VVDEC_SIMD_SSE41    = 2,
  VVDEC_SIMD_SSE42    = 3,
  VVDEC_SIMD_AVX      = 4,
  VVDEC_SIMD_AVX2     = 5,
  VVDEC_SIMD_AVX512   = 6
}SIMD_Extension;


/**
  \ingroup VVDecExternalInterfaces
  \enum ColorFormat
  The enum ColorFormat enumerates supported input color formats.
*/
typedef enum
{
  VVDEC_CF_INVALID = -1,                   ///< invalid color format
  VVDEC_CF_YUV400_PLANAR = 0,              ///< YUV400 planar color format
  VVDEC_CF_YUV420_PLANAR = 1,              ///< YUV420 planar color format
  VVDEC_CF_YUV422_PLANAR = 2,              ///< YUV422 planar color format
  VVDEC_CF_YUV444_PLANAR = 3               ///< YUV444 planar color format
}ColorFormat;

/**
  \ingroup VVDecExternalInterfaces
  The class InterlaceFormat enumerates several supported picture formats.
  The enumeration InterlaceFormat is following the definition of the syntax element pic_struct defined in HEVC standard.
*/
typedef enum
{
  VVDEC_FF_INVALID     = -1,               ///< invalid interlace format
  VVDEC_FF_PROGRESSIVE = 0,                ///< progressive coding picture format
  VVDEC_FF_TOP_FIELD   = 1,                ///< top field picture
  VVDEC_FF_BOT_FIELD   = 2,                ///< bottom field picture
  VVDEC_FF_TOP_BOT     = 3,                ///< interlaced frame (top field first in display order)
  VVDEC_FF_BOT_TOP     = 4,                ///< interlaced frame (bottom field first in display order)
  VVDEC_FF_TOP_BOT_TOP = 5,                ///< NOT SUPPORTED (top field, bottom field, top field repeated)
  VVDEC_FF_BOT_TOP_BOT = 6,                ///< NOT SUPPORTED (bottom field, top field, bottom field repeated)
  VVDEC_FF_FRAME_DOUB  = 7,                ///< NOT SUPPORTED (frame doubling)
  VVDEC_FF_FRAME_TRIP  = 8,                ///< NOT SUPPORTED (frame tripling)
  VVDEC_FF_TOP_PW_PREV = 9,                ///< top field    (is paired with previous bottom field)
  VVDEC_FF_BOT_PW_PREV = 10,               ///< bottom field (is paired with previous top field)
  VVDEC_FF_TOP_PW_NEXT = 11,               ///< top field    (is paired with next bottom field)
  VVDEC_FF_BOT_PW_NEXT = 12,               ///< bottom field (is paired with next top field)
}FrameFormat;

/**
  \ingroup VVDecExternalInterfaces
  The class SliceType enumerates several supported slice types.
*/
typedef enum
{
  VVDEC_SLICETYPE_I = 0,
  VVDEC_SLICETYPE_P,
  VVDEC_SLICETYPE_B,
  VVDEC_SLICETYPE_UNKNOWN
}VVDEC_SliceType;

typedef enum
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
}NalType;


typedef enum
{
  VVDEC_CT_Y = 0,              ///< Y component
  VVDEC_CT_U = 1,              ///< U component
  VVDEC_CT_V = 2              ///< V component
}ComponentType;


/**
  \ingroup VVDecExternalInterfaces
  The struct AccessUnit contains attributes that are assigned to the compressed output of the decoder for a specific input picture.
  The structure contains buffer and size information of the compressed payload as well as timing, access and debug information.
  The smallest output unit of HEVC decoders are NalUnits. A set of NalUnits that belong to the same access unit are delivered in a continuous bitstream,
  where the NalUnits are separated by three byte start codes.
  The Buffer to retrieve the compressed video chunks has to be allocated by the caller. The related attribute BufSize
*/
typedef struct vvdec_AccessUnit
{
  unsigned char*  m_pucBuffer  = NULL;         ///< pointer to buffer that retrieves the coded data,
  int             m_iBufSize   = 0;            ///< size of the allocated buffer in bytes
  int             m_iUsedSize  = 0;            ///< length of the coded data in bytes
  uint64_t        m_uiCts      = 0;            ///< composition time stamp in TicksPerSecond (see VVCDecoderParameter)
  uint64_t        m_uiDts      = 0;            ///< decoding time stamp in TicksPerSecond (see VVCDecoderParameter)
  bool            m_bCtsValid  = false;        ///< composition time stamp valid flag (true: valid, false: CTS not set)
  bool            m_bDtsValid  = false;        ///< decoding time stamp valid flag (true: valid, false: DTS not set)
  bool            m_bRAP       = false;        ///< random access point flag (true: AU is random access point, false: sequential access)
} vvdec_AccessUnit_t;


typedef struct vvdec_Component
{
  unsigned char*  m_pucBuffer        = NULL;     ///< base pointer, allocated picture buffer */
  uint32_t        m_uiByteOffset     = 0;        ///< offset to first sample in bytes
  uint32_t        m_uiBytesPerSample = 1;        ///< offset to first sample in bytes
  uint32_t        m_uiBitDepth       = 8;        ///< bit depth of component
  uint32_t        m_uiWidth          = 0;        ///< width of the plane
  uint32_t        m_uiHeight         = 0;        ///< height of the plane
  int32_t         m_iStride          = 0;        ///< stride (width + left margin + right margins) of plane in bytes

} vvdec_Component_t;

typedef struct vvdec_Vui
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
}vvdec_Vui_t;

typedef struct vvdec_Hrd
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
}vvdec_Hrd_t;

/**
  \ingroup VVDecExternalInterfaces
  The struct PicAttributes - contains decoder side information
*/
typedef struct vvdec_PicExtendedAttributes
{
  NalType         m_eNalType          = VVC_NAL_UNIT_INVALID;     ///< nal unit type
  VVDEC_SliceType m_eSliceType        = VVDEC_SLICETYPE_UNKNOWN;  ///< slice type (I/P/B) */
  bool            m_bRefPic           = false;              // reference picture
  int             m_iTemporalLayer    = 0;                  ///< temporal layer
  uint64_t        m_uiPOC             = 0;                  ///< picture order count

  unsigned int    m_uiBits            = 0;                  ///< bits of the compr. image packet

  vvdec_Vui      *m_pcVui             = NULL;                ///< if available, pointer to VUI (Video Usability Information)
  vvdec_Hrd      *m_pcHrd             = NULL;                ///< if available, pointer to HRD (Hypothetical Reference Decoder)
//  std::list<sei*> m_cSeiMsgLst;

} vvdec_PicExtendedAttributes_t;


/**
  \ingroup VVDecExternalInterfaces
  The struct InputPicture combines the struct PicBuffer and the optional PicAttributes class.
*/
typedef struct vvdec_Frame
{
  vvdec_Component   m_cComponent[3];                      ///< component plane for yuv
  uint32_t          m_uiNumComponents  = 0;               ///< number of color components
  uint32_t          m_uiWidth          = 0;               ///< width of the luminance plane
  uint32_t          m_uiHeight         = 0;               ///< height of the luminance plane
  uint32_t          m_uiBitDepth       = 0;               ///< bit depth of input signal (8: depth 8 bit, 10: depth 10 bit  )
  FrameFormat       m_eFrameFormat     = VVDEC_FF_INVALID;  ///< interlace format (VVC_FF_PROGRESSIVE)
  ColorFormat       m_eColorFormat     = VVDEC_CF_INVALID;  ///< color format     (VVC_CF_YUV420_PLANAR)
  uint64_t          m_uiSequenceNumber = 0;               ///< sequence number of the picture
  uint64_t          m_uiCts            = 0;               ///< composition time stamp in TicksPerSecond (see HEVCDecoderParameter)
  bool              m_bCtsValid        = false;           ///< composition time stamp valid flag (true: valid, false: CTS not set)

  vvdec_PicExtendedAttributes*  m_pcPicExtendedAttributes = NULL;    ///< pointer to PicAttribute that might be NULL, containing decoder side information
} vvdec_Frame_t;

/**
  \ingroup VVDecExternalInterfaces
  The struct VVCDecoderParameter is a container for decoder configuration parameters. This struct is used for initialization of an blank decoder
  as well as for reconfiguration of an already initialized decoder. The struct is equipped with an default constructor that initializes all parameters
  to default values for ease of use and best performance. However, some of the parameters has to be set by the caller, which can not be guessed by the decoder.
*/
typedef struct vvdec_Params
{
  int m_iThreads                       = -1;             ///< thread count        ( default: -1 )
  int m_iParseThreads                  = -1;             ///< parser thread count ( default: -1 )
#if 1 //RPR_YUV_OUTPUT
  int m_iUpscaledOutput                =  0;
#endif
  LogLevel m_eLogLevel                 = VVDEC_WARNING;     ///< verbosity level
  bool m_bDecodedPictureHashSEIEnabled = false;                ///<  Control handling of decoded picture hash SEI messages, true: check hash in SEI messages if available in the bitstream, false: ignore SEI message
  SIMD_Extension m_eSIMD_Extension     = VVDEC_SIMD_DEFAULT;   ///< set specific simd optimization (default: max. availalbe)
} vvdec_Params_t;

typedef void (*vvdec_loggingCallback)(void*, int, const char*, va_list);

/**
  This method returns the the decoder version as string.
  \param[in]  none
  \retval[ ]  const char* version number as string
*/
VVDEC_DECL const char* vvdec_get_version();

/**
  This method initializes the decoder instance.
  This method is used to initially set up the decoder with the assigned decoder parameter struct.
  The method fails if the assigned parameter struct does not pass the consistency check.
  Other possibilities for an unsuccessful memory initialization, or an machine with
  insufficient CPU-capabilities.
  \param[in]  vvdec_Params_t pointer of vvdec_Params struct that holds initial decoder parameters.
  \retval     vvdec_Params_t pointer of the decoder handler if successful, otherwise NULL
  \pre        The decoder must not be initialized (pointer of decoder handler must be null).
*/
VVDEC_DECL vvdec_decoder_t* vvdec_decoder_open( vvdec_Params_t *);

/**
 This method resets the decoder instance.
 This method clears the decoder and releases all internally allocated memory.
 Calling uninit cancels all pending decoding calls. In order to finish pending pictures use the flush method.
 \param[in]  vvdec_decoder_t pointer of decoder handler
 \retval     int if non-zero an error occurred (see ErrorCodes), otherwise VVDEC_OK indicates success.
 \pre        The decoder has to be initialized successfully.
*/
VVDEC_DECL int vvdec_decoder_close(vvdec_decoder_t *);

/**
 *  Set a logging callback with the given log level.
  \param[in]   vvdec_decoder_t pointer of decoder handler
  \param[in]   vvdec_loggingCallback implementation of the callback that is called when logging messages are written
  \param[in]   userData
  \param[in]   LogLevel logging message level (only logs of level >= given level are written
  \retval      int if non-zero an error occurred (see ErrorCodes), otherwise VVDEC_OK indicates success.
  \pre         The decoder has to be initialized successfully.
 */
VVDEC_DECL int vvdec_set_logging_callback(vvdec_decoder_t* , vvdec_loggingCallback callback, void *userData, LogLevel logLevel);

/**
  This method decodes a compressed image packet (bitstream).
  Compressed image packet are passed to the decoder in decoder order. A picture is returned by filling the assigned Picture struct.
  A picture is valid if the decoder call returns success and the Picture is not null.
  If the AccessUnit  m_iBufSize = 0, the decoder just returns a pending pictures chunk if available.
  \param[in]   vvdec_decoder_t pointer of decoder handler
  \param[in]   vvdec_AccessUnit_t pointer of AccessUnit that retrieves compressed access units and side information, data are valid if UsedSize attribute is non-zero and the call was successful.
  \param[out]  ppcPicture pointer to pointer of Picture structure containing a uncompressed picture and meta information.
  \retval      int if non-zero an error occurred or more data is needed, otherwise the retval indicates success VVDEC_OK
  \pre         The decoder has to be initialized successfully.
*/
VVDEC_DECL int vvdec_decode( vvdec_decoder_t *, vvdec_AccessUnit_t *accessUnit, vvdec_Frame_t **frame );

/**
  This method flushes the decoder.
  This call is used to get outstanding pictures after all compressed packets have been passed over into the decoder using the decode call.
  Using the flush method the decoder is signaled that there are no further compressed packets to decode.
  The caller should repeat the flush call until all pending pictures has been delivered to the caller, which is when the the function returns VVDEC_EOF or no picture.
  \param[in]  vvdec_decoder_t pointer to decoder handler
  \param[out]  ppcPicture pointer to pointer of Picture structure containing a uncompressed picture and meta information.
  \retval     int if non-zero an error occurred, otherwise the retval indicates success VVDEC_OK
  \pre        The decoder has to be initialized successfully.
*/
VVDEC_DECL int vvdec_flush( vvdec_decoder_t *, vvdec_Frame_t **frame );

/**
  This method finds SEI message for given picture
  \param[in]  vvdec_decoder_t pointer of decoder handler
  \param[out] sei_message_t pointer to found sei message, NULL if not found
  \retval     int if non-zero an error occurred, otherwise the retval indicates success VVDEC_OK
  \pre        The decoder has to be initialized successfully.
*/
VVDEC_DECL vvdec_sei_message_t* vvdec_find_frame_sei( vvdec_decoder_t *, SEIPayloadType seiPayloadType, vvdec_Frame_t *frame );

/**
  This method unreference an picture and frees the memory.
  This call is used to free the memory of an picture which is not used anymore.
  \param[in]  vvdec_decoder_t pointer of decoder handler
  \param[out] vvdec_Frame_t pointer of frame
  \retval     int if non-zero an error occurred, otherwise the retval indicates success VVDEC_OK
  \pre        The decoder has to be initialized successfully.
*/
VVDEC_DECL int vvdec_frame_unref( vvdec_decoder_t *, vvdec_Frame_t *frame );

/**
 This method returns the number of found errors if PictureHashSEI is enabled.
 \param[in]  vvdec_decoder_t pointer of decoder handler
 \retval     int if non-zero an error occurred, otherwise 0 indicates success.
 \pre        The decoder has to be initialized successfully.
*/
VVDEC_DECL int vvdec_get_number_of_errors_PicHashSEI( vvdec_decoder_t * );


/**
 This method returns decoder information
 \param[in]  vvdec_decoder_t pointer of decoder handler
 \retval     const char* decoder information
 \pre        The decoder has to be initialized successfully.
*/
VVDEC_DECL const char* vvdec_getDecoderInfo( vvdec_decoder_t * );


/**
 This method returns the last occurred error as a string.
 \param[in]  vvdec_decoder_t pointer of decoder handler
 \retval     const char* empty string for no error assigned
 \pre        The decoder has to be initialized successfully.
*/
VVDEC_DECL const char* vvdec_getLastError( vvdec_decoder_t * );

/**
 This method returns additional information about the last occurred error as a string (if availalbe).
 \param[in]  vvdec_decoder_t pointer of decoder handler
 \retval     const char* empty string for no error assigned
 \pre        The decoder has to be initialized successfully.
*/
VVDEC_DECL const char* vvdec_getLastAdditionalError( vvdec_decoder_t * );

/**
 This function returns a string according to the passed parameter nRet.
 \param[in]  nRet return value code (see ErrorCodes) to translate
 \retval[ ]  const char* empty string for no error
*/
VVDEC_DECL const char* vvdec_getErrorMsg( int nRet );

/**
 This function returns the NalType of a given AccessUnit.
 \param[in]  vvdec_AccessUnit_t pointer of AccessUnit that retrieves compressed access units and side information, data are valid if UsedSize attribute is non-zero and the call was successful.
 \retval[ ]  NalType found Nal Unit type
*/
VVDEC_DECL NalType vvdec_getNalUnitType ( vvdec_AccessUnit *rcAccessUnit );

/**
 This function returns the name of a given NalType
 \param[in]  NalType value of enum NalType
 \retval[ ]  const char* NalType as string
*/
VVDEC_DECL const char* vvdec_getNalUnitTypeAsString( NalType t );

/**
 This function returns true if a given NalType is of type picture or slice
 \param[in]  NalType value of enum NalType
 \retval[ ]  bool true if slice/picture, else false
*/
VVDEC_DECL bool vvdec_isNalUnitSlice ( NalType t );


#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /*_VVDEC_H_*/
