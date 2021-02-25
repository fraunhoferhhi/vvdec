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
#include <stdbool.h>

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

/* vvdec_loggingCallback:
   callback function to receive messages of the decoder library
*/
typedef void (*vvdec_loggingCallback)(void*, int, const char*, va_list);

/*
  \enum ErrorCodes
  The enum ErrorCodes enumerates error codes returned by the decoder.
*/
typedef enum
{
  VVDEC_OK                      = 0,        // success
  VVDEC_ERR_UNSPECIFIED         = -1,       // unspecified malfunction
  VVDEC_ERR_INITIALIZE          = -2,       // decoder not initialized or tried to initialize multiple times
  VVDEC_ERR_ALLOCATE            = -3,       // internal allocation error
  VVDEC_NOT_ENOUGH_MEM          = -5,       // allocated memory to small to receive decoded data. After allocating sufficient memory the failed call can be repeated.
  VVDEC_ERR_PARAMETER           = -7,       // inconsistent or invalid parameters
  VVDEC_ERR_NOT_SUPPORTED       = -10,      // unsupported request
  VVDEC_ERR_RESTART_REQUIRED    = -11,      // decoder requires restart
  VVDEC_ERR_CPU                 = -30,      // unsupported CPU SSE 4.1 needed
  VVDEC_TRY_AGAIN               = -40,      // decoder needs more input and cannot return a picture
  VVDEC_EOF                     = -50       // end of file
}ErrorCodes;

/*
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

/*
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


/*
  \enum ColorFormat
  The enum ColorFormat enumerates supported input color formats.
*/
typedef enum
{
  VVDEC_CF_INVALID = -1,                   // invalid color format
  VVDEC_CF_YUV400_PLANAR = 0,              // YUV400 planar color format
  VVDEC_CF_YUV420_PLANAR = 1,              // YUV420 planar color format
  VVDEC_CF_YUV422_PLANAR = 2,              // YUV422 planar color format
  VVDEC_CF_YUV444_PLANAR = 3               // YUV444 planar color format
}ColorFormat;

/*
  The class InterlaceFormat enumerates several supported picture formats.
  The enumeration InterlaceFormat is following the definition of the syntax element pic_struct defined in HEVC standard.
*/
typedef enum
{
  VVDEC_FF_INVALID     = -1,               // invalid interlace format
  VVDEC_FF_PROGRESSIVE = 0,                // progressive coding picture format
  VVDEC_FF_TOP_FIELD   = 1,                // top field picture
  VVDEC_FF_BOT_FIELD   = 2,                // bottom field picture
  VVDEC_FF_TOP_BOT     = 3,                // interlaced frame (top field first in display order)
  VVDEC_FF_BOT_TOP     = 4,                // interlaced frame (bottom field first in display order)
  VVDEC_FF_TOP_BOT_TOP = 5,                // NOT SUPPORTED (top field, bottom field, top field repeated)
  VVDEC_FF_BOT_TOP_BOT = 6,                // NOT SUPPORTED (bottom field, top field, bottom field repeated)
  VVDEC_FF_FRAME_DOUB  = 7,                // NOT SUPPORTED (frame doubling)
  VVDEC_FF_FRAME_TRIP  = 8,                // NOT SUPPORTED (frame tripling)
  VVDEC_FF_TOP_PW_PREV = 9,                // top field    (is paired with previous bottom field)
  VVDEC_FF_BOT_PW_PREV = 10,               // bottom field (is paired with previous top field)
  VVDEC_FF_TOP_PW_NEXT = 11,               // top field    (is paired with next bottom field)
  VVDEC_FF_BOT_PW_NEXT = 12,               // bottom field (is paired with next top field)
}FrameFormat;

/*
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
  VVDEC_CT_Y = 0,                  // Y component
  VVDEC_CT_U = 1,                  // U component
  VVDEC_CT_V = 2                   // V component
}ComponentType;


/*
  The struct AccessUnit contains attributes that are assigned to the compressed output of the decoder for a specific input picture.
  The structure contains buffer and size information of the compressed payload as well as timing, access and debug information.
  The smallest output unit of HEVC decoders are NalUnits. A set of NalUnits that belong to the same access unit are delivered in a continuous bitstream,
  where the NalUnits are separated by three byte start codes.
  The Buffer to retrieve the compressed video chunks has to be allocated by the caller. The related attribute BufSize
*/
typedef struct vvdec_accessUnit
{
  unsigned char*  payload;         // pointer to buffer that retrieves the coded data,
  int             payloadSize;     // size of the allocated buffer in bytes
  int             payloadUsedSize; // length of the coded data in bytes
  uint64_t        cts;             // composition time stamp in TicksPerSecond (see VVCDecoderParameter)
  uint64_t        dts;             // decoding time stamp in TicksPerSecond (see VVCDecoderParameter)
  bool            ctsValid;        // composition time stamp valid flag (true: valid, false: CTS not set)
  bool            dtsValid;        // decoding time stamp valid flag (true: valid, false: DTS not set)
  bool            rap;             // random access point flag (true: AU is random access point, false: sequential access)
} vvdec_accessUnit_t;

/* vvdec_accessUnit_alloc:
   Allocates an vvdec_accessUnit instance.
   The returned accessUnit is set to default values.
   The payload memory must be allocated seperately by using vvdec_accessUnit_alloc_payload.
   To free the memory use vvdec_accessUnit_free.
*/
VVDEC_DECL vvdec_accessUnit* vvdec_accessUnit_alloc();

/* vvdec_accessUnit_free:
   release storage of an vvdec_accessUnit instance.
   The payload memory is also released if not done yet.
*/
VVDEC_DECL void vvdec_accessUnit_free(vvdec_accessUnit *accessUnit );

/* vvdec_accessUnit_alloc_payload:
   Allocates the memory for an accessUnit payload.
   To free the memory use vvdec_accessUnit_free_payload.
   When the vvdec_accessUnit memory is released the payload memory is also released.
*/
VVDEC_DECL void vvdec_accessUnit_alloc_payload(vvdec_accessUnit *accessUnit, int payload_size );

/* vvdec_accessUnit_free_payload:
   release storage of the payload in an vvdec_accessUnit instance.
*/
VVDEC_DECL void vvdec_accessUnit_free_payload(vvdec_accessUnit *accessUnit );

/* vvdec_accessUnit_default:
  Initialize vvdec_accessUnit structure to default values
*/
VVDEC_DECL void vvdec_accessUnit_default(vvdec_accessUnit *accessUnit );

typedef struct vvdec_vui
{
  bool       aspectRatioInfoPresentFlag;
  bool       aspectRatioConstantFlag;
  bool       nonPackedFlag;
  bool       nonProjectedFlag;
  int        aspectRatioIdc;
  int        sarWidth;
  int        sarHeight;
  bool       colourDescriptionPresentFlag;
  int        colourPrimaries;
  int        transferCharacteristics;
  int        matrixCoefficients;
  bool       progressiveSourceFlag;
  bool       interlacedSourceFlag;
  bool       chromaLocInfoPresentFlag;
  int        chromaSampleLocTypeTopField;
  int        chromaSampleLocTypeBottomField;
  int        chromaSampleLocType;
  bool       overscanInfoPresentFlag;
  bool       overscanAppropriateFlag;
  bool       videoSignalTypePresentFlag;
  bool       videoFullRangeFlag;
}vvdec_vui_t;

typedef struct vvdec_hrd
{
  uint32_t   numUnitsInTick;
  uint32_t   timeScale;
  bool       generalNalHrdParamsPresentFlag;
  bool       generalVclHrdParamsPresentFlag;
  bool       generalSamePicTimingInAllOlsFlag;
  uint32_t   tickDivisor;
  bool       generalDecodingUnitHrdParamsPresentFlag;
  uint32_t   bitRateScale;
  uint32_t   cpbSizeScale;
  uint32_t   cpbSizeDuScale;
  uint32_t   hrdCpbCnt;
}vvdec_hrd_t;


/*
  The struct vvdec_picAttributes - contains decoder side information
*/
typedef struct vvdec_picAttributes
{
  NalType         nalType;         // nal unit type
  VVDEC_SliceType sliceType;       // slice type (I/P/B) */
  bool            isRefPic;        // reference picture
  int             temporalLayer;   // temporal layer
  uint64_t        poc;             // picture order count
  unsigned int    bits;            // bits of the compr. image packet
  vvdec_vui      *vui;             // if available, pointer to VUI (Video Usability Information)
  vvdec_hrd      *hrd;             // if available, pointer to HRD (Hypothetical Reference Decoder)
} vvdec_picAttributes_t;


typedef struct vvdec_plane
{
  unsigned char*  ptr     = nullptr;      ///< pointer to plane buffer
  int             width   = 0;            ///< width of the plane
  int             height  = 0;            ///< height of the plane
  int             stride  = 0;            ///< stride (width + left margin + right margins) of plane in samples
  uint32_t        bytesPerSample = 1;     ///< offset to first sample in bytes
} vvdec_plane_t;

/*
  The struct vvdec_frame combines the struct vvdec_plane and the optional picAttributes class.
*/
typedef struct vvdec_frame
{
  vvdec_plane     planes[ 3 ];                         ///< component plane for yuv
  uint32_t        numPlanes       = 0;                 ///< number of color components
  uint32_t        width           = 0;                 ///< width of the luminance plane
  uint32_t        height          = 0;                 ///< height of the luminance plane
  uint32_t        bitDepth        = 0;                 ///< bit depth of input signal (8: depth 8 bit, 10: depth 10 bit  )
  FrameFormat     frameFormat     = VVDEC_FF_INVALID;  ///< interlace format (VVC_FF_PROGRESSIVE)
  ColorFormat     colorFormat     = VVDEC_CF_INVALID;  ///< color format     (VVC_CF_YUV420_PLANAR)
  uint64_t        sequenceNumber  = 0;                 ///< sequence number of the picture
  uint64_t        cts             = 0;                 ///< composition time stamp in TicksPerSecond (see HEVCEncoderParameter)
  bool            ctsValid        = false;             ///< composition time stamp valid flag (true: valid, false: CTS not set)

  vvdec_picAttributes* picAttributes = NULL;     ///< pointer to PicAttribute that might be NULL, containing decoder side information
}vvdec_frame_t;

/*
  The struct vvdec_Params is a container for decoder configuration parameters.
  Use vvdec_default_params() to set default values.
*/
typedef struct vvdec_params
{
  int             threads                      = -1;                   ///< thread count        ( default: -1 )
  int             parseThreads                 = -1;                   ///< parser thread count ( default: -1 )
  bool            upscaleOutput                =  0;                   ///< do internal upscaling of rpl pictures to dest. resolution ( default: 0 )
  LogLevel        logLevel                     = VVDEC_WARNING;        ///< verbosity level
  bool            decodedPictureHashSEIEnabled = false;                ///<  Control handling of decoded picture hash SEI messages, true: check hash in SEI messages if available in the bitstream, false: ignore SEI message
  SIMD_Extension  simd                         = VVDEC_SIMD_DEFAULT;   ///< set specific simd optimization (default: max. availalbe)
} vvdec_params_t;

/* vvdec_params_default:
  Initialize vvdec_params structure to default values
*/
VVDEC_DECL void vvdec_params_default(vvdec_params *param);

/* vvdec_params_alloc:
   Allocates an vvdec_params_alloc instance.
   The returned params struct is set to default values.
*/
VVDEC_DECL vvdec_params* vvdec_params_alloc();

/* vvdec_params_free:
   release storage of an vvdec_params instance.
*/
VVDEC_DECL void vvdec_params_free(vvdec_params *params );

/* vvdec_get_version
  This method returns the the decoder version as string.
  \param[in]  none
  \retval[ ]  const char* version number as string
*/
VVDEC_DECL const char* vvdec_get_version();

/* vvdec_decoder_open
  This method initializes the decoder instance.
  This method is used to initially set up the decoder with the assigned decoder parameter struct.
  The method fails if the assigned parameter struct does not pass the consistency check.
  Other possibilities for an unsuccessful memory initialization, or an machine with
  insufficient CPU-capabilities.
  \param[in]  vvdec_params_t pointer of vvdec_params struct that holds initial decoder parameters.
  \retval     vvdec_params_t pointer of the decoder handler if successful, otherwise NULL
  \pre        The decoder must not be initialized (pointer of decoder handler must be null).
*/
VVDEC_DECL vvdec_decoder_t* vvdec_decoder_open( vvdec_params_t *);

/* vvdec_decoder_close
 This method resets the decoder instance.
 This method clears the decoder and releases all internally allocated memory.
 Calling uninit cancels all pending decoding calls. In order to finish pending pictures use the flush method.
 \param[in]  vvdec_decoder_t pointer of decoder handler
 \retval     int if non-zero an error occurred (see ErrorCodes), otherwise VVDEC_OK indicates success.
 \pre        The decoder has to be initialized successfully.
*/
VVDEC_DECL int vvdec_decoder_close(vvdec_decoder_t *);

/* vvdec_set_logging_callback
 *  Set a logging callback with the given log level.
  \param[in]   vvdec_decoder_t pointer of decoder handler
  \param[in]   vvdec_loggingCallback implementation of the callback that is called when logging messages are written
  \param[in]   userData
  \param[in]   LogLevel logging message level (only logs of level >= given level are written
  \retval      int if non-zero an error occurred (see ErrorCodes), otherwise VVDEC_OK indicates success.
  \pre         The decoder has to be initialized successfully.
 */
VVDEC_DECL int vvdec_set_logging_callback(vvdec_decoder_t* , vvdec_loggingCallback callback, void *userData, LogLevel logLevel);

/* vvdec_decode
  This method decodes a compressed image packet (bitstream).
  Compressed image packet are passed to the decoder in decoder order. A picture is returned by filling the assigned Picture struct.
  A picture is valid if the decoder call returns success and the Picture is not null.
  If the AccessUnit  m_iBufSize = 0, the decoder just returns a pending pictures chunk if available.
  \param[in]   vvdec_decoder_t pointer of decoder handler
  \param[in]   vvdec_accessUnit_t pointer of AccessUnit that retrieves compressed access units and side information, data are valid if UsedSize attribute is non-zero and the call was successful.
  \param[out]  vvdec_frame_t pointer to pointer of frame structure containing a uncompressed picture and meta information.
  \retval      int if non-zero an error occurred or more data is needed, otherwise the retval indicates success VVDEC_OK
  \pre         The decoder has to be initialized successfully.
*/
VVDEC_DECL int vvdec_decode( vvdec_decoder_t *, vvdec_accessUnit_t *accessUnit, vvdec_frame_t **frame );

/* vvdec_flush
  This method flushes the decoder.
  This call is used to get outstanding pictures after all compressed packets have been passed over into the decoder using the decode call.
  Using the flush method the decoder is signaled that there are no further compressed packets to decode.
  The caller should repeat the flush call until all pending pictures has been delivered to the caller, which is when the the function returns VVDEC_EOF or no picture.
  \param[in]  vvdec_decoder_t pointer to decoder handler
  \param[out] vvdec_frame_t pointer to pointer of frame structure containing a uncompressed picture and meta information.
  \retval     int if non-zero an error occurred, otherwise the retval indicates success VVDEC_OK
  \pre        The decoder has to be initialized successfully.
*/
VVDEC_DECL int vvdec_flush( vvdec_decoder_t *, vvdec_frame_t **frame );

/* vvdec_find_frame_sei
  This method finds SEI message in a given picture.
  To get the correct sei data for a given SEIPayloadType the payload have to be casted to the payload type.
  \param[in]  vvdec_decoder_t pointer of decoder handler
  \param[in]  SEIPayloadType payload type to search for
  \param[in]  vvdec_frame_t pointer of frame to search for sei
  \param[out] sei_message_t pointer to found sei message, NULL if not found
  \retval     int if non-zero an error occurred, otherwise the retval indicates success VVDEC_OK
  \pre        The decoder has to be initialized successfully.
*/
VVDEC_DECL vvdec_sei_message_t* vvdec_find_frame_sei( vvdec_decoder_t *, SEIPayloadType seiPayloadType, vvdec_frame_t *frame );

/* vvdec_frame_unref
  This method unreference an picture and frees the memory.
  This call is used to free the memory of an picture which is not used anymore.
  \param[in]  vvdec_decoder_t pointer of decoder handler
  \param[out] vvdec_frame_t pointer of frame to delete
  \retval     int if non-zero an error occurred, otherwise the retval indicates success VVDEC_OK
  \pre        The decoder has to be initialized successfully.
*/
VVDEC_DECL int vvdec_frame_unref( vvdec_decoder_t *, vvdec_frame_t *frame );

/* vvdec_get_hash_error_count
 This method returns the number of found errors if PictureHash SEI is enabled.
 \param[in]  vvdec_decoder_t pointer of decoder handler
 \retval     int if non-zero an error occurred, otherwise 0 indicates success.
 \pre        The decoder has to be initialized successfully.
*/
VVDEC_DECL int vvdec_get_hash_error_count( vvdec_decoder_t * );


/* vvdec_get_dec_information
 This method returns general decoder information
 \param[in]  vvdec_decoder_t pointer of decoder handler
 \retval     const char* decoder information
 \pre        The decoder has to be initialized successfully.
*/
VVDEC_DECL const char* vvdec_get_dec_information( vvdec_decoder_t * );


/* vvdec_get_last_error
 This method returns the last occurred error as a string.
 \param[in]  vvdec_decoder_t pointer of decoder handler
 \retval     const char* empty string for no error assigned
 \pre        The decoder has to be initialized successfully.
*/
VVDEC_DECL const char* vvdec_get_last_error( vvdec_decoder_t * );

/* vvdec_get_last_additional_error
 This method returns additional information about the last occurred error as a string (if availalbe).
 \param[in]  vvdec_decoder_t pointer of decoder handler
 \retval     const char* empty string for no error assigned
 \pre        The decoder has to be initialized successfully.
*/
VVDEC_DECL const char* vvdec_get_last_additional_error( vvdec_decoder_t * );

/* vvdec_get_error_msg
 This function returns a string according to the passed parameter nRet.
 \param[in]  nRet return value code (see ErrorCodes) to translate
 \retval[ ]  const char* empty string for no error
*/
VVDEC_DECL const char* vvdec_get_error_msg( int nRet );

/* vvdec_get_nal_unit_type
 This function returns the NalType of a given AccessUnit.
 \param[in]  vvdec_accessUnit_t pointer of accessUnit that retrieves compressed access units and
             side information, data are valid if UsedSize attribute is non-zero and the call was successful.
 \retval[ ]  NalType found Nal Unit type
*/
VVDEC_DECL NalType vvdec_get_nal_unit_type ( vvdec_accessUnit *accessUnit );

/* vvdec_get_nal_unit_type_name
 This function returns the name of a given NalType
 \param[in]  NalType value of enum NalType
 \retval[ ]  const char* NalType as string
*/
VVDEC_DECL const char* vvdec_get_nal_unit_type_name( NalType t );

/* vvdec_is_nal_unit_slice
 This function returns true if a given NalType is of type picture or slice
 \param[in]  NalType value of enum NalType
 \retval[ ]  bool true if slice/picture, else false
*/
VVDEC_DECL bool vvdec_is_nal_unit_slice ( NalType t );


#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /*_VVDEC_H_*/
