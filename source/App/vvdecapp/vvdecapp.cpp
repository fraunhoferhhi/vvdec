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

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <string.h>
#include <chrono>
#include <thread>

#include "vvdec/vvdec.h"
#include "vvdec/version.h"

#include "CmdLineParser.h"

#include "vvdecHelper.h"
#include "MD5StreamBuf.h"

#if defined (_WIN32) || defined (WIN32) || defined (_WIN64) || defined (WIN64)
#include <io.h>
#include <fcntl.h>
#endif

/*! Prototypes */
int writeYUVToFile( std::ostream *f, vvdecFrame *frame, bool y4mFormat = false );
int writeYUVToFileInterlaced( std::ostream *f, vvdecFrame *topField, vvdecFrame *botField = nullptr, bool y4mFormat = false );

void msgFnc( void *, int level, const char* fmt, va_list args )
{
  vfprintf( level == 1 ? stderr : stdout, fmt, args );
}

void msgFncErr( void *, int, const char* fmt, va_list args )
{
  vfprintf( stderr, fmt, args );
}

// this static variables are only needed for testing purposes, to check if external allocator is working properly
static std::vector<void*> extAllocVec;
static std::vector<void*> extFreeVec;
static unsigned extRefCount;
static unsigned extUnrefCount;

class BufferStorage
{
public:
BufferStorage()  = default;
~BufferStorage()
{
  if ( _isAllocated ) destroy();
}

int create( size_t size, size_t alignment )
{
  if( size == 0 ){ return VVDEC_ERR_ALLOCATE; }

#if  ( _WIN32 && ( _MSC_VER > 1300 ) ) || defined (__MINGW64_VERSION_MAJOR)
  _ptr = (unsigned char *)_aligned_malloc( sizeof(unsigned char)*(size), alignment );
#elif defined (__MINGW32__)
  _ptr = (unsigned char *)__mingw_aligned_malloc( sizeof(unsigned char)*(size), alignment )
#else
  if( posix_memalign( (void**)&_ptr, alignment, sizeof(unsigned char)*(size) ) )
  {
    return -1;
  }
#endif

  _size        = size;
  _isAllocated = true;
  return 0;
}

int buffer_ref()
{
  if( !_isAllocated) { return VVDEC_ERR_ALLOCATE; }
  _usecnt++;
  extRefCount++;
  return 0;
}

int buffer_unref()
{
  if( !_isAllocated) { return VVDEC_ERR_ALLOCATE; }
  if( 0 == _usecnt ) { return VVDEC_ERR_ALLOCATE; }
  _usecnt--;
  extUnrefCount++;

  if ( _usecnt == 0 )
  {
    destroy();
  }

  return 0;
}

unsigned char * getStorage() { return _ptr; }
size_t          size()       { return _size; }
bool            isUsed()     { return _isAllocated; }

private:
int destroy()
{
  if( !_isAllocated) { return VVDEC_ERR_ALLOCATE; }

#if     ( _WIN32 && ( _MSC_VER > 1300 ) ) || defined (__MINGW64_VERSION_MAJOR)
_aligned_free  ( _ptr );
#elif defined (__MINGW32__)
__mingw_aligned_free( _ptr );
#else
  free (_ptr);
#endif

  _isAllocated = false;
  _size        = 0;
  return 0;
}

private:
bool           _isAllocated = false;
unsigned char *_ptr         = nullptr;     // pointer to plane buffer
size_t         _size        = 0;
uint32_t       _usecnt      = 0;

};


void* picBufferCreateFnc( void *, vvdecComponentType, uint32_t size, uint32_t alignment, void **application_data )
{
  BufferStorage* s = new BufferStorage();
  //std::cout << "create(" << plane << ") " << size << " ptr " << (void*)s << std::endl;
  extAllocVec.push_back( (void*)s  );

  if( 0 != s->create(size, alignment) )
  {
    delete s;
    return nullptr;
  }
  // add a buffer reference to the created buffer
  s->buffer_ref();
  *application_data = (void*)s;
  return (void*)s->getStorage();
}

void picBufferUnrefFnc( void *, void *application_data )
{
  BufferStorage* s = (BufferStorage*)application_data;
  //std::cout << "destroy ptr  " << (void*)s << std::endl;
  extFreeVec.push_back( (void*)s  );

  // remove buffer reference and free memory when all references are released
  s->buffer_unref();
  if( !s->isUsed() )
    delete s;
}


int unrefBuf( void*& allocator )
{
  if( allocator )
  {
    BufferStorage* s = (BufferStorage*)allocator;
    s->buffer_unref();
    if( !s->isUsed() )
      delete s;
  }

  return 0;
}

/* refFrame
  create a new reference to all plane buffers
*/
void refFrame( vvdecFrame *frame )
{
  for( auto p : frame->planes )
  {
    if( p.allocator )
    {
      BufferStorage* s = (BufferStorage*)p.allocator ;
      s->buffer_ref();
    }
  }
}

/* unrefFrame
   unreference frame after it is rendered 
   when all buffer references are release the buffer must be freed
*/
void unrefFrame( vvdecFrame *frame )
{
  for( auto p : frame->planes )
  {
    if( p.allocator )
    {
      BufferStorage* s = (BufferStorage*)p.allocator;
      s->buffer_unref();
      if( !s->isUsed() )
        delete s;
    }
  }
}

void printSEI( vvdecDecoder *dec, vvdecFrame *frame, std::ostream * logStream )
{
  vvdecSEI *sei = vvdec_find_frame_sei( dec, VVDEC_BUFFERING_PERIOD, frame );
  if( sei )
  {
    *logStream << "vvdecapp [detail]: SEI BUFFERING_PERIOD: " << std::endl;
  }

  sei = vvdec_find_frame_sei( dec, VVDEC_PICTURE_TIMING, frame );
  if( sei )
  {
    *logStream << "vvdecapp [detail]: SEI PICTURE_TIMING" << std::endl;
  }

  sei = vvdec_find_frame_sei( dec, VVDEC_FILLER_PAYLOAD, frame );
  if( sei )
  {
    *logStream << "vvdecapp [detail]: SEI FILLER_PAYLOAD" << std::endl;
  }
  
  sei = vvdec_find_frame_sei( dec, VVDEC_USER_DATA_REGISTERED_ITU_T_T35, frame );
  if( sei )
  {
    *logStream << "vvdecapp [detail]: SEI USER_DATA_REGISTERED_ITU_T_T35" << std::endl;
  }

  sei = vvdec_find_frame_sei( dec, VVDEC_USER_DATA_UNREGISTERED, frame );
  if( sei )
  {
    *logStream << "vvdecapp [detail]: SEI USER_DATA_UNREGISTERED" << std::endl;
  }

  sei = vvdec_find_frame_sei( dec, VVDEC_FILM_GRAIN_CHARACTERISTICS, frame );
  if( sei )
  {
    *logStream << "vvdecapp [detail]: SEI FILM_GRAIN_CHARACTERISTICS" << std::endl;
  }

  sei = vvdec_find_frame_sei( dec, VVDEC_FRAME_PACKING, frame );
  if( sei )
  {
    *logStream << "vvdecapp [detail]: SEI FRAME_PACKING" << std::endl;
  }

  sei = vvdec_find_frame_sei( dec, VVDEC_PARAMETER_SETS_INCLUSION_INDICATION, frame );
  if( sei )
  {
    *logStream << "vvdecapp [detail]: SEI PARAMETER_SETS_INCLUSION_INDICATION" << std::endl;
  }

  sei = vvdec_find_frame_sei( dec, VVDEC_DECODING_UNIT_INFO, frame );
  if( sei )
  {
    *logStream << "vvdecapp [detail]: SEI DECODING_UNIT_INFO" << std::endl;
  }

  sei = vvdec_find_frame_sei( dec, VVDEC_DECODED_PICTURE_HASH, frame );
  if( sei )
  {
    *logStream << "vvdecapp [detail]: SEI DECODED_PICTURE_HASH " << std::endl;
  }

  sei = vvdec_find_frame_sei( dec, VVDEC_SCALABLE_NESTING, frame );
  if( sei )
  {
    *logStream << "vvdecapp [detail]: SEI SCALABLE_NESTING" << std::endl;
  }

  sei = vvdec_find_frame_sei( dec, VVDEC_MASTERING_DISPLAY_COLOUR_VOLUME, frame );
  if( sei )
  {
    vvdecSEIMasteringDisplayColourVolume* p = reinterpret_cast<vvdecSEIMasteringDisplayColourVolume *>(sei->payload);
    *logStream << "vvdecapp [detail]: SEI MASTERING_DISPLAY_COLOUR_VOLUME: lumaMin,Max " << p->minLuminance << "," << p->maxLuminance << std::endl;
  }

  sei = vvdec_find_frame_sei( dec, VVDEC_DEPENDENT_RAP_INDICATION, frame );
  if( sei )
  {
    *logStream << "vvdecapp [detail]: SEI DEPENDENT_RAP_INDICATION" << std::endl;
  }

  sei = vvdec_find_frame_sei( dec, VVDEC_EQUIRECTANGULAR_PROJECTION, frame );
  if( sei )
  {
    *logStream << "vvdecapp [detail]: SEI EQUIRECTANGULAR_PROJECTION" << std::endl;
  }

  sei = vvdec_find_frame_sei( dec, VVDEC_SPHERE_ROTATION, frame );
  if( sei )
  {
    *logStream << "vvdecapp [detail]: SEI SPHERE_ROTATION" << std::endl;
  }

  sei = vvdec_find_frame_sei( dec, VVDEC_REGION_WISE_PACKING, frame );
  if( sei )
  {
    *logStream << "vvdecapp [detail]: SEI REGION_WISE_PACKING" << std::endl;
  }

  sei = vvdec_find_frame_sei( dec, VVDEC_OMNI_VIEWPORT, frame );
  if( sei )
  {
    *logStream << "vvdecapp [detail]: SEI OMNI_VIEWPORT" << std::endl;
  }

  sei = vvdec_find_frame_sei( dec, VVDEC_GENERALIZED_CUBEMAP_PROJECTION, frame );
  if( sei )
  {
    *logStream << "vvdecapp [detail]: SEI GENERALIZED_CUBEMAP_PROJECTION" << std::endl;
  }

  sei = vvdec_find_frame_sei( dec, VVDEC_FRAME_FIELD_INFO, frame );
  if( sei )
  {
    *logStream << "vvdecapp [detail]: SEI FRAME_FIELD_INFO" << std::endl;
  }

  sei = vvdec_find_frame_sei( dec, VVDEC_SUBPICTURE_LEVEL_INFO, frame );
  if( sei )
  {
    *logStream << "vvdecapp [detail]: SEI SUBPICTURE_LEVEL_INFO" << std::endl;
  }

  sei = vvdec_find_frame_sei( dec, VVDEC_SAMPLE_ASPECT_RATIO_INFO, frame );
  if( sei )
  {
    *logStream << "vvdecapp [detail]: SEI SAMPLE_ASPECT_RATIO_INFO" << std::endl;
  }

  sei = vvdec_find_frame_sei( dec, VVDEC_CONTENT_LIGHT_LEVEL_INFO, frame );
  if( sei )
  {
    vvdecSEIContentLightLevelInfo* p = reinterpret_cast<vvdecSEIContentLightLevelInfo *>(sei->payload);
    *logStream << "vvdecapp [detail]: SEI CONTENT_LIGHT_LEVEL_INFO: " << p->maxContentLightLevel << "," << p->maxPicAverageLightLevel << std::endl;
  }

  sei = vvdec_find_frame_sei( dec, VVDEC_ALTERNATIVE_TRANSFER_CHARACTERISTICS, frame );
  if( sei )
  {
    vvdecSEIAlternativeTransferCharacteristics* p = reinterpret_cast<vvdecSEIAlternativeTransferCharacteristics *>(sei->payload);
    *logStream << "vvdecapp [detail]: SEI ALTERNATIVE_TRANSFER_CHARACTERISTICS: preferred_transfer_characteristics " << (int)p->preferred_transfer_characteristics << std::endl;
  }

  sei = vvdec_find_frame_sei( dec, VVDEC_AMBIENT_VIEWING_ENVIRONMENT, frame );
  if( sei )
  {
    *logStream << "vvdecapp [detail]: SEI AMBIENT_VIEWING_ENVIRONMENT" << std::endl;
  }
  
  sei = vvdec_find_frame_sei( dec, VVDEC_CONTENT_COLOUR_VOLUME, frame );
  if( sei )
  {
    *logStream << "vvdecapp [detail]: SEI CONTENT_COLOUR_VOLUME" << std::endl;   
  }
}

int main( int argc, char* argv[] )
{
  std::string cAppname = argv[0];
  std::size_t iPos = (int)cAppname.find_last_of("/");
  if( std::string::npos != iPos )
  {
    cAppname = cAppname.substr(iPos+1 );
  }
  else
  {
    iPos = (int)cAppname.find_last_of("\\");
    if( std::string::npos != iPos )
    {
      cAppname = cAppname.substr(iPos+1 );
    }
  }

  std::string cBitstreamFile = "";
  std::string cOutputFile    = "";
  int         iMaxFrames     = -1;
  int         iLoopCount     = 1;
  bool        y4mOutput      = false;
  bool        externAllocator = false;
  std::string cExpectedYuvMD5;
  std::string sTracingRule;
  std::string sTracingFile;
  vvdecParams params;
  vvdec_params_default(&params);
  int         iPrintPicHash = 0;

  params.logLevel = VVDEC_NOTICE;

  if(  argc > 1 && (!strcmp( (const char*) argv[1], "--help" ) || !strcmp( (const char*) argv[1], "-help" )) )
  {
    vvdecoderapp::CmdLineParser::print_usage( cAppname, params, false );
    return 0;
  }


  int iRet = vvdecoderapp::CmdLineParser::parse_command_line( argc, argv, params, cBitstreamFile, cOutputFile, iMaxFrames, iLoopCount, cExpectedYuvMD5, y4mOutput, externAllocator, sTracingFile, sTracingRule, iPrintPicHash );
  if( iRet != 0 )
  {
    if( iRet == 2 )
    {
      vvdecoderapp::CmdLineParser::print_usage( cAppname, params, false );
      return 0;
    }
    else if( iRet == 3 )
    {
      vvdecoderapp::CmdLineParser::print_usage( cAppname, params, true );
      return 0;
    }
    else if( iRet == 4 )
    {
      std::cout << cAppname << " version " << vvdec_get_version() << std::endl;
      return 0;
    }

    std::cerr << "vvdecapp [error]: cannot parse command line. run vvdecapp --help to see available options" << std::endl;
    return -1;
  }

  if( cBitstreamFile.empty() )
  {
    std::cerr << "vvdecapp [error]: no bitstream file given. run vvdecapp --help to see available options" << std::endl;
    return -1;
  }

  // open input file
  std::ifstream cInFile( cBitstreamFile.c_str(), std::fstream::binary );
  if( !cInFile )
  {
    std::cerr << "vvdecapp [error]: failed to open bitstream file " << cBitstreamFile << std::endl;
    return -1;
  }

  bool writeStdout = false;
  
  // open output file
  std::ios * pStream{nullptr};
  std::fstream cRecFile;
  if( !cOutputFile.empty() )
  {
    if( !strcmp( cOutputFile.c_str(), "-" ) )
    {
      writeStdout = true;
#if defined (_WIN32) || defined (WIN32) || defined (_WIN64) || defined (WIN64)
      if( _setmode( _fileno( stdout ), _O_BINARY ) == -1 )
      {
        std::cerr << "vvdecapp [error]: failed to set stdout to binary mode" << std::endl;
        return -1;
      }
#endif
    }
    else
    {
      const char * s = strrchr(cOutputFile.c_str(), '.');
      if (s && !strcmp(s, ".y4m"))
      {
        y4mOutput = true;
      }

      cRecFile.open( cOutputFile.c_str(), std::fstream::binary | std::fstream::out );
      if( !cRecFile )
      {
        std::cerr << "vvdecapp [error]: failed to open ouptut file " << cOutputFile << std::endl;
        return -1;
      }
      pStream = &cRecFile;
    }
  }
  
  std::ostream * outStream = writeStdout ? &std::cout : dynamic_cast<std::ostream*>(pStream);
  std::ostream * logStream = writeStdout ? &std::cerr : &std::cout;

  if( !cOutputFile.empty() && (nullptr == outStream || !outStream) )
  {
    *logStream << "vvdecapp [error]: failed to open ouptut file " << cOutputFile << std::endl;
    return -1;
  }

  // stream buffer to directly calculate MD5 hash over the YUV output
  vvdecoderapp::MD5StreamBuf md5Buf( 1024 * 1024 );
  std::ostream md5Stream( &md5Buf );

  vvdecDecoder *dec = nullptr;

  //> decoding loop
  vvdecAccessUnit* accessUnit = vvdec_accessUnit_alloc();
  vvdec_accessUnit_alloc_payload( accessUnit, MAX_CODED_PICTURE_SIZE );

#ifdef ENABLE_TRACING
  if( sTracingFile.size() || sTracingRule.size() )
  {
    if( vvdec_set_tracing( sTracingFile.c_str(), sTracingRule.c_str() ) != VVDEC_OK )
    {
      *logStream << "vvdecapp [error]: failed to initialize tracing to file '" << sTracingFile << "' with rule '" << sTracingRule << "'" << std::endl;
      return -1;
    }
  }
#endif   // ENABLE_TRACING

  bool bOutputInfoWritten = false;

  std::chrono::steady_clock::time_point cTPStartRun;
  std::chrono::steady_clock::time_point cTPEndRun;

  std::chrono::steady_clock::time_point cTPStart;
  std::chrono::steady_clock::time_point cTPEnd;

  std::vector<double > dFpsPerLoopVec;

  int iSEIHashErrCount = 0;

  int iLoop = 0;
  bool bContinue = true;
  while ( bContinue )
  {
    vvdecFrame* pcFrame     = NULL;
    vvdecFrame* pcPrevField = NULL;

    if( iLoopCount > 1 && params.logLevel >= VVDEC_INFO )
    {
      *logStream << "-------------------------------------" << std::endl;
      *logStream << "begin decoder loop #" << iLoop << std::endl;
      *logStream << "-------------------------------------" << std::endl;
    }

    // initialize the decoder
    if( externAllocator )
    {
      dec = vvdec_decoder_open_with_allocator( &params, picBufferCreateFnc, picBufferUnrefFnc );
    }
    else
    {
      dec = vvdec_decoder_open( &params );
    }

    if( nullptr == dec )
    {
      *logStream << "vvdecapp [error]: cannot init decoder" << std::endl;
      vvdec_accessUnit_free( accessUnit );
      return -1;
    }

    vvdec_set_logging_callback( dec, writeStdout ? msgFncErr : msgFnc );

    if( iLoop == 0 && params.logLevel >= VVDEC_INFO )
    {
      *logStream << vvdec_get_dec_information( dec ) << std::endl;
    }

    bool bFlushDecoder = false;

    cTPStartRun = std::chrono::steady_clock::now();
    cTPStart    = std::chrono::steady_clock::now();

    accessUnit->cts = 0; accessUnit->ctsValid = true;
    accessUnit->dts = 0; accessUnit->dtsValid = true;

    int iComprPics = 0;
    unsigned int uiFrames = 0;
    unsigned int uiFramesTmp = 0;
    unsigned int uiBitrate = 0;

    bool bTunedIn = false;
    unsigned int uiNoFrameAfterTuneInCount = 0;
    vvdecNalType eNalTypeSlice = VVC_NAL_UNIT_INVALID;
    bool bMultipleSlices = false;

    int iRead = 0;
    do
    {
      iRead = readBitstreamFromFile( &cInFile, accessUnit, false );
      //if( iRead > 0 )
      {
        vvdecNalType eNalType = vvdec_get_nal_unit_type( accessUnit );
        if( params.logLevel == VVDEC_DETAILS )
        {
          std::string cNal = getNalUnitTypeAsString( eNalType );
          *logStream << "vvdecapp [details]:  read nal " <<  cNal << " size " << accessUnit->payloadUsedSize << std::endl;
        }

        if( eNalType == VVC_NAL_UNIT_PH )
        {
          // picture header indicates multiple slices
          bMultipleSlices = true;
        }

        bool bIsSlice  = vvdec_is_nal_unit_slice( eNalType );
        if( bIsSlice )
        {
          if( bMultipleSlices )
          {
            if( eNalTypeSlice == VVC_NAL_UNIT_INVALID )
            {
              // set current slice type and increment pic count
              iComprPics++;
              eNalTypeSlice = eNalType;
            }
            else
            {
              bIsSlice = false; // prevent cts/dts increase if not first slice
            }
          }
          else
          {
            iComprPics++;
          }
        }

        if( eNalTypeSlice != VVC_NAL_UNIT_INVALID &&
            eNalType != eNalTypeSlice )
        {
          eNalTypeSlice = VVC_NAL_UNIT_INVALID; // reset slice type
        }


        if( iMaxFrames > 0 && iComprPics >= iMaxFrames )
        {
          iRead = -1;
        }

        // call decode

        iRet = vvdec_decode( dec, accessUnit, &pcFrame );
        if( bIsSlice )
        {
          accessUnit->cts++;
          accessUnit->dts++;
        }

        // check success
        if( iRet == VVDEC_EOF )
        {
          //std::cout << "flush" << std::endl;
          bFlushDecoder = true;
        }
        else if( iRet == VVDEC_TRY_AGAIN )
        {
          if( !bMultipleSlices )
          {
            if( params.logLevel >= VVDEC_VERBOSE ) *logStream << "vvdecapp [verbose]: more data needed to tune in" << std::endl;
            if( bTunedIn )
            {
              // after the first frame is returned, the decoder must always return a frame
              if( bIsSlice)
              {
                *logStream << "vvdecapp [error]: missing output picture!" << std::endl;
                uiNoFrameAfterTuneInCount++;
              }
            }
          }
        }
        else if( iRet == VVDEC_ERR_DEC_INPUT )
        {
          std::string cErr           = vvdec_get_last_error( dec );
          std::string cAdditionalErr = vvdec_get_last_additional_error( dec );
          *logStream << "vvdecapp [warning]: " << cErr << " (" << vvdec_get_error_msg( iRet ) << ")";
          if( !cAdditionalErr.empty() )
          {
            *logStream << " detail: " << vvdec_get_last_additional_error( dec ) << std::endl;
          }
          *logStream << std::endl;
        }
        else if( iRet != VVDEC_OK )
        {
          std::string cErr           = vvdec_get_last_error( dec );
          std::string cAdditionalErr = vvdec_get_last_additional_error( dec );
          *logStream << "vvdecapp [error]: decoding failed: " << cErr << " (" <<vvdec_get_error_msg( iRet ) << ")";
          if( !cAdditionalErr.empty() )
          {
            *logStream << " detail: " << vvdec_get_last_additional_error( dec ) << std::endl;
          }
          *logStream << std::endl;

          vvdec_accessUnit_free( accessUnit );
          return iRet;
        }

        if( NULL != pcFrame && pcFrame->ctsValid )
        {
          if ( externAllocator ) refFrame( pcFrame ); 

          if (!bOutputInfoWritten)
          {
            if( params.logLevel >= VVDEC_INFO )
            {
              *logStream << "vvdecapp [info]: SizeInfo: " << pcFrame->width << "x" << pcFrame->height << " (" << pcFrame->bitDepth << "b)" << std::endl;
              if( pcFrame->picAttributes && pcFrame->picAttributes->vui && pcFrame->picAttributes->vui->colourDescriptionPresentFlag )
              {
                *logStream << "vvdecapp [info]: VUI ColourDescription: colourPrim: " << pcFrame->picAttributes->vui->colourPrimaries << " transCharacteristics: " << pcFrame->picAttributes->vui->transferCharacteristics 
                           << " matrixCoefficients: " << pcFrame->picAttributes->vui->matrixCoefficients << std::endl;
              }
            }
            bOutputInfoWritten = true;
          }

          if( !bTunedIn )
          {
            bTunedIn = true;
          }

          uiFrames++;
          uiFramesTmp++;

          if( pcFrame->picAttributes )
          {
            uiBitrate += pcFrame->picAttributes->bits;
            (void)uiBitrate;
          }

          if( params.logLevel == VVDEC_DETAILS )
          {
            printSEI( dec, pcFrame, logStream );
          }

          if( pcFrame->frameFormat == VVDEC_FF_PROGRESSIVE )
          {
            if( iPrintPicHash > 1 )
            {
              printPicHash( pcFrame, logStream, uiFrames-1, iPrintPicHash-11 );
            }
            
            if( !cExpectedYuvMD5.empty() )
            {
              writeYUVToFile( &md5Stream, pcFrame );
            }
            if( cRecFile.is_open() || writeStdout )
            {
              if( 0 != writeYUVToFile( outStream, pcFrame, y4mOutput ) )
              {
                *logStream << "vvdecapp [error]: write of rec. yuv failed for picture seq. " <<  pcFrame->sequenceNumber << std::endl;
                vvdec_accessUnit_free( accessUnit );
                return iRet;
              }
            }
          }
          else if( pcFrame->frameFormat == VVDEC_FF_TOP_FIELD ||
                   pcFrame->frameFormat == VVDEC_FF_BOT_FIELD )
          {
            if( !pcPrevField )
            {
              pcPrevField = pcFrame;
            }
            else
            {
              if( !cExpectedYuvMD5.empty() )
              {
                writeYUVToFileInterlaced( &md5Stream, pcPrevField, pcFrame );
              }
              if( cRecFile.is_open() || writeStdout )
              {
                if( 0 != writeYUVToFileInterlaced( outStream, pcPrevField, pcFrame, y4mOutput ) )
                {
                  *logStream << "vvdecapp [error]: write of rec. yuv failed for picture seq. " <<  pcFrame->sequenceNumber << std::endl;
                  vvdec_accessUnit_free( accessUnit );
                  return iRet;
                }
              }
              
              if ( externAllocator ) unrefFrame( pcPrevField );
              vvdec_frame_unref( dec, pcPrevField );
              
              pcPrevField = nullptr;
            }
          }
          else
          {
            *logStream << "vvdecapp [error]: unsupported FrameFormat " << pcFrame->frameFormat << " for picture seq. " <<  pcFrame->sequenceNumber << std::endl;
            vvdec_accessUnit_free( accessUnit );
            return -1;
          }

          // free picture memory
          if( !pcPrevField || pcPrevField != pcFrame)
          {
            if ( externAllocator ) unrefFrame( pcFrame );             
            vvdec_frame_unref( dec, pcFrame );
          }
        }

        if( uiFrames && params.logLevel >= VVDEC_INFO )
        {
          cTPEnd = std::chrono::steady_clock::now();
          double dTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>((cTPEnd)-(cTPStart)).count();
          if( dTimeMs > 1000.0 )
          {
            *logStream << "vvdecapp [info]: decoded Frames: " << uiFrames << " Fps: " << uiFramesTmp << std::endl;
            cTPStart = std::chrono::steady_clock::now();
            uiFramesTmp = 0;
          }
        }
      }
    }
    while( iRead > 0 && !bFlushDecoder ); // end for frames

    //std::cout << "flush" << std::endl;

    // flush the decoder
    bFlushDecoder = true;
    while( bFlushDecoder)
    {
      vvdecFrame* pcFrame = NULL;

      // flush the decoder
      iRet = vvdec_flush( dec, &pcFrame );
      if( iRet != VVDEC_OK && iRet != VVDEC_EOF )
      {
        std::string cErr           = vvdec_get_last_error(dec);
        std::string cAdditionalErr = vvdec_get_last_additional_error(dec);
        *logStream << "vvdecapp [error]: decoding failed: " << cErr << " (" <<vvdec_get_error_msg( iRet ) << ")";
        if( !cAdditionalErr.empty() )
        {
          *logStream << " detail: " << cAdditionalErr;
        }
        *logStream << std::endl;
        vvdec_accessUnit_free( accessUnit );
        return iRet;
      }

      if( NULL != pcFrame  )
      {
        if ( externAllocator ) refFrame( pcFrame );

        uiFrames++;

        if( pcFrame->frameFormat == VVDEC_FF_PROGRESSIVE )
        {
          if( iPrintPicHash > 1 )
          {
            printPicHash( pcFrame, logStream, uiFrames-1, iPrintPicHash-11 );
          }
          
          if( !cExpectedYuvMD5.empty() )
          {
            writeYUVToFile( &md5Stream, pcFrame );
          }
          if( cRecFile.is_open() || writeStdout )
          {
            if( 0 != writeYUVToFile( outStream, pcFrame, y4mOutput ) )
            {
              *logStream << "vvdecapp [error]: write of rec. yuv failed for picture seq. " << pcFrame->sequenceNumber << std::endl;
              vvdec_accessUnit_free( accessUnit );
              return iRet;
            }
          }
        }
        else if( pcFrame->frameFormat == VVDEC_FF_TOP_FIELD ||
                 pcFrame->frameFormat == VVDEC_FF_BOT_FIELD )
        {
          if( !pcPrevField )
          {
            pcPrevField = pcFrame;
          }
          else
          {
            if( !cExpectedYuvMD5.empty() )
            {
              writeYUVToFileInterlaced( &md5Stream, pcPrevField, pcFrame );
            }
            if( cRecFile.is_open() || writeStdout )
            {
              if( 0 != writeYUVToFileInterlaced( outStream, pcPrevField, pcFrame, y4mOutput ) )
              {
                *logStream << "vvdecapp [error]: write of rec. yuv failed for picture seq. " << pcFrame->sequenceNumber << std::endl;
                vvdec_accessUnit_free( accessUnit );
                return iRet;
              }
            }

            if ( externAllocator ) unrefFrame( pcPrevField );
            vvdec_frame_unref( dec, pcPrevField );
            pcPrevField = nullptr;
          }
        }
        else
        {
          *logStream << "vvdecapp [error]: unsupported FrameFormat " << pcFrame->frameFormat << " for picture seq. " <<  pcFrame->sequenceNumber << std::endl;
          vvdec_accessUnit_free( accessUnit );
          return -1;
        }

        // free picture memory
        if( !pcPrevField || pcPrevField != pcFrame)
        {
          if ( externAllocator ) unrefFrame( pcFrame );
          vvdec_frame_unref( dec, pcFrame );
        }

        if( params.logLevel >= VVDEC_INFO )
        {
          cTPEnd = std::chrono::steady_clock::now();
          double dTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>((cTPEnd)-(cTPStart)).count();
          if( dTimeMs > 1000.0 )
          {
            *logStream << "vvdecapp [info]: decoded Frames: " << uiFrames << " Fps: " << uiFramesTmp << std::endl;
            cTPStart = std::chrono::steady_clock::now();
            uiFramesTmp = 0;
          }
        }
      }
      else
      {
        break;
      }
    };

    if( uiNoFrameAfterTuneInCount )
    {
      *logStream << "vvdecapp [error]: Decoder did not return " << uiNoFrameAfterTuneInCount << " pictures during decoding - came out too late" << std::endl;
    }

    cTPEndRun = std::chrono::steady_clock::now();
    double dTimeSec = std::chrono::duration_cast<std::chrono::milliseconds>((cTPEndRun)-(cTPStartRun)).count() / 1000.0;

    double dFps = dTimeSec ? ((double)uiFrames / dTimeSec) : uiFrames;
    dFpsPerLoopVec.push_back( dFps );
    if( params.logLevel >= VVDEC_INFO )
    {
      *logStream << "vvdecapp [info]: " << getTimePointAsString() << ": " << uiFrames << " frames decoded @ " << dFps << " fps (" << dTimeSec << " sec)\n" << std::endl;
    }

    iSEIHashErrCount = vvdec_get_hash_error_count(dec);
    if (iSEIHashErrCount )
    {
      *logStream << "vvdecapp [error]: MD5 checksum error ( " << iSEIHashErrCount << " errors )" << std::endl;
      vvdec_accessUnit_free( accessUnit );
      return iSEIHashErrCount;
    }

    if( iComprPics && !uiFrames )
    {
      *logStream << "vvdecapp [error]: read some input pictures (" << iComprPics << "), but no output was generated." << std::endl;
      vvdec_accessUnit_free( accessUnit );
      return -1;
    }

    if( !cExpectedYuvMD5.empty() )
    {
      std::transform( cExpectedYuvMD5.begin(), cExpectedYuvMD5.end(), cExpectedYuvMD5.begin(), (int ( * )( int ))std::tolower );

      const std::string yuvMD5 = md5Buf.finalizeHex();
      if( cExpectedYuvMD5 != yuvMD5 )
      {
        *logStream << "vvdecapp [error] full YUV output MD5 mismatch: " << cExpectedYuvMD5 << " != " << yuvMD5 << std::endl;
        vvdec_accessUnit_free( accessUnit );
        return -1;
      }
    }

    // un-initialize the decoder
    iRet = vvdec_decoder_close(dec);
    if( 0 != iRet )
    {
      *logStream << "vvdecapp [error]: cannot uninit decoder (" << iRet << ")" << std::endl;
      vvdec_accessUnit_free( accessUnit );
      return iRet;
    }

    if( iLoopCount < 0 || iLoop < iLoopCount-1 )
    {
      cInFile.clear();
      cInFile.seekg( 0, cInFile.beg );
      if(cInFile.bad() || cInFile.fail())
      {
        *logStream << "vvdecapp [error]: cannot seek to bitstream begin" << std::endl;
        vvdec_accessUnit_free( accessUnit );
        return -1;
      }
    }

    iLoop++;
    if( iLoopCount >= 0 && iLoop >= iLoopCount ){ bContinue = false;}
  }
  //< decoding loop finished

  // free memory of access unit
  vvdec_accessUnit_free( accessUnit );

  // close yuv output file
  if( !cOutputFile.empty() )
  {
    cRecFile.close();
  }

  cInFile.close();

  if( iLoopCount > 1 )
  {
    // print average fps over all decoder loops
    double dFpsSum  = .0;
    for( auto f : dFpsPerLoopVec )
    {
      dFpsSum += f;
    }
    if( params.logLevel >= VVDEC_INFO )
    {
      *logStream <<"vvdecapp [info]: avg. fps for " << dFpsPerLoopVec.size() << " loops: " << dFpsSum/dFpsPerLoopVec.size() << " Hz " << std::endl;
    }
  }

  if( externAllocator )
  {
   if ( (extAllocVec.size() != extFreeVec.size()) || ( extRefCount != extUnrefCount ) )
   {
      *logStream <<"vvdecapp [error]: extern alloc/free mismatch (allocated " << extAllocVec.size() << ", free " << extFreeVec.size() << ")" 
                 << " refcount " << extRefCount << " unrefcount " << extUnrefCount << std::endl;
      return -1;
   }
  }

  return 0;
}
