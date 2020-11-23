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

/** \file     Quant.h
    \brief    base quantization class (header)
*/

#ifndef __QUANT__
#define __QUANT__

#include "CommonDef.h"
#include "Unit.h"
#include "ChromaFormat.h"
#include "Contexts.h"
#include "ContextModelling.h"

#include "UnitPartitioner.h"

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Constants
// ====================================================================================================================

#define QP_BITS                 15

// ====================================================================================================================
// Class definition
// ====================================================================================================================
struct TrQuantParams
{
  int     rightShift;
  int     qScale;
};

/// QP struct
class QpParam
{
public:
  short  Qps[2];
  int8_t pers[2];
  int8_t rems[2];

private:
  
public:

  QpParam(const TransformUnit& tu, const ComponentID &compID, const bool allowACTQpoffset = true);
  
  int Qp ( const bool ts ) const { return Qps [ts?1:0]; }
  int per( const bool ts ) const { return pers[ts?1:0]; }
  int rem( const bool ts ) const { return rems[ts?1:0]; }

}; // END STRUCT DEFINITION QpParam

/// transform and quantization class
class Quant
{
public:
  int* getDequantCoeff           ( uint32_t list, int qp, uint32_t sizeX, uint32_t sizeY ) { return m_dequantCoef          [sizeX][sizeY][list][qp]; };  //!< get DeQuant Coefficent

  void setUseScalingList         ( bool bUseScalingList ) { m_scalingListEnabledFlag = bUseScalingList; };
#if JVET_P0365_SCALING_MATRIX_LFNST
#if JVET_R0380_SCALING_MATRIX_DISABLE_YCC_OR_RGB
  bool getUseScalingList         ( bool isTransformSkip, const bool lfnstApplied, const bool disableScalingMatrixForLFNSTBlks, const bool disableSMforACT)
  {
    return( m_scalingListEnabledFlag && !isTransformSkip && (!lfnstApplied || !disableScalingMatrixForLFNSTBlks) && !disableSMforACT);
  }
#else
  bool getUseScalingList         ( bool isTransformSkip, const bool lfnstApplied, const bool disableScalingMatrixForLFNSTBlks) 
  { 
    return( m_scalingListEnabledFlag && !isTransformSkip && (!lfnstApplied || !disableScalingMatrixForLFNSTBlks) );
  }
#endif
#else
  bool getUseScalingList         ( bool isTransformSkip ) { return m_scalingListEnabledFlag && !isTransformSkip; };
#endif
  void setScalingListDec         ( ScalingList &scalingList);
  void processScalingListDec     ( const int *coeff, int *dequantcoeff, int qpMod6, uint32_t height, uint32_t width, uint32_t ratio, int sizuNum, uint32_t dc);

  Quant();
  virtual ~Quant();

  // initialize class
  virtual void init( Slice *slice );
public:
  // de-quantization
  virtual void dequant           ( const TransformUnit &tu, CoeffBuf &dstCoeff, const ComponentID &compID, const QpParam &cQP );

private:
  void xInitScalingList   ( const Quant* other );
  void xDestroyScalingList();
  void xSetScalingListDec( const ScalingList &scalingList, uint32_t list, uint32_t size, int qp, uint32_t scalingListId );
  void xSetRecScalingListDec( const ScalingList &scalingList, uint32_t list, uint32_t sizew, uint32_t sizeh, int qp, uint32_t scalingListId );
  bool     m_scalingListEnabledFlag;

  int      *m_dequantCoef          [SCALING_LIST_SIZE_NUM][SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM][SCALING_LIST_REM_NUM]; ///< array of dequantization matrix coefficient 4x4
  int       m_dequantCoefBuf       [580644];


  void ( *DeQuant) (const int maxX,const int restX,const int maxY,const int scale,const TCoeffSig*const piQCoef,const size_t piQCfStride,TCoeff   *const piCoef,const int rightShift,const int inputMaximum,const TCoeff transformMaximum);
  void ( *DeQuantPCM) (const int maxX,const int restX,const int maxY,const int scale,TCoeff   *const piQCoef,const size_t piQCfStride,TCoeff   *const piCoef,const int rightShift,const int inputMaximum,const TCoeff transformMaximum);


#if ENABLE_SIMD_OPT_QUANT
  void initQuantX86();
  template <X86_VEXT vext>
  void _initQuantX86();

#endif

};// END CLASS DEFINITION Quant


//! \}

#endif // __QUANT__
