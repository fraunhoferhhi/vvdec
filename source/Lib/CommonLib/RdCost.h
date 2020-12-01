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

/** \file     RdCost.h
    \brief    RD cost computation classes (header)
*/

#ifndef __RDCOST__
#define __RDCOST__

#include "CommonDef.h"
#include "Unit.h"
#include "Buffer.h"

#include <math.h>

//! \ingroup CommonLib
//! \{

class DistParam;
class EncCfg;

// ====================================================================================================================
// Type definition
// ====================================================================================================================

// for function pointer
typedef Distortion (*FpDistFunc) (const DistParam&);

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// distortion parameter class
class DistParam
{
public:
  CPelBuf               org;
  CPelBuf               cur;
  FpDistFunc            distFunc;
  int                   bitDepth;

  // (vertical) subsampling shift (for reducing complexity)
  // - 0 = no subsampling, 1 = even rows, 2 = every 4th, etc.
  int                   subShift;
  DistParam() :
  org(), cur(), bitDepth( 0 ), subShift( 0 )
  { }
};

/// RD cost computation class
class RdCost
{
private:
  // for distortion

  static FpDistFunc m_afpDistortFunc[DF_TOTAL_FUNCTIONS]; // [eDFunc]

public:

  RdCost();
  ~RdCost();

#ifdef TARGET_SIMD_X86
  void initRdCostX86();
  template <X86_VEXT vext>
  void _initRdCostX86();
#endif

  static void setDistParam( DistParam &rcDP, const Pel* pOrg, const Pel* piRefY, ptrdiff_t iOrgStride, ptrdiff_t iRefStride, int bitDepth, int width, int height, int subShiftMode = 0 );

private:

  static Distortion xGetSAD           ( const DistParam& pcDtParam );
  static Distortion xGetSAD4          ( const DistParam& pcDtParam );
  static Distortion xGetSAD8          ( const DistParam& pcDtParam );
  static Distortion xGetSAD16         ( const DistParam& pcDtParam );
  static Distortion xGetSAD32         ( const DistParam& pcDtParam );
  static Distortion xGetSAD64         ( const DistParam& pcDtParam );
  static Distortion xGetSAD16N        ( const DistParam& pcDtParam );

#ifdef TARGET_SIMD_X86
  template< X86_VEXT vext >
  static Distortion xGetSAD_16xN_SIMD ( const DistParam& pcDtParam );
  template< int iWidth, X86_VEXT vext >
  static Distortion xGetSAD_NxN_SIMD  ( const DistParam& pcDtParam );
#endif
};// END CLASS DEFINITION RdCost

//! \}

#endif // __RDCOST__
