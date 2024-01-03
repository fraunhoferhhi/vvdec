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

/** \file     MotionInfo.h
    \brief    motion information handling classes (header)
    \todo     MvField seems to be better to be inherited from Mv
*/

#pragma once

#include "CommonDef.h"
#include "Mv.h"

namespace vvdec
{

// need miInvalid to be able to distinguish between different motion types, e.g. history, colocated or just motion info
static inline bool isMotionInvalid( int refIdx, const int miInvalid )
{
  static_assert( MH_NOT_VALID == -1 && MI_NOT_VALID == -1 && MF_NOT_VALID == -1 && CO_NOT_VALID == -1, "All not-valid motion need to be nagative for current impl!" );
  return refIdx < 0;
}

// need miInvalid to be able to distinguish between different motion types, e.g. history, colocated or just motion info
static inline bool isMotionValid( int refIdx, const int miInvalid )
{
  return !isMotionInvalid( refIdx, miInvalid );
}

// ====================================================================================================================
// Type definition
// ====================================================================================================================

/// parameters for AMVP
struct AMVPInfo
{
  Mv       mvCand[ AMVP_MAX_NUM_CANDS_MEM ];  ///< array of motion vector predictor candidates
  unsigned numCand;                       ///< number of motion vector predictor candidates
};

struct AffineAMVPInfo
{
  Mv       mvCandLT[ AMVP_MAX_NUM_CANDS_MEM ];  ///< array of affine motion vector predictor candidates for left-top corner
  Mv       mvCandRT[ AMVP_MAX_NUM_CANDS_MEM ];  ///< array of affine motion vector predictor candidates for right-top corner
  Mv       mvCandLB[ AMVP_MAX_NUM_CANDS_MEM ];  ///< array of affine motion vector predictor candidates for left-bottom corner
  unsigned numCand;                       ///< number of motion vector predictor candidates
};

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// class for motion vector with reference index
struct MvField
{
  Mv     mv;
  int8_t mfRefIdx = MF_NOT_VALID;

  MvField() = default;
  MvField( Mv const & cMv, const int iRefIdx ) : mv( cMv ), mfRefIdx(   iRefIdx ) {}

  void setMvField( Mv const & cMv, const int iRefIdx )
  {
    //CHECKD( iRefIdx == MF_NOT_VALID && cMv != Mv(0,0), "Must not happen." );
    mv     = cMv;
    mfRefIdx = iRefIdx;
  }

  bool operator==( const MvField& other ) const
  {
    //CHECKD( mfRefIdx == MF_NOT_VALID && mv != Mv(0,0), "Error in operator== of MvField." );
    //CHECKD( other.mfRefIdx == MF_NOT_VALID && other.mv != Mv(0,0), "Error in operator== of MvField." );
    return mfRefIdx == other.mfRefIdx && mv == other.mv;
  }
  bool operator!=( const MvField& other ) const
  {
    //CHECKD( mfRefIdx == MF_NOT_VALID && mv != Mv(0,0), "Error in operator!= of MvField." );
    //CHECKD( other.mfRefIdx == MF_NOT_VALID && other.mv != Mv(0,0), "Error in operator!= of MvField." );
    return mfRefIdx != other.mfRefIdx || mv != other.mv;
  }
};

struct MotionInfo
{
  Mv       mv      [NUM_REF_PIC_LIST_01];
  int8_t   miRefIdx[NUM_REF_PIC_LIST_01] = { MI_NOT_VALID, MI_NOT_VALID };

  bool operator==( const MotionInfo &mi ) const
  {
    if( miRefIdx[0] != mi.miRefIdx[0] ) return false;
    if( isMotionValid( miRefIdx[0], MI_NOT_VALID ) && mv[0] != mi.mv[0] ) return false;

    if( miRefIdx[1] != mi.miRefIdx[1] ) return false;
    if( isMotionValid( miRefIdx[1], MI_NOT_VALID ) && mv[1] != mi.mv[1] ) return false;

    return true;
  }

  bool operator!=( const MotionInfo &mi ) const
  {
    return !( *this == mi );
  }

  bool isInter() const
  {
    return interDir() != 0;
  }

  int interDir() const
  {
    int
    interDir  = isMotionInvalid( miRefIdx[0], MI_NOT_VALID ) ? 0 : 1;
    interDir += isMotionInvalid( miRefIdx[1], MI_NOT_VALID ) ? 0 : 2;
    return interDir;
  }
};

struct ColocatedMotionInfo
{
  Mv       mv      [NUM_REF_PIC_LIST_01];
  int8_t   coRefIdx[NUM_REF_PIC_LIST_01] = { CO_NOT_VALID, CO_NOT_VALID };

  int interDir() const
  {
    int
    interDir  = isMotionInvalid( coRefIdx[0], CO_NOT_VALID ) ? 0 : 1;
    interDir += isMotionInvalid( coRefIdx[1], CO_NOT_VALID ) ? 0 : 2;
    return interDir;
  }

  bool isInter() const
  {
    return interDir() != 0;
  }

  ColocatedMotionInfo &operator=( const MotionInfo &rhs )
  {
    static_assert( sizeof( MotionInfo ) == sizeof( ColocatedMotionInfo ), "MotionInfo and ColocatedMotionInfo require the same memory layout" );
    static_assert( std::is_standard_layout<MotionInfo>         ::value &&
                   std::is_standard_layout<ColocatedMotionInfo>::value,
                   "offsetof is only valid for standard layout types" );
    static_assert( offsetof( MotionInfo, mv )       == offsetof( ColocatedMotionInfo, mv ) &&
                   offsetof( MotionInfo, miRefIdx ) == offsetof( ColocatedMotionInfo, coRefIdx ),
                   "MotionInfo and ColocatedMotionInfo require the same memory layout" );

    GCC_WARNING_DISABLE_class_memaccess
    memcpy( this, &rhs, sizeof( MotionInfo ) );
    GCC_WARNING_RESET

    return *this;
  }
};

struct HPMVInfo
{
  Mv       mv      [NUM_REF_PIC_LIST_01];
  int8_t   mhRefIdx[NUM_REF_PIC_LIST_01] = { MH_NOT_VALID, MH_NOT_VALID };

  uint8_t  BcwIdx       = 0;
  bool     useAltHpelIf = false;

  HPMVInfo() = default;
  HPMVInfo( const MotionInfo& mi, uint8_t BcwIdx, bool useAltHpelIf )
  {
    mv[0] = mi.mv[0];
    mv[1] = mi.mv[1];

    mhRefIdx[0] = mi.miRefIdx[0];
    mhRefIdx[1] = mi.miRefIdx[1];

    this->BcwIdx       = BcwIdx;
    this->useAltHpelIf = useAltHpelIf;
  }

  bool operator==( const HPMVInfo& mi ) const
  {
    if( mhRefIdx[0] != mi.mhRefIdx[0] ) return false;
    if( isMotionValid( mhRefIdx[0], MH_NOT_VALID ) && mv[0] != mi.mv[0] ) return false;

    if( mhRefIdx[1] != mi.mhRefIdx[1] ) return false;
    if( isMotionValid (mhRefIdx[1], MH_NOT_VALID ) && mv[1] != mi.mv[1] ) return false;

    return true;
  }

  bool operator!=( const HPMVInfo& mi ) const
  {
    return !( *this == mi );
  }

  int interDir() const
  {
    int
    interDir  = isMotionInvalid( mhRefIdx[0], MH_NOT_VALID ) ? 0 : 1;
    interDir += isMotionInvalid( mhRefIdx[1], MH_NOT_VALID ) ? 0 : 2;
    return interDir;
  }
};

struct MotionHist
{
  static_vector<HPMVInfo, MAX_NUM_HMVP_CANDS> motionLut;
  static_vector<HPMVInfo, MAX_NUM_HMVP_CANDS> motionLutIbc;
  
  static void addMiToLut( static_vector<HPMVInfo, MAX_NUM_HMVP_CANDS>& lut, const HPMVInfo &mi )
  {
    size_t currCnt = lut.size();

    bool pruned      = false;
    int  sameCandIdx = 0;

    for( int idx = 0; idx < currCnt; idx++ )
    {
      if( lut[idx] == mi )
      {
        sameCandIdx = idx;
        pruned = true;
        break;
      }
    }

    if( pruned || currCnt == lut.capacity() )
    {
      lut.erase( lut.begin() + sameCandIdx );
    }

    lut.push_back( mi );
  }
};

}
