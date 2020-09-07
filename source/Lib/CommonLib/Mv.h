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

/** \file     Mv.h
    \brief    motion vector class (header)
*/

#ifndef __MV__
#define __MV__

#include "CommonDef.h"

class SPS;
class PPS;
struct Position;
struct Size;

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

enum MvPrecision
{
  MV_PRECISION_4PEL     = 0,      // 4-pel
  MV_PRECISION_INT      = 2,      // 1-pel, shift 2 bits from 4-pel
  MV_PRECISION_HALF     = 3,      // 1/2-pel
  MV_PRECISION_QUARTER  = 4,      // 1/4-pel (the precision of regular MV difference signaling), shift 4 bits from 4-pel
  MV_PRECISION_INTERNAL = 6,      // 1/16-pel (the precision of internal MV), shift 6 bits from 4-pel
};

/// basic motion vector class
class Mv
{
private:
  static const MvPrecision m_amvrPrecision[4];
  static const int mvClipPeriod = (1 << MV_BITS);
  static const int halMvClipPeriod = (1 << (MV_BITS - 1));

public:
  int   hor;     ///< horizontal component of motion vector
  int   ver;     ///< vertical component of motion vector

  // ------------------------------------------------------------------------------------------------------------------
  // constructors
  // ------------------------------------------------------------------------------------------------------------------

  Mv(                    ) : hor( 0    ), ver( 0    ) {}
  Mv( int iHor, int iVer ) : hor( iHor ), ver( iVer ) {}

  // ------------------------------------------------------------------------------------------------------------------
  // set
  // ------------------------------------------------------------------------------------------------------------------

  void  set       ( int iHor, int iVer)     { hor = iHor;  ver = iVer; }
  void  setHor    ( int i )                 { hor = i;                 }
  void  setVer    ( int i )                 { ver = i;                 }
  void  setZero   ()                        { hor = ver = 0;           }

  // ------------------------------------------------------------------------------------------------------------------
  // get
  // ------------------------------------------------------------------------------------------------------------------

  int   getHor    () const { return hor;          }
  int   getVer    () const { return ver;          }
  int   getAbsHor () const { return abs( hor );   }
  int   getAbsVer () const { return abs( ver );   }

  // ------------------------------------------------------------------------------------------------------------------
  // operations
  // ------------------------------------------------------------------------------------------------------------------

  const Mv& operator += ( const Mv& _rcMv )
  {
    hor += _rcMv.hor;
    ver += _rcMv.ver;

    return  *this;
  }

  const Mv& operator-= (const Mv& _rcMv)
  {
    hor -= _rcMv.hor;
    ver -= _rcMv.ver;

    return  *this;
  }

  const Mv& operator<<= (const int i)
  {
    hor <<= i;
    ver <<= i;
    return  *this;
  }

  const Mv& operator>>= ( const int i )
  {
    if (i != 0)
    {
      const int offset = (1 << (i - 1));
      hor = (hor + offset - (hor >= 0)) >> i;
      ver = (ver + offset - (ver >= 0)) >> i;
    }
    return  *this;
  }

  const Mv operator - ( const Mv& rcMv ) const
  {
    return Mv( hor - rcMv.hor, ver - rcMv.ver );
  }

  const Mv operator + ( const Mv& rcMv ) const
  {
    return Mv( hor + rcMv.hor, ver + rcMv.ver );
  }

  bool operator== ( const Mv& rcMv ) const
  {
    return ( hor == rcMv.hor && ver == rcMv.ver );
  }

  bool operator!= ( const Mv& rcMv ) const
  {
    return !( *this == rcMv );
  }

  const Mv scaleMv( int iScale ) const
  {
    const int mvx = Clip3( -131072, 131071, (iScale * getHor() + 128 - (iScale * getHor() >= 0)) >> 8);
    const int mvy = Clip3( -131072, 131071, (iScale * getVer() + 128 - (iScale * getVer() >= 0)) >> 8);
    return Mv( mvx, mvy );
  }

  void changePrecision(const MvPrecision& src, const MvPrecision& dst)
  {
    const int shift = (int)dst - (int)src;
    if (shift >= 0)
    {
      *this <<= shift;
    }
    else
    {
      const int rightShift = -shift;
      const int nOffset = 1 << (rightShift - 1);
      hor = hor >= 0 ? (hor + nOffset - 1) >> rightShift : (hor + nOffset) >> rightShift;
      ver = ver >= 0 ? (ver + nOffset - 1) >> rightShift : (ver + nOffset) >> rightShift;
    }
  }

  void changePrecisionAmvr(const int amvr, const MvPrecision& dst)
  {
    changePrecision(m_amvrPrecision[amvr], dst);
  }

  void roundToPrecision(const MvPrecision& src, const MvPrecision& dst)
  {
    changePrecision(src, dst);
    changePrecision(dst, src);
  }

  void roundToAmvrSignalPrecision(const MvPrecision& src, const int amvr)
  {
    roundToPrecision(src, m_amvrPrecision[amvr]);
  }

  Mv getSymmvdMv(const Mv& curMvPred, const Mv& tarMvPred)
  {
    return Mv(tarMvPred.hor - hor + curMvPred.hor, tarMvPred.ver - ver + curMvPred.ver);
  }

  void clipToStorageBitDepth()
  {
    hor = Clip3( -( 1 << 17 ), ( 1 << 17 ) - 1, hor );
    ver = Clip3( -( 1 << 17 ), ( 1 << 17 ) - 1, ver );
  }

  void mvCliptoStorageBitDepth()  // periodic clipping
  {
    hor = (hor + mvClipPeriod) & (mvClipPeriod - 1);
    hor = (hor >= halMvClipPeriod) ? (hor - mvClipPeriod) : hor;
    ver = (ver + mvClipPeriod) & (mvClipPeriod - 1);
    ver = (ver >= halMvClipPeriod) ? (ver - mvClipPeriod) : ver;
  }
};// END CLASS DEFINITION MV

namespace std
{
  template <>
  struct hash<Mv> : public unary_function<Mv, uint64_t>
  {
    uint64_t operator()(const Mv& value) const
    {
      return (((uint64_t)value.hor << 32) + value.ver);
    }
  };
};

#if JVET_R0058
extern void(*clipMv) ( Mv& rcMv, const Position& pos, const struct Size& size, const SPS& sps, const PPS& pps );
void clipMvInPic     ( Mv& rcMv, const Position& pos, const struct Size& size, const SPS& sps, const PPS& pps );
void clipMvInSubpic  ( Mv& rcMv, const Position& pos, const struct Size& size, const SPS& sps, const PPS& pps );
bool wrapClipMv      ( Mv& rcMv, const Position& pos, const struct Size& size, const SPS& sps, const PPS& pps );
#else
void clipMv    ( Mv& rcMv, const Position& pos,                   const SPS& sps, const PPS& pps, const int w = 0, const int h = 0 );
void clipMv    ( int& mvx, int& mvy, const Position& pos,                   const SPS& sps, const PPS& pps, const int w = 0, const int h = 0 );
bool wrapClipMv( Mv& rcMv, const Position& pos, const Size& size, const SPS& sps, const PPS& pps );
bool wrapClipMv( int& mvx, int& mvy, const Position& pos, const Size& size, const SPS& sps, const PPS& pps );
#endif

//#if JVET_O1164_PS
//bool wrapClipMv( Mv& rcMv, const Position& pos, const Size& size, const SPS& sps, const PPS& pps );
//bool wrapClipMv( int& mvx, int& mvy, const Position& pos, const Size& size, const SPS& sps, const PPS& pps );
//#else
//bool wrapClipMv( Mv& rcMv, const Position& pos, const Size& size, const SPS& sps );
//#endif

void roundAffineMv( int& mvx, int& mvy, int nShift );

//! \}

#endif // __MV__
