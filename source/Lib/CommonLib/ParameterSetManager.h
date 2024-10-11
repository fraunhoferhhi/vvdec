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

#pragma once

#include "CommonDef.h"
#include "Slice.h"

#include <map>
#include <vector>
#include <array>

namespace vvdec
{

template<class T, int MAX_ID>
class ParameterSetMap
{
public:
  struct MapData
  {
    AlignedByteVec     cNaluData;   // Can be empty
    std::shared_ptr<T> parameterSet;
    bool               bChanged = false;
  };

  ~ParameterSetMap()
  {
    m_paramsetMap.clear();
    m_lastActiveParameterSet.reset();
  }

  void storePS( int psId, T* ps, AlignedByteVec&& cNaluData )
  {
    CHECK( psId >= MAX_ID, "Invalid PS id" );
    if( m_paramsetMap.find( psId ) != m_paramsetMap.end() )
    {
      MapData& mapData   = m_paramsetMap[psId];

      // work out changed flag
      mapData.parameterSet->m_changedFlag |= ( mapData.cNaluData != cNaluData );

      // Don't throw away identical parameter sets as VTM does:
      //   The PPS can be identical to a previous one, but the SPS changed, so it needs to be interpreted differently.
      //
      // if( !mapData.parameterSet->m_changedFlag )
      // {
      //   // just keep the old one
      //   delete ps;
      //   return;
      // }

      if( find( m_activePsId.begin(), m_activePsId.end(), psId ) != m_activePsId.end() )
      {
        std::swap( m_paramsetMap[psId].parameterSet, m_lastActiveParameterSet );
      }
      m_paramsetMap[psId].cNaluData.clear();
      m_paramsetMap[psId].parameterSet.reset(ps);
    }
    else
    {
      m_paramsetMap[psId].parameterSet.reset(ps);
      ps->m_changedFlag = false;
    }

    if( !cNaluData.empty() )
    {
      m_paramsetMap[psId].cNaluData = std::move( cNaluData );
    }
    else
    {
      m_paramsetMap[psId].cNaluData.clear();
    }
  }

  T* getPS_nothrow( int psId )
  {
    auto it = m_paramsetMap.find( psId );
    return it == m_paramsetMap.end() ? nullptr : it->second.parameterSet.get();
  }

  const T* getPS_nothrow( int psId ) const
  {
    auto it = m_paramsetMap.find( psId );
    return it == m_paramsetMap.end() ? nullptr : it->second.parameterSet.get();
  }

  T* getPS( int psId )
  {
    T* ps = getPS_nothrow( psId );
    constexpr static int apsIdShift = std::is_same<T, APS>::value ? NUM_APS_TYPE_LEN : 0;
    CHECK( !ps, "Missing Parameter Set (id:" << ( psId >> apsIdShift ) << ')' );
    return ps;
  }

  const T* getPS( int psId ) const
  {
    const T* ps = getPS_nothrow( psId );
    constexpr static int apsIdShift = std::is_same<T, APS>::value ? NUM_APS_TYPE_LEN : 0;
    CHECK( !ps, "Missing Parameter Set (id:" << ( psId >> apsIdShift ) << ')' );
    return ps;
  }

  T* getFirstPS() const
  {
    return ( m_paramsetMap.begin() == m_paramsetMap.end() ) ? NULL : m_paramsetMap.begin()->second.parameterSet.get();
  }

  void setActive( int psId ) { m_activePsId.push_back( psId ); }
  void clearActive()         { m_activePsId.clear(); }
  void clearMap()            { m_paramsetMap.clear(); }

private:
  std::map<int, MapData> m_paramsetMap;
  std::shared_ptr<T>     m_lastActiveParameterSet;
  std::vector<int>       m_activePsId;
};


class ParameterSetManager
{
public:
  ParameterSetManager()  = default;
  ~ParameterSetManager() = default;

  using APSArray = std::array<const APS*, ALF_CTB_MAX_NUM_APS>;

  struct ActivePSs
  {
    const SPS*      sps;
    const PPS*      pps;
          APSArray* alfAPSs;
    const APS*      lmcsAps;
    const APS*      scalingListAps;
  };

  ActivePSs xActivateParameterSets( const bool isFirstSlice, const Slice* pPilot, const PicHeader* picHeader );

  //! store sequence parameter set and take ownership of it
  //! store picture parameter set and take ownership of it
  void                  storeVPS( VPS* vps, AlignedByteVec& naluData ) { m_vpsMap.storePS( vps->getVPSId(),                                             vps, std::move( naluData ) ); }
  void                  storeSPS( SPS* sps, AlignedByteVec& naluData ) { m_spsMap.storePS( sps->getSPSId(),                                             sps, std::move( naluData ) ); }
  void                  storePPS( PPS* pps, AlignedByteVec& naluData ) { m_ppsMap.storePS( pps->getPPSId(),                                             pps, std::move( naluData ) ); }
  void                  storeAPS( APS* aps, AlignedByteVec& naluData ) { m_apsMap.storePS( ( aps->getAPSId() << NUM_APS_TYPE_LEN ) + aps->getAPSType(), aps, std::move( naluData ) ); }

  //! get pointer to existing sequence parameter set
  const VPS*            getVPS        ( int vpsId )              const { if( !vpsId ) return nullptr; return m_vpsMap.getPS( vpsId );              }
  const SPS*            getSPS        ( int spsId )              const { return m_spsMap.getPS        ( spsId );                                   }
  const PPS*            getPPS        ( int ppsId )              const { return m_ppsMap.getPS        ( ppsId );                                   }
  const APS*            getAPS        ( int apsId, int apsType ) const { return m_apsMap.getPS        ( ( apsId << NUM_APS_TYPE_LEN ) + apsType ); }
  const APS*            getAPS_nothrow( int apsId, int apsType ) const { return m_apsMap.getPS_nothrow( ( apsId << NUM_APS_TYPE_LEN ) + apsType ); }
  // getter only used by DecLibParser::prepareLostPicture(). Is it really needed?
  const APSArray&       getAlfAPSs()                             const { return m_alfAPSs; }

  const SPS*            getFirstSPS()                            const { return m_spsMap.getFirstPS();                                     }
  const PPS*            getFirstPPS()                            const { return m_ppsMap.getFirstPS();                                     }
  const APS*            getFirstAPS()                            const { return m_apsMap.getFirstPS();                                     }

  //! activate a PPS and, depending on isIDR parameter, also SPS. returns true, if activation is successful
  bool                  activatePPS( int ppsId, bool isIRAP );
  bool                  activateAPS( int apsId, int apsType );

  const SPS*            getActiveSPS()                           const { return m_spsMap.getPS( m_activeSPSId );                           }

private:
  SPS*                  getSPS( int spsId )                            { return m_spsMap.getPS( spsId );                                   }
  PPS*                  getPPS( int ppsId )                            { return m_ppsMap.getPS( ppsId );                                   }
  APS*                  getAPS( int apsId, int apsType )               { return m_apsMap.getPS( ( apsId << NUM_APS_TYPE_LEN ) + apsType ); }

  ParameterSetMap<SPS, MAX_NUM_SPS>                            m_spsMap;
  ParameterSetMap<PPS, MAX_NUM_PPS>                            m_ppsMap;
  ParameterSetMap<APS, ALF_CTB_MAX_NUM_APS * MAX_NUM_APS_TYPE> m_apsMap;
  ParameterSetMap<VPS, MAX_NUM_VPS>                            m_vpsMap;

  APSArray m_alfAPSs;

  int m_activeSPSId = -1;   // -1 for nothing active
  int m_activeVPSId = -1;   // -1 for nothing active
};

}
