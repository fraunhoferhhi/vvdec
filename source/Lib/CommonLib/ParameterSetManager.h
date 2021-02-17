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

#ifndef PARAM_SET_MANAGER_H
#define PARAM_SET_MANAGER_H

#include "Common.h"
#include "Slice.h"

#include <map>
#include <vector>
#include <array>

void updateParameterSetChangedFlag( bool&                       bChanged,
                                    const std::vector<uint8_t>* pOldData,
                                    const std::vector<uint8_t>* pNewData );


template<class T, int MAX_ID>
class ParameterSetMap
{
public:
  struct MapData
  {
    std::vector<uint8_t> cNaluData;   // Can be empty
    std::shared_ptr<T>   parameterSet;
    bool                 bChanged = false;
  };

  ~ParameterSetMap()
  {
    m_paramsetMap.clear();
    m_lastActiveParameterSet.reset();
  }

  void storePS( int psId, T* ps, const std::vector<uint8_t>* pNaluData )
  {
    CHECK( psId >= MAX_ID, "Invalid PS id" );
    if( m_paramsetMap.find( psId ) != m_paramsetMap.end() )
    {
      MapData& mapData   = m_paramsetMap[psId];

      // work out changed flag
      updateParameterSetChangedFlag( mapData.parameterSet->m_changedFlag, &mapData.cNaluData, pNaluData );

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

    if( pNaluData != 0 )
    {
      m_paramsetMap[psId].cNaluData = std::vector<uint8_t>( *pNaluData );
    }
    else
    {
      m_paramsetMap[psId].cNaluData.clear();
    }
  }

  T* getPS( int psId )
  {
    auto it = m_paramsetMap.find( psId );
    return ( it == m_paramsetMap.end() ) ? NULL : ( it )->second.parameterSet.get();
  }

  const T* getPS( int psId ) const
  {
    auto it = m_paramsetMap.find( psId );
    return ( it == m_paramsetMap.end() ) ? NULL : ( it )->second.parameterSet.get();
  }

  T* getFirstPS()
  {
    return ( m_paramsetMap.begin() == m_paramsetMap.end() ) ? NULL : m_paramsetMap.begin()->second.parameterSet.get();
  }

  void setActive( int psId ) { m_activePsId.push_back( psId ); }
  void clearActive()         { m_activePsId.clear(); }
  void clearMap()            { m_paramsetMap.clear(); }

  std::vector<T*> getPPSforSPSId( int spdId );

private:
  std::map<int, MapData> m_paramsetMap;
  std::shared_ptr<T>     m_lastActiveParameterSet;
  std::vector<int>       m_activePsId;
};


template<>
std::vector<PPS*> ParameterSetMap<PPS, MAX_NUM_PPS>::getPPSforSPSId( int spdId );

class ParameterSetManager
{
public:
  ParameterSetManager()  = default;
  ~ParameterSetManager() = default;

  void           storeVPS( VPS *vps, const std::vector<uint8_t> &naluData )        { m_vpsMap.storePS( vps->getVPSId(), vps, &naluData ); }
  VPS*           getVPS( int vpsId )                                               { return m_vpsMap.getPS( vpsId ); };

  struct ActivePSs
  {
    const SPS*                     sps;
    const PPS*                     pps;
    std::array<APS*, ALF_CTB_MAX_NUM_APS>* alfAPSs;
    APS*                           lmcsAps;
    APS*                           scalingListAps;
  };
  ActivePSs xActivateParameterSets( const Slice* pSlicePilot, const PicHeader* picHeader );
  //! store sequence parameter set and take ownership of it
  void                  storeSPS( SPS* sps, const std::vector<uint8_t>& naluData ) { m_spsMap.storePS( sps->getSPSId(), sps, &naluData ); }
  //! get pointer to existing sequence parameter set
  SPS*                  getSPS( int spsId )                                        { return m_spsMap.getPS( spsId );                      }
  SPS*                  getFirstSPS()                                              { return m_spsMap.getFirstPS();                        }
  //! activate a SPS from a active parameter sets SEI message
  //! \returns true, if activation is successful
  // bool                  activateSPSWithSEI(int SPSId);
  const SPS*            getActiveSPS() const                                       { return m_spsMap.getPS( m_activeSPSId );              }

  //! store picture parameter set and take ownership of it
  void                  storePPS( PPS* pps, const std::vector<uint8_t>& naluData ) { m_ppsMap.storePS( pps->getPPSId(), pps, &naluData ); }
  //! get pointer to existing picture parameter set
  PPS*                  getPPS( int ppsId )                                        { return m_ppsMap.getPS( ppsId );                      }
  PPS*                  getFirstPPS()                                              { return m_ppsMap.getFirstPS();                        }
  //! activate a PPS and depending on isIDR parameter also SPS
  //! \returns true, if activation is successful
  bool                  activatePPS( int ppsId, bool isIRAP );

  // getter only used by DecLibParser::prepareLostPicture(). Is it really needed?
  std::array<APS*, ALF_CTB_MAX_NUM_APS>& getAlfAPSs()                              { return m_alfAPSs; }
  void                  storeAPS( APS* aps, const std::vector<uint8_t>& naluData ) { m_apsMap.storePS( ( aps->getAPSId() << NUM_APS_TYPE_LEN ) + aps->getAPSType(), aps, &naluData ); }
  APS*                  getAPS( int apsId, int apsType )                           { return m_apsMap.getPS( ( apsId << NUM_APS_TYPE_LEN ) + apsType );                                }
  APS*                  getFirstAPS()                                              { return m_apsMap.getFirstPS();                                                                    }
  bool                  activateAPS( int apsId, int apsType );

  std::vector<PPS*>     getPPSforSPSId( int spsId )                                { return m_ppsMap.getPPSforSPSId( spsId ); }

protected:
  ParameterSetMap<SPS, MAX_NUM_SPS>                    m_spsMap;
  ParameterSetMap<PPS, MAX_NUM_PPS>                    m_ppsMap;
  ParameterSetMap<APS, ALF_CTB_MAX_NUM_APS * MAX_NUM_APS_TYPE> m_apsMap;
  ParameterSetMap<VPS, MAX_NUM_VPS>                    m_vpsMap;

  std::array<APS*, ALF_CTB_MAX_NUM_APS> m_alfAPSs;
  
  int m_activeDPSId = -1;   // -1 for nothing active
  int m_activeSPSId = -1;   // -1 for nothing active
  int m_activeVPSId = -1;   // -1 for nothing active
};

#endif   // !PARAM_SET_MANAGER_H
