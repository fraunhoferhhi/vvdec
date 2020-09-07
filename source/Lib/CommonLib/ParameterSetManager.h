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

      if( !mapData.parameterSet->m_changedFlag )
      {
        // just keep the old one
        delete ps;
        return;
      }

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
