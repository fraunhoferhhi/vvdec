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

#include "ParameterSetManager.h"

void updateParameterSetChangedFlag( bool&                       bChanged,
                                    const std::vector<uint8_t>* pOldData,
                                    const std::vector<uint8_t>* pNewData )
{
  if( bChanged )
  {
    return;
  }

  if( pOldData == pNewData )
  {
    // pointers equal
    bChanged = false;
  }
  else if( pOldData == nullptr || pNewData == nullptr )
  {
    // either one is null, but not both
    bChanged = true;
  }

  // compare the contents
  bChanged = (*pNewData != *pOldData);
}

ParameterSetManager::ActivePSs ParameterSetManager::xActivateParameterSets( const Slice* pSlicePilot, const PicHeader* picHeader )
{
  PPS* pps = getPPS( picHeader->getPPSId() );   // this is a temporary PPS object. Do not store this value
  CHECK( pps == 0, "No PPS present" );

  SPS* sps = getSPS( pps->getSPSId() );   // this is a temporary SPS object. Do not store this value
  CHECK( sps == 0, "No SPS present" );

  if( NULL == pps->pcv )
  {
    delete getPPS( picHeader->getPPSId() )->pcv;
  }

  getPPS( picHeader->getPPSId() )->pcv = new PreCalcValues( *sps, *pps );

  sps->clearChangedFlag();
  pps->clearChangedFlag();

  if( false == activatePPS( picHeader->getPPSId(), pSlicePilot->isIRAP() ) )
  {
    THROW( "Parameter set activation failed!" );
  }

#if JVET_O1143_SUBPIC_BOUNDARY && !JVET_Q0044_SLICE_IDX_WITH_SUBPICS
    PPS* nonconstPPS = getPPS( picHeader->getPPSId() );
    nonconstPPS->initSubPic( *sps );
#endif

  m_alfAPSs.fill( nullptr );
  m_apsMap.clearActive();
  // luma APSs
  for( int i = 0; i < pSlicePilot->getTileGroupApsIdLuma().size(); i++ )
  {
    int apsId = pSlicePilot->getTileGroupApsIdLuma()[i];
    APS* alfApsL = getAPS( apsId, ALF_APS );

    if( alfApsL )
    {
      alfApsL->clearChangedFlag();
      m_alfAPSs[apsId] = alfApsL;
      if( false == activateAPS( apsId, ALF_APS ) )
      {
        THROW( "APS activation failed!" );
      }

      CHECK( sps->getUseCCALF() == false && ( alfApsL->getCcAlfAPSParam().newCcAlfFilter[0] || alfApsL->getCcAlfAPSParam().newCcAlfFilter[1] ),
             "When sps_ccalf_enabled_flag is 0, the values of alf_cc_cb_filter_signal_flag and alf_cc_cr_filter_signal_flag shall be equal to 0" );
      CHECK( sps->getChromaFormatIdc() == CHROMA_400 && alfApsL->chromaPresentFlag,
             "When ChromaArrayType is equal to 0, the value of aps_chroma_present_flag of an ALF_APS shall be equal to 0" );
    }
  }

  if( pSlicePilot->getTileGroupAlfEnabledFlag( COMPONENT_Cb ) || pSlicePilot->getTileGroupAlfEnabledFlag( COMPONENT_Cr ) )
  {
    // chroma APS
    int apsId = pSlicePilot->getTileGroupApsIdChroma();
    APS* alfApsC = getAPS( apsId, ALF_APS );
    if( alfApsC )
    {
      alfApsC->clearChangedFlag();
      m_alfAPSs[apsId] = alfApsC;
      if( false == activateAPS( apsId, ALF_APS ) )
      {
        THROW( "APS activation failed!" );
      }
      CHECK( sps->getUseCCALF() == false && ( alfApsC->getCcAlfAPSParam().newCcAlfFilter[0] || alfApsC->getCcAlfAPSParam().newCcAlfFilter[1] ),
             "When sps_ccalf_enabled_flag is 0, the values of alf_cc_cb_filter_signal_flag and alf_cc_cr_filter_signal_flag shall be equal to 0" );
    }
  }
  if( pSlicePilot->getTileGroupCcAlfCbEnabledFlag() )
  {
    if( !m_alfAPSs[pSlicePilot->getTileGroupCcAlfCbApsId()] )
    {
      int apsId = pSlicePilot->getTileGroupCcAlfCbApsId();
      APS *aps = getAPS( apsId, ALF_APS );
      if( aps )
      {
        m_alfAPSs[apsId] = aps;

        if( false == activateAPS( apsId, ALF_APS ) )
        {
          THROW( "APS activation failed!" );
        }
      }
    }
  }

  if( pSlicePilot->getTileGroupCcAlfCrEnabledFlag() )
  {
    if( !m_alfAPSs[pSlicePilot->getTileGroupCcAlfCrApsId()] )
    {
      int apsId = pSlicePilot->getTileGroupCcAlfCrApsId();
      APS *aps = getAPS( apsId, ALF_APS );
      if( aps )
      {
        m_alfAPSs[apsId] = aps;
        if( false == activateAPS( apsId, ALF_APS ) )
        {
          THROW( "APS activation failed!" );
        }
      }
    }
  }

  APS* lmcsAPS = nullptr;
  if( picHeader->getLmcsAPSId() != -1 )
  {
    lmcsAPS = getAPS( picHeader->getLmcsAPSId(), LMCS_APS );
    CHECK( lmcsAPS == 0, "No LMCS APS present" );
  }

  if( lmcsAPS )
  {
    lmcsAPS->clearChangedFlag();
    if( false == activateAPS( picHeader->getLmcsAPSId(), LMCS_APS ) )
    {
      THROW( "LMCS APS activation failed!" );
    }

    CHECK( sps->getChromaFormatIdc() == CHROMA_400 && lmcsAPS->chromaPresentFlag,
           "When ChromaArrayType is equal to 0, the value of aps_chroma_present_flag of an LMCS_APS shall be equal to 0");
    CHECK( lmcsAPS->getReshaperAPSInfo().maxNbitsNeededDeltaCW - 1 < 0 || lmcsAPS->getReshaperAPSInfo().maxNbitsNeededDeltaCW - 1 > sps->getBitDepth( CHANNEL_TYPE_LUMA ) - 2,
           "The value of lmcs_delta_cw_prec_minus1 of an LMCS_APS shall be in the range of 0 to BitDepth 2, inclusive" );
  }

  APS* scalingListAPS = nullptr;
  if( picHeader->getScalingListAPSId() != -1 )
  {
    scalingListAPS = getAPS( picHeader->getScalingListAPSId(), SCALING_LIST_APS );
    CHECK( scalingListAPS == 0, "No ScalingList APS present" );
  }

  if( scalingListAPS )
  {
    scalingListAPS->clearChangedFlag();

    if( false == activateAPS( picHeader->getScalingListAPSId(), SCALING_LIST_APS ) )
    {
      THROW( "LMCS APS activation failed!" );
    }

    CHECK( ( sps->getChromaFormatIdc() == CHROMA_400 && scalingListAPS->chromaPresentFlag ) || ( sps->getChromaFormatIdc() != CHROMA_400 && !scalingListAPS->chromaPresentFlag ),
           "The value of aps_chroma_present_flag of the APS NAL unit having aps_params_type equal to SCALING_APS and adaptation_parameter_set_id equal to ph_scaling_list_aps_id shall be equal to ChromaArrayType  = =  0 ? 0 : 1" );
  }

  return { sps, pps, &m_alfAPSs, lmcsAPS, scalingListAPS };
}

//! activate a PPS and depending on isIDR parameter also SPS
//! \returns true, if activation is successful
bool ParameterSetManager::activatePPS( int ppsId, bool isIRAP )
{
  PPS* pps = m_ppsMap.getPS( ppsId );
  if( pps )
  {
    int spsId = pps->getSPSId();
    {
      SPS* sps = m_spsMap.getPS( spsId );
      if( sps )
      {
        int vpsId = sps->getVPSId();
        if( vpsId != 0 )
        {
          VPS *vps = m_vpsMap.getPS(vpsId);
          if( vps )
          {
            m_activeVPSId = vpsId;
            m_vpsMap.setActive(vpsId);
          }
          else
          {
            msg( WARNING, "Warning: tried to activate PPS that refers to non-existing VPS." );
          }
        }
        else
        {
          //No actual VPS
          m_activeVPSId = -1;
          m_vpsMap.clearActive();
        }

        m_spsMap.clearActive();
        m_spsMap.setActive( spsId );
        m_activeSPSId = spsId;

        m_ppsMap.clearActive();
        m_ppsMap.setActive( ppsId );
        return true;
      }
      else
      {
        msg( WARNING, "Warning: tried to activate a PPS that refers to a non-existing SPS." );
      }
    }
  }
  else
  {
    msg( WARNING, "Warning: tried to activate non-existing PPS." );
  }

  // Failed to activate if reach here.
  m_activeSPSId = -1;
  return false;
}

template<>
std::vector<PPS*> ParameterSetMap<PPS, MAX_NUM_PPS>::getPPSforSPSId( int spsId )
{
  std::vector<PPS*> ppssforsps;

  for( auto& it : m_paramsetMap )
  {
    if( it.second.parameterSet->getSPSId() == spsId )
    {
      ppssforsps.push_back( it.second.parameterSet.get() );
    }
  }

  return ppssforsps;
}

bool ParameterSetManager::activateAPS( int apsId, int apsType )
{
  APS* aps = m_apsMap.getPS( ( apsId << NUM_APS_TYPE_LEN ) + apsType );
  if( aps )
  {
    m_apsMap.setActive( ( apsId << NUM_APS_TYPE_LEN ) + apsType );
    return true;
  }
  else
  {
    msg( WARNING, "Warning: tried to activate non-existing APS." );
  }
  return false;
}
