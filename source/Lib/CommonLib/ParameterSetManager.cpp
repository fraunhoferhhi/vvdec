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

#include "ParameterSetManager.h"

namespace vvdec
{

ParameterSetManager::ActivePSs ParameterSetManager::xActivateParameterSets( const bool isFirstSlice, const Slice* pSlicePilot, const PicHeader* picHeader )
{
  PPS* pps = getPPS( picHeader->getPPSId() );
  CHECK( pps == 0, "No PPS present" );

  SPS* sps = getSPS( pps->getSPSId() );
  CHECK( sps == 0, "No SPS present" );

  if( isFirstSlice )
  {
    CHECK( !pps->pcv->isCorrect( *sps, *pps ), "PPS has PCV already, but values chaged???" );

    sps->clearChangedFlag();
    pps->clearChangedFlag();

    if( false == activatePPS( picHeader->getPPSId(), pSlicePilot->isIRAP() ) )
    {
      THROW_RECOVERABLE( "Parameter set activation failed!" );
    }

    m_alfAPSs.fill( nullptr );
    m_apsMap.clearActive();
  }

  // luma APSs
  for( int i = 0; i < pSlicePilot->getAlfApsIdsLuma().size(); i++ )
  {
    int apsId = pSlicePilot->getAlfApsIdsLuma()[i];
    APS *alfApsL = getAPS( apsId, ALF_APS );

    if( alfApsL )
    {
      if( isFirstSlice ) alfApsL->clearChangedFlag();
      m_alfAPSs[apsId] = alfApsL;
      if( false == activateAPS( apsId, ALF_APS ) )
      {
        THROW_RECOVERABLE( "APS activation failed!" );
      }

      CHECK( sps->getUseCCALF() == false && ( alfApsL->getCcAlfAPSParam().newCcAlfFilter[0] || alfApsL->getCcAlfAPSParam().newCcAlfFilter[1] ),
             "When sps_ccalf_enabled_flag is 0, the values of alf_cc_cb_filter_signal_flag and alf_cc_cr_filter_signal_flag shall be equal to 0" );
      CHECK( sps->getChromaFormatIdc() == CHROMA_400 && alfApsL->chromaPresentFlag,
             "When ChromaArrayType is equal to 0, the value of aps_chroma_present_flag of an ALF_APS shall be equal to 0" );
    }
  }

  if( pSlicePilot->getAlfEnabledFlag( COMPONENT_Cb ) || pSlicePilot->getAlfEnabledFlag( COMPONENT_Cr ) )
  {
    // chroma APS
    int apsId = pSlicePilot->getAlfApsIdChroma();
    APS* alfApsC = getAPS( apsId , ALF_APS );
    if( alfApsC )
    {
      if( isFirstSlice ) alfApsC->clearChangedFlag();
      m_alfAPSs[apsId] = alfApsC;
      if( false == activateAPS( apsId, ALF_APS ) )
      {
        THROW_RECOVERABLE( "APS activation failed!" );
      }
      CHECK( sps->getUseCCALF() == false && ( alfApsC->getCcAlfAPSParam().newCcAlfFilter[0] || alfApsC->getCcAlfAPSParam().newCcAlfFilter[1] ),
             "When sps_ccalf_enabled_flag is 0, the values of alf_cc_cb_filter_signal_flag and alf_cc_cr_filter_signal_flag shall be equal to 0" );
    }
  }
  if( pSlicePilot->getCcAlfCbEnabledFlag() )
  {
    if( !m_alfAPSs[pSlicePilot->getCcAlfCbApsId()] )
    {
      int apsId = pSlicePilot->getCcAlfCbApsId();
      APS *aps = getAPS( apsId, ALF_APS );
      if( aps )
      {
        m_alfAPSs[apsId] = aps;

        if( false == activateAPS( apsId, ALF_APS ) )
        {
          THROW_RECOVERABLE( "APS activation failed!" );
        }
      }
    }
  }

  if( pSlicePilot->getCcAlfCrEnabledFlag() )
  {
    if( !m_alfAPSs[pSlicePilot->getCcAlfCrApsId()] )
    {
      int apsId = pSlicePilot->getCcAlfCrApsId();
      APS *aps = getAPS( apsId, ALF_APS );
      if( aps )
      {
        m_alfAPSs[apsId] = aps;
        if( false == activateAPS( apsId, ALF_APS ) )
        {
          THROW_RECOVERABLE( "APS activation failed!" );
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
    if( isFirstSlice ) lmcsAPS->clearChangedFlag();
    if( false == activateAPS( picHeader->getLmcsAPSId(), LMCS_APS ) )
    {
      THROW_RECOVERABLE( "LMCS APS activation failed!" );
    }

    CHECK( sps->getChromaFormatIdc() == CHROMA_400 && lmcsAPS->chromaPresentFlag,
           "When ChromaArrayType is equal to 0, the value of aps_chroma_present_flag of an LMCS_APS shall be equal to 0");
    CHECK( lmcsAPS->getReshaperAPSInfo().maxNbitsNeededDeltaCW - 1 < 0 || lmcsAPS->getReshaperAPSInfo().maxNbitsNeededDeltaCW - 1 > sps->getBitDepth() - 2,
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
    if( isFirstSlice ) scalingListAPS->clearChangedFlag();

    if( false == activateAPS( picHeader->getScalingListAPSId(), SCALING_LIST_APS ) )
    {
      THROW_RECOVERABLE( "LMCS APS activation failed!" );
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

}
