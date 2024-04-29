if( VVDEC_ENABLE_LOCAL_BITSTREAM_DOWNLOAD )
  # get bitstreams
  set( BITSTREAM_URL_BASE "https://visvn.fe.hhi.de/download/VVC/FDIS/" )
else()
  # get bitstreams
  set( BITSTREAM_URL_BASE "https://www.itu.int/wftp3/av-arch/jvet-site/bitstream_exchange/VVC/FDIS" )
endif()

# put bitstreams from the conformance set that are should be decodable and are working
list( APPEND BITSTREAM_FILES
  10b400_A_Bytedance_2.zip
  10b400_B_Bytedance_2.zip
  10b444_A_Kwai_3.zip
  10b444_B_Kwai_3.zip
  8b400_A_Bytedance_2.zip
  8b400_B_Bytedance_2.zip
  8b420_A_Bytedance_2.zip
  8b420_B_Bytedance_2.zip
  ACTPIC_A_Huawei_3.zip
  ACTPIC_B_Huawei_3.zip
  ACTPIC_C_Huawei_3.zip
  AFF_A_HUAWEI_2.zip
  AFF_B_HUAWEI_2.zip
  ALF_A_Huawei_3.zip
  ALF_B_Huawei_3.zip
  ALF_C_KDDI_3.zip
  ALF_D_Qualcomm_2.zip
  AMVR_A_HHI_3.zip
  AMVR_B_HHI_3.zip
  APSALF_A_Qualcomm_2.zip
  APSLMCS_A_Dolby_3.zip
  APSLMCS_B_Dolby_3.zip
  APSLMCS_C_Dolby_2.zip
  APSLMCS_D_Dolby_1.zip
  APSLMCS_E_Dolby_1.zip
  APSMULT_A_MediaTek_4.zip
  APSMULT_B_MediaTek_4.zip
  AUD_A_Broadcom_3.zip
  BCW_A_MediaTek_4.zip
  BDOF_A_MediaTek_4.zip
  BDPCM_A_Orange_2.zip
  BOUNDARY_A_Huawei_3.zip
  BUMP_A_LGE_2.zip
  BUMP_B_LGE_2.zip
  BUMP_C_LGE_2.zip
  CCALF_A_Sharp_3.zip
  CCALF_B_Sharp_3.zip
  CCALF_C_Sharp_3.zip
  CCALF_D_Sharp_3.zip
  CCLM_A_KDDI_2.zip
  CIIP_A_MediaTek_4.zip
  CodingToolsSets_A_Tencent_2.zip
  CodingToolsSets_B_Tencent_2.zip
  CodingToolsSets_C_Tencent_2.zip
  CodingToolsSets_D_Tencent_2.zip
  CodingToolsSets_E_Tencent_1.zip
  CROP_A_Panasonic_4.zip
  CROP_B_Panasonic_4.zip
  CST_A_MediaTek_4.zip
  CTU_A_MediaTek_4.zip
  CTU_B_MediaTek_4.zip
  CTU_C_MediaTek_4.zip
  CUBEMAP_A_MediaTek_3.zip
  CUBEMAP_B_MediaTek_3.zip
  CUBEMAP_C_MediaTek_3.zip
  DCI_A_Tencent_3.zip
  DCI_B_Tencent_3.zip
  DEBLOCKING_A_Sharp_3.zip
  DEBLOCKING_B_Sharp_2.zip
  DEBLOCKING_C_Huawei_3.zip
  DEBLOCKING_E_Ericsson_3.zip
  DEBLOCKING_F_Ericsson_2.zip
  DMVR_A_Huawei_3.zip
  DMVR_B_KDDI_4.zip
  DPB_A_Sharplabs_2.zip
  DPB_B_Sharplabs_2.zip
  DQ_A_HHI_3.zip
  DQ_B_HHI_3.zip
  ENT444HIGHTIER_A_Sony_3.zip
  ENT444HIGHTIER_B_Sony_3.zip
  ENT444HIGHTIER_C_Sony_3.zip
  ENT444HIGHTIER_D_Sony_3.zip
  ENT444MAINTIER_A_Sony_3.zip
  ENT444MAINTIER_B_Sony_3.zip
  ENT444MAINTIER_C_Sony_3.zip
  ENT444MAINTIER_D_Sony_3.zip
  ENTHIGHTIER_A_Sony_3.zip
  ENTHIGHTIER_B_Sony_3.zip
  ENTHIGHTIER_C_Sony_3.zip
  ENTHIGHTIER_D_Sony_3.zip
  ENTMAINTIER_A_Sony_3.zip
  ENTMAINTIER_B_Sony_3.zip
  ENTMAINTIER_C_Sony_3.zip
  ENTMAINTIER_D_Sony_3.zip
  ENTROPY_A_Chipsnmedia_2.zip
  ENTROPY_B_Sharp_2.zip
  ENTROPY_C_Qualcomm_1.zip
  ERP_A_MediaTek_3.zip
  FIELD_A_Panasonic_4.zip
  FIELD_B_Panasonic_2.zip
  FILLER_A_Bytedance_1.zip
  GDR_A_ERICSSON_2.zip
  GDR_B_NOKIA_2.zip
  GDR_C_NOKIA_2.zip
  GDR_D_ERICSSON_1.zip
  GPM_A_Alibaba_3.zip
  GPM_B_Alibaba_1.zip
  HLG_A_NHK_4.zip
  HLG_B_NHK_4.zip
  HRD_A_Fujitsu_3.zip
  HRD_B_Fujitsu_2.zip
  IBC_A_Tencent_2.zip
  IBC_B_Tencent_2.zip
  IBC_C_Tencent_2.zip
  IBC_D_Tencent_2.zip
  IBC_E_Tencent_1.zip
  IP_A_Huawei_2.zip
  IP_B_Nokia_1.zip
  ISP_A_HHI_3.zip
  ISP_B_HHI_3.zip
  JCCR_A_Nokia_2.zip
  JCCR_B_Nokia_2.zip
  JCCR_C_HHI_3.zip
  JCCR_D_HHI_3.zip
  JCCR_E_Nokia_1.zip
  JCCR_F_Nokia_1.zip
  LFNST_A_LGE_4.zip
  LFNST_B_LGE_4.zip
  LFNST_C_HHI_3.zip
  LFNST_D_HHI_3.zip
  LMCS_A_Dolby_3.zip
  LMCS_B_Dolby_2.zip
  LMCS_C_Dolby_1.zip
  LOSSLESS_A_HHI_3.zip
  LOSSLESS_B_HHI_3.zip
  LTRP_A_ERICSSON_3.zip
  MERGE_A_Qualcomm_2.zip
  MERGE_B_Qualcomm_2.zip
  MERGE_C_Qualcomm_2.zip
  MERGE_D_Qualcomm_2.zip
  MERGE_E_Qualcomm_2.zip
  MERGE_F_Qualcomm_2.zip
  MERGE_G_Qualcomm_2.zip
  MERGE_H_Qualcomm_2.zip
  MERGE_I_Qualcomm_2.zip
  MERGE_J_Qualcomm_2.zip
  MIP_A_HHI_3.zip
  MIP_B_HHI_3.zip
  MMVD_A_SAMSUNG_3.zip
  MNUT_A_Nokia_4.zip
  MNUT_B_Nokia_3.zip
  MPM_A_LGE_3.zip
  MRLP_A_HHI_2.zip
  MRLP_B_HHI_2.zip
  MTS_A_LGE_4.zip
  MTS_B_LGE_4.zip
  MTS_LFNST_A_LGE_4.zip
  MTS_LFNST_B_LGE_4.zip
  MVCOMP_A_Sharp_2.zip
  PDPC_A_Qualcomm_3.zip
  PDPC_B_Qualcomm_3.zip
  PDPC_C_Qualcomm_2.zip
  PHSH_B_Sharp_1.zip
  PMERGE_A_MediaTek_1.zip
  PMERGE_B_MediaTek_1.zip
  PMERGE_C_MediaTek_1.zip
  PMERGE_D_MediaTek_1.zip
  PMERGE_E_MediaTek_1.zip
  POC_A_Nokia_1.zip
  POUT_A_Sharplabs_2.zip
  PPS_A_Bytedance_1.zip
  PPS_B_Bytedance_1.zip
  PPS_C_Bytedance_1.zip
  PQ_A_Dolby_1.zip
  PROF_A_Interdigital_3.zip
  PROF_B_Interdigital_3.zip
  PSEXT_A_Nokia_2.zip
  PSEXT_B_Nokia_2.zip
  QTBTT_A_MediaTek_4.zip
  QUANT_A_Huawei_2.zip
  QUANT_B_Huawei_2.zip
  QUANT_C_Huawei_2.zip
  QUANT_D_Huawei_4.zip
  QUANT_E_Interdigital_1.zip
  RAP_A_HHI_1.zip
  RAP_B_HHI_1.zip
  RAP_C_HHI_1.zip
  RAP_D_HHI_1.zip
  RPL_A_ERICSSON_2.zip
  RPR_A_Alibaba_4.zip
  RPR_B_Alibaba_3.zip
  RPR_C_Alibaba_3.zip
  RPR_D_Qualcomm_1.zip
  SAO_A_SAMSUNG_3.zip
  SAO_B_SAMSUNG_3.zip
  SAO_C_SAMSUNG_3.zip
  SBT_A_HUAWEI_2.zip
  SbTMVP_A_Bytedance_3.zip
  SbTMVP_B_Bytedance_3.zip
  SCALING_A_InterDigital_1.zip
  SCALING_B_InterDigital_1.zip
  SCALING_C_InterDigital_1.zip
  SDH_A_Dolby_2.zip
  SLICES_A_HUAWEI_3.zip
  SMVD_A_HUAWEI_2.zip
  SPS_A_Bytedance_1.zip
  SPS_B_Bytedance_1.zip
  SPS_C_Bytedance_1.zip
  STILL444_A_KDDI_1.zip
  STILL444_B_ERICSSON_1.zip
  STILL_A_KDDI_1.zip
  STILL_B_ERICSSON_1.zip
  SUBPIC_A_HUAWEI_3.zip
  SUBPIC_B_HUAWEI_3.zip
  SUBPIC_C_ERICSSON_1.zip
  SUBPIC_D_ERICSSON_1.zip
  SUBPIC_E_MediaTek_1.zip
  SUFAPS_A_HHI_1.zip
  TEMPSCAL_A_Panasonic_4.zip
  TEMPSCAL_B_Panasonic_7.zip
  TEMPSCAL_C_Panasonic_4.zip
  TILE_A_Nokia_2.zip
  TILE_B_Nokia_2.zip
  TILE_C_Nokia_2.zip
  TILE_D_Nokia_2.zip
  TILE_E_Nokia_2.zip
  TILE_F_Nokia_2.zip
  TILE_G_Nokia_2.zip
  TMVP_A_Chipsnmedia_3.zip
  TMVP_B_Chipsnmedia_3.zip
  TMVP_C_Chipsnmedia_3.zip
  TMVP_D_Chipsnmedia_3.zip
  TRANS_A_Chipsnmedia_2.zip
  TRANS_B_Chipsnmedia_2.zip
  TRANS_C_Chipsnmedia_4.zip
  TRANS_D_Chipsnmedia_4.zip
  TREE_A_HHI_3.zip
  TREE_B_HHI_3.zip
  TREE_C_HHI_3.zip
  VIRTUAL_A_MediaTek_3.zip
  VIRTUAL_B_MediaTek_3.zip
  WP_A_InterDigital_3.zip
  WP_B_InterDigital_3.zip
  WPP_A_Sharp_3.zip
  WPP_B_Sharp_2.zip
  WRAP_A_InterDigital_4.zip
  WRAP_B_InterDigital_4.zip
  WRAP_C_InterDigital_4.zip
  WRAP_D_InterDigital_4.zip
)

# put bitstreams from the conformance set that should be decodable but aren't
list( APPEND BITSTREAM_FAULTY_FILES
)

# put bitstreams from the conformance set that are not supported (wrong profile/level etc)
list( APPEND BITSTREAM_NOT_MAIN10_FILES
  10b422_A_Sony_5.zip                               # MAIN_10_444 and MAIN_10_444_STILL_PICTURE is still experimental.
  10b422_B_Sony_5.zip                               # decodes correctly, when inferring sps_chroma_vertical_collocated_flag to be 0
  10b422_C_Sony_5.zip                               # MAIN_10_444 and MAIN_10_444_STILL_PICTURE is still experimental.
  10b422_D_Sony_5.zip                               # MAIN_10_444 and MAIN_10_444_STILL_PICTURE is still experimental.
  10b422_E_Sony_5.zip                               # MAIN_10_444 and MAIN_10_444_STILL_PICTURE is still experimental.
  10b422_F_Sony_5.zip                               # MAIN_10_444 and MAIN_10_444_STILL_PICTURE is still experimental.
  10b422_G_Sony_5.zip                               # palette mode is not yet supported
  10b422_H_Sony_5.zip                               # palette mode is not yet supported
  10b422_I_Sony_5.zip                               # palette mode is not yet supported
  10b422_J_Sony_5.zip                               # palette mode is not yet supported
  10b422_K_Sony_5.zip                               # palette mode is not yet supported
  10b422_L_Sony_5.zip                               # palette mode is not yet supported
  8b422_A_Sony_5.zip                                # MAIN_10_444 and MAIN_10_444_STILL_PICTURE is still experimental.
  8b422_B_Sony_5.zip                                # decodes correctly, when inferring sps_chroma_vertical_collocated_flag to be 0
  8b422_C_Sony_5.zip                                # MAIN_10_444 and MAIN_10_444_STILL_PICTURE is still experimental.
  8b422_D_Sony_5.zip                                # MAIN_10_444 and MAIN_10_444_STILL_PICTURE is still experimental.
  8b422_E_Sony_5.zip                                # MAIN_10_444 and MAIN_10_444_STILL_PICTURE is still experimental.
  8b422_F_Sony_5.zip                                # MAIN_10_444 and MAIN_10_444_STILL_PICTURE is still experimental.
  8b422_G_Sony_5.zip                                # palette mode is not yet supported
  8b422_H_Sony_5.zip                                # palette mode is not yet supported
  8b422_I_Sony_5.zip                                # palette mode is not yet supported
  8b422_J_Sony_5.zip                                # palette mode is not yet supported
  8b422_K_Sony_5.zip                                # palette mode is not yet supported
  8b422_L_Sony_5.zip                                # palette mode is not yet supported
  8b444_A_Kwai_2.zip
  8b444_B_Kwai_2.zip
  ACT_A_Kwai_3.zip                                  # palette mode is not yet supported
  ACT_B_Kwai_3.zip
  ILRPL_A_Huawei_2.zip                              # multi-layer not supported
  OLS_A_Tencent_5.zip                               # TODO ERROR: In function "parseVPS" in ../../source/Lib/DecoderLib/VLCReader.cpp:2053: needs to be adjusted, e.g. sublayer and independent layer stuff -> see VTM-9.0
  OLS_B_Tencent_5.zip                               # TODO ERROR: In function "parseVPS" in ../../source/Lib/DecoderLib/VLCReader.cpp:2053: needs to be adjusted, e.g. sublayer and independent layer stuff -> see VTM-9.0
  OLS_C_Tencent_5.zip                               # TODO ERROR: In function "parseVPS" in ../../source/Lib/DecoderLib/VLCReader.cpp:2053: needs to be adjusted, e.g. sublayer and independent layer stuff -> see VTM-9.0
  OPI_A_Nokia_1.zip                                 # TODO ERROR: In function "parseVPS" in ../../source/Lib/DecoderLib/VLCReader.cpp:2053: needs to be adjusted, e.g. sublayer and independent layer stuff -> see VTM-9.0
  OPI_B_Nokia_3.zip                                 # TODO ERROR: In function "parseVPS" in ../../source/Lib/DecoderLib/VLCReader.cpp:2053: needs to be adjusted, e.g. sublayer and independent layer stuff -> see VTM-9.0
  PALETTE_A_Alibaba_2.zip                           # palette mode is not yet supported
  PALETTE_B_Alibaba_2.zip                           # palette mode is not yet supported
  PALETTE_C_Alibaba_2.zip                           # palette mode is not yet supported
  PALETTE_D_Alibaba_2.zip                           # palette mode is not yet supported
  PALETTE_E_Alibaba_2.zip                           # palette mode is not yet supported
  SPATSCAL444_A_Qualcomm_2.zip
  SPATSCAL_A_Qualcomm_3.zip                         # TODO ERROR: In function "parseVPS" in ../../source/Lib/DecoderLib/VLCReader.cpp:2053: needs to be adjusted, e.g. sublayer and independent layer stuff -> see VTM-9.0
  VPS_A_INTEL_3.zip                                 # TODO ERROR: In function "parseVPS" in ../../source/Lib/DecoderLib/VLCReader.cpp:2053: needs to be adjusted, e.g. sublayer and independent layer stuff -> see VTM-9.0
  VPS_B_ERICSSON_1.zip                              # TODO ERROR: In function "parseVPS" in ../../source/Lib/DecoderLib/VLCReader.cpp:2053: needs to be adjusted, e.g. sublayer and independent layer stuff -> see VTM-9.0
  VPS_C_ERICSSON_2.zip                              # TODO ERROR: In function "parseVPS" in ../../source/Lib/DecoderLib/VLCReader.cpp:2053: needs to be adjusted, e.g. sublayer and independent layer stuff -> see VTM-9.0
)
