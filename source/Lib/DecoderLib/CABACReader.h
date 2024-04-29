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

/** \file     CABACReader.h
 *  \brief    Reader for low level syntax
 */

#pragma once

#include "BinDecoder.h"

#include "CommonLib/ContextModelling.h"
#include "CommonLib/UnitPartitioner.h"

namespace vvdec
{

class CABACReader
{
public:
  CABACReader()  = default;
  ~CABACReader() = default;

public:

  void        initCtxModels             ( Slice&                        slice );
  void        initBitstream             ( InputBitstream*               bitstream )
  {
    m_Bitstream = bitstream;
    // Add a byte to prevent readByteFlag from crossing the boundary
    m_Bitstream->inputZeroByte();
    m_BinDecoder.init( m_Bitstream );
  }
  const Ctx&  getCtx                    ()                                            const   { return m_BinDecoder.getCtx();  }
  void        setCtx                    ( const Ctx& ctx )                                    { m_BinDecoder.setCtx( ctx );    }

  // slice segment data (clause 7.3.8.1)
  bool        terminating_bit           ();
  void        remaining_bytes           ( bool                          noTrailingBytesExpected );

  // coding tree unit (clause 7.3.11.2)
  bool        coding_tree_unit          ( CodingStructure&              cs,     Slice* slice, const UnitArea& area, int (&qps)[2], unsigned  ctuRsAddr );
  bool        dt_implicit_qt_split      ( CodingStructure&              cs,     Partitioner& pL, CUCtx& cuCtxL, Partitioner& pC, CUCtx& cuCtxC );

  // sao (clause 7.3.11.3)
  void        sao                       ( CodingStructure&              cs,     unsigned        ctuRsAddr );
  void        readAlf                   ( CodingStructure&              cs,     unsigned        ctuRsAddr, const Partitioner& partitioner);

  short       readAlfCtuFilterIndex     ( CodingStructure&              cs,     unsigned        ctuRsAddr );

  // coding (quad)tree (clause 7.3.11.4)
  bool        coding_tree               ( CodingStructure&              cs,     Partitioner&    pm,       CUCtx& cuCtx );
  PartSplit   split_cu_mode             ( CodingStructure&              cs,     Partitioner&    pm );
  ModeType    mode_constraint           ( CodingStructure&              cs,     Partitioner&    pm,       const PartSplit splitMode );

  // coding unit (clause 7.3.11.5)
  bool        coding_unit               ( CodingUnit&                   cu,     Partitioner&    pm,       CUCtx& cuCtx );
  void        cu_skip_flag              ( CodingUnit&                   cu );
  void        pred_mode                 ( CodingUnit&                   cu );
  void        bdpcm_mode                ( CodingUnit&                   cu,     const ComponentID compID );
  void        cu_pred_data              ( CodingUnit&                   cu );
  void        cu_bcw_flag               ( CodingUnit&                   cu );
  void        extend_ref_line           ( CodingUnit&                   cu );
  void        intra_luma_pred_mode      ( CodingUnit&                   cu );
  bool        intra_chroma_lmc_mode     ( CodingUnit&               cu );
  void        intra_chroma_pred_mode    ( CodingUnit&               cu );
  void        cu_residual               ( CodingUnit&                   cu,     Partitioner&    pm,       CUCtx& cuCtx );
  void        rqt_root_cbf              ( CodingUnit&                   cu );
  void        adaptive_color_transform  ( CodingUnit&                   cu );
  void        sbt_mode                  ( CodingUnit&                   cu );
  void        mip_flag                  ( CodingUnit&                   cu );
  void        mip_pred_mode             ( CodingUnit&               cu );
  bool        end_of_ctu                ( CodingUnit&                   cu,     CUCtx&          cuCtx );

  // prediction unit (clause 7.3.11.7)
  void        prediction_unit           ( CodingUnit&               cu );
  void        general_merge_flag        ( CodingUnit&               cu );
  void        merge_data                ( CodingUnit&               cu );
  void        affine_flag               ( CodingUnit&                   cu );
  void        subblock_merge_flag       ( CodingUnit&                   cu );
  void        merge_idx                 ( CodingUnit&               cu );
  void        mmvd_merge_idx            ( CodingUnit&               cu );
  void        amvr_mode                 ( CodingUnit&                   cu );
  void        affine_amvr_mode          ( CodingUnit&                   cu );
  void        inter_pred_idc            ( CodingUnit&               cu );
  void        ref_idx                   ( CodingUnit&               cu,     RefPicList      eRefList );
  void        mvp_flag                  ( CodingUnit&               cu,     RefPicList      eRefList );
  void        ciip_flag                 ( CodingUnit&               cu );
  void        smvd_mode                 ( CodingUnit&               cu );

  // mvd coding (clause 7.3.11.8)
  void        mvd_coding( Mv &rMvd );

  // transform tree (clause 7.3.11.9)
  void        transform_tree            ( CodingStructure&              cs,     CodingUnit&     cu,       Partitioner&    pm,       CUCtx& cuCtx );
  bool        cbf_comp                  ( CodingUnit&                   cu,     const CompArea& area,     unsigned depth, const bool prevCbCbf = false, const bool useISP = false );

  // transform unit (clause 7.3.11.10)
  void        transform_unit            ( TransformUnit&                tu,     CUCtx&          cuCtx,    Partitioner&    pm );
  void        cu_qp_delta               ( CodingUnit&                   cu,     int             predQP, int8_t& qp );
  void        cu_chroma_qp_offset       ( CodingUnit&                   cu );

  // residual coding (clause 7.3.11.11)
  void        residual_coding           ( TransformUnit&                tu,     ComponentID     compID, CUCtx& cuCtx );
  void        ts_flag                   ( TransformUnit&                tu,     ComponentID     compID );
  void        mts_idx                   ( CodingUnit&                   cu,     CUCtx&          cuCtx  );
  void        residual_lfnst_mode       ( CodingUnit&                   cu,     CUCtx&          cuCtx  );
  void        isp_mode                  ( CodingUnit&                   cu );
  int         last_sig_coeff            ( CoeffCodingContext&           cctx,   TransformUnit& tu, ComponentID   compID );
  int         residual_coding_subblock  ( CoeffCodingContext&           cctx,   TCoeffSig*      coeff, const int stateTransTable, int& state, unsigned& signVal, int *&sigPos, unsigned &stateVal );
  void        residual_codingTS         ( TransformUnit&                tu,     ComponentID     compID );
  void        residual_coding_subblockTS( CoeffCodingContext&           cctx,   TCoeffSig*      coeff, CoeffSigBuf dstcoeff, int& maxX, int& maxY );
  void        joint_cb_cr               ( TransformUnit&                tu,     const int cbfMask );

private:
  unsigned    unary_max_symbol          ( unsigned ctxId0, unsigned ctxIdN, unsigned maxSymbol );
  unsigned    unary_max_eqprob          (                                   unsigned maxSymbol );
  unsigned    exp_golomb_eqprob         ( unsigned count );
#if ENABLE_TRACING
  unsigned    get_num_bits_read         () { return m_BinDecoder.getNumBitsRead(); }
#endif
  unsigned    code_unary_fixed          ( unsigned ctxId, unsigned unary_max, unsigned fixed );

  void        xReadTruncBinCode         ( uint32_t &symbol, uint32_t maxSymbol );

private:
  TCoeffSig       m_cffTmp [MAX_TU_SIZE_FOR_PROFILE * MAX_TU_SIZE_FOR_PROFILE];

  unsigned        m_signVal[256];
  int             m_numSig [256];
  unsigned        m_sub1   [256];
  CtxTpl          m_tplBuf [MAX_TU_SIZE_FOR_PROFILE * MAX_TU_SIZE_FOR_PROFILE];
  int             m_blkPos [MAX_TU_SIZE_FOR_PROFILE * MAX_TU_SIZE_FOR_PROFILE];
  BinDecoder      m_BinDecoder;
  InputBitstream* m_Bitstream = nullptr;
  Slice*          m_slice     = nullptr;
  Partitioner     m_partL, m_partC;
};


}
