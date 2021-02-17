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

/** \file     CABACReader.h
 *  \brief    Reader for low level syntax
 */

#ifndef __CABACREADER__
#define __CABACREADER__

#include "BinDecoder.h"

#include "CommonLib/ContextModelling.h"
#include "CommonLib/MotionInfo.h"
#include "CommonLib/UnitPartitioner.h"
#include "CommonLib/TrQuant.h"


class CABACReader
{
public:
  CABACReader( BinDecoder& binDecoder ) : m_BinDecoder( binDecoder ), m_Bitstream( 0 ) {}
  virtual ~CABACReader() {}

public:

  void        initCtxModels             ( Slice&                        slice );
  void        initBitstream             ( InputBitstream*               bitstream )           { m_Bitstream = bitstream; m_BinDecoder.init( m_Bitstream ); }
  const Ctx&  getCtx                    ()                                            const   { return m_BinDecoder.getCtx();  }
  Ctx&        getCtx                    ()                                                    { return m_BinDecoder.getCtx();  }

public:
  // slice segment data (clause 7.3.8.1)
  bool        terminating_bit           ();
  void        remaining_bytes           ( bool                          noTrailingBytesExpected );

  // coding tree unit (clause 7.3.11.2)
  bool        coding_tree_unit          ( CodingStructure&              cs,     Slice* slice, const UnitArea& area, int (&qps)[2], unsigned  ctuRsAddr );
  bool        dt_implicit_qt_split      ( CodingStructure&              cs,     Partitioner& pL, CUCtx& cuCtxL, Partitioner& pC, CUCtx& cuCtxC );

  // sao (clause 7.3.11.3)
  void        sao                       ( CodingStructure&              cs,     unsigned        ctuRsAddr );

  void        readAlfCtuFilterIndex     ( CodingStructure&              cs,     unsigned        ctuRsAddr );

  void        ccAlfFilterControlIdc     ( CodingStructure&              cs, const ComponentID compID, const int curIdx, uint8_t *filterControlIdc, Position lumaPos, int filterCount );

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
  bool        intra_chroma_lmc_mode     ( PredictionUnit&               pu );
  void        intra_chroma_pred_mode    ( PredictionUnit&               pu );
  void        cu_residual               ( CodingUnit&                   cu,     Partitioner&    pm,       CUCtx& cuCtx );
  void        rqt_root_cbf              ( CodingUnit&                   cu );
  void        adaptive_color_transform  ( CodingUnit&                   cu );
  void        sbt_mode                  ( CodingUnit&                   cu );
  void        mip_flag                  ( CodingUnit&                   cu );
  void        mip_pred_mode             ( PredictionUnit&               pu );
  bool        end_of_ctu                ( CodingUnit&                   cu,     CUCtx&          cuCtx );

  // prediction unit (clause 7.3.11.7)
  void        prediction_unit           ( PredictionUnit&               pu );
  void        general_merge_flag        ( PredictionUnit&               pu );
  void        merge_data                ( PredictionUnit&               pu );
  void        affine_flag               ( CodingUnit&                   cu );
  void        subblock_merge_flag       ( CodingUnit&                   cu );
  void        merge_idx                 ( PredictionUnit&               pu );
  void        mmvd_merge_idx            ( PredictionUnit&               pu );
  void        amvr_mode                 ( CodingUnit&                   cu );
  void        affine_amvr_mode          ( CodingUnit&                   cu );
  void        inter_pred_idc            ( PredictionUnit&               pu );
  void        ref_idx                   ( PredictionUnit&               pu,     RefPicList      eRefList );
  void        mvp_flag                  ( PredictionUnit&               pu,     RefPicList      eRefList );
  void        ciip_flag                 ( PredictionUnit&               pu );
  void        smvd_mode                 ( PredictionUnit&               pu );

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
  template<bool checkBnd>
  int         residual_coding_subblock  ( CoeffCodingContext&           cctx,   TCoeffSig*      coeff, const int stateTransTable, int& state, unsigned& signVal, int *&sigPos, unsigned &stateVal );
  void        residual_codingTS         ( TransformUnit&                tu,     ComponentID     compID );
  void        residual_coding_subblockTS( CoeffCodingContext&           cctx,   TCoeffSig*      coeff, CoeffSigBuf dstcoeff, int& maxX, int& maxY );
  void        joint_cb_cr               ( TransformUnit&                tu,     const int cbfMask );

private:
  unsigned    unary_max_symbol          ( unsigned ctxId0, unsigned ctxIdN, unsigned maxSymbol );
  unsigned    unary_max_eqprob          (                                   unsigned maxSymbol );
  unsigned    exp_golomb_eqprob         ( unsigned count );
  unsigned    get_num_bits_read         () { return m_BinDecoder.getNumBitsRead(); }
  unsigned    code_unary_fixed          ( unsigned ctxId, unsigned unary_max, unsigned fixed );

  void        xReadTruncBinCode         ( uint32_t &symbol, uint32_t maxSymbol );

public:
private:
  TCoeffSig       m_cffTmp  [( MAX_TU_SIZE_FOR_PROFILE + 2 ) * ( MAX_TU_SIZE_FOR_PROFILE + 2 )];

  unsigned        m_signVal[256];
  int             m_numSig [256];
  unsigned        m_sub1   [256];
  int             m_blkPos [MAX_TU_SIZE_FOR_PROFILE*MAX_TU_SIZE_FOR_PROFILE];
  BinDecoder&     m_BinDecoder;
  InputBitstream* m_Bitstream;
  Slice*          m_slice;
};


class CABACDecoder
{
public:
  CABACDecoder()
    : m_CABACReaderStd  ( m_BinDecoderStd )
  {}

  CABACReader*                getCABACReader    ()       { return &m_CABACReaderStd; }

private:
  BinDecoder              m_BinDecoderStd;
  CABACReader             m_CABACReaderStd;
};

#endif
