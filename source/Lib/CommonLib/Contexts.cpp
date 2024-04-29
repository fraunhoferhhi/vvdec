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

/** \file     Contexts.cpp
 *  \brief    Classes providing probability descriptions and contexts (also contains context initialization values)
 */

#include "Contexts.h"

#include <algorithm>
#include <cstring>
#include <limits>

namespace vvdec
{

const uint8_t ProbModelTables::m_RenormTable_32[32] =
{
  6,  5,  4,  4,
  3,  3,  3,  3,
  2,  2,  2,  2,
  2,  2,  2,  2,
  1,  1,  1,  1,
  1,  1,  1,  1,
  1,  1,  1,  1,
  1,  1,  1,  1
};


void BinProbModel::init( int qp, int initId )
{
  int slope       =   ( initId >> 3 ) -  4;
  int offset      = ( ( initId  & 7 ) * 18 ) + 1;
  int inistate    = ( ( slope   * ( qp - 16 ) ) >> 1 ) + offset;
  int state_clip  = inistate < 1 ? 1 : inistate > 127 ? 127 : inistate;
  int p1          = ( state_clip << 8 );

  m_state[0]      = p1 & MASK_0;
  m_state[1]      = p1 & MASK_1;
}

CtxSet::CtxSet( std::initializer_list<CtxSet> ctxSets )
{
  uint16_t minOffset = std::numeric_limits<uint16_t>::max();
  uint16_t maxOffset = 0;
  for( auto& set: ctxSets )
  {
    minOffset = std::min<uint16_t>( minOffset, set.Offset );
    maxOffset = std::max<uint16_t>( maxOffset, set.Offset + set.Size );
  }
  Offset = minOffset;
  Size   = maxOffset - minOffset;
}

const std::vector<uint8_t>& ContextSetCfg::getInitTable( unsigned initId )
{
  CHECK( initId >= (unsigned)sm_InitTables.size(),
                     "Invalid initId (" << initId << "), only " << sm_InitTables.size() << " tables defined." );
  return sm_InitTables[initId];
}


CtxSet ContextSetCfg::addCtxSet( std::initializer_list<std::initializer_list<uint8_t>> initSet2d )
{
  const std::size_t startIdx  = sm_InitTables[0].size();
  const std::size_t numValues = ( *initSet2d.begin() ).size();
        std::size_t setId     = 0;
  for( auto& initSet: initSet2d )
  {
    if( setId >= sm_InitTables.size() )
      break;

    CHECK( initSet.size() != numValues,
                       "Number of init values do not match for all sets (" << initSet.size() << " != " << numValues << ")." );

    for( auto& elemIter: initSet )
    {
      sm_InitTables[setId].push_back( elemIter );
    }

    setId++;
  }
  return CtxSet( (uint16_t)startIdx, (uint16_t)numValues );
}


std::array<std::vector<uint8_t>, NUMBER_OF_SLICE_TYPES + 1> ContextSetCfg::sm_InitTables;

// clang-format off
const CtxSet ContextSetCfg::SplitFlag = ContextSetCfg::addCtxSet
({
  {  18,  27,  15,  18,  28,  45,  26,   7,  23, },
  {  11,  35,  53,  12,   6,  30,  13,  15,  31, },
  {  19,  28,  38,  27,  29,  38,  20,  30,  31, },
  {  12,  13,   8,   8,  13,  12,   5,   9,   9, },
});

const CtxSet ContextSetCfg::SplitQtFlag = ContextSetCfg::addCtxSet
({
  {  26,  36,  38,  18,  34,  21, },
  {  20,  14,  23,  18,  19,   6, },
  {  27,   6,  15,  25,  19,  37, },
  {   0,   8,   8,  12,  12,   8, },
});

const CtxSet ContextSetCfg::SplitHvFlag = ContextSetCfg::addCtxSet
({
  {  43,  42,  37,  42,  44, },
  {  43,  35,  37,  34,  52, },
  {  43,  42,  29,  27,  44, },
  {   9,   8,   9,   8,   5, },
});

const CtxSet ContextSetCfg::Split12Flag = ContextSetCfg::addCtxSet
({
  {  28,  29,  28,  29, },
  {  43,  37,  21,  22, },
  {  36,  45,  36,  45, },
  {  12,  13,  12,  13, },
});

const CtxSet ContextSetCfg::ModeConsFlag = ContextSetCfg::addCtxSet
({
  {  25,  20, },
  {  25,  12, },
  { CNU, CNU, },
  {   1,   0, },
});

const CtxSet ContextSetCfg::SkipFlag = ContextSetCfg::addCtxSet
({
  {  57,  60,  46, },
  {  57,  59,  45, },
  {   0,  26,  28, },
  {   5,   4,   8, },
});

const CtxSet ContextSetCfg::MergeFlag = ContextSetCfg::addCtxSet
({
  {   6, },
  {  21, },
  {  26, },
  {   4, },
});

const CtxSet ContextSetCfg::RegularMergeFlag = ContextSetCfg::addCtxSet
({
  {  46,  15, },
  {  38,   7, },
  { CNU, CNU, },
  {   5,   5, },
});

const CtxSet ContextSetCfg::MergeIdx = ContextSetCfg::addCtxSet
({
  {  18, },
  {  20, },
  {  34, },
  {   4, },
});

const CtxSet ContextSetCfg::MmvdFlag = ContextSetCfg::addCtxSet
({
  {  25, },
  {  26, },
  { CNU, },
  {   4, },
});

const CtxSet ContextSetCfg::MmvdMergeIdx = ContextSetCfg::addCtxSet
({
  {  43, },
  {  43, },
  { CNU, },
  {  10, },
});

const CtxSet ContextSetCfg::MmvdStepMvpIdx = ContextSetCfg::addCtxSet
({
  {  59, },
  {  60, },
  { CNU, },
  {   0, },
});

const CtxSet ContextSetCfg::PredMode = ContextSetCfg::addCtxSet
({
  {  40,  35, },
  {  40,  35, },
  { CNU, CNU, },
  {   5,   1, },
});

const CtxSet ContextSetCfg::MultiRefLineIdx = ContextSetCfg::addCtxSet
({
  {  25,  59, },
  {  25,  58, },
  {  25,  60, },
  {   5,   8, },
});

const CtxSet ContextSetCfg::IPredMode[] =
{
  ContextSetCfg::addCtxSet
  ({
    {  44, },
    {  36, },
    {  45, },
    {   6, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  25, },
    {  25, },
    {  34, },
    {   5, },
  })
};

const CtxSet ContextSetCfg::IntraLumaPlanarFlag = ContextSetCfg::addCtxSet
({
  {  13,   6, },
  {  12,  20, },
  {  13,  28, },
  {   1,   5, },
});

const CtxSet ContextSetCfg::CclmModeFlag = ContextSetCfg::addCtxSet
({
  {  26, },
  {  34, },
  {  59, },
  {   4, },
});

const CtxSet ContextSetCfg::CclmModeIdx = ContextSetCfg::addCtxSet
({
  {  27, },
  {  27, },
  {  27, },
  {   9, },
});

const CtxSet ContextSetCfg::MipFlag = ContextSetCfg::addCtxSet
({
  {  56,  57,  50,  26, },
  {  41,  57,  58,  26, },
  {  33,  49,  50,  25, },
  {   9,  10,   9,   6, },
});

const CtxSet ContextSetCfg::DeltaQP = ContextSetCfg::addCtxSet
({
  { CNU, CNU, },
  { CNU, CNU, },
  { CNU, CNU, },
  { DWS, DWS, },
});

const CtxSet ContextSetCfg::InterDir = ContextSetCfg::addCtxSet
({
  {  14,  13,   5,   4,   3,  40, },
  {   7,   6,   5,  12,   4,  40, },
  { CNU, CNU, CNU, CNU, CNU, CNU, },
  {   0,   0,   1,   4,   4,   0, },
});

const CtxSet ContextSetCfg::RefPic = ContextSetCfg::addCtxSet
({
  {   5,  35, },
  {  20,  35, },
  { CNU, CNU, },
  {   0,   4, },
});

const CtxSet ContextSetCfg::SubblockMergeFlag = ContextSetCfg::addCtxSet
({
  {  25,  58,  45, },
  {  48,  57,  44, },
  { CNU, CNU, CNU, },
  {   4,   4,   4, },
});

const CtxSet ContextSetCfg::AffineFlag = ContextSetCfg::addCtxSet
({
  {  19,  13,   6, },
  {  12,  13,  14, },
  { CNU, CNU, CNU, },
  {   4,   0,   0, },
});

const CtxSet ContextSetCfg::AffineType = ContextSetCfg::addCtxSet
({
  {  35, },
  {  35, },
  { CNU, },
  {   4, },
});

const CtxSet ContextSetCfg::AffMergeIdx = ContextSetCfg::addCtxSet
({
  {   4, },
  {   5, },
  { CNU, },
  {   0, },
});

const CtxSet ContextSetCfg::BcwIdx = ContextSetCfg::addCtxSet
({
  {   5, },
  {   4, },
  { CNU, },
  {   1, },
});

const CtxSet ContextSetCfg::Mvd = ContextSetCfg::addCtxSet
({
  {  51,  36, },
  {  44,  43, },
  {  14,  45, },
  {   9,   5, },
});

const CtxSet ContextSetCfg::BDPCMMode = ContextSetCfg::addCtxSet
({
  {  19,  21,   0,  28, },
  {  40,  36,   0,  13, },
  {  19,  35,   1,  27, },
  {   1,   4,   1,   0, },
});

const CtxSet ContextSetCfg::QtRootCbf = ContextSetCfg::addCtxSet
({
  {  12, },
  {   5, },
  {   6, },
  {   4, },
});

const CtxSet ContextSetCfg::ACTFlag = ContextSetCfg::addCtxSet
({
  {  46, },
  {  46, },
  {  52, },
  {   1, },
});

const CtxSet ContextSetCfg::QtCbf[] =
{
  ContextSetCfg::addCtxSet
  ({
    {  15,   6,   5,  14, },
    {  23,   5,  20,   7, },
    {  15,  12,   5,   7, },
    {   5,   1,   8,   9, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  25,  37, },
    {  25,  28, },
    {  12,  21, },
    {   5,   0, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {   9,  36,  45, },
    {  25,  29,  45, },
    {  33,  28,  36, },
    {   2,   1,   0, },
  })
};

const CtxSet ContextSetCfg::SigCoeffGroup[] =
{
  ContextSetCfg::addCtxSet
  ({
    {  25,  45, },
    {  25,  30, },
    {  18,  31, },
    {   8,   5, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  25,  14, },
    {  25,  45, },
    {  25,  15, },
    {   5,   8, },
  })
};

const CtxSet ContextSetCfg::SigFlag[] =
{
  ContextSetCfg::addCtxSet
  ({
    {  17,  41,  49,  36,   1,  49,  50,  37,  48,  51,  58,  45, },
    {  17,  41,  42,  29,  25,  49,  43,  37,  33,  58,  51,  30, },
    {  25,  19,  28,  14,  25,  20,  29,  30,  19,  37,  30,  38, },
    {  12,   9,   9,  10,   9,   9,   9,  10,   8,   8,   8,  10, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {   9,  49,  50,  36,  48,  59,  59,  38, },
    {  17,  34,  35,  21,  41,  59,  60,  38, },
    {  25,  27,  28,  37,  34,  53,  53,  46, },
    {  12,  12,   9,  13,   4,   5,   8,   9, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  26,  45,  53,  46,  49,  54,  61,  39,  35,  39,  39,  39, },
    {  19,  38,  38,  46,  34,  54,  54,  39,   6,  39,  39,  39, },
    {  11,  38,  46,  54,  27,  39,  39,  39,  44,  39,  39,  39, },
    {   9,  13,   8,   8,   8,   8,   8,   5,   8,   0,   0,   0, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  34,  45,  38,  31,  58,  39,  39,  39, },
    {  35,  45,  53,  54,  44,  39,  39,  39, },
    {  19,  46,  38,  39,  52,  39,  39,  39, },
    {   8,  12,  12,   8,   4,   0,   0,   0, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  19,  54,  39,  39,  50,  39,  39,  39,   0,  39,  39,  39, },
    {  19,  39,  54,  39,  19,  39,  39,  39,  56,  39,  39,  39, },
    {  18,  39,  39,  39,  27,  39,  39,  39,   0,  39,  39,  39, },
    {   8,   8,   8,   8,   8,   0,   4,   4,   0,   0,   0,   0, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  34,  38,  54,  39,  41,  39,  39,  39, },
    {  34,  38,  62,  39,  26,  39,  39,  39, },
    {  11,  39,  39,  39,  19,  39,  39,  39, },
    {   8,   8,   8,   8,   4,   0,   0,   0, },
  })
};

const CtxSet ContextSetCfg::ParFlag[] =
{
  ContextSetCfg::addCtxSet
  ({
    {  33,  40,  25,  41,  26,  42,  25,  33,  26,  34,  27,  25,  41,  42,  42,  35,  33,  27,  35,  42,  43, },
    {  18,  17,  33,  18,  26,  42,  25,  33,  26,  42,  27,  25,  34,  42,  42,  35,  26,  27,  42,  20,  20, },
    {  33,  25,  18,  26,  34,  27,  25,  26,  19,  42,  35,  33,  19,  27,  35,  35,  34,  42,  20,  43,  20, },
    {   8,   9,  12,  13,  13,  13,  10,  13,  13,  13,  13,  13,  13,  13,  13,  13,  10,  13,  13,  13,  13, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  33,  25,  26,  34,  19,  27,  33,  42,  43,  35,  43, },
    {  25,  25,  26,  11,  19,  27,  33,  42,  35,  35,  43, },
    {  33,  25,  26,  42,  19,  27,  26,  50,  35,  20,  43, },
    {   8,  12,  12,  12,  13,  13,  13,  13,  13,  13,  13, },
  })
};

const CtxSet ContextSetCfg::GtxFlag[] =
{
  ContextSetCfg::addCtxSet
  ({
    {  25,   0,   0,  17,  25,  26,   0,   9,  25,  33,  19,   0,  25,  33,  26,  20,  25,  33,  27,  35,  22, },
    {  17,   0,   1,  17,  25,  18,   0,   9,  25,  33,  34,   9,  25,  18,  26,  20,  25,  18,  19,  27,  29, },
    {  25,   1,  40,  25,  33,  11,  17,  25,  25,  18,   4,  17,  33,  26,  19,  13,  33,  19,  20,  28,  22, },
    {   1,   5,   9,   9,   9,   6,   5,   9,  10,  10,   9,   9,   9,   9,   9,   9,   6,   8,   9,   9,  10, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  25,   1,  25,  33,  26,  12,  25,  33,  27,  28,  37, },
    {  17,   9,  25,  10,  18,   4,  17,  33,  19,  20,  29, },
    {  40,   9,  25,  18,  26,  35,  25,  26,  35,  28,  37, },
    {   1,   5,   8,   8,   9,   6,   6,   9,   8,   8,   9, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {   0,   0,  33,  34,  35,  21,  25,  34,  35,  28,  29,  40,  42,  43,  29,  30,  49,  36,  37,  45,  38, },
    {   0,  17,  26,  19,  35,  21,  25,  34,  20,  28,  29,  33,  27,  28,  29,  22,  34,  28,  44,  37,  38, },
    {  25,  25,  11,  27,  20,  21,  33,  12,  28,  21,  22,  34,  28,  29,  29,  30,  36,  29,  45,  30,  23, },
    {   9,   5,  10,  13,  13,  10,   9,  10,  13,  13,  13,   9,  10,  10,  10,  13,   8,   9,  10,  10,  13, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {   0,  40,  34,  43,  36,  37,  57,  52,  45,  38,  46, },
    {   0,  25,  19,  20,  13,  14,  57,  44,  30,  30,  23, },
    {  40,  33,  27,  28,  21,  37,  36,  37,  45,  38,  46, },
    {   8,   8,   9,  12,  12,  10,   5,   9,   9,   9,  13, },
  })
};

const CtxSet ContextSetCfg::LastX[] =
{
  ContextSetCfg::addCtxSet
  ({
    {   6,   6,  12,  14,   6,   4,  14,   7,   6,   4,  29,   7,   6,   6,  12,  28,   7,  13,  13,  35, },
    {   6,  13,  12,   6,   6,  12,  14,  14,  13,  12,  29,   7,   6,  13,  36,  28,  14,  13,   5,  26, },
    {  13,   5,   4,  21,  14,   4,   6,  14,  21,  11,  14,   7,  14,   5,  11,  21,  30,  22,  13,  42, },
    {   8,   5,   4,   5,   4,   4,   5,   4,   1,   0,   4,   1,   0,   0,   0,   0,   1,   0,   0,   0, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  19,   5,   4, },
    {  12,   4,  18, },
    {  12,   4,   3, },
    {   5,   4,   4, },
  })
};

const CtxSet ContextSetCfg::LastY[] =
{
  ContextSetCfg::addCtxSet
  ({
    {   5,   5,  20,  13,  13,  19,  21,   6,  12,  12,  14,  14,   5,   4,  12,  13,   7,  13,  12,  41, },
    {   5,   5,  12,   6,   6,   4,   6,  14,   5,  12,  14,   7,  13,   5,  13,  21,  14,  20,  12,  34, },
    {  13,   5,   4,   6,  13,  11,  14,   6,   5,   3,  14,  22,   6,   4,   3,   6,  22,  29,  20,  34, },
    {   8,   5,   8,   5,   5,   4,   5,   5,   4,   0,   5,   4,   1,   0,   0,   1,   4,   0,   0,   0, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  11,   5,  27, },
    {  11,   4,  18, },
    {  12,   4,   3, },
    {   6,   5,   5, },
  })
};

const CtxSet ContextSetCfg::MVPIdx = ContextSetCfg::addCtxSet
({
  {  34, },
  {  34, },
  {  42, },
  {  12, },
});

const CtxSet ContextSetCfg::SmvdFlag = ContextSetCfg::addCtxSet
({
  {  28, },
  {  28, },
  { CNU, },
  {   5, },
});

const CtxSet ContextSetCfg::SaoMergeFlag = ContextSetCfg::addCtxSet
({
  {   2, },
  {  60, },
  {  60, },
  {   0, },
});

const CtxSet ContextSetCfg::SaoTypeIdx = ContextSetCfg::addCtxSet
({
  {   2, },
  {   5, },
  {  13, },
  {   4, },
});

const CtxSet ContextSetCfg::LFNSTIdx = ContextSetCfg::addCtxSet
({
  {  52,  37,  27, },
  {  37,  45,  27, },
  {  28,  52,  42, },
  {   9,   9,  10, },
});

const CtxSet ContextSetCfg::RdpcmFlag = ContextSetCfg::addCtxSet
({
  { CNU, CNU, },
  { CNU, CNU, },
  { CNU, CNU, },
  { DWS, DWS, },
});

const CtxSet ContextSetCfg::RdpcmDir = ContextSetCfg::addCtxSet
({
  { CNU, CNU, },
  { CNU, CNU, },
  { CNU, CNU, },
  { DWS, DWS, },
});

const CtxSet ContextSetCfg::MTSIndex = ContextSetCfg::addCtxSet
({
  {  45,  25,  27,   0,  25,   17, },
  {  45,  40,  27,   0,  25,    9, },
  {  29,   0,  28,   0,  25,    9, },
  {   8,   0,   9,   0,   1,    1, },
});

const CtxSet ContextSetCfg::ISPMode = ContextSetCfg::addCtxSet
({
  {  33,  43, },
  {  33,  36, },
  {  33,  43, },
  {   9,   2, },
});

const CtxSet ContextSetCfg::SbtFlag = ContextSetCfg::addCtxSet
({
  {  41,  57, },
  {  56,  57, },
  { CNU, CNU, },
  {   1,   5, },
});

const CtxSet ContextSetCfg::SbtQuadFlag = ContextSetCfg::addCtxSet
({
  {  42, },
  {  42, },
  { CNU, },
  {  10, },
});

const CtxSet ContextSetCfg::SbtHorFlag = ContextSetCfg::addCtxSet
({
  {  35,  51,  27, },
  {  20,  43,  12, },
  { CNU, CNU, CNU, },
  {   8,   4,   1, },
});

const CtxSet ContextSetCfg::SbtPosFlag = ContextSetCfg::addCtxSet
({
  {  28, },
  {  28, },
  { CNU, },
  {  13, },
});

const CtxSet ContextSetCfg::ChromaQpAdjFlag = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
});

const CtxSet ContextSetCfg::ChromaQpAdjIdc = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
});

const CtxSet ContextSetCfg::ImvFlag = ContextSetCfg::addCtxSet
({
  {  59,  26,  50,  60,  38, },
  {  59,  48,  58,  60,  60, },
  { CNU,  34, CNU, CNU, CNU, },
  {   0,   5,   0,   0,   4, },
});

const CtxSet ContextSetCfg::ctbAlfFlag = ContextSetCfg::addCtxSet
({
  {  33,  52,  46,  25,  61,  54,  25,  61,  54, },
  {  13,  23,  46,   4,  61,  54,  19,  46,  54, },
  {  62,  39,  39,  54,  39,  39,  31,  39,  39, },
  {   0,   0,   0,   4,   0,   0,   1,   0,   0, },
});

const CtxSet ContextSetCfg::ctbAlfAlternative = ContextSetCfg::addCtxSet
({
  {  11,  26, },
  {  20,  12, },
  {  11,  11, },
  {   0,   0, },
});

const CtxSet ContextSetCfg::AlfUseTemporalFilt = ContextSetCfg::addCtxSet
({
  {  46, },
  {  46, },
  {  46, },
  {   0, },
});

const CtxSet ContextSetCfg::CcAlfFilterControlFlag = ContextSetCfg::addCtxSet
({
  {  25,  35,  38,  25,  28,  38, },
  {  18,  21,  38,  18,  21,  38, },
  {  18,  30,  31,  18,  30,  31, },
  {   4,   1,   4,   4,   1,   4, },
});

const CtxSet ContextSetCfg::CiipFlag = ContextSetCfg::addCtxSet
({
  {  57, },
  {  57, },
  { CNU, },
  {   1, },
});

const CtxSet ContextSetCfg::IBCFlag = ContextSetCfg::addCtxSet
({
  {   0,  43,  45, },
  {   0,  57,  44, },
  {  17,  42,  36, },
  {   1,   5,   8, },
});

const CtxSet ContextSetCfg::JointCbCrFlag = ContextSetCfg::addCtxSet
({
  {  42,  43,  52, },
  {  27,  36,  45, },
  {  12,  21,  35, },
  {   1,   1,   0, },
});

const CtxSet ContextSetCfg::TsSigCoeffGroup = ContextSetCfg::addCtxSet
({
  {  18,  35,  45, },
  {  18,  12,  29, },
  {  18,  20,  38, },
  {   5,   8,   8, },
});

const CtxSet ContextSetCfg::TsSigFlag = ContextSetCfg::addCtxSet
({
  {  25,  50,  37, },
  {  40,  35,  44, },
  {  25,  28,  38, },
  {  13,  13,   8, },
});

const CtxSet ContextSetCfg::TsParFlag = ContextSetCfg::addCtxSet
({
  {  11, },
  {   3, },
  {  11, },
  {   6, },
});

const CtxSet ContextSetCfg::TsGtxFlag = ContextSetCfg::addCtxSet
({
  { CNU,   3,   4,   4,   5, },
  { CNU,   2,  10,   3,   3, },
  { CNU,  10,   3,   3,   3, },
  { DWS,   1,   1,   1,   1, },
});

const CtxSet ContextSetCfg::TsLrg1Flag = ContextSetCfg::addCtxSet
({
  {  19,  11,   4,   6, },
  {  18,  11,   4,  28, },
  {  11,   5,   5,  14, },
  {   4,   2,   1,   6, },
});

const CtxSet ContextSetCfg::TsResidualSign = ContextSetCfg::addCtxSet
({
  {  35,  25,  46,  28,  33,  38, },
  {   5,  10,  53,  43,  25,  46, },
  {  12,  17,  46,  28,  25,  46, },
  {   1,   4,   4,   5,   8,   8, },
});
// clang-format on

const unsigned ContextSetCfg::NumberOfContexts = (unsigned)ContextSetCfg::sm_InitTables[0].size();

// combined sets
const CtxSet ContextSetCfg::Sao = { ContextSetCfg::SaoMergeFlag, ContextSetCfg::SaoTypeIdx };

const CtxSet ContextSetCfg::Alf = { ContextSetCfg::ctbAlfFlag, ContextSetCfg::ctbAlfAlternative, ContextSetCfg::AlfUseTemporalFilt };


void Ctx::init(int qp, int initId)
{
  const auto& initTable = ContextSetCfg::getInitTable( initId );
  CHECK( m_CtxBuffer.size() != initTable.size(),
                     "Size of init table (" << initTable.size() << ") does not match size of context buffer (" << m_CtxBuffer.size() << ")." );

  const auto& rateInitTable = ContextSetCfg::getInitTable( NUMBER_OF_SLICE_TYPES );
  CHECK( m_CtxBuffer.size() != rateInitTable.size(),
                     "Size of rate init table (" << rateInitTable.size() << ") does not match size of context buffer (" << m_CtxBuffer.size() << ")." );

  const int clippedQP = Clip3( 0, MAX_QP, qp );
  for( std::size_t k = 0; k < m_CtxBuffer.size(); k++ )
  {
    m_CtxBuffer[k].init( clippedQP, initTable[k] );
    m_CtxBuffer[k].setLog2WindowSize( rateInitTable[k] );
  }
}

}
