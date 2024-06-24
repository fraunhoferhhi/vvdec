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

/* This file is based on VFGS, available on
 * https://github.com/InterDigitalInc/VersatileFilmGrain
 *
 * VFGS implements film grain synthesis as a hardware model: it simulates the
 * output of a cost-effective hardware implementation in a video display
 * pipeline. Also, the C code is split into "fw" (firmware) and "hw" (hardware)
 * parts, and as self-explanatory as possible. See VFGS github repository for
 * more details.
 *
 * The VFGS github repository also contains other tools to experiment with film
 * grain synthesis (e.g. a graphical display and tuning tool for FGC SEI
 * message).
 */

#include "FilmGrain.h"

#include <cstdint>
#include <cstring>
#include <algorithm>

#include "CommonDef.h"

#if defined( TARGET_SIMD_X86 ) && defined( USE_SIMD )
#  include "FilmGrainImplX86.h"
#endif

namespace vvdec
{

// clang-format off
static constexpr int8_t Gaussian_LUT[2048] = {
   -11,  12, 103, -11,  42, -35,  12,  59,  77,  98, -87,   3,  65, -78,  45,  56,
   -51,  21,  13, -11, -20, -19,  33,-127,  17,  -6,-105,  18,  19,  71,  48, -10,
   -38,  42,  -2,  75, -67,  52, -90,  33, -47,  21,  -3, -56,  49,   1, -57, -42,
    -1, 120,-127,-108, -49,   9,  14, 127, 122, 109,  52, 127,   2,   7, 114,  19,
    30,  12,  77, 112,  82, -61,-127, 111, -52, -29,   2, -49, -24,  58, -29, -73,
    12, 112,  67,  79,  -3,-114, -87,  -6,  -5,  40,  58, -81,  49, -27, -31, -34,
  -105,  50,  16, -24, -35, -14, -15,-127, -55, -22, -55,-127,-112,   5, -26, -72,
   127, 127,  -2,  41,  87, -65, -16,  55,  19,  91, -81, -65, -64,  35,  -7, -54,
    99,  -7,  88, 125, -26,  91,   0,  63,  60, -14, -23, 113, -33, 116,  14,  26,
    51, -16, 107,  -8,  53,  38, -34,  17,  -7,   4, -91,   6,  63,  63, -15,  39,
   -36,  19,  55,  17, -51,  40,  33, -37, 126, -39,-118,  17, -30,   0,  19,  98,
    60, 101, -12, -73, -17, -52,  98,   3,   3,  60,  33,  -3,  -2,  10, -42,-106,
   -38,  14, 127,  16,-127, -31, -86, -39, -56,  46, -41,  75,  23, -19, -22, -70,
    74, -54,  -2,  32, -45,  17, -92,  59, -64, -67,  56,-102, -29, -87, -34, -92,
    68,   5, -74, -61,  93, -43,  14, -26, -38,-126, -17,  16,-127,  64,  34,  31,
    93,  17, -51, -59,  71,  77,  81, 127, 127,  61,  33,-106, -93,   0,   0,  75,
   -69,  71, 127, -19,-111,  30,  23,  15,   2,  39,  92,   5,  42,   2,  -6,  38,
    15, 114, -30, -37,  50,  44, 106,  27, 119,   7, -80,  25, -68, -21,  92, -11,
    -1,  18,  41, -50,  79,-127, -43, 127,  18,  11, -21,  32, -52,  27, -88, -90,
   -39, -19, -10,  24,-118,  72, -24, -44,   2,  12,  86,-107,  39, -33,-127,  47,
    51, -24, -22,  46,   0,  15, -35, -69,  -2, -74,  24,  -6,   0,  29,  -3,  45,
    32, -32, 117, -45,  79, -24, -17,-109, -10, -70,  88, -48,  24, -91, 120, -37,
    50,-127,  58,  32, -82, -10, -17,  -7,  46,-127, -15,  89, 127,  17,  98, -39,
   -33,  37,  42, -40, -32, -21, 105, -19,  19,  19, -59,  -9,  30,   0,-127,  34,
   127, -84,  75,  24, -40, -49,-127,-107, -14,  45, -75,   1,  30, -20,  41, -68,
   -40,  12, 127,  -3,   5,  20, -73, -59,-127,  -3,  -3, -53,  -6,-119,  93, 120,
   -80, -50,   0,  20, -46,  67,  78, -12, -22,-127,  36, -41,  56, 119,  -5,-116,
   -22,  68, -14, -90,  24, -82, -44,-127, 107, -25, -37,  40,  -7,  -7, -82,   5,
   -87,  44, -34,   9,-127,  39,  70,  49, -63,  74, -49, 109, -27, -89, -47, -39,
    44,  49,  -4,  60, -42,  80,   9,-127,  -9, -56, -49, 125, -66,  47,  36, 117,
    15, -11, -96, 109,  94, -17, -56,  70,   8, -14,  -5,  50,  37, -45, 120, -30,
   -76,  40, -46,   6,   3,  69,  17, -78,   1, -79,   6, 127,  43,  26, 127,-127,
    28, -55, -26,  55, 112,  48, 107,  -1, -77,  -1,  53,  -9, -22, -43, 123, 108,
   127, 102,  68,  46,   5,   1, 123, -13, -55, -34, -49,  89,  65,-105,  -5,  94,
   -53,  62,  45,  30,  46,  18, -35,  15,  41,  47, -98, -24,  94, -75, 127,-114,
   127, -68,   1, -17,  51, -95,  47,  12,  34, -45, -75,  89,-107,  -9, -58, -29,
  -109, -24, 127, -61, -13,  77, -45,  17,  19,  83, -24,   9, 127, -66,  54,   4,
    26,  13, 111,  43,-113, -22,  10, -24,  83,  67, -14,  75,-123,  59, 127, -12,
    99, -19,  64, -38,  54,   9,   7,  61, -56,   3, -57, 113,-104, -59,   3,  -9,
   -47,  74,  85, -55, -34,  12, 118,  28,  93, -72,  13, -99, -72, -20,  30,  72,
   -94,  19, -54,  64, -12, -63, -25,  65,  72, -10, 127,   0,-127, 103, -20, -73,
  -112,-103,  -6,  28, -42, -21, -59, -29, -26,  19,  -4, -51,  94, -58, -95, -37,
    35,  20, -69, 127, -19,-127, -22,-120, -53,  37,  74,-127,  -1, -12,-119, -53,
   -28,  38,  69,  17,  16,-114,  89,  62,  24,  37, -23,  49,-101, -32,  -9, -95,
   -53,   5,  93, -23, -49,  -8,  51,   3, -75, -90, -10, -39, 127, -86, -22,  20,
    20, 113,  75,  52, -31,  92, -63,   7, -12,  46,  36, 101, -43, -17, -53,  -7,
   -38, -76, -31, -21,  62,  31,  62,  20,-127,  31,  64,  36, 102, -85, -10,  77,
    80,  58, -79,  -8,  35,   8,  80, -24,  -9,   3, -17,  72, 127,  83, -87,  55,
    18,-119,-123,  36,  10, 127,  56, -55, 113,  13,  26,  32, -13, -48,  22, -13,
     5,  58,  27,  24,  26, -11, -36,  37, -92,  78,  81,   9,  51,  14,  67, -13,
     0,  32,  45, -76,  32, -39, -22, -49,-127, -27,  31,  -9,  36,  14,  71,  13,
    57,  12, -53, -86,  53, -44, -35,   2, 127,  12, -66, -44,  46,-115,   3,  10,
    56, -35, 119, -19, -61,  52, -59,-127, -49, -23,   4,  -5,  17, -82,  -6, 127,
    25,  79,  67,  64, -25,  14, -64, -37,-127, -28,  21, -63,  66, -53, -41, 109,
   -62,  15, -22,  13,  29, -63,  20,  27,  95, -44, -59,-116, -10,  79, -49,  22,
   -43, -16,  46, -47,-120, -36, -29, -52, -44,  29, 127, -13,  49,  -9,-127,  75,
   -28, -23,  88,  59,  11, -95,  81, -59,  58,  60, -26,  40, -92,  -3, -22, -58,
   -45, -59, -22, -53,  71, -29,  66, -32, -23,  14, -17, -66, -24, -28, -62,  47,
    38,  17,  16, -37, -24, -11,   8, -27, -19,  59,  45, -49, -47,  -4, -22, -81,
    30, -67,-127,  74, 102,   5, -18,  98,  34, -66,  42, -52,   7, -59,  24, -58,
   -19, -24,-118, -73,  91,  15, -16,  79, -32, -79,-127, -36,  41,  77, -83,   2,
    56,  22, -75, 127, -16, -21,  12,  31,  56,-113,-127,  90,  55,  61,  12,  55,
   -14,-113, -14,  32,  49, -67, -17,  91, -10,   1,  21,  69, -70,  99, -19,-112,
    66, -90, -10,  -9, -71, 127,  50, -81, -49,  24,  61, -61,-111,   7, -41, 127,
    88, -66, 108,-127,  -6,  36, -14,  41, -50,  14,  14,  73,-101, -28,  77, 127,
    -8,-100,  88,  38, 121,  88,-125, -60,  13, -94,-115,  20, -67, -87, -94,-119,
    44, -28, -30,  18,   5, -53, -61,  20, -43,  11, -77, -60,  13,  29,   3,   6,
   -72,  38, -60, -11, 108, -53,  41,  66, -12,-127,-127, -49,  24,  29,  46,  36,
    91,  34, -33, 116, -51, -34, -52,  91,   7, -83,  73, -26,-103,  24, -10,  76,
    84,   5,  68, -80, -13, -17, -32, -48,  20,  50,  26,  10,  63,-104, -14,  37,
   127, 114,  97,  35,   1, -33, -55, 127,-124, -33,  61,  -7, 119, -32,-127, -53,
   -42,  63,   3,  -5, -26,  70, -58, -33, -44, -43,  34, -56,-127, 127,  25, -35,
   -11,  16, -81,  29, -58,  40,-127,-127,  20, -47, -11, -36, -63, -52, -32, -82,
    78, -76, -73,   8,  27, -72,  -9, -74, -85, -86, -57,  25,  78, -10, -97,  35,
   -65,   8, -59,  14,   1, -42,  32, -88, -44,  17,  -3,  -9,  59,  40,  12,-108,
   -40,  24,  34,  18, -28,   2,  51,-110,  -4, 100,   1,  65,  22,   0, 127,  61,
    45,  25, -31,   6,   9,  -7, -48,  99,  16,  44,  -2, -40,  32, -39, -52,  10,
  -110, -19,  56,-127,  69,  26,  51,  92,  40,  61, -52,  45, -38,  13,  85, 122,
    27,  66,  45,-111, -83,  -3,  31,  37,  19, -36,  58,  71,  39, -78, -47,  58,
   -78,   8, -62, -36, -14,  61,  42,-127,  71,  -4,  24, -54,  52,-127,  67,  -4,
   -42,  30, -63,  59,  -3,  -1, -18, -46, -92, -81, -96, -14, -53, -10, -11, -77,
    13,   1,   8, -67,-127, 127, -28,  26, -14,  18, -13, -26,   2,  10, -46, -32,
   -15,  27, -31, -59,  59,  77,-121,  28,  40, -54, -62, -31, -21, -37, -32,  -6,
  -127, -25, -60,  70,-127, 112,-127, 127,  88,  -7, 116, 110,  53,  87,-127,   3,
    16,  23,  74,-106, -51,   3,  74, -82,-112, -74,  65,  81,  25,  53, 127, -45,
   -50,-103, -41, -65, -29,  79, -67,  64, -33, -30,  -8, 127,   0, -13, -51,  67,
   -14,   5, -92,  29, -35,  -8, -90, -57,  -3,  36,  43,  44, -31, -69,  -7,  36,
    39, -51,  43, -81,  58,   6, 127,  12,  57,  66,  46,  59, -43, -42,  41, -15,
  -120,  24,   3, -11,  19, -13,  51,  28,   3,  55, -48, -12,  -1,   2,  97, -19,
    29,  42,  13,  43,  78, -44,  56,-108, -43, -19, 127,  15, -11, -18, -81,  83,
   -37,  77,-109,  15,  65, -50,  43,  12,  13,  27,  28,  61,  57,  30,  26, 106,
   -18,  56,  13,  97,   4,  -8, -62,-103,  94, 108, -44,  52,  27, -47,  -9, 105,
   -53,  46,  89, 103, -33,  38, -34,  55,  51,  70, -94, -35, -87,-107, -19, -31,
     9, -19,  79, -14,  77,   5, -19,-107,  85,  21, -45, -39, -42,   9, -29,  74,
    47, -75,  60,-127, 120,-112, -57, -32,  41,   7,  79,  76,  66,  57,  41, -25,
    31,  37, -47, -36,  43, -73, -37,  63, 127, -69, -52,  90, -33, -61,  60, -55,
    44,  15,   4, -67,  13, -92,  64,  29, -39,  -3,  83,  -2, -38, -85, -86,  58,
    35, -69, -61,  29, -37, -95, -78,   4,  30,  -4, -32, -80, -22,  -9, -77,  46,
     7, -93, -71,  65,   9, -50, 127, -70,  26, -12, -39,-114,  63,-127,-100,   4,
   -32, 111,  22, -60,  65,-101,  26, -42,  21, -59, -27, -74,   2, -94,   6, 126,
     5,  76, -88,  -9, -43,-101, 127,   1, 125,  92, -63,  52,  56,   4,  81,-127,
   127,  80, 127, -29,  30, 116, -74, -17, -57, 105,  48,  45,  25, -72,  48, -38,
  -108,  31, -34,   4, -11,  41,-127,  52,-104, -43, -37,  52,   2,  47,  87,  -9,
    77,  27, -41, -25,  90,  86, -56,  75,  10,  33,  78,  58, 127, 127,  -7, -73,
    49, -33,-106, -35,  38,  57,  53, -17,  -4,  83,  52,-108,  54,-125,  28,  23,
    56, -43, -88, -17,  -6,  47,  23,  -9,   0, -13, 111,  75,  27, -52, -38, -34,
    39,  30,  66,  39,  38, -64,  38,   3,  21, -32, -51, -28,  54, -38, -87,  20,
    52, 115,  18, -81, -70,   0, -14, -46, -46,  -3, 125,  16, -14,  23, -82, -84,
   -69, -20, -65,-127,   9,  81, -49,  61,   7, -36, -45, -42,  57, -26,  47,  20,
   -85,  46, -13,  41, -37, -75, -60,  86, -78,-127,  12,  50,   2,  -3,  13,  47,
     5,  19, -78, -55, -27,  65, -71,  12,-108,  20, -16,  11, -31,  63, -55,  37,
    75, -17, 127, -73, -33, -28,-120, 105,  68, 106,-103,-106,  71,  61,   2,  23,
    -3,  33,  -5, -15, -67, -15, -23, -54,  15, -63,  76,  58,-110,   1,  83, -27,
    22,  75, -39, -17, -11,  64, -17,-127, -54, -66,  31,  96, 116,   3,-114,  -7,
  -108, -63,  97,   9,  50,   8,  75, -28,  72, 112, -36,-112,  95, -50,  23, -13,
   -19,  55,  21,  23,  92,  91,  22, -49,  16, -75,  23,   9, -49, -97, -37,  49,
   -36,  36,-127, -86,  43, 127, -24, -24,  84,  83, -35, -34, -12, 109, 102, -38,
    51, -68,  34,  19, -22,  49, -32, 127,  40,  24, -93,  -4,  -3, 105,   3, -58,
   -18,   8, 127, -18, 125,  68,  69, -62,  30, -36,  54, -57, -24,  17,  43, -36,
   -27, -57, -67, -21, -10, -49,  68,  12,  65,   4,  48,  55, 127, -75,  44,  89,
   -66, -13, -78, -82, -91,  22,  30,  33, -40, -87, -34,  96, -91,  39,  10, -64,
    -3, -12, 127, -50, -37, -56,  23, -35, -36, -54,  90, -91,   2,  50,  77,  -6,
  -127,  16,  46,  -5, -73,   0, -56, -18, -72,  28,  93,  60,  49,  20,  18, 111,
  -111,  32, -83,  47,  47, -10,  35, -88,  43,  57, -98, 127, -17,   0,   1, -39,
  -127,  -2,   0,  63,  93,   0,  36, -66, -61, -19,  39,-127,  58,  50, -17, 127,
    88, -43,-108, -51, -16,   7, -36,  68,  46, -14, 107,  40,  57,   7,  19,   8,
     3,  88, -90, -92, -18, -21, -24,  13,   7,  -4, -78, -91,  -4,   8, -35,  -5,
    19,   2,-111,   4, -66, -81, 122, -20, -34, -37, -84, 127,  68,  46,  17,  47
};

static constexpr uint32_t Seed_LUT[256] = {
   747538460, 1088979410, 1744950180, 1767011913, 1403382928,  521866116, 1060417601, 2110622736,
  1557184770,  105289385,  585624216, 1827676546, 1191843873, 1018104344, 1123590530,  663361569,
  2023850500,   76561770, 1226763489,   80325252, 1992581442,  502705249,  740409860,  516219202,
   557974537, 1883843076,  720112066, 1640137737, 1820967556,   40667586,  155354121, 1820967557,
  1115949072, 1631803309,   98284748,  287433856, 2119719977,  988742797, 1827432592,  579378475,
  1017745956, 1309377032, 1316535465, 2074315269, 1923385360,  209722667, 1546228260,  168102420,
   135274561,  355958469,  248291472, 2127839491,  146920100,  585982612, 1611702337,  696506029,
  1386498192, 1258072451, 1212240548, 1043171860, 1217404993, 1090770605, 1386498193,  169093201,
   541098240, 1468005469,  456510673, 1578687785, 1838217424, 2010752065, 2089828354, 1362717428,
   970073673,  854129835,  714793201, 1266069081, 1047060864, 1991471829, 1098097741,  913883585,
  1669598224, 1337918685, 1219264706, 1799741108, 1834116681,  683417731, 1120274457, 1073098457,
  1648396544,  176642749,   31171789,  718317889, 1266977808, 1400892508,  549749008, 1808010512,
    67112961, 1005669825,  903663673, 1771104465, 1277749632, 1229754427,  950632997, 1979371465,
  2074373264,  305357524, 1049387408, 1171033360, 1686114305, 2147468765, 1941195985,  117709841,
   809550080,  991480851, 1816248997, 1561503561,  329575568,  780651196, 1659144592, 1910793616,
   604016641, 1665084765, 1530186961, 1870928913,  809550081, 2079346113,   71307521,  876663040,
  1073807360,  832356664, 1573927377,  204073344, 2026918147, 1702476788, 2043881033,   57949587,
  2001393952, 1197426649, 1186508931,  332056865,  950043140,  890043474,  349099312,  148914948,
   236204097, 2022643605, 1441981517,  498130129, 1443421481,  924216797, 1817491777, 1913146664,
  1411989632,  929068432,  495735097, 1684636033, 1284520017,  432816184, 1344884865,  210843729,
   676364544,  234449232,   12112337, 1350619139, 1753272996, 2037118872, 1408560528,  533334916,
  1043640385,  357326099,  201376421,  110375493,  541106497,  416159637,  242512193,  777294080,
  1614872576, 1535546636,  870600145,  910810409, 1821440209, 1605432464, 1145147393,  951695441,
  1758494976, 1506656568, 1557150160,  608221521, 1073840384,  217672017,  684818688, 1750138880,
    16777217,  677990609,  953274371, 1770050213, 1359128393, 1797602707, 1984616737, 1865815816,
  2120835200, 2051677060, 1772234061, 1579794881, 1652821009, 1742099468, 1887260865,   46468113,
  1011925248, 1134107920,  881643832, 1354774993,  472508800, 1892499769, 1752793472, 1962502272,
   687898625,  883538000, 1354355153, 1761673473,  944820481, 2020102353,   22020353,  961597696,
  1342242816,  964808962, 1355809701,   17016649, 1386540177,  647682692, 1849012289,  751668241,
  1557184768,  127374604, 1927564752, 1045744913, 1614921984,   43588881, 1016185088, 1544617984,
  1090519041,  136122424,  215038417, 1563027841, 2026918145, 1688778833,  701530369, 1372639488,
  1342242817, 2036945104,  953274369, 1750192384,   16842753,  964808960, 1359020032, 1358954497
};

#define DEFINE_DCT2_P64_MATRIX(aa, ab, ac, ad, ae, af, ag, ah, ai, aj, ak, al, am, an, ao, ap, aq, ar, as, at, au, av, aw, ax, ay, az, ba, bb, bc, bd, be, bf, bg, bh, bi, bj, bk, bl, bm, bn, bo, bp, bq, br, bs, bt, bu, bv, bw, bx, by, bz, ca, cb, cc, cd, ce, cf, cg, ch, ci, cj, ck) \
{ \
  { aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa,  aa }, \
  { bf,  bg,  bh,  bi,  bj,  bk,  bl,  bm,  bn,  bo,  bp,  bq,  br,  bs,  bt,  bu,  bv,  bw,  bx,  by,  bz,  ca,  cb,  cc,  cd,  ce,  cf,  cg,  ch,  ci,  cj,  ck, -ck, -cj, -ci, -ch, -cg, -cf, -ce, -cd, -cc, -cb, -ca, -bz, -by, -bx, -bw, -bv, -bu, -bt, -bs, -br, -bq, -bp, -bo, -bn, -bm, -bl, -bk, -bj, -bi, -bh, -bg, -bf }, \
  { ap,  aq,  ar,  as,  at,  au,  av,  aw,  ax,  ay,  az,  ba,  bb,  bc,  bd,  be, -be, -bd, -bc, -bb, -ba, -az, -ay, -ax, -aw, -av, -au, -at, -as, -ar, -aq, -ap, -ap, -aq, -ar, -as, -at, -au, -av, -aw, -ax, -ay, -az, -ba, -bb, -bc, -bd, -be,  be,  bd,  bc,  bb,  ba,  az,  ay,  ax,  aw,  av,  au,  at,  as,  ar,  aq,  ap }, \
  { bg,  bj,  bm,  bp,  bs,  bv,  by,  cb,  ce,  ch,  ck, -ci, -cf, -cc, -bz, -bw, -bt, -bq, -bn, -bk, -bh, -bf, -bi, -bl, -bo, -br, -bu, -bx, -ca, -cd, -cg, -cj,  cj,  cg,  cd,  ca,  bx,  bu,  br,  bo,  bl,  bi,  bf,  bh,  bk,  bn,  bq,  bt,  bw,  bz,  cc,  cf,  ci, -ck, -ch, -ce, -cb, -by, -bv, -bs, -bp, -bm, -bj, -bg }, \
  { ah,  ai,  aj,  ak,  al,  am,  an,  ao, -ao, -an, -am, -al, -ak, -aj, -ai, -ah, -ah, -ai, -aj, -ak, -al, -am, -an, -ao,  ao,  an,  am,  al,  ak,  aj,  ai,  ah,  ah,  ai,  aj,  ak,  al,  am,  an,  ao, -ao, -an, -am, -al, -ak, -aj, -ai, -ah, -ah, -ai, -aj, -ak, -al, -am, -an, -ao,  ao,  an,  am,  al,  ak,  aj,  ai,  ah }, \
  { bh,  bm,  br,  bw,  cb,  cg, -ck, -cf, -ca, -bv, -bq, -bl, -bg, -bi, -bn, -bs, -bx, -cc, -ch,  cj,  ce,  bz,  bu,  bp,  bk,  bf,  bj,  bo,  bt,  by,  cd,  ci, -ci, -cd, -by, -bt, -bo, -bj, -bf, -bk, -bp, -bu, -bz, -ce, -cj,  ch,  cc,  bx,  bs,  bn,  bi,  bg,  bl,  bq,  bv,  ca,  cf,  ck, -cg, -cb, -bw, -br, -bm, -bh }, \
  { aq,  at,  aw,  az,  bc, -be, -bb, -ay, -av, -as, -ap, -ar, -au, -ax, -ba, -bd,  bd,  ba,  ax,  au,  ar,  ap,  as,  av,  ay,  bb,  be, -bc, -az, -aw, -at, -aq, -aq, -at, -aw, -az, -bc,  be,  bb,  ay,  av,  as,  ap,  ar,  au,  ax,  ba,  bd, -bd, -ba, -ax, -au, -ar, -ap, -as, -av, -ay, -bb, -be,  bc,  az,  aw,  at,  aq }, \
  { bi,  bp,  bw,  cd,  ck, -ce, -bx, -bq, -bj, -bh, -bo, -bv, -cc, -cj,  cf,  by,  br,  bk,  bg,  bn,  bu,  cb,  ci, -cg, -bz, -bs, -bl, -bf, -bm, -bt, -ca, -ch,  ch,  ca,  bt,  bm,  bf,  bl,  bs,  bz,  cg, -ci, -cb, -bu, -bn, -bg, -bk, -br, -by, -cf,  cj,  cc,  bv,  bo,  bh,  bj,  bq,  bx,  ce, -ck, -cd, -bw, -bp, -bi }, \
  { ad,  ae,  af,  ag, -ag, -af, -ae, -ad, -ad, -ae, -af, -ag,  ag,  af,  ae,  ad,  ad,  ae,  af,  ag, -ag, -af, -ae, -ad, -ad, -ae, -af, -ag,  ag,  af,  ae,  ad,  ad,  ae,  af,  ag, -ag, -af, -ae, -ad, -ad, -ae, -af, -ag,  ag,  af,  ae,  ad,  ad,  ae,  af,  ag, -ag, -af, -ae, -ad, -ad, -ae, -af, -ag,  ag,  af,  ae,  ad }, \
  { bj,  bs,  cb,  ck, -cc, -bt, -bk, -bi, -br, -ca, -cj,  cd,  bu,  bl,  bh,  bq,  bz,  ci, -ce, -bv, -bm, -bg, -bp, -by, -ch,  cf,  bw,  bn,  bf,  bo,  bx,  cg, -cg, -bx, -bo, -bf, -bn, -bw, -cf,  ch,  by,  bp,  bg,  bm,  bv,  ce, -ci, -bz, -bq, -bh, -bl, -bu, -cd,  cj,  ca,  br,  bi,  bk,  bt,  cc, -ck, -cb, -bs, -bj }, \
  { ar,  aw,  bb, -bd, -ay, -at, -ap, -au, -az, -be,  ba,  av,  aq,  as,  ax,  bc, -bc, -ax, -as, -aq, -av, -ba,  be,  az,  au,  ap,  at,  ay,  bd, -bb, -aw, -ar, -ar, -aw, -bb,  bd,  ay,  at,  ap,  au,  az,  be, -ba, -av, -aq, -as, -ax, -bc,  bc,  ax,  as,  aq,  av,  ba, -be, -az, -au, -ap, -at, -ay, -bd,  bb,  aw,  ar }, \
  { bk,  bv,  cg, -ce, -bt, -bi, -bm, -bx, -ci,  cc,  br,  bg,  bo,  bz,  ck, -ca, -bp, -bf, -bq, -cb,  cj,  by,  bn,  bh,  bs,  cd, -ch, -bw, -bl, -bj, -bu, -cf,  cf,  bu,  bj,  bl,  bw,  ch, -cd, -bs, -bh, -bn, -by, -cj,  cb,  bq,  bf,  bp,  ca, -ck, -bz, -bo, -bg, -br, -cc,  ci,  bx,  bm,  bi,  bt,  ce, -cg, -bv, -bk }, \
  { ai,  al,  ao, -am, -aj, -ah, -ak, -an,  an,  ak,  ah,  aj,  am, -ao, -al, -ai, -ai, -al, -ao,  am,  aj,  ah,  ak,  an, -an, -ak, -ah, -aj, -am,  ao,  al,  ai,  ai,  al,  ao, -am, -aj, -ah, -ak, -an,  an,  ak,  ah,  aj,  am, -ao, -al, -ai, -ai, -al, -ao,  am,  aj,  ah,  ak,  an, -an, -ak, -ah, -aj, -am,  ao,  al,  ai }, \
  { bl,  by, -ck, -bx, -bk, -bm, -bz,  cj,  bw,  bj,  bn,  ca, -ci, -bv, -bi, -bo, -cb,  ch,  bu,  bh,  bp,  cc, -cg, -bt, -bg, -bq, -cd,  cf,  bs,  bf,  br,  ce, -ce, -br, -bf, -bs, -cf,  cd,  bq,  bg,  bt,  cg, -cc, -bp, -bh, -bu, -ch,  cb,  bo,  bi,  bv,  ci, -ca, -bn, -bj, -bw, -cj,  bz,  bm,  bk,  bx,  ck, -by, -bl }, \
  { as,  az, -bd, -aw, -ap, -av, -bc,  ba,  at,  ar,  ay, -be, -ax, -aq, -au, -bb,  bb,  au,  aq,  ax,  be, -ay, -ar, -at, -ba,  bc,  av,  ap,  aw,  bd, -az, -as, -as, -az,  bd,  aw,  ap,  av,  bc, -ba, -at, -ar, -ay,  be,  ax,  aq,  au,  bb, -bb, -au, -aq, -ax, -be,  ay,  ar,  at,  ba, -bc, -av, -ap, -aw, -bd,  az,  as }, \
  { bm,  cb, -cf, -bq, -bi, -bx,  cj,  bu,  bf,  bt,  ci, -by, -bj, -bp, -ce,  cc,  bn,  bl,  ca, -cg, -br, -bh, -bw,  ck,  bv,  bg,  bs,  ch, -bz, -bk, -bo, -cd,  cd,  bo,  bk,  bz, -ch, -bs, -bg, -bv, -ck,  bw,  bh,  br,  cg, -ca, -bl, -bn, -cc,  ce,  bp,  bj,  by, -ci, -bt, -bf, -bu, -cj,  bx,  bi,  bq,  cf, -cb, -bm }, \
  { ab,  ac, -ac, -ab, -ab, -ac,  ac,  ab,  ab,  ac, -ac, -ab, -ab, -ac,  ac,  ab,  ab,  ac, -ac, -ab, -ab, -ac,  ac,  ab,  ab,  ac, -ac, -ab, -ab, -ac,  ac,  ab,  ab,  ac, -ac, -ab, -ab, -ac,  ac,  ab,  ab,  ac, -ac, -ab, -ab, -ac,  ac,  ab,  ab,  ac, -ac, -ab, -ab, -ac,  ac,  ab,  ab,  ac, -ac, -ab, -ab, -ac,  ac,  ab }, \
  { bn,  ce, -ca, -bj, -br, -ci,  bw,  bf,  bv, -cj, -bs, -bi, -bz,  cf,  bo,  bm,  cd, -cb, -bk, -bq, -ch,  bx,  bg,  bu, -ck, -bt, -bh, -by,  cg,  bp,  bl,  cc, -cc, -bl, -bp, -cg,  by,  bh,  bt,  ck, -bu, -bg, -bx,  ch,  bq,  bk,  cb, -cd, -bm, -bo, -cf,  bz,  bi,  bs,  cj, -bv, -bf, -bw,  ci,  br,  bj,  ca, -ce, -bn }, \
  { at,  bc, -ay, -ap, -ax,  bd,  au,  as,  bb, -az, -aq, -aw,  be,  av,  ar,  ba, -ba, -ar, -av, -be,  aw,  aq,  az, -bb, -as, -au, -bd,  ax,  ap,  ay, -bc, -at, -at, -bc,  ay,  ap,  ax, -bd, -au, -as, -bb,  az,  aq,  aw, -be, -av, -ar, -ba,  ba,  ar,  av,  be, -aw, -aq, -az,  bb,  as,  au,  bd, -ax, -ap, -ay,  bc,  at }, \
  { bo,  ch, -bv, -bh, -ca,  cc,  bj,  bt, -cj, -bq, -bm, -cf,  bx,  bf,  by, -ce, -bl, -br, -ck,  bs,  bk,  cd, -bz, -bg, -bw,  cg,  bn,  bp,  ci, -bu, -bi, -cb,  cb,  bi,  bu, -ci, -bp, -bn, -cg,  bw,  bg,  bz, -cd, -bk, -bs,  ck,  br,  bl,  ce, -by, -bf, -bx,  cf,  bm,  bq,  cj, -bt, -bj, -cc,  ca,  bh,  bv, -ch, -bo }, \
  { aj,  ao, -ak, -ai, -an,  al,  ah,  am, -am, -ah, -al,  an,  ai,  ak, -ao, -aj, -aj, -ao,  ak,  ai,  an, -al, -ah, -am,  am,  ah,  al, -an, -ai, -ak,  ao,  aj,  aj,  ao, -ak, -ai, -an,  al,  ah,  am, -am, -ah, -al,  an,  ai,  ak, -ao, -aj, -aj, -ao,  ak,  ai,  an, -al, -ah, -am,  am,  ah,  al, -an, -ai, -ak,  ao,  aj }, \
  { bp,  ck, -bq, -bo, -cj,  br,  bn,  ci, -bs, -bm, -ch,  bt,  bl,  cg, -bu, -bk, -cf,  bv,  bj,  ce, -bw, -bi, -cd,  bx,  bh,  cc, -by, -bg, -cb,  bz,  bf,  ca, -ca, -bf, -bz,  cb,  bg,  by, -cc, -bh, -bx,  cd,  bi,  bw, -ce, -bj, -bv,  cf,  bk,  bu, -cg, -bl, -bt,  ch,  bm,  bs, -ci, -bn, -br,  cj,  bo,  bq, -ck, -bp }, \
  { au, -be, -at, -av,  bd,  as,  aw, -bc, -ar, -ax,  bb,  aq,  ay, -ba, -ap, -az,  az,  ap,  ba, -ay, -aq, -bb,  ax,  ar,  bc, -aw, -as, -bd,  av,  at,  be, -au, -au,  be,  at,  av, -bd, -as, -aw,  bc,  ar,  ax, -bb, -aq, -ay,  ba,  ap,  az, -az, -ap, -ba,  ay,  aq,  bb, -ax, -ar, -bc,  aw,  as,  bd, -av, -at, -be,  au }, \
  { bq, -ci, -bl, -bv,  cd,  bg,  ca, -by, -bi, -cf,  bt,  bn,  ck, -bo, -bs,  cg,  bj,  bx, -cb, -bf, -cc,  bw,  bk,  ch, -br, -bp,  cj,  bm,  bu, -ce, -bh, -bz,  bz,  bh,  ce, -bu, -bm, -cj,  bp,  br, -ch, -bk, -bw,  cc,  bf,  cb, -bx, -bj, -cg,  bs,  bo, -ck, -bn, -bt,  cf,  bi,  by, -ca, -bg, -cd,  bv,  bl,  ci, -bq }, \
  { ae, -ag, -ad, -af,  af,  ad,  ag, -ae, -ae,  ag,  ad,  af, -af, -ad, -ag,  ae,  ae, -ag, -ad, -af,  af,  ad,  ag, -ae, -ae,  ag,  ad,  af, -af, -ad, -ag,  ae,  ae, -ag, -ad, -af,  af,  ad,  ag, -ae, -ae,  ag,  ad,  af, -af, -ad, -ag,  ae,  ae, -ag, -ad, -af,  af,  ad,  ag, -ae, -ae,  ag,  ad,  af, -af, -ad, -ag,  ae }, \
  { br, -cf, -bg, -cc,  bu,  bo, -ci, -bj, -bz,  bx,  bl,  ck, -bm, -bw,  ca,  bi,  ch, -bp, -bt,  cd,  bf,  ce, -bs, -bq,  cg,  bh,  cb, -bv, -bn,  cj,  bk,  by, -by, -bk, -cj,  bn,  bv, -cb, -bh, -cg,  bq,  bs, -ce, -bf, -cd,  bt,  bp, -ch, -bi, -ca,  bw,  bm, -ck, -bl, -bx,  bz,  bj,  ci, -bo, -bu,  cc,  bg,  cf, -br }, \
  { av, -bb, -ap, -bc,  au,  aw, -ba, -aq, -bd,  at,  ax, -az, -ar, -be,  as,  ay, -ay, -as,  be,  ar,  az, -ax, -at,  bd,  aq,  ba, -aw, -au,  bc,  ap,  bb, -av, -av,  bb,  ap,  bc, -au, -aw,  ba,  aq,  bd, -at, -ax,  az,  ar,  be, -as, -ay,  ay,  as, -be, -ar, -az,  ax,  at, -bd, -aq, -ba,  aw,  au, -bc, -ap, -bb,  av }, \
  { bs, -cc, -bi, -cj,  bl,  bz, -bv, -bp,  cf,  bf,  cg, -bo, -bw,  by,  bm, -ci, -bh, -cd,  br,  bt, -cb, -bj, -ck,  bk,  ca, -bu, -bq,  ce,  bg,  ch, -bn, -bx,  bx,  bn, -ch, -bg, -ce,  bq,  bu, -ca, -bk,  ck,  bj,  cb, -bt, -br,  cd,  bh,  ci, -bm, -by,  bw,  bo, -cg, -bf, -cf,  bp,  bv, -bz, -bl,  cj,  bi,  cc, -bs }, \
  { ak, -am, -ai,  ao,  ah,  an, -aj, -al,  al,  aj, -an, -ah, -ao,  ai,  am, -ak, -ak,  am,  ai, -ao, -ah, -an,  aj,  al, -al, -aj,  an,  ah,  ao, -ai, -am,  ak,  ak, -am, -ai,  ao,  ah,  an, -aj, -al,  al,  aj, -an, -ah, -ao,  ai,  am, -ak, -ak,  am,  ai, -ao, -ah, -an,  aj,  al, -al, -aj,  an,  ah,  ao, -ai, -am,  ak }, \
  { bt, -bz, -bn,  cf,  bh,  ck, -bi, -ce,  bo,  by, -bu, -bs,  ca,  bm, -cg, -bg, -cj,  bj,  cd, -bp, -bx,  bv,  br, -cb, -bl,  ch,  bf,  ci, -bk, -cc,  bq,  bw, -bw, -bq,  cc,  bk, -ci, -bf, -ch,  bl,  cb, -br, -bv,  bx,  bp, -cd, -bj,  cj,  bg,  cg, -bm, -ca,  bs,  bu, -by, -bo,  ce,  bi, -ck, -bh, -cf,  bn,  bz, -bt }, \
  { aw, -ay, -au,  ba,  as, -bc, -aq,  be,  ap,  bd, -ar, -bb,  at,  az, -av, -ax,  ax,  av, -az, -at,  bb,  ar, -bd, -ap, -be,  aq,  bc, -as, -ba,  au,  ay, -aw, -aw,  ay,  au, -ba, -as,  bc,  aq, -be, -ap, -bd,  ar,  bb, -at, -az,  av,  ax, -ax, -av,  az,  at, -bb, -ar,  bd,  ap,  be, -aq, -bc,  as,  ba, -au, -ay,  aw }, \
  { bu, -bw, -bs,  by,  bq, -ca, -bo,  cc,  bm, -ce, -bk,  cg,  bi, -ci, -bg,  ck,  bf,  cj, -bh, -ch,  bj,  cf, -bl, -cd,  bn,  cb, -bp, -bz,  br,  bx, -bt, -bv,  bv,  bt, -bx, -br,  bz,  bp, -cb, -bn,  cd,  bl, -cf, -bj,  ch,  bh, -cj, -bf, -ck,  bg,  ci, -bi, -cg,  bk,  ce, -bm, -cc,  bo,  ca, -bq, -by,  bs,  bw, -bu }, \
  { aa, -aa, -aa,  aa,  aa, -aa, -aa,  aa,  aa, -aa, -aa,  aa,  aa, -aa, -aa,  aa,  aa, -aa, -aa,  aa,  aa, -aa, -aa,  aa,  aa, -aa, -aa,  aa,  aa, -aa, -aa,  aa,  aa, -aa, -aa,  aa,  aa, -aa, -aa,  aa,  aa, -aa, -aa,  aa,  aa, -aa, -aa,  aa,  aa, -aa, -aa,  aa,  aa, -aa, -aa,  aa,  aa, -aa, -aa,  aa,  aa, -aa, -aa,  aa }, \
  { bv, -bt, -bx,  br,  bz, -bp, -cb,  bn,  cd, -bl, -cf,  bj,  ch, -bh, -cj,  bf, -ck, -bg,  ci,  bi, -cg, -bk,  ce,  bm, -cc, -bo,  ca,  bq, -by, -bs,  bw,  bu, -bu, -bw,  bs,  by, -bq, -ca,  bo,  cc, -bm, -ce,  bk,  cg, -bi, -ci,  bg,  ck, -bf,  cj,  bh, -ch, -bj,  cf,  bl, -cd, -bn,  cb,  bp, -bz, -br,  bx,  bt, -bv }, \
  { ax, -av, -az,  at,  bb, -ar, -bd,  ap, -be, -aq,  bc,  as, -ba, -au,  ay,  aw, -aw, -ay,  au,  ba, -as, -bc,  aq,  be, -ap,  bd,  ar, -bb, -at,  az,  av, -ax, -ax,  av,  az, -at, -bb,  ar,  bd, -ap,  be,  aq, -bc, -as,  ba,  au, -ay, -aw,  aw,  ay, -au, -ba,  as,  bc, -aq, -be,  ap, -bd, -ar,  bb,  at, -az, -av,  ax }, \
  { bw, -bq, -cc,  bk,  ci, -bf,  ch,  bl, -cb, -br,  bv,  bx, -bp, -cd,  bj,  cj, -bg,  cg,  bm, -ca, -bs,  bu,  by, -bo, -ce,  bi,  ck, -bh,  cf,  bn, -bz, -bt,  bt,  bz, -bn, -cf,  bh, -ck, -bi,  ce,  bo, -by, -bu,  bs,  ca, -bm, -cg,  bg, -cj, -bj,  cd,  bp, -bx, -bv,  br,  cb, -bl, -ch,  bf, -ci, -bk,  cc,  bq, -bw }, \
  { al, -aj, -an,  ah, -ao, -ai,  am,  ak, -ak, -am,  ai,  ao, -ah,  an,  aj, -al, -al,  aj,  an, -ah,  ao,  ai, -am, -ak,  ak,  am, -ai, -ao,  ah, -an, -aj,  al,  al, -aj, -an,  ah, -ao, -ai,  am,  ak, -ak, -am,  ai,  ao, -ah,  an,  aj, -al, -al,  aj,  an, -ah,  ao,  ai, -am, -ak,  ak,  am, -ai, -ao,  ah, -an, -aj,  al }, \
  { bx, -bn, -ch,  bg, -ce, -bq,  bu,  ca, -bk, -ck,  bj, -cb, -bt,  br,  cd, -bh,  ci,  bm, -by, -bw,  bo,  cg, -bf,  cf,  bp, -bv, -bz,  bl,  cj, -bi,  cc,  bs, -bs, -cc,  bi, -cj, -bl,  bz,  bv, -bp, -cf,  bf, -cg, -bo,  bw,  by, -bm, -ci,  bh, -cd, -br,  bt,  cb, -bj,  ck,  bk, -ca, -bu,  bq,  ce, -bg,  ch,  bn, -bx }, \
  { ay, -as, -be,  ar, -az, -ax,  at,  bd, -aq,  ba,  aw, -au, -bc,  ap, -bb, -av,  av,  bb, -ap,  bc,  au, -aw, -ba,  aq, -bd, -at,  ax,  az, -ar,  be,  as, -ay, -ay,  as,  be, -ar,  az,  ax, -at, -bd,  aq, -ba, -aw,  au,  bc, -ap,  bb,  av, -av, -bb,  ap, -bc, -au,  aw,  ba, -aq,  bd,  at, -ax, -az,  ar, -be, -as,  ay }, \
  { by, -bk,  cj,  bn, -bv, -cb,  bh, -cg, -bq,  bs,  ce, -bf,  cd,  bt, -bp, -ch,  bi, -ca, -bw,  bm,  ck, -bl,  bx,  bz, -bj,  ci,  bo, -bu, -cc,  bg, -cf, -br,  br,  cf, -bg,  cc,  bu, -bo, -ci,  bj, -bz, -bx,  bl, -ck, -bm,  bw,  ca, -bi,  ch,  bp, -bt, -cd,  bf, -ce, -bs,  bq,  cg, -bh,  cb,  bv, -bn, -cj,  bk, -by }, \
  { af, -ad,  ag,  ae, -ae, -ag,  ad, -af, -af,  ad, -ag, -ae,  ae,  ag, -ad,  af,  af, -ad,  ag,  ae, -ae, -ag,  ad, -af, -af,  ad, -ag, -ae,  ae,  ag, -ad,  af,  af, -ad,  ag,  ae, -ae, -ag,  ad, -af, -af,  ad, -ag, -ae,  ae,  ag, -ad,  af,  af, -ad,  ag,  ae, -ae, -ag,  ad, -af, -af,  ad, -ag, -ae,  ae,  ag, -ad,  af }, \
  { bz, -bh,  ce,  bu, -bm,  cj,  bp, -br, -ch,  bk, -bw, -cc,  bf, -cb, -bx,  bj, -cg, -bs,  bo,  ck, -bn,  bt,  cf, -bi,  by,  ca, -bg,  cd,  bv, -bl,  ci,  bq, -bq, -ci,  bl, -bv, -cd,  bg, -ca, -by,  bi, -cf, -bt,  bn, -ck, -bo,  bs,  cg, -bj,  bx,  cb, -bf,  cc,  bw, -bk,  ch,  br, -bp, -cj,  bm, -bu, -ce,  bh, -bz }, \
  { az, -ap,  ba,  ay, -aq,  bb,  ax, -ar,  bc,  aw, -as,  bd,  av, -at,  be,  au, -au, -be,  at, -av, -bd,  as, -aw, -bc,  ar, -ax, -bb,  aq, -ay, -ba,  ap, -az, -az,  ap, -ba, -ay,  aq, -bb, -ax,  ar, -bc, -aw,  as, -bd, -av,  at, -be, -au,  au,  be, -at,  av,  bd, -as,  aw,  bc, -ar,  ax,  bb, -aq,  ay,  ba, -ap,  az }, \
  { ca, -bf,  bz,  cb, -bg,  by,  cc, -bh,  bx,  cd, -bi,  bw,  ce, -bj,  bv,  cf, -bk,  bu,  cg, -bl,  bt,  ch, -bm,  bs,  ci, -bn,  br,  cj, -bo,  bq,  ck, -bp,  bp, -ck, -bq,  bo, -cj, -br,  bn, -ci, -bs,  bm, -ch, -bt,  bl, -cg, -bu,  bk, -cf, -bv,  bj, -ce, -bw,  bi, -cd, -bx,  bh, -cc, -by,  bg, -cb, -bz,  bf, -ca }, \
  { am, -ah,  al,  an, -ai,  ak,  ao, -aj,  aj, -ao, -ak,  ai, -an, -al,  ah, -am, -am,  ah, -al, -an,  ai, -ak, -ao,  aj, -aj,  ao,  ak, -ai,  an,  al, -ah,  am,  am, -ah,  al,  an, -ai,  ak,  ao, -aj,  aj, -ao, -ak,  ai, -an, -al,  ah, -am, -am,  ah, -al, -an,  ai, -ak, -ao,  aj, -aj,  ao,  ak, -ai,  an,  al, -ah,  am }, \
  { cb, -bi,  bu,  ci, -bp,  bn, -cg, -bw,  bg, -bz, -cd,  bk, -bs, -ck,  br, -bl,  ce,  by, -bf,  bx,  cf, -bm,  bq, -cj, -bt,  bj, -cc, -ca,  bh, -bv, -ch,  bo, -bo,  ch,  bv, -bh,  ca,  cc, -bj,  bt,  cj, -bq,  bm, -cf, -bx,  bf, -by, -ce,  bl, -br,  ck,  bs, -bk,  cd,  bz, -bg,  bw,  cg, -bn,  bp, -ci, -bu,  bi, -cb }, \
  { ba, -ar,  av, -be, -aw,  aq, -az, -bb,  as, -au,  bd,  ax, -ap,  ay,  bc, -at,  at, -bc, -ay,  ap, -ax, -bd,  au, -as,  bb,  az, -aq,  aw,  be, -av,  ar, -ba, -ba,  ar, -av,  be,  aw, -aq,  az,  bb, -as,  au, -bd, -ax,  ap, -ay, -bc,  at, -at,  bc,  ay, -ap,  ax,  bd, -au,  as, -bb, -az,  aq, -aw, -be,  av, -ar,  ba }, \
  { cc, -bl,  bp, -cg, -by,  bh, -bt,  ck,  bu, -bg,  bx,  ch, -bq,  bk, -cb, -cd,  bm, -bo,  cf,  bz, -bi,  bs, -cj, -bv,  bf, -bw, -ci,  br, -bj,  ca,  ce, -bn,  bn, -ce, -ca,  bj, -br,  ci,  bw, -bf,  bv,  cj, -bs,  bi, -bz, -cf,  bo, -bm,  cd,  cb, -bk,  bq, -ch, -bx,  bg, -bu, -ck,  bt, -bh,  by,  cg, -bp,  bl, -cc }, \
  { ac, -ab,  ab, -ac, -ac,  ab, -ab,  ac,  ac, -ab,  ab, -ac, -ac,  ab, -ab,  ac,  ac, -ab,  ab, -ac, -ac,  ab, -ab,  ac,  ac, -ab,  ab, -ac, -ac,  ab, -ab,  ac,  ac, -ab,  ab, -ac, -ac,  ab, -ab,  ac,  ac, -ab,  ab, -ac, -ac,  ab, -ab,  ac,  ac, -ab,  ab, -ac, -ac,  ab, -ab,  ac,  ac, -ab,  ab, -ac, -ac,  ab, -ab,  ac }, \
  { cd, -bo,  bk, -bz, -ch,  bs, -bg,  bv, -ck, -bw,  bh, -br,  cg,  ca, -bl,  bn, -cc, -ce,  bp, -bj,  by,  ci, -bt,  bf, -bu,  cj,  bx, -bi,  bq, -cf, -cb,  bm, -bm,  cb,  cf, -bq,  bi, -bx, -cj,  bu, -bf,  bt, -ci, -by,  bj, -bp,  ce,  cc, -bn,  bl, -ca, -cg,  br, -bh,  bw,  ck, -bv,  bg, -bs,  ch,  bz, -bk,  bo, -cd }, \
  { bb, -au,  aq, -ax,  be,  ay, -ar,  at, -ba, -bc,  av, -ap,  aw, -bd, -az,  as, -as,  az,  bd, -aw,  ap, -av,  bc,  ba, -at,  ar, -ay, -be,  ax, -aq,  au, -bb, -bb,  au, -aq,  ax, -be, -ay,  ar, -at,  ba,  bc, -av,  ap, -aw,  bd,  az, -as,  as, -az, -bd,  aw, -ap,  av, -bc, -ba,  at, -ar,  ay,  be, -ax,  aq, -au,  bb }, \
  { ce, -br,  bf, -bs,  cf,  cd, -bq,  bg, -bt,  cg,  cc, -bp,  bh, -bu,  ch,  cb, -bo,  bi, -bv,  ci,  ca, -bn,  bj, -bw,  cj,  bz, -bm,  bk, -bx,  ck,  by, -bl,  bl, -by, -ck,  bx, -bk,  bm, -bz, -cj,  bw, -bj,  bn, -ca, -ci,  bv, -bi,  bo, -cb, -ch,  bu, -bh,  bp, -cc, -cg,  bt, -bg,  bq, -cd, -cf,  bs, -bf,  br, -ce }, \
  { an, -ak,  ah, -aj,  am,  ao, -al,  ai, -ai,  al, -ao, -am,  aj, -ah,  ak, -an, -an,  ak, -ah,  aj, -am, -ao,  al, -ai,  ai, -al,  ao,  am, -aj,  ah, -ak,  an,  an, -ak,  ah, -aj,  am,  ao, -al,  ai, -ai,  al, -ao, -am,  aj, -ah,  ak, -an, -an,  ak, -ah,  aj, -am, -ao,  al, -ai,  ai, -al,  ao,  am, -aj,  ah, -ak,  an }, \
  { cf, -bu,  bj, -bl,  bw, -ch, -cd,  bs, -bh,  bn, -by,  cj,  cb, -bq,  bf, -bp,  ca,  ck, -bz,  bo, -bg,  br, -cc, -ci,  bx, -bm,  bi, -bt,  ce,  cg, -bv,  bk, -bk,  bv, -cg, -ce,  bt, -bi,  bm, -bx,  ci,  cc, -br,  bg, -bo,  bz, -ck, -ca,  bp, -bf,  bq, -cb, -cj,  by, -bn,  bh, -bs,  cd,  ch, -bw,  bl, -bj,  bu, -cf }, \
  { bc, -ax,  as, -aq,  av, -ba, -be,  az, -au,  ap, -at,  ay, -bd, -bb,  aw, -ar,  ar, -aw,  bb,  bd, -ay,  at, -ap,  au, -az,  be,  ba, -av,  aq, -as,  ax, -bc, -bc,  ax, -as,  aq, -av,  ba,  be, -az,  au, -ap,  at, -ay,  bd,  bb, -aw,  ar, -ar,  aw, -bb, -bd,  ay, -at,  ap, -au,  az, -be, -ba,  av, -aq,  as, -ax,  bc }, \
  { cg, -bx,  bo, -bf,  bn, -bw,  cf,  ch, -by,  bp, -bg,  bm, -bv,  ce,  ci, -bz,  bq, -bh,  bl, -bu,  cd,  cj, -ca,  br, -bi,  bk, -bt,  cc,  ck, -cb,  bs, -bj,  bj, -bs,  cb, -ck, -cc,  bt, -bk,  bi, -br,  ca, -cj, -cd,  bu, -bl,  bh, -bq,  bz, -ci, -ce,  bv, -bm,  bg, -bp,  by, -ch, -cf,  bw, -bn,  bf, -bo,  bx, -cg }, \
  { ag, -af,  ae, -ad,  ad, -ae,  af, -ag, -ag,  af, -ae,  ad, -ad,  ae, -af,  ag,  ag, -af,  ae, -ad,  ad, -ae,  af, -ag, -ag,  af, -ae,  ad, -ad,  ae, -af,  ag,  ag, -af,  ae, -ad,  ad, -ae,  af, -ag, -ag,  af, -ae,  ad, -ad,  ae, -af,  ag,  ag, -af,  ae, -ad,  ad, -ae,  af, -ag, -ag,  af, -ae,  ad, -ad,  ae, -af,  ag }, \
  { ch, -ca,  bt, -bm,  bf, -bl,  bs, -bz,  cg,  ci, -cb,  bu, -bn,  bg, -bk,  br, -by,  cf,  cj, -cc,  bv, -bo,  bh, -bj,  bq, -bx,  ce,  ck, -cd,  bw, -bp,  bi, -bi,  bp, -bw,  cd, -ck, -ce,  bx, -bq,  bj, -bh,  bo, -bv,  cc, -cj, -cf,  by, -br,  bk, -bg,  bn, -bu,  cb, -ci, -cg,  bz, -bs,  bl, -bf,  bm, -bt,  ca, -ch }, \
  { bd, -ba,  ax, -au,  ar, -ap,  as, -av,  ay, -bb,  be,  bc, -az,  aw, -at,  aq, -aq,  at, -aw,  az, -bc, -be,  bb, -ay,  av, -as,  ap, -ar,  au, -ax,  ba, -bd, -bd,  ba, -ax,  au, -ar,  ap, -as,  av, -ay,  bb, -be, -bc,  az, -aw,  at, -aq,  aq, -at,  aw, -az,  bc,  be, -bb,  ay, -av,  as, -ap,  ar, -au,  ax, -ba,  bd }, \
  { ci, -cd,  by, -bt,  bo, -bj,  bf, -bk,  bp, -bu,  bz, -ce,  cj,  ch, -cc,  bx, -bs,  bn, -bi,  bg, -bl,  bq, -bv,  ca, -cf,  ck,  cg, -cb,  bw, -br,  bm, -bh,  bh, -bm,  br, -bw,  cb, -cg, -ck,  cf, -ca,  bv, -bq,  bl, -bg,  bi, -bn,  bs, -bx,  cc, -ch, -cj,  ce, -bz,  bu, -bp,  bk, -bf,  bj, -bo,  bt, -by,  cd, -ci }, \
  { ao, -an,  am, -al,  ak, -aj,  ai, -ah,  ah, -ai,  aj, -ak,  al, -am,  an, -ao, -ao,  an, -am,  al, -ak,  aj, -ai,  ah, -ah,  ai, -aj,  ak, -al,  am, -an,  ao,  ao, -an,  am, -al,  ak, -aj,  ai, -ah,  ah, -ai,  aj, -ak,  al, -am,  an, -ao, -ao,  an, -am,  al, -ak,  aj, -ai,  ah, -ah,  ai, -aj,  ak, -al,  am, -an,  ao }, \
  { cj, -cg,  cd, -ca,  bx, -bu,  br, -bo,  bl, -bi,  bf, -bh,  bk, -bn,  bq, -bt,  bw, -bz,  cc, -cf,  ci,  ck, -ch,  ce, -cb,  by, -bv,  bs, -bp,  bm, -bj,  bg, -bg,  bj, -bm,  bp, -bs,  bv, -by,  cb, -ce,  ch, -ck, -ci,  cf, -cc,  bz, -bw,  bt, -bq,  bn, -bk,  bh, -bf,  bi, -bl,  bo, -br,  bu, -bx,  ca, -cd,  cg, -cj }, \
  { be, -bd,  bc, -bb,  ba, -az,  ay, -ax,  aw, -av,  au, -at,  as, -ar,  aq, -ap,  ap, -aq,  ar, -as,  at, -au,  av, -aw,  ax, -ay,  az, -ba,  bb, -bc,  bd, -be, -be,  bd, -bc,  bb, -ba,  az, -ay,  ax, -aw,  av, -au,  at, -as,  ar, -aq,  ap, -ap,  aq, -ar,  as, -at,  au, -av,  aw, -ax,  ay, -az,  ba, -bb,  bc, -bd,  be }, \
  { ck, -cj,  ci, -ch,  cg, -cf,  ce, -cd,  cc, -cb,  ca, -bz,  by, -bx,  bw, -bv,  bu, -bt,  bs, -br,  bq, -bp,  bo, -bn,  bm, -bl,  bk, -bj,  bi, -bh,  bg, -bf,  bf, -bg,  bh, -bi,  bj, -bk,  bl, -bm,  bn, -bo,  bp, -bq,  br, -bs,  bt, -bu,  bv, -bw,  bx, -by,  bz, -ca,  cb, -cc,  cd, -ce,  cf, -cg,  ch, -ci,  cj, -ck }, \
}

static constexpr int8_t DCT2_64[64][64] = \
  DEFINE_DCT2_P64_MATRIX(64, 83, 36, 89, 75, 50, 18, 90, 87, 80, 70, 57, 43, 25, 9, 90, 90, 88, 85, 82, 78, 73, 67, 61, 54, 46, 38, 31, 22, 13, 4, 91, 90, 90, 90, 88, 87, 86, 84, 83, 81, 79, 77, 73, 71, 69, 65, 62, 59, 56, 52, 48, 44, 41, 37, 33, 28, 24, 20, 15, 11, 7, 2);
// clang-format on

/** Apply iDCT2 to block B[64][64] + clipping */
static void idct2_64( int8_t B[][64] )
{
  int16_t X[64][64];
  int     i, j, k;
  int32_t acc;

  /* 1st pass (DCT2_64'*B) = vertical */
  for( j = 0; j < 64; j++ )
  {
    for( i = 0; i < 64; i++ )
    {
      acc = 256;
      for( k = 0; k < 64; k++ )
      {
        acc += (int32_t) DCT2_64[k][j] * B[k][i];   // iDCT bases are vertical (transpose of DCT2_64)
      }

      X[j][i] = ( acc >> 9 );
    }
  }

  /* 2nd pass (...)*DCT2_64 = horizontal + clipping */
  for( j = 0; j < 64; j++ )
  {
    for( i = 0; i < 64; i++ )
    {
      acc = 256;
      for( k = 0; k < 64; k++ )
      {
        acc += (int32_t) X[j][k] * DCT2_64[k][i];
      }

      acc >>= 9;
      if( acc > 127 )
      {
        acc = 127;
      }
      if( acc < -127 )
      {
        acc = -127;
      }
      B[j][i] = acc;
    }
  }
}

/** Apply iDCT2 to block B[32][32] + clipping */
static void idct2_32( int8_t B[][32] )
{
  int16_t X[32][32];
  int     i, j, k;
  int32_t acc;

  /* 1st pass (R32'*B) = vertical */
  for( j = 0; j < 32; j++ )
  {
    for( i = 0; i < 32; i++ )
    {
      acc = 128;
      for( k = 0; k < 32; k++ )
      {
        acc += (int32_t) DCT2_64[k * 2][j] * B[k][i];   // iDCT bases are vertical (transpose of DCT2_64=DCT)
      }

      X[j][i] = ( acc >> 8 );
    }
  }

  /* 2nd pass (...)*R32 = horizontal + clipping */
  for( j = 0; j < 32; j++ )
  {
    for( i = 0; i < 32; i++ )
    {
      acc = 256;
      for( k = 0; k < 32; k++ )
      {
        acc += (int32_t) X[j][k] * DCT2_64[k * 2][i];
      }

      acc >>= 9;
      if( acc > 127 )
      {
        acc = 127;
      }
      if( acc < -127 )
      {
        acc = -127;
      }
      B[j][i] = acc;
    }
  }
}

static void make_sei_ff_pattern64( int8_t B[][64], int fh, int fv )
{
  int      k, l;
  uint32_t n;
  fh = 4 * ( fh + 1 );
  fv = 4 * ( fv + 1 );

  n = Seed_LUT[0];
  memset( B, 0, 64 * 64 * sizeof( int8_t ) );
  for( l = 0; l < 64; l++ )
  {
    for( k = 0; k < 64; k += 4 )
    {
      if( k < fh && l < fv )
      {
        B[l][k + 0] = Gaussian_LUT[( n + 0 ) & 2047];
        B[l][k + 1] = Gaussian_LUT[( n + 1 ) & 2047];
        B[l][k + 2] = Gaussian_LUT[( n + 2 ) & 2047];
        B[l][k + 3] = Gaussian_LUT[( n + 3 ) & 2047];
      }
      n = prng( n );
    }
  }
  B[0][0] = 0;
  idct2_64( B );
}

static void make_sei_ff_pattern32( int8_t B[][32], int fh, int fv )
{
  int      k, l;
  uint32_t n;
  fh = 2 * ( fh + 1 );
  fv = 2 * ( fv + 1 );

  n = Seed_LUT[1];
  memset( B, 0, 32 * 32 * sizeof( int8_t ) );
  for( l = 0; l < 32; l++ )
  {
    for( k = 0; k < 32; k += 2 )
    {
      if( k < fh && l < fv )
      {
        B[l][k + 0] = Gaussian_LUT[( n + 0 ) & 2047];
        B[l][k + 1] = Gaussian_LUT[( n + 1 ) & 2047];
      }
      n = prng( n );
    }
  }
  B[0][0] = 0;
  idct2_32( B );
}

static void make_ar_pattern( int8_t buf[], int8_t P[], int size, const int16_t ar_coef[], int nb_coef, int shift, int scale, uint32_t seed )
{
  int16_t  coef[4][7];
  int      L = 0;
  int      x, y, i, j, k;
  int      g;
  int      subx, suby, width, height;
  uint32_t rnd = seed;

  memset( coef, 0, sizeof( coef ) );
  subx = suby = ( size == 32 ) ? 2 : 1;
  width       = ( subx > 1 ) ? 44 : 82;
  height      = ( suby > 1 ) ? 38 : 73;

  switch( nb_coef )
  {
  case 6:
    // SEI.AR mode
    coef[3][2] = ar_coef[1];                                                            // left
    coef[2][3] = ( ar_coef[1] * ar_coef[4] ) >> scale;                                  // top
    coef[2][2] = ( ar_coef[3] * ar_coef[4] ) >> scale;                                  // top-left
    coef[2][4] = ( ar_coef[3] * ar_coef[4] ) >> scale;                                  // top-right
    coef[3][1] = ar_coef[5];                                                            // left-left
    coef[1][3] = ( (int32_t) ar_coef[5] * ar_coef[4] * ar_coef[4] ) >> ( 2 * scale );   // top-top

    L = 2;
    break;

  default:
    THROW_FATAL( "nb_coef != 6 not implemented" );
  }
  if( nb_coef != 6 )
  {
    for( k = 0, j = -L; j <= 0; j++ )
    {
      for( i = -L; i <= L && ( i < 0 || j < 0 ); i++, k++ )
      {
        coef[3 + j][3 + i] = ar_coef[k];
      }
    }
  }

  memset( buf, 0, width * height );   // debug (not needed)
  for( y = 0; y < height; y++ )
  {
    for( x = 0; x < width; x++ )
    {
      // Filter
      g = 0;
      if( y >= 3 && y < height && x >= 3 && x < width - 3 )
      {
        for( j = -3; j <= 0; j++ )
        {
          for( i = -3; i <= 3 && ( i < 0 || j < 0 ); i++ )
          {
            g += (int) coef[3 + j][3 + i] * buf[width * ( y + j ) + x + i];
          }
        }

        g = round( g, scale );
      }

      // Add random noise
      g += round( Gaussian_LUT[rnd & 2047], shift );
      rnd = prng( rnd );

      buf[width * y + x] = Clip3( -127, 127, g );
    }
  }

  // Copy cropped area to output
  memset( P, 0, size * size );
  for( y = 0; y < 64 / suby; y++ )
  {
    for( x = 0; x < 64 / subx; x++ )
    {
      P[size * y + x] = buf[width * ( 3 + 6 / suby + y ) + ( 3 + 6 / subx + x )];
    }
  }
}

static int same_pattern( fgs_sei* cfg, int32_t a, int32_t b )
{
  int16_t* coef_a = &cfg->comp_model_value[0][0][0] + a;
  int16_t* coef_b = &cfg->comp_model_value[0][0][0] + b;

  for( int i = 1; i < SEI_MAX_MODEL_VALUES; i++ )
  {
    if( coef_a[i] != coef_b[i] )
    {
      return 0;
    }
  }

  return 1;
}

void FilmGrain::set_seed( uint32_t seed )
{
  m_line_rnd = m_line_rnd_up = seed;
}

/** Initialize "hardware" interface from FGS SEI parameters */
void FilmGrain::init_sei()
{
  int8_t        P[64 * 64];
  int8_t        Lbuf[73 * 82];
  int8_t        Cbuf[38 * 44];
  uint8_t       slut[256];
  uint8_t       plut[256];
  uint8_t       intensities[VFGS_MAX_PATTERNS];
  uint32_t      patterns[VFGS_MAX_PATTERNS];
  uint8_t       np = 0;   // number of patterns
  uint8_t       a, b, i;
  unsigned char all0 = 1;

  int c, k;

  for( c = 0; c < 3; c++ )
  {
    memset( slut, 0, sizeof( slut ) );
    if( c < 2 )
    {
      np = 0;
      memset( intensities, 0, sizeof( intensities ) );
      memset( patterns, ~0, sizeof( patterns ) );
    }
    // 1. Look for different patterns, up to max supported number
    if( fgs.comp_model_present_flag[c] )
    {
      for( k = 0; k < fgs.num_intensity_intervals[c]; k++ )
      {
        a           = fgs.intensity_interval_lower_bound[c][k];
        uint32_t id = SEI_MAX_MODEL_VALUES * ( k + 256 * c );

        for( i = 0; i < VFGS_MAX_PATTERNS; i++ )
        {
          if( same_pattern( &fgs, patterns[i], id ) )
          {
            break;
          }
        }

        if( i == VFGS_MAX_PATTERNS && np < VFGS_MAX_PATTERNS )   // can add it
        {
          // keep them sorted (by intensity). The goal of this sort is
          // to enable meaningful pattern interpolation
          for( i = np; i > 0; i-- )
          {
            if( intensities[i - 1] > a )
            {
              intensities[i] = intensities[i - 1];
              patterns[i]    = patterns[i - 1];
            }
            else
            {
              break;
            }
          }
          intensities[i] = a;
          patterns[i]    = id;
          np++;
        }
      }
    }
    if( c == 0 || c == 2 )
    {
      // 2. Register the patterns (with correct order)
      for( i = 0; i < np; i++ )
      {
        int16_t* coef = &fgs.comp_model_value[0][0][0] + patterns[i];

        if( c == 0 )
        {
          if( fgs.model_id )
          {
            make_ar_pattern( Lbuf, P, 64, coef, 6, 1, fgs.log2_scale_factor, Seed_LUT[0] );
          }
          else
          {
            make_sei_ff_pattern64( (int8_t( * )[64]) P, coef[1], coef[2] );
          }

          m_impl->set_luma_pattern( i, P );
        }
        else if( c == 2 )
        {
          if( fgs.model_id )
          {
            make_ar_pattern( Cbuf, P, 32, coef, 6, 1, fgs.log2_scale_factor, Seed_LUT[1] );
          }
          else
          {
            make_sei_ff_pattern32( (int8_t( * )[32]) P, coef[1], coef[2] );
          }
          m_impl->set_chroma_pattern( i, P );
        }
      }
      // 3. Fill up LUTs
      for( int cc = std::min( c, 1 ); cc <= c; cc++ )
      {
        if( fgs.comp_model_present_flag[cc] )
        {
          memset( plut, 255, sizeof( plut ) );
          // 3a. Fill valid patterns
          for( k = 0; k < fgs.num_intensity_intervals[cc]; k++ )
          {
            a           = fgs.intensity_interval_lower_bound[cc][k];
            b           = fgs.intensity_interval_upper_bound[cc][k];
            uint32_t id = SEI_MAX_MODEL_VALUES * ( k + 256 * cc );

            for( i = 0; i < VFGS_MAX_PATTERNS; i++ )
            {
              if( same_pattern( &fgs, patterns[i], id ) )
              {
                break;
              }
            }
            // Note: if not found, could try to find interpolation value

            for( int l = a; l <= b; l++ )
            {
              slut[l] = (uint8_t) fgs.comp_model_value[cc][k][0];
              if( i < VFGS_MAX_PATTERNS )
              {
                plut[l] = i << 4;
              }
            }
          }
          // 3b. Fill holes (no interp. yet, just repeat last)
          i       = 0;
          int tmp = 0;
          for( k = 0; k < 256; k++ )
          {
            if( plut[k] == 255 )
            {
              plut[k] = i;
            }
            else
            {
              i = plut[k];
            }
            tmp += plut[k];
          }
          if( tmp != 0 )
          {
            all0 = 0;
          }
        }
        else
        {
          memset( plut, 0, sizeof( plut ) );
          all0 = 1;
        }
        // 3c. Register LUTs
        m_impl->set_scale_lut( cc, slut );
        m_impl->set_pattern_lut( cc, plut, all0 );
      }
    }
  }

  m_impl->set_scale_shift( fgs.log2_scale_factor - ( fgs.model_id ? 1 : 0 ) );   // -1 for grain shift in pattern generation (see above)
}

FilmGrain::FilmGrain()
{
#if defined( TARGET_SIMD_X86 ) && defined( USE_SIMD )
  m_impl = FilmGrainImplX86<UNDEFINED>::makeFilmGrainImpl();
#else
  m_impl = std::make_unique<FilmGrainImpl>();
#endif
}

void FilmGrain::updateFGC( vvdecSEIFilmGrainCharacteristics* fgc )
{
  // Copy SEI message in vfgs structure format
  // TODO: check some values and warn about unsupported stuff ?
  fgs.model_id          = fgc->filmGrainModelId;
  fgs.log2_scale_factor = fgc->log2ScaleFactor;
  for( int c = 0; c < 3; c++ )
  {
    vvdecCompModel& cm = fgc->compModel[c];
    if( cm.presentFlag )
    {
      fgs.comp_model_present_flag[c] = 1;
      fgs.num_intensity_intervals[c] = cm.numIntensityIntervals;
      fgs.num_model_values[c]        = cm.numModelValues;
      for( int i = 0; i < fgs.num_intensity_intervals[c]; i++ )
      {
        vvdecCompModelIntensityValues& cmiv      = cm.intensityValues[i];
        fgs.intensity_interval_lower_bound[c][i] = cmiv.intensityIntervalLowerBound;
        fgs.intensity_interval_upper_bound[c][i] = cmiv.intensityIntervalUpperBound;
        for( int v = 0; v < fgs.num_model_values[c]; v++ )
        {
          fgs.comp_model_value[c][i][v] = cmiv.compModelValue[v];
        }
        // Fill with default model values (VFGS needs them; it actually ignores num_model_values)
        switch( fgs.num_model_values[c] )
        {
          // clang-format off
          case 0:
          case 1: fgs.comp_model_value[c][i][1] = fgs.model_id ? 0 : 8;                               // H high cutoff / 1st AR coef (left & top)
          case 2: fgs.comp_model_value[c][i][2] = fgs.model_id ? 0 : fgs.comp_model_value[c][i][1];   // V high cutoff / x-comp corr
          case 3: fgs.comp_model_value[c][i][3] = 0;                                                  // H low cutoff / 2nd AR coef (top-left, top-right)
          case 4: fgs.comp_model_value[c][i][4] = fgs.model_id << fgs.log2_scale_factor;              // V low cutoff / aspect ratio
                  fgs.comp_model_value[c][i][5] = 0;                                                  // x-comp corr / 3rd AR coef (left-left, top-top)
          // clang-format on
        }
      }
    }
  }

  // Conversion of component model values for 4:2:0 chroma format
  if( fgs.model_id == 0 )
  {
    for( int c = 1; c < 3; c++ )
    {
      if( fgs.comp_model_present_flag[c] )
      {
        for( int k = 0; k < fgs.num_intensity_intervals[c]; k++ )
        {
          fgs.comp_model_value[c][k][1] = std::max( 2, std::min( 14, fgs.comp_model_value[c][k][1] << 1 ) );   // Horizontal frequency
          fgs.comp_model_value[c][k][2] = std::max( 2, std::min( 14, fgs.comp_model_value[c][k][2] << 1 ) );   // Vertical frequency
          fgs.comp_model_value[c][k][0] >>= 1;
        }
      }
    }
  }

  init_sei();

  //  if (!m_bFgs)
  //    // TODO: get something random
  //      // TODO: make seed also impact the pattern gen
  //    set_seed(uint32_t seed);
}

void FilmGrain::prepareBlockSeeds( int width, int height )
{
  m_line_seeds.resize( ( height + 15 ) / 16 );

  m_prev_frame_line_rnd_up = m_line_rnd_up;

  uint32_t rnd = 0;
  for( int y = 0; y < m_line_seeds.size(); ++y )
  {
    // Generate / backup / restore per-line random seeds (needed to make multi-line blocks)
    if( y != 0 )
    {
      // new line of blocks
      m_line_rnd_up = m_line_rnd;
      m_line_rnd    = rnd;
    }

    m_line_seeds[y] = m_line_rnd;

    // Crank random generator
    rnd = m_line_rnd;
    for( int x = 0; x < ( width + 15 ) / 16; ++x )
    {
      rnd = prng( rnd );
    }
  }
}

void FilmGrain::setColorFormat( vvdecColorFormat fmt )
{
  switch( fmt )
  {
    // clang-format off
    case VVDEC_CF_YUV400_PLANAR: m_impl->set_chroma_subsampling( 0, 0 ); break;
    case VVDEC_CF_YUV420_PLANAR: m_impl->set_chroma_subsampling( 2, 2 ); break;
    case VVDEC_CF_YUV422_PLANAR: m_impl->set_chroma_subsampling( 2, 1 ); break;
    case VVDEC_CF_YUV444_PLANAR: m_impl->set_chroma_subsampling( 1, 1 ); break;
    default: THROW_FATAL( "invalid color format: " );
    // clang-format on
  }
}

void FilmGrain::add_grain_line( void* Y, void* U, void* V, int y, int width )
{
  uint32_t rnd_up = y < 16 ? m_prev_frame_line_rnd_up : m_line_seeds[y / 16 - 1];
  uint32_t rnd    = m_line_seeds[y / 16];

  int16_t grain[3][32];
  uint8_t scale[3][32];

  // Process line
  for( int x = 0; x < width; x += 16 )
  {
    // Process pixels for each color component
    if( fgs.comp_model_present_flag[0] )
    {
      m_impl->add_grain_block( Y, 0, x, y, width, rnd, rnd_up, grain, scale );
    }
    if( U && V )
    {
      if( fgs.comp_model_present_flag[1] )
      {
        m_impl->add_grain_block( U, 1, x, y, width, rnd, rnd_up, grain, scale );
      }
      if( fgs.comp_model_present_flag[2] )
      {
        m_impl->add_grain_block( V, 2, x, y, width, rnd, rnd_up, grain, scale );
      }
    }
    // Crank random generator
    rnd    = prng( rnd );
    rnd_up = prng( rnd_up );   // upper block (overlapping)
  }
}

}   // namespace vvdec
