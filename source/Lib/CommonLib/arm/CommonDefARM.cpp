/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2018-2026, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVdeC Authors.
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

/** \file     CommonDefARM.cpp
 */

#include "CommonDefARM.h"

#if defined( __APPLE__ )
#include <cstdint>
#include <sys/sysctl.h>
#endif

#if defined( _WIN32 )
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

#if TARGET_SIMD_ARM
#if defined( __linux__ ) || HAVE_ELF_AUX_INFO
#include <sys/auxv.h>  // getauxval / elf_aux_info
#endif
#endif

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT

namespace vvdec
{
using namespace arm_simd;

const static std::vector<std::pair<ARM_VEXT, std::string>> vext_names{
  { UNDEFINED, ""         },
  { SCALAR,    "SCALAR"   },
  { NEON,      "NEON"     },
#if TARGET_SIMD_ARM_RDM
  { NEON_RDM,  "NEON_RDM" },
#endif
#if TARGET_SIMD_ARM_SVE
  { SVE,       "SVE"      },
#endif
#if TARGET_SIMD_ARM_SVE2
  { SVE2,      "SVE2"     },
#endif
};

const std::string& arm_vext_to_string( ARM_VEXT vext )
{
  for( auto& it : vext_names )
  {
    if( it.first == vext )
    {
      return it.second;
    }
  }
  THROW_FATAL( "Invalid SIMD extension value " << vext );
}

ARM_VEXT string_to_arm_vext( const std::string& ext_name )
{
  if( ext_name.empty() )
  {
    return UNDEFINED;
  }

  for( auto& it : vext_names )
  {
    if( it.second == ext_name )
    {
      return it.first;
    }
  }

  THROW_FATAL( "Invalid SIMD Mode string: \"" << ext_name << "\"" );
}

#if defined( __linux__ ) || HAVE_ELF_AUX_INFO

// Define hwcap values ourselves: building with an old auxv header where these
// hwcap values are not defined should not prevent features from being enabled.
#define AARCH64_HWCAP_ASIMDRDM  ( 1 << 12 )
#define AARCH64_HWCAP_SVE       ( 1 << 22 )
#define AARCH64_HWCAP2_SVE2     ( 1 << 1 )

static ARM_VEXT _get_arm_extensions()
{
  // We assume Neon is always supported for relevant Arm processors.
  ARM_VEXT ext = NEON;

#if TARGET_SIMD_ARM_RDM
#if HAVE_ELF_AUX_INFO
  unsigned long hwcap = 0;
  elf_aux_info( AT_HWCAP, &hwcap, sizeof(hwcap) );
#else
  unsigned long hwcap = getauxval( AT_HWCAP );
#endif // HAVE_ELF_AUX_INFO

#if TARGET_SIMD_ARM_SVE2
#if HAVE_ELF_AUX_INFO
  unsigned long hwcap2 = 0;
  elf_aux_info( AT_HWCAP2, &hwcap2, sizeof(hwcap2) );
#else
  unsigned long hwcap2 = getauxval( AT_HWCAP2 );
#endif // HAVE_ELF_AUX_INFO
#endif // TARGET_SIMD_ARM_SVE2
#endif // TARGET_SIMD_ARM_RDM

#if TARGET_SIMD_ARM_RDM
  if( hwcap & AARCH64_HWCAP_ASIMDRDM )
  {
    ext = NEON_RDM;
#if TARGET_SIMD_ARM_SVE
    if( hwcap & AARCH64_HWCAP_SVE )
    {
      ext = SVE;
#if TARGET_SIMD_ARM_SVE2
      if( hwcap2 & AARCH64_HWCAP2_SVE2 )
      {
        ext = SVE2;
      }
#endif // TARGET_SIMD_ARM_SVE2
    }
#endif // TARGET_SIMD_ARM_SVE
  }
#endif // TARGET_SIMD_ARM_RDM

  return ext;
}

#elif defined( __APPLE__ )

static bool have_feature( const char* feature )
{
  int64_t feature_present = 0;
  size_t  size            = sizeof( feature_present );
  if( sysctlbyname( feature, &feature_present, &size, nullptr, 0 ) != 0 )
  {
    return false;
  }
  return feature_present != 0;
}

static ARM_VEXT _get_arm_extensions()
{
  // We assume Neon is always supported for relevant Apple Silicon processors.
  // The SVE and SVE2 features are not available at time of writing.
  ARM_VEXT ext = NEON;

#if TARGET_SIMD_ARM_RDM
  if( have_feature( "hw.optional.arm.FEAT_RDM" ) )
  {
    ext = NEON_RDM;
  }
#endif

  return ext;
}

#elif defined( _WIN32 )

static ARM_VEXT _get_arm_extensions()
{
  // We assume Neon is always supported for relevant Arm processors.
  ARM_VEXT ext = NEON;

  // There is no flag to detect the Neon RDM feature at time of writing, so
  // check `PF_ARM_V82_DP_INSTRUCTIONS_AVAILABLE` instead. This is fine since
  // `FEAT_RDM` is a mandatory feature from Armv8.1-A and `FEAT_DotProd` is
  // only available from Armv8.1-A or later (originally it was available from
  // Armv8.2-A or later, hence the unfortunate "V82" in the macro name).
  // Transitively, if `FEAT_DotProd` is available then `FEAT_RDM` must also be!
#if TARGET_SIMD_ARM_RDM && defined( PF_ARM_V82_DP_INSTRUCTIONS_AVAILABLE )
  if( IsProcessorFeaturePresent( PF_ARM_V82_DP_INSTRUCTIONS_AVAILABLE ) )
  {
    ext = NEON_RDM;
  }
#endif

#if TARGET_SIMD_ARM_SVE && defined( PF_ARM_SVE_INSTRUCTIONS_AVAILABLE )
  if( ext >= NEON_RDM && IsProcessorFeaturePresent( PF_ARM_SVE_INSTRUCTIONS_AVAILABLE ) )
  {
    ext = SVE;
  }
#endif

#if TARGET_SIMD_ARM_SVE2 && defined( PF_ARM_SVE2_INSTRUCTIONS_AVAILABLE )
  if( ext >= SVE && IsProcessorFeaturePresent( PF_ARM_SVE2_INSTRUCTIONS_AVAILABLE ) )
  {
    ext = SVE2;
  }
#endif

  return ext;
}

#else

static ARM_VEXT _get_arm_extensions()
{
  // We assume Neon is always supported for relevant Arm processors.
  // No other extensions supported on non-Linux platforms for now.
  return NEON;
}

#endif // defined( __linux__ ) || HAVE_ELF_AUX_INFO

ARM_VEXT read_arm_extension_flags( ARM_VEXT request )
{
  static ARM_VEXT max_supported = _get_arm_extensions();
  static ARM_VEXT ext_flags     = max_supported;

  if( request != UNDEFINED )
  {
    if( request > max_supported )
    {
      THROW_FATAL( "requested SIMD level (" << request << ") not supported by current CPU (max " << max_supported << ")." );
    }
    ext_flags = request;
  }

  return ext_flags;
};

const std::string& read_arm_extension_name()
{
  return arm_vext_to_string( read_arm_extension_flags() );
}

} // namespace vvdec

#endif // defined(TARGET_SIMD_ARM) && ENABLE_SIMD_OPT
