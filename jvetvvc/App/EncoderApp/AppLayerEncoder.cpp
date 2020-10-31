/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2020, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     AppLayerEncoder.cpp
    \brief    Encoder application class
*/

#include <list>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <iomanip>

#include "AppLayerEncoder.h"
#include "EncoderLib/AnnexBwrite.h"
#include "EncoderLib/EncoderSharedComponents.h"

using namespace std;

//! \ingroup EncoderApp
//! \{

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================

AppLayerEncoder::AppLayerEncoder( fstream& bitStream, EncoderSharedComponents& encoderSharedComponents )
  : m_layerEncoder( encoderSharedComponents )
  , m_bitstream( bitStream )
{
  m_numFramesReceived = 0;
  m_totalBytes = 0;
  m_essentialBytes = 0;
#if JVET_O0756_CALCULATE_HDRMETRICS
  m_metricTime = std::chrono::milliseconds(0);
#endif
  m_numEncoded = 0;
  m_flush = false;
}

AppLayerEncoder::~AppLayerEncoder()
{
}

void AppLayerEncoder::xInitLibCfg()
{
  VPS& vps = *m_layerEncoder.getVPS();
#if JVET_S0163_ON_TARGETOLS_SUBLAYERS
  if (m_targetOlsIdx != 500)
  {
    vps.m_targetOlsIdx = m_targetOlsIdx;
  }
  else
  {
    vps.m_targetOlsIdx = -1;
  }
#else
  vps.m_targetOlsIdx = m_targetOlsIdx;
#endif

  vps.setMaxLayers( m_maxLayers );

  if (vps.getMaxLayers() > 1)
  {
    vps.setVPSId(1);  //JVET_P0205 vps_video_parameter_set_id shall be greater than 0 for multi-layer coding
  }
  else
  {
    vps.setVPSId(0);
    vps.setEachLayerIsAnOlsFlag(1); // If vps_max_layers_minus1 is equal to 0,
                                    // the value of vps_each_layer_is_an_ols_flag is inferred to be equal to 1.
                                    // Otherwise, when vps_all_independent_layers_flag is equal to 0,
                                    // the value of vps_each_layer_is_an_ols_flag is inferred to be equal to 0.
  }
  vps.setMaxSubLayers(m_maxSublayers);
  if (vps.getMaxLayers() > 1 && vps.getMaxSubLayers() > 1)
  {
    vps.setDefaultPtlDpbHrdMaxTidFlag(m_defaultPtlDpbHrdMaxTidFlag);
  }
  if (vps.getMaxLayers() > 1)
  {
    vps.setAllIndependentLayersFlag(m_allIndependentLayersFlag);
    if (!vps.getAllIndependentLayersFlag())
    {
      vps.setEachLayerIsAnOlsFlag(0);
      for (int i = 0; i < m_maxTempLayer; i++)
      {
        vps.setPredDirection(i, 0);
      }
      for (int i = 0; i < m_predDirectionArray.size(); i++)
      {
        if (m_predDirectionArray[i] != ' ')
        {
          vps.setPredDirection(i >> 1, int(m_predDirectionArray[i] - 48));
        }
      }
    }
  }

#if JVET_R0193
  m_cfgVPSParameters.m_maxTidILRefPicsPlus1.resize(vps.getMaxLayers(), std::vector<uint32_t>(vps.getMaxLayers(), MAX_TLAYER));
#endif
  for (int i = 0; i < vps.getMaxLayers(); i++)
  {
    vps.setGeneralLayerIdx( m_layerId[i], i );
    vps.setLayerId(i, m_layerId[i]);

    if (i > 0 && !vps.getAllIndependentLayersFlag())
    {
      vps.setIndependentLayerFlag( i, m_numRefLayers[i] ? false : true );

      if (!vps.getIndependentLayerFlag(i))
      {
        for (int j = 0, k = 0; j < i; j++)
        {
          if (m_refLayerIdxStr[i].find(to_string(j)) != std::string::npos)
          {
            vps.setDirectRefLayerFlag(i, j, true);
            vps.setInterLayerRefIdc( i, j, k );
            vps.setDirectRefLayerIdx(i, k++, j);
          }
          else
          {
            vps.setDirectRefLayerFlag(i, j, false);
          }
        }
#if JVET_R0193
        string::size_type beginStr = m_maxTidILRefPicsPlus1Str[i].find_first_not_of(" ", 0);
        string::size_type endStr = m_maxTidILRefPicsPlus1Str[i].find_first_of(" ", beginStr);
        int t = 0;
        while (string::npos != beginStr || string::npos != endStr)
        {
          m_cfgVPSParameters.m_maxTidILRefPicsPlus1[i][t++] = std::stoi(m_maxTidILRefPicsPlus1Str[i].substr(beginStr, endStr - beginStr));
          beginStr = m_maxTidILRefPicsPlus1Str[i].find_first_not_of(" ", endStr);
          endStr = m_maxTidILRefPicsPlus1Str[i].find_first_of(" ", beginStr);
        }
#endif
      }
    }
  }


  if (vps.getMaxLayers() > 1)
  {
    if (vps.getAllIndependentLayersFlag())
    {
      vps.setEachLayerIsAnOlsFlag(m_eachLayerIsAnOlsFlag);
      if (vps.getEachLayerIsAnOlsFlag() == 0)
      {
        vps.setOlsModeIdc(2); // When vps_all_independent_layers_flag is equal to 1 and vps_each_layer_is_an_ols_flag is equal to 0, the value of vps_ols_mode_idc is inferred to be equal to 2
      }
    }
    if (!vps.getEachLayerIsAnOlsFlag())
    {
      if (!vps.getAllIndependentLayersFlag())
      {
        vps.setOlsModeIdc(m_olsModeIdc);
      }
      if (vps.getOlsModeIdc() == 2)
      {
        vps.setNumOutputLayerSets(m_numOutputLayerSets);
        for (int i = 1; i < vps.getNumOutputLayerSets(); i++)
        {
          for (int j = 0; j < vps.getMaxLayers(); j++)
          {
            if (m_olsOutputLayerStr[i].find(to_string(j)) != std::string::npos)
            {
              vps.setOlsOutputLayerFlag(i, j, 1);
            }
            else
            {
              vps.setOlsOutputLayerFlag(i, j, 0);
            }
          }
        }
      }
    }
  }
  CHECK( m_numPtlsInVps == 0, "There has to be at least one PTL structure in the VPS." );
  vps.setNumPtls                                                 ( m_numPtlsInVps );
  vps.setPtPresentFlag                                           (0, 1);
  for (int i = 0; i < vps.getNumPtls(); i++)
  {
    if( i > 0 )
      vps.setPtPresentFlag                                         (i, 0);
    vps.setPtlMaxTemporalId                                      (i, vps.getMaxSubLayers() - 1);
  }
  for (int i = 0; i < vps.getNumOutputLayerSets(); i++)
  {
    vps.setOlsPtlIdx                                             (i, m_olsPtlIdx[i]);
  }
  std::vector<ProfileTierLevel> ptls;
  ptls.resize(vps.getNumPtls());
  // PTL0 shall be the same as the one signalled in the SPS
  ptls[0].setLevelIdc                                            ( m_level );
  ptls[0].setProfileIdc                                          ( m_profile);
  ptls[0].setTierFlag                                            ( m_levelTier );
  ptls[0].setFrameOnlyConstraintFlag                             ( m_frameOnlyConstraintFlag);
  ptls[0].setMultiLayerEnabledFlag                               ( m_multiLayerEnabledFlag);
  CHECK((m_profile == Profile::MAIN_10 || m_profile == Profile::MAIN_10_444
         || m_profile == Profile::MAIN_10_STILL_PICTURE || m_profile == Profile::MAIN_10_444_STILL_PICTURE)
          && m_multiLayerEnabledFlag,
        "ptl_multilayer_enabled_flag shall be equal to 0 for non-multilayer profiles");
  ptls[0].setNumSubProfile                                       ( m_numSubProfile );
  for (int i = 0; i < m_numSubProfile; i++)
  {
    ptls[0].setSubProfileIdc                                   (i, m_subProfile[i]);
  }
  for(int i = 1; i < vps.getNumPtls(); i++)
  {
    ptls[i].setLevelIdc                                          (m_levelPtl[i]);
  }
  vps.setProfileTierLevel(ptls);
  vps.setVPSExtensionFlag                                        ( false );
  m_layerEncoder.setProfile                                           ( m_profile);
  m_layerEncoder.setLevel                                             ( m_levelTier, m_level);
  m_layerEncoder.setFrameOnlyConstraintFlag                           ( m_frameOnlyConstraintFlag);
  m_layerEncoder.setMultiLayerEnabledFlag                             ( m_multiLayerEnabledFlag || m_maxLayers > 1);
  m_layerEncoder.setNumSubProfile                                     ( m_numSubProfile );
  for (int i = 0; i < m_numSubProfile; i++)
  {
    m_layerEncoder.setSubProfile(i, m_subProfile[i]);
  }

  m_layerEncoder.setPrintMSEBasedSequencePSNR                         ( m_printMSEBasedSequencePSNR);
  m_layerEncoder.setPrintFrameMSE                                     ( m_printFrameMSE);
  m_layerEncoder.setPrintHexPsnr(m_printHexPsnr);
  m_layerEncoder.setPrintSequenceMSE                                  ( m_printSequenceMSE);
  m_layerEncoder.setPrintMSSSIM                                       ( m_printMSSSIM );
  m_layerEncoder.setCabacZeroWordPaddingEnabled                       ( m_cabacZeroWordPaddingEnabled );

  m_layerEncoder.setFrameRate                                         ( m_iFrameRate );
  m_layerEncoder.setFrameSkip                                         ( m_FrameSkip );
  m_layerEncoder.setTemporalSubsampleRatio                            ( m_temporalSubsampleRatio );
  m_layerEncoder.setSourceWidth                                       ( m_iSourceWidth );
  m_layerEncoder.setSourceHeight                                      ( m_iSourceHeight );
  m_layerEncoder.setConformanceWindow                                 ( m_confWinLeft / SPS::getWinUnitX( m_InputChromaFormatIDC ), m_confWinRight / SPS::getWinUnitX( m_InputChromaFormatIDC ), m_confWinTop / SPS::getWinUnitY( m_InputChromaFormatIDC ), m_confWinBottom / SPS::getWinUnitY( m_InputChromaFormatIDC ) );
  m_layerEncoder.setScalingRatio                                      ( m_scalingRatioHor, m_scalingRatioVer );
  m_layerEncoder.setRprEnabled                                        (m_rprEnabledFlag);
  m_layerEncoder.setResChangeInClvsEnabled                            ( m_resChangeInClvsEnabled );
  m_layerEncoder.setSwitchPocPeriod                                   ( m_switchPocPeriod );
  m_layerEncoder.setUpscaledOutput                                    ( m_upscaledOutput );
  m_layerEncoder.setFramesToBeEncoded                                 ( m_framesToBeEncoded );

  m_layerEncoder.setAvoidIntraInDepLayer                              ( m_avoidIntraInDepLayer );

  //====== SPS constraint flags =======
  m_layerEncoder.setGciPresentFlag                                    ( m_gciPresentFlag );
  if (m_layerEncoder.getGciPresentFlag())
  {
    m_layerEncoder.setNonPackedConstraintFlag(m_nonPackedConstraintFlag);
    m_layerEncoder.setNonProjectedConstraintFlag(m_nonProjectedConstraintFlag);
    m_layerEncoder.setOneTilePerPicConstraintFlag(m_oneTilePerPicConstraintFlag);
    m_layerEncoder.setPicHeaderInSliceHeaderConstraintFlag(m_picHeaderInSliceHeaderConstraintFlag);
    m_layerEncoder.setOneSlicePerPicConstraintFlag(m_oneSlicePerPicConstraintFlag);
    m_layerEncoder.setNoIdrRplConstraintFlag(m_noIdrRplConstraintFlag);
    CHECK(m_noIdrRplConstraintFlag&& m_idrRefParamList, "IDR RPL shall be deactivated when gci_no_idr_rpl_constraint_flag equal to 1");

    m_layerEncoder.setNoRectSliceConstraintFlag(m_noRectSliceConstraintFlag);
    CHECK(m_noRectSliceConstraintFlag && !m_rasterSliceFlag, "Rectangular slice shall be deactivated when gci_no_rectangular_slice_constraint_flag equal to 1");

    m_layerEncoder.setOneSlicePerSubpicConstraintFlag(m_oneSlicePerSubpicConstraintFlag);
    CHECK(m_oneSlicePerSubpicConstraintFlag && !m_singleSlicePerSubPicFlag, "Each picture shall consist of one and only one rectangular slice when gci_one_slice_per_subpic_constraint_flag equal to 1");

    m_layerEncoder.setNoSubpicInfoConstraintFlag(m_noSubpicInfoConstraintFlag);
    CHECK(m_noSubpicInfoConstraintFlag&& m_subPicInfoPresentFlag, "Subpicture information shall not present when gci_no_subpic_info_constraint_flag equal to 1");
    m_layerEncoder.setOnePictureOnlyConstraintFlag(m_onePictureOnlyConstraintFlag);
    m_layerEncoder.setIntraOnlyConstraintFlag(m_intraOnlyConstraintFlag);
    m_layerEncoder.setNoIdrConstraintFlag(m_noIdrConstraintFlag);
    m_layerEncoder.setNoGdrConstraintFlag(m_noGdrConstraintFlag);
    m_layerEncoder.setAllLayersIndependentConstraintFlag(m_allLayersIndependentConstraintFlag);
    m_layerEncoder.setNoCuQpDeltaConstraintFlag(m_noCuQpDeltaConstraintFlag);

    m_layerEncoder.setNoTrailConstraintFlag(m_noTrailConstraintFlag);
    CHECK(m_noTrailConstraintFlag && m_iIntraPeriod != 1, "TRAIL shall be deactivated when m_noTrailConstraintFlag is equal to 1");

    m_layerEncoder.setNoStsaConstraintFlag(m_noStsaConstraintFlag);
    CHECK(m_noStsaConstraintFlag && (m_iIntraPeriod != 1 || xHasNonZeroTemporalID()), "STSA shall be deactivated when m_noStsaConstraintFlag is equal to 1");

    m_layerEncoder.setNoRaslConstraintFlag(m_noRaslConstraintFlag);
    CHECK(m_noRaslConstraintFlag && (m_iIntraPeriod != 1 || xHasLeadingPicture()), "RASL shall be deactivated when m_noRaslConstraintFlag is equal to 1");

    m_layerEncoder.setNoRadlConstraintFlag(m_noRadlConstraintFlag);
    CHECK(m_noRadlConstraintFlag && (m_iIntraPeriod != 1 || xHasLeadingPicture()), "RADL shall be deactivated when m_noRadlConstraintFlag is equal to 1");

    m_layerEncoder.setNoCraConstraintFlag(m_noCraConstraintFlag);
    CHECK(m_noCraConstraintFlag && (m_iDecodingRefreshType == 1), "CRA shall be deactivated when m_noCraConstraintFlag is equal to 1");

    m_layerEncoder.setNoRprConstraintFlag(m_noRprConstraintFlag);
    CHECK(m_noRprConstraintFlag && m_rprEnabledFlag, "Reference picture resampling shall be deactivated when m_noRprConstraintFlag is equal to 1");

    m_layerEncoder.setNoResChangeInClvsConstraintFlag(m_noResChangeInClvsConstraintFlag);
    CHECK(m_noResChangeInClvsConstraintFlag && m_resChangeInClvsEnabled, "Resolution change in CLVS shall be deactivated when m_noResChangeInClvsConstraintFlag is equal to 1");

    m_layerEncoder.setMaxBitDepthConstraintIdc(m_maxBitDepthConstraintIdc);
    CHECK(m_internalBitDepth[CHANNEL_TYPE_LUMA] > m_maxBitDepthConstraintIdc, "Internal bit depth shall be less than or equal to m_maxBitDepthConstraintIdc");

    m_layerEncoder.setMaxChromaFormatConstraintIdc(m_maxChromaFormatConstraintIdc);
    CHECK(m_chromaFormatIDC > m_maxChromaFormatConstraintIdc, "Chroma format Idc shall be less than or equal to m_maxBitDepthConstraintIdc");

    m_layerEncoder.setNoMttConstraintFlag(m_noMttConstraintFlag);
    CHECK(m_noMttConstraintFlag && (m_uiMaxMTTHierarchyDepth || m_uiMaxMTTHierarchyDepthI || m_uiMaxMTTHierarchyDepthIChroma), "Mtt shall be deactivated when m_bNoMttConstraintFlag is equal to 1");

    m_layerEncoder.setNoQtbttDualTreeIntraConstraintFlag(m_noQtbttDualTreeIntraConstraintFlag);
    CHECK(m_noQtbttDualTreeIntraConstraintFlag && m_dualTree, "Dual tree shall be deactivated when m_bNoQtbttDualTreeIntraConstraintFlag is equal to 1");

    m_layerEncoder.setMaxLog2CtuSizeConstraintIdc(m_maxLog2CtuSizeConstraintIdc);
    CHECK( m_uiCTUSize > (1<<(m_maxLog2CtuSizeConstraintIdc)), "CTUSize shall be less than or equal to 1 << m_maxLog2CtuSize");

    m_layerEncoder.setNoPartitionConstraintsOverrideConstraintFlag(m_noPartitionConstraintsOverrideConstraintFlag);
    CHECK(m_noPartitionConstraintsOverrideConstraintFlag && m_SplitConsOverrideEnabledFlag, "Partition override shall be deactivated when m_noPartitionConstraintsOverrideConstraintFlag is equal to 1");

    m_layerEncoder.setNoSaoConstraintFlag(m_noSaoConstraintFlag);
    CHECK(m_noSaoConstraintFlag && m_bUseSAO, "SAO shall be deactivated when m_bNoSaoConstraintFlag is equal to 1");

    m_layerEncoder.setNoAlfConstraintFlag(m_noAlfConstraintFlag);
    CHECK(m_noAlfConstraintFlag && m_alf, "ALF shall be deactivated when m_bNoAlfConstraintFlag is equal to 1");

    m_layerEncoder.setNoCCAlfConstraintFlag(m_noCCAlfConstraintFlag);
    CHECK(m_noCCAlfConstraintFlag && m_ccalf, "CCALF shall be deactivated when m_noCCAlfConstraintFlag is equal to 1");

    m_layerEncoder.setNoWeightedPredictionConstraintFlag(m_noWeightedPredictionConstraintFlag);
    CHECK(m_noWeightedPredictionConstraintFlag && (m_useWeightedPred || m_useWeightedBiPred), "Weighted Prediction shall be deactivated when m_bNoWeightedPredictionConstraintFlag is equal to 1");

    m_layerEncoder.setNoRefWraparoundConstraintFlag(m_noRefWraparoundConstraintFlag);
    CHECK(m_noRefWraparoundConstraintFlag && m_wrapAround, "Wrap around shall be deactivated when m_bNoRefWraparoundConstraintFlag is equal to 1");

    m_layerEncoder.setNoTemporalMvpConstraintFlag(m_noTemporalMvpConstraintFlag);
    CHECK(m_noTemporalMvpConstraintFlag && m_TMVPModeId, "Temporal MVP shall be deactivated when m_bNoTemporalMvpConstraintFlag is equal to 1");

    m_layerEncoder.setNoSbtmvpConstraintFlag(m_noSbtmvpConstraintFlag);
    CHECK(m_noSbtmvpConstraintFlag && m_sbTmvpEnableFlag,
          "SbTMVP shall be deactivated when m_bNoSbtmvpConstraintFlag is equal to 1");

    m_layerEncoder.setNoAmvrConstraintFlag(m_noAmvrConstraintFlag);
    CHECK(m_noAmvrConstraintFlag && (m_ImvMode != IMV_OFF || m_AffineAmvr), "AMVR shall be deactivated when m_bNoAmvrConstraintFlag is equal to 1");

    m_layerEncoder.setNoBdofConstraintFlag(m_noBdofConstraintFlag);
    CHECK(m_noBdofConstraintFlag && m_BIO, "BIO shall be deactivated when m_bNoBdofConstraintFlag is equal to 1");

    m_layerEncoder.setNoDmvrConstraintFlag(m_noDmvrConstraintFlag);
    CHECK(m_noDmvrConstraintFlag && m_DMVR, "DMVR shall be deactivated when m_noDmvrConstraintFlag is equal to 1");

    m_layerEncoder.setNoCclmConstraintFlag(m_noCclmConstraintFlag);
    CHECK(m_noCclmConstraintFlag && m_LMChroma, "CCLM shall be deactivated when m_bNoCclmConstraintFlag is equal to 1");

    m_layerEncoder.setNoMtsConstraintFlag(m_noMtsConstraintFlag);
    CHECK(m_noMtsConstraintFlag && (m_MTS || m_MTSImplicit), "MTS shall be deactivated when m_bNoMtsConstraintFlag is equal to 1");

    m_layerEncoder.setNoSbtConstraintFlag(m_noSbtConstraintFlag);
    CHECK(m_noSbtConstraintFlag && m_SBT, "SBT shall be deactivated when mm_noSbtConstraintFlag_nonPackedConstraintFlag is equal to 1");

    m_layerEncoder.setNoAffineMotionConstraintFlag(m_noAffineMotionConstraintFlag);
    CHECK(m_noAffineMotionConstraintFlag && m_Affine, "Affine shall be deactivated when m_bNoAffineMotionConstraintFlag is equal to 1");

    m_layerEncoder.setNoBcwConstraintFlag(m_noBcwConstraintFlag);
    CHECK(m_noBcwConstraintFlag && m_bcw, "BCW shall be deactivated when m_bNoBcwConstraintFlag is equal to 1");

    m_layerEncoder.setNoIbcConstraintFlag(m_noIbcConstraintFlag);
    CHECK(m_noIbcConstraintFlag && m_IBCMode, "IBC shall be deactivated when m_noIbcConstraintFlag is equal to 1");

    m_layerEncoder.setNoCiipConstraintFlag(m_noCiipConstraintFlag);
    CHECK(m_noCiipConstraintFlag && m_ciip, "CIIP shall be deactivated when m_bNoCiipConstraintFlag is equal to 1");

    m_layerEncoder.setNoGeoConstraintFlag(m_noGeoConstraintFlag);
    CHECK(m_noGeoConstraintFlag && m_Geo, "GEO shall be deactivated when m_noGeoConstraintFlag is equal to 1");

    m_layerEncoder.setNoLadfConstraintFlag(m_noLadfConstraintFlag);
    CHECK(m_noLadfConstraintFlag && m_LadfEnabed, "LADF shall be deactivated when m_bNoLadfConstraintFlag is equal to 1");

    m_layerEncoder.setNoTransformSkipConstraintFlag(m_noTransformSkipConstraintFlag);
    CHECK(m_noTransformSkipConstraintFlag && m_useTransformSkip, "Transform skip shall be deactivated when m_noTransformSkipConstraintFlag is equal to 1");

    m_layerEncoder.setNoLumaTransformSize64ConstraintFlag(m_noLumaTransformSize64ConstraintFlag);
    CHECK(m_noLumaTransformSize64ConstraintFlag && m_log2MaxTbSize > 5, "Max transform size shall be less than 64 when m_noLumaTransformSize64ConstraintFlag is equal to 1");

    m_layerEncoder.setNoBDPCMConstraintFlag(m_noBDPCMConstraintFlag);
    CHECK(m_noBDPCMConstraintFlag && m_useBDPCM, "BDPCM shall be deactivated when m_noBDPCMConstraintFlag is equal to 1");

    m_layerEncoder.setNoJointCbCrConstraintFlag(m_noJointCbCrConstraintFlag);
    CHECK(m_noJointCbCrConstraintFlag && m_JointCbCrMode, "JCCR shall be deactivated when m_noJointCbCrConstraintFlag is equal to 1");

    m_layerEncoder.setNoDepQuantConstraintFlag(m_noDepQuantConstraintFlag);
    CHECK(m_noDepQuantConstraintFlag && m_depQuantEnabledFlag, "DQ shall be deactivated when m_bNoDepQuantConstraintFlag is equal to 1");

    m_layerEncoder.setNoSignDataHidingConstraintFlag(m_noSignDataHidingConstraintFlag);
    CHECK(m_noSignDataHidingConstraintFlag && m_signDataHidingEnabledFlag, "SDH shall be deactivated when m_bNoSignDataHidingConstraintFlag is equal to 1");

    m_layerEncoder.setNoApsConstraintFlag(m_noApsConstraintFlag);
    CHECK(m_noApsConstraintFlag && (m_lmcsEnabled || (m_useScalingListId != SCALING_LIST_OFF)), "LMCS and explict scaling list shall be deactivated when m_noApsConstraintFlag is equal to 1");

    m_layerEncoder.setNoMrlConstraintFlag(m_noMrlConstraintFlag);
    CHECK(m_noMrlConstraintFlag && m_MRL, "MRL shall be deactivated when m_noMrlConstraintFlag is equal to 1");

    m_layerEncoder.setNoIspConstraintFlag(m_noIspConstraintFlag);
    CHECK(m_noIspConstraintFlag && m_ISP, "ISP shall be deactivated when m_noIspConstraintFlag is equal to 1");

    m_layerEncoder.setNoMipConstraintFlag(m_noMipConstraintFlag);
    CHECK(m_noMipConstraintFlag && m_MIP, "MIP shall be deactivated when m_noMipConstraintFlag is equal to 1");

    m_layerEncoder.setNoLfnstConstraintFlag(m_noLfnstConstraintFlag);
    CHECK(m_noLfnstConstraintFlag && m_LFNST, "LFNST shall be deactivated when m_noLfnstConstraintFlag is equal to 1");

    m_layerEncoder.setNoMmvdConstraintFlag(m_noMmvdConstraintFlag);
    CHECK(m_noMmvdConstraintFlag && m_MMVD, "MMVD shall be deactivated when m_noMmvdConstraintFlag is equal to 1");

    m_layerEncoder.setNoSmvdConstraintFlag(m_noSmvdConstraintFlag);
    CHECK(m_noSmvdConstraintFlag && m_SMVD, "SMVD shall be deactivated when m_noSmvdConstraintFlag is equal to 1");

    m_layerEncoder.setNoProfConstraintFlag(m_noProfConstraintFlag);
    CHECK(m_noProfConstraintFlag && m_PROF, "PROF shall be deactivated when m_noProfConstraintFlag is equal to 1");

    m_layerEncoder.setNoPaletteConstraintFlag(m_noPaletteConstraintFlag);
    CHECK(m_noPaletteConstraintFlag && m_PLTMode, "Palette shall be deactivated when m_noPaletteConstraintFlag is equal to 1");

    m_layerEncoder.setNoActConstraintFlag(m_noActConstraintFlag);
    CHECK(m_noActConstraintFlag && m_useColorTrans, "ACT shall be deactivated when m_noActConstraintFlag is equal to 1");

    m_layerEncoder.setNoLmcsConstraintFlag(m_noLmcsConstraintFlag);
    CHECK(m_noLmcsConstraintFlag && m_lmcsEnabled, "LMCS shall be deactivated when m_noLmcsConstraintFlag is equal to 1");

    m_layerEncoder.setNoExplicitScaleListConstraintFlag(m_noExplicitScaleListConstraintFlag);
    CHECK(m_noExplicitScaleListConstraintFlag && m_useScalingListId != SCALING_LIST_OFF, "Explicit scaling list shall be deactivated when m_noExplicitScaleListConstraintFlag is equal to 1");

    m_layerEncoder.setNoVirtualBoundaryConstraintFlag(m_noVirtualBoundaryConstraintFlag);
    CHECK(m_noVirtualBoundaryConstraintFlag && m_virtualBoundariesEnabledFlag, "Virtuall boundaries shall be deactivated when m_noVirtualBoundaryConstraintFlag is equal to 1");
    m_layerEncoder.setNoChromaQpOffsetConstraintFlag(m_noChromaQpOffsetConstraintFlag);
    CHECK(m_noChromaQpOffsetConstraintFlag && m_cuChromaQpOffsetSubdiv, "Chroma Qp offset shall be 0 when m_noChromaQpOffsetConstraintFlag is equal to 1");
  }
  else
  {
    m_layerEncoder.setNonPackedConstraintFlag(false);
    m_layerEncoder.setNonProjectedConstraintFlag(false);
    m_layerEncoder.setAllLayersIndependentConstraintFlag(false);
    m_layerEncoder.setNoResChangeInClvsConstraintFlag(false);
    m_layerEncoder.setOneTilePerPicConstraintFlag(false);
    m_layerEncoder.setPicHeaderInSliceHeaderConstraintFlag(false);
    m_layerEncoder.setOneSlicePerPicConstraintFlag(false);
    m_layerEncoder.setNoIdrRplConstraintFlag(false);
    m_layerEncoder.setNoRectSliceConstraintFlag(false);
    m_layerEncoder.setOneSlicePerSubpicConstraintFlag(false);
    m_layerEncoder.setNoSubpicInfoConstraintFlag(false);
    m_layerEncoder.setOnePictureOnlyConstraintFlag(false);
    m_layerEncoder.setIntraOnlyConstraintFlag(false);
    m_layerEncoder.setMaxBitDepthConstraintIdc(16);
    m_layerEncoder.setMaxChromaFormatConstraintIdc(3);
    m_layerEncoder.setNoMttConstraintFlag(false);
    m_layerEncoder.setNoQtbttDualTreeIntraConstraintFlag(false);
    m_layerEncoder.setNoPartitionConstraintsOverrideConstraintFlag(false);
    m_layerEncoder.setNoSaoConstraintFlag(false);
    m_layerEncoder.setNoAlfConstraintFlag(false);
    m_layerEncoder.setNoCCAlfConstraintFlag(false);
    m_layerEncoder.setNoWeightedPredictionConstraintFlag(false);
    m_layerEncoder.setNoRefWraparoundConstraintFlag(false);
    m_layerEncoder.setNoTemporalMvpConstraintFlag(false);
    m_layerEncoder.setNoSbtmvpConstraintFlag(false);
    m_layerEncoder.setNoAmvrConstraintFlag(false);
    m_layerEncoder.setNoBdofConstraintFlag(false);
    m_layerEncoder.setNoDmvrConstraintFlag(false);
    m_layerEncoder.setNoCclmConstraintFlag(false);
    m_layerEncoder.setNoMtsConstraintFlag(false);
    m_layerEncoder.setNoSbtConstraintFlag(false);
    m_layerEncoder.setNoAffineMotionConstraintFlag(false);
    m_layerEncoder.setNoBcwConstraintFlag(false);
    m_layerEncoder.setNoIbcConstraintFlag(false);
    m_layerEncoder.setNoCiipConstraintFlag(false);
    m_layerEncoder.setNoGeoConstraintFlag(false);
    m_layerEncoder.setNoLadfConstraintFlag(false);
    m_layerEncoder.setNoTransformSkipConstraintFlag(false);
    m_layerEncoder.setNoBDPCMConstraintFlag(false);
    m_layerEncoder.setNoJointCbCrConstraintFlag(false);
    m_layerEncoder.setNoCuQpDeltaConstraintFlag(false);
    m_layerEncoder.setNoDepQuantConstraintFlag(false);
    m_layerEncoder.setNoSignDataHidingConstraintFlag(false);
    m_layerEncoder.setNoTrailConstraintFlag(false);
    m_layerEncoder.setNoStsaConstraintFlag(false);
    m_layerEncoder.setNoRaslConstraintFlag(false);
    m_layerEncoder.setNoRadlConstraintFlag(false);
    m_layerEncoder.setNoIdrConstraintFlag(false);
    m_layerEncoder.setNoCraConstraintFlag(false);
    m_layerEncoder.setNoGdrConstraintFlag(false);
    m_layerEncoder.setNoApsConstraintFlag(false);
    m_layerEncoder.setNoMrlConstraintFlag(false);
    m_layerEncoder.setNoIspConstraintFlag(false);
    m_layerEncoder.setNoMipConstraintFlag(false);
    m_layerEncoder.setNoLfnstConstraintFlag(false);
    m_layerEncoder.setNoMmvdConstraintFlag(false);
    m_layerEncoder.setNoSmvdConstraintFlag(false);
    m_layerEncoder.setNoProfConstraintFlag(false);
    m_layerEncoder.setNoPaletteConstraintFlag(false);
    m_layerEncoder.setNoActConstraintFlag(false);
    m_layerEncoder.setNoLmcsConstraintFlag(false);
    m_layerEncoder.setNoChromaQpOffsetConstraintFlag(false);
  }

  //====== Coding Structure ========
  m_layerEncoder.setIntraPeriod                                       ( m_iIntraPeriod );
  m_layerEncoder.setDecodingRefreshType                               ( m_iDecodingRefreshType );
  m_layerEncoder.setGOPSize                                           ( m_iGOPSize );
  m_layerEncoder.setDrapPeriod                                        ( m_drapPeriod );
  m_layerEncoder.setReWriteParamSets                                  ( m_rewriteParamSets );
  m_layerEncoder.setRPLList0                                          ( m_RPLList0);
  m_layerEncoder.setRPLList1                                          ( m_RPLList1);
  m_layerEncoder.setIDRRefParamListPresent                            ( m_idrRefParamList );
  m_layerEncoder.setGopList                                           ( m_GOPList );

  for(int i = 0; i < MAX_TLAYER; i++)
  {
    m_layerEncoder.setMaxNumReorderPics                               ( m_maxNumReorderPics[i], i );
    m_layerEncoder.setMaxDecPicBuffering                              ( m_maxDecPicBuffering[i], i );
  }
  for( uint32_t uiLoop = 0; uiLoop < MAX_TLAYER; ++uiLoop )
  {
    m_layerEncoder.setLambdaModifier                                  ( uiLoop, m_adLambdaModifier[ uiLoop ] );
  }
  m_layerEncoder.setIntraLambdaModifier                               ( m_adIntraLambdaModifier );
  m_layerEncoder.setIntraQpFactor                                     ( m_dIntraQpFactor );

  m_layerEncoder.setBaseQP                                            ( m_iQP );

#if X0038_LAMBDA_FROM_QP_CAPABILITY
  m_layerEncoder.setIntraQPOffset                                     ( m_intraQPOffset );
  m_layerEncoder.setLambdaFromQPEnable                                ( m_lambdaFromQPEnable );
#endif
  m_layerEncoder.setChromaQpMappingTableParams                         (m_chromaQpMappingTableParams);

  m_layerEncoder.setPad                                               ( m_aiPad );

  m_layerEncoder.setAccessUnitDelimiter                               ( m_AccessUnitDelimiter );
  m_layerEncoder.setEnablePictureHeaderInSliceHeader                  ( m_enablePictureHeaderInSliceHeader );

  m_layerEncoder.setMaxTempLayer                                      ( m_maxTempLayer );

  //===== Slice ========

  //====== Loop/Deblock Filter ========
  m_layerEncoder.setLoopFilterDisable                                 ( m_bLoopFilterDisable       );
  m_layerEncoder.setLoopFilterOffsetInPPS                             ( m_loopFilterOffsetInPPS );
  m_layerEncoder.setLoopFilterBetaOffset                              ( m_loopFilterBetaOffsetDiv2  );
  m_layerEncoder.setLoopFilterTcOffset                                ( m_loopFilterTcOffsetDiv2    );
  m_layerEncoder.setLoopFilterCbBetaOffset                            ( m_loopFilterCbBetaOffsetDiv2  );
  m_layerEncoder.setLoopFilterCbTcOffset                              ( m_loopFilterCbTcOffsetDiv2    );
  m_layerEncoder.setLoopFilterCrBetaOffset                            ( m_loopFilterCrBetaOffsetDiv2  );
  m_layerEncoder.setLoopFilterCrTcOffset                              ( m_loopFilterCrTcOffsetDiv2    );
#if W0038_DB_OPT
  m_layerEncoder.setDeblockingFilterMetric                            ( m_deblockingFilterMetric );
#else
  m_cEncLib.setDeblockingFilterMetric                            ( m_DeblockingFilterMetric );
#endif

  //====== Motion search ========
  m_layerEncoder.setDisableIntraPUsInInterSlices                      ( m_bDisableIntraPUsInInterSlices );
  m_layerEncoder.setMotionEstimationSearchMethod                      ( m_motionEstimationSearchMethod  );
  m_layerEncoder.setSearchRange                                       ( m_iSearchRange );
  m_layerEncoder.setBipredSearchRange                                 ( m_bipredSearchRange );
  m_layerEncoder.setClipForBiPredMeEnabled                            ( m_bClipForBiPredMeEnabled );
  m_layerEncoder.setFastMEAssumingSmootherMVEnabled                   ( m_bFastMEAssumingSmootherMVEnabled );
  m_layerEncoder.setMinSearchWindow                                   ( m_minSearchWindow );
  m_layerEncoder.setRestrictMESampling                                ( m_bRestrictMESampling );

  //====== Quality control ========
  m_layerEncoder.setMaxDeltaQP                                        ( m_iMaxDeltaQP  );
  m_layerEncoder.setCuQpDeltaSubdiv                                   ( m_cuQpDeltaSubdiv );
  m_layerEncoder.setCuChromaQpOffsetSubdiv                            ( m_cuChromaQpOffsetSubdiv );
  m_layerEncoder.setChromaCbQpOffset                                  ( m_cbQpOffset     );
  m_layerEncoder.setChromaCrQpOffset                                  ( m_crQpOffset  );
  m_layerEncoder.setChromaCbQpOffsetDualTree                          ( m_cbQpOffsetDualTree );
  m_layerEncoder.setChromaCrQpOffsetDualTree                          ( m_crQpOffsetDualTree );
  m_layerEncoder.setChromaCbCrQpOffset                                ( m_cbCrQpOffset         );
  m_layerEncoder.setChromaCbCrQpOffsetDualTree                        ( m_cbCrQpOffsetDualTree );
#if ER_CHROMA_QP_WCG_PPS
  m_layerEncoder.setWCGChromaQpControl                                ( m_wcgChromaQpControl );
#endif
#if W0038_CQP_ADJ
  m_layerEncoder.setSliceChromaOffsetQpIntraOrPeriodic                ( m_sliceChromaQpOffsetPeriodicity, m_sliceChromaQpOffsetIntraOrPeriodic );
#endif
  m_layerEncoder.setChromaFormatIdc                                   ( m_chromaFormatIDC  );
  m_layerEncoder.setUseAdaptiveQP                                     ( m_bUseAdaptiveQP  );
  m_layerEncoder.setQPAdaptationRange                                 ( m_iQPAdaptationRange );
#if ENABLE_QPA
  m_layerEncoder.setUsePerceptQPA                                     ( m_bUsePerceptQPA && !m_bUseAdaptiveQP );
  m_layerEncoder.setUseWPSNR                                          ( m_bUseWPSNR );
#endif
  m_layerEncoder.setExtendedPrecisionProcessingFlag                   ( m_extendedPrecisionProcessingFlag );
  m_layerEncoder.setHighPrecisionOffsetsEnabledFlag                   ( m_highPrecisionOffsetsEnabledFlag );

  m_layerEncoder.setWeightedPredictionMethod( m_weightedPredictionMethod );

  //====== Tool list ========
#if SHARP_LUMA_DELTA_QP
  m_layerEncoder.setLumaLevelToDeltaQPControls                        ( m_lumaLevelToDeltaQPMapping );
#endif
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  m_layerEncoder.setDeltaQpRD( (m_costMode==COST_LOSSLESS_CODING) ? 0 : m_uiDeltaQpRD );
#else
  m_cEncLib.setDeltaQpRD                                         ( m_uiDeltaQpRD  );
#endif
  m_layerEncoder.setFastDeltaQp                                       ( m_bFastDeltaQP  );
  m_layerEncoder.setUseASR                                            ( m_bUseASR      );
  m_layerEncoder.setUseHADME                                          ( m_bUseHADME    );
  m_layerEncoder.setdQPs                                              ( m_aidQP        );
  m_layerEncoder.setUseRDOQ                                           ( m_useRDOQ     );
  m_layerEncoder.setUseRDOQTS                                         ( m_useRDOQTS   );
#if T0196_SELECTIVE_RDOQ
  m_layerEncoder.setUseSelectiveRDOQ                                  ( m_useSelectiveRDOQ );
#endif
  m_layerEncoder.setRDpenalty                                         ( m_rdPenalty );
  m_layerEncoder.setCTUSize                                           ( m_uiCTUSize );
  m_layerEncoder.setSubPicInfoPresentFlag                             ( m_subPicInfoPresentFlag );
  if(m_subPicInfoPresentFlag)
  {
    m_layerEncoder.setNumSubPics                                      ( m_numSubPics );
    m_layerEncoder.setSubPicSameSizeFlag                              ( m_subPicSameSizeFlag );
    m_layerEncoder.setSubPicCtuTopLeftX                               ( m_subPicCtuTopLeftX );
    m_layerEncoder.setSubPicCtuTopLeftY                               ( m_subPicCtuTopLeftY );
    m_layerEncoder.setSubPicWidth                                     ( m_subPicWidth );
    m_layerEncoder.setSubPicHeight                                    ( m_subPicHeight );
    m_layerEncoder.setSubPicTreatedAsPicFlag                          ( m_subPicTreatedAsPicFlag );
    m_layerEncoder.setLoopFilterAcrossSubpicEnabledFlag               ( m_loopFilterAcrossSubpicEnabledFlag );
    m_layerEncoder.setSubPicIdMappingInSpsFlag                        ( m_subPicIdMappingInSpsFlag );
    m_layerEncoder.setSubPicIdLen                                     ( m_subPicIdLen );
    m_layerEncoder.setSubPicIdMappingExplicitlySignalledFlag          ( m_subPicIdMappingExplicitlySignalledFlag );
    if (m_subPicIdMappingExplicitlySignalledFlag)
    {
      m_layerEncoder.setSubPicId                                      ( m_subPicId );
    }
  }
  else
  {
    m_layerEncoder.setNumSubPics                                      ( 1 );
    m_layerEncoder.setSubPicIdMappingExplicitlySignalledFlag          ( false );
  }

  m_layerEncoder.setUseSplitConsOverride                              ( m_SplitConsOverrideEnabledFlag );
  // convert the Intra Chroma minQT setting from chroma unit to luma unit
  m_uiMinQT[2] <<= getChannelTypeScaleX(CHANNEL_TYPE_CHROMA, m_chromaFormatIDC);
  m_layerEncoder.setMinQTSizes                                        ( m_uiMinQT );
  m_layerEncoder.setMaxMTTHierarchyDepth                              ( m_uiMaxMTTHierarchyDepth, m_uiMaxMTTHierarchyDepthI, m_uiMaxMTTHierarchyDepthIChroma );
  m_layerEncoder.setMaxBTSizes                                        ( m_uiMaxBT );
  m_layerEncoder.setMaxTTSizes                                        ( m_uiMaxTT );
  m_layerEncoder.setDualITree                                         ( m_dualTree );
  m_layerEncoder.setLFNST                                             ( m_LFNST );
  m_layerEncoder.setUseFastLFNST                                      ( m_useFastLFNST );
  m_layerEncoder.setSbTmvpEnabledFlag(m_sbTmvpEnableFlag);
  m_layerEncoder.setAffine                                            ( m_Affine );
  m_layerEncoder.setAffineType                                        ( m_AffineType );
  m_layerEncoder.setPROF                                              ( m_PROF );
  m_layerEncoder.setBIO                                               (m_BIO);
  m_layerEncoder.setUseLMChroma                                       ( m_LMChroma );
  m_layerEncoder.setHorCollocatedChromaFlag                           ( m_horCollocatedChromaFlag );
  m_layerEncoder.setVerCollocatedChromaFlag                           ( m_verCollocatedChromaFlag );
  m_layerEncoder.setIntraMTS                                          ( m_MTS & 1 );
  m_layerEncoder.setInterMTS                                          ( ( m_MTS >> 1 ) & 1 );
  m_layerEncoder.setMTSIntraMaxCand                                   ( m_MTSIntraMaxCand );
  m_layerEncoder.setMTSInterMaxCand                                   ( m_MTSInterMaxCand );
  m_layerEncoder.setImplicitMTS                                       ( m_MTSImplicit );
  m_layerEncoder.setUseSBT                                            ( m_SBT );
  m_layerEncoder.setSBTFast64WidthTh                                  ( m_SBTFast64WidthTh );
  m_layerEncoder.setUseCompositeRef                                   ( m_compositeRefEnabled );
  m_layerEncoder.setUseSMVD                                           ( m_SMVD );
  m_layerEncoder.setUseBcw                                            ( m_bcw );
  m_layerEncoder.setUseBcwFast                                        ( m_BcwFast );
#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
  m_layerEncoder.setUseLadf                                           ( m_LadfEnabed );
  if ( m_LadfEnabed )
  {
    m_layerEncoder.setLadfNumIntervals                                ( m_LadfNumIntervals);
    for ( int k = 0; k < m_LadfNumIntervals; k++ )
    {
      m_layerEncoder.setLadfQpOffset( m_LadfQpOffset[k], k );
      m_layerEncoder.setLadfIntervalLowerBound(m_LadfIntervalLowerBound[k], k);
    }
  }
#endif
  m_layerEncoder.setUseCiip                                        ( m_ciip );
  m_layerEncoder.setUseGeo                                            ( m_Geo );
  m_layerEncoder.setUseHashME                                         ( m_HashME );

  m_layerEncoder.setAllowDisFracMMVD                                  ( m_allowDisFracMMVD );
  m_layerEncoder.setUseAffineAmvr                                     ( m_AffineAmvr );
  m_layerEncoder.setUseAffineAmvrEncOpt                               ( m_AffineAmvrEncOpt );
  m_layerEncoder.setDMVR                                              ( m_DMVR );
  m_layerEncoder.setMMVD                                              ( m_MMVD );
  m_layerEncoder.setMmvdDisNum                                        (m_MmvdDisNum);
  m_layerEncoder.setRGBFormatFlag(m_rgbFormat);
  m_layerEncoder.setUseColorTrans(m_useColorTrans);
  m_layerEncoder.setPLTMode                                           ( m_PLTMode );
  m_layerEncoder.setJointCbCr                                         ( m_JointCbCrMode );
  m_layerEncoder.setIBCMode                                           ( m_IBCMode );
  m_layerEncoder.setIBCLocalSearchRangeX                              ( m_IBCLocalSearchRangeX );
  m_layerEncoder.setIBCLocalSearchRangeY                              ( m_IBCLocalSearchRangeY );
  m_layerEncoder.setIBCHashSearch                                     ( m_IBCHashSearch );
  m_layerEncoder.setIBCHashSearchMaxCand                              ( m_IBCHashSearchMaxCand );
  m_layerEncoder.setIBCHashSearchRange4SmallBlk                       ( m_IBCHashSearchRange4SmallBlk );
  m_layerEncoder.setIBCFastMethod                                     ( m_IBCFastMethod );

  m_layerEncoder.setUseWrapAround                                     ( m_wrapAround );
  m_layerEncoder.setWrapAroundOffset                                  ( m_wrapAroundOffset );

  // ADD_NEW_TOOL : (encoder app) add setting of tool enabling flags and associated parameters here
  m_layerEncoder.setVirtualBoundariesEnabledFlag                      ( m_virtualBoundariesEnabledFlag );
  if( m_layerEncoder.getVirtualBoundariesEnabledFlag() )
  {
    m_layerEncoder.setVirtualBoundariesPresentFlag                      ( m_virtualBoundariesPresentFlag );
    m_layerEncoder.setNumVerVirtualBoundaries                           ( m_numVerVirtualBoundaries );
    m_layerEncoder.setNumHorVirtualBoundaries                           ( m_numHorVirtualBoundaries );
    for( unsigned i = 0; i < m_numVerVirtualBoundaries; i++ )
    {
      m_layerEncoder.setVirtualBoundariesPosX                           ( m_virtualBoundariesPosX[ i ], i );
    }
    for( unsigned i = 0; i < m_numHorVirtualBoundaries; i++ )
    {
      m_layerEncoder.setVirtualBoundariesPosY                           ( m_virtualBoundariesPosY[ i ], i );
    }
  }

  m_layerEncoder.setMaxCUWidth                                        ( m_uiCTUSize );
  m_layerEncoder.setMaxCUHeight                                       ( m_uiCTUSize );
  m_layerEncoder.setLog2MinCodingBlockSize                            ( m_log2MinCuSize );
  m_layerEncoder.setLog2MaxTbSize                                     ( m_log2MaxTbSize );
  m_layerEncoder.setUseEncDbOpt(m_encDbOpt);
  m_layerEncoder.setUseFastLCTU                                       ( m_useFastLCTU );
  m_layerEncoder.setFastInterSearchMode                               ( m_fastInterSearchMode );
  m_layerEncoder.setUseEarlyCU                                        ( m_bUseEarlyCU  );
  m_layerEncoder.setUseFastDecisionForMerge                           ( m_useFastDecisionForMerge  );
  m_layerEncoder.setUseEarlySkipDetection                             ( m_useEarlySkipDetection );
  m_layerEncoder.setUseFastMerge                                      ( m_useFastMrg );
  m_layerEncoder.setUsePbIntraFast                                    ( m_usePbIntraFast );
  m_layerEncoder.setUseAMaxBT                                         ( m_useAMaxBT );
  m_layerEncoder.setUseE0023FastEnc                                   ( m_e0023FastEnc );
  m_layerEncoder.setUseContentBasedFastQtbt                           ( m_contentBasedFastQtbt );
  m_layerEncoder.setUseNonLinearAlfLuma                               ( m_useNonLinearAlfLuma );
  m_layerEncoder.setUseNonLinearAlfChroma                             ( m_useNonLinearAlfChroma );
  m_layerEncoder.setMaxNumAlfAlternativesChroma                       ( m_maxNumAlfAlternativesChroma );
  m_layerEncoder.setUseMRL                                            ( m_MRL );
  m_layerEncoder.setUseMIP                                            ( m_MIP );
  m_layerEncoder.setUseFastMIP                                        ( m_useFastMIP );
  m_layerEncoder.setFastLocalDualTreeMode                             ( m_fastLocalDualTreeMode );
  m_layerEncoder.setUseReconBasedCrossCPredictionEstimate             ( m_reconBasedCrossCPredictionEstimate );
  m_layerEncoder.setUseTransformSkip                                  ( m_useTransformSkip      );
  m_layerEncoder.setUseTransformSkipFast                              ( m_useTransformSkipFast  );
  m_layerEncoder.setUseChromaTS                                       ( m_useChromaTS && m_useTransformSkip);
  m_layerEncoder.setUseBDPCM                                          ( m_useBDPCM );
  m_layerEncoder.setTransformSkipRotationEnabledFlag                  ( m_transformSkipRotationEnabledFlag );
  m_layerEncoder.setTransformSkipContextEnabledFlag                   ( m_transformSkipContextEnabledFlag   );
  m_layerEncoder.setPersistentRiceAdaptationEnabledFlag               ( m_persistentRiceAdaptationEnabledFlag );
  m_layerEncoder.setCabacBypassAlignmentEnabledFlag                   ( m_cabacBypassAlignmentEnabledFlag );
  m_layerEncoder.setLog2MaxTransformSkipBlockSize                     ( m_log2MaxTransformSkipBlockSize  );
  m_layerEncoder.setFastUDIUseMPMEnabled                              ( m_bFastUDIUseMPMEnabled );
  m_layerEncoder.setFastMEForGenBLowDelayEnabled                      ( m_bFastMEForGenBLowDelayEnabled );
  m_layerEncoder.setUseBLambdaForNonKeyLowDelayPictures               ( m_bUseBLambdaForNonKeyLowDelayPictures );
  m_layerEncoder.setUseISP                                            ( m_ISP );
  m_layerEncoder.setUseFastISP                                        ( m_useFastISP );

  // set internal bit-depth and constants
  for (uint32_t channelType = 0; channelType < MAX_NUM_CHANNEL_TYPE; channelType++)
  {
    m_layerEncoder.setBitDepth((ChannelType)channelType, m_internalBitDepth[channelType]);
    m_layerEncoder.setInputBitDepth((ChannelType)channelType, m_inputBitDepth[channelType]);
  }

  m_layerEncoder.setMaxNumMergeCand                                   ( m_maxNumMergeCand );
  m_layerEncoder.setMaxNumAffineMergeCand                             ( m_maxNumAffineMergeCand );
  m_layerEncoder.setMaxNumGeoCand                                     ( m_maxNumGeoCand );
  m_layerEncoder.setMaxNumIBCMergeCand                                ( m_maxNumIBCMergeCand );

  //====== Weighted Prediction ========
  m_layerEncoder.setUseWP                                             ( m_useWeightedPred     );
  m_layerEncoder.setWPBiPred                                          ( m_useWeightedBiPred   );

  //====== Parallel Merge Estimation ========
  m_layerEncoder.setLog2ParallelMergeLevelMinus2(m_log2ParallelMergeLevel - 2);
  m_layerEncoder.setMixedLossyLossless(m_mixedLossyLossless);
  m_layerEncoder.setSliceLosslessArray(m_sliceLosslessArray);

  //====== Tiles and Slices ========
  m_layerEncoder.setNoPicPartitionFlag( !m_picPartitionFlag );
  if( m_picPartitionFlag )
  {
    m_layerEncoder.setTileColWidths( m_tileColumnWidth );
    m_layerEncoder.setTileRowHeights( m_tileRowHeight );
    m_layerEncoder.setRectSliceFlag( !m_rasterSliceFlag );
    m_layerEncoder.setNumSlicesInPic( m_numSlicesInPic );
    m_layerEncoder.setTileIdxDeltaPresentFlag( m_tileIdxDeltaPresentFlag );
    m_layerEncoder.setRectSlices( m_rectSlices );
    m_layerEncoder.setRasterSliceSizes( m_rasterSliceSize );
    m_layerEncoder.setLFCrossTileBoundaryFlag( !m_disableLFCrossTileBoundaryFlag );
    m_layerEncoder.setLFCrossSliceBoundaryFlag( !m_disableLFCrossSliceBoundaryFlag );
  }
  else
  {
    m_layerEncoder.setRectSliceFlag( true );
    m_layerEncoder.setNumSlicesInPic( 1 );
    m_layerEncoder.setTileIdxDeltaPresentFlag( 0 );
    m_layerEncoder.setLFCrossTileBoundaryFlag( true );
    m_layerEncoder.setLFCrossSliceBoundaryFlag( true );
  }

  //====== Sub-picture and Slices ========
  m_layerEncoder.setSingleSlicePerSubPicFlagFlag                      ( m_singleSlicePerSubPicFlag );
  m_layerEncoder.setUseSAO                                            ( m_bUseSAO );
  m_layerEncoder.setTestSAODisableAtPictureLevel                      ( m_bTestSAODisableAtPictureLevel );
  m_layerEncoder.setSaoEncodingRate                                   ( m_saoEncodingRate );
  m_layerEncoder.setSaoEncodingRateChroma                             ( m_saoEncodingRateChroma );
  m_layerEncoder.setMaxNumOffsetsPerPic                               ( m_maxNumOffsetsPerPic);

  m_layerEncoder.setSaoCtuBoundary                                    ( m_saoCtuBoundary);

  m_layerEncoder.setSaoGreedyMergeEnc                                 ( m_saoGreedyMergeEnc);
  m_layerEncoder.setIntraSmoothingDisabledFlag                        (!m_enableIntraReferenceSmoothing );
  m_layerEncoder.setDecodedPictureHashSEIType                         ( m_decodedPictureHashSEIType );
  m_layerEncoder.setSubpicDecodedPictureHashType                      ( m_subpicDecodedPictureHashType );
  m_layerEncoder.setDependentRAPIndicationSEIEnabled                  ( m_drapPeriod > 0 );
  m_layerEncoder.setBufferingPeriodSEIEnabled                         ( m_bufferingPeriodSEIEnabled );
  m_layerEncoder.setPictureTimingSEIEnabled                           ( m_pictureTimingSEIEnabled );
  m_layerEncoder.setFrameFieldInfoSEIEnabled                          ( m_frameFieldInfoSEIEnabled );
   m_layerEncoder.setBpDeltasGOPStructure                             ( m_bpDeltasGOPStructure );
  m_layerEncoder.setDecodingUnitInfoSEIEnabled                        ( m_decodingUnitInfoSEIEnabled );
  m_layerEncoder.setScalableNestingSEIEnabled                         ( m_scalableNestingSEIEnabled );
  m_layerEncoder.setHrdParametersPresentFlag                          ( m_hrdParametersPresentFlag );
  m_layerEncoder.setFramePackingArrangementSEIEnabled                 ( m_framePackingSEIEnabled );
  m_layerEncoder.setFramePackingArrangementSEIType                    ( m_framePackingSEIType );
  m_layerEncoder.setFramePackingArrangementSEIId                      ( m_framePackingSEIId );
  m_layerEncoder.setFramePackingArrangementSEIQuincunx                ( m_framePackingSEIQuincunx );
  m_layerEncoder.setFramePackingArrangementSEIInterpretation          ( m_framePackingSEIInterpretation );
  m_layerEncoder.setParameterSetsInclusionIndicationSEIEnabled        (m_parameterSetsInclusionIndicationSEIEnabled);
  m_layerEncoder.setSelfContainedClvsFlag                             (m_selfContainedClvsFlag);
  m_layerEncoder.setErpSEIEnabled                                     ( m_erpSEIEnabled );
  m_layerEncoder.setErpSEICancelFlag                                  ( m_erpSEICancelFlag );
  m_layerEncoder.setErpSEIPersistenceFlag                             ( m_erpSEIPersistenceFlag );
  m_layerEncoder.setErpSEIGuardBandFlag                               ( m_erpSEIGuardBandFlag );
  m_layerEncoder.setErpSEIGuardBandType                               ( m_erpSEIGuardBandType );
  m_layerEncoder.setErpSEILeftGuardBandWidth                          ( m_erpSEILeftGuardBandWidth );
  m_layerEncoder.setErpSEIRightGuardBandWidth                         ( m_erpSEIRightGuardBandWidth );
  m_layerEncoder.setSphereRotationSEIEnabled                          ( m_sphereRotationSEIEnabled );
  m_layerEncoder.setSphereRotationSEICancelFlag                       ( m_sphereRotationSEICancelFlag );
  m_layerEncoder.setSphereRotationSEIPersistenceFlag                  ( m_sphereRotationSEIPersistenceFlag );
  m_layerEncoder.setSphereRotationSEIYaw                              ( m_sphereRotationSEIYaw );
  m_layerEncoder.setSphereRotationSEIPitch                            ( m_sphereRotationSEIPitch );
  m_layerEncoder.setSphereRotationSEIRoll                             ( m_sphereRotationSEIRoll );
  m_layerEncoder.setOmniViewportSEIEnabled                            ( m_omniViewportSEIEnabled );
  m_layerEncoder.setOmniViewportSEIId                                 ( m_omniViewportSEIId );
  m_layerEncoder.setOmniViewportSEICancelFlag                         ( m_omniViewportSEICancelFlag );
  m_layerEncoder.setOmniViewportSEIPersistenceFlag                    ( m_omniViewportSEIPersistenceFlag );
  m_layerEncoder.setOmniViewportSEICntMinus1                          ( m_omniViewportSEICntMinus1 );
  m_layerEncoder.setOmniViewportSEIAzimuthCentre                      ( m_omniViewportSEIAzimuthCentre );
  m_layerEncoder.setOmniViewportSEIElevationCentre                    ( m_omniViewportSEIElevationCentre );
  m_layerEncoder.setOmniViewportSEITiltCentre                         ( m_omniViewportSEITiltCentre );
  m_layerEncoder.setOmniViewportSEIHorRange                           ( m_omniViewportSEIHorRange );
  m_layerEncoder.setOmniViewportSEIVerRange                           ( m_omniViewportSEIVerRange );
#if JVET_T0053_ANNOTATED_REGIONS_SEI
  m_layerEncoder.setAnnotatedRegionSEIFileRoot                        ( m_arSEIFileRoot );
#endif
  m_layerEncoder.setRwpSEIEnabled                                     ( m_rwpSEIEnabled );
  m_layerEncoder.setRwpSEIRwpCancelFlag                               ( m_rwpSEIRwpCancelFlag );
  m_layerEncoder.setRwpSEIRwpPersistenceFlag                          ( m_rwpSEIRwpPersistenceFlag );
  m_layerEncoder.setRwpSEIConstituentPictureMatchingFlag              ( m_rwpSEIConstituentPictureMatchingFlag );
  m_layerEncoder.setRwpSEINumPackedRegions                            ( m_rwpSEINumPackedRegions );
  m_layerEncoder.setRwpSEIProjPictureWidth                            ( m_rwpSEIProjPictureWidth );
  m_layerEncoder.setRwpSEIProjPictureHeight                           ( m_rwpSEIProjPictureHeight );
  m_layerEncoder.setRwpSEIPackedPictureWidth                          ( m_rwpSEIPackedPictureWidth );
  m_layerEncoder.setRwpSEIPackedPictureHeight                         ( m_rwpSEIPackedPictureHeight );
  m_layerEncoder.setRwpSEIRwpTransformType                            ( m_rwpSEIRwpTransformType );
  m_layerEncoder.setRwpSEIRwpGuardBandFlag                            ( m_rwpSEIRwpGuardBandFlag );
  m_layerEncoder.setRwpSEIProjRegionWidth                             ( m_rwpSEIProjRegionWidth );
  m_layerEncoder.setRwpSEIProjRegionHeight                            ( m_rwpSEIProjRegionHeight );
  m_layerEncoder.setRwpSEIRwpSEIProjRegionTop                         ( m_rwpSEIRwpSEIProjRegionTop );
  m_layerEncoder.setRwpSEIProjRegionLeft                              ( m_rwpSEIProjRegionLeft );
  m_layerEncoder.setRwpSEIPackedRegionWidth                           ( m_rwpSEIPackedRegionWidth );
  m_layerEncoder.setRwpSEIPackedRegionHeight                          ( m_rwpSEIPackedRegionHeight );
  m_layerEncoder.setRwpSEIPackedRegionTop                             ( m_rwpSEIPackedRegionTop );
  m_layerEncoder.setRwpSEIPackedRegionLeft                            ( m_rwpSEIPackedRegionLeft );
  m_layerEncoder.setRwpSEIRwpLeftGuardBandWidth                       ( m_rwpSEIRwpLeftGuardBandWidth );
  m_layerEncoder.setRwpSEIRwpRightGuardBandWidth                      ( m_rwpSEIRwpRightGuardBandWidth );
  m_layerEncoder.setRwpSEIRwpTopGuardBandHeight                       ( m_rwpSEIRwpTopGuardBandHeight );
  m_layerEncoder.setRwpSEIRwpBottomGuardBandHeight                    ( m_rwpSEIRwpBottomGuardBandHeight );
  m_layerEncoder.setRwpSEIRwpGuardBandNotUsedForPredFlag              ( m_rwpSEIRwpGuardBandNotUsedForPredFlag );
  m_layerEncoder.setRwpSEIRwpGuardBandType                            ( m_rwpSEIRwpGuardBandType );
  m_layerEncoder.setGcmpSEIEnabled                                    ( m_gcmpSEIEnabled );
  m_layerEncoder.setGcmpSEICancelFlag                                 ( m_gcmpSEICancelFlag );
  m_layerEncoder.setGcmpSEIPersistenceFlag                            ( m_gcmpSEIPersistenceFlag );
  m_layerEncoder.setGcmpSEIPackingType                                ( (uint8_t) m_gcmpSEIPackingType );
  m_layerEncoder.setGcmpSEIMappingFunctionType                        ( (uint8_t) m_gcmpSEIMappingFunctionType );
  m_layerEncoder.setGcmpSEIFaceIndex                                  ( m_gcmpSEIFaceIndex );
  m_layerEncoder.setGcmpSEIFaceRotation                               ( m_gcmpSEIFaceRotation );
  m_layerEncoder.setGcmpSEIFunctionCoeffU                             ( m_gcmpSEIFunctionCoeffU );
  m_layerEncoder.setGcmpSEIFunctionUAffectedByVFlag                   ( m_gcmpSEIFunctionUAffectedByVFlag );
  m_layerEncoder.setGcmpSEIFunctionCoeffV                             ( m_gcmpSEIFunctionCoeffV );
  m_layerEncoder.setGcmpSEIFunctionVAffectedByUFlag                   ( m_gcmpSEIFunctionVAffectedByUFlag );
  m_layerEncoder.setGcmpSEIGuardBandFlag                              ( m_gcmpSEIGuardBandFlag );
  m_layerEncoder.setGcmpSEIGuardBandType                              ( m_gcmpSEIGuardBandType );
  m_layerEncoder.setGcmpSEIGuardBandBoundaryExteriorFlag              ( m_gcmpSEIGuardBandBoundaryExteriorFlag );
  m_layerEncoder.setGcmpSEIGuardBandSamplesMinus1                     ( (uint8_t) m_gcmpSEIGuardBandSamplesMinus1 );
  m_layerEncoder.setSubpicureLevelInfoSEICfg                          ( m_cfgSubpictureLevelInfoSEI );
  m_layerEncoder.setSampleAspectRatioInfoSEIEnabled                   ( m_sampleAspectRatioInfoSEIEnabled );
  m_layerEncoder.setSariCancelFlag                                    ( m_sariCancelFlag );
  m_layerEncoder.setSariPersistenceFlag                               ( m_sariPersistenceFlag );
  m_layerEncoder.setSariAspectRatioIdc                                ( m_sariAspectRatioIdc );
  m_layerEncoder.setSariSarWidth                                      ( m_sariSarWidth );
  m_layerEncoder.setSariSarHeight                                     ( m_sariSarHeight );
  m_layerEncoder.setMCTSEncConstraint                                 ( m_MCTSEncConstraint );
  m_layerEncoder.setMasteringDisplaySEI                               ( m_masteringDisplay );
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
  m_layerEncoder.setSEIAlternativeTransferCharacteristicsSEIEnable    ( m_preferredTransferCharacteristics>=0 );
  m_layerEncoder.setSEIPreferredTransferCharacteristics               ( uint8_t(m_preferredTransferCharacteristics) );
#endif
  // film grain charcteristics
  m_layerEncoder.setFilmGrainCharactersticsSEIEnabled                 ( m_fgcSEIEnabled );
  m_layerEncoder.setFilmGrainCharactersticsSEICancelFlag              ( m_fgcSEICancelFlag );
  m_layerEncoder.setFilmGrainCharactersticsSEIPersistenceFlag         ( m_fgcSEIPersistenceFlag );
  m_layerEncoder.setFilmGrainCharactersticsSEIModelID                 ( (uint8_t)m_fgcSEIModelID );
  m_layerEncoder.setFilmGrainCharactersticsSEISepColourDescPresent    ( m_fgcSEISepColourDescPresentFlag );
  m_layerEncoder.setFilmGrainCharactersticsSEIBlendingModeID          ( (uint8_t) m_fgcSEIBlendingModeID );
  m_layerEncoder.setFilmGrainCharactersticsSEILog2ScaleFactor         ( (uint8_t) m_fgcSEILog2ScaleFactor );
  for (int i = 0; i < MAX_NUM_COMPONENT; i++)
  {
    m_layerEncoder.setFGCSEICompModelPresent                            ( m_fgcSEICompModelPresent[i], i );
  }
  // content light level
  m_layerEncoder.setCLLSEIEnabled                                     ( m_cllSEIEnabled);
  m_layerEncoder.setCLLSEIMaxContentLightLevel                        ( (uint16_t) m_cllSEIMaxContentLevel );
  m_layerEncoder.setCLLSEIMaxPicAvgLightLevel                         ( (uint16_t) m_cllSEIMaxPicAvgLevel );
  // ambient viewing enviornment
  m_layerEncoder.setAmbientViewingEnvironmentSEIEnabled               ( m_aveSEIEnabled );
  m_layerEncoder.setAmbientViewingEnvironmentSEIIlluminance           ( m_aveSEIAmbientIlluminance );
  m_layerEncoder.setAmbientViewingEnvironmentSEIAmbientLightX         ( (uint16_t) m_aveSEIAmbientLightX );
  m_layerEncoder.setAmbientViewingEnvironmentSEIAmbientLightY         ( (uint16_t) m_aveSEIAmbientLightY );
  // content colour volume SEI
  m_layerEncoder.setCcvSEIEnabled                                     ( m_ccvSEIEnabled );
  m_layerEncoder.setCcvSEICancelFlag                                  ( m_ccvSEICancelFlag );
  m_layerEncoder.setCcvSEIPersistenceFlag                             ( m_ccvSEIPersistenceFlag );
  m_layerEncoder.setCcvSEIEnabled                                     ( m_ccvSEIEnabled );
  m_layerEncoder.setCcvSEICancelFlag                                  ( m_ccvSEICancelFlag );
  m_layerEncoder.setCcvSEIPersistenceFlag                             ( m_ccvSEIPersistenceFlag );
  m_layerEncoder.setCcvSEIPrimariesPresentFlag                        ( m_ccvSEIPrimariesPresentFlag );
  m_layerEncoder.setCcvSEIMinLuminanceValuePresentFlag                ( m_ccvSEIMinLuminanceValuePresentFlag );
  m_layerEncoder.setCcvSEIMaxLuminanceValuePresentFlag                ( m_ccvSEIMaxLuminanceValuePresentFlag );
  m_layerEncoder.setCcvSEIAvgLuminanceValuePresentFlag                ( m_ccvSEIAvgLuminanceValuePresentFlag );
  for(int i = 0; i < MAX_NUM_COMPONENT; i++) {
    m_layerEncoder.setCcvSEIPrimariesX                                  ( m_ccvSEIPrimariesX[i], i );
    m_layerEncoder.setCcvSEIPrimariesY                                  ( m_ccvSEIPrimariesY[i], i );
  }
  m_layerEncoder.setCcvSEIMinLuminanceValue                           ( m_ccvSEIMinLuminanceValue );
  m_layerEncoder.setCcvSEIMaxLuminanceValue                           ( m_ccvSEIMaxLuminanceValue );
  m_layerEncoder.setCcvSEIAvgLuminanceValue                           ( m_ccvSEIAvgLuminanceValue );
  m_layerEncoder.setEntropyCodingSyncEnabledFlag                      ( m_entropyCodingSyncEnabledFlag );
  m_layerEncoder.setEntryPointPresentFlag                             ( m_entryPointPresentFlag );
  m_layerEncoder.setTMVPModeId                                        ( m_TMVPModeId );
  m_layerEncoder.setSliceLevelRpl                                     ( m_sliceLevelRpl  );
  m_layerEncoder.setSliceLevelDblk                                    ( m_sliceLevelDblk );
  m_layerEncoder.setSliceLevelSao                                     ( m_sliceLevelSao  );
  m_layerEncoder.setSliceLevelWp                                      ( m_sliceLevelWp );
  m_layerEncoder.setSliceLevelDeltaQp                                 ( m_sliceLevelDeltaQp );
  m_layerEncoder.setSliceLevelAlf                                     ( m_sliceLevelAlf  );
  m_layerEncoder.setUseScalingListId                                  ( m_useScalingListId  );
  m_layerEncoder.setScalingListFileName                               ( m_scalingListFileName );
  m_layerEncoder.setDisableScalingMatrixForLfnstBlks                  ( m_disableScalingMatrixForLfnstBlks );
  if ( m_layerEncoder.getUseColorTrans() && m_layerEncoder.getUseScalingListId() )
  {
    m_layerEncoder.setDisableScalingMatrixForAlternativeColourSpace     ( m_disableScalingMatrixForAlternativeColourSpace );
  }
  if ( m_layerEncoder.getDisableScalingMatrixForAlternativeColourSpace() )
  {
    m_layerEncoder.setScalingMatrixDesignatedColourSpace                ( m_scalingMatrixDesignatedColourSpace );
  }
  m_layerEncoder.setDepQuantEnabledFlag                               ( m_depQuantEnabledFlag);
  m_layerEncoder.setSignDataHidingEnabledFlag                         ( m_signDataHidingEnabledFlag);
  m_layerEncoder.setUseRateCtrl                                       ( m_RCEnableRateControl );
  m_layerEncoder.setTargetBitrate                                     ( m_RCTargetBitrate );
  m_layerEncoder.setKeepHierBit                                       ( m_RCKeepHierarchicalBit );
  m_layerEncoder.setLCULevelRC                                        ( m_RCLCULevelRC );
  m_layerEncoder.setUseLCUSeparateModel                               ( m_RCUseLCUSeparateModel );
  m_layerEncoder.setInitialQP                                         ( m_RCInitialQP );
  m_layerEncoder.setForceIntraQP                                      ( m_RCForceIntraQP );
#if U0132_TARGET_BITS_SATURATION
  m_layerEncoder.setCpbSaturationEnabled                              ( m_RCCpbSaturationEnabled );
  m_layerEncoder.setCpbSize                                           ( m_RCCpbSize );
  m_layerEncoder.setInitialCpbFullness                                ( m_RCInitialCpbFullness );
#endif
  m_layerEncoder.setCostMode                                          ( m_costMode );
  m_layerEncoder.setTSRCdisableLL                                     ( m_TSRCdisableLL );
  m_layerEncoder.setUseRecalculateQPAccordingToLambda                 ( m_recalculateQPAccordingToLambda );
  m_layerEncoder.setDCIEnabled                                        ( m_DCIEnabled );
  m_layerEncoder.setVuiParametersPresentFlag                          ( m_vuiParametersPresentFlag );
  m_layerEncoder.setSamePicTimingInAllOLS                             ( m_samePicTimingInAllOLS );
  m_layerEncoder.setAspectRatioInfoPresentFlag                        ( m_aspectRatioInfoPresentFlag);
  m_layerEncoder.setAspectRatioIdc                                    ( m_aspectRatioIdc );
  m_layerEncoder.setSarWidth                                          ( m_sarWidth );
  m_layerEncoder.setSarHeight                                         ( m_sarHeight );
  m_layerEncoder.setColourDescriptionPresentFlag                      ( m_colourDescriptionPresentFlag );
  m_layerEncoder.setColourPrimaries                                   ( m_colourPrimaries );
  m_layerEncoder.setTransferCharacteristics                           ( m_transferCharacteristics );
  m_layerEncoder.setMatrixCoefficients                                ( m_matrixCoefficients );
  m_layerEncoder.setProgressiveSourceFlag                             ( m_progressiveSourceFlag);
  m_layerEncoder.setInterlacedSourceFlag                              ( m_interlacedSourceFlag);
  m_layerEncoder.setChromaLocInfoPresentFlag                          ( m_chromaLocInfoPresentFlag );
  m_layerEncoder.setChromaSampleLocTypeTopField                       ( m_chromaSampleLocTypeTopField );
  m_layerEncoder.setChromaSampleLocTypeBottomField                    ( m_chromaSampleLocTypeBottomField );
  m_layerEncoder.setChromaSampleLocType                               ( m_chromaSampleLocType );
  m_layerEncoder.setOverscanInfoPresentFlag                           ( m_overscanInfoPresentFlag );
  m_layerEncoder.setOverscanAppropriateFlag                           ( m_overscanAppropriateFlag );
  m_layerEncoder.setVideoFullRangeFlag                                ( m_videoFullRangeFlag );
  m_layerEncoder.setEfficientFieldIRAPEnabled                         ( m_bEfficientFieldIRAPEnabled );
  m_layerEncoder.setHarmonizeGopFirstFieldCoupleEnabled               ( m_bHarmonizeGopFirstFieldCoupleEnabled );
  m_layerEncoder.setSummaryOutFilename                                ( m_summaryOutFilename );
  m_layerEncoder.setSummaryPicFilenameBase                            ( m_summaryPicFilenameBase );
  m_layerEncoder.setSummaryVerboseness                                ( m_summaryVerboseness );
  m_layerEncoder.setIMV                                               ( m_ImvMode );
  m_layerEncoder.setIMV4PelFast                                       ( m_Imv4PelFast );
  m_layerEncoder.setDecodeBitstream                                   ( 0, m_decodeBitstreams[0] );
  m_layerEncoder.setDecodeBitstream                                   ( 1, m_decodeBitstreams[1] );
  m_layerEncoder.setSwitchPOC                                         ( m_switchPOC );
  m_layerEncoder.setSwitchDQP                                         ( m_switchDQP );
  m_layerEncoder.setFastForwardToPOC                                  ( m_fastForwardToPOC );
  m_layerEncoder.setForceDecodeBitstream1                             ( m_forceDecodeBitstream1 );
  m_layerEncoder.setStopAfterFFtoPOC                                  ( m_stopAfterFFtoPOC );
  m_layerEncoder.setBs2ModPOCAndType                                  ( m_bs2ModPOCAndType );
  m_layerEncoder.setDebugCTU                                          ( m_debugCTU );
#if ENABLE_SPLIT_PARALLELISM
  m_layerEncoder.setNumSplitThreads                                   ( m_numSplitThreads );
  m_layerEncoder.setForceSingleSplitThread                            ( m_forceSplitSequential );
#endif
  m_layerEncoder.setUseALF                                            ( m_alf );
  m_layerEncoder.setUseCCALF                                          ( m_ccalf );
  m_layerEncoder.setCCALFQpThreshold                                  ( m_ccalfQpThreshold );
  m_layerEncoder.setLmcs                                              ( m_lmcsEnabled );
  m_layerEncoder.setReshapeSignalType                                 ( m_reshapeSignalType );
  m_layerEncoder.setReshapeIntraCMD                                   ( m_intraCMD );
  m_layerEncoder.setReshapeCW                                         ( m_reshapeCW );
  m_layerEncoder.setReshapeCSoffset                                   ( m_CSoffset );

#if JVET_O0756_CALCULATE_HDRMETRICS
  for (int i=0; i<hdrtoolslib::NB_REF_WHITE; i++)
  {
    m_layerEncoder.setWhitePointDeltaE                                ( i, m_whitePointDeltaE[i] );
  }
  m_layerEncoder.setMaxSampleValue                                    ( m_maxSampleValue );
  m_layerEncoder.setSampleRange                                       ( m_sampleRange );
  m_layerEncoder.setColorPrimaries                                    ( m_colorPrimaries );
  m_layerEncoder.setEnableTFunctionLUT                                ( m_enableTFunctionLUT );
  for (int i=0; i<2; i++)
  {
    m_layerEncoder.setChromaLocation                                    ( i, m_chromaLocation );
    m_layerEncoder.setChromaUPFilter                                    ( m_chromaUPFilter );
  }
  m_layerEncoder.setCropOffsetLeft                                    ( m_cropOffsetLeft );
  m_layerEncoder.setCropOffsetTop                                     ( m_cropOffsetTop );
  m_layerEncoder.setCropOffsetRight                                   ( m_cropOffsetRight );
  m_layerEncoder.setCropOffsetBottom                                  ( m_cropOffsetBottom );
  m_layerEncoder.setCalculateHdrMetrics                               ( m_calculateHdrMetrics );
#endif
#if JVET_S0163_ON_TARGETOLS_SUBLAYERS
  m_layerEncoder.setOPIEnabled                                        ( m_OPIEnabled );
  if (m_OPIEnabled)
  {
    if (m_maxTemporalLayer != 500)
    {
      m_layerEncoder.setHtidPlus1                                         ( m_maxTemporalLayer + 1);
    }
    if (m_targetOlsIdx != 500)
    {
      m_layerEncoder.setTargetOlsIdx                                      ( m_targetOlsIdx );
    }
  }
#endif
  m_layerEncoder.setGopBasedTemporalFilterEnabled(m_gopBasedTemporalFilterEnabled);
  m_layerEncoder.setNumRefLayers                                      ( m_numRefLayers );

  m_layerEncoder.setVPSParameters(m_cfgVPSParameters);
}

void AppLayerEncoder::xCreateLib( std::list<PelUnitBuf*>& recBufList, const int layerId )
{
  // Video I/O
  m_cVideoIOYuvInputFile.open( m_inputFileName,     false, m_inputBitDepth, m_MSBExtendedBitDepth, m_internalBitDepth );  // read  mode
#if EXTENSION_360_VIDEO
  m_cVideoIOYuvInputFile.skipFrames(m_FrameSkip, m_inputFileWidth, m_inputFileHeight, m_InputChromaFormatIDC);
#else
  const int sourceHeight = m_isField ? m_iSourceHeightOrg : m_iSourceHeight;
  m_cVideoIOYuvInputFile.skipFrames(m_FrameSkip, m_iSourceWidth - m_aiPad[0], sourceHeight - m_aiPad[1], m_InputChromaFormatIDC);
#endif
  if (!m_reconFileName.empty())
  {
    if (m_packedYUVMode && ((m_outputBitDepth[CH_L] != 10 && m_outputBitDepth[CH_L] != 12)
        || ((m_iSourceWidth & (1 + (m_outputBitDepth[CH_L] & 3))) != 0)))
    {
      EXIT ("Invalid output bit-depth or image width for packed YUV output, aborting\n");
    }
    if (m_packedYUVMode && (m_chromaFormatIDC != CHROMA_400) && ((m_outputBitDepth[CH_C] != 10 && m_outputBitDepth[CH_C] != 12)
        || (((m_iSourceWidth / SPS::getWinUnitX (m_chromaFormatIDC)) & (1 + (m_outputBitDepth[CH_C] & 3))) != 0)))
    {
      EXIT ("Invalid chroma output bit-depth or image width for packed YUV output, aborting\n");
    }

    std::string reconFileName = m_reconFileName;
    if( m_reconFileName.compare( "/dev/null" ) &&  (m_maxLayers > 1) )
    {
      size_t pos = reconFileName.find_last_of('.');
      if (pos != string::npos)
      {
        reconFileName.insert( pos, std::to_string( layerId ) );
      }
      else
      {
        reconFileName.append( std::to_string( layerId ) );
      }
    }
    m_cVideoIOYuvReconFile.open( reconFileName, true, m_outputBitDepth, m_outputBitDepth, m_internalBitDepth );  // write mode
  }

  // create the encoder
  m_layerEncoder.create( layerId );

  // create the output buffer
  for( int i = 0; i < (m_iGOPSize + 1 + (m_isField ? 1 : 0)); i++ )
  {
    recBufList.push_back( new PelUnitBuf );
  }
}

void AppLayerEncoder::xDestroyLib()
{
  // Video I/O
  m_cVideoIOYuvInputFile.close();
  m_cVideoIOYuvReconFile.close();

  // Neo Decoder
  m_layerEncoder.destroy();
}

void AppLayerEncoder::xInitLib(bool isFieldCoding)
{
  m_layerEncoder.init(isFieldCoding, this );
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

void AppLayerEncoder::createLib( const int layerIdx )
{
  const int sourceHeight = m_isField ? m_iSourceHeightOrg : m_iSourceHeight;
  UnitArea unitArea( m_chromaFormatIDC, Area( 0, 0, m_iSourceWidth, sourceHeight ) );

  m_orgPic = new PelStorage;
  m_trueOrgPic = new PelStorage;
  m_orgPic->create( unitArea );
  m_trueOrgPic->create( unitArea );
  if(m_gopBasedTemporalFilterEnabled)
  {
    m_filteredOrgPic = new PelStorage;
    m_filteredOrgPic->create( unitArea );
  }

  if( !m_bitstream.is_open() )
  {
    m_bitstream.open( m_bitstreamFileName.c_str(), fstream::binary | fstream::out );
    if( !m_bitstream )
    {
      EXIT( "Failed to open bitstream file " << m_bitstreamFileName.c_str() << " for writing\n" );
    }
  }

  // initialize internal class & member variables and VPS
  xInitLibCfg();
  const int layerId = m_layerEncoder.getVPS() == nullptr ? 0 : m_layerEncoder.getVPS()->getLayerId( layerIdx );
  xCreateLib( m_recBufList, layerId );
  xInitLib( m_isField );

  printChromaFormat();

#if EXTENSION_360_VIDEO
  m_ext360 = new TExt360AppEncTop( *this, m_layerEncoder.getGOPEncoder()->getExt360Data(), *( m_layerEncoder.getGOPEncoder() ), *m_orgPic );
#endif

  if( m_gopBasedTemporalFilterEnabled )
  {
    m_temporalFilter.init( m_FrameSkip, m_inputBitDepth, m_MSBExtendedBitDepth, m_internalBitDepth, m_iSourceWidth, m_iSourceHeight,
      m_aiPad, m_bClipInputVideoToRec709Range, m_inputFileName, m_chromaFormatIDC,
      m_inputColourSpaceConvert, m_iQP, m_gopBasedTemporalFilterStrengths,
      m_gopBasedTemporalFilterFutureReference );
  }
}

void AppLayerEncoder::destroyLib()
{
  printf( "\nLayerId %2d", m_layerEncoder.getLayerId() );

  m_layerEncoder.printSummary( m_isField );

  // delete used buffers in encoder class
  m_layerEncoder.deletePicBuffer();

  for( auto &p : m_recBufList )
  {
    delete p;
  }
  m_recBufList.clear();

  xDestroyLib();

  if( m_bitstream.is_open() )
  {
    m_bitstream.close();
  }

  m_orgPic->destroy();
  m_trueOrgPic->destroy();
  delete m_trueOrgPic;
  delete m_orgPic;
  if(m_gopBasedTemporalFilterEnabled)
  {
    m_filteredOrgPic->destroy();
    delete m_filteredOrgPic;
  }
#if EXTENSION_360_VIDEO
  delete m_ext360;
#endif

  printRateSummary();
}

bool AppLayerEncoder::encodePrep( bool& eos )
{
  // main encoder loop
  const InputColourSpaceConversion ipCSC = m_inputColourSpaceConvert;
  const InputColourSpaceConversion snrCSC = ( !m_snrInternalColourSpace ) ? m_inputColourSpaceConvert : IPCOLOURSPACE_UNCHANGED;

  // read input YUV file
#if EXTENSION_360_VIDEO
  if( m_ext360->isEnabled() )
  {
    m_ext360->read( m_cVideoIOYuvInputFile, *m_orgPic, *m_trueOrgPic, ipCSC );
  }
  else
  {
    m_cVideoIOYuvInputFile.read( *m_orgPic, *m_trueOrgPic, ipCSC, m_aiPad, m_InputChromaFormatIDC, m_bClipInputVideoToRec709Range );
  }
#else
  m_cVideoIOYuvInputFile.read( *m_orgPic, *m_trueOrgPic, ipCSC, m_aiPad, m_InputChromaFormatIDC, m_bClipInputVideoToRec709Range );
#endif

  if( m_gopBasedTemporalFilterEnabled )
  {
    m_temporalFilter.filter( m_orgPic, m_numFramesReceived );
    m_filteredOrgPic->copyFrom(*m_orgPic);
  }

  // increase number of received frames
  m_numFramesReceived++;

  eos = ( m_isField && ( m_numFramesReceived == ( m_framesToBeEncoded >> 1 ) ) ) || ( !m_isField && ( m_numFramesReceived == m_framesToBeEncoded ) );

  // if end of file (which is only detected on a read failure) flush the encoder of any queued pictures
  if( m_cVideoIOYuvInputFile.isEof() )
  {
    m_flush = true;
    eos = true;
    m_numFramesReceived--;
    m_layerEncoder.setFramesToBeEncoded( m_numFramesReceived );
  }

  bool keepDoing = false;

  // call encoding function for one frame
  if( m_isField )
  {
    keepDoing = m_layerEncoder.encodePrep( eos, m_flush ? 0 : m_orgPic, m_flush ? 0 : m_trueOrgPic, m_flush ? 0 : m_filteredOrgPic, snrCSC, m_recBufList, m_numEncoded, m_isTopFieldFirst );
  }
  else
  {
    keepDoing = m_layerEncoder.encodePrep( eos, m_flush ? 0 : m_orgPic, m_flush ? 0 : m_trueOrgPic, m_flush ? 0 : m_filteredOrgPic, snrCSC, m_recBufList, m_numEncoded );
  }

  return keepDoing;
}

bool AppLayerEncoder::encode()
{
  const InputColourSpaceConversion snrCSC = ( !m_snrInternalColourSpace ) ? m_inputColourSpaceConvert : IPCOLOURSPACE_UNCHANGED;
  bool keepDoing = false;

  // call encoding function for one frame
  if( m_isField )
  {
    keepDoing = m_layerEncoder.encode( snrCSC, m_recBufList, m_numEncoded, m_isTopFieldFirst );
  }
  else
  {
    keepDoing = m_layerEncoder.encode( snrCSC, m_recBufList, m_numEncoded );
  }

#if JVET_O0756_CALCULATE_HDRMETRICS
    m_metricTime = m_layerEncoder.getMetricTime();
#endif

  // output when the entire GOP was proccessed
  if( !keepDoing )
  {
    // write bistream to file if necessary
    if( m_numEncoded > 0 )
    {
      xWriteOutput( m_numEncoded, m_recBufList );
    }
    // temporally skip frames
    if( m_temporalSubsampleRatio > 1 )
    {
#if EXTENSION_360_VIDEO
      m_cVideoIOYuvInputFile.skipFrames( m_temporalSubsampleRatio - 1, m_inputFileWidth, m_inputFileHeight, m_InputChromaFormatIDC );
#else
    const int sourceHeight = m_isField ? m_iSourceHeightOrg : m_iSourceHeight;
    m_cVideoIOYuvInputFile.skipFrames( m_temporalSubsampleRatio - 1, m_iSourceWidth - m_aiPad[0], sourceHeight - m_aiPad[1], m_InputChromaFormatIDC );
#endif
    }
  }

  return keepDoing;
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

/**
  Write access units to output file.
  \param bitstreamFile  target bitstream file
  \param iNumEncoded    number of encoded frames
  \param accessUnits    list of access units to be written
 */
void AppLayerEncoder::xWriteOutput( int iNumEncoded, std::list<PelUnitBuf*>& recBufList )
{
  const InputColourSpaceConversion ipCSC = (!m_outputInternalColourSpace) ? m_inputColourSpaceConvert : IPCOLOURSPACE_UNCHANGED;
  std::list<PelUnitBuf*>::iterator iterPicYuvRec = recBufList.end();
  int i;

  for ( i = 0; i < iNumEncoded; i++ )
  {
    --iterPicYuvRec;
  }

  if (m_isField)
  {
    //Reinterlace fields
    for ( i = 0; i < iNumEncoded/2; i++ )
    {
      const PelUnitBuf*  pcPicYuvRecTop     = *(iterPicYuvRec++);
      const PelUnitBuf*  pcPicYuvRecBottom  = *(iterPicYuvRec++);

      if (!m_reconFileName.empty())
      {
        m_cVideoIOYuvReconFile.write( *pcPicYuvRecTop, *pcPicYuvRecBottom,
                                      ipCSC,
                                      false, // TODO: m_packedYUVMode,
                                      m_confWinLeft, m_confWinRight, m_confWinTop, m_confWinBottom, NUM_CHROMA_FORMAT, m_isTopFieldFirst );
      }
    }
  }
  else
  {
    for ( i = 0; i < iNumEncoded; i++ )
    {
      const PelUnitBuf* pcPicYuvRec = *(iterPicYuvRec++);
      if (!m_reconFileName.empty())
      {
        if( m_layerEncoder.isResChangeInClvsEnabled() && m_layerEncoder.getUpscaledOutput() )
        {
          const SPS& sps = *m_layerEncoder.getSPS( 0 );
          const PPS& pps = *m_layerEncoder.getPPS( ( sps.getMaxPicWidthInLumaSamples() != pcPicYuvRec->get( COMPONENT_Y ).width || sps.getMaxPicHeightInLumaSamples() != pcPicYuvRec->get( COMPONENT_Y ).height ) ? ENC_PPS_ID_RPR : 0 );

          m_cVideoIOYuvReconFile.writeUpscaledPicture( sps, pps, *pcPicYuvRec, ipCSC, m_packedYUVMode, m_layerEncoder.getUpscaledOutput(), NUM_CHROMA_FORMAT, m_bClipOutputVideoToRec709Range );
        }
        else
        {
          m_cVideoIOYuvReconFile.write( pcPicYuvRec->get( COMPONENT_Y ).width, pcPicYuvRec->get( COMPONENT_Y ).height, *pcPicYuvRec, ipCSC, m_packedYUVMode,
            m_confWinLeft, m_confWinRight, m_confWinTop, m_confWinBottom, NUM_CHROMA_FORMAT, m_bClipOutputVideoToRec709Range );
        }
      }
    }
  }
}


void AppLayerEncoder::outputAU( const AccessUnit& au )
{
  const vector<uint32_t>& stats = writeAnnexBAccessUnit(m_bitstream, au);
  rateStatsAccum(au, stats);
  m_bitstream.flush();
}


/**
 *
 */
void AppLayerEncoder::rateStatsAccum(const AccessUnit& au, const std::vector<uint32_t>& annexBsizes)
{
  AccessUnit::const_iterator it_au = au.begin();
  vector<uint32_t>::const_iterator it_stats = annexBsizes.begin();

  for (; it_au != au.end(); it_au++, it_stats++)
  {
    switch ((*it_au)->m_nalUnitType)
    {
    case NAL_UNIT_CODED_SLICE_TRAIL:
    case NAL_UNIT_CODED_SLICE_STSA:
    case NAL_UNIT_CODED_SLICE_IDR_W_RADL:
    case NAL_UNIT_CODED_SLICE_IDR_N_LP:
    case NAL_UNIT_CODED_SLICE_CRA:
    case NAL_UNIT_CODED_SLICE_GDR:
    case NAL_UNIT_CODED_SLICE_RADL:
    case NAL_UNIT_CODED_SLICE_RASL:
#if JVET_S0163_ON_TARGETOLS_SUBLAYERS
    case NAL_UNIT_OPI:
#endif
    case NAL_UNIT_DCI:
    case NAL_UNIT_VPS:
    case NAL_UNIT_SPS:
    case NAL_UNIT_PPS:
    case NAL_UNIT_PH:
    case NAL_UNIT_PREFIX_APS:
    case NAL_UNIT_SUFFIX_APS:
      m_essentialBytes += *it_stats;
      break;
    default:
      break;
    }

    m_totalBytes += *it_stats;
  }
}

void AppLayerEncoder::printRateSummary()
{
  double time = (double) m_numFramesReceived / m_iFrameRate * m_temporalSubsampleRatio;
  msg( DETAILS,"Bytes written to file: %u (%.3f kbps)\n", m_totalBytes, 0.008 * m_totalBytes / time );
  if (m_summaryVerboseness > 0)
  {
    msg(DETAILS, "Bytes for SPS/PPS/APS/Slice (Incl. Annex B): %u (%.3f kbps)\n", m_essentialBytes, 0.008 * m_essentialBytes / time);
  }
}

void AppLayerEncoder::printChromaFormat()
{
  if( g_verbosity >= DETAILS )
  {
    std::cout << std::setw(43) << "Input ChromaFormatIDC = ";
    switch (m_InputChromaFormatIDC)
    {
    case CHROMA_400:  std::cout << "  4:0:0"; break;
    case CHROMA_420:  std::cout << "  4:2:0"; break;
    case CHROMA_422:  std::cout << "  4:2:2"; break;
    case CHROMA_444:  std::cout << "  4:4:4"; break;
    default:
      THROW( "invalid chroma fomat");
    }
    std::cout << std::endl;

    std::cout << std::setw(43) << "Output (internal) ChromaFormatIDC = ";
    switch (m_layerEncoder.getChromaFormatIdc())
    {
    case CHROMA_400:  std::cout << "  4:0:0"; break;
    case CHROMA_420:  std::cout << "  4:2:0"; break;
    case CHROMA_422:  std::cout << "  4:2:2"; break;
    case CHROMA_444:  std::cout << "  4:4:4"; break;
    default:
      THROW( "invalid chroma fomat");
    }
    std::cout << "\n" << std::endl;
  }
}

//! \}
