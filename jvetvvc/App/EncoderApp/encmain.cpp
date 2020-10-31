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

/** \file     encmain.cpp
    \brief    Encoder application main
*/

#include <time.h>
#include <iostream>
#include <chrono>
#include <ctime>

#include "EncoderLib/EncoderSharedComponents.h"
#include "AppLayerEncoder.h"
#include "Utilities/program_options_lite.h"

//! \ingroup EncoderApp
//! \{

static const uint32_t settingNameWidth = 66;
static const uint32_t settingHelpWidth = 84;
static const uint32_t settingValueWidth = 3;
// --------------------------------------------------------------------------------------------------------------------- //

//macro value printing function

#define PRINT_CONSTANT(NAME, NAME_WIDTH, VALUE_WIDTH) std::cout << std::setw(NAME_WIDTH) << #NAME << " = " << std::setw(VALUE_WIDTH) << NAME << std::endl;

static void printMacroSettings()
{
  if( g_verbosity >= DETAILS )
  {
    std::cout << "Non-environment-variable-controlled macros set as follows: \n" << std::endl;

    //------------------------------------------------

    //setting macros

    PRINT_CONSTANT( RExt__DECODER_DEBUG_BIT_STATISTICS,                         settingNameWidth, settingValueWidth );
    PRINT_CONSTANT( RExt__HIGH_BIT_DEPTH_SUPPORT,                               settingNameWidth, settingValueWidth );
    PRINT_CONSTANT( RExt__HIGH_PRECISION_FORWARD_TRANSFORM,                     settingNameWidth, settingValueWidth );
    PRINT_CONSTANT( ME_ENABLE_ROUNDING_OF_MVS,                                  settingNameWidth, settingValueWidth );

    //------------------------------------------------

    std::cout << std::endl;
  }
}

// ====================================================================================================================
// Main function
// ====================================================================================================================

int main(int argc, char* argv[])
{
  // print information
  fprintf( stdout, "\n" );
  fprintf( stdout, "VVCSoftware: VTM Encoder Version %s ", VTM_VERSION );
  fprintf( stdout, NVM_ONOS );
  fprintf( stdout, NVM_COMPILEDBY );
  fprintf( stdout, NVM_BITS );
#if defined(__AVX512F__) || (__AVX512DQ__) || (__AVX512BW__)
  fprintf( stdout, "[SIMD=AVX512]" );
#elif defined(__AVX2__)
  fprintf( stdout, "[SIMD=AVX2]" );
#elif defined(__AVX__)
  fprintf( stdout, "[SIMD=AVX]" );
#elif defined(__SSE4_2__)
  fprintf( stdout, "[SIMD=SSE4.2]" );
#elif defined(__SSE4_1__)
  fprintf( stdout, "[SIMD=SSE4.1]" );
#else
  fprintf( stdout, "[SIMD=NONE]" );
#endif
/*#if ENABLE_SIMD_OPT
  std::string SIMD;
  df::program_options_lite::Options opts;
  opts.addOptions()
    ( "SIMD", SIMD, string( "" ), "" )
    ( "c", df::program_options_lite::parseConfigFile, "" );
  df::program_options_lite::SilentReporter err;
  df::program_options_lite::scanArgv( opts, argc, ( const char** ) argv, err );
  fprintf( stdout, "[SIMD=%s] ", read_x86_extension( SIMD ) );
#endif*/
#if ENABLE_TRACING
  fprintf( stdout, "[ENABLE_TRACING] " );
#endif
#if EXTENSION_360_VIDEO
  fprintf( stdout, "\nVVCSoftware: 360Lib Soft Version %s", VERSION_360Lib );
#endif
#if EXTENSION_HDRTOOLS
  fprintf( stdout, "\nVVCSoftware: HDRTools Version %s", VERSION );
#endif
#if ENABLE_SPLIT_PARALLELISM
#include "../../../libgomp/gomp-constants.h"
#include "../../../libgomp/pthread_win32/_ptw32.h"
  fprintf( stdout, "\nVVCSoftware: libgomp / pthreads 32bit  : %d.0 / %d.%d ", GOMP_VERSION, __PTW32_VERSION_MAJOR, __PTW32_VERSION_MINOR );
  fprintf( stdout, "[SPLIT_PARALLEL (%d jobs)]", PARL_SPLIT_MAX_NUM_JOBS );
  const char* waitPolicy = getenv( "OMP_WAIT_POLICY" );
  const char* maxThLim   = getenv( "OMP_THREAD_LIMIT" );
  fprintf( stdout, waitPolicy ? "[OMP: WAIT_POLICY=%s," : "[OMP: WAIT_POLICY=,", waitPolicy );
  fprintf( stdout, maxThLim   ? "THREAD_LIMIT=%s" : "THREAD_LIMIT=", maxThLim );
  fprintf( stdout, "]" );
#endif
  fprintf( stdout, "\n" );

  std::fstream bitstream;
  EncoderSharedComponents encSharedComponents;

  std::vector<AppLayerEncoder*> appLayerEncoder(1);
  bool resized = false;
  int layerIdx = 0;

  initROM();
  TComHash::initBlockSizeToIndex();

  char** layerArgv = new char*[argc];

  do
  {
    appLayerEncoder[layerIdx] = new AppLayerEncoder( bitstream, encSharedComponents );
    // create application encoder class per layer
    appLayerEncoder[layerIdx]->create();

    // parse configuration per layer
    try
    {
      int j = 0;
      for( int i = 0; i < argc; i++ )
      {
        if( argv[i][0] == '-' && argv[i][1] == 'l' )
        {
          if (argc <= i + 1)
          {
            THROW("Command line parsing error: missing parameter after -lx\n");
          }
          int numParams = 1; // count how many parameters are consumed
          // check for long parameters, which start with "--"
          const std::string param = argv[i + 1];
          if (param.rfind("--", 0) != 0)
          {
            // only short parameters have a second parameter for the value
            if (argc <= i + 2)
            {
              THROW("Command line parsing error: missing parameter after -lx\n");
            }
            numParams++;
          }
          // check if correct layer index
          if( argv[i][2] == std::to_string( layerIdx ).c_str()[0] )
          {
            layerArgv[j] = argv[i + 1];
            if (numParams > 1)
            {
              layerArgv[j + 1] = argv[i + 2];
            }
            j+= numParams;
          }
          i += numParams;
        }
        else
        {
          layerArgv[j] = argv[i];
          j++;
        }
      }

      if( !appLayerEncoder[layerIdx]->parseCfg( j, layerArgv ) )
      {
        appLayerEncoder[layerIdx]->destroy();
        return 1;
      }
    }
    catch( df::program_options_lite::ParseFailure &e )
    {
      std::cerr << "Error parsing option \"" << e.arg << "\" with argument \"" << e.val << "\"." << std::endl;
      return 1;
    }

    appLayerEncoder[layerIdx]->createLib( layerIdx );

    if( !resized )
    {
      appLayerEncoder.resize( appLayerEncoder[layerIdx]->getMaxLayers() );
      resized = true;
    }

    layerIdx++;
  } while( layerIdx < appLayerEncoder.size() );

  delete[] layerArgv;

  if (layerIdx > 1)
  {
    VPS* vps = appLayerEncoder[0]->getVPS();

    //check chroma format and bit-depth for dependent layers
    for (uint32_t i = 0; i < layerIdx; i++)
    {
      int curLayerChromaFormatIdc = appLayerEncoder[i]->getChromaFormatIDC();
      int curLayerBitDepth = appLayerEncoder[i]->getBitDepth();
      for (uint32_t j = 0; j < layerIdx; j++)
      {
        if (vps->getDirectRefLayerFlag(i, j))
        {
          int refLayerChromaFormatIdcInVPS = appLayerEncoder[j]->getChromaFormatIDC();
          CHECK(curLayerChromaFormatIdc != refLayerChromaFormatIdcInVPS, "The chroma formats of the current layer and the reference layer are different");
          int refLayerBitDepthInVPS = appLayerEncoder[j]->getBitDepth();
          CHECK(curLayerBitDepth != refLayerBitDepthInVPS, "The bit-depth of the current layer and the reference layer are different");
        }
      }
    }
  }

#if PRINT_MACRO_VALUES
  printMacroSettings();
#endif

  // starting time
  auto startTime  = std::chrono::steady_clock::now();
  std::time_t startTime2 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  fprintf(stdout, " started @ %s", std::ctime(&startTime2) );
  clock_t startClock = clock();

  // call encoding function per layer
  bool eos = false;

  while( !eos )
  {
    // read GOP
    bool keepLoop = true;
    while( keepLoop )
    {
      for( auto & encApp : appLayerEncoder )
      {
#ifndef _DEBUG
        try
        {
#endif
          keepLoop = encApp->encodePrep( eos );
#ifndef _DEBUG
        }
        catch( Exception &e )
        {
          std::cerr << e.what() << std::endl;
          return EXIT_FAILURE;
        }
        catch( const std::bad_alloc &e )
        {
          std::cout << "Memory allocation failed: " << e.what() << std::endl;
          return EXIT_FAILURE;
        }
#endif
      }
    }

    // encode GOP
    keepLoop = true;
    while( keepLoop )
    {
      for( auto & encApp : appLayerEncoder )
      {
#ifndef _DEBUG
        try
        {
#endif
          keepLoop = encApp->encode();
#ifndef _DEBUG
        }
        catch( Exception &e )
        {
          std::cerr << e.what() << std::endl;
          return EXIT_FAILURE;
        }
        catch( const std::bad_alloc &e )
        {
          std::cout << "Memory allocation failed: " << e.what() << std::endl;
          return EXIT_FAILURE;
        }
#endif
      }
    }
  }
  // ending time
  clock_t endClock = clock();
  auto endTime = std::chrono::steady_clock::now();
  std::time_t endTime2 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
#if JVET_O0756_CALCULATE_HDRMETRICS
  auto metricTime = appLayerEncoder[0]->getMetricTime();

  for( int layerIdx = 1; layerIdx < appLayerEncoder.size(); layerIdx++ )
  {
    metricTime += appLayerEncoder[layerIdx]->getMetricTime();
  }
  auto totalTime      = std::chrono::duration_cast<std::chrono::milliseconds>( endTime - startTime ).count();
  auto encTime        = std::chrono::duration_cast<std::chrono::milliseconds>( endTime - startTime - metricTime ).count();
  auto metricTimeuser = std::chrono::duration_cast<std::chrono::milliseconds>( metricTime ).count();
#else
  auto encTime = std::chrono::duration_cast<std::chrono::milliseconds>( endTime - startTime).count();
#endif

  for( auto & encApp : appLayerEncoder )
  {
    encApp->destroyLib();

    // destroy application encoder class per layer
    encApp->destroy();

    delete encApp;
  }

  // destroy ROM
  destroyROM();

  appLayerEncoder.clear();

  printf( "\n finished @ %s", std::ctime(&endTime2) );

#if JVET_O0756_CALCULATE_HDRMETRICS
  printf(" Encoding Time (Total Time): %12.3f ( %12.3f ) sec. [user] %12.3f ( %12.3f ) sec. [elapsed]\n",
         ((endClock - startClock) * 1.0 / CLOCKS_PER_SEC) - (metricTimeuser/1000.0),
         (endClock - startClock) * 1.0 / CLOCKS_PER_SEC,
         encTime / 1000.0,
         totalTime / 1000.0);
#else
  printf(" Total Time: %12.3f sec. [user] %12.3f sec. [elapsed]\n",
         (endClock - startClock) * 1.0 / CLOCKS_PER_SEC,
         encTime / 1000.0);
#endif

  return 0;
}

//! \}
