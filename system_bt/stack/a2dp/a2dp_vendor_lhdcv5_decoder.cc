/*
 * Copyright (C) 2016 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "a2dp_vendor_lhdcv5_decoder"

#include "a2dp_vendor_lhdcv5_decoder.h"

#include <dlfcn.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include <lhdcv5BT_dec.h>

#include "bt_common.h"
#include "common/time_util.h"
#include "osi/include/log.h"
#include "osi/include/osi.h"



#define A2DP_LHDC_FUNC_DISABLE		0
#define A2DP_LHDC_FUNC_ENABLE		1

#define LHDCV5_DEC_MAX_SAMPLES_PER_FRAME  256
#define LHDCV5_DEC_MAX_CHANNELS           2
#define LHDCV5_DEC_MAX_BIT_DEPTH          32
#define LHDCV5_DEC_FRAME_NUM              16
#define LHDCV5_DEC_BUF_BYTES              (LHDCV5_DEC_FRAME_NUM * LHDCV5_DEC_MAX_SAMPLES_PER_FRAME * LHDCV5_DEC_MAX_CHANNELS * (LHDCV5_DEC_MAX_BIT_DEPTH >> 3))
#define LHDCV5_DEC_PACKET_NUM             8

#define LHDCV5_DEC_INPUT_BUF_BYTES        1024

#define LHDCV5_DEC_PKT_HDR_BYTES          2

typedef struct {
  uint32_t    sample_rate;
  uint8_t     bits_per_sample;
  lhdc_ver_t  version;
  uint8_t     func_ar;
  uint8_t     func_jas;
  uint8_t     func_meta;

  uint8_t     decode_buf[LHDCV5_DEC_PACKET_NUM][LHDCV5_DEC_BUF_BYTES];
  uint32_t    dec_buf_idx;

  uint8_t     dec_input_buf[LHDCV5_DEC_INPUT_BUF_BYTES];
  uint32_t    dec_input_buf_bytes;

  decoded_data_callback_t decode_callback;
} tA2DP_LHDCV5_DECODER_CB;

static tA2DP_LHDCV5_DECODER_CB a2dp_lhdcv5_decoder_cb;


#define _V5DEC_REC_FILE_
#if defined(_V5DEC_REC_FILE_)
#define V5RAW_FILE_NAME "/sdcard/Download/lhdcv5dec.raw"
#define V5PCM_FILE_NAME "/sdcard/Download/v5decoded.pcm"
static FILE *rawFile = NULL;
static FILE *pcmFile = NULL;
#endif

//
// The LHDCV5 decoder shared library, and the functions to use
//
static const char* LHDC_DECODER_LIB_NAME = "liblhdcv5BT_dec.so";
static void* lhdc_decoder_lib_handle = NULL;

static const char* LHDCDEC_INIT_DECODER_NAME = "lhdcBT_dec_init_decoder";
typedef int (*tLHDCDEC_INIT_DECODER)(tLHDCV5_DEC_CONFIG *config);

static const char* LHDCDEC_CHECK_FRAME_DATA_ENOUGH_NAME = "lhdcBT_dec_check_frame_data_enough";
typedef int (*tLHDCDEC_CHECK_FRAME_DATA_ENOUGH)(const uint8_t *frameData, uint32_t frameBytes, uint32_t *packetBytes);

static const char* LHDCDEC_DECODE_NAME = "lhdcBT_dec_decode";
typedef int (*tLHDCDEC_DECODE)(const uint8_t *frameData, uint32_t frameBytes, uint8_t* pcmData, uint32_t* pcmBytes, uint32_t bits_depth);

static const char* LHDCDEC_DEINIT_DECODER_NAME = "lhdcBT_dec_deinit_decoder";
typedef int (*tLHDCDEC_DEINIT_DECODER)(void);

static tLHDCDEC_INIT_DECODER lhdcdec_init_decoder;
static tLHDCDEC_CHECK_FRAME_DATA_ENOUGH lhdcdec_check_frame_data_enough;
static tLHDCDEC_DECODE lhdcdec_decode;
static tLHDCDEC_DEINIT_DECODER lhdcdec_deinit_decoder;


//  ----------------------------------------------------------------
//  H0   |    H1     |    H2     |  P0-P3   | P4-P5   | P6[5:0]  |
//  losc | mediaType | codecType | vendorId | codecId | SampRate |
//  ----------------------------------------------------------------
//  P7[2:0]   | P7[5:4]    | P7[7:6]       | P8[3:0] | P8[4]       |
//  bit depth | MaxBitRate | MinBitRate    | Version | FrameLen5ms |
//  ----------------------------------------------------------------
//  P9[0] | P9[1]  | P9[2]   | P9[6] | P9[7]       | P10[0]      |
//  HasAR | HasJAS | HasMeta | HasLL | HasLossless | FeatureOnAR |
//  ----------------------------------------------------------------
#define A2DP_LHDCV5_CODEC_INFO_ATTR_1 (3+6)
#define A2DP_LHDCV5_CODEC_INFO_ATTR_2 (3+7)
#define A2DP_LHDCV5_CODEC_INFO_ATTR_3 (3+8)
#define A2DP_LHDCV5_CODEC_INFO_ATTR_4 (3+9)


bool a2dp_lhdcv5_decoder_save_codec_info (const uint8_t* p_codec_info)
{
  if (p_codec_info == NULL) {
    return false;
  }

  if (lhdc_decoder_lib_handle == NULL) {
    return false;
  }

  // Sampling Frequency
  if (p_codec_info[A2DP_LHDCV5_CODEC_INFO_ATTR_1] &
      A2DP_LHDCV5_SAMPLING_FREQ_44100) {
    a2dp_lhdcv5_decoder_cb.sample_rate = 44100;
  } else if (p_codec_info[A2DP_LHDCV5_CODEC_INFO_ATTR_1] &
      A2DP_LHDCV5_SAMPLING_FREQ_48000) {
    a2dp_lhdcv5_decoder_cb.sample_rate = 48000;
  } else if (p_codec_info[A2DP_LHDCV5_CODEC_INFO_ATTR_1] &
      A2DP_LHDCV5_SAMPLING_FREQ_96000) {
    a2dp_lhdcv5_decoder_cb.sample_rate = 96000;
  } else if (p_codec_info[A2DP_LHDCV5_CODEC_INFO_ATTR_1] &
      A2DP_LHDCV5_SAMPLING_FREQ_192000) {
    a2dp_lhdcv5_decoder_cb.sample_rate = 192000;
  } else {
    return false;
  }

  // Bit Depth
  if (p_codec_info[A2DP_LHDCV5_CODEC_INFO_ATTR_2] &
      A2DP_LHDCV5_BIT_FMT_16) {
    a2dp_lhdcv5_decoder_cb.bits_per_sample = 16;
  } else if (p_codec_info[A2DP_LHDCV5_CODEC_INFO_ATTR_2] &
      A2DP_LHDCV5_BIT_FMT_24) {
    a2dp_lhdcv5_decoder_cb.bits_per_sample = 24;
  } else if (p_codec_info[A2DP_LHDCV5_CODEC_INFO_ATTR_2] &
      A2DP_LHDCV5_BIT_FMT_32) {
    a2dp_lhdcv5_decoder_cb.bits_per_sample = 32;
  } else {
    return false;
  }

  // version
  if (p_codec_info[A2DP_LHDCV5_CODEC_INFO_ATTR_3] &
      A2DP_LHDCV5_VER_1) {
    a2dp_lhdcv5_decoder_cb.version = VERSION_5;
  } else {
    return false;
  }

  // AR
  if (p_codec_info[A2DP_LHDCV5_CODEC_INFO_ATTR_4] &
      A2DP_LHDCV5_FEATURE_AR) {
    a2dp_lhdcv5_decoder_cb.func_ar = A2DP_LHDC_FUNC_ENABLE;
  } else {
    a2dp_lhdcv5_decoder_cb.func_ar = A2DP_LHDC_FUNC_DISABLE;
  }

  // JAS
  if (p_codec_info[A2DP_LHDCV5_CODEC_INFO_ATTR_4] &
      A2DP_LHDCV5_FEATURE_JAS) {
    a2dp_lhdcv5_decoder_cb.func_jas = A2DP_LHDC_FUNC_ENABLE;
  } else {
    a2dp_lhdcv5_decoder_cb.func_jas = A2DP_LHDC_FUNC_DISABLE;
  }

  // META
  if (p_codec_info[A2DP_LHDCV5_CODEC_INFO_ATTR_4] &
      A2DP_LHDCV5_FEATURE_META) {
    a2dp_lhdcv5_decoder_cb.func_meta = A2DP_LHDC_FUNC_ENABLE;
  } else {
    a2dp_lhdcv5_decoder_cb.func_meta = A2DP_LHDC_FUNC_DISABLE;
  }

  return true;
}


static void* load_func(const char* func_name) {

  void* func_ptr = NULL;

  if ((func_name == NULL) ||
      (lhdc_decoder_lib_handle == NULL)) {
    LOG_ERROR( "%s: null ptr", __func__);
    return NULL;
  }

  func_ptr = dlsym(lhdc_decoder_lib_handle, func_name);

  if (func_ptr == NULL) {
    LOG_ERROR(
        "%s: cannot find function '%s' in the encoder library: %s",
        __func__, func_name, dlerror());
    A2DP_VendorUnloadDecoderLhdcV5();
    return NULL;
  }

  return func_ptr;
}


bool A2DP_VendorLoadDecoderLhdcV5(void) {

  if (lhdc_decoder_lib_handle != NULL) {
    return true;  // Already loaded
  }

  // Open the encoder library
  lhdc_decoder_lib_handle = dlopen(LHDC_DECODER_LIB_NAME, RTLD_NOW);
  if (lhdc_decoder_lib_handle == NULL) {
    LOG_ERROR( "%s: cannot open LHDCV5 decoder library %s", __func__, dlerror());
    return false;
  }

  // Load all functions
  lhdcdec_init_decoder = (tLHDCDEC_INIT_DECODER)load_func(LHDCDEC_INIT_DECODER_NAME);
  if (lhdcdec_init_decoder == NULL) return false;

  lhdcdec_check_frame_data_enough = (tLHDCDEC_CHECK_FRAME_DATA_ENOUGH)load_func(LHDCDEC_CHECK_FRAME_DATA_ENOUGH_NAME);
  if (lhdcdec_check_frame_data_enough == NULL) return false;

  lhdcdec_decode = (tLHDCDEC_DECODE)load_func(LHDCDEC_DECODE_NAME);
  if (lhdcdec_decode == NULL) return false;

  lhdcdec_deinit_decoder = (tLHDCDEC_DEINIT_DECODER)load_func(LHDCDEC_DEINIT_DECODER_NAME);
  if (lhdcdec_deinit_decoder == NULL) return false;

  LOG_DEBUG( "%s: LHDCV5 decoder library loaded", __func__);
  return true;
}


void A2DP_VendorUnloadDecoderLhdcV5(void) {
  a2dp_vendor_lhdcv5_decoder_cleanup();
  LOG_DEBUG( "%s: LHDCV5 decoder library unloaded", __func__);
}


bool a2dp_vendor_lhdcv5_decoder_init(decoded_data_callback_t decode_callback) {
  int  fn_ret;
  tLHDCV5_DEC_CONFIG lhdcdec_config;

  if ((lhdc_decoder_lib_handle == NULL) ||
      (lhdcdec_init_decoder == NULL) ||
      (lhdcdec_deinit_decoder == NULL)) {
    return false;
  }

  lhdcdec_deinit_decoder();

  lhdcdec_config.version = a2dp_lhdcv5_decoder_cb.version;
  lhdcdec_config.sample_rate = a2dp_lhdcv5_decoder_cb.sample_rate;
  lhdcdec_config.bits_depth = a2dp_lhdcv5_decoder_cb.bits_per_sample;

  fn_ret = lhdcdec_init_decoder(&lhdcdec_config);

  if (fn_ret != LHDCBT_DEC_FUNC_SUCCEED) {
    LOG_ERROR( "%s: LHDCV5 decoder init fail %d", __func__, fn_ret);
    return false;
  }

  a2dp_lhdcv5_decoder_cb.dec_buf_idx = 0;
  a2dp_lhdcv5_decoder_cb.dec_input_buf_bytes = 0;
  a2dp_lhdcv5_decoder_cb.decode_callback = decode_callback;

#if defined(_V5DEC_REC_FILE_)
  if (rawFile == NULL) {
    rawFile = fopen(V5RAW_FILE_NAME,"wb");
    LOG_DEBUG( "%s: Create recode file = %p", __func__, rawFile);
  }
  if (pcmFile == NULL) {
    pcmFile = fopen(V5PCM_FILE_NAME,"wb");
    LOG_DEBUG( "%s: Create recode file = %p", __func__, pcmFile);
  }
#endif
  return true;
}


void a2dp_vendor_lhdcv5_decoder_cleanup(void) {
  int  fn_ret;

  if (lhdc_decoder_lib_handle == NULL) {
    return;
  }

  if (lhdcdec_deinit_decoder != NULL) {
    fn_ret = lhdcdec_deinit_decoder();
    if (fn_ret != LHDCBT_DEC_FUNC_SUCCEED) {
      LOG_ERROR( "%s: LHDCV5 decoder deinit fail %d", __func__, fn_ret);
      return;
    }
  } else {
    LOG_ERROR( "%s: Cannot deinit LHDCV5 decoder", __func__);
    return;
  }

  dlclose(lhdc_decoder_lib_handle);
  lhdc_decoder_lib_handle = NULL;

#if defined(_V5DEC_REC_FILE_)
  if (rawFile != NULL) {
    fclose(rawFile);
    rawFile = NULL;
    remove(V5RAW_FILE_NAME);
  }
  if (pcmFile != NULL) {
    fclose(pcmFile);
    pcmFile = NULL;
    remove(V5PCM_FILE_NAME);
  }
#endif
  LOG_DEBUG( "%s:  LHDCV5 decoder deinited", __func__);
}


bool a2dp_vendor_lhdcv5_decoder_decode_packet(BT_HDR* p_buf) {
  int fn_ret;
  uint8_t *data;
  size_t data_size;
  uint32_t out_used = 0;
  uint32_t dec_buf_idx;
  uint8_t *ptr_src;
  uint8_t *ptr_dst;
  uint32_t packet_bytes;
  uint32_t i;

  if (p_buf == NULL) {
    return false;
  }

  data = p_buf->data + p_buf->offset;
  data_size = p_buf->len;

  dec_buf_idx = a2dp_lhdcv5_decoder_cb.dec_buf_idx++;
  if (a2dp_lhdcv5_decoder_cb.dec_buf_idx >= LHDCV5_DEC_PACKET_NUM) {
    a2dp_lhdcv5_decoder_cb.dec_buf_idx = 0;
  }

  if (data_size == 0) {
    LOG_ERROR( "%s: Empty packet", __func__);
    return false;
  }

  if ((lhdc_decoder_lib_handle == NULL) ||
      (lhdcdec_decode == NULL)) {
    LOG_ERROR( "%s: Invalid handle!", __func__);
    return false;
  }

#if defined(_V5DEC_REC_FILE_)
  if (rawFile != NULL && data_size > 0) {
    fwrite(data + LHDCV5_DEC_PKT_HDR_BYTES, sizeof(uint8_t),
        data_size - LHDCV5_DEC_PKT_HDR_BYTES, rawFile);
  }
#endif

  if ((a2dp_lhdcv5_decoder_cb.dec_input_buf_bytes + data_size) > LHDCV5_DEC_INPUT_BUF_BYTES)
  {
    // the data queued is useless
    // discard them
    a2dp_lhdcv5_decoder_cb.dec_input_buf_bytes = 0;

    if (data_size > LHDCV5_DEC_INPUT_BUF_BYTES)
    {
      // input data is too big (more than buffer size)!!
      // just ingore it, and do nothing
      return true;
    }
  }

  memcpy (&(a2dp_lhdcv5_decoder_cb.dec_input_buf[a2dp_lhdcv5_decoder_cb.dec_input_buf_bytes]),
      data,
      data_size);
  a2dp_lhdcv5_decoder_cb.dec_input_buf_bytes += data_size;

  packet_bytes = 0;
  fn_ret = lhdcdec_check_frame_data_enough(a2dp_lhdcv5_decoder_cb.dec_input_buf,
      a2dp_lhdcv5_decoder_cb.dec_input_buf_bytes,
      &packet_bytes);

  if (fn_ret == LHDCBT_DEC_FUNC_INPUT_NOT_ENOUGH) {
    return true;
  } else if (fn_ret != LHDCBT_DEC_FUNC_SUCCEED) {
    LOG_ERROR( "%s: fail to check frame data!", __func__);
    // clear the data in the input buffer
    a2dp_lhdcv5_decoder_cb.dec_input_buf_bytes = 0;
    return false;
  }

  if (packet_bytes != (a2dp_lhdcv5_decoder_cb.dec_input_buf_bytes - LHDCV5_DEC_PKT_HDR_BYTES)) {
    // strange!
    // queued data is NOT exactly equal to one packet!
    // maybe wrong data in buffer
    // discard data queued previously, and save input data
    LOG_ERROR( "%s: queued data is NOT exactly equal to one packet! packet (%d),  input (%d)",
        __func__, packet_bytes, a2dp_lhdcv5_decoder_cb.dec_input_buf_bytes);

    a2dp_lhdcv5_decoder_cb.dec_input_buf_bytes = 0;
    memcpy (&(a2dp_lhdcv5_decoder_cb.dec_input_buf[a2dp_lhdcv5_decoder_cb.dec_input_buf_bytes]),
        data,
        data_size);
    a2dp_lhdcv5_decoder_cb.dec_input_buf_bytes += data_size;
    return true;
  }

  out_used = sizeof(a2dp_lhdcv5_decoder_cb.decode_buf[dec_buf_idx]);
  fn_ret = lhdcdec_decode(a2dp_lhdcv5_decoder_cb.dec_input_buf,
      a2dp_lhdcv5_decoder_cb.dec_input_buf_bytes,
      a2dp_lhdcv5_decoder_cb.decode_buf[dec_buf_idx],
      &out_used,
      a2dp_lhdcv5_decoder_cb.bits_per_sample);

  // finish decoding
  // clear the data in the input buffer
  a2dp_lhdcv5_decoder_cb.dec_input_buf_bytes = 0;

  if (fn_ret != LHDCBT_DEC_FUNC_SUCCEED) {
    LOG_ERROR( "%s: fail to decode lhdc stream!", __func__);
    return false;
  }

  if (a2dp_lhdcv5_decoder_cb.bits_per_sample == 24) { //PCM_24_BIT_PACKCED
    ptr_src = a2dp_lhdcv5_decoder_cb.decode_buf[dec_buf_idx];
    ptr_dst = a2dp_lhdcv5_decoder_cb.decode_buf[dec_buf_idx];

    for (i = 0; i < (out_used >> 2) ; i++) {
      *ptr_dst++ = *ptr_src++;
      *ptr_dst++ = *ptr_src++;
      *ptr_dst++ = *ptr_src++;
      ptr_src++;
    }
    out_used = (out_used >> 2) * 3;
  } else if (a2dp_lhdcv5_decoder_cb.bits_per_sample == 32) {
    ptr_dst = a2dp_lhdcv5_decoder_cb.decode_buf[dec_buf_idx];

    for (i = 0; i < (out_used >> 2) ; i++) {
      ptr_dst[3] = ptr_dst[2];
      ptr_dst[2] = ptr_dst[1];
      ptr_dst[1] = ptr_dst[0];
      ptr_dst[0] = 0;
      ptr_dst+=4;
    }
  }

#if defined(_V5DEC_REC_FILE_)
  if (pcmFile != NULL && out_used > 0 && out_used <= sizeof(a2dp_lhdcv5_decoder_cb.decode_buf[dec_buf_idx])) {
    int write_bytes;
    write_bytes = fwrite(a2dp_lhdcv5_decoder_cb.decode_buf[dec_buf_idx], sizeof(uint8_t), out_used, pcmFile);
  }
#endif

  a2dp_lhdcv5_decoder_cb.decode_callback(
      reinterpret_cast<uint8_t*>(a2dp_lhdcv5_decoder_cb.decode_buf[dec_buf_idx]), out_used);

  return true;
}

void a2dp_vendor_lhdcv5_decoder_start(void) {
  LOG_INFO("%s", __func__);
  // do nothing
}

void a2dp_vendor_lhdcv5_decoder_suspend(void) {
  LOG_INFO("%s", __func__);
  // do nothing
}

void a2dp_vendor_lhdcv5_decoder_configure(const uint8_t* p_codec_info) {
  //int32_t sample_rate;
  //int32_t bits_per_sample;
  //int32_t channel_mode;

  if (p_codec_info == NULL) {
    LOG_ERROR("%s: p_codec_info is NULL", __func__);
    return;
  }

  LOG_ERROR("JIMM %s: enter", __func__);
}
