#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "lhdcv5BT_dec.h"

#define LOG_NDEBUG 0
#define LOG_TAG "lhdcv5BT_dec"
#include <cutils/log.h>

static uint8_t serial_no = 0xff;

// description
//   a function to log in LHDC decoder library
// Parameter
//   msg: char string to print
static void print_log_cb(char *msg)
{
  if (msg == NULL) {
    return;
  }

  ALOGD("[LHDCV5] %s", msg);
}


// description
//   check number of frames in one packet and return pointer to first byte of 1st frame in current packet
// Parameter
//   input: pointer to input buffer
//   input_len: length (bytes) of input buffer pointed by input
//   pLout: pointer to pointer to output buffer
//   pLlen: length (bytes) of encoded stream in output buffer
// return:
//   > 0: number of frames in current packet
//   == 0: No frames in current packet
//   < 0: error
static int assemble_lhdc_packet(uint8_t *input, uint32_t input_len,
    uint8_t **pLout, uint32_t *pLlen, int upd_seq_no)
{
  uint8_t hdr = 0, seqno = 0xff;
  int ret = LHDCBT_DEC_FUNC_FAIL;
  uint32_t status = 0;
  uint32_t lhdc_total_frame_nb = 0;

  if ((input == NULL) ||
      (pLout == NULL) ||
      (pLlen == NULL)) {
    return LHDCBT_DEC_FUNC_FAIL;
  }

  if (input_len < 2) {
    return LHDCBT_DEC_FUNC_FAIL;
  }

  hdr = (*input);
  input++;
  seqno = (*input);
  input++;
  input_len -= 2;

  //Check latency and update value when changed.
  status = hdr & A2DP_LHDC_HDR_LATENCY_MASK;

  //Get number of frame in packet.
  status = (hdr & A2DP_LHDC_HDR_FRAME_NO_MASK) >> 2;

  if (status <= 0) {
    ALOGD("%s: No any frame in packet.", __func__);
    return 0;
  }

  lhdc_total_frame_nb = status;

  if (seqno != serial_no) {
    ALOGD("%s: Packet lost! now(%d), expect(%d)", __func__, seqno, serial_no);
    //serial_no = seqno;
    //return LHDCBT_DEC_FUNC_INVALID_SEQ_NO;
  }

  if (upd_seq_no == LHDCBT_DEC_UPD_SEQ_NO) {
    serial_no = seqno + 1;
  }

  *pLlen = input_len;
  *pLout = input;

  ret = (int) lhdc_total_frame_nb;

  ALOGD("%s: ret total frame number (%d)", __func__, ret);
  return ret;
}


// description
//   init. LHDC v4 decoder 
// Parameter
//   config: configuration data for LHDC v4 decoder
// return:
//   == 0: succeed
//   < 0: error
int lhdcBT_dec_init_decoder(tLHDCV5_DEC_CONFIG *config)
{
  if (config == NULL) {
    return LHDCBT_DEC_FUNC_FAIL;
  }

  ALOGD("%s: bits_depth:%d sample_rate=%d version=%d", __func__,
      config->bits_depth, config->sample_rate, config->version);

  if ((config->bits_depth != LHDCV5BT_BIT_DEPTH_16) &&
      (config->bits_depth != LHDCV5BT_BIT_DEPTH_24) &&
      (config->bits_depth != LHDCV5BT_BIT_DEPTH_32)) {
    ALOGD("%s: bits_depth %d not supported", __func__, config->bits_depth);
    return LHDCBT_DEC_FUNC_FAIL;
  }

  if ((config->sample_rate != LHDCV5BT_SAMPLE_RATE_44K) &&
      (config->sample_rate != LHDCV5BT_SAMPLE_RATE_48K) &&
      (config->sample_rate != LHDCV5BT_SAMPLE_RATE_96K) &&
      (config->sample_rate != LHDCV5BT_SAMPLE_RATE_192K)) {
    ALOGD("%s: sample_rate %d not supported", __func__, config->sample_rate);
    return LHDCBT_DEC_FUNC_FAIL;
  }

  if ((config->version != VERSION_5)) {
    ALOGD("%s: version %d not supported", __func__, config->version);
    return LHDCBT_DEC_FUNC_FAIL;
  }

  lhdc_register_log_cb(&print_log_cb);

  ALOGD("%s: init lhdcv5 decoder", __func__);
  lhdcv5_util_init_decoder(config->bits_depth, config->sample_rate, 400000, config->version);
  lhdcv5_util_dec_channel_selsect(LHDC_OUTPUT_STEREO);

  serial_no = 0xff;

  return LHDCBT_DEC_FUNC_SUCCEED;
}


// description
//   check whether all frames of one packet are in buffer?
// Parameter
//   frameData: pointer to input buffer
//   frameBytes: length (bytes) of input buffer pointed by frameData
// return:
//   == 0: succeed
//   < 0: error
int lhdcBT_dec_check_frame_data_enough(const uint8_t *frameData,
    uint32_t frameBytes, uint32_t *packetBytes)
{
  uint8_t *frameDataStart = (uint8_t *)frameData;
  uint8_t *in_buf = NULL;
  uint32_t in_len = 0;
  uint32_t frame_num = 0;
  lhdc_frame_Info_t lhdc_frame_Info;
  uint32_t ptr_offset = 0;
  bool fn_ret;

  if ((frameData == NULL) || (packetBytes == NULL)) {
    return LHDCBT_DEC_FUNC_FAIL;
  }

  ALOGD("%s: enter, frameBytes (%d)", __func__, (int)frameBytes);

  *packetBytes = 0;

  frame_num = assemble_lhdc_packet(frameDataStart, frameBytes, &in_buf, &in_len,
      LHDCBT_DEC_NOT_UPD_SEQ_NO);

  if (frame_num == 0) {
    ALOGD("%s: assemble_lhdc_packet (%d)", __func__, (int)frame_num);
    return LHDCBT_DEC_FUNC_SUCCEED;
  }

  ALOGD("%s: in_buf (%p), frameData (%p), in_len (%d), frame_num (%d)", __func__,
      in_buf, frameData, (int)in_len, (int) frame_num);

  ptr_offset = 0;

  while ((frame_num > 0) && (ptr_offset < in_len))
  {
    fn_ret = lhdcv5_util_dec_fetch_frame_info(in_buf + ptr_offset, &lhdc_frame_Info);
    if (fn_ret == false) {
      ALOGD("%s: fetch frame info fail (%d)", __func__, (int)frame_num);
      return LHDCBT_DEC_FUNC_FAIL;
    }

    ALOGD("%s: lhdcFetchFrameInfo  frame_num (%d), ptr_offset (%d), "
        "lhdc_frame_Info.frame_len (%d), in_len (%d)", __func__,
        (int)frame_num, (int)ptr_offset, (int)lhdc_frame_Info.frame_len, (int)in_len);

    if ((ptr_offset + lhdc_frame_Info.frame_len) > in_len) {
      ALOGD(" %s: Not Enough... frame_num(%d), ptr_offset(%d), "
          "frame_len(%d), in_len (%d)", __func__,
          (int)frame_num, (int)ptr_offset,
          (int)lhdc_frame_Info.frame_len, (int)in_len);
      return LHDCBT_DEC_FUNC_INPUT_NOT_ENOUGH;
    }

    ptr_offset += lhdc_frame_Info.frame_len;

    frame_num--;
  }

  *packetBytes = ptr_offset;

  return LHDCBT_DEC_FUNC_SUCCEED;
}


// description
//   decode all frames in one packet
// Parameter
//   frameData: pointer to input buffer
//   frameBytes: length (bytes) of input buffer pointed by frameData
//   pcmData: pointer to output buffer
//   pcmBytes: length (bytes) of pcm samples in output buffer
// return:
//   == 0: succeed
//   < 0: error
int lhdcBT_dec_decode(const uint8_t *frameData, uint32_t frameBytes,
    uint8_t* pcmData, uint32_t* pcmBytes, uint32_t bits_depth)
{
  uint8_t *frameDataStart = (uint8_t *)frameData;
  uint32_t dec_sum = 0;
  uint32_t lhdc_out_len = 0;
  uint8_t *in_buf = NULL;
  uint32_t in_len = 0;
  uint32_t frame_num = 0;
  lhdc_frame_Info_t lhdc_frame_Info;
  uint32_t ptr_offset = 0;
  bool fn_ret;
  uint32_t frame_samples;
  uint32_t frame_bytes;
  uint32_t pcmSpaceBytes;

  ALOGD("%s: enter frameBytes %d", __func__, (int)frameBytes);

  if ((frameData == NULL) ||
      (pcmData == NULL) ||
      (pcmBytes == NULL)) {
    return LHDCBT_DEC_FUNC_FAIL;
  }

  pcmSpaceBytes = *pcmBytes;
  *pcmBytes = 0;

  /*
  if(frameBytes >= 16) {
    for(int i=0; i<16; i++) {
      ALOGD(" %s: dumpFrame[%d]= 0x%02X", __func__, i, (int)*(frameDataStart+i));
    }
  } else {
    for(int i=0; i<(int)frameBytes; i++) {
      ALOGD(" %s: dumpFrame[%d]= 0x%02X", __func__, i, (int)*(frameDataStart+i));
    }
  }
   */

  frame_num = assemble_lhdc_packet(frameDataStart, frameBytes, &in_buf, &in_len,
      LHDCBT_DEC_UPD_SEQ_NO);

  if (frame_num == 0) {
    return LHDCBT_DEC_FUNC_SUCCEED;
  }

  frame_samples = lhdcv5_util_dec_get_sample_size();
  if (bits_depth == 16) {
    frame_bytes = frame_samples * 2 * 2;
  } else {
    frame_bytes = frame_samples * 4 * 2;
  }
  ALOGD("%s: frame_samples=%d", __func__, (int)frame_samples);

  ptr_offset = 0;
  dec_sum = 0;

  while ((frame_num > 0) && (ptr_offset < in_len))
  {
    fn_ret = lhdcv5_util_dec_fetch_frame_info(in_buf + ptr_offset, &lhdc_frame_Info);
    if (fn_ret == false) {
      ALOGD("%s: fetch frame info fail (%d)", __func__, (int)frame_num);
      return LHDCBT_DEC_FUNC_FAIL;
    }

    if ((ptr_offset + lhdc_frame_Info.frame_len) > in_len) {
      return LHDCBT_DEC_FUNC_INPUT_NOT_ENOUGH;
    }

    if ((dec_sum + frame_bytes) > pcmSpaceBytes) {
      return LHDCBT_DEC_FUNC_OUTPUT_NOT_ENOUGH;
    }

    //ALOGD("%s: get ptr_offset=%d, dec_sum=%d", __func__, ptr_offset, dec_sum);
    lhdc_out_len = lhdcv5_util_dec_process(((uint8_t *)pcmData) + dec_sum,
        in_buf + ptr_offset, lhdc_frame_Info.frame_len);

    ALOGD("%s: frm=%d, frame_len=%d out_len=%d..", __func__,
        (int)frame_num, (int)lhdc_frame_Info.frame_len, (int)lhdc_out_len);

    ptr_offset += lhdc_frame_Info.frame_len;
    dec_sum += lhdc_out_len;

    frame_num--;
  }

  *pcmBytes = (uint32_t) dec_sum;

  return LHDCBT_DEC_FUNC_SUCCEED;
}


// description
//   de-initialize (free) all resources allocated by LHDC V5 decoder
// Parameter
//   none
// return:
//   == 0: success
int lhdcBT_dec_deinit_decoder(void)
{
  int32_t ret = 0;

  ret = lhdcv5_util_dec_destroy();
  ALOGD("%s: ret %d", __func__, ret);

  return LHDCBT_DEC_FUNC_SUCCEED;
}

