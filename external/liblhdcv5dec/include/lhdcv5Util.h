/*
 * lhdcv5Util.h
 *
 *  Created on: 2022/03/18
 *      Author: jimmy chen
 */

#ifndef LHDC_UTIL_H
#define LHDC_UTIL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Copy definition from external
#define BTIF_BD_ADDR_SIZE    6

// Define for LHDC stream type.
typedef enum {
  LHDC_STRM_TYPE_COMBINE,
  LHDC_STRM_TYPE_SPLIT
}LHDC_STRM_TYPE;

typedef enum {
  VERSION_2 = 200,
  VERSION_3 = 300,
  VERSION_4 = 400,
  VERSION_LLAC = 500,
  VERSION_5 = 550
}lhdc_ver_t;

typedef enum {
  LHDCV2_BLOCK_SIZE = 512,
  LHDCV3_BLOCK_SIZE = 256,
}lhdc_block_size_t;

typedef struct savi_bt_local_info_t{
  uint8_t bt_addr[BTIF_BD_ADDR_SIZE];
  const char *bt_name;
  uint8_t bt_len;
  uint8_t ble_addr[BTIF_BD_ADDR_SIZE];
  const char *ble_name;
  uint8_t ble_len;
}savi_bt_local_info;

typedef struct _lhdc_frame_Info
{
  uint32_t frame_len;
  uint32_t isSplit;
  uint32_t isLeft;

} lhdc_frame_Info_t;


typedef enum {
  LHDC_OUTPUT_STEREO = 0,
  LHDC_OUTPUT_LEFT_CAHNNEL,
  LHDC_OUTPUT_RIGHT_CAHNNEL,
} lhdc_channel_t;

typedef int LHDCSample;

typedef void (*print_log_fp)(char*  msg);
typedef int (*LHDC_GET_BT_INFO)(savi_bt_local_info * bt_info);



#define A2DP_LHDC_HDR_LATENCY_LOW   0x00
#define A2DP_LHDC_HDR_LATENCY_MID   0x01
#define A2DP_LHDC_HDR_LATENCY_HIGH  0x02
#define A2DP_LHDC_HDR_LATENCY_MASK  (A2DP_LHDC_HDR_LATENCY_MID | A2DP_LHDC_HDR_LATENCY_HIGH)

#define A2DP_LHDC_HDR_FRAME_NO_MASK 0xfc


int32_t lhdcv5_util_init_decoder(uint32_t bitPerSample, uint32_t sampleRate, uint32_t scaleTo16Bits, lhdc_ver_t version);
uint32_t lhdcv5_util_dec_put_data(uint8_t * pInpBuf, uint32_t NumBytes);
uint32_t lhdcv5_util_dec_process(uint8_t * pOutBuf, uint8_t * pInput, uint32_t len);
bool lhdcv5_util_set_license(uint8_t * licTable, LHDC_GET_BT_INFO pFunc);
int32_t lhdcv5_util_set_license_check_period (uint8_t period);
char *lhdcv5_util_dec_get_version();

int32_t lhdcv5_util_dec_destroy();

void lhdc_register_log_cb(print_log_fp cb);

uint32_t lhdcv5_util_dec_get_sample_size (void);
bool lhdcv5_util_dec_fetch_frame_info(uint8_t * frameData, lhdc_frame_Info_t * frameInfo);

uint32_t lhdcv5_util_dec_channel_selsect(lhdc_channel_t channel_type);

#ifdef __cplusplus
}
#endif
#endif /* End of LHDC_UTIL_H */
