#ifndef _LHDCV5BT_DEC_H_
#define _LHDCV5BT_DEC_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "lhdcv5Util.h"

#define LHDCV5BT_SAMPLE_RATE_44K    (44100)
#define LHDCV5BT_SAMPLE_RATE_48K    (48000)
#define LHDCV5BT_SAMPLE_RATE_96K    (96000)
#define LHDCV5BT_SAMPLE_RATE_192K   (192000)

#define LHDCV5BT_BIT_DEPTH_16    (16)
#define LHDCV5BT_BIT_DEPTH_24    (24)
#define LHDCV5BT_BIT_DEPTH_32    (32)


typedef struct  
{
  lhdc_ver_t version;
  uint32_t   sample_rate;
  uint8_t    bits_depth;
} tLHDCV5_DEC_CONFIG;



int lhdcBT_dec_init_decoder(tLHDCV5_DEC_CONFIG *config);
int lhdcBT_dec_check_frame_data_enough(const uint8_t *frameData, uint32_t frameBytes, uint32_t *packetBytes);
int lhdcBT_dec_decode(const uint8_t *frameData, uint32_t frameBytes, uint8_t* pcmData, uint32_t* pcmBytes, uint32_t bits_depth);
int lhdcBT_dec_deinit_decoder(void);


#define LHDCBT_DEC_NOT_UPD_SEQ_NO			0
#define LHDCBT_DEC_UPD_SEQ_NO				1


#define LHDCBT_DEC_FUNC_SUCCEED             0
#define LHDCBT_DEC_FUNC_FAIL                -1
#define LHDCBT_DEC_FUNC_INPUT_NOT_ENOUGH    -2
#define LHDCBT_DEC_FUNC_OUTPUT_NOT_ENOUGH   -3
#define LHDCBT_DEC_FUNC_INVALID_SEQ_NO		-4

#ifdef __cplusplus
}
#endif
#endif /* _LHDCBT_DEC_H_ */
