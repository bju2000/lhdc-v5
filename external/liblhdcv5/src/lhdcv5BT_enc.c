#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "lhdcv5BT.h"
#include "lhdcv5BT_ext_func.h"

#define LOG_TAG "lhdcv5BT_enc"
#include <cutils/log.h>

#define UP_RATE_TIME_CNT         			3000    // ABR bitrate upgrade checking time interval(ms): (UP_RATE_TIME_CNT * 20ms)
#define DOWN_RATE_TIME_CNT       			4  		  // ABR bitrate downgrade checking time interval(ms): (DOWN_RATE_TIME_CNT * 20ms)
#define ABR_UP_QUEUE_LENGTH_THRESHOLD     1   // A threshold of ABR bitrate upgrade check
#define ABR_DOWN_QUEUE_LENGTH_THRESHOLD   0   // A threshold of ABR bitrate downgrade check

// ABR: bit rate adjustment tables
//static uint32_t auto_bitrate_adjust_table_lhdcv5_std[] = {128, 192, 256, 320, 400, 600};
//static uint32_t auto_bitrate_adjust_table_lhdcv5_hi[] = {256, 320, 400, 600, 600, 600};
//static uint32_t auto_bitrate_adjust_table_lhdcv5_std[] = {128, 192, 256, 320, 400, 400};
//static uint32_t auto_bitrate_adjust_table_lhdcv5_hi[] = {256, 320, 400, 400, 400, 400};
static uint32_t auto_bitrate_adjust_table_lhdcv5_44k[] = {128, 192, 240, 320, 400, 400};
static uint32_t auto_bitrate_adjust_table_lhdcv5_48k[] = {128, 192, 256, 320, 400, 400};
static uint32_t auto_bitrate_adjust_table_lhdcv5_96k[] = {256, 320, 400, 400, 400, 400};
static uint32_t auto_bitrate_adjust_table_lhdcv5_192k[] = {256, 320, 400, 400, 400, 400};

//#define LHDCV5_STD_BITRATE_ELEMENTS_SIZE	(sizeof(auto_bitrate_adjust_table_lhdcv5_std) / sizeof(uint32_t))
//#define LHDCV5_HI_BITRATE_ELEMENTS_SIZE		(sizeof(auto_bitrate_adjust_table_lhdcv5_hi) / sizeof(uint32_t))
#define LHDCV5_44K_BITRATE_ELEMENTS_SIZE  (sizeof(auto_bitrate_adjust_table_lhdcv5_44k) / sizeof(uint32_t))
#define LHDCV5_48K_BITRATE_ELEMENTS_SIZE  (sizeof(auto_bitrate_adjust_table_lhdcv5_48k) / sizeof(uint32_t))
#define LHDCV5_96K_BITRATE_ELEMENTS_SIZE  (sizeof(auto_bitrate_adjust_table_lhdcv5_96k) / sizeof(uint32_t))
#define LHDCV5_192K_BITRATE_ELEMENTS_SIZE  (sizeof(auto_bitrate_adjust_table_lhdcv5_192k) / sizeof(uint32_t))

#define LHDCV5_ABR_DEFAULT_BITRATE     (LHDCV5_QUALITY_LOW)


static const char * rate_to_string
(
    LHDCV5_QUALITY_T	q
)
{
  switch (q)
  {
  case LHDCV5_QUALITY_LOW0:
    return "LHDCV5_QUALITY_LOW0";
  case LHDCV5_QUALITY_LOW1:
    return "LHDCV5_QUALITY_LOW1";
  case LHDCV5_QUALITY_LOW2:
    return "LHDCV5_QUALITY_LOW2";
  case LHDCV5_QUALITY_LOW3:
    return "LHDCV5_QUALITY_LOW3";
  case LHDCV5_QUALITY_LOW4:
    return "LHDCV5_QUALITY_LOW4";
  case LHDCV5_QUALITY_LOW:
    return "LHDCV5_QUALITY_LOW";
  case LHDCV5_QUALITY_MID:
    return "LHDCV5_QUALITY_MID";
  case LHDCV5_QUALITY_HIGH:
    return "LHDCV5_QUALITY_HIGH";
  case LHDCV5_QUALITY_HIGH1:
    return "LHDCV5_QUALITY_HIGH1";
  case LHDCV5_QUALITY_AUTO:
    return "LHDCV5_QUALITY_AUTO";
  default:
    ALOGW ("%s: Incorrect quality(%d)",  __func__, q);
    return "UNKNOW_QUALITY";
  }
}


//----------------------------------------------------------------
// lhdcv5_encoder_inx_of_abr_bitrate ()
//
// return the bit rate (index) respond to input bit rate (kbps)
//	Parameter
//		abr_type: ABR type
//		bitrate: bit rate (kbps)
//		bitrate_inx: ABR bit rate (index) returned
//	Return
//		LHDCV5_FRET_SUCCESS: succeed to return the bit rate (index)
//		otherwise: fail to return the bit rate (index)
//----------------------------------------------------------------
static int lhdcv5_encoder_inx_of_abr_bitrate
(
    LHDCV5_ABR_TYPE_T abr_type,
    uint32_t  bitrate,
    uint32_t  *bitrate_inx
)
{
  uint32_t element_size = LHDCV5_48K_BITRATE_ELEMENTS_SIZE;
  uint32_t *abr_table = NULL;

  if (bitrate_inx == NULL)
  {
    ALOGW ("%s: Input parameter is NULL!!!", __func__);
    return LHDCV5_FRET_INVALID_INPUT_PARAM;
  }

  if (abr_type == LHDCV5_ABR_44K_RES)
  {
    element_size = LHDCV5_44K_BITRATE_ELEMENTS_SIZE;

    abr_table = &(auto_bitrate_adjust_table_lhdcv5_44k[0]);
  }
  else if (abr_type == LHDCV5_ABR_48K_RES)
  {
    element_size = LHDCV5_48K_BITRATE_ELEMENTS_SIZE;

    abr_table = &(auto_bitrate_adjust_table_lhdcv5_48k[0]);
  }
  else if (abr_type == LHDCV5_ABR_96K_RES)
  {
    element_size = LHDCV5_96K_BITRATE_ELEMENTS_SIZE;

    abr_table = &(auto_bitrate_adjust_table_lhdcv5_96k[0]);
  }
  else if (abr_type == LHDCV5_ABR_192K_RES)
  {
    element_size = LHDCV5_192K_BITRATE_ELEMENTS_SIZE;

    abr_table = &(auto_bitrate_adjust_table_lhdcv5_192k[0]);
  }
  else
  {
    ALOGW ("%s: Invalid ABR type (%d)!", __func__, abr_type);
    return LHDCV5_FRET_INVALID_INPUT_PARAM;
  }

  if ((bitrate <= 0) || (bitrate > abr_table[element_size - 1]))
  {
    ALOGW ("%s: bit rate is out of range (%d)!!!", __func__, bitrate);
    return LHDCV5_FRET_INVALID_INPUT_PARAM;
  }


  for (uint32_t i = 0; i < element_size; i++)
  {
    if (abr_table[i] >= bitrate)
    {
      *bitrate_inx = i;
      return LHDCV5_FRET_SUCCESS;
    }
  }

  ALOGW ("%s: Fail to find bit rate (index) (%d)!", __func__, bitrate);
  return LHDCV5_FRET_ERROR;
}


//----------------------------------------------------------------
// lhdcv5_encoder_adjust_bitrate ()
//
// Adjust bit rate automatically according to number of packets in queue for LHDC 5.0 encoding
//	Parameter
//		lhdcBT: a pointer to the resource allocated and is returned 
//				by function lhdcBT_get_handle ()
//		handle: a pointer to ABR parameters
//		queueLen: number of packets in queue
//	Return
//		LHDCV5_FRET_SUCCESS: succeed to adjust bit rate automatically 
//		otherwise: fail to adjust bit rate automatically 
//----------------------------------------------------------------
static int lhdcv5_encoder_adjust_bitrate
(
    HANDLE_LHDCV5_BT  lhdcBT,
    lhdcv5_abr_para_t *handle,
    uint32_t  queueLen
) 
{
  uint32_t element_size = LHDCV5_48K_BITRATE_ELEMENTS_SIZE;
  uint32_t new_abr_bitrate_inx = 0;
  uint32_t new_bitrate_inx = 0;
  uint32_t new_bitrate_inx_set = 0;
  uint32_t last_bitrate_inx = 0;
  uint32_t *abr_table = &(auto_bitrate_adjust_table_lhdcv5_48k[0]);
  LHDCV5_ABR_TYPE_T	abr_type = LHDCV5_ABR_48K_RES;
  int32_t func_ret = LHDCV5_FRET_SUCCESS;
  uint32_t queueLength = 0;

  if (lhdcBT == NULL)
  {
    ALOGW ("%s: lhdcBT is NULL!", __func__);
    return LHDCV5_FRET_INVALID_HANDLE_CB;
  }

  if (handle == NULL)
  {
    ALOGW ("%s: Handle is NULL!", __func__);
    return LHDCV5_FRET_INVALID_HANDLE_PARA;
  }

  if (handle->qualityStatus != LHDCV5_QUALITY_AUTO)
  {
    ALOGW ("%s: Not ABR (%d)", __func__, handle->qualityStatus);
    return LHDCV5_FRET_SUCCESS;
  }

  if (queueLen < 0)
  {
    ALOGW ("%s: Invalid queue length (%u)!", __func__, queueLen);
    return LHDCV5_FRET_INVALID_HANDLE_PARA;
  }

  if (handle->sample_rate == LHDCV5_SR_44100HZ)
  {
    abr_type = LHDCV5_ABR_44K_RES;
    element_size = LHDCV5_44K_BITRATE_ELEMENTS_SIZE;
    abr_table = &(auto_bitrate_adjust_table_lhdcv5_44k[0]);
  }
  else if (handle->sample_rate == LHDCV5_SR_48000HZ)
  {
    abr_type = LHDCV5_ABR_48K_RES;
    element_size = LHDCV5_48K_BITRATE_ELEMENTS_SIZE;
    abr_table = &(auto_bitrate_adjust_table_lhdcv5_48k[0]);
  }
  else if (handle->sample_rate == LHDCV5_SR_96000HZ)
  {
    abr_type = LHDCV5_ABR_96K_RES;
    element_size = LHDCV5_96K_BITRATE_ELEMENTS_SIZE;
    abr_table = &(auto_bitrate_adjust_table_lhdcv5_96k[0]);
  }
  else if (handle->sample_rate == LHDCV5_SR_192000HZ)
  {
    abr_type = LHDCV5_ABR_192K_RES;
    element_size = LHDCV5_192K_BITRATE_ELEMENTS_SIZE;
    abr_table = &(auto_bitrate_adjust_table_lhdcv5_192k[0]);
  }
  else
  {
    ALOGW ("%s: Sample rate is invalid (%u)!", __func__, handle->sample_rate);
    return LHDCV5_FRET_CODEC_NOT_READY;
  }

  if (handle->dnBitrateCnt > 0 && handle->dnBitrateCnt >= DOWN_RATE_TIME_CNT)
  {
    queueLength = handle->dnBitrateSum / handle->dnBitrateCnt;

    //handle->dnBitrateSum = 0;
    //handle->dnBitrateCnt = 0;
    func_ret = lhdcv5_util_reset_down_bitrate (handle);
    if (func_ret != LHDCV5_FRET_SUCCESS)
    {
      ALOGW ("%s:(DN) lhdcv5_util_reset_down_bitrate error %d",  __func__, func_ret);
      goto fail;
    }

    if (queueLength > ABR_DOWN_QUEUE_LENGTH_THRESHOLD)
    {
      ALOGD ("%s:(DN) dnBitrateSum:%u / dnBitrateCnt:%u = queueLength:%u", __func__,
          handle->dnBitrateSum, handle->dnBitrateCnt, queueLength);

      new_abr_bitrate_inx = 0;
      func_ret = lhdcv5_util_get_bitrate_inx (abr_table[new_abr_bitrate_inx], &new_bitrate_inx);
      if (func_ret != LHDCV5_FRET_SUCCESS)
      {
        ALOGW ("%s:(DN) lhdcv5_util_get_bitrate_inx error %d",  __func__, func_ret);
        goto fail;
      }

      func_ret = lhdcv5_util_get_bitrate_inx (handle->lastBitrate, &last_bitrate_inx);
      if (func_ret != LHDCV5_FRET_SUCCESS)
      {
        ALOGW ("%s:(DN) lhdcv5_util_get_bitrate_inx error %d",  __func__, func_ret);
        goto fail;
      }

      if (new_bitrate_inx < last_bitrate_inx)
      {
        func_ret = lhdcv5_util_set_target_bitrate_inx (lhdcBT, new_bitrate_inx, &new_bitrate_inx_set, false);
        if (func_ret != LHDCV5_FRET_SUCCESS)
        {
          ALOGW ("%s:(DN) lhdcv5_util_set_target_bitrate_inx error %d", __func__, func_ret);
          goto fail;
        }

        ALOGD ("%s:(DN) [AutoBiTrAtE] Down bitrate[%u] to new bitrate(%u)[%u], queueLength(%u)",  __func__,
            last_bitrate_inx, abr_table[new_abr_bitrate_inx], new_bitrate_inx_set, queueLength);

        func_ret = lhdcv5_util_reset_up_bitrate (handle);
        if (func_ret != LHDCV5_FRET_SUCCESS)
        {
          ALOGW ("%s:(DN) lhdcv5_util_reset_up_bitrate error %d", __func__, func_ret);
          goto fail;
        }
      }
      else
      {
        ALOGD ("%s:(DN) [AutoBiTrAtE] Down bitrate condition fails, new rate:%u, current rate:%u",  __func__,
            abr_table[new_abr_bitrate_inx],
            handle->lastBitrate);
      }
    }
  }

  if (handle->upBitrateCnt > 0 && handle->upBitrateCnt >= UP_RATE_TIME_CNT)
  {
    uint32_t queueLength = handle->upBitrateSum / handle->upBitrateCnt;
    uint32_t queuSumTmp = handle->upBitrateSum;

    ALOGV ("%s:(UP) Before upBitrateSum:%u / upBitrateCnt:%u = queueLength:%u", __func__,
        handle->upBitrateSum, handle->upBitrateCnt, queueLength);

    //handle->upBitrateSum = 0;
    //handle->upBitrateCnt = 0;
    func_ret = lhdcv5_util_reset_up_bitrate (handle);
    if (func_ret != LHDCV5_FRET_SUCCESS)
    {
      ALOGW ("%s:(UP) lhdcv5_util_reset_up_bitrate error %d", __func__, func_ret);
      goto fail;
    }

    func_ret = lhdcv5_encoder_inx_of_abr_bitrate (abr_type,
        handle->lastBitrate,
        &new_abr_bitrate_inx);
    if (func_ret != LHDCV5_FRET_SUCCESS)
    {
      ALOGW ("%s:(UP) lhdcv5_encoder_inx_of_abr_bitrate error %d", __func__, func_ret);
      goto fail;
    }

    if (new_abr_bitrate_inx < (element_size - 1))
    {
      new_abr_bitrate_inx++;
    }

    func_ret = lhdcv5_util_get_bitrate_inx (abr_table[new_abr_bitrate_inx], &new_bitrate_inx);
    if (func_ret != LHDCV5_FRET_SUCCESS)
    {
      ALOGW ("%s:(UP) lhdcv5_util_get_bitrate_inx error %d", __func__, func_ret);
      goto fail;
    }

    func_ret = lhdcv5_util_get_bitrate_inx (handle->lastBitrate, &last_bitrate_inx);
    if (func_ret != LHDCV5_FRET_SUCCESS)
    {
      ALOGW ("%s:(UP) lhdcv5_util_get_bitrate_inx error %d", __func__, func_ret);
      goto fail;
    }

    if ((new_bitrate_inx > last_bitrate_inx) && (queuSumTmp < ABR_UP_QUEUE_LENGTH_THRESHOLD))
    {
      func_ret = lhdcv5_util_set_target_bitrate_inx (lhdcBT, new_bitrate_inx, &new_bitrate_inx_set, false);
      if (func_ret != LHDCV5_FRET_SUCCESS)
      {
        ALOGW ("%s:(UP) lhdcv5_util_set_target_bitrate_inx error %d", __func__, func_ret);
        goto fail;
      }
      ALOGD ("%s:(UP) [AutoBiTrAtE] Up bitrate[%u] to new_bitrate(%u)[%u], queueLength(%u)",
          __func__, last_bitrate_inx, abr_table[new_abr_bitrate_inx], new_bitrate_inx_set, queueLength);

      func_ret = lhdcv5_util_reset_down_bitrate (handle);
      if (func_ret != LHDCV5_FRET_SUCCESS)
      {
        ALOGW ("%s:(UP) lhdcv5_util_reset_down_bitrate error %d", __func__, func_ret);
        goto fail;
      }
    }
    else
    {
      ALOGD ("%s:(UP) [AutoBiTrAtE] Up bitrate condition fails, new rate:%u, current rate:%u, queue sum:%u",  __func__,
          abr_table[new_abr_bitrate_inx],
          handle->lastBitrate,
          queuSumTmp);
    }
  }

  if (queueLen > 0)
  {
    handle->upBitrateSum += queueLen;
    handle->dnBitrateSum += queueLen;
  }

  handle->upBitrateCnt++;
  handle->dnBitrateCnt++;

  fail:
  return func_ret;
}


/*
 ******************************************************************
 LHDC library public API group
 ******************************************************************
 */

//----------------------------------------------------------------
// lhdcv5BT_free_handle ()
//
// Free all resources allocated
//	Parameter
//		handle: a pointer to the resource allocated and is returned 
//				by function lhdcv5BT_get_handle ()
//	Return
//		LHDCV5_FRET_SUCCESS: Succeed to free all resources
//		otherwise: Fail to free all resources
//----------------------------------------------------------------
int32_t lhdcv5BT_free_handle
(
    HANDLE_LHDCV5_BT	handle
) 
{
  int32_t func_ret = LHDCV5_FRET_SUCCESS;

  if (handle == NULL)
  {
    ALOGW ("%s: Handle is NULL!", __func__);
    return LHDCV5_FRET_INVALID_HANDLE_CB;
  }

  // reset resources
  func_ret = lhdcv5_util_free_handle (handle);

  // free handle
  if(handle)
  {
    ALOGD ("%s: free handle %p!", __func__, handle);
    free(handle);
    handle = NULL;
  }

  return func_ret;
}


//----------------------------------------------------------------
// lhdcv5BT_get_handle ()
//
// Allocate resources required by LHDC 5.0 Encoder 
// 	Parameter
//		version: version defined in BT A2DP capability
//		handle: a pointer to the resource allocated 
// 	Return
//		LHDCV5_FRET_SUCCESS: Succeed to allocate resources
//		otherwise: Fail to allocate resources
//----------------------------------------------------------------
int32_t lhdcv5BT_get_handle
(
    uint32_t			version,
    HANDLE_LHDCV5_BT	*handle
) 
{
  int32_t		func_ret = LHDCV5_FRET_SUCCESS;
  uint32_t mem_req_bytes = 0;
  HANDLE_LHDCV5_BT hLhdcBT = NULL;

  if (version != LHDCV5_VERSION_1)
  {
    ALOGW ("%s: Invalid version (%u)!", __func__, version);
    return LHDCV5_FRET_ERROR;
  }

  if (handle == NULL)
  {
    ALOGW ("%s: Input parameter is NULL!", __func__);
    return LHDCV5_FRET_INVALID_INPUT_PARAM;
  }

  func_ret = lhdcv5_util_get_mem_req (version, &mem_req_bytes);

  if (func_ret != LHDCV5_FRET_SUCCESS || mem_req_bytes <= 0)
  {
    ALOGW ("%s: Fail to get required memory size (%d)!", __func__, func_ret);
    return LHDCV5_FRET_ERROR;
  }

  hLhdcBT = (HANDLE_LHDCV5_BT)malloc(mem_req_bytes);
  if (hLhdcBT == NULL)
  {
    ALOGW ("%s: Fail to allocate memory for encoder!", __func__);
    return LHDCV5_FRET_ERROR;
  }

  func_ret = lhdcv5_util_get_handle (version, hLhdcBT, mem_req_bytes);

  if (func_ret != LHDCV5_FRET_SUCCESS)
  {
    ALOGW ("%s: Fail to get handle (%d)!", __func__, func_ret);
    free(hLhdcBT);
    return LHDCV5_FRET_ERROR;
  }

  *handle = hLhdcBT;

  if ((*handle) == NULL)
  {
    ALOGW ("%s: Get handle NULL!", __func__);
    return LHDCV5_FRET_INVALID_HANDLE_CB;
  }

  return LHDCV5_FRET_SUCCESS;
}


//----------------------------------------------------------------
// lhdcv5BT_get_bitrate ()
//
// Get the bit rate used during LHDC 5.0 encoding
//	Parameter
//		handle: a pointer to the resource allocated and is returned 
//				by function lhdcv5BT_get_handle ()
//		bitrate: a pointer to bit rate used during LHDC 5.0 encoding,
//				 range [64000, 1000000]
//	Return
//		LHDCV5_FRET_SUCCESS: Succeed to allocate resources
//		otherwise: Fail to allocate resources
//----------------------------------------------------------------
int32_t lhdcv5BT_get_bitrate
(
    HANDLE_LHDCV5_BT	handle,
    uint32_t			*bitrate
) 
{
  int32_t		func_ret = LHDCV5_FRET_SUCCESS;

  if (handle == NULL)
  {
    ALOGW ("%s: Handle is NULL!", __func__);
    return LHDCV5_FRET_INVALID_HANDLE_CB;
  }

  if (bitrate == NULL)
  {
    ALOGW ("%s: Input parameter is NULL!", __func__);
    return LHDCV5_FRET_INVALID_INPUT_PARAM;
  }

  func_ret = lhdcv5_util_get_target_bitrate (handle, bitrate);

  if (func_ret != LHDCV5_FRET_SUCCESS)
  {
    ALOGW ("%s: Failed to get bit rate (%d)!", __func__, func_ret);
    return LHDCV5_FRET_ERROR;
  }

  if ((*bitrate < 64000) || (*bitrate > 1000000))
  {
    ALOGW ("%s: Invalid bit rate returned (%u)!", __func__, *bitrate);
    return LHDCV5_FRET_ERROR;
  }

  return LHDCV5_FRET_SUCCESS;
}


//----------------------------------------------------------------
// lhdcv5BT_set_bitrate ()
//
// Set the bit rate used during LHDC 5.0 encoding
//	Parameter
//		handle: a pointer to the resource allocated and is returned 
//				by function lhdcBT_get_handle ()
//		bitrate_inx: an index of bit rate to set
//	Return
//		LHDCV5_FRET_SUCCESS: succeed to set the bit rate
//		Other: fail to set the bit rate
//----------------------------------------------------------------
int32_t lhdcv5BT_set_bitrate
(
    HANDLE_LHDCV5_BT 	handle,
    uint32_t			bitrate_inx
)
{
  uint32_t	bitrate_inx_set = bitrate_inx;
  int32_t		func_ret = LHDCV5_FRET_SUCCESS;

  if (handle == NULL)
  {
    ALOGW ("%s: Handle is NULL!", __func__);
    return LHDCV5_FRET_INVALID_HANDLE_CB;
  }

  if ((bitrate_inx < LHDCV5_QUALITY_LOW0) ||
      (bitrate_inx >= LHDCV5_QUALITY_INVALID))
  {
    ALOGW ("%s: Invalid bit rate index (%u)!", __func__, bitrate_inx);
    return LHDCV5_FRET_INVALID_INPUT_PARAM;
  }

  if (bitrate_inx == LHDCV5_QUALITY_RESET_AUTO) {
    bitrate_inx_set = LHDCV5_ABR_DEFAULT_BITRATE;
    ALOGD ("%s: [Reset BiTrAtE] reset to bitrate (%s)", __func__, rate_to_string (bitrate_inx_set));
    bitrate_inx = LHDCV5_ABR_DEFAULT_BITRATE;
    // change bitrate only, not update quality index
    func_ret = lhdcv5_util_set_target_bitrate_inx (handle, bitrate_inx, &bitrate_inx_set, false);
  } else {
    // also update quality index
    func_ret = lhdcv5_util_set_target_bitrate_inx (handle, bitrate_inx, &bitrate_inx_set, true);
  }

  if (func_ret != LHDCV5_FRET_SUCCESS)
  {
    ALOGW ("%s: failed to set bitrate (%d)!", __func__, func_ret);
    return LHDCV5_FRET_ERROR;
  }

  ALOGD ("%s: Update target bitrate(%s)",  __func__,
      rate_to_string (bitrate_inx_set));

  return LHDCV5_FRET_SUCCESS;
}


//----------------------------------------------------------------
// lhdcv5BT_set_max_bitrate ()
//
// Set the MAX. bit rate for LHDC 5.0 encoding
//	Parameter
//		handle: a pointer to the resource allocated and is returned 
//				by function lhdcBT_get_handle ()
//		max_bitrate_inx: MAX. bit rate (index) for LHDC 5.0 encoding
//	Return
//		LHDCV5_FRET_SUCCESS: succeed to set the MAX. bit rate
//		Other: fail to set the MAX. bit rate
//----------------------------------------------------------------
int32_t lhdcv5BT_set_max_bitrate
(
    HANDLE_LHDCV5_BT	handle,
    uint32_t			max_bitrate_inx
) 
{
  uint32_t max_bitrate_inx_set = max_bitrate_inx;
  int32_t func_ret = LHDCV5_FRET_SUCCESS;

  if (handle == NULL)
  {
    ALOGW ("%s: Handle is NULL!", __func__);
    return LHDCV5_FRET_INVALID_HANDLE_CB;
  }

  if ((max_bitrate_inx < LHDCV5_QUALITY_LOW) ||
      (max_bitrate_inx >= LHDCV5_QUALITY_AUTO))
  {
    ALOGW ("%s: Invalid max bit rate index (%u)!", __func__, max_bitrate_inx);
    return LHDCV5_FRET_INVALID_INPUT_PARAM;
  }

  func_ret = lhdcv5_util_set_max_bitrate_inx (handle, max_bitrate_inx, &max_bitrate_inx_set);

  if (func_ret != LHDCV5_FRET_SUCCESS)
  {
    ALOGW ("%s: failed to set max. bit rate index (%u), (%d)!", __func__, max_bitrate_inx, func_ret);
    return LHDCV5_FRET_ERROR;
  }

  ALOGD ("%s: Update Max target bitrate(%s)",  __func__, rate_to_string (max_bitrate_inx_set));

  return LHDCV5_FRET_SUCCESS;
}


//----------------------------------------------------------------
// lhdcv5BT_set_min_bitrate ()
//
// Set the MIN. bit rate for LHDC 5.0 encoding
//	Parameter
//		handle: a pointer to the resource allocated and is returned 
//				by function lhdcBT_get_handle ()
//		min_bitrate_inx: MIN. bit rate (index) for LHDC 5.0 encoding
//	Return
//		LHDCV5_FRET_SUCCESS: succeed to set the MIN. bit rate
//		Other: fail to set the MIN. bit rate
//----------------------------------------------------------------

int32_t lhdcv5BT_set_min_bitrate
(
    HANDLE_LHDCV5_BT 	handle,
    uint32_t			min_bitrate_inx
)
{
  uint32_t 	min_bitrate_inx_set = (uint32_t) LHDCV5_QUALITY_LOW0;
  int32_t			func_ret = LHDCV5_FRET_SUCCESS;

  if (handle == NULL)
  {
    ALOGW ("%s: Handle is NULL!", __func__);
    return LHDCV5_FRET_INVALID_HANDLE_CB;
  }

  if ((min_bitrate_inx < LHDCV5_QUALITY_LOW0) ||
      (min_bitrate_inx > LHDCV5_QUALITY_LOW))
  {
    ALOGW ("%s: Invalid min bit rate index (%u)!", __func__, min_bitrate_inx);
    return LHDCV5_FRET_INVALID_INPUT_PARAM;
  }

  func_ret = lhdcv5_util_set_min_bitrate_inx (handle, min_bitrate_inx, &min_bitrate_inx_set);

  if (func_ret != LHDCV5_FRET_SUCCESS)
  {
    ALOGW ("%s: failed to set min. bit rate (%d)!", __func__, func_ret);
    return LHDCV5_FRET_ERROR;
  }

  return LHDCV5_FRET_SUCCESS;
}


//----------------------------------------------------------------
// lhdcv5BT_adjust_bitrate ()
//
// Adjust bit rate automatically according to number of packets in queue for LHDC 5.0 encoding
//	Parameter
//		handle: a pointer to the resource allocated and is returned 
//				by function lhdcBT_get_handle ()
//		queue_len: number of packets in queue
//	Return
//		LHDCV5_FRET_SUCCESS: succeed to adjust bit rate automatically 
//		Other: fail to adjust bit rate automatically 
//----------------------------------------------------------------

int32_t lhdcv5BT_adjust_bitrate
(
    HANDLE_LHDCV5_BT 	handle,
    uint32_t			queueLen
) 
{
  LHDCV5_ENC_TYPE_T	enc_type = LHDCV5_ENC_TYPE_LHDCV5;
  lhdcv5_abr_para_t	* abr_para = NULL;
  int32_t				func_ret = LHDCV5_FRET_SUCCESS;

  if (handle == NULL)
  {
    ALOGW ("%s: Handle is NULL!", __func__);
    return LHDCV5_FRET_INVALID_HANDLE_CB;
  }

  if (queueLen < 0)
  {
    ALOGW ("%s: Invalid input queue length (%u)!", __func__, queueLen);
    return LHDCV5_FRET_INVALID_INPUT_PARAM;
  }

  func_ret = lhdcv5_util_adjust_bitrate (handle, &enc_type, &abr_para);

  if ((func_ret != LHDCV5_FRET_SUCCESS) || (abr_para == NULL))
  {
    ALOGW ("%s: Failed to get auto bit rate parameters (%d) (%p)!", __func__, func_ret, abr_para);
    return LHDCV5_FRET_ERROR;
  }

  switch (enc_type)
  {
  case LHDCV5_ENC_TYPE_LHDCV5:

    func_ret = lhdcv5_encoder_adjust_bitrate (handle, abr_para, queueLen);

    if (func_ret != LHDCV5_FRET_SUCCESS)
    {
      ALOGW ("%s: Failed to adjust auto bit rate (%d)!", __func__, func_ret);
      return LHDCV5_FRET_ERROR;
    }
    break;

  default:
    ALOGW ("%s: Invalid encode type (%d)!", __func__, enc_type);
    return LHDCV5_FRET_INVALID_CODEC;
  }

  return LHDCV5_FRET_SUCCESS;
}


//----------------------------------------------------------------
// lhdcv5BT_set_ext_func_state ()
//
// Set the ext. function state (AR, JAS, Meta, LARC)
//	Parameter
//		handle: a pointer to the resource allocated and is returned 
//				by function lhdcBT_get_handle ()
//		field: specify the ext. function
//		enabled: ext. function is set to “enabled” (true) or “disabled” (false)
//		priv: a pointer to the exra data needed for ext. function
//		priv_data_len: number of bytes of the extra data
//	Return
//		LHDCV5_FRET_SUCCESS: succeed to set specified ext. function to “enabled” (true) or “disabled” (false)
//		Other: fail to set .
//----------------------------------------------------------------

int32_t lhdcv5BT_set_ext_func_state
(
    HANDLE_LHDCV5_BT 	handle,
    LHDCV5_EXT_FUNC_T	field,
    bool 				enabled,
    void 				* priv,
    uint32_t 			priv_data_len
)
{
  int32_t	func_ret = LHDCV5_FRET_SUCCESS;

  if (handle == NULL)
  {
    ALOGW ("%s: Handle is NULL!", __func__);
    return LHDCV5_FRET_INVALID_HANDLE_CB;
  }

  if ((field < LHDCV5_EXT_FUNC_AR) ||
      (field >= LHDCV5_EXT_FUNC_INVALID))
  {
    ALOGW ("%s: Invalid ext. func. field (%d)!", __func__, field);
    return LHDCV5_FRET_INVALID_INPUT_PARAM;
  }

  func_ret = lhdcv5_util_set_ext_func_state (handle,
      (LHDCV5_EXT_FUNC_T) field,
      enabled,
      priv,
      priv_data_len,	// data length MUST be 8 for META
      LHDCV5_META_LOOP_CNT_STD);

  if (func_ret != LHDCV5_FRET_SUCCESS)
  {
    ALOGW ("%s: failed to set ext. function state (%d)!", __func__, func_ret);
    return LHDCV5_FRET_ERROR;
  }

  return LHDCV5_FRET_SUCCESS;
}


//----------------------------------------------------------------
// lhdcv5BT_init_encoder ()
//
// Initialize LHDC 5.0 encoder
//	Parameter
//		handle: a pointer to the resource allocated and is returned 
//				by function lhdcBT_get_handle ()
//		sampling_freq: sample frequency
//		bit_per_sample: bits per sample
//		bitrate_inx: bit rate index
//		mtu: BT A2DP MTU 
//		interval: interval: period of time triggering LHDC 5.0 encoding in ms
//	Return
//		LHDCV5_FRET_SUCCESS: succeed to initialize 
//		Other: fail to initialize.
//----------------------------------------------------------------
int32_t lhdcv5BT_init_encoder
(
    HANDLE_LHDCV5_BT 	handle,
    uint32_t 			sampling_freq,
    uint32_t 			bits_per_sample,
    uint32_t 			bitrate_inx,
    uint32_t 			mtu,
    uint32_t 			interval
) 
{
  int32_t		func_ret = LHDCV5_FRET_SUCCESS;

  if (handle == NULL)
  {
    ALOGW ("%s: Handle is NULL!", __func__);
    return LHDCV5_FRET_INVALID_HANDLE_CB;
  }

  if ((sampling_freq != LHDCV5_SR_44100HZ) &&
      (sampling_freq != LHDCV5_SR_48000HZ) &&
      (sampling_freq != LHDCV5_SR_96000HZ) &&
      (sampling_freq != LHDCV5_SR_192000HZ))
  {
    ALOGW ("%s: Invalid sampling frequency (%u)!", __func__, sampling_freq);
    return LHDCV5_FRET_INVALID_INPUT_PARAM;
  }

  if ((bits_per_sample != LHDCV5BT_SMPL_FMT_S16) &&
      (bits_per_sample != LHDCV5BT_SMPL_FMT_S24) &&
      (bits_per_sample != LHDCV5BT_SMPL_FMT_S32))
  {
    ALOGW ("%s: Invalid bits per sample (%u)!", __func__, bits_per_sample);
    return LHDCV5_FRET_INVALID_INPUT_PARAM;
  }

  if ((bitrate_inx < LHDCV5_QUALITY_LOW0) ||
      (bitrate_inx > LHDCV5_QUALITY_AUTO))
  {
    ALOGW ("%s: Invalid bit rate (index) (%d)!", __func__, bitrate_inx);
    return LHDCV5_FRET_INVALID_INPUT_PARAM;
  }


  func_ret = lhdcv5_util_init_encoder (handle,
      (unsigned int) sampling_freq,
      (unsigned int) bits_per_sample,
      (unsigned int) bitrate_inx,
      (unsigned int) LHDCV5_FRAME_5MS,
      (unsigned int) mtu,
      (unsigned int) interval);

  if (func_ret != LHDCV5_FRET_SUCCESS)
  {
    ALOGW ("%s: Failed to init LHDC 5.0 encoder (%d)!", __func__, func_ret);
    return LHDCV5_FRET_ERROR;
  }

  return LHDCV5_FRET_SUCCESS;
}


//----------------------------------------------------------------
// lhdcv5BT_get_block_Size ()
//
// Get number of samples per block for LHDC 5.0 encoder
//	Parameter
//		handle: a pointer to the resource allocated and is returned by function lhdcv5Bt_get_handle ()
//		samples_per_frame: number of samples per block returned
//	Return
//		LHDCV5_FRET_SUCCESS: succeed to get number of samples per block 
//		Other: fail to get number of samples per block.
//----------------------------------------------------------------
int32_t lhdcv5BT_get_block_Size
(
    HANDLE_LHDCV5_BT	handle,
    uint32_t			* samples_per_frame
)
{
  int32_t		func_ret = LHDCV5_FRET_SUCCESS;


  if (handle == NULL)
  {
    ALOGW ("%s: Handle is NULL!", __func__);
    return LHDCV5_FRET_INVALID_HANDLE_CB;
  }

  if (samples_per_frame == NULL)
  {
    ALOGW ("%s: Input parameter is NULL!", __func__);
    return LHDCV5_FRET_INVALID_INPUT_PARAM;
  }

  func_ret = lhdcv5_util_get_block_Size (handle, samples_per_frame);

  if ((func_ret != LHDCV5_FRET_SUCCESS) || ((*samples_per_frame) <= 0))
  {
    ALOGW ("%s: Failed to get block size (%d) (%d)!", __func__, func_ret, *samples_per_frame);
    return LHDCV5_FRET_ERROR;
  }

  return LHDCV5_FRET_SUCCESS;
}


//----------------------------------------------------------------
// lhdcv5BT_encode ()
//
// Encode pcm samples by LHDC 5.0
//	Parameter
//		handle: a pointer to the resource allocated and is returned 
//				by function lhdcBT_get_handle ()
//		p_in_pcm: a pointer to a buffer contains PCM samples for encoding
//		p_out_buf: a pointer to a buffer to put encoded stream
//		out_buf_bytes: output buffer's size (in byte)
//		p_out_bytes: a pointer to number of bytes of encoded stream in buffer
//		p_out_frames: a pointer to number of frames of encoded stream in buffer
//	Return
//		LHDCV5_FRET_SUCCESS: succeed to encode pcm samples
//		Other: fail to encode pcm samples
//----------------------------------------------------------------
int32_t lhdcv5BT_encode
(
    HANDLE_LHDCV5_BT 	handle,
    void				* p_in_pcm,
    uint32_t			pcm_bytes,
    uint8_t				* p_out_buf,
    uint32_t			out_buf_bytes,
    uint32_t 			* p_out_bytes,
    uint32_t 			* p_out_frames
)
{
  int32_t		func_ret = LHDCV5_FRET_SUCCESS;

  if (handle == NULL)
  {
    ALOGW ("%s: Handle is NULL!", __func__);
    return LHDCV5_FRET_INVALID_HANDLE_CB;
  }

  if ((p_in_pcm == NULL) || (p_out_buf == NULL) ||
      (p_out_bytes == NULL) || (p_out_frames == NULL))
  {
    ALOGW ("%s: input parameter is NULL!", __func__);
    return LHDCV5_FRET_INVALID_INPUT_PARAM;
  }

  func_ret = lhdcv5_util_enc_process (handle,
      p_in_pcm,
      pcm_bytes,
      p_out_buf,
      out_buf_bytes,
      p_out_bytes,
      p_out_frames);

  if (func_ret != LHDCV5_FRET_SUCCESS)
  {
    ALOGW ("%s: Failed to encode pcm samples (%d)!", __func__, func_ret);
    return LHDCV5_FRET_ERROR;
  }

  return LHDCV5_FRET_SUCCESS;
}



/*
 ******************************************************************
 Extend API functions group
 ******************************************************************
 */
static bool lhdcBT_code_ver_wrap
(
    unsigned char *pucConfig,
    unsigned int *exFuncCode,
    unsigned int *exFuncVer
)
{
  *exFuncVer = (((unsigned int) pucConfig[3]) & ((unsigned int)0xff)) |
      ((((unsigned int) pucConfig[2]) & ((unsigned int)0xff)) << 8)  |
      ((((unsigned int) pucConfig[1]) & ((unsigned int)0xff)) << 16) |
      ((((unsigned int) pucConfig[0]) & ((unsigned int)0xff)) << 24);

  *exFuncCode = (((unsigned int) pucConfig[7]) & ((unsigned int)0xff)) |
      ((((unsigned int) pucConfig[6]) & ((unsigned int)0xff)) << 8)  |
      ((((unsigned int) pucConfig[5]) & ((unsigned int)0xff)) << 16) |
      ((((unsigned int) pucConfig[4]) & ((unsigned int)0xff)) << 24);

  return true;
}

//
// META, JAS
//
//----------------------------------------------------------------
// lhdcBT_set_cfg_meta_v1 ()
//
// Set configuration for meta data
//	Parameter
//		handle: a pointer to the resource allocated and is returned 
//				by function lhdcBT_get_handle ()
//		userConfig: a pointer to configuration
//		configLen: number of bytes of configuration
//	Return
//		EXTEND_FUNC_RET_OK: succeed to set configuration  
//		Other: fail to set configuration.
//----------------------------------------------------------------
static int lhdcBT_set_cfg_meta_v1
(
    HANDLE_LHDCV5_BT 	handle,
    const char			*userConfig,
    const int 			configLen
) 
{
  PST_LHDC_SET_META pset_meta = (PST_LHDC_SET_META) userConfig;
  unsigned char	* pmeta_metadata = NULL;
  int				func_ret = LHDCV5_FRET_SUCCESS;

  if (handle == NULL)
  {
    ALOGW ("(LHDC-exAPI) %s: Handle is NULL (%p)!",  __func__, handle);
    return EXTEND_FUNC_RET_INVALID_HANDLE;
  }

  if (userConfig == NULL)
  {
    ALOGW ("(LHDC-exAPI) %s: Config is NULL (%p)!",  __func__, userConfig);
    return EXTEND_FUNC_RET_INVALID_PARAMETER;
  }

  if (configLen < (int) sizeof (ST_LHDC_SET_META))
  {
    // Buffer is to small
    ALOGW ("(LHDC-exAPI) %s: Buffer too small (%d)!",  __func__, configLen);
    return EXTEND_FUNC_RET_BUF_UNDERRUN;
  }

  if (pset_meta->meta_ver != META_ST_VER_V2)
  {
    ALOGW ("(LHDC-exAPI) %s: version is not match (%d)!",  __func__, pset_meta->meta_ver);
    return EXTEND_FUNC_RET_INVALID_PARAMETER;
  }

  if (pset_meta->meta_mem_size != (int) sizeof (ST_LHDC_SET_META))
  {
    ALOGW("(LHDC-exAPI) %s: META data size is  not match (%d)!", __func__, pset_meta->meta_mem_size);
    return EXTEND_FUNC_RET_INVALID_PARAMETER;
  }

  if (configLen < ((int) pset_meta->meta_metadata_length) + ((int) sizeof (ST_LHDC_SET_META)))
  {
    ALOGW("(LHDC-exAPI) %s: cfg size too small (%d)!", __func__, configLen);
    return EXTEND_FUNC_RET_INVALID_PARAMETER;
  }

  pmeta_metadata = (uint8_t*) (pset_meta + 1);

  func_ret = lhdcv5_util_set_ext_func_state (handle,
      LHDCV5_EXT_FUNC_META,
      pset_meta->meta_enable,
      pmeta_metadata,
      pset_meta->meta_metadata_length,
      pset_meta->meta_set);

  if (func_ret != LHDCV5_FRET_SUCCESS)
  {
    ALOGW ("(LHDC-exAPI) %s: Fail to set META data (%d)!",  __func__, func_ret);
    return EXTEND_FUNC_RET_ERROR;
  }

  return EXTEND_FUNC_RET_OK;
}


//----------------------------------------------------------------
// lhdcBT_get_cfg_meta_v1 ()
//
// Get configuration for meta data
//	Parameter
//		handle: a pointer to the resource allocated and is returned 
//				by function lhdcBT_get_handle ()
//		userConfig: a pointer to configuration
//		configLen: number of bytes of configuration
//	Return
//		EXTEND_FUNC_RET_OK: succeed to get configuration   
//		Other: fail to get configuration.
//----------------------------------------------------------------
static int lhdcBT_get_cfg_meta_v1
(
    HANDLE_LHDCV5_BT 	handle,
    char				* userConfig,
    const int 			configLen
) 
{
  PST_LHDC_GET_META pget_meta = (PST_LHDC_GET_META) userConfig;
  bool 		jas_enabled = false;
  bool		meta_enabled = false;
  int			func_ret = LHDCV5_FRET_SUCCESS;


  if (handle == NULL)
  {
    ALOGW ("(LHDC-exAPI) %s: Handle is NULL (%p)!",  __func__, handle);
    return EXTEND_FUNC_RET_INVALID_HANDLE;
  }

  if (userConfig == NULL)
  {
    ALOGW ("(LHDC-exAPI) %s: User Config is NULL (%p)!",  __func__, userConfig);
    return EXTEND_FUNC_RET_INVALID_PARAMETER;
  }

  if (configLen < (int) sizeof (ST_LHDC_GET_META))
  {
    // Buffer is to small
    ALOGW ("(LHDC-exAPI) %s: Buffer too small (%d)!",  __func__, configLen);
    return EXTEND_FUNC_RET_BUF_UNDERRUN;
  }

  func_ret = lhdcv5_util_get_ext_func_state (handle, LHDCV5_EXT_FUNC_META, &meta_enabled);
  if (func_ret != LHDCV5_FRET_SUCCESS)
  {
    ALOGW ("(LHDC-exAPI) %s: Fail to get META flag (%d)!",  __func__, func_ret);
    return EXTEND_FUNC_RET_ERROR;
  }
  ALOGW ("(LHDC-exAPI) %s: LHDCV5_EXT_FUNC_META: meta_enabled=%d",  __func__, meta_enabled);

  func_ret = lhdcv5_util_get_ext_func_state (handle, LHDCV5_EXT_FUNC_JAS, &jas_enabled);
  if (func_ret != LHDCV5_FRET_SUCCESS)
  {
    ALOGW ("(LHDC-exAPI) %s: Fail to get JAS flag (%d)!",  __func__, func_ret);
    return EXTEND_FUNC_RET_ERROR;
  }
  ALOGW ("(LHDC-exAPI) %s: LHDCV5_EXT_FUNC_JAS: jas_enabled=%d",  __func__, jas_enabled);

  pget_meta->meta_ver = META_ST_VER_V2;
  pget_meta->meta_mem_size = (int) sizeof (ST_LHDC_GET_META);
  pget_meta->meta_st = meta_enabled ? 0x03 : 0x01;  // Get current frame include metadata or not
  pget_meta->jas_status = jas_enabled ? 1 : 0;

  return EXTEND_FUNC_RET_OK;
}


//
// AR Function
//
//----------------------------------------------------------------
// lhdcBT_set_data_gyro_2d_v1 ()
//
// Set data for gyro (x, y)
//	Parameter
//		handle: a pointer to the resource allocated and is returned 
//				by function lhdcBT_get_handle ()
//		userData: a pointer to data
//		dataLen: number of bytes of data
//	Return
//		0: succeed to set data  
//		Other: fail to set data.
//----------------------------------------------------------------
static int lhdcBT_set_data_gyro_2d_v1
(
    HANDLE_LHDCV5_BT 	handle,
    const char 			* userData,
    const int 			dataLen
) 
{
  PST_LHDC_AR_GYRO pargyro = (PST_LHDC_AR_GYRO) userData;
  int 		func_ret = LHDCV5_FRET_SUCCESS;

  if (handle == NULL)
  {
    ALOGW ("(LHDC-exAPI) %s: Handle is NULL (%p)!",  __func__, handle);
    return EXTEND_FUNC_RET_INVALID_HANDLE;
  }

  if (userData == NULL)
  {
    ALOGW ("(LHDC-exAPI) %s: User Data is NULL (%p)!",  __func__, userData);
    return EXTEND_FUNC_RET_INVALID_PARAMETER;
  }

  if (dataLen < (int) sizeof (ST_LHDC_AR_GYRO))
  {
    // Buffer is to small
    ALOGW ("(LHDC-exAPI) %s: Buffer too small (%d)!",  __func__, dataLen);
    return EXTEND_FUNC_RET_BUF_UNDERRUN;
  }


  ALOGV ("(LHDC-exAPI) %s: set coordinate[x:%d y:%d z:%d]",  __func__,
      pargyro->world_coordinate_x,
      pargyro->world_coordinate_y,
      pargyro->world_coordinate_z);

  func_ret = lhdcv5_util_ar_set_gyro_pos (handle,
      pargyro->world_coordinate_x,
      pargyro->world_coordinate_y,
      pargyro->world_coordinate_z);

  if (func_ret != LHDCV5_FRET_SUCCESS)
  {
    ALOGD ("(LHDC-exAPI) %s: Fail to set gyro's position[x:%d y:%d z:%d] for AR (%d)!",  __func__,
        pargyro->world_coordinate_x,
        pargyro->world_coordinate_y,
        pargyro->world_coordinate_z,
        func_ret);
    return EXTEND_FUNC_RET_ERROR;
  }

  return EXTEND_FUNC_RET_OK;
}


//----------------------------------------------------------------
// lhdcBT_set_cfg_ar_v3 ()
//
// Set configuration for AR
//	Parameter
//		handle: a pointer to the resource allocated and is returned 
//				by function lhdcBT_get_handle ()
//		userConfig: a pointer to configuration
//		configLen: number of bytes of configuration
//	Return
//		0: succeed to set configuration  
//		Other: fail to set configuration.
//----------------------------------------------------------------
static int lhdcBT_set_cfg_ar_v3
(
    HANDLE_LHDCV5_BT 	handle,
    const char 			* userConfig,
    const int 			configLen
) 
{
  PST_LHDC_AR pset_ar_cfg = (PST_LHDC_AR) userConfig;
  int 		func_ret = LHDCV5_FRET_SUCCESS;

  if (handle == NULL)
  {
    ALOGW("(LHDC-exAPI) %s: Handle is NULL (%p)!",  __func__, handle);
    return EXTEND_FUNC_RET_INVALID_HANDLE;
  }

  if (userConfig == NULL)
  {
    ALOGW("(LHDC-exAPI) %s: User Config is NULL (%p)!",  __func__, userConfig);
    return EXTEND_FUNC_RET_INVALID_PARAMETER;
  }

  if (configLen < (int) sizeof (ST_LHDC_AR))
  {
    // Buffer is to small
    ALOGW("(LHDC-exAPI) %s: Buffer too small (%d)!",  __func__, configLen);
    return EXTEND_FUNC_RET_BUF_UNDERRUN;
  }

  ALOGD("(LHDC-exAPI) %s: ver(%d) size(%d) app_ar_enabled(%d) Ch_Pos(%d %d %d %d %d %d)"
      , __func__,
      pset_ar_cfg->ver,
      pset_ar_cfg->size,
      pset_ar_cfg->app_ar_enabled,
      pset_ar_cfg->Ch1_Pos, pset_ar_cfg->Ch2_Pos, pset_ar_cfg->Ch2_Pos,
      pset_ar_cfg->Ch4_Pos, pset_ar_cfg->Ch5_Pos, pset_ar_cfg->Ch6_Pos);

  ALOGD("(LHDC-exAPI) %s: PreGain(L:R): ch1[%f:%f] ch2[%f:%f] ch3[%f:%f] ch4[%f:%f] ch5[%f:%f] ch6[%f:%f]"
      , __func__,
      pset_ar_cfg->Ch1_L_PreGain, pset_ar_cfg->Ch1_R_PreGain,
      pset_ar_cfg->Ch2_L_PreGain, pset_ar_cfg->Ch2_R_PreGain,
      pset_ar_cfg->Ch3_L_PreGain, pset_ar_cfg->Ch3_R_PreGain,
      pset_ar_cfg->Ch4_L_PreGain, pset_ar_cfg->Ch4_R_PreGain,
      pset_ar_cfg->Ch5_L_PreGain, pset_ar_cfg->Ch5_R_PreGain,
      pset_ar_cfg->Ch6_L_PreGain, pset_ar_cfg->Ch6_R_PreGain);

  ALOGD("(LHDC-exAPI) %s: PostGain: ch1[%f] ch2[%f] ch3[%f] ch4[%f] ch5[%f] ch6[%f]"
      , __func__,
      pset_ar_cfg->Ch1_PostGain, pset_ar_cfg->Ch2_PostGain,
      pset_ar_cfg->Ch3_PostGain, pset_ar_cfg->Ch4_PostGain,
      pset_ar_cfg->Ch5_PostGain, pset_ar_cfg->Ch6_PostGain);

  ALOGD("(LHDC-exAPI) %s: Dry_Val(%f) Wet_Val(%f)" , __func__,
      pset_ar_cfg->Dry_Val, pset_ar_cfg->Wet_Val);

  ALOGD("(LHDC-exAPI) %s: Dis[%f %f %f %f %f] Rev[%f %f %f %f %f]" , __func__,
      pset_ar_cfg->Dis_1, pset_ar_cfg->Dis_2, pset_ar_cfg->Dis_3, pset_ar_cfg->Dis_4, pset_ar_cfg->Dis_5,
      pset_ar_cfg->Rev_1, pset_ar_cfg->Rev_2, pset_ar_cfg->Rev_3, pset_ar_cfg->Rev_4, pset_ar_cfg->Rev_5);

  ALOGD("(LHDC-exAPI) %s: Rev_gain(%f) ThreeD_gain(%f)" , __func__,
      pset_ar_cfg->Rev_gain, pset_ar_cfg->ThreeD_gain);

  ALOGD ("(LHDC-exAPI) %s: to set AR enable: %d",  __func__, pset_ar_cfg->app_ar_enabled);
  func_ret = lhdcv5_util_ar_set_cfg (handle,
      &pset_ar_cfg->Ch1_Pos,
      6,
      &pset_ar_cfg->Ch1_L_PreGain,
      32,
      pset_ar_cfg->app_ar_enabled);

  if (func_ret != LHDCV5_FRET_SUCCESS)
  {
    ALOGW ("(LHDC-exAPI) %s: func_ret %d",  __func__, func_ret);
    return EXTEND_FUNC_RET_ERROR;
  }

  return EXTEND_FUNC_RET_OK;
}


//----------------------------------------------------------------
// lhdcBT_get_cfg_ar_v1 ()
//
// Get configuration for AR
//	Parameter
//		handle: a pointer to the resource allocated and is returned 
//				by function lhdcBT_get_handle ()
//		userConfig: a pointer to configuration
//		configLen: number of bytes of configuration
//	Return
//		EXTEND_FUNC_RET_OK: succeed to get configuration   
//		Other: fail to get configuration.
//----------------------------------------------------------------
static int lhdcBT_get_cfg_ar_v1
(
    HANDLE_LHDCV5_BT 	handle,
    char 				* userConfig,
    const int 			configLen
) 
{
  PST_LHDC_AR pset_ar_cfg = (PST_LHDC_AR) userConfig;
  int 		func_ret = LHDCV5_FRET_SUCCESS;

  if (handle == NULL)
  {
    ALOGW ("(LHDC-exAPI) %s: Handle is NULL (%p)!",  __func__, handle);
    return EXTEND_FUNC_RET_INVALID_HANDLE;
  }

  if (userConfig == NULL)
  {
    ALOGW ("(LHDC-exAPI) %s: User Config is NULL (%p)!",  __func__, userConfig);
    return EXTEND_FUNC_RET_INVALID_PARAMETER;
  }

  if (configLen < (int) sizeof (ST_LHDC_AR))
  {
    // Buffer is to small
    ALOGW ("(LHDC-exAPI) %s: Buffer too small (%d)!",  __func__, configLen);
    return EXTEND_FUNC_RET_BUF_UNDERRUN;
  }

  func_ret = lhdcv5_util_ar_get_cfg (handle,
      &pset_ar_cfg->Ch1_Pos,
      6,
      &pset_ar_cfg->Ch1_L_PreGain,
      32);

  if (func_ret != LHDCV5_FRET_SUCCESS)
  {
    ALOGW ("(LHDC-exAPI) %s: Fail to get AR config (%d)!",  __func__, func_ret);
    return EXTEND_FUNC_RET_ERROR;
  }

  return EXTEND_FUNC_RET_OK;
}

// 1. API -- Set User Config (Extend)
//----------------------------------------------------------------
// lhdcv5BT_set_user_exconfig ()
//
// Set configuration
//	Parameter
//		handle: a pointer to the resource allocated and is returned 
//				by function lhdcBT_get_handle ()
//		userConfig: a pointer to configuration
//		clen: number of bytes of configuration
//	Return
//		EXTEND_FUNC_RET_OK: succeed to set configuration   
//		Other: fail to set configuration.
//----------------------------------------------------------------
int lhdcv5BT_set_user_exconfig
(
    HANDLE_LHDCV5_BT 	handle,
    const char			* userConfig,
    const int 			clen
) 
{
  unsigned char 	* pucConfig = (unsigned char *) userConfig;
  unsigned int 	exFuncVer;
  unsigned int 	exFuncCode;
  int 			func_ret = EXTEND_FUNC_RET_OK;

  if (handle == NULL)
  {
    ALOGW ("(LHDC-exAPI) %s: Handle is NULL (%p)!",  __func__, handle);
    return EXTEND_FUNC_RET_INVALID_HANDLE;
  }

  if (userConfig == NULL)
  {
    ALOGW ("(LHDC-exAPI) %s: User Config is NULL (%p)!",  __func__, userConfig);
    return EXTEND_FUNC_RET_INVALID_PARAMETER;
  }

  if (clen < (int) EXTEND_FUNC_MIN_BUFFER_LEN)
  {
    // Buffer is to small
    ALOGW ("(LHDC-exAPI) %s: Buffer too small (%d)!",  __func__, clen);
    return EXTEND_FUNC_RET_BUF_UNDERRUN;
  }

  lhdcBT_code_ver_wrap(pucConfig, &exFuncCode , &exFuncVer);

  switch (exFuncCode)
  {
  case EXTEND_FUNC_CODE_SET_CONFIG_META:

    switch (exFuncVer)
    {
    case EXTEND_FUNC_VER_SET_CONFIG_META_V1:
      ALOGD ("(LHDC-exAPI) %s: SET_CONFIG_META",  __func__);
      func_ret = lhdcBT_set_cfg_meta_v1 (handle, userConfig, clen);

      if (func_ret != LHDCV5_FRET_SUCCESS)
      {
        ALOGW ("(LHDC-exAPI) %s: Fail to set META data (%d)!",  __func__, func_ret);
        return EXTEND_FUNC_RET_ERROR;
      }
      break;

    default:
      ALOGW ("(LHDC-exAPI) %s: Invalid Ex. Function Version (0x%X)!",  __func__, exFuncVer);
      return EXTEND_FUNC_RET_VERSION_NOT_SUPPORT;
    }
    break;

    case EXTEND_FUNC_CODE_SET_CONFIG_AR:
      switch (exFuncVer)
      {
      case EXTEND_FUNC_VER_SET_CONFIG_AR_V3:
        ALOGD ("(LHDC-exAPI) %s: SET_CONFIG_AR",  __func__);
        func_ret = lhdcBT_set_cfg_ar_v3 (handle, userConfig, clen);

        if (func_ret != LHDCV5_FRET_SUCCESS)
        {
          ALOGW ("(LHDC-exAPI) %s: Fail to set AR config (%d)!",  __func__, func_ret);
          return EXTEND_FUNC_RET_ERROR;
        }
        break;

      default:
        ALOGW ("(LHDC-exAPI) %s: Invalid Ex. Function Version (0x%X)!",  __func__, exFuncVer);
        return EXTEND_FUNC_RET_VERSION_NOT_SUPPORT;
      }
      break;

      default:
        ALOGW ("(LHDC-exAPI) %s: Invalid Ex. Function Code (0x%X)!",  __func__, exFuncCode);
        return EXTEND_FUNC_RET_FUNC_NOT_SUPPORT;
  } // switch (exFuncCode)

  return EXTEND_FUNC_RET_OK;
}


// 2. API -- Get User Config (Extend)
//----------------------------------------------------------------
// lhdcv5BT_get_user_exconfig ()
//
// Get configuration
//	Parameter
//		handle: a pointer to the resource allocated and is returned 
//				by function lhdcBT_get_handle ()
//		userConfig: a pointer to configuration
//		clen: number of bytes of configuration
//	Return
//		EXTEND_FUNC_RET_OK: succeed to get configuration   
//		Other: fail to get configuration.
//----------------------------------------------------------------

int lhdcv5BT_get_user_exconfig
(
    HANDLE_LHDCV5_BT 	handle,
    char				* userConfig,
    int 				clen
) 
{
  unsigned char 	* pucConfig = (unsigned char *) userConfig;
  unsigned int 	exFuncVer;
  unsigned int 	exFuncCode;
  int 			func_ret = EXTEND_FUNC_RET_OK;

  if (handle == NULL)
  {
    ALOGW ("(LHDC-exAPI) %s: Handle is NULL (%p)!",  __func__, handle);
    return EXTEND_FUNC_RET_INVALID_HANDLE;
  }

  if (userConfig == NULL)
  {
    ALOGW ("(LHDC-exAPI) %s: User Config is NULL (%p)!",  __func__, userConfig);
    return EXTEND_FUNC_RET_INVALID_PARAMETER;
  }

  if (clen < (int)EXTEND_FUNC_MIN_BUFFER_LEN)
  {
    // Buffer is to small
    ALOGW ("(LHDC-exAPI) %s: Buffer too small (%d)!",  __func__, clen);
    return EXTEND_FUNC_RET_BUF_UNDERRUN;
  }

  lhdcBT_code_ver_wrap(pucConfig, &exFuncCode , &exFuncVer);

  switch (exFuncCode)
  {
  case EXTEND_FUNC_CODE_GET_CONFIG_META:

    switch (exFuncVer)
    {
    case EXTEND_FUNC_VER_GET_CONFIG_META_V1:
      ALOGD ("(LHDC-exAPI) %s: GET_CONFIG_META",  __func__);
      func_ret = lhdcBT_get_cfg_meta_v1 (handle, userConfig, clen);

      if (func_ret != LHDCV5_FRET_SUCCESS)
      {
        ALOGW ("(LHDC-exAPI) %s: Fail to get META data (%d)!",  __func__, func_ret);
        return EXTEND_FUNC_RET_ERROR;
      }
      break;

    default:
      ALOGW ("(LHDC-exAPI) %s: Invalid Ex. Function Version (0x%X)!",  __func__, exFuncVer);
      return EXTEND_FUNC_RET_VERSION_NOT_SUPPORT;
    }
    break;

    case EXTEND_FUNC_CODE_GET_CONFIG_AR:

      switch (exFuncVer)
      {
      case EXTEND_FUNC_VER_GET_CONFIG_AR_V1:
        ALOGD ("(LHDC-exAPI) %s: GET_CONFIG_AR",  __func__);
        func_ret = lhdcBT_get_cfg_ar_v1 (handle, userConfig, clen);

        if (func_ret != LHDCV5_FRET_SUCCESS)
        {
          ALOGW ("(LHDC-exAPI) %s: Fail to get AR config (%d)!",  __func__, func_ret);
          return EXTEND_FUNC_RET_ERROR;
        }
        break;

      default:
        ALOGW ("(LHDC-exAPI) %s: Invalid Ex. Function Version (0x%X)!",  __func__, exFuncVer);
        return EXTEND_FUNC_RET_VERSION_NOT_SUPPORT;
      }
      break;

      default:
        ALOGW ("(LHDC-exAPI) %s: Invalid Ex. Function Code (0x%X)!",  __func__, exFuncCode);
        return EXTEND_FUNC_RET_FUNC_NOT_SUPPORT;
  } // switch (exFuncCode)


  return EXTEND_FUNC_RET_OK;
}


// 3. API -- Set User Data (Extend)
//----------------------------------------------------------------
// lhdcv5BT_set_user_exdata ()
//
// Set data
//	Parameter
//		handle: a pointer to the resource allocated and is returned 
//				by function lhdcBT_get_handle ()
//		userData: a pointer to data
//		clen: number of bytes of data
//	Return
//		EXTEND_FUNC_RET_OK: succeed to set data   
//		Other: fail to set data.
//----------------------------------------------------------------
void lhdcv5BT_set_user_exdata
(
    HANDLE_LHDCV5_BT 	handle,
    const char			* userData,
    const int 			clen
) 
{
  unsigned char 	* pucData = (unsigned char *) userData;
  unsigned int 	exFuncVer;
  unsigned int 	exFuncCode;
  int 			func_ret = EXTEND_FUNC_RET_OK;

  if (handle == NULL)
  {
    ALOGW ("(LHDC-exAPI) %s: Handle is NULL (%p)!",  __func__, handle);
    return; // EXTEND_FUNC_RET_INVALID_HANDLE;
  }

  if (userData == NULL)
  {
    ALOGW ("(LHDC-exAPI) %s: User Data is NULL (%p)!",  __func__, userData);
    return; // EXTEND_FUNC_RET_INVALID_PARAMETER;
  }

  if (clen < (int) EXTEND_FUNC_MIN_BUFFER_LEN)
  {
    // Buffer is to small
    ALOGW ("(LHDC-exAPI) %s: Buffer is too small (%d)!",  __func__, clen);
    return;
  }

  lhdcBT_code_ver_wrap(pucData, &exFuncCode , &exFuncVer);

  switch (exFuncCode)
  {
  case EXTEND_FUNC_CODE_SET_DATA_GYRO2D:

    switch (exFuncVer)
    {
    case EXTEND_FUNC_VER_SET_DATA_GYRO2D_V1:
      ALOGD ("(LHDC-exAPI) %s: SET_DATA_GYRO",  __func__);
      func_ret = lhdcBT_set_data_gyro_2d_v1 (handle, userData, clen);

      if (func_ret != LHDCV5_FRET_SUCCESS)
      {
        ALOGW ("(LHDC-exAPI) %s: Fail to get gyro's data (%d)!",  __func__, func_ret);
        return;
      }
      break;

    default:
      ALOGW ("(LHDC-exAPI) %s: Invalid Ex. Function Version (0x%X)!",  __func__, exFuncVer);
      break; // EXTEND_FUNC_RET_FUNC_NOT_SUPPORT;
    }
    break;

    default:
      ALOGW ("(LHDC-exAPI) %s: Invalid Ex. Function Code (0x%X)!",  __func__, exFuncCode);
      break;
  } // switch (exFuncCode)

  return;
}


// 4. API -- Get Version
//----------------------------------------------------------------
// lhdcv5BT_get_user_exApiver ()
//
// Get version
//	Parameter
//		handle: a pointer to the resource allocated and is returned 
//				by function lhdcBT_get_handle ()
//		version: a pointer to version
//		clen: number of bytes of version
//	Return
//		0: succeed to set data   
//		Other: fail to set data.
//----------------------------------------------------------------
int lhdcv5BT_get_user_exApiver
(
    HANDLE_LHDCV5_BT 	handle,
    char 				* version,
    int 				clen
) 
{
  unsigned char 	* pucApiVer = (unsigned char *) version;
  unsigned int 	exFuncVer = 0;
  unsigned int 	exFuncCode = 0;

  if (handle == NULL)
  {
    ALOGW ("(LHDC-exAPI) %s: Handle is NULL (%p)!",  __func__, handle);
    return EXTEND_FUNC_RET_INVALID_HANDLE;
  }

  if (version == NULL)
  {
    ALOGW ("(LHDC-exAPI) %s: API verion pointer is NULL (%p)!",  __func__, version);
    return EXTEND_FUNC_RET_INVALID_PARAMETER;
  }

  if (clen < (int)EXTEND_FUNC_MIN_BUFFER_LEN)
  {
    // Buffer is to small
    ALOGW ("(LHDC-exAPI) %s: Buffer too small (%d)!",  __func__, clen);
    return EXTEND_FUNC_RET_BUF_UNDERRUN;
  }

  lhdcBT_code_ver_wrap(pucApiVer, &exFuncCode , &exFuncVer);

  switch (exFuncCode)
  {
  // Config APIs:
  case EXTEND_FUNC_CODE_SET_CONFIG_META:
    exFuncVer = EXTEND_FUNC_VER_SET_CONFIG_META_V1;
    break;

  case EXTEND_FUNC_CODE_SET_CONFIG_AR:
    exFuncVer = EXTEND_FUNC_VER_SET_CONFIG_AR_V3;
    break;

  case EXTEND_FUNC_CODE_GET_CONFIG_META:
    exFuncVer = EXTEND_FUNC_VER_GET_CONFIG_META_V1;
    break;

  case EXTEND_FUNC_CODE_GET_CONFIG_AR:
    exFuncVer = EXTEND_FUNC_VER_GET_CONFIG_AR_V1;
    break;

    // Data APIs:
  case EXTEND_FUNC_CODE_SET_DATA_GYRO2D:
    exFuncVer = EXTEND_FUNC_VER_SET_DATA_GYRO2D_V1;
    break;

    // A2DP codec config APIs:
  case EXTEND_FUNC_CODE_GET_SPECIFIC:
    exFuncVer = EXTEND_FUNC_VER_GET_SPECIFIC_V2;
    break;

  default:
    ALOGW ("(LHDC-exAPI) %s: Invalid Ex. Function Code (0x%X)!",  __func__, exFuncCode);
    return EXTEND_FUNC_RET_FUNC_NOT_SUPPORT;
  } // switch (exFuncCode)

  // fill version of target API
  pucApiVer[3] = (unsigned char) (exFuncVer & ((unsigned int)0xff));
  pucApiVer[2] = (unsigned char) ((exFuncVer >> 8) & ((unsigned int)0xff));
  pucApiVer[1] = (unsigned char) ((exFuncVer >> 16) & ((unsigned int)0xff));
  pucApiVer[0] = (unsigned char) ((exFuncVer >> 24) & ((unsigned int)0xff));

  ALOGD ("(LHDC-exAPI) %s: APICode:[%02X %02X %02X %02X] Ver:[%02X %02X %02X %02X]",  __func__,
      pucApiVer[4],
      pucApiVer[5],
      pucApiVer[6],
      pucApiVer[7],
      pucApiVer[0],
      pucApiVer[1],
      pucApiVer[2],
      pucApiVer[3]);

  return EXTEND_FUNC_RET_OK;
}

