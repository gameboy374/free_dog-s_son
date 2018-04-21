#ifndef __UWB_H__
#define __UWB_H__

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "deca_device_api.h"
#include "deca_regs.h"

#define MAX_ANCHORS             6

typedef enum uwbEvent_e {
  eventTimeout,
  eventPacketReceived,
  eventPacketSent,
  eventReceiveTimeout,
  eventReceiveFailed,
} uwbEvent_t;

typedef struct uwbConfig_s {
  unsigned char mode;
  unsigned char address[8];
  unsigned char anchorListSize;
  unsigned char anchors[MAX_ANCHORS];
  float position[3];
  float positionEnabled;
} uwbConfig_t;

typedef struct uwbAlgorithm_s {
  void (*init)(uwbConfig_t * config);
  unsigned int (*onEvent)(uwbEvent_t event);
} uwbAlgorithm_t;

/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef signed long long int64;
typedef unsigned long long uint64;

extern uwbAlgorithm_t uwbTwrAnchorAlgorithm;
extern uwbAlgorithm_t uwbTwrTagAlgorithm;
void uwb_init(void);
void uwb_task(void);
uint64 get_tx_timestamp_u64(void);
uint64 get_rx_timestamp_u64(void);
uwbConfig_t * uwb_get_config(void);

#endif
