#ifndef __UWB_H__
#define __UWB_H__

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

extern uwbAlgorithm_t uwbTwrAnchorAlgorithm;
extern uwbAlgorithm_t uwbTwrTagAlgorithm;
void uwb_init(void);

#endif