#include <string.h>
#include <stdio.h>
#include "dw1000.h"
#include "mac.h"

static uint8_t base_address[] = {0,0,0,0,0,0,0xcf,0xbc};

// The four packets for ranging
#define POLL 0x01   // Poll is initiated by the tag
#define ANSWER 0x02
#define FINAL 0x03
#define REPORT 0x04 // Report contains all measurement from the anchor

typedef struct {
  uint8_t pollRx[5];
  uint8_t answerTx[5];
  uint8_t finalRx[5];

  float pressure;
  float temperature;
  float asl;
  uint8_t pressure_ok;
} __attribute__((packed)) reportPayload_t;

typedef union timestamp_u {
  uint8_t raw[5];
  uint64_t full;
  struct {
    uint32_t low32;
    uint8_t high8;
  } __attribute__((packed));
  struct {
    uint8_t low8;
    uint32_t high32;
  } __attribute__((packed));
} timestamp_t;

// Timestamps for ranging
static dwTime_t poll_tx;
static dwTime_t poll_rx;
static dwTime_t answer_tx;
static dwTime_t answer_rx;
static dwTime_t final_tx;
static dwTime_t final_rx;

static const double C = 299792458.0;       // Speed of light
static const double tsfreq = 499.2e6 * 128;  // Timestamp counter frequency

#define ANTENNA_OFFSET 154.6   // In meter
#define ANTENNA_DELAY  (ANTENNA_OFFSET*499.2e6*128)/299792458.0 // In radio tick

static packet_t rxPacket;
static packet_t txPacket;
static volatile uint8_t curr_seq = 0;
static int curr_anchor = 0;
uwbConfig_t config;

// #define printf(...)
#define debug(...) // printf(__VA_ARGS__)

static void txcallback()
{
  dwTime_t departure;
  DW_GetTransmitTimestamp(dev, &departure);
  departure.full += (ANTENNA_DELAY/2);

  debug("TXCallback\r\n");

  switch (txPacket.payload[0]) {
    case POLL:
      poll_tx = departure;
      break;
    case FINAL:
      final_tx = departure;
      break;
  }
}

#define TYPE 0
#define SEQ 1

static void rxcallback() {
  dwTime_t arival = { .full=0 };
  int dataLength = DW_GetDataLength(dev);

  if (dataLength == 0) return;

  memset(&rxPacket, 0, MAC802154_HEADER_LENGTH);

  debug("RXCallback(%d): ", dataLength);

  DW_GetData(dev, (uint8_t*)&rxPacket, dataLength);

  if (memcmp(rxPacket.destAddress, config.address, 8)) {
    debug("Not for me! for %02x with %02x\r\n", rxPacket.destAddress[0], rxPacket.payload[0]);
    DW_NewReceive(dev);
    DW_SetDefaults(dev);
    DW_StartReceive(dev);
    return;
  }

  memcpy(txPacket.destAddress, rxPacket.sourceAddress, 8);
  memcpy(txPacket.sourceAddress, rxPacket.destAddress, 8);

  switch(rxPacket.payload[TYPE]) {
    // Tag received messages
    case ANSWER:
      debug("ANSWER\r\n");

      if (rxPacket.payload[SEQ] != curr_seq) {
        debug("Wrong sequence number!\r\n");
        return;
      }

      txPacket.payload[0] = FINAL;
      txPacket.payload[SEQ] = rxPacket.payload[SEQ];

      DW_NewTransmit(dev);
      DW_SetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);

      DW_WaitForResponse(dev, true);
      DW_StartTransmit(dev);

      DW_GetReceiveTimestamp(dev, &arival);
      arival.full -= (ANTENNA_DELAY/2);
      answer_rx = arival;
      break;
    case REPORT:
    {
      reportPayload_t *report = (reportPayload_t *)(rxPacket.payload+2);
      double tround1, treply1, treply2, tround2, tprop_ctn, tprop, distance;

      debug("REPORT\r\n");

      if (rxPacket.payload[SEQ] != curr_seq) {
        printf("Wrong sequence number!\r\n");
        return;
      }

      memcpy(&poll_rx, &report->pollRx, 5);
      memcpy(&answer_tx, &report->answerTx, 5);
      memcpy(&final_rx, &report->finalRx, 5);

      printf("%02x%08x ", (unsigned int)poll_tx.high8, (unsigned int)poll_tx.low32);
      printf("%02x%08x\r\n", (unsigned int)poll_rx.high8, (unsigned int)poll_rx.low32);
      printf("%02x%08x ", (unsigned int)answer_tx.high8, (unsigned int)answer_tx.low32);
      printf("%02x%08x\r\n", (unsigned int)answer_rx.high8, (unsigned int)answer_rx.low32);
      printf("%02x%08x ", (unsigned int)final_tx.high8, (unsigned int)final_tx.low32);
      printf("%02x%08x\r\n", (unsigned int)final_rx.high8, (unsigned int)final_rx.low32);

      tround1 = answer_rx.low32 - poll_tx.low32;
      treply1 = answer_tx.low32 - poll_rx.low32;
      tround2 = final_rx.low32 - answer_tx.low32;
      treply2 = final_tx.low32 - answer_rx.low32;

      printf("%08x %08x\r\n", (unsigned int)tround1, (unsigned int)treply2);
      printf("\\    /   /     \\\r\n");
      printf("%08x %08x\r\n", (unsigned int)treply1, (unsigned int)tround2);

      tprop_ctn = ((tround1*tround2) - (treply1*treply2)) / (tround1 + tround2 + treply1 + treply2);

      printf("TProp (ctn): %d\r\n", (unsigned int)tprop_ctn);

      tprop = tprop_ctn/tsfreq;
      distance = C * tprop;

      printf("distance %d: %5dmm\r\n", rxPacket.sourceAddress[0], (unsigned int)(distance*1000));

      DW_GetReceiveTimestamp(dev, &arival);
      arival.full -= (ANTENNA_DELAY/2);
      printf("Total in-air time (ctn): 0x%08x\r\n", (unsigned int)(arival.low32-poll_tx.low32));

      break;
    }
  }
}

void initiateRanging()
{
  printf ("Interrogating anchor %d\r\n",  config.anchors[curr_anchor]);
  base_address[0] =  config.anchors[curr_anchor];
  curr_anchor ++;
  if (curr_anchor > config.anchorListSize) {
    curr_anchor = 0;
  }
  dwt_forcetrxoff();//TODO:shall it can be removed?

  txPacket.payload[TYPE] = POLL;
  txPacket.payload[SEQ] = ++curr_seq;

  memcpy(txPacket.sourceAddress, config.address, 8);
  memcpy(txPacket.destAddress, base_address, 8);

  dwt_writetxdata(sizeof(txPacket), txPacket, 0); /* Zero offset in TX buffer. */
  dwt_writetxfctrl(sizeof(txPacket), 0, 1); /* Zero offset in TX buffer, ranging. */
  
  /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
   * set by dwt_setrxaftertxdelay() has elapsed. */
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

}

static uint32_t twrTagOnEvent(uwbEvent_t event)
{
    uint32_t timeout_val = MAX_TIMEOUT;
    switch(event) {
        case eventPacketReceived:
          rxcallback();
          // 10ms between rangings
          timeout_val = 10;
          break;
        case eventPacketSent:
          txcallback();
          timeout_val = 10;
          break;
        case eventTimeout:
          initiateRanging();
          timeout_val = 10;
          break;
        case eventReceiveFailed:
          // Try again ranging in 10ms
          timeout_val = 10;
          break;
        default:
          //TODO:Handle Error
          break;
    }

    return timeout_val;
}

static void twrTagInit(uwbConfig_t * newconfig)
{
  config = *newconfig;

  // Initialize the packet in the TX buffer
  MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
  txPacket.pan = 0xbccf;

  // onEvent is going to be called with eventTimeout which will start ranging
}

uwbAlgorithm_t uwbTwrTagAlgorithm = {
  .init = twrTagInit,
  .onEvent = twrTagOnEvent,
};
