#include "mac.h"
#include "lpp.h"
#include "uwb.h"

uint8_t base_address[] = {0,0,0,0,0,0,0xcf,0xbc};

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.46 ms with above configuration. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 4000
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500
/* Receive final timeout. See NOTE 5 below. */
#define FINAL_RX_TIMEOUT_UUS 5000

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* String used to display measured distance on LCD screen (16 characters maximum). */
char dist_str[16] = {0};

// System configuration
static struct uwbConfig_s config;
#define MAX_TIMEOUT 2000

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
static timestamp_t poll_rx;
static timestamp_t answer_tx;
static timestamp_t final_rx;

float pressure, temperature, asl;
bool pressure_ok;

const double C = 299792458.0;       // Speed of light
const double tsfreq = 499.2e6 * 128;  // Timestamp counter frequency

#define ANTENNA_OFFSET 154.6   // In meter
#define ANTENNA_DELAY  (ANTENNA_OFFSET*499.2e6*128)/299792458.0 // In radio tick

static packet_t rxPacket;
static packet_t txPacket;
static volatile uint8_t curr_tag = 0;

// #define printf(...)
#define debug(...) // printf(__VA_ARGS__)

static void txcallback()
{
  timestamp_t departure;
  departure.full = get_tx_timestamp_u64();

  debug("TXCallback: ");

  switch (txPacket.payload[0]) {
    case ANSWER:
    debug("ANSWER to %02x at %04x\r\n", txPacket.destAddress[0], (unsigned int)departure.low32);
      answer_tx = departure;
      break;
    case REPORT:
      debug("REPORT\r\n");
      break;
  }
}

#define TYPE 0
#define SEQ 1
#define LPP_HEADER 2
#define LPP_TYPE 3
#define LPP_PAYLOAD 4

static void rxcallback() {
    timestamp_t arival = { .full=0 };
    uint32 resp_tx_time;
    int dataLength = 0;
    int payloadLength = 2;
    int ret;
    
    /* A frame has been received, read it into the local buffer. */
    dataLength = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;

    if (dataLength == 0) return;

    memset(&rxPacket, 0, MAC802154_HEADER_LENGTH);

    debug("RXCallback(%d): ", dataLength);

    dwt_readrxdata((uint8_t*)&rxPacket, dataLength, 0);

    if (memcmp(rxPacket.destAddress, config.address, 8))
    {
        debug("Not for me! for %02x with %02x\r\n", rxPacket.destAddress[0], rxPacket.payload[0]);
        dwt_forcetrxoff();
        dwt_rxreset();

        return;
    }

    memcpy(txPacket.destAddress, rxPacket.sourceAddress, 8);
    memcpy(txPacket.sourceAddress, rxPacket.destAddress, 8);

    switch(rxPacket.payload[TYPE]) {
        // Anchor received messages
        case POLL:
        {
            debug("POLL from %02x at %04x\r\n", rxPacket.sourceAddress[0], (unsigned int)arival.low32);

            curr_tag = rxPacket.sourceAddress[0];

            txPacket.payload[TYPE] = ANSWER;
            txPacket.payload[SEQ] = rxPacket.payload[SEQ];

            if (config.positionEnabled) {
                txPacket.payload[LPP_HEADER] = SHORT_LPP;
                txPacket.payload[LPP_TYPE] = LPP_SHORT_ANCHOR_POSITION;

                struct lppShortAnchorPosition_s *pos = (struct lppShortAnchorPosition_s*) &txPacket.payload[LPP_PAYLOAD];
                memcpy(pos->position, config.position, 3*sizeof(float));

                payloadLength += 2 + sizeof(struct lppShortAnchorPosition_s);
            }

            arival.full = get_rx_timestamp_u64();
            /* Set send time for response. See NOTE 9 below. */
            resp_tx_time = (arival.full + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
            dwt_setdelayedtrxtime(resp_tx_time);
            
            /* Set expected delay and timeout for final message reception. See NOTE 4 and 5 below. */
            dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
            dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

            dwt_writetxdata(MAC802154_HEADER_LENGTH+payloadLength, (uint8_t*)&txPacket, 0); /* Zero offset in TX buffer. */
            dwt_writetxfctrl(MAC802154_HEADER_LENGTH+payloadLength, 0, 1); /* Zero offset in TX buffer, ranging. */
            ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
            
            /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 11 below. */
            if (ret == DWT_ERROR)
            {
                dwt_setrxtimeout(0);
                dwt_forcetrxoff();
                dwt_rxreset();
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                break;
            }
            poll_rx = arival;
        break;
        }
        case FINAL:
        {
            if (curr_tag == rxPacket.sourceAddress[0]) {
                reportPayload_t *report = (reportPayload_t *)(txPacket.payload+2);

                debug("FINAL\r\n");

                arival.full = get_rx_timestamp_u64();
                final_rx = arival;

                txPacket.payload[TYPE] = REPORT;
                txPacket.payload[SEQ] = rxPacket.payload[SEQ];
                memcpy(&report->pollRx, &poll_rx, 5);
                memcpy(&report->answerTx, &answer_tx, 5);
                memcpy(&report->finalRx, &final_rx, 5);
                report->pressure = pressure;
                report->temperature = temperature;
                report->asl = asl;
                report->pressure_ok = pressure_ok;

                dwt_setrxaftertxdelay(10);
                dwt_setrxtimeout(0);
                dwt_writetxdata(sizeof(txPacket), (uint8 *)&txPacket, 0); /* Zero offset in TX buffer. */
                dwt_writetxfctrl(sizeof(txPacket), 0, 1); /* Zero offset in TX buffer, ranging. */
                ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
                /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 11 below. */
                if (ret == DWT_ERROR)
                {
                    dwt_setrxtimeout(0);
                    dwt_forcetrxoff();
                    dwt_rxreset();
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);
                }
            }
            else {
                dwt_setrxtimeout(0);
                dwt_forcetrxoff();
                dwt_rxreset();
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
            }
        break;
        }
        case SHORT_LPP:
        {
            if(curr_tag == rxPacket.sourceAddress[0] && dataLength-MAC802154_HEADER_LENGTH > 1) {
                lppHandleShortPacket((char *)&rxPacket.payload[1], dataLength-MAC802154_HEADER_LENGTH-1);
            }

            dwt_setrxtimeout(0);
            dwt_forcetrxoff();
            dwt_rxreset();
            dwt_rxenable(DWT_START_RX_IMMEDIATE);

            break;
        }
    }
}

static uint32_t twrAnchorOnEvent(uwbEvent_t event)
{
  switch(event) {
    case eventPacketReceived:
        rxcallback();
        break;
    case eventPacketSent:
        txcallback();
        break;
    case eventTimeout:
    case eventReceiveFailed:
        dwt_setrxtimeout(0);
        dwt_forcetrxoff();
        dwt_rxreset();
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
      break;
    default:
      //TODO:Handle Error
      break;
  }

  return MAX_TIMEOUT;
}

static void twrAnchorInit(uwbConfig_t * newconfig)
{

  config = *newconfig;

  // Initialize the packet in the TX buffer
  MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
  txPacket.pan = 0xbccf;

  // onEvent is going to be called with eventTimeout which will start receiving
}

uwbAlgorithm_t uwbTwrAnchorAlgorithm = {
  .init = twrAnchorInit,
  .onEvent = twrAnchorOnEvent,
};
