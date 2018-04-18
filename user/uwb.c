#include "uwb.h"

#define     TWR_ANCHOR              0
#define     TWR_TAG                 1
#define     CURRENT_TAG             TWR_TAG

struct {
  uwbAlgorithm_t *algorithm;
  char *name;
} availableAlgorithms[] = {
  {.algorithm = &uwbTwrAnchorAlgorithm, .name = "TWR Anchor"},
  {.algorithm = &uwbTwrTagAlgorithm,    .name = "TWR Tag"},
  {NULL, NULL},
};
static uwbAlgorithm_t *algorithm = NULL;

static dwt_config_t ic_config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

static uwbConfig_t config_t = {
    .mode = 0,
    .address = {0,0,0,0,0,0,0xcf,0xbc},
    .anchorListSize = 0,
    .anchors = {0},
    .position = {0.0},
    .positionEnabled = 0,
};

/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

/* Frames used in the ranging process. See NOTE 2 below. */
static uint8 rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10
/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4
/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 24
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

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
/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
#define PRE_TIMEOUT 0

/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef signed long long int64;
typedef unsigned long long uint64;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;
static uint64 final_rx_ts;

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;

/* String used to display measured distance on LCD screen (16 characters maximum). */
char dist_str[16] = {0};

/* Declaration of static functions. */
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);
static bool irq_flag = false;

static void uwb_irq_callback(void)
{
    irq_flag = true;
}

static void uwb_handle_interrupt(void) {
    uint32 sysstatus = 0;
	// read current status and handle events
	//SYS_STATUS_ID register length is 5,but this function only use the low 4 bytes
	sysstatus = dwt_read32bitreg(SYS_STATUS_ID);
	//check clock error
	if(sysstatus & SYS_STATUS_CLKPLL_LL & SYS_STATUS_RFPLL_LL) {
		 /* TODO handle error */ 
		 break;
	}
	
	//TX Over
	if(sysstatus & SYS_STATUS_TXFRS) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX); // Clear TX event bits
		algorithm->onEvent(eventPacketSent);
	}
	
	//Receive OK
	if(sysstatus & SYS_STATUS_RXFCG) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_GOOD); // Clear all receive status bits
        algorithm->onEvent(eventPacketReceived);
	}

	//Receive Timeout
	if(sysstatus & SYS_STATUS_ALL_RX_TO) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXRFTO); // Clear RX timeout event bits
        algorithm->onEvent(eventReceiveTimeout);
	}

	//Receive Error
	if(sysstatus & SYS_STATUS_ALL_RX_ERR)
	{
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR); // Clear RX error event bits
        algorithm->onEvent(eventReceiveFailed);
	}
}

void uwb_init(void)
{
    /* Reset and initialise DW1000.
     * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
     * performance. */
    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
    spi_set_rate_low();
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {
        lcd_display_str("INIT FAILED");
        while (1)
        { };
    }
    spi_set_rate_high();

    dwt_configure(&ic_config);
    port_set_deca_isr(uwb_irq_callback);
    dwt_setinterrupt(SYS_MASK_MTXFRS|SYS_MASK_MRXFCG|SYS_MASK_MRXRFTO, ENABLE);
    /* Apply default antenna delay value. See NOTE 1 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* Set preamble timeout for expected frames. See NOTE 6 below. */
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);

    algorithm = availableAlgorithms[CURRENT_TAG].algorithm;
    algorithm->init(&config_t);
}

void uwb_task(void)
{
    if(irq_flag)
    {
        uwb_handle_interrupt();
        irq_flag = false;
    }
}
