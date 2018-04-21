#include "uwb.h"
#include "port.h"

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
/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
#define PRE_TIMEOUT 0

/* Declaration of static val. */
static bool irq_flag = false;


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

struct uwbConfig_s * uwb_get_config(void)
{
    return &config_t;
}

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
		 return;
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
        //lcd_display_str("INIT FAILED");
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
