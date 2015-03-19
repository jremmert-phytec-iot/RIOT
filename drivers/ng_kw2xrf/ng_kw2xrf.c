/*
 * Copyright (C) 2015 PHYTEC Messtechnik GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     kw2xrf
 * @{
 * @file        kw2xrf.c
 * @brief       Basic functionality of kw2xrf driver
 *
 * @author      Johann Fischer <j.fischer@phytec.de>
 * @author      Jonas Remmert <j.remmert@phytec.de>
 * @}
 */
#include "crash.h"
#include "ng_kw2xrf.h"
#include "kw2xrf_spi.h"
#include "kw2xrf_reg.h"
#include "kw2xrf_internal.h"
#include "net/ng_pkt.h"
#include "periph_conf.h"
#include "periph/gpio.h"

#define ENABLE_DEBUG    (0)
/* Enables integrated testing functions, such as LED-toggling
 * on state-change and idle-wait functions to test arbitration
 * using multiple boards.
 */
#define TESTING_FUNCTIONS   (1)
#include "debug.h"

ng_netdev_t *kw2xrf_netdev;
static uint16_t radio_pan;
static uint8_t  radio_channel;
static uint16_t radio_address;
static uint64_t radio_address_long;

#define MKW2XDRF_OUTPUT_POWER_MAX  8        /**< Maximum output power of the kw2x-rf radio in dBm */
#define MKW2XDRF_OUTPUT_POWER_MIN  (-35)    /**< Minimum output power of the kw2x-rf radio in dBm */

/* Modem_PA_PWR Register (PA Power Control) has a valid range from 3-31 */
#define MKW2XDRF_PA_RANGE_MAX      31       /**< Maximum value of PA Power Control Register */
#define MKW2XDRF_PA_RANGE_MIN      3        /**< Minimum value of PA Power Control Register */

/* PLL integer lookup table */
static const uint8_t pll_int_lt[16] = {
    11, 11, 11, 11,
    11, 11, 12, 12,
    12, 12, 12, 12,
    13, 13, 13, 13
};

/* PLL frequency fractional lookup table */
static const uint16_t pll_frac_lt[16] = {
    10240, 20480, 30720, 40960,
    51200, 61440, 6144, 16384,
    26624, 36864, 47104, 57344,
    2048, 12288, 22528, 32768
};

static const uint8_t pow_lt[44] = {
    3, 4, 4, 5,
    6, 6, 7, 7,
    8, 9, 9, 10,
    11, 11, 12, 13,
    13, 14, 14, 15,
    16, 16, 17, 18,
    18, 19, 20, 20,
    21, 21, 22, 23,
    23, 24, 25, 25,
    26, 27, 27, 28,
    28, 29, 30, 31
};

static const int level_lt[29] = {
    -35, -34, -32, -31,
    -29, -28, -26, -25,
    -23, -22, -20, -19,
    -17, -16, -14, -13,
    -11, -10, -8, -7,
    -5, -4, -2, -1,
    1, 2, 4, 5,
    7
};
/**************Internal Functions****************************/

int kw2xrf_set_tx_power(int pow)
{
    if (pow > MKW2XDRF_OUTPUT_POWER_MAX) {
        pow = MKW2XDRF_OUTPUT_POWER_MAX;
    }

    if (pow < MKW2XDRF_OUTPUT_POWER_MIN) {
        pow = MKW2XDRF_OUTPUT_POWER_MIN;
    }

    uint8_t level = pow_lt[pow - MKW2XDRF_OUTPUT_POWER_MIN];
    kw2xrf_write_dreg(MKW2XDM_PA_PWR, MKW2XDM_PA_PWR(level));
    return pow;
}

int _set_channel(uint8_t *val, size_t len)
{
    if (val[0] < 11 || val[0] > 26) {
        DEBUG("Invalid channel %i set. Valid channels are 11 through 26\n", val[0]);
        return -EINVAL;
    }

    if (len != 2 || val[1] != 0) {
        DEBUG("kw2xrf: set channel failed, len: %u, val[0]:%u\n", len, val[0]);
        return -EINVAL;
    }

    /*
     * Fc = 2405 + 5(k - 11) , k = 11,12,...,26
     *
     * Equation for PLL frequency, MKW2xD Reference Manual, p.255 :
     * F = ((PLL_INT0 + 64) + (PLL_FRAC0/65536))32MHz
     *
     */
    val[0] -= 11;
    kw2xrf_write_dreg(MKW2XDM_PLL_INT0, MKW2XDM_PLL_INT0_VAL(pll_int_lt[val[0]]));
    kw2xrf_write_dreg(MKW2XDM_PLL_FRAC0_LSB, (uint8_t)pll_frac_lt[val[0]]);
    kw2xrf_write_dreg(MKW2XDM_PLL_FRAC0_MSB, (uint8_t)(pll_frac_lt[val[0]] >> 8));
    return 2;
}

int _get_channel(uint8_t *val, size_t max)
{
    if (max < 2) {
        return -EOVERFLOW;
    }
    uint8_t pll_int = kw2xrf_read_dreg(MKW2XDM_PLL_INT0);
    uint16_t pll_frac = kw2xrf_read_dreg(MKW2XDM_PLL_FRAC0_LSB);
    pll_frac |= ((uint16_t)kw2xrf_read_dreg(MKW2XDM_PLL_FRAC0_MSB) << 8);

    for (int i = 0; i < 16; i++) {
        if ((pll_frac_lt[i] == pll_frac) && (pll_int_lt[i] == pll_int)) {
            val[0] = i + 11;
            val[1] = 0;
            return 2;
        }
    }
    return -EINVAL;
}

void _set_sequence(kw2xrf_physeq_t seq)
{
    uint8_t reg = kw2xrf_read_dreg(MKW2XDM_PHY_CTRL1);
    reg &= ~MKW2XDM_PHY_CTRL1_XCVSEQ_MASK; /* set last three bit to 0 */

    //if (reg) { /* If radio is not in idle: Abort sequence */
   	if (kw2xrf_read_dreg(MKW2XDM_SEQ_STATE)) {
        /* abort any ongoing sequence */
        DEBUG("tx: abort SEQ_STATE: %x\n", kw2xrf_read_dreg(MKW2XDM_SEQ_STATE));
        kw2xrf_write_dreg(MKW2XDM_PHY_CTRL1, reg);
        //kw2xrf_write_dreg(MKW2XDM_PHY_CTRL1, MKW2XDM_PHY_CTRL1_XCVSEQ(0));
    	while (kw2xrf_read_dreg(MKW2XDM_SEQ_STATE));
    }
    /* Save unneccessary cpu time and exit at this point if possible */
    if (seq == XCVSEQ_IDLE){
        return;
    }
    /* Progrmm new sequence */
    reg |= MKW2XDM_PHY_CTRL1_XCVSEQ(seq);  /* set last three bit to 1 */
    kw2xrf_write_dreg(MKW2XDM_PHY_CTRL1, reg);
    DEBUG("ng_kw2xrf: Set sequence to %i\n", seq);
    //while (!(kw2xrf_read_dreg(MKW2XDM_SEQ_STATE)));
}

int _set_tx(void)
{
    _set_sequence(XCVSEQ_TRANSMIT);
    return 0;
}

int _set_rx(void)
{
    DEBUG("ng_kw2xrf: info: switch_to_rx\n");
    _set_sequence(XCVSEQ_RECEIVE);
    return 0;
}

/* TODO: Rename, function is general and not rx-specific */
void kw2xrf_rx_irq(void *args)
{
    uint8_t irqst1 = kw2xrf_read_dreg(MKW2XDM_IRQSTS1);
    uint8_t irqst2 = kw2xrf_read_dreg(MKW2XDM_IRQSTS2);
    int8_t status;
    ng_netdev_event_t ng_netdev_event;
    DEBUG("rx_irq: IRQSTS1: %x\n", irqst1);
    DEBUG("rx_irq: IRQSTS2: %x\n", irqst2);
    /*DEBUG("rx_irq: IRQSTS2: %x\n", kw2xrf_read_dreg(MKW2XDM_IRQSTS2));
    DEBUG("rx_irq: CTRL1: %x\n", kw2xrf_read_dreg(MKW2XDM_PHY_CTRL1));
    DEBUG("rx_irq: CTRL2: %x\n", kw2xrf_read_dreg(MKW2XDM_PHY_CTRL2));
    DEBUG("rx_irq: CTRL3: %x\n", kw2xrf_read_dreg(MKW2XDM_PHY_CTRL3));
    DEBUG("rx_irq: CTRL4: %x\n", kw2xrf_read_dreg(MKW2XDM_PHY_CTRL4));
    DEBUG("rx_irq: PLL_INT0: %x\n", kw2xrf_read_dreg(MKW2XDM_PLL_INT0));
    DEBUG("rx_irq: PLL_FRAC0_LSB: %x\n", kw2xrf_read_dreg(MKW2XDM_PLL_FRAC0_LSB));
    DEBUG("rx_irq: PLL_FRAC0_MSB: %x\n", kw2xrf_read_dreg(MKW2XDM_PLL_FRAC0_MSB));
    DEBUG("rx_irq: PA_PWR: %x\n", kw2xrf_read_dreg(MKW2XDM_PA_PWR));
    DEBUG("rx_irq: PWR_MODES: %x\n", kw2xrf_read_dreg(MKW2XDM_PWR_MODES));
    */
    if (irqst1 & MKW2XDM_IRQSTS1_RXIRQ) {
        /* handle receive */
        //kw2xrf_rx_handler();
        ng_netdev_event = NETDEV_EVENT_RX_COMPLETE;
        _set_sequence(XCVSEQ_IDLE);
        _set_sequence(XCVSEQ_RECEIVE);

        //typedef void (*ng_netdev_event_cb_t)(ng_netdev_event_t type, void *arg);
        kw2xrf_write_dreg(MKW2XDM_IRQSTS1, MKW2XDM_IRQSTS1_RXIRQ | MKW2XDM_IRQSTS1_SEQIRQ);
    }

    if (irqst1 & MKW2XDM_IRQSTS1_CCAIRQ) {
        #if (TESTING_FUNCTIONS)
        LED_R_ON; /* Measurement indication for TX duration */
        #endif
        DEBUG("kw2xrf: CCA_irq, CCA completed\n");
        kw2xrf_write_dreg(MKW2XDM_IRQSTS1, MKW2XDM_IRQSTS1_CCAIRQ);
    }

    if (irqst1 & MKW2XDM_IRQSTS1_SEQIRQ) {
        ng_netdev_event = NETDEV_EVENT_CCA_CHANNEL_BUSY;        /* default value */
        if ((irqst1 & MKW2XDM_IRQSTS1_CCAIRQ) &&
            (irqst2 & MKW2XDM_IRQSTS2_CCA)) {                   /* CCA: Channel busy */
            LED_R_OFF; /* Measurement indication for TX duration */
            ng_netdev_event = NETDEV_EVENT_CCA_CHANNEL_BUSY;
            DEBUG("kw2xrf: CCA_irq, Channel Busy\n");
            kw2xrf_write_dreg(MKW2XDM_IRQSTS1, MKW2XDM_IRQSTS1_CCAIRQ | MKW2XDM_IRQSTS1_SEQIRQ);
            _set_sequence(XCVSEQ_IDLE);
        }
        if (irqst1 & MKW2XDM_IRQSTS1_TXIRQ) {
            LED_R_OFF; /* Measurement indication for TX duration */
            ng_netdev_event = NETDEV_EVENT_TX_COMPLETE;
            DEBUG("kw2xrf: TX_irq, Transmission completed\n");
            kw2xrf_write_dreg(MKW2XDM_IRQSTS1, MKW2XDM_IRQSTS1_TXIRQ | MKW2XDM_IRQSTS1_SEQIRQ);
            _set_sequence(XCVSEQ_IDLE);
        }
        kw2xrf_netdev->event_cb(ng_netdev_event, &status);
    }
    DEBUG("kw2xrf: rx_irq: ng_netdev_event: %i\n", ng_netdev_event);
}

/* Set up interrupt sources, triggered by the radio-module */
void kw2xrf_init_interrupts(void)
{
    /* set up GPIO-pin used for IRQ */
    gpio_init_int(GPIO_KW2XDRF, GPIO_NOPULL, GPIO_FALLING, &kw2xrf_rx_irq, NULL);

    /* Clear interrupt status flags by writing ones to Interrupt Request Status Reg 1 (IRQSTS1) */
    kw2xrf_write_dreg(MKW2XDM_IRQSTS1, 0xff);
    /* Clear Packet Buffer Underrun Error IRQ status and Wake Interrupt Status */
    kw2xrf_write_dreg(MKW2XDM_IRQSTS2, MKW2XDM_IRQSTS2_PB_ERR_IRQ
                      | MKW2XDM_IRQSTS2_WAKE_IRQ);
    /* Clear timer interrupt flags */
    kw2xrf_write_dreg(MKW2XDM_IRQSTS3, 0x0f);
    /* Print interrupt flag status bits  */
    DEBUG("IRQSTS1: %x\n", kw2xrf_read_dreg(MKW2XDM_IRQSTS1));
    DEBUG("IRQSTS2: %x\n", kw2xrf_read_dreg(MKW2XDM_IRQSTS2));
    DEBUG("IRQSTS3: %x\n", kw2xrf_read_dreg(MKW2XDM_IRQSTS3));
    uint8_t reg = kw2xrf_read_dreg(MKW2XDM_PHY_CTRL2);
    /* Enable interrupt on RX operation */
    reg &= ~(MKW2XDM_PHY_CTRL2_RXMSK);
    /* Enable interrupt on CCA ready */
    reg &= ~(MKW2XDM_PHY_CTRL2_CCAMSK); /* only for measurement indication */
    /* Enable interrupt on TX ready */
    //reg &= ~(MKW2XDM_PHY_CTRL2_TXMSK);
    /* Enable interrupt on SEQ ready */
    reg &= ~(MKW2XDM_PHY_CTRL2_SEQMSK);
    kw2xrf_write_dreg(MKW2XDM_PHY_CTRL2, reg);
    /* activate promiscous mode for testing*/
    reg = kw2xrf_read_dreg(MKW2XDM_PHY_CTRL4);
    reg |= MKW2XDM_PHY_CTRL4_PROMISCUOUS;
    kw2xrf_write_dreg(MKW2XDM_PHY_CTRL4, reg);
    DEBUG("PHY_CTRL2: %x\n", kw2xrf_read_dreg(MKW2XDM_PHY_CTRL2));
}

int _set_pan(uint8_t *val, size_t len)
{
    if (len != 2) {
        return -EINVAL;
    }
    kw2xrf_write_iregs(MKW2XDMI_MACPANID0_LSB, val, 2);
    return 2;
}

int _get_pan(uint8_t *val, size_t max)
{
    if (max < 2) {
        return -EOVERFLOW;
    }
    kw2xrf_read_iregs(MKW2XDMI_MACPANID0_LSB, val, 2);
    return 2;
}

int kw2xrf_on(void)
{
    uint8_t tmp;
    /* check modem's crystal oscillator, CLK_OUT shall be 4MHz */
    tmp = kw2xrf_read_dreg(MKW2XDM_CLK_OUT_CTRL);

    if (tmp != 0x8Bu) {
        return -1;
    }

    DEBUG("SEQ_STATE: %x\n", kw2xrf_read_dreg(MKW2XDM_SEQ_STATE));

    /* enable RFon mode */
    kw2xrf_write_dreg(MKW2XDM_PWR_MODES,
                      (MKW2XDM_PWR_MODES_XTALEN | MKW2XDM_PWR_MODES_PMC_MODE));

    /* abort any ongoing sequence */
    _set_sequence(XCVSEQ_IDLE);

    return 0;
}

uint8_t _channel_clear(void){
    _set_sequence(XCVSEQ_CCA);   /* start CCA, interrupt is triggered if ready */
    LED_R_ON; /* Measurement indication for CCA duration */
    return 0;
}

int _get_address(uint8_t *val, size_t len)
{
    if (len == 2) {
        kw2xrf_read_iregs(MKW2XDMI_MACSHORTADDRS0_LSB, val, 2);
        return 2;
    }
    else if (len == 8){
        kw2xrf_read_iregs(MKW2XDMI_MACLONGADDRS0_0, val, 8);
        return 8;
    }
    return -ENOTSUP;
}

int _set_address(uint8_t *val, size_t len)
{
    if (len == 2) {
        kw2xrf_write_iregs(MKW2XDMI_MACSHORTADDRS0_LSB, val, 2);
        return 2;
    }
    else if (len == 8){
        kw2xrf_write_iregs(MKW2XDMI_MACLONGADDRS0_0, val, 8);
        return 8;
    }
    return -ENOTSUP;
}

/************************************************************/

int kw2xrf_init(ng_netdev_t *dev) {
    uint8_t reg = 0;
    uint8_t init_val[2];
    DEBUG("ng_kw2xrf: Initializing\n");
    //dev->type = NETDEV_TYPE_802154;
    //dev->more = NULL;
    kw2xrf_netdev = dev;
    kw2xrf_spi_init();
    if (!(kw2xrf_on() == 0)) {
        core_panic(0x42, "Could not start MKW2XD radio transceiver");
    }

    kw2xrf_set_tx_power(0);

    /* Set radio pan id. */
#ifdef MODULE_CONFIG
    radio_pan = sysconfig.radio_pan_id;
#else
    radio_pan = MKW2XDRF_DEFAULT_RADIO_PAN;
#endif
    init_val[0] = (uint8_t)radio_pan;
    radio_pan = radio_pan >> 8;
    init_val[1] = (uint8_t)radio_pan;
    _set_pan((uint8_t *)init_val, 2);

    /* Set radio channel. */
#ifdef MODULE_CONFIG
    radio_channel = sysconfig.radio_channel;
#else
    radio_channel = MKW2XDRF_DEFAULT_CHANNR;
#endif
    init_val[0] = radio_channel;
    init_val[1] = 0;
    _set_channel((uint8_t *)init_val, 2);

    DEBUG("MKW2XDRF initialized and set to channel %i and pan %i.\n",
    MKW2XDRF_DEFAULT_CHANNR, radio_pan);

    kw2xrf_read_iregs(MKW2XDMI_MACSHORTADDRS0_LSB, (uint8_t *)&radio_address, 2);
    kw2xrf_read_iregs(MKW2XDMI_MACLONGADDRS0_0, (uint8_t *)&radio_address_long, 8);

    /* CCA Setup */
    reg = kw2xrf_read_dreg(MKW2XDM_PHY_CTRL4);
    reg |= MKW2XDM_PHY_CTRL4_CCATYPE(1); /* Set up CCA mode 1 (RSSI threshold) */
    kw2xrf_write_dreg(MKW2XDM_PHY_CTRL4, reg);

    /* Set CCA mode 1 Threshold (unitless RM p.113, default 75 ->~70dBm) */
    //reg= 4b;
    //kw2xrf_write_iregs(MKW2XDMI_CCA1_THRESH,(uint8_t *)&reg, 1);
    /*******************/

    kw2xrf_init_interrupts();
    DEBUG("Initialization done..\n");
    return 0;
}

int _add_cb(ng_netdev_t *dev, ng_netdev_event_cb_t cb)
{
    if (dev == NULL) {
        return -ENODEV;
    }
    if (dev->event_cb != NULL) {
        return -ENOBUFS;
    }
    dev->event_cb = cb;
    return 0;
}

int _rem_cb(ng_netdev_t *dev, ng_netdev_event_cb_t cb)
{
    if (dev == NULL) {
        return -ENODEV;
    }
    if (dev->event_cb != cb) {
        return -ENOENT;
    }
    dev->event_cb = NULL;
    return 0;
}

int _get(ng_netdev_t *netdev, ng_netconf_opt_t opt, void *value, size_t max_len)
{
    int res = 0;
    switch (opt) {
        case NETCONF_OPT_ADDRESS:
            res = _get_address((uint8_t *)value, max_len);
            return res;
        case NETCONF_OPT_CHANNEL:
            res = _get_channel((uint8_t *)value, max_len);
            return res;
        case NETCONF_OPT_NID:
            res = _get_pan((uint8_t *)value, max_len);
            return res;
        case NETCONF_OPT_IS_CHANNEL_CLR:
            return _channel_clear();
        case NETCONF_OPT_STATE:
            return 0;
        default:
            return -ENOTSUP;
    }
}

int _set(ng_netdev_t *netdev, ng_netconf_opt_t opt, void *value, size_t value_len)
{
    uint8_t reg;
    switch (opt) {
        case NETCONF_OPT_CHANNEL:
            return _set_channel((uint8_t *)value, value_len);
        case NETCONF_OPT_ADDRESS:
            return _set_address((uint8_t *)value, value_len);
        case NETCONF_OPT_NID:
            return _set_pan((uint8_t *)value, value_len);
        case NETCONF_OPT_IS_CHANNEL_CLR:
            return _channel_clear();
        case NETCONF_OPT_CCA_BEFORE_TX:
            reg = kw2xrf_read_dreg(MKW2XDM_PHY_CTRL1);
            reg |= MKW2XDM_PHY_CTRL1_CCABFRTX;          /* Set up CCA before TX */
            kw2xrf_write_dreg(MKW2XDM_PHY_CTRL1, reg);
            return 0;
        case NETCONF_OPT_AUTOACK:
            reg = kw2xrf_read_dreg(MKW2XDM_PHY_CTRL1);
            /* Set up HW generated automatic ACK after Receive */
            reg |= MKW2XDM_PHY_CTRL1_AUTOACK;
            kw2xrf_write_dreg(MKW2XDM_PHY_CTRL1, reg);
            return 0;
        case NETCONF_OPT_STATE:
            switch (*((int *)value))
                case NETCONF_STATE_TX:
                    DEBUG("KW2xrf: Function _set_tx entered");
                    _set_tx();
                    return 2;
                case NETCONF_STATE_RX:
                    _set_rx();
                    return 2;
        default:
            return -ENOTSUP;
    }
}

void _isr_event(ng_netdev_t *netdev, uint32_t event_type)
{
}

int _send(ng_netdev_t *netdev, ng_pktsnip_t *pkt)
{
    kw2xrf_t *dev = (kw2xrf_t *)netdev;
    uint8_t test[] = "_test";
    test[0] = 6;
    for (int i = 0; i < test[0]; i++) {
        dev->buf[i] = test[i];
    }
    DEBUG("ng_kw2xrf: send packet with size %i\n", dev->buf[0]);

    kw2xrf_write_fifo(dev->buf, dev->buf[0]);

    return 0;
}

/* implementation of the netdev interface */
const ng_netdev_driver_t kw2xrf_driver = {
    .send_data = _send,
    .add_event_callback = _add_cb,
    .rem_event_callback = _rem_cb,
    .get = _get,
    .set = _set,
    .isr_event = _isr_event,
};
