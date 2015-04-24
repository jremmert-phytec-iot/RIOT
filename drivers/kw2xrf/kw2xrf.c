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
#include "kw2xrf.h"
#include "kw2xrf_spi.h"
#include "kw2xrf_reg.h"
#include "mutex.h"
#include "msg.h"
#include "periph/gpio.h"
#include "net/ng_netbase.h"
//#include "net/ieee802154_frame.h"

#define ENABLE_DEBUG    (1)
#include "debug.h"

/**
 * @brief   Internal driver event type in case of an unexpected interrupt
 */
#define ISR_EVENT_UNKNOWN      (0x0020)

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

int _set_tx_power(kw2xrf_t *dev, int8_t *val, size_t len)
{

    if (len < 1) {
        return -EOVERFLOW;
    }

    if (val[0] > MKW2XDRF_OUTPUT_POWER_MAX) {
        val[0] = MKW2XDRF_OUTPUT_POWER_MAX;
    }

    if (val[0] < MKW2XDRF_OUTPUT_POWER_MIN) {
        val[0] = MKW2XDRF_OUTPUT_POWER_MIN;
    }

    uint8_t level = pow_lt[val[0] - MKW2XDRF_OUTPUT_POWER_MIN];
    kw2xrf_write_dreg(MKW2XDM_PA_PWR, MKW2XDM_PA_PWR(level));
    return 2;
}

int _get_channel(kw2xrf_t *dev, uint8_t *val, size_t max)
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

int _get_sequence(void)
{
   int reg = 0;
   reg = kw2xrf_read_dreg(MKW2XDM_PHY_CTRL1);
   reg &= MKW2XDM_PHY_CTRL1_XCVSEQ_MASK;
   return reg;
}

void _set_sequence(kw2xrf_t *dev, kw2xrf_physeq_t seq)
{
    uint8_t reg = 0;

    /* Only interrupt interruptable states */
    uint8_t curr_seq = kw2xrf_read_dreg(MKW2XDM_PHY_CTRL1);
    curr_seq &= (MKW2XDM_PHY_CTRL1_XCVSEQ_MASK);

    if ((curr_seq == XCVSEQ_RECEIVE) || (curr_seq == XCVSEQ_CONTINUOUS_CCA)) {
        /* Clear all pending interrupts */
        gpio_irq_disable(GPIO_KW2XDRF);

        /* abort any ongoing sequence */
        DEBUG("tx: abort SEQ_STATE: %x\n", kw2xrf_read_dreg(MKW2XDM_SEQ_STATE));
        reg = kw2xrf_read_dreg(MKW2XDM_PHY_CTRL1);
        reg &= ~(MKW2XDM_PHY_CTRL1_XCVSEQ_MASK);
        kw2xrf_write_dreg(MKW2XDM_PHY_CTRL1, reg);

        /* Mask all possible interrupts */
        reg = kw2xrf_read_dreg(MKW2XDM_PHY_CTRL3);
        reg |= MKW2XDM_PHY_CTRL3_WAKE_MSK;
        kw2xrf_write_dreg(MKW2XDM_PHY_CTRL3, reg);

        kw2xrf_write_dreg(MKW2XDM_IRQSTS1, 0x7f);
        kw2xrf_write_dreg(MKW2XDM_IRQSTS2, 0x03);
        kw2xrf_write_dreg(MKW2XDM_IRQSTS3, 0xff);

        gpio_irq_enable(GPIO_KW2XDRF);
    }

    /* Wait for all other states for be finished */
    while (kw2xrf_read_dreg(MKW2XDM_SEQ_STATE));

    /* For all sequences only enable SEQ-irq, that is set when sequence was completed */
    reg = kw2xrf_read_dreg(MKW2XDM_PHY_CTRL2);
    reg &= ~(MKW2XDM_PHY_CTRL2_SEQMSK);
    kw2xrf_write_dreg(MKW2XDM_PHY_CTRL2, reg);

    /* Progrmm new sequence */
    switch (seq) {
        case XCVSEQ_IDLE:
        dev->state = NETCONF_STATE_SLEEP;
        break;
        case XCVSEQ_RECEIVE:
        dev->state = NETCONF_STATE_IDLE;
        break;
        case XCVSEQ_TRANSMIT:
        dev->state = NETCONF_STATE_TX;
        break;
        case XCVSEQ_CCA:
        dev->state = NETCONF_STATE_TX;
        break;
        case XCVSEQ_TX_RX:
        dev->state = NETCONF_STATE_TX;
        break;
        case XCVSEQ_CONTINUOUS_CCA:
        dev->state = NETCONF_STATE_TX;
        break;
        default:
        DEBUG("kw2xrf: undefined state assigned to phy\n");
        dev->state = NETCONF_STATE_IDLE;
    }
    DEBUG("kw2xrf: Set sequence to %i\n", seq);
    reg = kw2xrf_read_dreg(MKW2XDM_PHY_CTRL1);
    reg &= ~(MKW2XDM_PHY_CTRL1_XCVSEQ_MASK);
    reg |= MKW2XDM_PHY_CTRL1_XCVSEQ(seq);
    kw2xrf_write_dreg(MKW2XDM_PHY_CTRL1, reg);
}

int _set_channel(kw2xrf_t *dev, uint8_t *val, size_t len)
{
    /* Save old sequence to restore this state later */
    uint8_t old_seq = _get_sequence();
    if(old_seq){
    _set_sequence(dev, XCVSEQ_IDLE);
    }
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
    uint8_t tmp = val[0] - 11;
    kw2xrf_write_dreg(MKW2XDM_PLL_INT0, MKW2XDM_PLL_INT0_VAL(pll_int_lt[tmp]));
    kw2xrf_write_dreg(MKW2XDM_PLL_FRAC0_LSB, (uint8_t)pll_frac_lt[tmp]);
    kw2xrf_write_dreg(MKW2XDM_PLL_FRAC0_MSB, (uint8_t)(pll_frac_lt[tmp] >> 8));

    DEBUG("kw2xrf: set channel to %u\n", val[0]);
    if(old_seq){
        _set_sequence(dev, old_seq);
    }

    return 2;
}

void kw2xrf_irq_handler(void *args)
{
    msg_t msg;
    kw2xrf_t *dev = (kw2xrf_t *)args;

    DEBUG("kw2xrf_isr_handler: called\n");
    /* notify driver thread about the interrupt */
    msg.type = NG_NETDEV_MSG_TYPE_EVENT;
    msg_send_int(&msg, dev->mac_pid);

}

/* Set up interrupt sources, triggered by the radio-module */
void kw2xrf_init_interrupts(kw2xrf_t *dev)
{
     /* Clear all pending interrupts */
    kw2xrf_write_dreg(MKW2XDM_IRQSTS1, 0x7f);
    kw2xrf_write_dreg(MKW2XDM_IRQSTS2, 0x03);
    kw2xrf_write_dreg(MKW2XDM_IRQSTS3, 0xff);

    /* Disable all interrups:
     * Selectively enable only one interrupt source selectively in sequence manager.
     * After reset state all interrupts are disabled, except WAKE_IRQ.
     */
    int reg = kw2xrf_read_dreg(MKW2XDM_PHY_CTRL3);
    reg |= MKW2XDM_PHY_CTRL3_WAKE_MSK;
    kw2xrf_write_dreg(MKW2XDM_PHY_CTRL3, reg);

    /* set up GPIO-pin used for IRQ */
    gpio_init_int(GPIO_KW2XDRF, GPIO_NOPULL, GPIO_FALLING, &kw2xrf_irq_handler, dev);
}

int _set_pan(kw2xrf_t *dev, uint16_t pan)
{
    dev->radio_pan = pan;

    uint8_t val_ar[2];
    val_ar[0] = (pan >> 8);
    val_ar[1] = (uint8_t)pan;
    kw2xrf_write_iregs(MKW2XDMI_MACPANID0_LSB, val_ar, 2);
    return 2;
}

int _get_proto(kw2xrf_t *dev, uint8_t *val, size_t max)
{
    if (max < sizeof(ng_nettype_t)) {
        return -EOVERFLOW;
    }
    memcpy(val, &(dev->proto), sizeof(ng_nettype_t));
    return sizeof(ng_nettype_t);
}

int _set_proto(kw2xrf_t *dev, uint8_t *val, size_t len)
{
    if (len != sizeof(ng_nettype_t)) {
        return -EINVAL;
    }
    memcpy(&(dev->proto), val, sizeof(ng_nettype_t));
    return sizeof(ng_nettype_t);
}

int kw2xrf_on(kw2xrf_t *dev)
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
    _set_sequence(dev, XCVSEQ_IDLE);

    dev->state = NETCONF_STATE_SLEEP;
    return 0;
}

int _set_addr(kw2xrf_t *dev, uint16_t addr)
{
    dev->addr_short = addr;

    uint8_t val_ar[2];
    val_ar[0] = (addr >> 8);
    val_ar[1] = (uint8_t)addr;
    kw2xrf_write_iregs(MKW2XDMI_MACSHORTADDRS0_LSB, val_ar, 2);
    return 2;
}

int _set_addr_long(kw2xrf_t *dev, uint64_t addr)
{
    dev->addr_long = addr;

    uint8_t val_ar[8];

    val_ar[0] = (uint8_t)(addr >> 56);
    val_ar[1] = (uint8_t)(addr >> 48);
    val_ar[2] = (uint8_t)(addr >> 40);
    val_ar[3] = (uint8_t)(addr >> 32);
    val_ar[4] = (uint8_t)(addr >> 24);
    val_ar[5] = (uint8_t)(addr >> 16);
    val_ar[6] = (uint8_t)(addr >> 8);
    val_ar[7] = (uint8_t)(addr);

    kw2xrf_write_iregs(MKW2XDMI_MACLONGADDRS0_0, val_ar, 8);

    return 8;
}

int kw2xrf_init(kw2xrf_t *dev)
{
    uint8_t reg = 0;
    uint8_t tmp[2];

    /* check device parameters */
    if (dev == NULL) {
        return -ENODEV;
    }

    kw2xrf_spi_init();

    if (!(kw2xrf_on(dev) == 0)) {
        core_panic(0x42, "Could not start MKW2XD radio transceiver");
    }

    /* Gerneral initialization of interrupt sources.
     * sets radio to idle modewith all interrupt masked
     */
    kw2xrf_init_interrupts(dev);

    /* set device driver */
    dev->driver = &kw2xrf_driver;
    /* set default options */
    dev->proto = KW2XRF_DEFAULT_PROTOCOL;
    dev->option = 0;

    /* set default short address */
    uint16_t tmp_addr = KW2XRF_DEFAULT_SHORT_ADDR >> 8;
    tmp_addr |= (KW2XRF_DEFAULT_SHORT_ADDR << 8);
    _set_addr(dev, tmp_addr);

    /* set default TX-Power */
    dev->tx_power = KW2XRF_DEFAULT_TX_POWER;
    _set_tx_power(dev, &(dev->tx_power), sizeof(dev->tx_power));

    /* set default channel */
    dev->radio_channel = KW2XRF_DEFAULT_CHANNEL;
    tmp[0] = dev->radio_channel;
    tmp[1] = 0;
    _set_channel(dev, tmp, 2);
    /* set default PAN ID */
    _set_pan(dev, dev->radio_pan);

    /* CCA Setup */
    reg = kw2xrf_read_dreg(MKW2XDM_PHY_CTRL4);
    reg |= MKW2XDM_PHY_CTRL4_CCATYPE(1); /* Set up CCA mode 1 (RSSI threshold) */
    kw2xrf_write_dreg(MKW2XDM_PHY_CTRL4, reg);
    DEBUG("kw2xrf: Initialized and set to channel %i and pan %i.\n",
          KW2XRF_DEFAULT_CHANNEL, KW2XRF_DEFAULT_PANID);

    /* Switch to Receive state per default after initialization */
    _set_sequence(dev, XCVSEQ_RECEIVE);
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
    kw2xrf_t *dev = (kw2xrf_t *)netdev;
    if (dev == NULL) {
        return -ENODEV;
    }

    switch (opt) {
        case NETCONF_OPT_ADDRESS:
            if (max_len < sizeof(uint16_t))
            {
                return -EOVERFLOW;
            }
            *((uint16_t*)value) = dev->addr_short;
            return 2;

        case NETCONF_OPT_ADDRESS_LONG:
            if (max_len < sizeof(uint64_t))
            {
                return -EOVERFLOW;
            }
            *((uint64_t*)value) = dev->addr_long;
            return 8;

        case NETCONF_OPT_NID:
            if (max_len < sizeof(uint16_t))
            {
                return -EOVERFLOW;
            }
            *((uint16_t*)value) = dev->radio_pan;
            return 2;
        case NETCONF_OPT_CHANNEL:
            return _get_channel(dev, (uint8_t *)value, max_len);
        case NETCONF_OPT_PROTO:
            return _get_proto(dev, (uint8_t *)value, max_len);
        case NETCONF_OPT_STATE:
            if (max_len < sizeof(ng_netconf_state_t)) {
                return -EOVERFLOW;
            }
            *(int *)value = *(ng_netconf_state_t *)&(dev->state);
            return 0;
        case NETCONF_OPT_TX_POWER:
            if (max_len < 1) {
                return -EOVERFLOW;
            }
            *(int16_t *)value = dev->tx_power;
            return 0;
        case NETCONF_OPT_RAWMODE:
            if (max_len < sizeof(ng_netconf_state_t)) {
                return -EOVERFLOW;
            }
            if (dev->option & OPT_RAWDUMP) {
                 *((ng_netconf_enable_t *)value) = NETCONF_ENABLE;
            }
            else {
                *((ng_netconf_enable_t *)value) = NETCONF_DISABLE;
            }
        default:
            return -ENOTSUP;
    }
}

int _set(ng_netdev_t *netdev, ng_netconf_opt_t opt, void *value, size_t value_len)
{
    uint8_t reg = 0;
    kw2xrf_t *dev = (kw2xrf_t *)netdev;
    if (dev == NULL) {
        return -ENODEV;
    }

    switch (opt) {
        case NETCONF_OPT_CHANNEL:
            return _set_channel(dev, (uint8_t *)value, value_len);
        case NETCONF_OPT_ADDRESS:
            if (value_len > sizeof(uint16_t))
            {
                return -EOVERFLOW;
            }
            return _set_addr(dev, *((uint16_t *)value));
        case NETCONF_OPT_ADDRESS_LONG:
            if (value_len > sizeof(uint64_t))
            {
                return -EOVERFLOW;
            }
            return _set_addr_long(dev, *((uint64_t *)value));
        case NETCONF_OPT_NID:
            if (value_len > sizeof(uint16_t))
            {
                return -EOVERFLOW;
            }
            return _set_pan(dev, *((uint16_t *)value));
        case NETCONF_OPT_IS_CHANNEL_CLR:
            _set_sequence(dev, XCVSEQ_CCA);
            return 0;
        case NETCONF_OPT_TX_POWER:
            _set_tx_power(dev, (int8_t *)value, value_len);
            return 0;
        case NETCONF_OPT_PROTO:
            return _set_proto(dev, (uint8_t *)value, value_len);
        case NETCONF_OPT_AUTOACK:
            /* Set up HW generated automatic ACK after Receive */
            reg |= MKW2XDM_PHY_CTRL1_AUTOACK;
            kw2xrf_write_dreg(MKW2XDM_PHY_CTRL1, reg);
            return 0;
        case NETCONF_OPT_PROMISCUOUSMODE:
            reg = kw2xrf_read_dreg(MKW2XDM_PHY_CTRL4);
            reg |= MKW2XDM_PHY_CTRL4_PROMISCUOUS;
            kw2xrf_write_dreg(MKW2XDM_PHY_CTRL4, reg);
            return 0;
        case NETCONF_OPT_RAWMODE:
            if (value_len > sizeof(ng_netconf_enable_t)) {
                return -EOVERFLOW;
            }
            if(((bool *)value)[0]) {
                dev->option |= OPT_RAWDUMP;
            }
            else {
                dev->option &= ~(OPT_RAWDUMP);
            }
            return sizeof(ng_netconf_enable_t);
        case NETCONF_OPT_PRELOADING:
            if (value_len > sizeof(ng_netconf_enable_t)) {
                return -EOVERFLOW;
            }
            if(((bool *)value)[0]) {
                dev->option |= OPT_PRELOADING;
            }
            else {
                dev->option &= ~(OPT_PRELOADING);
            }
            return sizeof(ng_netconf_enable_t);
        /* TODO: Maybe this function is neccessary for future CSMA MAC,
         *       currently it is not used.
         */
        /*case NETCONF_OPT_CCA_BEFORE_TX:
            reg = kw2xrf_read_dreg(MKW2XDM_PHY_CTRL1);
            reg |= MKW2XDM_PHY_CTRL1_CCABFRTX;
            kw2xrf_write_dreg(MKW2XDM_PHY_CTRL1, reg);
            return 0;*/
        case NETCONF_OPT_STATE:
                if(*((ng_netconf_state_t *)value) == NETCONF_STATE_TX) {
                    DEBUG("kw2xrf: Sending now.\n");
                    _set_sequence(dev, XCVSEQ_TRANSMIT);
                    return 0;
                }
                else if(*((ng_netconf_state_t *)value) == NETCONF_STATE_SLEEP) {
                    _set_sequence(dev, XCVSEQ_IDLE);
                    return 0;
                }
                else if(*((ng_netconf_state_t *)value) == NETCONF_STATE_IDLE) {
                    _set_sequence(dev, XCVSEQ_RECEIVE);
                    return 0;
                }
                /* TODO: Implement Off state here, when LPM functions are implemented */
        default:
            return -ENOTSUP;
    }
}

void _receive_data(kw2xrf_t *dev)
{
    uint8_t pkt_len = 0;
    ng_netif_hdr_t *hdr;
    uint8_t index = 0;
    uint8_t src_addr_len = 0;
    uint8_t dst_addr_len = 0;
    pkt_len = kw2xrf_read_dreg(MKW2XDM_RX_FRM_LEN);
    ng_pktsnip_t *payload;

    /* read PSDU */
    kw2xrf_read_fifo(dev->buf, pkt_len + 1);

    /* If RAW-mode is selected direclty forward pkt, MAC does the rest */
    if (dev->option & OPT_RAWDUMP) {
        payload = ng_pktbuf_add(NULL, NULL, pkt_len, NG_NETTYPE_UNDEF);
        if (payload == NULL ) {
            DEBUG("kw2xf: error: unable to allocate RAW data\n");
            return;
        }
        payload->data = dev->buf;
        dev->event_cb(NETDEV_EVENT_RX_COMPLETE, payload);
        return;
    }

    if((dev->buf[0] & 0x07) == 0x00) {
        DEBUG("kw2xrf: Beacon received; not handled yet -> dischard message\n");
        return;
    }
    if((dev->buf[0] & 0x07) == 0x02) {
        DEBUG("kw2xrf: ACK received; not handled yet -> dischard message\n");
        return;
    }
    if((dev->buf[0] & 0x07) == 0x03) {
        DEBUG("kw2xrf: MAC-cmd received; not handled yet -> dischard message\n");
    }
    if((dev->buf[0] & 0x07) != 0x01) {   /* No Data frame either */
        DEBUG("kw2xrf: undefined message received; -> dischard message\n");
        return;
    }
    DEBUG("kw2xrf: Message received: size %i\n", pkt_len);

    /* src 16bit addr */
    if((dev->buf[1] & 0x0c) == 0x08) {
        src_addr_len = 0x02;
    }
    /* src 64bit addr */
    else if((dev->buf[1] & 0x0c) == 0x0c) {
        src_addr_len = 0x08;
    }
    else {
        DEBUG("Bogus src address length.\n");
    }
    /* dst 16bit addr */
    if((dev->buf[1] & 0xc0) == 0x80) {
        dst_addr_len = 0x02;
    }
    /* dst 64bit addr */
    else if((dev->buf[1] & 0xc0) == 0xc0) {
        dst_addr_len = 0x08;
    }
    else {
        DEBUG("Bogus src address length.\n");
    }
    DEBUG("Src addr len: %i, Dst addr len: %i", src_addr_len, dst_addr_len);
    /* allocate a pktsnip for generic header */
    ng_pktsnip_t *hdr_snip = ng_pktbuf_add(NULL, NULL, sizeof(ng_netif_hdr_t)+
                             src_addr_len, dev->proto);
    if (hdr_snip == NULL) {
        DEBUG("kw2xrf: ERROR allocating header in packet buffer on RX\n");
        ng_pktbuf_release(hdr_snip);
        return;
    }
    hdr = (ng_netif_hdr_t *)hdr_snip->data;
    /* init generic header */
    ng_netif_hdr_init(hdr, src_addr_len, dst_addr_len);
    /* append src address into memory */
    if(hdr->dst_l2addr_len == 2) {
        ng_netif_hdr_set_src_addr(hdr, &(dev->buf[7]), hdr->src_l2addr_len);
        index = 7 + hdr->src_l2addr_len;
    }
    else {
        ng_netif_hdr_set_src_addr(hdr, &(dev->buf[11]), hdr->src_l2addr_len);
        index = 11 + hdr->src_l2addr_len;
    }
    hdr->if_pid = thread_getpid();
    hdr->rssi = kw2xrf_read_dreg(MKW2XDM_RSSI_CCA_CNT);
    hdr->lqi = dev->buf[pkt_len];

    payload = ng_pktbuf_add(hdr_snip, (void *)&(dev->buf[index + 1]),
                                 pkt_len - index, dev->proto);
    if(payload == NULL) {
        DEBUG("kw2xrf: ERROR allocating payload in packet buffer on RX\n");
        ng_pktbuf_release(hdr_snip);
        return;
    }
    dev->event_cb(NETDEV_EVENT_RX_COMPLETE, payload);
}

void _isr_event(ng_netdev_t *netdev, uint32_t event_type)
{
    kw2xrf_t *dev = (kw2xrf_t *)netdev;
    uint8_t irqst1 = kw2xrf_read_dreg(MKW2XDM_IRQSTS1);
    uint8_t irqst2 = kw2xrf_read_dreg(MKW2XDM_IRQSTS2);

    if ((irqst1 & MKW2XDM_IRQSTS1_RXIRQ) && (irqst1 & MKW2XDM_IRQSTS1_SEQIRQ)) {
        /* RX */
        DEBUG("kw2xrf: RX Int\n");
        _receive_data(dev);
        _set_sequence(dev, XCVSEQ_RECEIVE);
        kw2xrf_write_dreg(MKW2XDM_IRQSTS1, MKW2XDM_IRQSTS1_RXIRQ | MKW2XDM_IRQSTS1_SEQIRQ);
    }
    else if ((irqst1 & MKW2XDM_IRQSTS1_TXIRQ) && (irqst1 & MKW2XDM_IRQSTS1_SEQIRQ)) {
        /* TX_Complete */
        /* Device is automatically in Radio-idle state when TX is done */
        _set_sequence(dev, XCVSEQ_RECEIVE);
        DEBUG("kw2xrf: TX Complete\n");
        kw2xrf_write_dreg(MKW2XDM_IRQSTS1, MKW2XDM_IRQSTS1_TXIRQ | MKW2XDM_IRQSTS1_SEQIRQ);
    }
    else if ((irqst1 & MKW2XDM_IRQSTS1_CCAIRQ) && (irqst1 & MKW2XDM_IRQSTS1_SEQIRQ)) {
        /* TX_Started (CCA_done) */
        if (irqst2 & MKW2XDM_IRQSTS2_CCA) {
            DEBUG("kw2xrf: CCA done -> Channel busy\n");
        }
        DEBUG("kw2xrf: CCA done -> Channel idle\n");
        kw2xrf_write_dreg(MKW2XDM_IRQSTS1, MKW2XDM_IRQSTS1_CCAIRQ | MKW2XDM_IRQSTS1_SEQIRQ);
        _set_sequence(dev, XCVSEQ_RECEIVE);
    }
    else {
        /* Unknown event */
        /* Clear all interrupts to prevent ISR-loop */
        kw2xrf_write_dreg(MKW2XDM_IRQSTS1, 0x7f);
        kw2xrf_write_dreg(MKW2XDM_IRQSTS2, 0x03);
        kw2xrf_write_dreg(MKW2XDM_IRQSTS3, 0xff);
        DEBUG("kw2xrf_isr_event: unknown Interrupt\n");
        _set_sequence(dev, XCVSEQ_RECEIVE);
        return;
    }
}

uint8_t _assemble_tx_buf(kw2xrf_t *dev, ng_pktsnip_t *pkt)
{
    ng_netif_hdr_t *hdr;
    hdr = (ng_netif_hdr_t *)pkt->data;
    int index = 0;

    if (pkt == NULL) {
        return -ENOMSG;
    }
    if (dev == NULL) {
        ng_pktbuf_release(pkt);
        return -ENODEV;
    }

    /* get netif header check address length */
    hdr = (ng_netif_hdr_t *)pkt->data;
    if (!(hdr->dst_l2addr_len == 2 || hdr->dst_l2addr_len == 8)) {
        ng_pktbuf_release(pkt);
        return -ENOMSG;
    }

    /* FCF, set up data frame */
    dev->buf[1] = 0x01;
    /* set sequence number */
    dev->buf[3] = dev->seq_nr++;

    index = 4;

    if (hdr->dst_l2addr_len == 2) {
        /* set to short addressing mode */
        dev->buf[2] = 0x88;
        /* set destination pan_id */
        dev->buf[index++] = (uint8_t)((dev->radio_pan) >> 8);
        dev->buf[index++] = (uint8_t)((dev->radio_pan) & 0xff);
        /* set destination address, byte order is inverted */
        dev->buf[index++] = (ng_netif_hdr_get_dst_addr(hdr))[1];
        dev->buf[index++] = (ng_netif_hdr_get_dst_addr(hdr))[0];
        /* set source pan_id */
        dev->buf[index++] = (uint8_t)((dev->radio_pan) >> 8);
        dev->buf[index++] = (uint8_t)((dev->radio_pan) & 0xff);
        /* set source address */
        dev->buf[index++] = (uint8_t)((dev->addr_short) >> 8);
        dev->buf[index++] = (uint8_t)((dev->addr_short) & 0xff);
    }

    if (hdr->dst_l2addr_len == 8) {
        /* set destination pan_id, wireshark expects it there */
        dev->buf[index++] = (uint8_t)((dev->radio_pan) >> 8);
        dev->buf[index++] = (uint8_t)((dev->radio_pan) & 0xff);
        /* default to use long address mode for src and dst */
        dev->buf[2] |= 0xcc;
        /* set destination address located directly after ng_ifhrd_t in memory */
        memcpy(&(dev->buf)[index], ng_netif_hdr_get_dst_addr(hdr), 8);
        index += 8;
        /* set source pan_id, wireshark expects it there */
        dev->buf[index++] = (uint8_t)((dev->radio_pan) >> 8);
        dev->buf[index++] = (uint8_t)((dev->radio_pan) & 0xff);

        /* set source address */
        dev->buf[index++] = (uint8_t)((dev->addr_long) >> 56);
        dev->buf[index++] = (uint8_t)((dev->addr_long) >> 48);
        dev->buf[index++] = (uint8_t)((dev->addr_long) >> 40);
        dev->buf[index++] = (uint8_t)((dev->addr_long) >> 32);
        dev->buf[index++] = (uint8_t)((dev->addr_long) >> 24);
        dev->buf[index++] = (uint8_t)((dev->addr_long) >> 16);
        dev->buf[index++] = (uint8_t)((dev->addr_long) >> 8);
        dev->buf[index++] = (uint8_t)((dev->addr_long) & 0xff);
    }
    return index;
}


int _send(ng_netdev_t *netdev, ng_pktsnip_t *pkt)
{
    uint8_t index=0;
    kw2xrf_t *dev = (kw2xrf_t*) netdev;
    ng_pktsnip_t *payload = pkt->next;

    if (pkt == NULL) {
        return -ENOMSG;
    }
    if (netdev == NULL) {
        ng_pktbuf_release(pkt);
        return -ENODEV;
    }

    if (pkt->type == NG_NETTYPE_NETIF) {
        /* Build header and fills this already into the tx-buf */
        index = _assemble_tx_buf(dev, pkt);
        DEBUG("Assembled header for NG_NETTYPE_UNDEF to tx-buf, index: %i\n", index);
    }
    else if (pkt->type == NG_NETTYPE_UNDEF) {
        /* IEEE packet is already included in the header,
        * no need to build the header manually */
        DEBUG("Incoming packet of type NG_NETTYPE_802154: %i\n", index);
        DEBUG("size of pktsnip: %i\n", pkt->size);
        for (int i=0; i < pkt->size; i++) {
            uint8_t *tmp = pkt->data;
            dev->buf[index+i+1] = tmp[i];
        }
        /* count bytes */
        index += pkt->size;
    }
    else {
        DEBUG("This Driver does not support this type of packet\n");
        return -ENOTSUP;
    }

    while (payload) {
        /* check we don't exceed FIFO size */
        if (index+2+payload->size > KW2XRF_MAX_PKT_LENGTH) {
            ng_pktbuf_release(pkt);
            DEBUG("Packet exceeded FIFO size.\n");
            return -ENOBUFS;
        }
        for (int i=0; i < payload->size; i++) {
            uint8_t *tmp = payload->data;
            dev->buf[index+i+1] = tmp[i];
        }
        /* count bytes */
        index += payload->size;

        /* next snip */
        payload = payload->next;
    }
    dev->buf[0] = index+2; /* set packet size, reserve additional */

    /* Disable IRQ to prevent TX-ready irq while spi write is in progress */
    ng_pktbuf_release(pkt);
    DEBUG("kw2xrf: packet with size %i loaded to tx_buf\n", dev->buf[0]);
    kw2xrf_write_fifo(dev->buf, dev->buf[0]);
    if ((dev->option & OPT_PRELOADING) == NETCONF_DISABLE) {
        DEBUG("kw2xrf: Sending now.\n");
        _set_sequence(dev, XCVSEQ_TRANSMIT);
    }

    return index;
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
