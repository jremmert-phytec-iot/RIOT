/*
 * Copyright (C) 2016 PHYTEC Messtechnik GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     drivers_kw2xrf
 * @{
 * @file
 * @brief       Internal function of kw2xrf driver
 *
 * @author      Johann Fischer <j.fischer@phytec.de>
 * @}
 */

#include "panic.h"
#include "kw2xrf.h"
#include "kw2xrf_spi.h"
#include "kw2xrf_reg.h"
#include "kw2xrf_getset.h"
#include "kw2xrf_intern.h"
#include "overwrites.h"

#define ENABLE_DEBUG    (1)
#include "debug.h"

void kw2xrf_disable_interrupts(kw2xrf_t *dev)
{
    /* Clear and disable all interrupts */
    kw2xrf_write_dreg(dev, MKW2XDM_PHY_CTRL2, 0xff);
    int reg = kw2xrf_read_dreg(dev, MKW2XDM_PHY_CTRL3);
    reg |= MKW2XDM_PHY_CTRL3_WAKE_MSK | MKW2XDM_PHY_CTRL3_PB_ERR_MSK;
    kw2xrf_write_dreg(dev, MKW2XDM_PHY_CTRL3, 0x03);

    kw2xrf_write_dreg(dev, MKW2XDM_IRQSTS1, 0x7f);
    kw2xrf_write_dreg(dev, MKW2XDM_IRQSTS2, 0x03);
    kw2xrf_write_dreg(dev, MKW2XDM_IRQSTS3, 0xff);
}

/* update overwrites register */
void kw2xrf_update_overwrites(kw2xrf_t *dev)
{
    kw2xrf_write_dreg(dev, MKW2XDM_OVERWRITE_VER, overwrites_direct[0].data);
    for (uint8_t i = 0; i < sizeof(overwrites_indirect)/sizeof(overwrites_t); i++) {
        kw2xrf_write_iregs(dev, overwrites_indirect[i].address,
                           (uint8_t *)&(overwrites_indirect[i].data), 1);
    }
}

void kw2xrf_set_out_clk(kw2xrf_t *dev)
{
    /* TODO: add clock select */
    /* check modem's crystal oscillator, CLK_OUT shall be 4MHz */
    uint8_t tmp = kw2xrf_read_dreg(dev, MKW2XDM_CLK_OUT_CTRL);
    if (tmp != 0x8Bu) {
        core_panic(PANIC_GENERAL_ERROR, "Could not start MKW2XD radio transceiver");
    }
}

void kw2xrf_set_power_mode(kw2xrf_t *dev, kw2xrf_powermode_t pm)
{
    uint8_t reg = 0;
    switch (pm) {
        case KW2XRF_HIBERNATE:
            reg = 0;
            break;

        case KW2XRF_DOZE:
            reg = MKW2XDM_PWR_MODES_XTALEN;
            break;

        case KW2XRF_IDLE:
            reg = MKW2XDM_PWR_MODES_XTALEN | MKW2XDM_PWR_MODES_PMC_MODE;
            break;

        case KW2XRF_AUTODOZE:
            reg = MKW2XDM_PWR_MODES_XTALEN | MKW2XDM_PWR_MODES_AUTODOZE;
            break;
    }

    kw2xrf_write_dreg(dev, MKW2XDM_PWR_MODES, reg);
}

int kw2xrf_can_switch_to_idle(kw2xrf_t *dev)
{
    uint8_t state = kw2xrf_read_dreg(dev, MKW2XDM_SEQ_STATE);
    uint8_t seq = kw2xrf_read_dreg(dev, MKW2XDM_PHY_CTRL1) & MKW2XDM_PHY_CTRL1_XCVSEQ_MASK;
    DEBUG("[kw2xrf]: state 0x%0x, seq 0x%0x\n", state, seq);

    if ((seq != XCVSEQ_TRANSMIT) && (seq != XCVSEQ_TX_RX)) {
        return 1;
    }

    if (state != 0) {
        return 0;
    }

    return 0;
}

/** Load the timer value (Setting Current Time) */
static void kw2xrf_timer_load(kw2xrf_t *dev, uint32_t value)
{
    kw2xrf_write_dregs(dev, MKW2XDM_T1CMP_LSB, (uint8_t *)&value, sizeof(value));
    kw2xrf_set_dreg_bit(dev, MKW2XDM_PHY_CTRL4, MKW2XDM_PHY_CTRL4_TMRLOAD);
}

static uint32_t kw2xrf_timer_get(kw2xrf_t *dev)
{
    uint32_t tmp;
    kw2xrf_read_dregs(dev, MKW2XDM_EVENT_TIMER_LSB, (uint8_t*)&tmp, sizeof(tmp));
    return tmp;
}

/** Set an absolute timeout value for the given compare register of the Event Timer */
static void kw2xrf_timer_set_absolute(kw2xrf_t *dev, uint8_t cmp_reg, uint32_t value)
{
    kw2xrf_write_dregs(dev, cmp_reg, (uint8_t *)&value, 3);
}

/** Set an timeout value for the given compare register of the Event Timer */
static void kw2xrf_timer_set(kw2xrf_t *dev, uint8_t cmp_reg, uint32_t timeout)
{
    uint32_t now = kw2xrf_timer_get(dev);

    DEBUG("[kw2xrf] timer now: %" PRIx32 ", set %" PRIx32 "\n", now, now + timeout);
    kw2xrf_timer_set_absolute(dev, cmp_reg, now + timeout);
}

void kw2xrf_timer_init(kw2xrf_t *dev, kw2xrf_timer_timebase_t tb)
{
    uint8_t tmp = MKW2XDMI_TMR_PRESCALE_SET(tb);

    kw2xrf_write_iregs(dev, MKW2XDMI_TMR_PRESCALE, &tmp, 1);
    kw2xrf_timer_load(dev, 0);
}

void kw2xrf_timer2_seq_start_on(kw2xrf_t *dev)
{
    kw2xrf_set_dreg_bit(dev, MKW2XDM_PHY_CTRL1, MKW2XDM_PHY_CTRL1_TMRTRIGEN);
}

void kw2xrf_timer2_seq_start_off(kw2xrf_t *dev)
{
    kw2xrf_clear_dreg_bit(dev, MKW2XDM_PHY_CTRL1, MKW2XDM_PHY_CTRL1_TMRTRIGEN);
}

void kw2xrf_timer3_seq_abort_on(kw2xrf_t *dev)
{
    kw2xrf_set_dreg_bit(dev, MKW2XDM_PHY_CTRL4, MKW2XDM_PHY_CTRL4_TC3TMOUT);
}

void kw2xrf_timer3_seq_abort_off(kw2xrf_t *dev)
{
    kw2xrf_clear_dreg_bit(dev, MKW2XDM_PHY_CTRL4, MKW2XDM_PHY_CTRL4_TC3TMOUT);
}

void kw2xrf_trigger_tx_ops_enable(kw2xrf_t *dev, uint32_t timeout)
{
    kw2xrf_timer_set(dev, MKW2XDM_T2CMP_LSB, timeout);
    kw2xrf_set_dreg_bit(dev, MKW2XDM_PHY_CTRL3, MKW2XDM_PHY_CTRL3_TMR2CMP_EN);
}

void kw2xrf_trigger_tx_ops_disable(kw2xrf_t *dev)
{
    kw2xrf_clear_dreg_bit(dev, MKW2XDM_PHY_CTRL3, MKW2XDM_PHY_CTRL3_TMR2CMP_EN);
    kw2xrf_write_dreg(dev, MKW2XDM_IRQSTS3, MKW2XDM_IRQSTS3_TMR2IRQ);
    uint32_t now = kw2xrf_timer_get(dev);
    DEBUG("[kw2xrf] now: %" PRIx32 "\n", now);
}

void kw2xrf_abort_rx_ops_enable(kw2xrf_t *dev, uint32_t timeout)
{
    kw2xrf_timer_set(dev, MKW2XDM_T3CMP_LSB, timeout);
    kw2xrf_set_dreg_bit(dev, MKW2XDM_PHY_CTRL3, MKW2XDM_PHY_CTRL3_TMR3CMP_EN);
}

void kw2xrf_abort_rx_ops_disable(kw2xrf_t *dev)
{
    kw2xrf_clear_dreg_bit(dev, MKW2XDM_PHY_CTRL3, MKW2XDM_PHY_CTRL3_TMR3CMP_EN);
    kw2xrf_write_dreg(dev, MKW2XDM_IRQSTS3, MKW2XDM_IRQSTS3_TMR3IRQ);
    uint32_t now = kw2xrf_timer_get(dev);
    DEBUG("[kw2xrf] now: %" PRIx32 "\n", now);
}

void kw2xrf_seq_timeout_on(kw2xrf_t *dev, uint32_t timeout)
{
    kw2xrf_mask_irq_b(dev);
    kw2xrf_timer_set(dev, MKW2XDM_T4CMP_LSB, timeout);

    /* enable and clear irq for timer 3 */
    uint8_t irqsts3 = kw2xrf_read_dreg(dev, MKW2XDM_IRQSTS3) & 0xf0;
    irqsts3 &= ~MKW2XDM_IRQSTS3_TMR4MSK;
    irqsts3 |= MKW2XDM_IRQSTS3_TMR4IRQ;
    kw2xrf_write_dreg(dev, MKW2XDM_IRQSTS3, irqsts3);

    kw2xrf_set_dreg_bit(dev, MKW2XDM_PHY_CTRL3, MKW2XDM_PHY_CTRL3_TMR4CMP_EN);
    kw2xrf_enable_irq_b(dev);
}

void kw2xrf_seq_timeout_off(kw2xrf_t *dev)
{
    kw2xrf_clear_dreg_bit(dev, MKW2XDM_PHY_CTRL3, MKW2XDM_PHY_CTRL3_TMR4CMP_EN);
    kw2xrf_write_dreg(dev, MKW2XDM_IRQSTS3, MKW2XDM_IRQSTS3_TMR4IRQ);
    uint32_t now = kw2xrf_timer_get(dev);
    DEBUG("[kw2xrf] now: %" PRIx32 "\n", now);
}

uint32_t kw2xrf_get_timestamp(kw2xrf_t *dev)
{
    uint32_t tmp;
    kw2xrf_read_dregs(dev, MKW2XDM_TIMESTAMP_LSB, (uint8_t*)&tmp, sizeof(tmp));
    return tmp;
}
