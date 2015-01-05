/*
 * Copyright (C) 2014 Freie Universität Berlin
 * Copyright (C) 2014 PHYTEC Messtechnik GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_kinetis_common_adc
 * @{
 *
 * @file
 * @brief       Low-level ADC driver implementation
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Johann Fischer <j.fischer@phytec.de>
 * @author      Jonas Remmert <j.remmert@phytec.de>
 *
 * @}
 */
#include <stdio.h>

#include "cpu.h"
#include "periph/adc.h"
#include "periph_conf.h"

/* guard in case that no ADC device is defined */
#if ADC_NUMOF

typedef struct {
    int max_value;
} adc_config_t;

adc_config_t adc_config[ADC_NUMOF];

int adc_init(adc_t dev, adc_precision_t precision)
{
    ADC_Type *adc = 0;
    PORT_Type *port[ADC_MAX_CHANNELS];
    uint8_t pins[ADC_MAX_CHANNELS];
    uint8_t af[ADC_MAX_CHANNELS];
    adc_poweron(dev);
    int channels = 0;

    switch (dev) {
#if ADC_0_EN

        case ADC_0:
            adc = ADC_0_DEV;
            port[0] = ADC_0_CH0_PORT;
            port[1] = ADC_0_CH1_PORT;
            port[2] = ADC_0_CH2_PORT;
            port[3] = ADC_0_CH3_PORT;
            port[4] = ADC_0_CH4_PORT;
            port[5] = ADC_0_CH5_PORT;
            pins[0] = ADC_0_CH0_PIN;
            pins[1] = ADC_0_CH1_PIN;
            pins[2] = ADC_0_CH2_PIN;
            pins[3] = ADC_0_CH3_PIN;
            pins[4] = ADC_0_CH4_PIN;
            pins[5] = ADC_0_CH5_PIN;
            af[0] = ADC_0_CH0_PIN_AF;
            af[1] = ADC_0_CH1_PIN_AF;
            af[2] = ADC_0_CH2_PIN_AF;
            af[3] = ADC_0_CH3_PIN_AF;
            af[4] = ADC_0_CH4_PIN_AF;
            af[5] = ADC_0_CH5_PIN_AF;
            channels = ADC_0_CHANNELS;
            ADC_0_PORT_CLKEN();
            break;
#endif

        default:
            return -1;
    }

    if (channels > ADC_MAX_CHANNELS) {
        return -1;
    }

    for (int i = 0; i < channels; i++) {
        port[i]->PCR[pins[i]] = PORT_PCR_MUX(af[i]);
        //adc->adcchan
        //adc->CONTROLS[i].CnV = 0;
    }

    /* set control registers */
    adc->CFG1 = ADC_CFG1_ADIV(2) | ADC_CFG1_ADLSMP_MASK | ADC_CFG1_MODE(2);

    /* select ADxxb channels, high speed operation */
    adc->CFG2 = ADC_CFG2_MUXSEL_MASK | ADC_CFG2_ADHSC_MASK;

    adc->CV1 = 42;
    adc->CV2 = 666;

    /* select software trigger, external ref pins */
    adc->SC2 = ADC_SC2_REFSEL(0);

    /* select hardware average over 32 samples */
    adc->SC3 = ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(3);

    /* select input channel */
    adc->SC1[0] = ADC_SC1_ADCH(ADC_0_CH0);
    adc->SC1[1] = ADC_SC1_ADCH(ADC_0_CH0);


    /* set precision */
    switch (precision) {
        case ADC_RES_6BIT:
        case ADC_RES_8BIT:
            adc_poweroff(dev);
            return -1;
            break;

        case ADC_RES_10BIT:
        case ADC_RES_12BIT:
        case ADC_RES_14BIT:
        case ADC_RES_16BIT:
            break;
    }

    return 0;
}

int adc_sample(adc_t dev, int channel)
{
    ADC_Type *adc = 0;

    switch (dev) {
#if ADC_0_EN

        case ADC_0:
            adc = ADC_0_DEV;

            /* start single conversion on corresponding channel */
            switch (channel) {
                case 0:
                    adc->SC1[0] = ADC_SC1_ADCH(ADC_0_CH0);
                    break;

                case 1:
                    adc->SC1[0] = ADC_SC1_ADCH(ADC_0_CH1);
                    break;

                case 2:
                    adc->SC1[0] = ADC_SC1_ADCH(ADC_0_CH2);
                    break;

                case 3:
                    adc->SC1[0] = ADC_SC1_ADCH(ADC_0_CH3);
                    break;

                case 4:
                    adc->SC1[0] = ADC_SC1_ADCH(ADC_0_CH4);
                    break;

                case 5:
                    adc->SC1[0] = ADC_SC1_ADCH(ADC_0_CH5);
                    break;

                default:
                    return -1;
            }

            break;
#endif
    }

    /* wait until conversion is complete */
    while (!(adc->SC1[0] & ADC_SC1_COCO_MASK));

    /* read and return result */
    return (int)adc->R[0];
}

void adc_poweron(adc_t dev)
{
    switch (dev) {
#if ADC_0_EN

        case ADC_0:
            ADC_0_CLKEN();
            break;
#endif
#if ADC_1_EN

        case ADC_1:
            ADC_1_CLKEN();
            break;
#endif
    }
}

void adc_poweroff(adc_t dev)
{
    switch (dev) {
#if ADC_0_EN

        case ADC_0:
            ADC_0_CLKDIS();
            break;
#endif
#if ADC_1_EN

        case ADC_1:
            ADC_1_CLKDIS();
            break;
#endif
    }
}

int adc_map(adc_t dev, int value, int min, int max)
{
    return (int)adc_mapf(dev, value, (float)min, (float)max);
}

float adc_mapf(adc_t dev, int value, float min, float max)
{
    return ((max - min) / ((float)adc_config[dev].max_value)) * value;
}

#endif /* ADC_NUMOF */
