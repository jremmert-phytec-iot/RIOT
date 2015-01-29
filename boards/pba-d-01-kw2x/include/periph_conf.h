/*
 * Copyright (C) 2014 Freie Universität Berlin
 * Copyright (C) 2014 PHYTEC Messtechnik GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     board_pba-d-01-kw2x
 * @{
 *
 * @file
 * @name        Peripheral MCU configuration for the phyWAVE-KW22 Board
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Johann Fischer <j.fischer@phytec.de>
 * @author      Jonas Remmert <j.remmert@phytec.de>
 */

#ifndef __PERIPH_CONF_H
#define __PERIPH_CONF_H

#include "cpu-conf.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @name Clock system configuration
 * @{
 */
#define KINETIS_CPU_USE_MCG               1

#define KINETIS_MCG_USE_ERC               1
#define KINETIS_MCG_USE_PLL               1
#define KINETIS_MCG_DCO_RANGE             (24000000U)
#define KINETIS_MCG_ERC_OSCILLATOR        0
#define KINETIS_MCG_ERC_FRDIV             2
#define KINETIS_MCG_ERC_RANGE             1
#define KINETIS_MCG_ERC_FREQ              4000000
#define KINETIS_MCG_PLL_PRDIV             1
#define KINETIS_MCG_PLL_VDIV0             0
#define KINETIS_MCG_PLL_FREQ              48000000

#define CLOCK_CORECLOCK                   KINETIS_MCG_PLL_FREQ
/** @} */


/**
 * @name Timer configuration
 * @{
 */
#define TIMER_NUMOF                       (1U)
#define TIMER_0_EN                        1
#define TIMER_1_EN                        0
#define TIMER_IRQ_PRIO                    1
#define TIMER_DEV                         PIT
#define TIMER_MAX_VALUE                   (0xffffffff)
#define TIMER_CLOCK                       CLOCK_CORECLOCK
#define TIMER_CLKEN()                     (SIM->SCGC6 |= (SIM_SCGC6_PIT_MASK))

/* Timer 0 configuration */
#define TIMER_0_PRESCALER_CH              0
#define TIMER_0_COUNTER_CH                1
#define TIMER_0_ISR                       isr_pit1
#define TIMER_0_IRQ_CHAN                  PIT1_IRQn

/* Timer 1 configuration */
#define TIMER_1_PRESCALER_CH              2
#define TIMER_1_COUNTER_CH                3
#define TIMER_1_ISR                       isr_pit3
#define TIMER_1_IRQ_CHAN                  PIT3_IRQn
/** @} */


/**
 * @name UART configuration
 * @{
 */
#define UART_NUMOF          (1U)
#define UART_0_EN           1
#define UART_1_EN           0
#define UART_IRQ_PRIO       1
#define UART_CLK            (48e6)

/* UART 0 device configuration */
#define KINETIS_UART        UART_Type
#define UART_0_DEV          UART2
#define UART_0_CLKEN()      (SIM->SCGC4 |= (SIM_SCGC4_UART2_MASK))
#define UART_0_CLK          UART_CLK
#define UART_0_IRQ_CHAN     UART2_RX_TX_IRQn
#define UART_0_ISR          isr_uart2_rx_tx
/* UART 0 pin configuration */
#define UART_0_PORT_CLKEN() (SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK))
#define UART_0_PORT         PORTD
#define UART_0_RX_PIN       2
#define UART_0_TX_PIN       3
#define UART_0_AF           3

/* UART 1 device configuration */
#define UART_1_DEV          UART0
#define UART_1_CLKEN()      (SIM->SCGC4 |= (SIM_SCGC4_UART0_MASK))
#define UART_1_CLK          UART_CLK
#define UART_1_IRQ_CHAN     UART0_RX_TX_IRQn
#define UART_1_ISR          isr_uart0_rx_tx
/* UART 1 pin configuration */
#define UART_1_PORT_CLKEN() (SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK))
#define UART_1_PORT         PORTD
#define UART_1_RX_PIN       6
#define UART_1_TX_PIN       7
#define UART_1_AF           3
/** @} */


/**
 * @name ADC configuration
 * @{
 */
#define ADC_NUMOF           (1U)
#define ADC_0_EN            1
#define ADC_MAX_CHANNELS    6

/* ADC 0 configuration */
#define ADC_0_DEV           ADC0
#define ADC_0_CHANNELS      6
#define ADC_0_CLKEN()       (SIM->SCGC6 |= (SIM_SCGC6_ADC0_MASK))
#define ADC_0_CLKDIS()      (SIM->SCGC6 &= ~(SIM_SCGC6_ADC0_MASK))
#define ADC_0_PORT_CLKEN()  (SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK))
/* ADC 0 channel 0 pin config */
#define ADC_0_CH0           1
#define ADC_0_CH0_PIN       2
#define ADC_0_CH0_PIN_AF    0
#define ADC_0_CH0_PORT      PORTE
/* ADC 0 channel 1 pin config */
#define ADC_0_CH1           1 /* PTE3 uses the same ADC_CH as PTE2, in single channel mode only one of them can be selected */
#define ADC_0_CH1_PIN       3
#define ADC_0_CH1_PIN_AF    0
#define ADC_0_CH1_PORT      PORTE
/* ADC 0 channel 2 pin config */
#define ADC_0_CH2           22
#define ADC_0_CH2_PIN       7
#define ADC_0_CH2_PIN_AF    0
#define ADC_0_CH2_PORT      PORTD
/* ADC 0 channel 3 pin config */
#define ADC_0_CH3           6
#define ADC_0_CH3_PIN       5
#define ADC_0_CH3_PIN_AF    0
#define ADC_0_CH3_PORT      PORTD
/* ADC 0 channel 4 pin config */
#define ADC_0_CH4           10
#define ADC_0_CH4_PIN       0
#define ADC_0_CH4_PIN_AF    0
#define ADC_0_CH4_PORT      PORTE
/* ADC 0 channel 5 pin config */
#define ADC_0_CH5           11
#define ADC_0_CH5_PIN       1
#define ADC_0_CH5_PIN_AF    0
#define ADC_0_CH5_PORT      PORTE
/** @} */


/**
 * @name PWM configuration
 * @{
 */
#define PWM_NUMOF           (1U)
#define PWM_0_EN            1
#define PWM_MAX_CHANNELS    4

/* PWM 0 device configuration */
#define PWM_0_DEV           FTM0
#define PWM_0_CHANNELS      3
#define PWM_0_CLK           (48e6)
#define PWM_0_CLKEN()       (SIM->SCGC6 |= (SIM_SCGC6_FTM0_MASK))
#define PWM_0_CLKDIS()      (SIM->SCGC6 &= ~(SIM_SCGC6_FTM0_MASK))
/* PWM 0 pin configuration */
#define PWM_0_PORT_CLKEN()  (SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTA_MASK))

#define PWM_0_PIN_CH0       4
#define PWM_0_FTMCHAN_CH0   1
#define PWM_0_PORT_CH0      PORTA
#define PWM_0_PIN_AF_CH0    3

#define PWM_0_PIN_CH1       4
#define PWM_0_FTMCHAN_CH1   4
#define PWM_0_PORT_CH1      PORTD
#define PWM_0_PIN_AF_CH1    4

#define PWM_0_PIN_CH2       6
#define PWM_0_FTMCHAN_CH2   6
#define PWM_0_PORT_CH2      PORTD
#define PWM_0_PIN_AF_CH2    4

#define PWM_0_PIN_CH3       1
#define PWM_0_FTMCHAN_CH3   1
#define PWM_0_PORT_CH3      PORTA
#define PWM_0_PIN_AF_CH3    3
/** @} */


/**
 * @name SPI configuration
 * @{
 */
#define SPI_NUMOF           (1U)
#define SPI_0_EN            1
#define SPI_IRQ_PRIO        1

/* SPI 0 device config */
#define SPI_0_DEV               SPI0
#define SPI_0_INDEX             0
#define SPI_0_CTAS              0
#define SPI_0_CLKEN()           (SIM->SCGC6 |= (SIM_SCGC6_SPI0_MASK))
#define SPI_0_CLKDIS()          (SIM->SCGC6 &= ~(SIM_SCGC6_SPI0_MASK))
#define SPI_0_IRQ               SPI0_IRQn
#define SPI_0_IRQ_HANDLER       isr_spi0
#define SPI_0_FREQ              (48e6)

/* SPI 0 pin configuration */
#define SPI_0_PORT              PORTC
#define SPI_0_PORT_CLKEN()      (SIM->SCGC5 |= (SIM_SCGC5_PORTC_MASK))
#define SPI_0_AF                2

#define SPI_0_PCS0_PIN          4
#define SPI_0_SCK_PIN           5
#define SPI_0_SOUT_PIN          6
#define SPI_0_SIN_PIN           7

#define SPI_0_PCS0_ACTIVE_LOW   1
/** @} */


/**
 * @name I2C configuration
 * @{
 */
#define I2C_NUMOF               (1U)
#define I2C_CLK                 (48e6)
#define I2C_0_EN                1
#define I2C_IRQ_PRIO            1

/* I2C 0 device configuration */
#define I2C_0_DEV               I2C1
#define I2C_0_CLKEN()           (SIM->SCGC4 |= (SIM_SCGC4_I2C1_MASK))
#define I2C_0_CLKDIS()          (SIM->SCGC4 &= ~(SIM_SCGC4_I2C1_MASK))
#define I2C_0_IRQ               I2C1_IRQn
#define I2C_0_IRQ_HANDLER       isr_i2c1
/* I2C 0 pin configuration */
#define I2C_0_PORT              PORTE
#define I2C_0_PORT_CLKEN()      (SIM->SCGC5 |= (SIM_SCGC5_PORTE_MASK))
#define I2C_0_PIN_AF            6
#define I2C_0_SDA_PIN           0
#define I2C_0_SCL_PIN           1
#define I2C_0_PORT_CFG          (PORT_PCR_MUX(I2C_0_PIN_AF) | PORT_PCR_ODE_MASK)

/** @} */


/**
 * @name GPIO configuration
 * @{
 */
#define GPIO_NUMOF          24
#define GPIO_0_EN           0        /* SPI0_CS0 */
#define GPIO_1_EN           0        /* SPI0_CLK */
#define GPIO_2_EN           0        /* SPI0_MOSI */
#define GPIO_3_EN           0        /* SPI0_MISO */
#define GPIO_4_EN           0        /* UART2_RX */
#define GPIO_5_EN           0        /* UART2_TX */
#define GPIO_6_EN           0
#define GPIO_7_EN           0
#define GPIO_8_EN           0        /* I2CSDA */
#define GPIO_9_EN           0        /* I2CSCL */
#define GPIO_10_EN          0
#define GPIO_11_EN          0
#define GPIO_12_EN          0
#define GPIO_13_EN          0        /* USB VOUT 3V3 */
#define GPIO_14_EN          0        /* USB VREGIN */
#define GPIO_15_EN          0
#define GPIO_16_EN          0
#define GPIO_17_EN          0
#define GPIO_18_EN          0
#define GPIO_19_EN          1
#define GPIO_20_EN          1
#define GPIO_21_EN          1
#define GPIO_22_EN          1        /* User Button, use as input */
#define GPIO_23_EN          1        /* KW2XRF INT */
#define GPIO_IRQ_PRIO       1
#define ISR_PORT_A          isr_porta
#define ISR_PORT_B          isr_portb
#define ISR_PORT_C          isr_portc
#define ISR_PORT_D          isr_portd

/* GPIO channel 0 config */
#define GPIO_0_DEV          GPIOC    /* DIO2; extension connecotr D10; SPI0_CS0 */
#define GPIO_0_PORT         PORTC
#define GPIO_0_PORT_BASE    PORTC_BASE
#define GPIO_0_PIN          4
#define GPIO_0_CLKEN()      (SIM->SCGC5 |= (SIM_SCGC5_PORTC_MASK))
#define GPIO_0_IRQ          PORTC_IRQn
/* GPIO channel 1 config */
#define GPIO_1_DEV          GPIOC    /* DIO3; extension connecotr D13; SPI0_CLK */
#define GPIO_1_PORT         PORTC
#define GPIO_1_PORT_BASE    PORTC_BASE
#define GPIO_1_PIN          5
#define GPIO_1_CLKEN()      (SIM->SCGC5 |= (SIM_SCGC5_PORTC_MASK))
#define GPIO_1_IRQ          PORTC_IRQn
/* GPIO channel 2 config */
#define GPIO_2_DEV          GPIOC    /* DIO4; extension connecotr D11;  SPI0_MOSI */
#define GPIO_2_PORT         PORTC
#define GPIO_2_PORT_BASE    PORTC_BASE
#define GPIO_2_PIN          6
#define GPIO_2_CLKEN()      (SIM->SCGC5 |= (SIM_SCGC5_PORTC_MASK))
#define GPIO_2_IRQ          PORTC_IRQn
/* GPIO channel 3 config */
#define GPIO_3_DEV          GPIOC    /* DIO5; extension connecotr D12;  SPI0_MISO */
#define GPIO_3_PORT         PORTC
#define GPIO_3_PORT_BASE    PORTC_BASE
#define GPIO_3_PIN          7
#define GPIO_3_CLKEN()      (SIM->SCGC5 |= (SIM_SCGC5_PORTC_MASK))
#define GPIO_3_IRQ          PORTC_IRQn
/* GPIO channel 4 config */
#define GPIO_4_DEV          GPIOD    /* DIO7; extension connecotr D0;  UART2_RX */
#define GPIO_4_PORT         PORTD
#define GPIO_4_PORT_BASE    PORTD_BASE
#define GPIO_4_PIN          2
#define GPIO_4_CLKEN()      (SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK))
#define GPIO_4_IRQ          PORTD_IRQn
/* GPIO channel 5 config */
#define GPIO_5_DEV          GPIOD    /* DIO8; extension connecotr D1;  UART2_TX */
#define GPIO_5_PORT         PORTD
#define GPIO_5_PORT_BASE    PORTD_BASE
#define GPIO_5_PIN          3
#define GPIO_5_CLKEN()      (SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK))
#define GPIO_5_IRQ          PORTD_IRQn
/* GPIO channel 6 config */
#define GPIO_6_DEV          GPIOD    /* DIO10; extension connecotr A3 */
#define GPIO_6_PORT         PORTD
#define GPIO_6_PORT_BASE    PORTD_BASE
#define GPIO_6_PIN          5
#define GPIO_6_CLKEN()      (SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK))
#define GPIO_6_IRQ          PORTD_IRQn
/* GPIO channel 7 config */
#define GPIO_7_DEV          GPIOD    /* DIO12; extension connecotr A2 */
#define GPIO_7_PORT         PORTD
#define GPIO_7_PORT_BASE    PORTD_BASE
#define GPIO_7_PIN          7
#define GPIO_7_CLKEN()      (SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK))
#define GPIO_7_IRQ          PORTD_IRQn
/* GPIO channel 8 config */
#define GPIO_8_DEV          GPIOE    /* DIO13; extension connecotr A4;  I2CSDA */
#define GPIO_8_PORT         PORTE
#define GPIO_8_PORT_BASE    PORTE_BASE
#define GPIO_8_PIN          0
#define GPIO_8_CLKEN()      (SIM->SCGC5 |= (SIM_SCGC5_PORTE_MASK))
#define GPIO_8_IRQ          PORTE_IRQn
/* GPIO channel 9 config */
#define GPIO_9_DEV          GPIOE    /* DIO14; extension connecotr A5;  I2CSCL */
#define GPIO_9_PORT         PORTE
#define GPIO_9_PORT_BASE    PORTE_BASE
#define GPIO_9_PIN          1
#define GPIO_9_CLKEN()      (SIM->SCGC5 |= (SIM_SCGC5_PORTE_MASK))
#define GPIO_9_IRQ          PORTE_IRQn
/* GPIO channel 10 config */
#define GPIO_10_DEV          GPIOE   /* DIO15; extension connecotr A0 */
#define GPIO_10_PORT         PORTE
#define GPIO_10_PORT_BASE    PORTE_BASE
#define GPIO_10_PIN          2
#define GPIO_10_CLKEN()      (SIM->SCGC5 |= (SIM_SCGC5_PORTE_MASK))
#define GPIO_10_IRQ          PORTE_IRQn
/* GPIO channel 11 config */
#define GPIO_11_DEV          GPIOE   /* DIO16; extension connecotr A1 */
#define GPIO_11_PORT         PORTE
#define GPIO_11_PORT_BASE    PORTE_BASE
#define GPIO_11_PIN          3
#define GPIO_11_CLKEN()      (SIM->SCGC5 |= (SIM_SCGC5_PORTE_MASK))
#define GPIO_11_IRQ          PORTE_IRQn
/* GPIO channel 12 config */
#define GPIO_12_DEV          GPIOE   /* DIO17; extension connecotr D2 */
#define GPIO_12_PORT         PORTE
#define GPIO_12_PORT_BASE    PORTE_BASE
#define GPIO_12_PIN          4
#define GPIO_12_CLKEN()      (SIM->SCGC5 |= (SIM_SCGC5_PORTE_MASK))
#define GPIO_12_IRQ          PORTE_IRQn
/* GPIO channel 13 config */
#define GPIO_13_DEV          GPIOE   /* DIO20; extension connecotr D14, USB OUT3V3 */
#define GPIO_13_PORT         PORTE
#define GPIO_13_PORT_BASE    PORTE_BASE
#define GPIO_13_PIN          18
#define GPIO_13_CLKEN()      (SIM->SCGC5 |= (SIM_SCGC5_PORTE_MASK))
#define GPIO_13_IRQ          PORTE_IRQn
/* GPIO channel 14 config */
#define GPIO_14_DEV          GPIOE   /* DIO21; extension connecotr D15, USB VREGIN */
#define GPIO_14_PORT         PORTE
#define GPIO_14_PORT_BASE    PORTE_BASE
#define GPIO_14_PIN          19
#define GPIO_14_CLKEN()      (SIM->SCGC5 |= (SIM_SCGC5_PORTE_MASK))
#define GPIO_14_IRQ          PORTE_IRQn
/* GPIO channel 15 config */
#define GPIO_15_DEV          GPIOA   /* DIO24; extension connecotr D7 */
#define GPIO_15_PORT         PORTA
#define GPIO_15_PORT_BASE    PORTA_BASE
#define GPIO_15_PIN          1
#define GPIO_15_CLKEN()      (SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK))
#define GPIO_15_IRQ          PORTA_IRQn
/* GPIO channel 16 config */
#define GPIO_16_DEV          GPIOA   /* DIO25; extension connecotr D6 */
#define GPIO_16_PORT         PORTA
#define GPIO_16_PORT_BASE    PORTA_BASE
#define GPIO_16_PIN          2
#define GPIO_16_CLKEN()      (SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK))
#define GPIO_16_IRQ          PORTA_IRQn
/* GPIO channel 17 config */
#define GPIO_17_DEV          GPIOA   /* DIO28; extension connecotr D8 */
#define GPIO_17_PORT         PORTA
#define GPIO_17_PORT_BASE    PORTA_BASE
#define GPIO_17_PIN          18
#define GPIO_17_CLKEN()      (SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK))
#define GPIO_17_IRQ          PORTA_IRQn
/* GPIO channel 18 config */
#define GPIO_18_DEV          GPIOA   /* DIO29; extension connecotr D4 */
#define GPIO_18_PORT         PORTA
#define GPIO_18_PORT_BASE    PORTA_BASE
#define GPIO_18_PIN          19
#define GPIO_18_CLKEN()      (SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK))
#define GPIO_18_IRQ          PORTA_IRQn
/* GPIO channel 19 config */
#define GPIO_19_DEV          GPIOD   /* DIO9; extension connecotr D9; LED_G */
#define GPIO_19_PORT         PORTD
#define GPIO_19_PORT_BASE    PORTD_BASE
#define GPIO_19_PIN          4
#define GPIO_19_CLKEN()      (SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK))
#define GPIO_19_IRQ          PORTD_IRQn
/* GPIO channel 20 config */
#define GPIO_20_DEV          GPIOD   /* DIO11; extension connecotr D5; LED_R */
#define GPIO_20_PORT         PORTD
#define GPIO_20_PORT_BASE    PORTD_BASE
#define GPIO_20_PIN          6
#define GPIO_20_CLKEN()      (SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK))
#define GPIO_20_IRQ          PORTD_IRQn
/* GPIO channel 21 config */
#define GPIO_21_DEV          GPIOA   /* DIO27; extension connecotr D3; LED_B */
#define GPIO_21_PORT         PORTA
#define GPIO_21_PORT_BASE    PORTA_BASE
#define GPIO_21_PIN          4
#define GPIO_21_CLKEN()      (SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK))
#define GPIO_21_IRQ          PORTA_IRQn
/* GPIO channel 22 config */
#define GPIO_22_DEV          GPIOD   /* DIO06; extension connecotr --; User_Button */
#define GPIO_22_PORT         PORTD
#define GPIO_22_PORT_BASE    PORTD_BASE
#define GPIO_22_PIN          1
#define GPIO_22_CLKEN()      (SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK))
#define GPIO_22_IRQ          PORTD_IRQn
/* GPIO channel 23 config */
#define GPIO_23_DEV          KW2XDRF_GPIO
#define GPIO_23_PORT         KW2XDRF_PORT
#define GPIO_23_PORT_BASE    KW2XDRF_PORT_BASE
#define GPIO_23_PIN          KW2XDRF_IRQ_PIN
#define GPIO_23_CLKEN()      KW2XDRF_PORT_CLKEN()
#define GPIO_23_IRQ          KW2XDRF_PORT_IRQn
#define GPIO_KW2XDRF         GPIO_23
/** @} */

/**
* @name RTC configuration
* @{
*/
#define RTC_NUMOF            (1U)
#define RTC_DEV              RTC
#define RTC_UNLOCK()         (SIM->SCGC6 |= (SIM_SCGC6_RTC_MASK))
/** @} */

/**
 * @name Random Number Generator configuration
 * @{
 */
#define RANDOM_NUMOF         (1U)
#define KINETIS_RNGA         RNG
#define RANDOM_CLKEN()       (SIM->SCGC6 |= (1 << 9))
#define RANDOM_CLKDIS()      (SIM->SCGC6 &= ~(1 << 9))

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __PERIPH_CONF_H */
/** @} */
