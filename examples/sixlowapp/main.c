/*
 * Copyright (C) 2014 INRIA
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @file
 * @brief       6LoWPAN example application - main function
 *
 * @author      Oliver Hahm <oliver.hahm@inria.fr>
 */

#include <stdio.h>

#include "kernel.h"
#include "thread.h"
#include "net_if.h"
#include "posix_io.h"
#include "shell.h"
#include "shell_commands.h"
#include "board_uart0.h"

#include "hdc1000.h"
#include "tmp006.h"
#include "mpl3115a2.h"
#include "mag3110.h"
#include "mma8652.h"

#include "sixlowapp.h"

#define RCV_BUFFER_SIZE     (32)

kernel_pid_t hack_msg_pid;

static uint8_t sensor_stat;

#define SENSOR_ENABLED_HDC1000    0x01
#define SENSOR_ENABLED_TMP006     0x02
#define SENSOR_ENABLED_MPL3115    0x04
#define SENSOR_ENABLED_MAG3110    0x08
#define SENSOR_ENABLED_MMA8652    0x10
#define SENSOR_ENABLED_TCS37727   0x20

static hdc1000_t hdc1000_dev;
static tmp006_t tmp006_dev;
static mpl3115a2_t mpl3115a2_dev;
static mag3110_t mag3110_dev;
static mma8652_t mma8652_dev;

int temp, hum;
float tamb, tobj;
uint32_t pressure;
int16_t m_x, m_y, m_z;
int16_t a_x, a_y, a_z;

kernel_pid_t sixlowapp_udp_server_pid = KERNEL_PID_UNDEF;

char addr_str[IPV6_MAX_ADDR_STR_LEN];
char monitor_stack_buffer[KERNEL_CONF_STACKSIZE_MAIN];
char udp_server_stack_buffer[KERNEL_CONF_STACKSIZE_MAIN];

const shell_command_t shell_commands[] = {
    {"ping", "Send an ICMPv6 echo request to another node", sixlowapp_send_ping},
    {"nc", "RIOT netcat - arbitrary UDP connections and listens", sixlowapp_netcat},
    {NULL, NULL, NULL}
};

static char sensor_server_stack_buffer[KERNEL_CONF_STACKSIZE_MAIN];
static char hack_msg_server_stack_buffer[KERNEL_CONF_STACKSIZE_MAIN];

void sensors_init(void)
{
    sensor_stat = 0;

    if (hdc1000_init(&hdc1000_dev, I2C_0, 0x43) == 0) {
        sensor_stat |= SENSOR_ENABLED_HDC1000;
    }

    if (tmp006_init(&tmp006_dev, I2C_0, 0x41, TMP006_CONFIG_CR_DEF) == 0) {
        sensor_stat |= SENSOR_ENABLED_TMP006;
    }

    if (mpl3115a2_init(&mpl3115a2_dev, I2C_0, 0x60, MPL3115A2_OS_RATIO_DEFAULT) == 0) {
        sensor_stat |= SENSOR_ENABLED_MPL3115;
        if (mpl3115a2_set_active(&mpl3115a2_dev)) {
            sensor_stat &= ~SENSOR_ENABLED_MPL3115;
        }
    }

    if (mag3110_init(&mag3110_dev, I2C_0, 0x0e, MAG3110_DROS_DEFAULT) == 0) {
        sensor_stat |= SENSOR_ENABLED_MAG3110;
        if (mag3110_set_active(&mag3110_dev)) {
            sensor_stat &= ~SENSOR_ENABLED_MAG3110;
        }
    }

    if (mma8652_init(&mma8652_dev, I2C_0, 0x1d, MMA8652_DATARATE_DEFAULT,
                     MMA8652_FS_RANGE_DEFAULT) == 0) {
        sensor_stat |= SENSOR_ENABLED_MMA8652;
        if (mma8652_set_active(&mma8652_dev)) {
            sensor_stat &= ~SENSOR_ENABLED_MMA8652;
        }
    }
    printf("sensor_stat: 0x%x\n", sensor_stat);
}

static void *sensor_server(void *arg)
{
    (void) arg;
    uint8_t status;

    sensors_init();

    timex_t sleep = timex_set(1, 0);

    while (1) {
        if (sensor_stat & SENSOR_ENABLED_HDC1000) {
            uint16_t rawtemp, rawhum;

            if (hdc1000_startmeasure(&hdc1000_dev)) {
                sensor_stat &= ~SENSOR_ENABLED_HDC1000;
            }
            vtimer_usleep(HDC1000_CONVERSION_TIME);

            hdc1000_read(&hdc1000_dev, &rawtemp, &rawhum);
            hdc1000_convert(rawtemp, rawhum,  &temp, &hum);
        }

        if (sensor_stat & SENSOR_ENABLED_TMP006) {
            int16_t rawtemp, rawvolt;
            uint8_t drdy;

            tmp006_read(&tmp006_dev, &rawvolt, &rawtemp, &drdy);
            if (drdy) {
                tmp006_convert(rawvolt, rawtemp,  &tamb, &tobj);
            }
        }

        if (sensor_stat & SENSOR_ENABLED_MPL3115) {
            mpl3115a2_read_pressure(&mpl3115a2_dev, &pressure, &status);
        }

        if (sensor_stat & SENSOR_ENABLED_MAG3110) {
            mag3110_read(&mag3110_dev, &m_x, &m_y, &m_z, &status);
        }

        if (sensor_stat & SENSOR_ENABLED_MMA8652) {
            mma8652_read(&mma8652_dev, &a_x, &a_y, &a_z, &status);
        }

        vtimer_sleep(sleep);
    }

    return NULL;
}

static msg_t msg_q[RCV_BUFFER_SIZE];

static void *hack_msg_server(void *arg)
{
    (void) arg;
    msg_t m;
    char tx[32] = "supe\0";
    char *arg_nc[] = { "nc", "ff02::1", "5000", "test"};
    unsigned int argc_nc = 4;
    unsigned int argc_msg = 3;
    char *rx;

    msg_init_queue(msg_q, RCV_BUFFER_SIZE);

    while (1) {
        msg_receive(&m);
	rx = (char *) m.content.ptr;

	if (strstr(rx, "get:hdc1000") != NULL) {
            snprintf(tx, sizeof(tx)-1, "temp:%d,rh:%d\n", temp, hum);
        } else if (strstr(rx, "get:mpl3115a2") != NULL) {
            snprintf(tx, sizeof(tx)-1, "pressure:%d\n", (unsigned int)pressure);
        } else if (strstr(rx, "get:mma8652") != NULL) {
            snprintf(tx, sizeof(tx)-1, "x:%d,y:%d,z:%d\n", a_x, a_y, a_z);
        } else if (strstr(rx, "get:mag3110") != NULL) {
            snprintf(tx, sizeof(tx)-1, "x:%d,y:%d,z:%d\n", m_x, m_y, m_z);
        } else if (strstr(rx, "get:tmp006") != NULL) {
            snprintf(tx, sizeof(tx)-1, "tamb:%d,tobj:%d\n", (int)(tamb*100), (int)(tobj*100));
        } else if (strstr(rx, "set:rled,val:0") != NULL ||
			(strstr(rx, "set:rled,val:00") != NULL)) {
		LED_R_OFF;
		snprintf(tx, sizeof(tx)-1, "LED_R_OFF\n");
        } else if (strstr(rx, "set:rled,val:") != NULL) {
		LED_R_ON;
		snprintf(tx, sizeof(tx)-1, "LED_R_ON\n");
        } else if (strstr(rx, "set:gled,val:0") != NULL ||
			(strstr(rx, "set:gled,val:00") != NULL)) {
		LED_G_OFF;
		snprintf(tx, sizeof(tx)-1, "LED_G_OFF\n");
        } else if (strstr(rx, "set:gled,val:") != NULL) {
		LED_G_ON;
		snprintf(tx, sizeof(tx)-1, "LED_G_ON\n");
        } else if (strstr(rx, "set:bled,val:0") != NULL ||
			(strstr(rx, "set:bled,val:00") != NULL)) {
		LED_B_OFF;
		snprintf(tx, sizeof(tx)-1, "LED_B_OFF\n");
        } else if (strstr(rx, "set:bled,val:") != NULL) {
		LED_B_ON;
		snprintf(tx, sizeof(tx)-1, "LED_B_ON\n");
        } else {
            continue;
        }

	arg_nc[argc_msg] = tx;
	sixlowapp_netcat(argc_nc, (char **) &arg_nc);
    }

    return NULL;
}

int main(void)
{
    puts("RIOT 6LoWPAN example v"APP_VERSION);

    sixlowpan_lowpan_init_interface(IF_ID);

    /* start thread for monitor mode */
    kernel_pid_t monitor_pid = thread_create(monitor_stack_buffer,
                                             sizeof(monitor_stack_buffer),
                                             PRIORITY_MAIN - 2,
                                             CREATE_STACKTEST,
                                             sixlowapp_monitor, NULL,
                                             "monitor");

    ipv6_register_packet_handler(monitor_pid);

    /* Start the UDP server thread */
    sixlowapp_udp_server_pid = thread_create(udp_server_stack_buffer,
                                             sizeof(udp_server_stack_buffer),
                                             PRIORITY_MAIN, CREATE_STACKTEST,
                                             sixlowapp_udp_server_loop, NULL,
                                             "UDP receiver");

    thread_create(
                  sensor_server_stack_buffer,
                  sizeof(sensor_server_stack_buffer),
                  PRIORITY_MAIN - 1,
                  CREATE_STACKTEST,
                  sensor_server,
                  NULL,
                  "sensor server");

    hack_msg_pid = thread_create(
		  hack_msg_server_stack_buffer,
                  sizeof(hack_msg_server_stack_buffer),
                  PRIORITY_MAIN - 1,
                  CREATE_STACKTEST,
                  hack_msg_server,
                  NULL,
                  "hack_msg_server");

    /* Open the UART0 for the shell */
    posix_open(uart0_handler_pid, 0);
    /* initialize the shell */
    shell_t shell;
    shell_init(&shell, shell_commands, UART0_BUFSIZE, uart0_readc, uart0_putc);
    /* start the shell loop */
    shell_run(&shell);

    return 0;
}
