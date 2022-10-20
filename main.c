/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"


/// \tag::uart_advanced[]

#define UART_ID uart0
#define BAUD_RATE 9600
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN 0
#define UART_RX_PIN 1

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

static volatile int gps_raw_end = false;
static volatile int gps_print_available = false;
static int gps_chars_rx = 0;
static char gps_raw_data[80];
static char gps_print_data[80];

// RX interrupt handler
void on_uart_rx() {
    if (uart_is_readable(UART_ID)) {
        uint8_t ch = uart_getc(UART_ID);
        if (ch == '$')
        {
            gps_chars_rx = 0;
        }
        else if (ch == '*')
        {
            gps_raw_end = true;
        }

        gps_raw_data[gps_chars_rx] = ch;
        gps_chars_rx++;
    }

    if (gps_raw_end)
    {
        gps_raw_data[gps_chars_rx] = '\0';
        strncpy(gps_print_data, (const char *)gps_raw_data, 80);
        //printf(gps_print_data);
        //printf("\r\n");
        gps_raw_end = false;
        gps_print_available = true;
    }
}

void setup()
{
    // Clean gps_raw_data
    memset(gps_raw_data, 0, 80);
    memset(gps_print_data, 0, 80);

    // Setup USB UART
    stdio_usb_init();
    
    // Set up our UART with a basic baud rate.
    uart_init(UART_ID, BAUD_RATE);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Actually, we want a different speed
    // The call will return the actual baud rate selected, which will be as close as
    // possible to that requested
    int __unused actual = uart_set_baudrate(UART_ID, BAUD_RATE);

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(UART_ID, false, false);

    // Set our data format
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID, false);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
}

void parse_comma_delimited_str(char *string, char **fields, int max_fields)
{
	int i = 0;
	fields[i++] = string;

	while ((i < max_fields) && NULL != (string = strchr(string, ','))) {
		*string = '\0';
		fields[i++] = ++string;
	}
}

int main() {
    char *field[20];

    setup();

    while (1)
    {
        if (gps_print_available)
        {
            if (strncmp(&gps_print_data[3], "GGA", 3) == 0) {
                parse_comma_delimited_str(gps_print_data, field, 20);
                //debug_print_fields(i,field);
                printf("UTC Time  :%s\r\n",field[1]);
                printf("Latitude  :%s\r\n",field[2]);
                printf("Longitude :%s\r\n",field[4]);
                printf("Altitude  :%s\r\n",field[9]);
                printf("Satellites:%s\r\n",field[7]);

                char *p;
                int utc_hhmmss, utc_milliseconds, utc_ms_scale;

                if ( (p=strchr(field[1], '.' )) != NULL ) {
                    sscanf( field[1], "%d.%d", &utc_hhmmss, &utc_milliseconds );
                    utc_ms_scale = strlen( p+1 );
                    printf("UTC seconds       :%d\r\n",utc_hhmmss);
                    printf("UTC milliseconds  :%d\r\n",utc_milliseconds);
                    printf("UTC scale         :%d\r\n",utc_ms_scale);
                }

                int lat_DDmm, lat_mm, lat_mm_scale;

                if ( (p=strchr(field[2], '.' )) != NULL ) {
                    sscanf( field[2], "%d.%d", &lat_DDmm, &lat_mm );
                    lat_mm_scale = strlen( p+1 );
                    printf("LAT DDmm     :%d\r\n",lat_DDmm);
                    printf("LAT mm       :%d\r\n",lat_mm);
                    printf("LAT mm scale :%d\r\n",lat_mm_scale);
                }

                int lon_DDmm, lon_mm, lon_mm_scale;

                if ( (p=strchr(field[4], '.' )) != NULL ) {
                    sscanf( field[4], "%d.%d", &lon_DDmm, &lon_mm );
                    lat_mm_scale = strlen( p+1 );
                    printf("LON DDmm     :%d\r\n",lon_DDmm);
                    printf("LON mm       :%d\r\n",lon_mm);
                    printf("LON mm scale :%d\r\n",lon_mm_scale);
                }

                int alt_m, alt_cm, alt_cm_scale;

                if ( (p=strchr(field[9], '.' )) != NULL ) {
                    sscanf( field[9], "%d.%d", &alt_m, &alt_cm );
                    lat_mm_scale = strlen( p+1 );
                    printf("LAT DDmm     :%d\r\n",alt_m);
                    printf("LAT mm       :%d\r\n",alt_cm);
                    printf("LAT mm scale :%d\r\n",alt_cm_scale);
                }
            }
            gpio_put(LED_PIN, 1);
            //printf(gps_print_data);
            //printf("\r\n");
            gpio_put(LED_PIN, 0);
            gps_print_available = false;
        }
    }
}
