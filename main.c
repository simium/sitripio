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

#define LED_PIN PICO_DEFAULT_LED_PIN

static volatile int gps_raw_end = false;
static volatile int gps_print_available = false;
static int gps_chars_rx = 0;

#define BUF_SIZE 100
static char gps_raw_data[BUF_SIZE];
static char gps_print_data[BUF_SIZE];

enum gps_fields {GPS_ID = 0,
                GPS_TIME,
                GPS_LATITUDE,
                GPS_LATITUDE_NS,
                GPS_LONGITUDE,
                GPS_LONGITUDE_WE,
                GPS_QUALITY,
                GPS_NUM_SATS,
                GPS_HDOP,
                GPS_ALTITUDE,
                GPS_GEOID_HEIGHT}; 

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
        strncpy(gps_print_data, (const char *)gps_raw_data, BUF_SIZE);
        gps_raw_end = false;
        gps_print_available = true;
    }
}

void setup()
{
    // Clean gps_raw_data
    memset(gps_raw_data, 0, BUF_SIZE);
    memset(gps_print_data, 0, BUF_SIZE);

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

int quick_pow10(int n)
{
    static int pow10[10] = {
        1, 10, 100, 1000, 10000, 
        100000, 1000000, 10000000, 100000000, 1000000000
    };

    return pow10[n]; 
}

int main() {
    char *gps_field[20];

    setup();

    while (1)
    {
        if (gps_print_available)
        {
            gpio_put(LED_PIN, 1);
            if (strncmp(&gps_print_data[3], "GGA", 3) == 0) {
                parse_comma_delimited_str(gps_print_data, gps_field, 20);
                
#ifdef SITRIPIO_DBG
                printf("UTC Time  :%s\r\n", gps_field[GPS_TIME]);
                printf("Latitude  :%s\r\n", gps_field[GPS_LATITUDE]);
                printf("N/S       :%s\r\n", gps_field[GPS_LATITUDE_NS]);
                printf("Longitude :%s\r\n", gps_field[GPS_LONGITUDE]);
                printf("E/W       :%s\r\n", gps_field[GPS_LONGITUDE_WE]);
                printf("Altitude  :%s\r\n", gps_field[GPS_ALTITUDE]);
                printf("Satellites:%s\r\n", gps_field[GPS_NUM_SATS]);
#endif
                char *p;
                int HHMMSSmmm, utc_hhmmss, utc_milliseconds, utc_ms_scale = 0;
                int latitude, lat_DDmm, lat_mm, lat_mm_scale = 0;
                int longitude, lon_DDmm, lon_mm, lon_mm_scale = 0;
                int altitude, alt_m, alt_cm, alt_cm_scale = 0;
                int pow_scale;

                if ( (p=strchr(gps_field[GPS_TIME], '.' )) != NULL ) {
                    sscanf( gps_field[GPS_TIME], "%d.%d", &utc_hhmmss, &utc_milliseconds );
                    utc_ms_scale = strlen( p+1 );

                    pow_scale = quick_pow10(utc_ms_scale);
                    HHMMSSmmm = utc_hhmmss*pow_scale + utc_milliseconds;

                    printf("HHMMSSmmm  :%d\r\n", HHMMSSmmm);
                }

                if ( (p=strchr(gps_field[GPS_LATITUDE], '.' )) != NULL ) {
                    sscanf( gps_field[GPS_LATITUDE], "%d.%d", &lat_DDmm, &lat_mm );
                    lat_mm_scale = strlen( p+1 );

                    pow_scale = quick_pow10(lat_mm_scale);
                    latitude = lat_DDmm*pow_scale + lat_mm;

                    if (gps_field[GPS_LATITUDE_NS][0] == 'S')
                        latitude = -latitude;

                    printf("LATITUDE  :%d\r\n", latitude);
                }

                if ( (p=strchr(gps_field[GPS_LONGITUDE], '.' )) != NULL ) {
                    sscanf( gps_field[GPS_LONGITUDE], "%d.%d", &lon_DDmm, &lon_mm );
                    lon_mm_scale = strlen( p+1 );

                    pow_scale = quick_pow10(lon_mm_scale);
                    longitude = lon_DDmm*pow_scale + lon_mm;

                    printf("LONGITUDE :%d\r\n", longitude);

                    if (gps_field[GPS_LONGITUDE_WE][0] == 'W')
                        longitude = -longitude;
                }

                if ( (p=strchr(gps_field[GPS_ALTITUDE], '.' )) != NULL ) {
                    sscanf( gps_field[GPS_ALTITUDE], "%d.%d", &alt_m, &alt_cm );
                    lat_mm_scale = strlen( p+1 );
                    // printf("LAT DDmm     :%d\r\n",alt_m);
                    // printf("LAT mm       :%d\r\n",alt_cm);
                    // printf("LAT mm scale :%d\r\n",alt_cm_scale);

                    pow_scale = quick_pow10(alt_cm_scale);
                    altitude = alt_m*pow_scale + alt_cm;

                    printf("ALTITUDE  :%d\r\n", altitude);
                }

                printf("sizeof(int) = %d\r\n", sizeof(int));
            }
            gpio_put(LED_PIN, 0);
            gps_print_available = false;
        }
    }
}
