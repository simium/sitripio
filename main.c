/**
 * Based A LOT on code from the PICO SDK examples.
 *
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

#include "hardware/adc.h"
#include "hardware/gpio.h"

#include "pico/stdlib.h"
#include "pico/lorawan.h"

// edit with LoRaWAN Node Region and OTAA settings 
#include "lorawan_config.h"

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
static volatile int gps_pulses_in = 0;
static volatile int can_tx_data = false;
static int gps_chars_rx = 0;
static int debug_tx = 0;

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

// pin configuration for SX1276 radio module
const struct lorawan_sx1276_settings sx1276_settings = {
    .spi = {
        .inst = PICO_DEFAULT_SPI_INSTANCE,
        .mosi = PICO_DEFAULT_SPI_TX_PIN,
        .miso = PICO_DEFAULT_SPI_RX_PIN,
        .sck  = PICO_DEFAULT_SPI_SCK_PIN,
        .nss  = 8
    },
    .reset = 9,
    .dio0  = 7,
    .dio1  = 10
};

// ABP settings
const struct lorawan_abp_settings abp_settings = {
    .device_address = LORAWAN_DEV_ADDR,
    .network_session_key = LORAWAN_NETWORK_SESSION_KEY,
    .app_session_key = LORAWAN_APP_SESSION_KEY,
    .channel_mask = LORAWAN_CHANNEL_MASK
};

// variables for receiving data
int receive_length = 0;
uint8_t receive_buffer[242];
uint8_t receive_port = 0;

// functions used in main
void internal_temperature_init()
{
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);
}

float internal_temperature_get()
{
    const float v_ref = 3.3;

    // select and read the ADC
    adc_select_input(4);
    uint16_t adc_raw = adc_read();

    // convert the raw ADC value to a voltage
    float adc_voltage = adc_raw * v_ref / 4095.0f;

    // convert voltage to temperature, using the formula from 
    // section 4.9.4 in the RP2040 datasheet
    //   https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf
    float adc_temperature = 27.0 - ((adc_voltage - 0.706) / 0.001721);

    return adc_temperature;
}

void gpio_callback(uint gpio, uint32_t events) {
    gps_pulses_in++;
    //printf("%d\r\n", gps_pulses_in);
}

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

    // initialize the LoRaWAN stack
    printf("Initilizating LoRaWAN ... ");
    if (lorawan_init_abp(&sx1276_settings, LORAWAN_REGION, &abp_settings) < 0) {
        printf("failed!!!\n");
        while (1) {
            tight_loop_contents();
        }
    } else {
        printf("success!\n");
    }

    //Start the join process and wait
    printf("Joining LoRaWAN network ...");
    lorawan_join();

    while (!lorawan_is_joined()) {
        lorawan_process_timeout_ms(1000);
        printf(".");
    }
    printf(" joined successfully!\n");
    
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

    gpio_set_irq_enabled_with_callback(6, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    internal_temperature_init();
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
    char lora_payload[20];
    int lora_payload_p;

    setup();

    // send payload
    const char* message = "C3POALIVE";

    while (1)
    {
        lorawan_process();

        if (gps_pulses_in >= 30)
        {
            printf("30 good GPS RXd\r\n");
            gps_pulses_in = 0;
            can_tx_data = true;
            gpio_put(LED_PIN, 1);

            debug_tx++;
        }

        if (debug_tx>=10)
        {
            // try to send an unconfirmed uplink message
            printf("sending unconfirmed message '%s' ... ", message);
            if (lorawan_send_unconfirmed(message, strlen(message), 2) < 0) {
                printf("Ping::OK\n");
            } else {
                printf("Ping::KO\n");
            }

            debug_tx = 0;
        }

        if (gps_print_available)
        {
            memset(lora_payload, 0, 20);
            lora_payload_p = 0;

            if (strncmp(&gps_print_data[3], "GGA", 3) == 0) {
                parse_comma_delimited_str(gps_print_data, gps_field, 20);
                
#if 1
                printf("UTC Time  :%s\r\n", gps_field[GPS_TIME]);
                printf("Latitude  :%s\r\n", gps_field[GPS_LATITUDE]);
                printf("N/S       :%s\r\n", gps_field[GPS_LATITUDE_NS]);
                printf("Longitude :%s\r\n", gps_field[GPS_LONGITUDE]);
                printf("E/W       :%s\r\n", gps_field[GPS_LONGITUDE_WE]);
                printf("Altitude  :%s\r\n", gps_field[GPS_ALTITUDE]);
                printf("Satellites:%s\r\n", gps_field[GPS_NUM_SATS]);
#endif
                char *p;
                int utc_hh, utc_mm, utc_ss, utc_milliseconds, utc_ms_scale = 0;
                int latitude, lat_DD, lat_mm1, lat_mm2, lat_mm_scale = 0;
                int longitude, lon_DD, lon_mm1, lon_mm2, lon_mm_scale = 0;
                int altitude, alt_m, alt_cm, alt_cm_scale = 0;
                int pow_scale;
                float latitude_f, lat_mm_f;
                float longitude_f, lon_mm_f;

                if ( (p=strchr(gps_field[GPS_TIME], '.' )) != NULL ) {
                    sscanf( gps_field[GPS_TIME], "%02d%02d%02d.%d", &utc_hh, &utc_mm, &utc_ss, &utc_milliseconds );
                    utc_ms_scale = strlen( p+1 );

                    pow_scale = quick_pow10(utc_ms_scale);
                    //HHMMSSmmm = utc_hhmmss*pow_scale + utc_milliseconds;

                    //printf("HHMMSS.mmm = %02d:%02d:%02d.%d\r\n", utc_hh, utc_mm, utc_ss, utc_milliseconds);
                }

                if ( (p=strchr(gps_field[GPS_LATITUDE], '.' )) != NULL ) {
                    sscanf( gps_field[GPS_LATITUDE], "%02d%02d.%d", &lat_DD, &lat_mm1, &lat_mm2);
                    lat_mm_scale = strlen( p+1 );

                    pow_scale = quick_pow10(lat_mm_scale);
                    lat_mm_f = (float)lat_mm1 + (float)(lat_mm2)/pow_scale;

                    latitude_f = (float)(lat_DD) + lat_mm_f/60.0;

                    if (gps_field[GPS_LATITUDE_NS][0] == 'S')
                        latitude_f = -latitude_f;

                    latitude = (int)(latitude_f*1000000.0);

                    memcpy(&lora_payload[lora_payload_p], &latitude, sizeof(latitude));
                    lora_payload_p += sizeof(latitude);
                }

                if ( (p=strchr(gps_field[GPS_LONGITUDE], '.' )) != NULL ) {
                    sscanf( gps_field[GPS_LONGITUDE], "%03d%02d.%d", &lon_DD, &lon_mm1, &lon_mm2);
                    lon_mm_scale = strlen( p+1 );

                    pow_scale = quick_pow10(lon_mm_scale);
                    lon_mm_f = (float)lon_mm1 + (float)(lon_mm2)/pow_scale;

                    longitude_f = (float)(lon_DD) + lon_mm_f/60.0;

                    if (gps_field[GPS_LONGITUDE_WE][0] == 'W')
                        longitude_f = -longitude_f;

                    longitude = (int)(longitude_f*1000000.0);

                    memcpy(&lora_payload[lora_payload_p], &longitude, sizeof(longitude));
                    lora_payload_p += sizeof(longitude);
                }

                if ( (p=strchr(gps_field[GPS_ALTITUDE], '.' )) != NULL ) {
                    sscanf( gps_field[GPS_ALTITUDE], "%d.%d", &alt_m, &alt_cm );
                    lat_mm_scale = strlen( p+1 );

                    pow_scale = quick_pow10(alt_cm_scale);
                    altitude = alt_m*pow_scale + alt_cm;

                    memcpy(&lora_payload[lora_payload_p], &altitude, sizeof(altitude));
                    lora_payload_p += sizeof(altitude);
                }

                {
                    // get the internal temperature
                    int8_t adc_temperature_byte = internal_temperature_get();

                    memcpy(&lora_payload[lora_payload_p], &adc_temperature_byte, sizeof(adc_temperature_byte));
                    lora_payload_p += sizeof(adc_temperature_byte);
                }
                
                if (can_tx_data)
                {
                    // send payload
                    if (lorawan_send_unconfirmed(lora_payload, lora_payload_p, 2) < 0) {
                        printf("TX failed!!!\n");
                    } else {
                        printf("TX success!\n");
                    }

#if 1
                    for (int i = 0; i < lora_payload_p; i++)
                    {
                        printf("%02X ", lora_payload[i]);
                    }
                    printf("\r\n");
#endif
                    can_tx_data = false;
                    gpio_put(LED_PIN, 0);
                }
            }
            gps_print_available = false;
        }
    }
}
