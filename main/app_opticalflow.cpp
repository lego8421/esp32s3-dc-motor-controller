#include "app_opticalflow.h"

// std
#include <stdio.h>
#include <string.h>

// free rtos
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// esp
#include "esp_task.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"


static const char *TAG = "opticalflow";

static const uint8_t OPTICALFLOW_CHIP_ID = 0x49;
static const uint8_t OPTICALFLOW_CHIP_ID_INVERSE = 0xB6;

static const gpio_num_t OPTICALFLOW_MISO = GPIO_NUM_7;
static const gpio_num_t OPTICALFLOW_MOSI = GPIO_NUM_8;
static const gpio_num_t OPTICALFLOW_CLK = GPIO_NUM_9;
static const gpio_num_t OPTICALFLOW_CS = GPIO_NUM_10;

spi_host_device_t opticalflow_spi_host = SPI3_HOST;
spi_device_handle_t opticalflow_spi_handle;


static int opticalflow_spi_read(uint8_t* buffer, int len)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.rx_buffer = buffer;
    t.length = len * 8;

    spi_device_polling_transmit(opticalflow_spi_handle, &t);
    return t.rxlength;
}

static int opticalflow_spi_write(uint8_t* buffer, int len)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.tx_buffer = buffer;
    t.length = len * 8;

    spi_device_polling_transmit(opticalflow_spi_handle, &t);
    return t.rxlength;
}
static int opticalflow_spi_read_write(uint8_t* tx_buffer, int tx_len, uint8_t* rx_buffer, int rx_len)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.tx_buffer = tx_buffer;
    t.length = tx_len * 8;
    t.rx_buffer = rx_buffer;

    spi_device_polling_transmit(opticalflow_spi_handle, &t);
    return t.rxlength;
}

void opticalflow_init()
{
    // spi
    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.miso_io_num = OPTICALFLOW_MISO;
    buscfg.mosi_io_num = OPTICALFLOW_MOSI;
    buscfg.sclk_io_num = OPTICALFLOW_CLK;
    buscfg.quadwp_io_num = GPIO_NUM_NC;
    buscfg.quadhd_io_num = GPIO_NUM_NC;
    buscfg.max_transfer_sz = 0;

    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(devcfg));
    devcfg.clock_speed_hz = 4000000;
    // devcfg.clock_speed_hz = 100000;
    devcfg.mode = 3;
    devcfg.spics_io_num = OPTICALFLOW_CS;
    devcfg.queue_size = 16;
    devcfg.cs_ena_pretrans = 3;
    devcfg.cs_ena_posttrans = 3;
    devcfg.flags = 0;

    ESP_ERROR_CHECK(spi_bus_initialize(opticalflow_spi_host, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(opticalflow_spi_host, &devcfg, &opticalflow_spi_handle));

    // power on reset
    opticalflow_write_register(0x3A, 0x5A);
    vTaskDelay(pdMS_TO_TICKS(5));

    // test the spi comm, check chip id and inverse chip id
    uint8_t chip_id = opticalflow_read_register(0x00);
    uint8_t inverse_chip_id = opticalflow_read_register(0x5F);

    if (chip_id != OPTICALFLOW_CHIP_ID || inverse_chip_id != OPTICALFLOW_CHIP_ID_INVERSE) {
        ESP_LOGE(TAG, "read chip id fail - 0x%X, 0x%X", chip_id, inverse_chip_id);
    }

    // reading the motion registers one time
    opticalflow_read_register(0x02);
    opticalflow_read_register(0x03);
    opticalflow_read_register(0x04);
    opticalflow_read_register(0x05);
    opticalflow_read_register(0x06);
    vTaskDelay(pdMS_TO_TICKS(1));

    opticalflow_init_registers();
}

void opticalflow_read(int16_t *delta_x, int16_t *delta_y)
{
    opticalflow_read_register(0x02);

    *delta_x = ((int16_t)opticalflow_read_register(0x04) << 8) | opticalflow_read_register(0x03);
    *delta_y = ((int16_t)opticalflow_read_register(0x06) << 8) | opticalflow_read_register(0x05);
}

void opticalflow_enable_frame_buffer()
{
    // magic frame readout registers
    opticalflow_write_register(0x7F, 0x07);
    opticalflow_write_register(0x41, 0x1D);
    opticalflow_write_register(0x4C, 0x00);
    opticalflow_write_register(0x7F, 0x08);
    opticalflow_write_register(0x6A, 0x38);
    opticalflow_write_register(0x7F, 0x00);
    opticalflow_write_register(0x55, 0x04);
    opticalflow_write_register(0x40, 0x80);
    opticalflow_write_register(0x4D, 0x11);

    // more magic?
    opticalflow_write_register(0x70, 0x00);
    opticalflow_write_register(0x58, 0xFF);

    uint8_t check = 0;

    do {
        // the status register
        uint8_t temp = opticalflow_read_register(0x58);

        // rightshift 6 bits so only top two stay
        check = temp >> 6;
    } while(check == 0x03); // while bits aren't set denoting ready state
}

void opticalflow_read_frame_buffer(uint8_t *frame_buffer)
{
    int count = 0;
    uint8_t a; // temp value for reading register
    uint8_t b; // temp value for second register
    uint8_t hold; // holding value for checking bits
    uint8_t mask = 0x0c; // mask to take bits 2 and 3 from b
    uint8_t pixel = 0; // temp holding value for pixel

    // for 1 frame of 1225 pixels (35*35)
    for (int ii = 0; ii < 1225; ii++) {
        do {
            // if data is either invalid status
            // check status bits 6 and 7
            // if 01 move upper 6 bits into temp value
            // if 00 or 11, reread
            // else lower 2 bits into temp value
            a = opticalflow_read_register(0x58);
            // right shift to leave top two bits for ease of check.
            hold = a >> 6;
        } while((hold == 0x03) || (hold == 0x00));

        // if data is upper 6 bits
        if (hold == 0x01) {
            // read next set to get lower 2 bits
            b = opticalflow_read_register(0x58);
            pixel = a; // set pixel to a
            pixel = pixel << 2; // push left to 7:2
            pixel += (b & mask); // set lower 2 from b to 1:0
            frame_buffer[count++] = pixel; // put temp value in fbuffer array
        }
    }
    opticalflow_write_register(0x70, 0x00);   //More magic?
    opticalflow_write_register(0x58, 0xFF);

    uint8_t check;

    do {
        // read status register
        uint8_t temp = opticalflow_read_register(0x58);
        // rightshift 6 bits so only top two stay
        check = temp>>6;
    } while(check == 0x03); // while bits aren't set denoting ready state
}

void opticalflow_init_registers()
{
    opticalflow_write_register(0x7F, 0x00);
    opticalflow_write_register(0x61, 0xAD);
    opticalflow_write_register(0x7F, 0x03);
    opticalflow_write_register(0x40, 0x00);
    opticalflow_write_register(0x7F, 0x05);
    opticalflow_write_register(0x41, 0xB3);
    opticalflow_write_register(0x43, 0xF1);
    opticalflow_write_register(0x45, 0x14);
    opticalflow_write_register(0x5B, 0x32);
    opticalflow_write_register(0x5F, 0x34);
    opticalflow_write_register(0x7B, 0x08);
    opticalflow_write_register(0x7F, 0x06);
    opticalflow_write_register(0x44, 0x1B);
    opticalflow_write_register(0x40, 0xBF);
    opticalflow_write_register(0x4E, 0x3F);
    opticalflow_write_register(0x7F, 0x08);
    opticalflow_write_register(0x65, 0x20);
    opticalflow_write_register(0x6A, 0x18);
    opticalflow_write_register(0x7F, 0x09);
    opticalflow_write_register(0x4F, 0xAF);
    opticalflow_write_register(0x5F, 0x40);
    opticalflow_write_register(0x48, 0x80);
    opticalflow_write_register(0x49, 0x80);
    opticalflow_write_register(0x57, 0x77);
    opticalflow_write_register(0x60, 0x78);
    opticalflow_write_register(0x61, 0x78);
    opticalflow_write_register(0x62, 0x08);
    opticalflow_write_register(0x63, 0x50);
    opticalflow_write_register(0x7F, 0x0A);
    opticalflow_write_register(0x45, 0x60);
    opticalflow_write_register(0x7F, 0x00);
    opticalflow_write_register(0x4D, 0x11);
    opticalflow_write_register(0x55, 0x80);
    opticalflow_write_register(0x74, 0x1F);
    opticalflow_write_register(0x75, 0x1F);
    opticalflow_write_register(0x4A, 0x78);
    opticalflow_write_register(0x4B, 0x78);
    opticalflow_write_register(0x44, 0x08);
    opticalflow_write_register(0x45, 0x50);
    opticalflow_write_register(0x64, 0xFF);
    opticalflow_write_register(0x65, 0x1F);
    opticalflow_write_register(0x7F, 0x14);
    opticalflow_write_register(0x65, 0x60);
    opticalflow_write_register(0x66, 0x08);
    opticalflow_write_register(0x63, 0x78);
    opticalflow_write_register(0x7F, 0x15);
    opticalflow_write_register(0x48, 0x58);
    opticalflow_write_register(0x7F, 0x07);
    opticalflow_write_register(0x41, 0x0D);
    opticalflow_write_register(0x43, 0x14);
    opticalflow_write_register(0x4B, 0x0E);
    opticalflow_write_register(0x45, 0x0F);
    opticalflow_write_register(0x44, 0x42);
    opticalflow_write_register(0x4C, 0x80);
    opticalflow_write_register(0x7F, 0x10);
    opticalflow_write_register(0x5B, 0x02);
    opticalflow_write_register(0x7F, 0x07);
    opticalflow_write_register(0x40, 0x41);
    opticalflow_write_register(0x70, 0x00);
    vTaskDelay(pdMS_TO_TICKS(100));

    opticalflow_write_register(0x32, 0x44);
    opticalflow_write_register(0x7F, 0x07);
    opticalflow_write_register(0x40, 0x40);
    opticalflow_write_register(0x7F, 0x06);
    opticalflow_write_register(0x62, 0xf0);
    opticalflow_write_register(0x63, 0x00);
    opticalflow_write_register(0x7F, 0x0D);
    opticalflow_write_register(0x48, 0xC0);
    opticalflow_write_register(0x6F, 0xd5);
    opticalflow_write_register(0x7F, 0x00);
    opticalflow_write_register(0x5B, 0xa0);
    opticalflow_write_register(0x4E, 0xA8);
    opticalflow_write_register(0x5A, 0x50);
    opticalflow_write_register(0x40, 0x80);
}

uint8_t opticalflow_read_register(uint8_t reg)
{
    uint8_t value = 0;
    uint8_t tx_buf[2] = {0, };
    tx_buf[0] = reg;
    tx_buf[1] = 0;

    uint8_t rx_buf[2] = {0, };

    opticalflow_spi_read_write(tx_buf, 2, rx_buf, 2);
    value = rx_buf[1];

    return value;
}

void opticalflow_write_register(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {0, };
    buf[0] = reg | 0x80;
    buf[1] = val;

    opticalflow_spi_write(buf, 2);
}
