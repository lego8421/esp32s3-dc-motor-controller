#include "app_loadcell.h"

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

// app
#include "define.h"

static const char *TAG = "loadcell";

static const gpio_num_t LOADCELL_DOUT = GPIO_NUM_5;
static const gpio_num_t LOADCELL_CLK = GPIO_NUM_6;

spi_host_device_t loadcell_spi_host = SPI2_HOST;
spi_device_handle_t loadcell_spi_handle;

static float loadcell_scale = 1.0f;
static int32_t loadcell_offset = 0;

static int spi_read(spi_device_handle_t handle, uint8_t* buffer, int len)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.rx_buffer = buffer;
    t.length = len * 8;

    spi_device_polling_transmit(handle, &t);
    return t.rxlength;
}

static void spi_init(spi_host_device_t host, spi_device_handle_t *handle, gpio_num_t dout, gpio_num_t clock)
{
    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.miso_io_num = dout;
    buscfg.mosi_io_num = GPIO_NUM_NC;
    buscfg.sclk_io_num = clock;
    buscfg.quadwp_io_num = GPIO_NUM_NC;
    buscfg.quadhd_io_num = GPIO_NUM_NC;
    buscfg.max_transfer_sz = 32;

    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(devcfg));
    devcfg.clock_speed_hz = 100000;
    devcfg.mode = 0;
    devcfg.spics_io_num = GPIO_NUM_NC;
    devcfg.queue_size = 16;
    devcfg.flags = 0;

    ESP_ERROR_CHECK(spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(host, &devcfg, handle));
}

static void gpio_init(gpio_num_t dout, gpio_num_t clock)
{
    gpio_config_t gpio_conf;
    memset(&gpio_conf, 0, sizeof(gpio_config_t));
    gpio_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_conf.pull_up_en = GPIO_PULLUP_ENABLE;

    // clk
    gpio_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    gpio_conf.pin_bit_mask = 1ULL << clock;
    gpio_config(&gpio_conf);

    // dout
    gpio_conf.mode = GPIO_MODE_INPUT;
    gpio_conf.pin_bit_mask = 1ULL << dout;
    gpio_config(&gpio_conf);
}

void loadcell_init()
{
    gpio_init(LOADCELL_DOUT, LOADCELL_CLK);
    spi_init(loadcell_spi_host, &loadcell_spi_handle, LOADCELL_DOUT, LOADCELL_CLK);
    loadcell_offset = 0;
    loadcell_scale = 1.0f;
}

bool loadcell_is_ready()
{
    return gpio_get_level(LOADCELL_DOUT) == 0;
}

void loadcell_wait_ready(uint32_t delay_ms)
{
    while (!loadcell_is_ready()) {
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

bool loadcell_wait_ready_retry(int retries, uint32_t delay_ms)
{
    int count = 0;
    while (count < retries) {
        if (loadcell_is_ready()) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        count++;
    }
    return false;
}

bool loadcell_ready_timeout(uint32_t timeout, uint32_t delay_ms)
{
    uint32_t millisStarted = micros();
    while (micros() - millisStarted < timeout * 1000) {
        if (loadcell_is_ready()) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    return false;
}

int32_t loadcell_read()
{
    loadcell_ready_timeout(10);

    // Define structures for reading data into.
    int32_t value = 0;
    uint8_t buffer[4] = {0, };

    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL(&mux);

    // Pulse the clock pin 24 times to read the data.
    spi_read(loadcell_spi_handle, &buffer[2], 1);
    spi_read(loadcell_spi_handle, &buffer[1], 1);
    spi_read(loadcell_spi_handle, &buffer[0], 1);
    delay_us(1);

    for (unsigned int i = 0; i < 1; i++) {
        gpio_set_level(LOADCELL_CLK, 1);
        delay_us(1);
        gpio_set_level(LOADCELL_CLK, 0);
        delay_us(1);
    }

    portEXIT_CRITICAL(&mux);

    // Replicate the most significant bit to pad out a 32-bit signed integer
    if (buffer[2] & 0x80) {
        buffer[3] = 0xFF;
    } else {
        buffer[3] = 0x00;
    }

    // Construct a 32-bit signed integer
    value = *(int32_t*)buffer;
    return value;
}

int32_t loadcell_read_average(uint8_t times)
{
    long sum = 0;
    for (uint8_t i = 0; i < times; i++) {
        sum += loadcell_read();
        // Probably will do no harm on AVR but will feed the Watchdog Timer (WDT) on ESP.
        // https://github.com/bogde/HX711/issues/73
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return sum / times;
}

float loadcell_get_value(uint8_t times)
{
    return loadcell_read_average(times) - loadcell_offset;
}

float loadcell_get_units(uint8_t times)
{
    // loadcell_scale = loadcell_get_value / known_weight
    return loadcell_get_value(times) / loadcell_scale;
}

void loadcell_tare(uint8_t times)
{
    float sum = loadcell_read_average(times);
    loadcell_set_offset(sum);
}

void loadcell_set_scale(float scale)
{
    loadcell_scale = scale;
}

float loadcell_get_scale()
{
    return loadcell_scale;
}

void loadcell_set_offset(int32_t offset)
{
    loadcell_offset = offset;
}

long loadcell_get_offset()
{
    return loadcell_offset;
}
