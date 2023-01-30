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


static const char *TAG = "loadcell";

#define LOADCELL_CH_NUM 2   // ch0: external, ch1: internal

static const gpio_num_t LOADCELL_DOUT[LOADCELL_CH_NUM] = {GPIO_NUM_5, GPIO_NUM_7};
static const gpio_num_t LOADCELL_CLK[LOADCELL_CH_NUM] = {GPIO_NUM_6, GPIO_NUM_8};

spi_host_device_t loadcell_spi_host[LOADCELL_CH_NUM] = {SPI2_HOST, SPI3_HOST};
spi_device_handle_t loadcell_spi_handle[LOADCELL_CH_NUM];

static float loadcell_scale[LOADCELL_CH_NUM];
static int32_t loadcell_offset[LOADCELL_CH_NUM];

#define NOP() asm volatile ("nop")

uint32_t IRAM_ATTR micros()
{
    return (uint32_t) (esp_timer_get_time());
}

void IRAM_ATTR delay_us(uint32_t us)
{
    uint32_t m = micros();
    if (us) {
        uint32_t e = (m + us);
        if(m > e){
            while (micros() > e){
                NOP();
            }
        }
        while (micros() < e){
            NOP();
        }
    }
}

static bool gpio_is_ready(uint8_t ch)
{
    return gpio_get_level(LOADCELL_DOUT[ch]) == 0;
}

static void gpio_wait_for_ready(uint8_t ch, uint32_t delay_ms)
{
    while (!gpio_is_ready(ch)) {
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

static bool gpio_wait_ready_retry(uint8_t ch, int retries, uint32_t delay_ms)
{
    int count = 0;
    while (count < retries) {
        if (gpio_is_ready(ch)) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        count++;
    }
    return false;
}

static bool gpio_ready_timeout(uint8_t ch, uint32_t timeout, uint32_t delay_ms)
{
    uint32_t millisStarted = micros();
    while (micros() - millisStarted < timeout * 1000) {
        if (gpio_is_ready(ch)) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    return false;
}

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

void loadcell_init()
{
    gpio_config_t gpio_conf;
    memset(&gpio_conf, 0, sizeof(gpio_config_t));
    gpio_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_conf.pull_up_en = GPIO_PULLUP_ENABLE;

    // clk
    gpio_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    gpio_conf.pin_bit_mask = (1ULL << LOADCELL_CLK[0]) | (1ULL << LOADCELL_CLK[1]);
    gpio_config(&gpio_conf);

    // dout
    gpio_conf.mode = GPIO_MODE_INPUT;
    gpio_conf.pin_bit_mask = (1ULL << LOADCELL_DOUT[0]) | (1ULL << LOADCELL_DOUT[1]);
    gpio_config(&gpio_conf);

    // parameter
    // spi_init(loadcell_spi_host[0], &loadcell_spi_handle[0], LOADCELL_DOUT[0], LOADCELL_CLK[0]);
    for (int i = 0; i < LOADCELL_CH_NUM; i++) {
        spi_init(loadcell_spi_host[i], &loadcell_spi_handle[i], LOADCELL_DOUT[i], LOADCELL_CLK[i]);
        loadcell_offset[i] = 0;
        loadcell_scale[i] = 1.0f;
    }
}

int32_t loadcell_read(uint8_t ch)
{
    if (ch >= LOADCELL_CH_NUM) {
        return 0;
    }

    // gpio_wait_for_ready(ch, 1);

    // Define structures for reading data into.
    int32_t value = 0;
    uint8_t data[4] = {0, };

    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL(&mux);

    // Pulse the clock pin 24 times to read the data.
    spi_read(loadcell_spi_handle[ch], &data[2], 1);
    spi_read(loadcell_spi_handle[ch], &data[1], 1);
    spi_read(loadcell_spi_handle[ch], &data[0], 1);

    for (unsigned int i = 0; i < 1; i++) {
        gpio_set_level(LOADCELL_CLK[ch], 1);
        delay_us(1);
        gpio_set_level(LOADCELL_CLK[ch], 0);
        delay_us(1);
    }

    portEXIT_CRITICAL(&mux);

    // Replicate the most significant bit to pad out a 32-bit signed integer
    if (data[2] & 0x80) {
        data[3] = 0xFF;
    } else {
        data[3] = 0x00;
    }

    // Construct a 32-bit signed integer
    value = *(uint32_t*)data;

    int32_t ret = static_cast<int32_t>(value);
    if (ret == -1) {
        ret = 0;
    }
    return ret;
}

int32_t loadcell_read_average(uint8_t ch, uint8_t times)
{
    if (ch >= LOADCELL_CH_NUM) {
        return 0;
    }

    long sum = 0;
    for (uint8_t i = 0; i < times; i++) {
        sum += loadcell_read(ch);
        // Probably will do no harm on AVR but will feed the Watchdog Timer (WDT) on ESP.
        // https://github.com/bogde/HX711/issues/73
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return sum / times;
}

float loadcell_get_value(uint8_t ch, uint8_t times)
{
    if (ch >= LOADCELL_CH_NUM) {
        return 0.0;
    }

    return loadcell_read_average(ch, times) - loadcell_offset[ch];
}

float loadcell_get_units(uint8_t ch, uint8_t times)
{
    if (ch >= LOADCELL_CH_NUM) {
        return 0.0;
    }

    // loadcell_scale = loadcell_get_value / known_weight
    return loadcell_get_value(ch, times) / loadcell_scale[ch];
}

void loadcell_tare(uint8_t ch, uint8_t times)
{
    if (ch >= LOADCELL_CH_NUM) {
        return;
    }

    float sum = loadcell_read_average(ch, times);
    loadcell_set_offset(ch, sum);
}

void loadcell_set_scale(uint8_t ch, float scale)
{
    if (ch >= LOADCELL_CH_NUM) {
        return;
    }

    loadcell_scale[ch] = scale;
}

float loadcell_get_scale(uint8_t ch)
{
    if (ch >= LOADCELL_CH_NUM) {
        return 0.0f;
    }

    return loadcell_scale[ch];
}

void loadcell_set_offset(uint8_t ch, int32_t offset)
{
    if (ch >= LOADCELL_CH_NUM) {
        return;
    }

    loadcell_offset[ch] = offset;
}

long loadcell_get_offset(uint8_t ch)
{
    if (ch >= LOADCELL_CH_NUM) {
        return 0;
    }

    return loadcell_offset[ch];
}
