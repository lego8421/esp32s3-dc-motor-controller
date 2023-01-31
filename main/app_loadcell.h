#ifndef LOADCELL_H
#define LOADCELL_H

// std
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


// Initialize library with data output pin and clock input pin.
void loadcell_init();

// Check if HX711 is ready
// from the datasheet: When output data is not ready for retrieval, digital output pin DOUT is high. Serial clock
// input PD_SCK should be low. When DOUT goes to low, it indicates data is ready for retrieval.
bool loadcell_is_ready();

// Wait for the HX711 to become ready
void loadcell_wait_ready(uint32_t delay_ms=1);
bool loadcell_wait_ready_retry(int retries=3, uint32_t delay_ms=1);
bool loadcell_ready_timeout(uint32_t timeout=1000, uint32_t delay_ms=1);

// waits for the chip to be ready and returns a reading
int32_t loadcell_read();

// returns an average reading; times = how many times to read
int32_t loadcell_read_average(uint8_t times=10);

// returns (loadcell_read_average() - OFFSET), that is the current value without the tare weight; times = how many readings to do
float loadcell_get_value(uint8_t times=1);

// returns loadcell_get_value() divided by SCALE, that is the raw value divided by a value obtained via calibration
// times = how many readings to do
float loadcell_get_units(uint8_t times=1);

// set the OFFSET value for tare weight; times = how many times to read the tare value
void loadcell_tare(uint8_t times=10);

// set the SCALE value; this value is used to convert the raw data to "human readable" data (measure units)
void loadcell_set_scale(float scale=1.f);

// get the current SCALE
float loadcell_get_scale();

// set OFFSET, the value that's subtracted from the actual reading (tare weight)
void loadcell_set_offset(int32_t offset=0);

// get the current OFFSET
long loadcell_get_offset();

#ifdef __cplusplus
}
#endif

#endif // LOADCELL_H
