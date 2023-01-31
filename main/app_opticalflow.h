#ifndef OPTICAL_FLOW_H
#define OPTICAL_FLOW_H

// std
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


// Initialize library with data output pin and clock input pin.
void opticalflow_init();

// waits for the chip to be ready and returns a reading
void opticalflow_read(int16_t *delta_x, int16_t *delta_y);

// get the current SCALE
void opticalflow_enable_frame_buffer();

// set OFFSET, the value that's subtracted from the actual reading (tare weight)
void opticalflow_read_frame_buffer(uint8_t *frame_buffer);

void opticalflow_init_registers();

uint8_t opticalflow_read_register(uint8_t reg);

void opticalflow_write_register(uint8_t reg, uint8_t val);

#ifdef __cplusplus
}
#endif

#endif // OPTICAL_FLOW_H
