/* Edge Impulse inferencing library
 * Copyright (c) 2021 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef _EI_CLASSIFIER_PORTING_H_
#define _EI_CLASSIFIER_PORTING_H_

#include <stdint.h>
#include <stdlib.h>

#if defined(__cplusplus) && EI_C_LINKAGE == 1
extern "C" {
#endif // defined(__cplusplus)

/**
 * @brief      Get next available byte
 *
 * @return     byte
 */
char ei_get_serial_byte(void);

/**
 * @brief      Connect to putchar of target
 *
 * @param[in]  c The chararater
 */
void ei_putchar(char c);

/**
 * Print wrapper around printf()
 * This is used internally to print debug information.
 */
__attribute__ ((format (printf, 1, 2)))
void ei_printf(const char *format, ...);

/**
 * Override this function if your target cannot properly print floating points
 * If not overriden, this will be sent through `ei_printf()`.
 */
void ei_printf_float(float f);

/**
 * Wrapper around malloc
 */
void *ei_malloc(size_t size);

/**
 * Wrapper around calloc
 */
void *ei_calloc(size_t nitems, size_t size);

/**
 * Wrapper around free
 */
void ei_free(void *ptr);

#endif // _EI_CLASSIFIER_PORTING_H_
