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

#include "ei_classifier_porting.h"

#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
// Include FreeRTOS for delay
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


#define EI_WEAK_FN __attribute__((weak))

char ei_get_serial_byte(void)
{
    char ch = getchar();
    // for some reason ESP32 only gets 10 (\n)and AT server has 13 (\r) as terminator character...
    if (ch == '\n') {
        ch = '\r';
    }

    return ch;
}

void ei_putchar(char c)
{
    /* Send char to serial output */
    putchar(c);
}

/**
 *  Printf function uses vsnprintf and output using USB Serial
 */
EI_WEAK_FN void ei_printf(const char *format, ...)
{
    static char print_buf[1024] = { 0 };

    va_list args;
    va_start(args, format);
    int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
    va_end(args);

    if (r > 0) {
       printf(print_buf);
    }
}

EI_WEAK_FN void ei_printf_float(float f)
{
    ei_printf("%f", f);
}

EI_WEAK_FN void *ei_malloc(size_t size)
{
    return malloc(size);
}

EI_WEAK_FN void *ei_calloc(size_t nitems, size_t size)
{
    return calloc(nitems, size);
}

EI_WEAK_FN void ei_free(void *ptr)
{
    free(ptr);
}
