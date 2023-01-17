#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


typedef struct path_interpolation_ {
    float target_q;
    float target_qd;
    float desired_q;
} path_interpolation;

void path_init(path_interpolation* path_, float current_q);
void path_set_target(path_interpolation* path_, float target_q, float target_qd);
void path_calcuate(path_interpolation* path_, float dt);

#ifdef __cplusplus
}
#endif
