#include <string.h>
#include <math.h>

#include "path_interpolation.h"


void path_init(path_interpolation* path_, float current_q)
{
    path_->target_q = current_q;
    path_->target_qd = 0;
    path_->desired_q = current_q;
}

void path_set_target(path_interpolation* path_, float target_q, float target_qd)
{
    path_->target_q = target_q;
    path_->target_qd = target_qd;
}

void path_calcuate(path_interpolation* path_, float dt)
{
    float diff = (path_->target_q - path_->desired_q) / dt;

    if (fabs(diff) >= path_->target_qd) {
        if (path_->target_q >= path_->desired_q) {
            path_->desired_q += path_->target_qd * dt;
        } else {
            path_->desired_q -= path_->target_qd * dt;
        }
    } else {
        path_->desired_q = path_->target_q;
    }

    return;
}
