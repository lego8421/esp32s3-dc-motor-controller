#ifndef LQR_STEER_CONTROL_H
#define LQR_STEER_CONTROL_H

#include <vector>

#include "cubic_spline_planner.h"
#include "pid_controller.h"
#include <fstream>

#define _USE_MATH_DEFINES

typedef struct ControlState_
{
    double x;
    double y;
    double yaw;
    double v;

    ControlState_()
    {

    }

    ControlState_(double x, double y, double yaw, double v)
    {
        this->x = x;
        this->y = y;
        this->yaw = yaw;
        this->v = v;
    }
} ControlState;

class lqr_steer_control
{
public:
    lqr_steer_control();
    ~lqr_steer_control();

public:
    bool update(double dt);
    void generate_spline(ControlState init_state, std::vector<WayPoint> waypoints, double target_speed, double ds=1.0);
    void calc_ref_trajectory(double dt, ModelMatrix& reference_point, ModelMatrix& reference_steer);

private:
    int calculate_nearest_index(ControlState state, std::vector<Point> points, int pind, double& min_distance);
    void smooth_yaw(std::vector<Point> &points);
    ModelMatrix predict_motion(ControlState x0, double a, double delta, ModelMatrix x_ref, double dt);
    void linear_mpc_control(ModelMatrix x_ref, ModelMatrix x_bar, ControlState x0, ModelMatrix d_ref);
    ControlState update_state(ControlState state, double a, double delta, double dt);
    ModelMatrix dlqr(ModelMatrix A, ModelMatrix B, ModelMatrix Q, ModelMatrix R);
    int lqr_steering_control(ControlState state, double& steer, double& pe, double& pth_e);
    ModelMatrix solve_DARE(ModelMatrix A, ModelMatrix B, ModelMatrix Q, ModelMatrix R);

private:
    double t;                  // 누적 시간
    double dt;
    ControlState init_state;
    ControlState goal_state;
    std::vector<Point> points; // spline 된 좌표값 + yaw + speed, 굴곡

    ControlState state;        // 현재 상태
    int target_ind;             // 목표로 가려는 point의 index값
    double dl;                  //
    std::vector<double> oa;     // accel
    std::vector<double> odelta; // steer
    pid_controller path_pid;

public :
    ControlState get_state() const {
        return this->state;
    }

    void set_state(ControlState state) {
        this->state = state;
    }
};

#endif // LQR_STEER_CONTROL_H
