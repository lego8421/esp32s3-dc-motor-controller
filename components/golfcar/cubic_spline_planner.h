#ifndef CUBIC_SPLINE_PLANNER_H
#define CUBIC_SPLINE_PLANNER_H

#include <math.h>
#include <vector>

#include "model_matrix.h"


typedef struct WayPoint_ {
    double x;
    double y;

    WayPoint_(double x, double y)
    {
        this->x = x;
        this->y = y;
    }

} WayPoint;

typedef struct Point_ {
    double x;
    double y;
    double yaw;
    double k;
    double speed;
} Point;

class CubicSpline1D
{
public:
    CubicSpline1D(std::vector<double> x, std::vector<double> y);
    ~CubicSpline1D();

public:
    double calculate_position(double x);
    double calculate_first_derivative(double x);
    double calculate_second_derivative(double x);

private:
    int search_index(double x);
    ModelMatrix calculate_a(std::vector<double> diff_x);
    ModelMatrix calculate_b(std::vector<double> diff_x, std::vector<double> coeff_a);

private:
    std::vector<double> x;
    std::vector<double> y;

    std::vector<double> a;
    std::vector<double> b;
    ModelMatrix c;
    std::vector<double> d;
};

class CubicSpline2D
{
public:
    CubicSpline2D(std::vector<WayPoint> waypoints);
    ~CubicSpline2D();

public:
    std::vector<Point> generate_spline_course(double speed, double ds=0.1);

private:
    void calculate_position(double s, double* x, double* y);
    double calculate_curvature(double s);
    double calculate_yaw(double s);
    std::vector<double> calculate_s(std::vector<double> x, std::vector<double> y);

private:
    std::vector<double> s;
    CubicSpline1D *sx;
    CubicSpline1D *sy;

    std::vector<double> ds;
};

double pi_2_pi(double angle);

#endif
