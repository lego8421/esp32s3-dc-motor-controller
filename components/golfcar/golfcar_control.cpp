#include "golfcar_control.h"

#include <stdio.h>
#include <math.h>
#include <vector>

#include "cubic_spline_planner.h"

GolfCarControl::GolfCarControl()
{
    this->t = 0.0;
    this->command = kCommandInit;
    this->status = kStatusInit;
}

GolfCarControl::~GolfCarControl()
{
    
}

uint32_t GolfCarControl::set_way_points(std::vector<float> point)
{
    if (point.size() != 3) {
        return kValueSize;
    }

    if (point[2] > 2 * M_PI || point[2] < -2 * M_PI) {
        return kValueRange;
    }

    this->command = kCommandSpline;

    return 0;
}

void GolfCarControl::update(double dt) {
    this->t += dt;
    cycleUpdate();
}

void GolfCarControl::cycleUpdate() {
    switch (this->command) {
        case kCommandInit: {

        } break;
        case kCommandSpline: {
            std::vector<WayPoint> waypoints;
            waypoints.push_back(WayPoint(0.00, 0.00));
            waypoints.push_back(WayPoint(1.70, 2.70));
            waypoints.push_back(WayPoint(4.60, -0.20));
            waypoints.push_back(WayPoint(8.40, 2.70));
            waypoints.push_back(WayPoint(11.20, 5.30));
            waypoints.push_back(WayPoint(11.00, 8.10));
            waypoints.push_back(WayPoint(12.10, 10.20));
            waypoints.push_back(WayPoint(9.40, 12.80));
            waypoints.push_back(WayPoint(7.40, 11.30));
            waypoints.push_back(WayPoint(3.90, 14.10));
            waypoints.push_back(WayPoint(2.00, 12.10));
            waypoints.push_back(WayPoint(-0.50, 8.80));
            waypoints.push_back(WayPoint(-3.40, 7.50));
            waypoints.push_back(WayPoint(-6.50, 7.10));
            waypoints.push_back(WayPoint(-9.20, 3.80));
            waypoints.push_back(WayPoint(-9.70, 1.80));
            waypoints.push_back(WayPoint(-10.10, -0.70));
            waypoints.push_back(WayPoint(-11.50, -3.10));
            waypoints.push_back(WayPoint(-8.80, -4.30));
            waypoints.push_back(WayPoint(-6.30, -5.70));
            waypoints.push_back(WayPoint(-3.70, -6.80));
            waypoints.push_back(WayPoint(-0.80, -6.00));
            waypoints.push_back(WayPoint(1.60, -4.30));
            waypoints.push_back(WayPoint(0.00, 0.00));

            this->command = kCommandInit;
            this->status = kStatusSpline;
        } break;

        default: {

        } break;
    }

    switch (this->status) {
        case kStatusInit: {

        } break;

        case kStatusSpline: {

        } break;

        default: {

        } break;
    }
}