#ifndef GOLFCAR_CONTROL_H
#define GOLFCAR_CONTROL_H

#include <stdint.h>
#include <string>
#include <vector>


class GolfCarControl
{
public:
    GolfCarControl();
    ~GolfCarControl();

public:
    enum ErrorType
    {
        kValueSize = 1,
        kValueRange = 2,
    };

    enum Command
    {
        kCommandInit = 0,
        kCommandSpline = 1,
    };

    enum Status
    {
        kStatusInit = 0,
        kStatusSpline = 1,
    };

public:
    uint32_t set_way_points(std::vector<float> point);
    void update(double dt);

private:
    void cycleUpdate();

private:
    double t;
    Command command;
    Status status;
};

#endif