#ifndef STATE_ESTIMATE_H
#define STATE_ESTIMATE_H

#include <string>
#include <iostream>
#include <thread>
#include "common/ClockFactory.hpp"
#include "common/GeodeticConverter.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

namespace StateEstimator
{
struct State
{
    uint64_t time;
    double x;
    double y;
    double z;
    double qx;
    double qy;
    double qz;
    double qw;
    double err_x;
    double err_y;
    double err_z;
    double err_qx;
    double err_qy;
    double err_qz;
    double err_qw;
};

class StateEstimator
{
private:
    msr::airlib::MultirotorRpcLibClient& client;
    msr::airlib::GeodeticConverter geodeticConverter;
    State state;
    bool shouldStop;
    std::thread* threadPtr;

public:
    StateEstimator(msr::airlib::MultirotorRpcLibClient& client);
    State getState();
    void start();
    void stop();
    void runEstimatorSync();
};
}

#endif