#include <string>
#include <iostream>
#include "common/ClockFactory.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "./StateEstimator.hpp"

StateEstimator::StateEstimator::StateEstimator(msr::airlib::MultirotorRpcLibClient& clientArg): client{clientArg}{};
StateEstimator::State StateEstimator::StateEstimator::getState(){
            return {
                time: 0,
                x: 0.0,
                y: 0.0,
                z: 0.0,
                qx: 0.0,
                qy: 0.0,
                qz: 0.0,
                qw: 0.0,
                err_x: 0.0,
                err_y: 0.0,
                err_z: 0.0,
                err_qx: 0.0,
                err_qy: 0.0,
                err_qz: 0.0,
                err_qw: 0.0,
            };
        };