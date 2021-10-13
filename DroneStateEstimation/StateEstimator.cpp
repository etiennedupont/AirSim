#include <string>
#include <iostream>
#include "common/ClockFactory.hpp"
#include "common/GeodeticConverter.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/GeodeticConverter.hpp"
#include "./StateEstimator.hpp"

StateEstimator::StateEstimator::StateEstimator(msr::airlib::MultirotorRpcLibClient& clientArg)
    : client{clientArg}{
        msr::airlib::GpsBase::Output gpsData{this->client.getGpsData()};
        this->geodeticConverter = msr::airlib::GeodeticConverter(gpsData.gnss.geo_point);
    };

StateEstimator::State StateEstimator::StateEstimator::getState(){
            msr::airlib::GpsBase::Output gpsData{this->client.getGpsData()};
            double x{};
            double y{};
            double z{};
            geodeticConverter.geodetic2Ned(gpsData.gnss.geo_point.latitude, 
                gpsData.gnss.geo_point.longitude, gpsData.gnss.geo_point.altitude, &x, &y, &z);
            return {
                time: gpsData.time_stamp,
                x: x,
                y: y,
                z: z,
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