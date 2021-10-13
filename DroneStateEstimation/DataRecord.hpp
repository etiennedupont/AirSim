#ifndef DATA_RECORD_H
#define DATA_RECORD_H

#include <sys/stat.h>
#include <string>
#include <iostream>
#include <fstream>
#include "common/ClockFactory.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "./StateEstimator.hpp"

namespace data_record
{
void addLineToFile(
    const std::string& filePath,
    StateEstimator::State line)
{
    std::ofstream file;
    file.open(filePath, std::ios::app);
    file << line.time << ','
         << line.x << ',' << line.y << ',' << line.z << ','
         << line.qx << ',' << line.qy << ',' << line.qz << ',' << line.qw << ','
         << line.err_x << ',' << line.err_y << ',' << line.err_z << ','
         << line.err_qx << ',' << line.err_qy << ',' << line.err_qz << ',' << line.err_qw << '\n';
    file.close();
}
void createFile(const std::string& filePath)
{
    std::ofstream file;
    file.open(filePath);
    file << "time,x,y,z,qx,qy,qz,qw,err_x,err_y,err_z,err_qx,err_qy,err_qz,err_qw\n";
    file.close();
}
inline bool file_exist(const std::string& name)
{
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}
void recordDataPoint(
    msr::airlib::MultirotorRpcLibClient& client,
    const std::string& filePathGroundTruth,
    StateEstimator::StateEstimator& stateEstimator,
    const std::string& filePathPrediction)
{
    const uint64_t timestamp_millis = static_cast<uint64_t>(msr::airlib::ClockFactory::get()->nowNanos() / 1.0E6);
    msr::airlib::Kinematics::State state{ client.simGetGroundTruthKinematics() };
    addLineToFile(filePathGroundTruth, {
        time : timestamp_millis,
        x : state.pose.position.x(),
        y : state.pose.position.y(),
        z : state.pose.position.z(),
        qx : state.pose.orientation.x(),
        qy : state.pose.orientation.y(),
        qz : state.pose.orientation.z(),
        qw : state.pose.orientation.w(),
        err_x : 0,
        err_y : 0,
        err_z : 0,
        err_qx : 0,
        err_qy : 0,
        err_qz : 0,
        err_qw : 0,
    });
    addLineToFile(filePathPrediction, stateEstimator.getState());
}

void recordSync(
    uint64_t timeRecording,
    uint64_t periodMs,
    msr::airlib::MultirotorRpcLibClient& client,
    const std::string& filePathGroundTruth,
    StateEstimator::StateEstimator& stateEstimator,
    const std::string& filePathPrediction)
{
    const uint64_t start{ static_cast<uint64_t>(msr::airlib::ClockFactory::get()->nowNanos() / 1.0E6) };
    uint64_t lastCheckpoint{ start };
    while (lastCheckpoint - start < timeRecording) {
        const uint64_t current{ static_cast<uint64_t>(msr::airlib::ClockFactory::get()->nowNanos() / 1.0E6) };
        if (current - lastCheckpoint >= periodMs) {
            lastCheckpoint = current;
            recordDataPoint(client, filePathGroundTruth, stateEstimator, filePathPrediction);
        }
    }
}
}
#endif