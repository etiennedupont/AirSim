// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <iostream>
#include <chrono>
#include <math.h>
#include "./DataRecord.hpp"
#include "./StateEstimator.hpp"

int main()
{
    using namespace msr::airlib;
    msr::airlib::MultirotorRpcLibClient client;
    StateEstimator::StateEstimator stateEstimator{ client };
    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;
    typedef common_utils::FileSystem FileSystem;

    try {
        client.confirmConnection();

        // std::cout << "Press Enter to get FPV image" << std::endl;
        // std::cin.get();
        vector<ImageRequest> request = { ImageRequest("0", ImageType::Scene), ImageRequest("1", ImageType::DepthPlanar, true) };
        const vector<ImageResponse>& response = client.simGetImages(request);
        std::cout << "# of images received: " << response.size() << std::endl;

        if (false && response.size() > 0) {
            std::cout << "Enter path with ending separator to save images (leave empty for no save)" << std::endl;
            std::string path;
            std::getline(std::cin, path);

            for (const ImageResponse& image_info : response) {
                std::cout << "Image uint8 size: " << image_info.image_data_uint8.size() << std::endl;
                std::cout << "Image float size: " << image_info.image_data_float.size() << std::endl;

                if (path != "") {
                    std::string file_path = FileSystem::combine(path, std::to_string(image_info.time_stamp));
                    std::cout << "file_path = " << file_path << std::endl;
                    if (image_info.pixels_as_float) {
                        std::cout << "case 0" << std::endl;
                        Utils::writePFMfile(image_info.image_data_float.data(), image_info.width, image_info.height, file_path + ".pfm");
                    }
                    else {
                        std::cout << "case 1" << std::endl;
                        std::ofstream file(file_path + ".png", std::ios::binary);
                        file.write(reinterpret_cast<const char*>(image_info.image_data_uint8.data()), image_info.image_data_uint8.size());
                        file.close();
                    }
                }
            }
        }

        std::cout << "Press Enter to arm the drone" << std::endl;
        std::cin.get();

        client.enableApiControl(true);
        client.armDisarm(true);

        // auto barometer_data = client.getBarometerData();
        // std::cout << "Barometer data \n"
        //           << "barometer_data.time_stamp \t" << barometer_data.time_stamp << std::endl
        //           << "barometer_data.altitude \t" << barometer_data.altitude << std::endl
        //           << "barometer_data.pressure \t" << barometer_data.pressure << std::endl
        //           << "barometer_data.qnh \t" << barometer_data.qnh << std::endl;

        // auto imu_data = client.getImuData();
        // std::cout << "IMU data \n"
        //           << "imu_data.time_stamp \t" << imu_data.time_stamp << std::endl
        //           << "imu_data.orientation \t" << imu_data.orientation << std::endl
        //           << "imu_data.angular_velocity \t" << imu_data.angular_velocity << std::endl
        //           << "imu_data.linear_acceleration \t" << imu_data.linear_acceleration << std::endl;

        // auto gps_data = client.getGpsData();
        // std::cout << "GPS data \n"
        //           << "gps_data.time_stamp \t" << gps_data.time_stamp << std::endl
        //           << "gps_data.gnss.time_utc \t" << gps_data.gnss.time_utc << std::endl
        //           << "gps_data.gnss.geo_point \t" << gps_data.gnss.geo_point << std::endl
        //           << "gps_data.gnss.eph \t" << gps_data.gnss.eph << std::endl
        //           << "gps_data.gnss.epv \t" << gps_data.gnss.epv << std::endl
        //           << "gps_data.gnss.velocity \t" << gps_data.gnss.velocity << std::endl
        //           << "gps_data.gnss.fix_type \t" << gps_data.gnss.fix_type << std::endl;

        // auto magnetometer_data = client.getMagnetometerData();
        // std::cout << "Magnetometer data \n"
        //           << "magnetometer_data.time_stamp \t" << magnetometer_data.time_stamp << std::endl
        //           << "magnetometer_data.magnetic_field_body \t" << magnetometer_data.magnetic_field_body << std::endl;

        float takeoff_timeout = 2;
        client.takeoffAsync(takeoff_timeout)->waitOnLastTask();

        // switch to explicit hover mode so that this is the fall back when
        // move* commands are finished.
        std::this_thread::sleep_for(std::chrono::duration<double>(5));
        client.hoverAsync()->waitOnLastTask();

        // std::cout << "Press Enter to fly in a 10m box pattern at 3 m/s velocity" << std::endl;
        // std::cin.get();
        // moveByVelocityZ is an offboard operation, so we need to set offboard mode.
        // client.enableApiControl(true);

        auto position = client.getMultirotorState().getPosition();
        float z = position.z(); // current position (NED coordinate system).
        const float speed = 3.0f;
        const float size = 30.0f;
        const float duration = size / speed;
        DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedom;
        YawMode yaw_mode(true, 360 / duration);

        // Rotate the drone by pi/2
        const float rotationYaw{ 3.1415 / 2 };
        const float durationRotation{ rotationYaw / speed };
        client.moveByRollPitchYawrateZAsync(0, 0, speed, z, durationRotation);
        std::this_thread::sleep_for(std::chrono::duration<double>(2.0));
        // Make a circle without yaw
        msr::airlib::vector<msr::airlib::Vector3r> myPath{};
        const float radiusCircle{ 5.0f };
        const int numberPoints{ 20 };
        for (int indexPoint{ 0 }; indexPoint < numberPoints; ++indexPoint) {
            myPath.push_back({ radiusCircle * sin(3.1415 * 2 * indexPoint / (numberPoints - 1)),
                               radiusCircle * (cos(3.1415 * 2 * indexPoint / (numberPoints - 1)) - 1),
                               z });
        }
        myPath.push_back({ 5, 0, z });
        myPath.push_back({ 5, 10, z });
        myPath.push_back({ 0, 10, z });
        myPath.push_back({ 0, 0, z });
        myPath.push_back({ -5, 0, z });
        myPath.push_back({ -5, 10, z });
        myPath.push_back({ 0, 10, z });
        myPath.push_back({ 0, 0, z });
        data_record::createFile("/Users/etiennedupont/Code/AirSim/DroneStateEstimation/output/gt.csv");
        data_record::createFile("/Users/etiennedupont/Code/AirSim/DroneStateEstimation/output/pred.csv");
        client.moveOnPathAsync(myPath, speed);
        data_record::recordSync(
            (int)(radiusCircle * 2 * 3.1415 / speed + 80 / speed + 1) * 1000,
            100,
            client,
            "/Users/etiennedupont/Code/AirSim/DroneStateEstimation/output/gt.csv",
            stateEstimator,
            "/Users/etiennedupont/Code/AirSim/DroneStateEstimation/output/pred.csv");
        client.hoverAsync()->waitOnLastTask();
        client.landAsync(takeoff_timeout)->waitOnLastTask();

        std::cout << "Press Enter to disarm" << std::endl;
        std::cin.get();
        client.reset();
        client.armDisarm(false);
    }
    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl
                  << msg << std::endl;
    }

    return 0;
}
