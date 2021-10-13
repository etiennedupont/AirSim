cmake_minimum_required(VERSION 3.5.0)
project(DroneStateEstimation)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake-modules") 
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../cmake-modules/CommonSetup.cmake")
CommonSetup()

IncludeEigen()

# Find GTSAM, either from a local build, or from a Debian/Ubuntu package.
find_package(GTSAM REQUIRED)
# Find boost
find_package(Boost COMPONENTS program_options REQUIRED)


SetupConsoleBuild()

## Specify additional locations of header files
include_directories(
  ${AIRSIM_ROOT}/DroneStateEstimation
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
)

AddExecutableSource()
			
CommonTargetLink()
target_link_libraries(${PROJECT_NAME} PRIVATE gtsam)
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PUBLIC AirLib)
target_link_libraries(${PROJECT_NAME} PUBLIC ${RPC_LIB})
