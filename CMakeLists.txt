cmake_minimum_required(VERSION 3.5)
project(raspicam2)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

FIND_LIBRARY(mmal_LIBS REQUIRED NAMES mmal PATHS /opt/vc/lib)
FIND_LIBRARY(mmalcore_LIBS REQUIRED NAMES mmal_core PATHS /opt/vc/lib)
FIND_LIBRARY(mmalutil_LIBS REQUIRED NAMES mmal_util PATHS /opt/vc/lib)
FIND_LIBRARY(mmalvcclient_LIBS REQUIRED NAMES mmal_vc_client PATHS /opt/vc/lib)
FIND_LIBRARY(vcos_LIBS REQUIRED NAMES vcos PATHS /opt/vc/lib)
FIND_LIBRARY(bcmhost_LIBS REQUIRED NAMES bcm_host PATHS /opt/vc/lib)
FIND_LIBRARY(vchostif_LIBS REQUIRED NAMES vchostif PATHS /opt/vc/lib)

include_directories(include /opt/vc/include)

add_library(raspicli STATIC src/RaspiCLI.cpp)
add_library(raspicamcontrol STATIC src/RaspiCamControl.cpp)

add_executable(raspicam2_node src/raspicam2_node.cpp)
ament_target_dependencies(raspicam2_node rclcpp sensor_msgs)
target_link_libraries(raspicam2_node raspicamcontrol raspicli
    ${bcmhost_LIBS} ${vcos_LIBS} ${mmal_LIBS} ${mmalcore_LIBS}
    ${mmalutil_LIBS}  ${mmalvcclient_LIBS}  ${vchostif_LIBS} )

install(TARGETS
    raspicam2_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()