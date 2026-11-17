cmake_minimum_required(VERSION 3.10)
project(xbox_wireless_usb_test VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)

add_executable(xbox_usb_test src/main.cpp)
target_link_libraries(xbox_usb_test PRIVATE PkgConfig::LIBUSB)
target_compile_options(xbox_usb_test PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS xbox_usb_test RUNTIME DESTINATION bin)

# Standalone PLC teleoperation (no ROS dependency)
add_executable(xbox_plc_teleop
    src/teleop_main.cpp
    src/gamepad.cpp
    src/plc_udp.cpp
)
target_include_directories(xbox_plc_teleop PRIVATE include)
target_link_libraries(xbox_plc_teleop PRIVATE PkgConfig::LIBUSB)
target_compile_options(xbox_plc_teleop PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS xbox_plc_teleop RUNTIME DESTINATION bin)

include(CTest)
if(BUILD_TESTING)
    add_executable(plc_packet_test tests/plc_packet_test.cpp src/plc_udp.cpp)
    target_include_directories(plc_packet_test PRIVATE include)
    target_compile_options(plc_packet_test PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME plc_packet_test COMMAND plc_packet_test)
endif()

# Camera apps are independent of the PLC motor-control executable.
# On Ubuntu, pkg-config opencv4 is more reliable than probing optional CMake
# package components; the package exports all linked OpenCV libraries incl. DNN.
pkg_check_modules(OPENCV4 QUIET IMPORTED_TARGET opencv4)
if(OPENCV4_FOUND)
    message(STATUS "OpenCV via pkg-config: ${OPENCV4_VERSION}")
    message(STATUS "OpenCV DNN available through opencv4 link interface")
    add_executable(person_vision src/person_vision_main.cpp)
    target_include_directories(person_vision PRIVATE include)
    target_link_libraries(person_vision PRIVATE PkgConfig::OPENCV4)
    target_compile_options(person_vision PRIVATE -Wall -Wextra -Wpedantic)
    install(TARGETS person_vision RUNTIME DESTINATION bin)

    add_executable(person_leg_vision src/person_leg_vision_main.cpp)
    target_include_directories(person_leg_vision PRIVATE include)
    target_link_libraries(person_leg_vision PRIVATE PkgConfig::OPENCV4)
    target_compile_options(person_leg_vision PRIVATE -Wall -Wextra -Wpedantic)
    install(TARGETS person_leg_vision RUNTIME DESTINATION bin)
    message(STATUS "person_leg_vision target: ENABLED")
else()
    message(WARNING "opencv4 pkg-config package not found: skipping camera apps. Install libopencv-dev and pkg-config.")
endif()

if(BUILD_TESTING)
    # The tracker itself is pure C++11 and can be tested without a camera.
    add_executable(person_tracker_test tests/person_tracker_test.cpp)
    target_include_directories(person_tracker_test PRIVATE include)
    target_compile_options(person_tracker_test PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME person_tracker_test COMMAND person_tracker_test)
endif()
