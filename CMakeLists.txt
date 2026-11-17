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

# Camera-only vision programs; never link them to PlcUdp or motor control.
# Resolve OpenCV ONCE. A second find_package with a different COMPONENTS list
# can invalidate OpenCV_FOUND in some distro configurations.
find_package(OpenCV QUIET)
if(OpenCV_FOUND)
    message(STATUS "OpenCV found: ${OpenCV_VERSION}")
    message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")

    add_executable(person_vision src/person_vision_main.cpp)
    target_include_directories(person_vision PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(person_vision PRIVATE ${OpenCV_LIBS})
    target_compile_options(person_vision PRIVATE -Wall -Wextra -Wpedantic)
    install(TARGETS person_vision RUNTIME DESTINATION bin)

    # OpenCV's exported target is authoritative when the package is installed
    # using apt on Ubuntu, including version 4.6.
    list(FIND OpenCV_LIBS opencv_dnn _opencv_dnn_index)
    if(TARGET opencv_dnn OR NOT _opencv_dnn_index EQUAL -1)
        add_executable(person_leg_vision src/person_leg_vision_main.cpp)
        target_include_directories(person_leg_vision PRIVATE include ${OpenCV_INCLUDE_DIRS})
        target_link_libraries(person_leg_vision PRIVATE ${OpenCV_LIBS})
        target_compile_options(person_leg_vision PRIVATE -Wall -Wextra -Wpedantic)
        install(TARGETS person_leg_vision RUNTIME DESTINATION bin)
        message(STATUS "person_leg_vision: enabled (OpenCV DNN available)")
    else()
        message(WARNING "OpenCV found but DNN component unavailable. Install libopencv-dev/libopencv-dnn-dev and reconfigure.")
    endif()
else()
    message(STATUS "OpenCV not found; skipping camera apps (install libopencv-dev)")
endif()

if(BUILD_TESTING)
    # The tracker itself is pure C++11 and can be tested without a camera.
    add_executable(person_tracker_test tests/person_tracker_test.cpp)
    target_include_directories(person_tracker_test PRIVATE include)
    target_compile_options(person_tracker_test PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME person_tracker_test COMMAND person_tracker_test)
endif()
