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
