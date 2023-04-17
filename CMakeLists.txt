cmake_minimum_required(VERSION 3.8.0)
project(librover LANGUAGES CXX DESCRIPTION "C++ Library for communicating with Rover Robotics products")

add_compile_options(-std=c++17 -lpthread -g -O0)

add_library(librover SHARED
    src/protocol_pro.cpp
    src/comm_serial.cpp
    src/utils.cpp
    src/comm_can.cpp
    src/comm_can_spi.cpp
    src/protocol_pro_2.cpp
    src/control.cpp
    src/protocol_mini.cpp
    src/vesc.cpp
    src/utilities.cpp
    src/protocol_zero_2.cpp
    src/protocol_mega.cpp)

target_include_directories(librover PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    /usr/local/include
    PRIVATE src)

find_library(LIBMPSSE_LIBRARIES NAMES mpsse PATHS /usr/local/lib)

target_link_libraries(librover
    ${LIBMPSSE_LIBRARIES}
    pthread
    ftdi1)

install(TARGETS librover
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})

add_executable(debug.out src/debug.cpp)

target_link_libraries(debug.out
    librover
    pthread
    ftdi1)