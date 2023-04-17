cmake_minimum_required(VERSION 3.8.0)
project(librover LANGUAGES CXX DESCRIPTION "C++ Library for communicating with Rover Robotics products")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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
    src/protocol_mega.cpp
)

# Find and include the mpsse library
find_library(LIBMPSSE_LIBRARIES NAMES mpsse)
find_path(LIBMPSSE_INCLUDE_DIR mpsse.h)
if(LIBMPSSE_LIBRARIES AND LIBMPSSE_INCLUDE_DIR)
    target_include_directories(librover PRIVATE ${LIBMPSSE_INCLUDE_DIR})
    target_link_libraries(librover PRIVATE ${LIBMPSSE_LIBRARIES})
else()
    message(FATAL_ERROR "Could not find libmpsse library")
endif()

target_include_directories(librover PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src
)

install(TARGETS librover
    LIBRARY DESTINATION lib
    PUBLIC_HEADER DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

add_executable(debug.out src/debug.cpp)
target_link_libraries(debug.out PRIVATE librover pthread ftdi1)