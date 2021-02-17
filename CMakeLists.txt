cmake_minimum_required(VERSION 2.8.3)
project(librover LANGUAGES CXX DESCRIPTION "C++ Library for communicating with Rover Robotics products")
set(DEFAULT_BUILD_TYPE CMAKE_CXX_FLAGS_RELEASE)
add_compile_options(-std=c++14)

include(GNUInstallDirs)
set(SOURCES_FILES 
src/comm_serial.cpp
src/comm_can.cpp
src/comm_base.cpp
src/utils.cpp)
add_library(librover ${PROJECT_SOURCE_DIR}/src/protocol_pro.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)



install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})
