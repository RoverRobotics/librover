cmake_minimum_required(VERSION 3.8.0)
project(librover LANGUAGES CXX DESCRIPTION "C++ Library for communicating with Rover Robotics products")

add_compile_options(-std=c++17 -lpthread -g -O0)
#add_compile_options(-std=c++17 -DDEBUG -lpthread)



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

# set_property(TARGET debug PROPERTY COMPILE_OPTIONS "-std=c++17;-pthread;-DDEBUG")

set(CMAKE_INSTALL_LIBDIR "/usr/lib")
set(CMAKE_INSTALL_INCLUDEDIR "/usr/include")
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

install(TARGETS librover
   LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
   PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})


install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})

add_compile_options(-std=c++17 -DDEBUG -lpthread -g -O0 -Lmpsse)
add_executable(debug.out src/debug.cpp)
target_link_libraries(debug.out LINK_PUBLIC librover pthread mpsse ftdi1)
