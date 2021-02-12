cmake_minimum_required(VERSION 2.8.3)
project(librover LANGUAGES CXX DESCRIPTION "C++ Library for communicating with Rover Robotics products")
set(DEFAULT_BUILD_TYPE CMAKE_CXX_FLAGS_RELEASE)
add_compile_options(-std=c++14)

include(GNUInstallDirs)
set(SOURCES_FILES 
# src/protocol_pro.cpp
# src/protocol_zero.cpp
src/protocol_base.cpp
src/comm_serial.cpp
src/comm_can.cpp
src/comm_base.cpp
src/utils.cpp)
add_library(librover ${PROJECT_SOURCE_DIR}/src/protocol_zero.cpp ${PROJECT_SOURCE_DIR}/src/protocol_pro.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

# set_target_properties(${PROJECT_NAME} PROPERTIES
#     VERSION ${PROJECT_VERSION}
#     SOVERSION 1)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})

# install(EXPORT MyLibConfig DESTINATION share/MyLib/cmake)
# ###########
# ## Build ##
# ###########
# src/protocol_pro.cpp
#                         src/protocol_zero.cpp
#                         src/protocol_base.cpp
#                         src/comm_serial.cpp
#                         src/comm_can.cpp
#                         src/comm_base.cpp
#                         src/utils.cpp
# include_directories(
#   include
# )

# add_library(librover SHARED
#                         src/protocol_pro.cpp
#                         src/protocol_zero.cpp
#                         src/protocol_base.cpp
#                         src/comm_serial.cpp
#                         src/comm_can.cpp
#                         src/comm_base.cpp
#                         src/utils.cpp)

# # set_target_properties(librover PROPERTIES VERSION ${PROJECT_VERSION})
# ##set output .version 
# #set_target_properties(mylib PROPERTIES SOVERSION 1)

# set_target_properties(librovern PROPERTIES PUBLIC_HEADER include/librover.h)
# target_include_directories(librover PRIVATE include)
# include(GNUInstallDirs)

# ###########
# ## Build ##
# ###########

# install(TARGETS librover
#     LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# configure_file(librover.pc.in librover.pc @ONLY)

# ###########
# ## Install ##
# ###########
# install(TARGETS librover
#     LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
# install(FILES ${CMAKE_BINARY_DIR}/librover.pc DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig)
