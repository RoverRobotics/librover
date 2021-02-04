cmake_minimum_required(VERSION 2.8.3)
project(rover-common LANGUAGES CXX DESCRIPTION "RoverRobotic Robot Common Driver Library")
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
add_library(rover-common ${PROJECT_SOURCE_DIR}/src/protocol_zero.cpp ${PROJECT_SOURCE_DIR}/src/protocol_pro.cpp)

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

# add_library(rover-common SHARED
#                         src/protocol_pro.cpp
#                         src/protocol_zero.cpp
#                         src/protocol_base.cpp
#                         src/comm_serial.cpp
#                         src/comm_can.cpp
#                         src/comm_base.cpp
#                         src/utils.cpp)

# # set_target_properties(rover-common PROPERTIES VERSION ${PROJECT_VERSION})
# ##set output .version 
# #set_target_properties(mylib PROPERTIES SOVERSION 1)

# set_target_properties(rover-common PROPERTIES PUBLIC_HEADER include/rover-common.h)
# target_include_directories(rover-common PRIVATE include)
# include(GNUInstallDirs)

# ###########
# ## Build ##
# ###########

# install(TARGETS rover-common
#     LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# configure_file(rover-common.pc.in rover-common.pc @ONLY)

# ###########
# ## Install ##
# ###########
# install(TARGETS rover-common
#     LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
# install(FILES ${CMAKE_BINARY_DIR}/rover-common.pc DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig)
