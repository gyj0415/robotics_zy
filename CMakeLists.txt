cmake_minimum_required(VERSION 3.22)
project(robotics VERSION 0.0.1 LANGUAGES C CXX ASM)

file(GLOB_RECURSE SOURCES
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc"
)

add_library(robotics STATIC ${SOURCES})
target_include_directories(robotics
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/internal
)

target_link_libraries(robotics PUBLIC bsp m components::math)

set_target_properties(robotics PROPERTIES
        C_STANDARD 17
        CXX_STANDARD 23
        CXX_STANDARD_REQUIRED YES
        POSITION_INDEPENDENT_CODE NO
)

add_library(components::robotics ALIAS robotics)
