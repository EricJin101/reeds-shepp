cmake_minimum_required(VERSION 3.5.1)

project(reeds-shepp)
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_library(${TARGET_NAME} SHARED
        src/reeds_shepp_curve.cc
        )

add_executable(${PROJECT_NAME}_node main.cc)
