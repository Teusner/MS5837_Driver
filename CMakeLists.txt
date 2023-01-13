cmake_minimum_required(VERSION 3.1)
project(MS5837_Driver VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17 CACHE STRING "The C++ standard to use")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Driver Include dir
set(MS5837_DRIVER_HEADERS include)

# Driver
add_library(ms5837_driver src/ms5837_driver.cpp)
target_include_directories(ms5837_driver PUBLIC ${MS5837_DRIVER_HEADERS})

# Detecting if project is included into another
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    option(MY_PROJECT_BUILD_EXAMPLES   "whether or not examples should be built" ON)

    if(MY_PROJECT_BUILD_EXAMPLES)
        add_subdirectory(examples)
    endif()
endif()