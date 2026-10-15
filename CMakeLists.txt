cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME ACS5)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# LTO for flight builds only; Debug stays -O0/-g3 debuggable.
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/control/arming.c
    Core/Src/control/flight_sm.c
    Core/Src/control/quat.c
    Core/Src/control/roll_ctrl.c
    Core/Src/filter/attitude.c
    Core/Src/filter/mahony.c
    Core/Src/sensor_drivers/aggregator.c
    Core/Src/sensor_drivers/spi_dev.c
    Core/Src/sensor_drivers/imu/iim42653.c
    Core/Src/sensor_drivers/baro/ms5611.c
    Core/Src/sensor_drivers/baro/ms5611_math.c
    Core/Src/sensor_drivers/mag/mmc5983ma.c
    Core/Src/sensor_drivers/servos/servo_t75.c
    Core/Src/sensor_drivers/servos/servo_t75_math.c
    Core/Src/sensor_drivers/gnss/max_m10s.c
    Core/Src/sensor_drivers/gnss/ubx_parse.c
    Core/Src/tasks/gnss_task.c
    Core/Src/usb_print.c
    Core/Src/utils/axes_remap.c
    Core/Src/utils/timestamp.c
    Core/Src/buzzer/buzzer.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc
    Core/Inc/sensor_drivers
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Add linker options
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -u _printf_float
)
