# Host-side tests for the platform-independent modules; configured via the "host" preset.
set(CMAKE_C_EXTENSIONS OFF)

option(ACS5_SANITIZE "Build host tests with ASan + UBSan" ON)

set(SRC ${PROJECT_SOURCE_DIR}/Core/Src)

add_library(host_test_flags INTERFACE)
target_include_directories(host_test_flags INTERFACE
    ${PROJECT_SOURCE_DIR}/Core/Inc
    ${PROJECT_SOURCE_DIR}/Core/Inc/sensor_drivers
)
target_compile_options(host_test_flags INTERFACE -O2 -Wall -Wextra -Werror -UNDEBUG)
target_link_libraries(host_test_flags INTERFACE m)
if(ACS5_SANITIZE)
    target_compile_options(host_test_flags INTERFACE
        -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer)
    target_link_options(host_test_flags INTERFACE -fsanitize=address,undefined)
endif()

# acs5_host_test(<name> <module sources relative to Core/Src>...)
function(acs5_host_test name)
    list(TRANSFORM ARGN PREPEND ${SRC}/)
    add_executable(${name} ${name}.c ${ARGN})
    target_link_libraries(${name} PRIVATE host_test_flags)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

acs5_host_test(test_arming          control/arming.c control/flight_sm.c)
acs5_host_test(test_attitude_est    filter/attitude.c filter/mahony.c control/quat.c)
acs5_host_test(test_axes_remap      utils/axes_remap.c)
acs5_host_test(test_control_rate)
acs5_host_test(test_flight_sm       control/flight_sm.c)
acs5_host_test(test_geo_ned         utils/geo_ned.c)
acs5_host_test(test_ms5611_math     sensor_drivers/baro/ms5611_math.c)
acs5_host_test(test_quat            control/quat.c)
acs5_host_test(test_roll_ctrl       control/roll_ctrl.c control/quat.c)
acs5_host_test(test_roll_profile    control/roll_ctrl.c control/quat.c)
acs5_host_test(test_servo_bench     sensor_drivers/servos/servo_bench.c)
acs5_host_test(test_servo_t75_math  sensor_drivers/servos/servo_t75_math.c)
acs5_host_test(test_timestamp_math)
acs5_host_test(test_ubx_parse       sensor_drivers/gnss/ubx_parse.c)

add_executable(sim_roll sim_roll.c ${SRC}/control/roll_ctrl.c ${SRC}/control/quat.c)
target_link_libraries(sim_roll PRIVATE host_test_flags)
foreach(scenario nominal spin weak strong noisy trim)
    add_test(NAME sim_${scenario} COMMAND sim_roll scenario ${scenario})
endforeach()
add_test(NAME sim_monte_carlo COMMAND sim_roll mc 200 12345)
