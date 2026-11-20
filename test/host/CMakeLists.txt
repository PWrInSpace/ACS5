# Host-side tests for the platform-independent modules; configured via the "host" preset.
set(CMAKE_C_EXTENSIONS OFF)

option(ACS5_SANITIZE "Build host tests with ASan + UBSan" ON)

set(SRC ${PROJECT_SOURCE_DIR}/Core/Src)

add_library(host_test_flags INTERFACE)
target_include_directories(host_test_flags INTERFACE
    ${PROJECT_SOURCE_DIR}/Core/Inc
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

acs5_host_test(test_arming          flight/arming.c flight/flight_sm.c)
acs5_host_test(test_attitude_est    filter/attitude.c filter/mahony.c utils/quat.c)
acs5_host_test(test_axes_remap      utils/axes_remap.c)
acs5_host_test(test_baro_datum      flight/baro_datum.c)
acs5_host_test(test_control_rate)
acs5_host_test(test_estimator_policy flight/estimator_policy.c)
acs5_host_test(test_flight_sm       flight/flight_sm.c)
acs5_host_test(test_flight_pipeline filter/attitude.c filter/mahony.c utils/quat.c
               flight/flight_sm.c flight/estimator_policy.c control/qbar.c)
acs5_host_test(test_geo_ned         utils/geo_ned.c)
acs5_host_test(test_log_record      log/log_record.c)
acs5_host_test(test_ms5611_math     sensor_drivers/baro/ms5611_math.c)
acs5_host_test(test_mmc5983ma_math  sensor_drivers/mag/mmc5983ma_math.c)
acs5_host_test(test_qbar            control/qbar.c)
acs5_host_test(test_quat            utils/quat.c)
acs5_host_test(test_roll_ctrl       control/roll_ctrl.c utils/quat.c)
acs5_host_test(test_roll_profile    control/roll_ctrl.c utils/quat.c)
acs5_host_test(test_servo_bench     sensor_drivers/servos/servo_bench.c)
acs5_host_test(test_servo_t75_math  sensor_drivers/servos/servo_t75_math.c)
acs5_host_test(test_timestamp_math)
acs5_host_test(test_ubx_parse       sensor_drivers/gnss/ubx_parse.c)

add_executable(sim_roll sim_roll.c ${SRC}/control/roll_ctrl.c ${SRC}/utils/quat.c)
target_link_libraries(sim_roll PRIVATE host_test_flags)
foreach(scenario nominal spin weak strong noisy trim)
    add_test(NAME sim_${scenario} COMMAND sim_roll scenario ${scenario})
endforeach()
add_test(NAME sim_monte_carlo COMMAND sim_roll mc 200 12345)

find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_test(NAME log_sample_write
         COMMAND test_log_record write ${CMAKE_CURRENT_BINARY_DIR}/sample_log.bin)
set_tests_properties(log_sample_write PROPERTIES FIXTURES_SETUP log_sample)
add_test(NAME decode_log
         COMMAND Python3::Interpreter ${CMAKE_CURRENT_SOURCE_DIR}/check_decode_log.py
                 ${CMAKE_CURRENT_BINARY_DIR}/sample_log.bin)
set_tests_properties(decode_log PROPERTIES FIXTURES_REQUIRED log_sample)
