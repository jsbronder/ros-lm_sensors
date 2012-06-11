cmake_minimum_required(VERSION 2.6.0)

# Has to be before we call add_executable()
execute_process(COMMAND
    rospack libs-only-L diagnostic_updater
    OUTPUT_STRIP_TRAILING_WHITESPACE
    OUTPUT_VARIABLE libs_only_L)
separate_arguments(libs_only_L)
link_directories(${libs_only_L})

add_executable(lm_sensors
    LMSensor.cpp
    LMSensorList.cpp
    main.cpp)

execute_process(COMMAND
    rospack libs-only-l diagnostic_updater
    OUTPUT_STRIP_TRAILING_WHITESPACE
    OUTPUT_VARIABLE libs_only_l)
separate_arguments(libs_only_l)
target_link_libraries(lm_sensors ${libs_only_l} sensors)

execute_process(COMMAND
    rospack cflags-only-I diagnostic_updater
    OUTPUT_STRIP_TRAILING_WHITESPACE
    OUTPUT_VARIABLE cflags_only_I)
separate_arguments(cflags_only_I)
include_directories(${cflags_only_I})

execute_process(COMMAND
    rospack cflags-only-other diagnostic_updater
    OUTPUT_STRIP_TRAILING_WHITESPACE
    OUTPUT_VARIABLE cflags_only_other)
if (cflags_only_other)
    set_target_properties(lm_sensors
        PROPERTIES
        COMPILE_FLAGS ${cflags_only_other})
endif()

execute_process(COMMAND
    rospack libs-only-other diagnostic_updater
    OUTPUT_STRIP_TRAILING_WHITESPACE
    OUTPUT_VARIABLE libs_only_other)
if (libs_only_other)
    set_target_properties(lm_sensors
        PROPERTIES
        LINK_FLAGS ${libs_only_other})
endif()

