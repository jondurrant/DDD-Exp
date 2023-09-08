if (DEFINED DISTANCE_SENSOR_PATH)
	message("Using Given DISTANCE_SENSOR_PATH '${DISTANCE_SENSOR_PATH}')")
else ()
	set(DISTANCE_SENSOR_PATH "${CMAKE_CURRENT_LIST_DIR}lib/pico-distance-sensor/")
    message("Using local DISTANCE_SENSOR_PATH '${DISTANCE_SENSOR_PATH}')")
endif ()

add_library(distance_sensor STATIC
    ${DISTANCE_SENSOR_PATH}/src/distance_sensor.cpp
)

pico_generate_pio_header(distance_sensor ${DISTANCE_SENSOR_PATH}/src/sensor.pio)

target_link_libraries(distance_sensor pico_stdlib hardware_pio hardware_irq)


# Add include directory
target_include_directories(distance_sensor PUBLIC 
    ${DISTANCE_SENSOR_PATH}/include
)
