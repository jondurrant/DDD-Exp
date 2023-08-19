if (DEFINED MICRO_ROS_PATH)
	message("Using Given MICRO_ROS_PATH '${MICRO_ROS_PATH}')")
else ()
	set(MICRO_ROS_PATH "${CMAKE_CURRENT_LIST_DIR}lib/micro_ros_raspberrypi_pico_sdk/")
    message("Using local MICRO_ROS_PATH '${MICRO_ROS_PATH}')")
endif ()

add_library(micro_ros STATIC)
#target_sources(micro_ros PUBLIC
   # ${MICRO_ROS_PATH}/pico_uart_transport.c
#)

# Add include directory
target_include_directories(micro_ros PUBLIC 
    ${MICRO_ROS_PATH}
    ${MICRO_ROS_PATH}/libmicroros/include
)

# Add the standard library to the build
target_link_libraries(micro_ros PUBLIC pico_stdlib microros)

link_directories(${MICRO_ROS_PATH}/libmicroros)