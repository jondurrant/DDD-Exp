add_executable(${NAME}
        main.cpp
        
        Agent.cpp
        BlinkAgent.cpp
        GPIOInputMgr.cpp
        GPIOObserver.cpp
        
        MotorMgr.cpp
        MotorPID.cpp
        
        PubEntities.cpp
        MotorsAgent.cpp
        DDD.cpp
        HCSR04Agent.cpp
        
        uRosBridge.cpp
        uRosEntities.cpp
        pico_usb_transport.c
        freertos_allocators.c
        )

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(${NAME} 
	pico_stdlib 
	hardware_pwm
	FreeRTOS-Kernel-Heap4
	micro_ros
	pico_rand
	eigen
	distance_sensor
	)

# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})

# enable usb output, disable uart output
pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 1)
target_compile_definitions(${NAME} PRIVATE
    PICO_DEFAULT_UART_RX_PIN=16
    PICO_DEFAULT_UART_TX_PIN=17
)