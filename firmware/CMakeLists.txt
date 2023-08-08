cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(picodrone_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PICO_BOARD pico_w)
pico_sdk_init()

add_executable(picodrone
	src/main.c
	lib/bluetooth/bt.c
	lib/mpu6050/mpu.c
	lib/control/control.c
)


pico_enable_stdio_usb(picodrone 1)
pico_enable_stdio_uart(picodrone 0)

pico_add_extra_outputs(picodrone)

target_include_directories(picodrone PRIVATE
	lib/bluetooth
	lib/mpu6050
	lib/control
	config
)

target_link_libraries(picodrone 
	pico_stdlib
	pico_cyw43_arch_none
	pico_btstack_classic
	pico_btstack_cyw43

	hardware_i2c
	hardware_pwm
)

