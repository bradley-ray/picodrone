cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(picodrone_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
pico_sdk_init()

add_executable(picodrone
	main.c
)

pico_enable_stdio_usb(picodrone 1)
pico_enable_stdio_uart(picodrone 0)

pico_add_extra_outputs(picodrone)

target_link_libraries(picodrone pico_stdlib)
