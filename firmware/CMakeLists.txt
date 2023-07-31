cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)
include(FreeRTOS_Kernel_import.cmake)

project(picodrone_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PICO_BOARD pico_w)
pico_sdk_init()

add_executable(picodrone
	main.c
)

pico_enable_stdio_usb(picodrone 1)
pico_enable_stdio_uart(picodrone 0)


target_include_directories(picodrone PRIVATE
	${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(picodrone 
	pico_stdlib 
	pico_cyw43_arch_none
	FreeRTOS-Kernel 
	FreeRTOS-Kernel-Heap1
)

pico_add_extra_outputs(picodrone)
