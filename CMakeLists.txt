# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/cmfcmf/dev/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(my-project C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(my-project main.cpp ws2811.cpp)

pico_generate_pio_header(my-project ${CMAKE_CURRENT_LIST_DIR}/ws2811.pio)

pico_set_program_name(my-project "my-project")
pico_set_program_version(my-project "0.1")

pico_enable_stdio_uart(my-project 1)
pico_enable_stdio_usb(my-project 1)

# Add the standard library to the build
target_link_libraries(my-project pico_stdlib pico_multicore pico_mem_ops hardware_pio hardware_dma)

pico_add_extra_outputs(my-project)

