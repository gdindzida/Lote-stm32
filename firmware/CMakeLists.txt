cmake_minimum_required(VERSION 3.20)

project(Lote-stm32 C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(MCU_FLAGS
-mcpu=cortex-m4
-mthumb
-mfloat-abi=hard
-mfpu=fpv4-sp-d16
)

add_compile_options(
${MCU_FLAGS}
-ffreestanding
-fno-exceptions
-fno-rtti
-Wall
)

add_definitions(-DSTM32G4)

add_link_options(
${MCU_FLAGS}
-T${CMAKE_SOURCE_DIR}/ld/linker.ld
-nostartfiles
)

include_directories(
../modules/libopencm3/include
)

link_directories(
../modules/libopencm3/lib
)

add_executable(firmware.elf
src/main.cpp
src/syscall.c
src/sysmem.c
startup/startup.S
)

target_link_libraries(firmware.elf
opencm3_stm32g4
)

add_custom_command(TARGET firmware.elf POST_BUILD
COMMAND arm-none-eabi-objcopy -O binary firmware.elf firmware.bin
COMMAND arm-none-eabi-size firmware.elf
)