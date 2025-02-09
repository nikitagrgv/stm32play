set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.27)

option(ENABLE_DATA_STATISTIC "Enable Data Statistic" ON)
option(DEBUG_OPTIMIZED "Use -Og instead of -O0" OFF)

set(DEVICE_NAME "STM32F103" CACHE STRING "STM32F103 or STM32F401")
set_property(CACHE DEVICE_NAME PROPERTY STRINGS "STM32F103" "STM32F401")

# specify cross-compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# project settings
project(stmplay C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

add_compile_options(-fno-exceptions)

# uncomment to mitigate c++17 absolute addresses warnings
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register")

# Enable assembler files preprocessing
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    if (DEBUG_OPTIMIZED)
        add_compile_options(-Og)
    else ()
        add_compile_options(-O0)
    endif ()
    add_compile_options(-g)
    add_definitions(-DDEBUG)
endif ()

if ("${DEVICE_NAME}" STREQUAL "STM32F103")
    message(STATUS "Selected Device: F103")

    add_definitions(-DSTM32F103)
    add_definitions(-DSTM32F103xB)
    include_directories(Drivers/CMSIS/Device/ST/STM32F1xx/Include)
    set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/stm32f103c8tx_flash.ld)
    add_link_options(-mcpu=cortex-m3 -mthumb -mthumb-interwork -mfloat-abi=soft)
    add_compile_options(-mcpu=cortex-m3 -mthumb -mthumb-interwork -mfloat-abi=soft)
elseif ("${DEVICE_NAME}" STREQUAL "STM32F401")
    message(STATUS "Selected Device: F401")

    add_definitions(-DSTM32F401)
    add_definitions(-DSTM32F401xC)
    include_directories(Drivers/CMSIS/Device/ST/STM32F4xx/Include)
    set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/stm32f401ccux_flash.ld)
    add_link_options(-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
    add_compile_options(-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
else ()
    message(FATAL_ERROR "Invalid Device Name")
endif ()

include_directories(src Drivers/CMSIS/Include)

if (ENABLE_DATA_STATISTIC)
    add_definitions(-DENABLE_DATA_STATISTIC)
endif ()

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-T ${LINKER_SCRIPT})
add_link_options(--specs=nano.specs)
add_link_options(-u _printf_float)

set(EXECUTABLE_NAME ${PROJECT_NAME}.elf)

add_executable(${EXECUTABLE_NAME} ${LINKER_SCRIPT})

add_subdirectory(src)

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${EXECUTABLE_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${EXECUTABLE_NAME}> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${EXECUTABLE_NAME}> ${BIN_FILE}
        COMMENT " Building ${HEX_FILE}
            Building ${BIN_FILE}")
