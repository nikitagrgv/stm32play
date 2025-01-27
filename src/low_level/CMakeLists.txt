
target_sources(${EXECUTABLE_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/startup_stm32f103c8tx.s
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32f1xx_it.c
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32f1xx_it.h
        ${CMAKE_CURRENT_SOURCE_DIR}/syscalls.c
        ${CMAKE_CURRENT_SOURCE_DIR}/sysmem.c
        ${CMAKE_CURRENT_SOURCE_DIR}/system_stm32f1xx.c
)

if ("${DEVICE_NAME}" STREQUAL "STM32F103")
    add_subdirectory(f103c8)
elseif ("${DEVICE_NAME}" STREQUAL "STM32F401")
    add_subdirectory(f401xc)
else ()
    message(FATAL_ERROR "Invalid Device Name")
endif ()
