
target_sources(${EXECUTABLE_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/startup_stm32f103c8tx.s
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32f1xx_it.c
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32f1xx_it.h
        ${CMAKE_CURRENT_SOURCE_DIR}/syscalls.c
        ${CMAKE_CURRENT_SOURCE_DIR}/sysmem.c
        ${CMAKE_CURRENT_SOURCE_DIR}/system_stm32f1xx.c
)
