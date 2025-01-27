
target_sources(${EXECUTABLE_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/startup_stm32f401xc.s
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32f4xx_it.c
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32f4xx_it.h
        ${CMAKE_CURRENT_SOURCE_DIR}/system_stm32f4xx.c
)
