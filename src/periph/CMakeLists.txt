
add_subdirectory(device)

target_sources(${EXECUTABLE_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/EXTI.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/EXTI.h
        ${CMAKE_CURRENT_SOURCE_DIR}/GPIO.h
        ${CMAKE_CURRENT_SOURCE_DIR}/IRQ.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/IRQ.h
        ${CMAKE_CURRENT_SOURCE_DIR}/PeriphBase.h
        ${CMAKE_CURRENT_SOURCE_DIR}/RCC.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/RCC.h
        ${CMAKE_CURRENT_SOURCE_DIR}/SysTick.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/SysTick.h
        ${CMAKE_CURRENT_SOURCE_DIR}/TIM.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TIM.h
        ${CMAKE_CURRENT_SOURCE_DIR}/USART.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/USART.h
)
