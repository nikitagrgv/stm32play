
target_sources(${EXECUTABLE_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/CommandBuffer.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/CommandBuffer.h
        ${CMAKE_CURRENT_SOURCE_DIR}/CommandExecutor.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/CommandExecutor.h
        ${CMAKE_CURRENT_SOURCE_DIR}/DHT11Command.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/DHT11Command.h
        ${CMAKE_CURRENT_SOURCE_DIR}/PrintCommand.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/PrintCommand.h
)
