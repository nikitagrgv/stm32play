file(GLOB_RECURSE SOURCES "*.*")

add_subdirectory(commands)
add_subdirectory(core)
add_subdirectory(debug)
add_subdirectory(drivers)
add_subdirectory(low_level)
add_subdirectory(periph)
add_subdirectory(utils)

target_sources(${EXECUTABLE_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/DeviceCMSIS.h
        ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Print.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Print.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Sleep.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Sleep.h
)
