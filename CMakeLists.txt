cmake_minimum_required(VERSION 3.16)
project(drone_surveillance_system VERSION 1.0.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add compile options
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")

# Set output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Find required packages
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)

# Try to find spdlog, if not found we'll use header-only mode
find_package(spdlog QUIET)

# Find MAVSDK using pkg-config
pkg_check_modules(MAVSDK REQUIRED mavsdk)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${MAVSDK_INCLUDE_DIRS}
)

# Add link directories
link_directories(${MAVSDK_LIBRARY_DIRS})

# Define source files explicitly (better than GLOB)
set(SOURCES
    src/main.cpp
    src/DroneController.cpp
    src/mission_utils.cpp
    src/result_types.cpp
    src/logger.cpp
)

# Create main executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link libraries - include dl for dynamic linking
target_link_libraries(${PROJECT_NAME}
    mavsdk
    ${OpenCV_LIBS}
    Threads::Threads
    dl
)

# Link spdlog if found
if(spdlog_FOUND)
    target_link_libraries(${PROJECT_NAME} spdlog::spdlog)
    target_compile_definitions(${PROJECT_NAME} PRIVATE SPDLOG_COMPILED_LIB)
endif()

# Install targets
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
)

# Enable testing
enable_testing()

# Print configuration
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "MAVSDK libraries: ${MAVSDK_LIBRARIES}")
message(STATUS "MAVSDK include dirs: ${MAVSDK_INCLUDE_DIRS}")
message(STATUS "Source files: ${SOURCES}")