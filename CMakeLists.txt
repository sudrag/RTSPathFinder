cmake_minimum_required(VERSION 3.10)
project(RTSPathFinder)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add executable
add_executable(RTSPathFinder src/main.cpp include/PathFinder.cpp)

# Include directories
target_include_directories(RTSPathFinder PUBLIC include)

# Define the data folder path
set(DATA_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}/data)

# Copy the data folder to the build directory
add_custom_target(CopyData ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${DATA_FOLDER} ${CMAKE_CURRENT_BINARY_DIR}/data
)

# Ensure the data files are available before running the executable
add_dependencies(${PROJECT_NAME} CopyData)

# Add libraries (using nlohmann_json)
find_package(nlohmann_json CONFIG REQUIRED)
target_link_libraries(RTSPathFinder PRIVATE nlohmann_json::nlohmann_json)
