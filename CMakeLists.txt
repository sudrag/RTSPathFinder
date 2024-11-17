cmake_minimum_required(VERSION 3.10)
project(RTSPathFinder)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add executable
add_executable(RTSPathFinder src/main.cpp include/PathFinder.cpp)

# Include directories
target_include_directories(RTSPathFinder PUBLIC include)

# Add libraries (using nlohmann_json)
find_package(nlohmann_json CONFIG REQUIRED)
target_link_libraries(RTSPathFinder PRIVATE nlohmann_json::nlohmann_json)
