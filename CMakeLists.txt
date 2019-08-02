cmake_minimum_required(VERSION 3.0)
project(ScenRelationInference CXX C)

set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

### install lib, only do that if *.c or *.cpp exist
# since C srcs exist, use *.c* as pattern, not *.cpp

# Eigen structure
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# export cmake setting to the source code
include_directories(${PROJECT_SOURCE_DIR}/include)

# library
add_library(parse_tool SHARED ${PROJECT_SOURCE_DIR}/src/parse_tool.cpp)
add_library(geometry_feature SHARED ${PROJECT_SOURCE_DIR}/src/geometry_feature.cpp)
add_library(template_object SHARED ${PROJECT_SOURCE_DIR}/src/template_object.cpp)
add_library(pose_data_parse SHARED ${PROJECT_SOURCE_DIR}/src/pose_data_parse.cpp)

add_library(scene_inference SHARED ${PROJECT_SOURCE_DIR}/src/scene_inference.cpp)
target_link_libraries(scene_inference geometry_feature)

add_executable(YCB_template ${PROJECT_SOURCE_DIR}/include/configuru.hpp ${PROJECT_SOURCE_DIR}/src/create_ycb_template.cpp)
target_link_libraries(YCB_template Eigen3::Eigen parse_tool geometry_feature template_object scene_inference)

# linking with dependences
add_executable(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/main.cpp)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen geometry_feature template_object parse_tool scene_inference pose_data_parse)
