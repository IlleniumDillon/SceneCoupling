# Set the minimum required version of CMake
cmake_minimum_required(VERSION 3.0)

# Set the project name
project(PathPlanning)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)

# find the required packages
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Add the include directory
include_directories(
    ${CMAKE_SOURCE_DIR}/AgentRepresent/
    ${CMAKE_SOURCE_DIR}/PathPlan/
    ${CMAKE_SOURCE_DIR}/SceneRepresent/
    ${CMAKE_SOURCE_DIR}/Visualize/
    ${CMAKE_SOURCE_DIR}/Thirdparty/jsoncpp/include/
)

# Add the source files
file(GLOB AgentRepresent_SRC ${CMAKE_SOURCE_DIR}/AgentRepresent/*.cpp)
file(GLOB PathPlan_SRC ${CMAKE_SOURCE_DIR}/PathPlan/*.cpp)
file(GLOB SceneRepresent_SRC ${CMAKE_SOURCE_DIR}/SceneRepresent/*.cpp)
file(GLOB Visualize_SRC ${CMAKE_SOURCE_DIR}/Visualize/*.cpp)
set(
    JSONCPP_SRC
    ${CMAKE_SOURCE_DIR}/Thirdparty/jsoncpp/src/lib_json/json_reader.cpp
    ${CMAKE_SOURCE_DIR}/Thirdparty/jsoncpp/src/lib_json/json_value.cpp
    ${CMAKE_SOURCE_DIR}/Thirdparty/jsoncpp/src/lib_json/json_writer.cpp
    ${CMAKE_SOURCE_DIR}/Thirdparty/jsoncpp/src/lib_json/json_tool.h
    ${CMAKE_SOURCE_DIR}/Thirdparty/jsoncpp/src/lib_json/json_valueiterator.inl
)

# Add the executable target
add_executable(
    PathPlanningTest
    testmain.cpp
    ${AgentRepresent_SRC}
    ${PathPlan_SRC}
    ${SceneRepresent_SRC}
    ${Visualize_SRC}
    ${JSONCPP_SRC}
)

# Link the libraries
target_link_libraries(
    PathPlanningTest
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
)