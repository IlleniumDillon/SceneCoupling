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
    ${CMAKE_SOURCE_DIR}/MapRepresent/
    ${CMAKE_SOURCE_DIR}/PathPlan/
    ${CMAKE_SOURCE_DIR}/Visualize/
    ${CMAKE_SOURCE_DIR}/SceneRepresent/
)

# Add the source files
file(GLOB MapRepresent_SRC ${CMAKE_SOURCE_DIR}/MapRepresent/*.cpp)
file(GLOB PathPlan_SRC ${CMAKE_SOURCE_DIR}/PathPlan/*.cpp)
file(GLOB Visualize_SRC ${CMAKE_SOURCE_DIR}/Visualize/*.cpp)
file(GLOB SceneRepresent_SRC ${CMAKE_SOURCE_DIR}/SceneRepresent/*.cpp)

# Add the executable target
add_executable(
    PathPlanningTest
    testmain.cpp
    ${MapRepresent_SRC}
    ${PathPlan_SRC}
    ${Visualize_SRC}
    ${SceneRepresent_SRC}
)

# Link the libraries
target_link_libraries(
    PathPlanningTest
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
)