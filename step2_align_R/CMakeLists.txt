cmake_minimum_required(VERSION 2.8)

project(align_R)
set(CMAKE_CXX_STANDARD 11)

# ceres
find_package(Ceres REQUIRED)
# opencv
set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev)
find_package(OpenCV REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/include
    ${CERES_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} "main.cpp"
        src/setting.cc
        src/tool.cc
        src/optimize.cc)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${CERES_LIBRARIES} ${OpenCV_LIBS} ${EIGEN3_LIBS})
