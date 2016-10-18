cmake_minimum_required(VERSION 2.8)
project(oni2klg)
find_package(OpenCV 2.4.3 REQUIRED)
find_package(OpenNI2 REQUIRED) 
include_directories(
${PROJECT_SOURCE_DIR}
${OpenNI2_INCLUDE_PATH}
${BOOST_INCLUDE_DIRS})
add_executable(oni2klg
oni2klg.cpp)
target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${OpenNI2_LIBRARY})

project(tum2klg)
find_package(OpenCV 2.4.3 REQUIRED)
find_package(OpenNI2 REQUIRED) 
find_package(Boost 1.45.0 COMPONENTS) 
add_executable(tum2klg
tum2klg.cpp kdtree.cpp)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${OpenNI2_LIBRARY})

