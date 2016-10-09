cmake_minimum_required(VERSION 2.8)
project(FormatConverter)
find_package(OpenCV 2.4.3 REQUIRED)
find_package(OpenNI2 REQUIRED) 
add_executable(FormatConverter
oni2klg.cpp)
target_link_libraries(FormatConverter ${PROJECT_NAME}
${OpenCV_LIBS}
${OpenNI2_LIBRARY})
