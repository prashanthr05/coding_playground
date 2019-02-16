cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)
find_package(YARP 3.1.100 REQUIRED COMPONENTS OS sig cv)
find_package(OpenCV REQUIRED)

project(VisualOdometry  VERSION 0.1.0.0
                        LANGUAGES CXX)

option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)

set(VISUAL_ODOM_INCLUDES #include/VisualOdom.h
                         include/Calibrator.h)

set(VISUAL_ODOM_SRC_FILES #src/VisualOdom.cpp
                          src/Calibrator.cpp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${OpenCV_INCLUDE_DIRS})
                    
add_library(calibrator ${VISUAL_ODOM_SRC_FILES}
                       ${VISUAL_ODOM_INCLUDES})

target_link_libraries(calibrator ${YARP_LIBRARIES} 
                                 ${OpenCV_LIBS})

add_executable(main src/main.cpp)

target_link_libraries(main calibrator ${OpenCV_LIBS})
