cmake_minimum_required(VERSION 3.0)
find_package(YARP REQUIRED)

add_executable(transformTest src/tf_test.cpp)
target_link_libraries(transformTest ${YARP_LIBRARIES})

