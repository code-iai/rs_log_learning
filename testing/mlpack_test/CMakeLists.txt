cmake_minimum_required (VERSION 2.6)
project (mlpacktest)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMake")
find_package(mlpack REQUIRED)
include_directories(${mlpack_INCLUDE_DIRS})
include_directories(${SDK_DIR}/usr/include/libxml2) # mlpack references just libxml
set(LIBS ${LIBS} ${mlpack_LIBRARIES})

message(${MLPACK_INCLUDE_DIR})
add_executable(mlpack_test mlpack_test.cxx)
target_link_libraries(mlpack_test mlpack armadillo)
