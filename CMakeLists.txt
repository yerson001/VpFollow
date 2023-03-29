cmake_minimum_required(VERSION 2.8)

project(VpFollow)

find_package( OpenCV REQUIRED )
find_package(Eigen3 REQUIRED)
set(CMAKE_CXX_STANDARD 17)
MESSAGE( [Main] " EIGEN3_INCLUDE_DIRS = ${EIGEN3_INCLUDE_DIRS}")
include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories( ${OpenCV_INCLUDE_DIRS})

file(GLOB INCLUDE_FILES
  "include/*.h"
)
include_directories(include)

MACRO(add_example name)
  ADD_EXECUTABLE(${name}
      source/${name}.cpp
      include/functions.h
      include/laneDetection.h
      include/vanishpoint.h
      include/transformation.h)

  TARGET_LINK_LIBRARIES(${name} ${OpenCV_LIBS})
ENDMACRO()

add_example(main)
