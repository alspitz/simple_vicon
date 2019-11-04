cmake_minimum_required(VERSION 2.8.3)
project(simple_vicon)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
      roscpp
)

catkin_package(
      INCLUDE_DIRS include
      LIBRARIES vicon
      CATKIN_DEPENDS
)

set(vicon_SDK_DIR "${CMAKE_CURRENT_LIST_DIR}/../vicon_mocap/vicon_driver/vicon_sdk/Linux64")

include_directories(include ${catkin_INCLUDE_DIRS} ${vicon_SDK_DIR})
link_directories(${catkin_LIBRARY_DIRS} ${vicon_SDK_DIR})
add_definitions(${catkin_DEFINITIONS})

add_library(viconsdk SHARED IMPORTED)
set_target_properties(viconsdk PROPERTIES IMPORTED_LOCATION "${vicon_SDK_DIR}/libViconDataStreamSDK_CPP.so")

add_library(multirotor_integrated
  src/ViconDriver.cpp
)

target_link_libraries(multirotor_integrated
  ${catkin_LIBRARIES}
  viconsdk
)

install(TARGETS multirotor_integrated DESTINATION lib)
install(FILES "${vicon_SDK_DIR}/libViconDataStreamSDK_CPP.so" DESTINATION lib)
install(FILES "${vicon_SDK_DIR}/libboost_chrono-mt.so.1.51.0" DESTINATION lib)
install(FILES "${vicon_SDK_DIR}/libboost_system-mt.so.1.51.0" DESTINATION lib)
install(FILES "${vicon_SDK_DIR}/libboost_thread-mt.so.1.51.0" DESTINATION lib)
