cmake_minimum_required(VERSION 2.8.3)
project(simple_vicon)

find_package(Git QUIET)
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
# Update submodules as needed
    option(GIT_SUBMODULE "Check submodules during build" ON)
    if(GIT_SUBMODULE)
        message(STATUS "Submodule update")
        execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                        RESULT_VARIABLE GIT_SUBMOD_RESULT)
        if(NOT GIT_SUBMOD_RESULT EQUAL "0")
            message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
        endif()
    endif()
endif()

if(NOT EXISTS "${PROJECT_SOURCE_DIR}/extern/vicon-datastream-sdk/CMakeLists.txt")
    message(FATAL_ERROR "The submodules were not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
endif()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  tf
)

add_message_files(
  FILES
  Subject.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
      INCLUDE_DIRS include
      LIBRARIES vicon_driver vicon_driver_ros
      CATKIN_DEPENDS roscpp
)

add_subdirectory(extern/vicon-datastream-sdk)

add_library(vicon_driver
  src/ViconDriver.cpp
)
target_include_directories(vicon_driver PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(vicon_driver PUBLIC
  ${catkin_LIBRARIES}
  ViconDataStreamSDK_CPP
)
target_compile_features(vicon_driver PRIVATE cxx_std_17)

add_library(vicon_driver_ros
  src/ViconDriverROS.cpp
)
target_include_directories(vicon_driver_ros PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(vicon_driver_ros PUBLIC ${catkin_LIBRARIES} vicon_driver)
add_dependencies(vicon_driver_ros ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(vicon_test src/vicon_test.cpp)
target_link_libraries(vicon_test PUBLIC
  vicon_driver
)

add_executable(vicon_ros src/vicon_ros.cpp)
target_link_libraries(vicon_ros PUBLIC
  vicon_driver_ros
)

install(TARGETS vicon_driver DESTINATION lib)
install(TARGETS vicon_driver_ros DESTINATION lib)
