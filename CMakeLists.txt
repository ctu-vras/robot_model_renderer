# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.10.2)
project(robot_model_renderer)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  cras_cpp_common
  cv_bridge
  geometry_msgs
  image_geometry
  image_transport
  nodelet
  pluginlib
  resource_retriever
  roscpp
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  urdf
)

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_POLICY_DEFAULT_CMP0012 NEW)  # required due to a bug in assimp 5.0
find_package(assimp REQUIRED)
if(NOT DEFINED ASSIMP_LIBRARIES AND TARGET assimp::assimp)
  # ASSIMP > 5.0 no longer defines ASSIMP_LIBRARIES
  set(ASSIMP_LIBRARIES assimp::assimp)
endif()

find_package(OGRE QUIET)
if(OGRE_FOUND)
  set(${PROJECT_NAME}_OGRE_VERSION ${OGRE_VERSION})
  set(${PROJECT_NAME}_OGRE_INCLUDE_DIRS ${OGRE_INCLUDE_DIRS} CACHE STRING "OGRE include directories")
  set(${PROJECT_NAME}_OGRE_LIBRARIES ${OGRE_LIBRARIES} CACHE STRING "OGRE libraries")
  set(${PROJECT_NAME}_OGRE_PLUGIN_DIR ${OGRE_PLUGIN_DIR} CACHE STRING "OGRE libraries")
else()
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(X REQUIRED OGRE)

  # Set OGRE_* variables as cmake-based find_package would do
  set(${PROJECT_NAME}_OGRE_VERSION ${X_OGRE_VERSION})
  set(${PROJECT_NAME}_OGRE_INCLUDE_DIRS ${X_INCLUDE_DIRS} CACHE STRING "OGRE include directories")
  message(STATUS "${PROJECT_NAME}_OGRE_INCLUDE_DIRS=${${PROJECT_NAME}_OGRE_INCLUDE_DIRS}")

  # Find absolute path of OGRE libraries.
  if(DEFINED OGRE_LIBRARIES)
    set(${PROJECT_NAME}_OGRE_LIBRARIES ${OGRE_LIBRARIES} CACHE STRING "OGRE libraries")
  else()
    set(_OGRE_LIBRARIES)
    foreach(_lib ${X_LIBRARIES})
      set(_lib_tag "OGRE_LIB_${_lib}")
      find_library(${_lib_tag} NAMES ${_lib} HINTS ${X_LIBRARY_DIRS} PATHS ${X_LIBRARY_DIRS})
      list(APPEND _OGRE_LIBRARIES ${${_lib_tag}})
    endforeach()
    set(${PROJECT_NAME}_OGRE_LIBRARIES ${_OGRE_LIBRARIES} CACHE STRING "OGRE libraries")
  endif()
  message(STATUS "${PROJECT_NAME}_OGRE_LIBRARIES=${${PROJECT_NAME}_OGRE_LIBRARIES}")

  ## Fetch OGRE_PLUGIN_DIR variable from pkg-config
  if(DEFINED OGRE_PLUGIN_DIR)
    set(${PROJECT_NAME}_OGRE_PLUGIN_DIR ${OGRE_PLUGIN_DIR} CACHE STRING "OGRE libraries")
  else()
    execute_process(
      COMMAND ${PKG_CONFIG_EXECUTABLE} --variable=plugindir OGRE
      OUTPUT_VARIABLE ${PROJECT_NAME}_OGRE_PLUGIN_DIR
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
  endif()
  message(STATUS "${PROJECT_NAME}_OGRE_PLUGIN_DIR=${${PROJECT_NAME}_OGRE_PLUGIN_DIR}")
endif()

catkin_package()

# Create modern CMake targets for dependencies that do not provide them

add_library(OPENCV INTERFACE)
#catkin_lint: ignore_once external_interface_path
target_include_directories(OPENCV SYSTEM INTERFACE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(OPENCV INTERFACE ${OpenCV_LIBRARIES})

add_library(ASSIMP INTERFACE)
#catkin_lint: ignore_once external_interface_path
target_include_directories(ASSIMP SYSTEM INTERFACE ${ASSIMP_INCLUDE_DIRS})
target_link_libraries(ASSIMP INTERFACE ${ASSIMP_LIBRARIES})

add_library(OGRE INTERFACE)
#catkin_lint: ignore_once external_interface_path
target_include_directories(OGRE SYSTEM INTERFACE ${${PROJECT_NAME}_OGRE_INCLUDE_DIRS})
target_link_libraries(OGRE INTERFACE ${${PROJECT_NAME}_OGRE_LIBRARIES})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(ENV_CONFIG_FILE ${CMAKE_CURRENT_BINARY_DIR}/src/env_config.cpp)
configure_file(src/env_config.cpp.in ${ENV_CONFIG_FILE} @ONLY)

add_library(${PROJECT_NAME}_distortion
  src/compositors/OgreCameraDistortion.cc
  src/compositors/OgreInvertColors.cc
  src/compositors/OgreOutline.cc
)
target_link_libraries(${PROJECT_NAME}_distortion PUBLIC
  ${catkin_LIBRARIES} OPENCV OGRE Eigen3::Eigen
)

add_library(${PROJECT_NAME}_ogre_helpers
  ${ENV_CONFIG_FILE}
  src/ogre_helpers/object.cpp
  src/ogre_helpers/ogre_logging.cpp
  src/ogre_helpers/orthographic.cpp
  src/ogre_helpers/render_system.cpp
  src/ogre_helpers/shape.cpp
)
target_link_libraries(${PROJECT_NAME}_ogre_helpers PUBLIC
  ${catkin_LIBRARIES} OGRE
)

add_library(${PROJECT_NAME}_robot
  src/robot/mesh_loader.cpp
  src/robot/mesh_optimizer.cpp
  src/robot/robot.cpp
  src/robot/robot_joint.cpp
  src/robot/robot_link.cpp
  src/robot/shape_filter.cpp
  src/robot/shape_inflation_registry.cpp
  src/robot/tf_link_updater.cpp
)
target_link_libraries(${PROJECT_NAME}_robot
  PUBLIC ${PROJECT_NAME}_ogre_helpers ${catkin_LIBRARIES} OGRE
  PRIVATE ASSIMP
)

add_library(${PROJECT_NAME}
  src/RobotModelRenderer.cpp
  src/RosCameraRobotModelRenderer.cpp
  src/pinhole_camera.cpp
  src/utils/ogre_opencv.cpp
  src/utils/sensor_msgs.cpp
  src/utils/sensor_msgs_ogre.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_robot
  ${PROJECT_NAME}_ogre_helpers
  ${PROJECT_NAME}_distortion
  ${catkin_LIBRARIES} OPENCV OGRE
)

add_library(${PROJECT_NAME}_c_api
  src/c_api.cpp
)
target_link_libraries(${PROJECT_NAME}_c_api
  ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}_nodelet
  nodelets/${PROJECT_NAME}_nodelet.cpp
)
target_link_libraries(${PROJECT_NAME}_nodelet
  ${PROJECT_NAME} ${catkin_LIBRARIES} Threads::Threads
)
cras_node_from_nodelet(${PROJECT_NAME}_nodelet ${PROJECT_NAME}::RobotModelRendererNodelet OUTPUT_NAME ${PROJECT_NAME} ANONYMOUS)

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_c_api
  ${PROJECT_NAME}_distortion
  ${PROJECT_NAME}_nodelet
  ${PROJECT_NAME}_ogre_helpers
  ${PROJECT_NAME}_robot
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY ogre_media/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ogre_media)
install(DIRECTORY rviz/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)

install(FILES nodelet.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  find_package(rostest REQUIRED)
  find_package(roslaunch REQUIRED)

  roslint_custom(catkin_lint "-W2" .)

  # Roslint C++ - checks formatting and some other rules for C++ files
  file(GLOB_RECURSE ROSLINT_FILES include/*.h include/*.hpp src/*.cpp nodes/*.cpp nodelets/*.cpp test/*.cpp)
  set(ROSLINT_CPP_OPTS "--extensions=h,hpp,cpp;--linelength=120;--filter=\
    -build/header_guard,-build/include,-readability/namespace,-whitespace/braces,-runtime/references,\
    -build/c++11,-readability/nolint,-readability/todo,-legal/copyright")
  roslint_cpp(${ROSLINT_FILES})

  roslint_add_test()

  roslaunch_add_file_check(launch/${PROJECT_NAME}.launch IGNORE_UNSET_ARGS)
endif()
