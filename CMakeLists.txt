cmake_minimum_required(VERSION 3.10.2)
project(robot_model_renderer)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cras_cpp_common
  cv_bridge
  image_geometry
  image_transport
  nodelet
  resource_retriever
  roscpp
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  urdf
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

# Prefer newer vendor-specific OpenGL library
if (POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)
endif()
find_package(OpenGL REQUIRED)

find_package(GLEW REQUIRED )
find_package(GLUT REQUIRED )

set(CMAKE_POLICY_DEFAULT_CMP0012 NEW)  # required due to a bug in assimp 5.0
find_package(assimp REQUIRED)
if(NOT DEFINED ASSIMP_LIBRARIES AND TARGET assimp::assimp)
  # ASSIMP > 5.0 no longer defines ASSIMP_LIBRARIES
  set(ASSIMP_LIBRARIES assimp::assimp)
endif()

find_package(OGRE QUIET)
if(NOT OGRE_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(X REQUIRED OGRE)

  # Set OGRE_* variables as cmake-based find_package would do
  set(OGRE_VERSION ${X_OGRE_VERSION})
  set(OGRE_INCLUDE_DIRS ${X_INCLUDE_DIRS})
  message(STATUS "OGRE_INCLUDE_DIRS=${OGRE_INCLUDE_DIRS}")

  # Find absolute path of OGRE libraries.
  if(NOT DEFINED OGRE_LIBRARIES)
    set(_OGRE_LIBRARIES)
    foreach(_lib ${X_LIBRARIES})
      set(_lib_tag "OGRE_LIB_${_lib}")
      find_library(${_lib_tag} NAMES ${_lib} HINTS ${X_LIBRARY_DIRS} PATHS ${X_LIBRARY_DIRS})
      list(APPEND _OGRE_LIBRARIES ${${_lib_tag}})
    endforeach(_lib)

    set(OGRE_LIBRARIES ${_OGRE_LIBRARIES} CACHE STRING "Absolute paths to OGRE libs")
  endif(NOT DEFINED OGRE_LIBRARIES)
  message(STATUS "OGRE_LIBRARIES=${OGRE_LIBRARIES}")

  ## Fetch OGRE_PLUGIN_DIR variable from pkg-config
  if(NOT DEFINED OGRE_PLUGIN_DIR)
    execute_process(
      COMMAND ${PKG_CONFIG_EXECUTABLE} --variable=plugindir OGRE
      OUTPUT_VARIABLE OGRE_PLUGIN_DIR
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
  endif(NOT DEFINED OGRE_PLUGIN_DIR)
  message(STATUS "OGRE_PLUGIN_DIR=${OGRE_PLUGIN_DIR}")
endif()

if(NOT OGRE_VERSION VERSION_LESS "1.10.0" AND NOT MSVC)
  add_compile_options(-Wno-error=deprecated-declarations)
endif()

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
include_directories(SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  ${ASSIMP_INCLUDE_DIRS}
  ${GLEW_INCLUDE_DIR}
  ${GLUT_INCLUDE_DIR}
  ${OGRE_INCLUDE_DIRS}
)

set(ENV_CONFIG_FILE ${CMAKE_CURRENT_BINARY_DIR}/src/env_config.cpp)
configure_file(src/env_config.cpp.in ${ENV_CONFIG_FILE} @ONLY)

add_library(${PROJECT_NAME}_distortion
  src/compositors/OgreDistortionPass.cc
  src/compositors/OgreInvertColors.cc
  src/compositors/OgreOutline.cc
)
target_link_libraries(${PROJECT_NAME}_distortion PUBLIC
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OGRE_LIBRARIES} Eigen3::Eigen
)

add_library(${PROJECT_NAME}_ogre_helpers
  ${ENV_CONFIG_FILE}
  src/ogre_helpers/apply_visibility_bits.cpp
  src/ogre_helpers/camera_base.cpp
  src/ogre_helpers/initialization.cpp
  src/ogre_helpers/object.cpp
  src/ogre_helpers/ogre_logging.cpp
  src/ogre_helpers/orthographic.cpp
  src/ogre_helpers/render_system.cpp
  src/ogre_helpers/shape.cpp
)
target_link_libraries(${PROJECT_NAME}_ogre_helpers PUBLIC
  ${catkin_LIBRARIES} ${ASSIMP_LIBRARIES} ${OPENGL_LIBRARIES} ${OGRE_LIBRARIES}
)

add_library(${PROJECT_NAME}_robot
  src/robot/mesh_loader.cpp
  src/robot/robot.cpp
  src/robot/robot_joint.cpp
  src/robot/robot_link.cpp
  src/robot/tf_link_updater.cpp
)
target_link_libraries(${PROJECT_NAME}_robot PUBLIC
  ${PROJECT_NAME}_ogre_helpers
  ${catkin_LIBRARIES} ${ASSIMP_LIBRARIES} ${OPENGL_LIBRARIES} ${OGRE_LIBRARIES}
)

add_library(${PROJECT_NAME}
  src/pinhole_camera.cpp
  src/RobotModelRenderer.cpp
  src/RosCameraRobotModelRenderer.cpp
  src/utils/ogre_opencv.cpp
  src/utils/sensor_msgs.cpp
  src/utils/sensor_msgs_ogre.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_robot
  ${PROJECT_NAME}_ogre_helpers
  ${PROJECT_NAME}_distortion
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${ASSIMP_LIBRARIES} ${OPENGL_LIBRARIES} ${OGRE_LIBRARIES}
)

add_library(${PROJECT_NAME}_nodelet
  nodelets/${PROJECT_NAME}_nodelet.cpp
)
target_link_libraries(${PROJECT_NAME}_nodelet
  ${PROJECT_NAME} ${catkin_LIBRARIES} Threads::Threads
)
cras_node_from_nodelet(${PROJECT_NAME}_nodelet ${PROJECT_NAME}::RobotModelRendererNodelet OUTPUT_NAME ${PROJECT_NAME} ANONYMOUS)

install(TARGETS
  ${PROJECT_NAME}_ogre_helpers
  ${PROJECT_NAME}_nodelet
  ${PROJECT_NAME}_robot
  ${PROJECT_NAME}_distortion
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  launch/${PROJECT_NAME}.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES
  nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
