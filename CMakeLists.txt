cmake_minimum_required(VERSION 2.8.3)
project(rs_log_learning)
find_package(catkin REQUIRED COMPONENTS robosherlock roscpp cv_bridge
  std_msgs message_generation
)
################################################################################
## Constants for project                                                      ##
################################################################################
set(NAMESPACE rs_log_learning)
set(TYPESYSTEM_CPP_PATH ${PROJECT_SOURCE_DIR}/include/rs_log_learning/types)
set(TYPESYSTEM_XML_PATH ${PROJECT_SOURCE_DIR}/descriptors/typesystem)
set(ANNOTATOR_PATH      ${PROJECT_SOURCE_DIR}/descriptors/annotators)
set(ENGINE_PATH         ${PROJECT_SOURCE_DIR}/descriptors/analysis_engines)
set(CMAKE_MODULE_PATH   ${PROJECT_SOURCE_DIR}/cmake)
################################################################################
## Configure project configuration files                                      ##
################################################################################
configure_file(${RS_PROJECT_CONFIG} ${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}_config.cmake @ONLY)
################################################################################
## Find all include directories                                               ##
################################################################################
find_include_dirs(RS_INCLUDE_DIRS_LIST)

# include mlpack
find_package(mlpack REQUIRED)
include_directories(${mlpack_INCLUDE_DIRS})
include_directories(${SDK_DIR}/usr/include/libxml2) # mlpack references just libxml
set(LIBS ${LIBS} ${mlpack_LIBRARIES})

# find all include dirs
find_include_dirs(RS_INCLUDE_DIRS_LIST)

# gtk includes and libs
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTKMM gtkmm-2.4)

include_directories(${GTKMM_INCLUDE_DIRS})
link_directories(${GTKMM_LIBRARY_DIRS})

MESSAGE(STATUS "GTKMM include debug:  " ${GTKMM_INCLUDE_DIRS})

################################################################################
## Update analysis engines, typesystem and include all relevant files         ##
################################################################################
##Update xml list of annotators inside analysis engines
update_analysis_engines(robosherlock)
## generate classes from the typesystem xml files
generate_type_system(robosherlock)
#find all relevant files
find_additional_files()
################################################################################



include_directories(
  include
  ${RS_INCLUDE_DIRS_LIST}
  ${catkin_INCLUDE_DIRS}
)

add_service_files(
  FILES
  ImageGTAnnotation.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs 
   sensor_msgs
)

catkin_package(
  INCLUDE_DIRS ${RS_INCLUDE_DIRS_LIST} include
  CATKIN_DEPENDS message_runtime
)

rs_add_library(rs_mindPalaceAnnotator 
		src/MPCore.cpp 
		src/MindPalaceAnnotator.cpp 
		src/LearnAnnotationStorage.cpp
		src/MPIdentifiable.cpp 
		src/containers/Geometry.cpp
		src/containers/GroundTruth.cpp
		src/containers/LearningAnnotation.cpp
		src/containers/SemanticColor.cpp
		src/algorithms/mpAlgorithm.cpp
		src/algorithms/NearestNeighborAlgorithm.cpp
		src/algorithms/DecisionTreeAlgorithm.cpp)
target_link_libraries(rs_mindPalaceAnnotator rs_core mlpack armadillo ${catkin_LIBRARIES})

rs_add_library(rs_mpGTAnnotator src/mpGTAnnotator.cpp)
target_link_libraries(rs_mpGTAnnotator rs_core ${GTKMM_LIBRARIES} ${catkin_LIBRARIES})

add_executable(mpGTui src/mpGTui.cpp src/roiDrawingArea.cpp)
add_dependencies(mpGTui rs_log_learning_generate_messages_cpp sensor_msgs)
target_link_libraries(mpGTui rs_core ${GTKMM_LIBRARIES} ${catkin_LIBRARIES})
