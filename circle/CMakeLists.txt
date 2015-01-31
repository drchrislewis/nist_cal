cmake_minimum_required(VERSION 2.8.3)
project(circle)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs cv_bridge tf image_transport roslint industrial_extrinsic_cal)

set ( CMAKE_SHARED_LINKER_FLAGS "-Wl")

# Ceres
FIND_PACKAGE(Ceres REQUIRED)
IF (CERES_FOUND)
  MESSAGE("-- Found Ceres version ${CERES_VERSION}: ${CERES_INCLUDE_DIRS}")
ENDIF (CERES_FOUND)

# Eigen
FIND_PACKAGE(Eigen REQUIRED)
IF (EIGEN_FOUND)
  MESSAGE("-- Found Eigen version ${EIGEN_VERSION}: ${EIGEN_INCLUDE_DIRS}")
ENDIF (EIGEN_FOUND)

# Opencv
FIND_PACKAGE(OpenCV REQUIRED)
IF (OPENCV_FOUND)
  MESSAGE("-- Found Opencv version ${OPENCV_VERSION}: ${OPENCV_INCLUDE_DIRS}")
ENDIF (OPENCV_FOUND)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system python)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   linear_mv.srv
   n_images.srv
#   Service2.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs circle # Or other packages containing msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include 
   LIBRARIES industrial_extrinsic_cal
   CATKIN_DEPENDS roscpp std_msgs rosconsole
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} /usr/include/python2.7 /usr/include
)

## Declare a cpp library
add_library(circle_detector
  SHARED
  src/circledetector.cpp
  src/ceres_cost_utils.cpp
  )

## Something about the USB4 library
add_library(libUsb4 SHARED IMPORTED)
set_property(TARGET libUsb4 PROPERTY IMPORTED_LOCATION /usr/lib/libusdusb4.so)

## Something about the aravis library
add_library(libaravis-0.2 SHARED IMPORTED)
set_property(TARGET libaravis-0.2 PROPERTY IMPORTED_LOCATION /usr/local/lib/libaravis-0.2.so)


## Declare a cpp executable
# add_executable(industrial_extrinsic_cal_node src/industrial_extrinsic_cal_node.cpp)
add_executable(ellipse_test src/ellipse_test.cpp)
add_executable(boeing_eval src/ba_ex_cal.cpp )
add_executable(circletest src/circletest.cpp)
add_executable(bullseye_anal src/bullseye_analysis.cpp)
add_executable(bullseye_anal2 src/bullseye_analysis2.cpp)
add_executable(bullseye_anal3 src/bullseye_analysis3.cpp)
add_executable(bullseye_anal4 src/bullseye_analysis4.cpp)
add_executable(bullseye_anal5 src/bullseye_analysis5.cpp)
add_executable(bullseye_anal6 src/bullseye_analysis6.cpp)
add_executable(slant_anal src/slant_analysis.cpp)
add_executable(linearRail src/linear_rail.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(industrial_extrinsic_cal_node industrial_extrinsic_cal_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(ellipse_test ${CERES_LIBRARIES} ${OpenCV_LIBS} circle_detector)
target_link_libraries(boeing_eval ${CERES_LIBRARIES}  ${catkin_LIBRARIES} yaml-cpp industrial_extrinsic_cal)
#target_link_libraries(circle_detector python2.7 boost_python ${OpenCV_LIBS} )
#target_link_libraries(circletest ${CERES_LIBRARIES}  ${catkin_LIBRARIES} yaml-cpp ${OpenCV_LIBS} python2.7 boost_python circle_detector)
target_link_libraries(circle_detector ${OpenCV_LIBS} )
target_link_libraries(circletest ${CERES_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp ${OpenCV_LIBS} circle_detector)
target_link_libraries(bullseye_anal ${CERES_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp ${OpenCV_LIBS} circle_detector)
target_link_libraries(bullseye_anal2 ${CERES_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp ${OpenCV_LIBS} circle_detector)
target_link_libraries(bullseye_anal3 ${CERES_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp ${OpenCV_LIBS} circle_detector)
target_link_libraries(bullseye_anal4 ${CERES_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp ${OpenCV_LIBS} circle_detector)
target_link_libraries(bullseye_anal5 ${CERES_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp ${OpenCV_LIBS} circle_detector)
target_link_libraries(bullseye_anal6 ${CERES_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp ${OpenCV_LIBS} circle_detector)
target_link_libraries(slant_anal ${CERES_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp ${OpenCV_LIBS} circle_detector)
target_link_libraries(linearRail ${CERES_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp ${OpenCV_LIBS} libUsb4 libaravis-0.2)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS industrial_extrinsic_cal industrial_extrinsic_cal_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_industrial_extrinsic_cal.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

## ROSlint checks
## For more information see: http://wiki.ros.org/roslint
##
## CPPlint options justifications
## - whitespace/* - errors are ignored, it is assumed that the follwing eclipse
##   format file has been applied: 
##   http://wiki.ros.org/IDEs?action=AttachFile&do=get&target=ROS_format.xml
## - build/header_guard - #ifdef/#ifndef naming doesn't match ROS sytle
## - readability/stream - does not apply to logging (the typical use in ROS)
## - runtime/references - pass by reference is allowed for output parameters
set(ROSLINT_CPP_OPTS 
"--filter=-whitespace,-build/header_guard,-build/include_order,-readability/streams,-runtime/references")
file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS 
     RELATIVE ${PROJECT_SOURCE_DIR} src/*.cpp include/*.h)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
