cmake_minimum_required(VERSION 2.8.3)
project(rqt_cpp_common)
# Load catkin and all dependencies required for this package
#find_package(catkin REQUIRED COMPONENTS message_generation roscpp)
find_package(catkin REQUIRED)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

set(rqt_cpp_common_SRCS
  src/rqt_cpp_common/widgets_list.cpp
)

set(rqt_cpp_common_HDRS
  include/rqt_cpp_common/widgets_list.h
)

set(rqt_cpp_common_UIS
  resource/widgets_list.ui
)

set(rqt_cpp_common_INCLUDE_DIRECTORIES
  include
  include/rqt_cpp_common
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
	INCLUDE_DIRS ${rqt_cpp_common_INCLUDE_DIRECTORIES}
	LIBRARIES ${PROJECT_NAME} 
	#CATKIN_DEPENDS roscpp
)

qt4_wrap_cpp(rqt_cpp_common_MOCS ${rqt_cpp_common_HDRS})
qt4_wrap_ui(rqt_cpp_common_UIS_H ${rqt_cpp_common_UIS})

include_directories(${rqt_cpp_common_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS} ${MARBLE_INCLUDE_DIR})
add_library(${PROJECT_NAME} ${rqt_cpp_common_SRCS} ${rqt_cpp_common_MOCS} ${rqt_cpp_common_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

find_package(class_loader)
#class_loader_hide_library_symbols(${PROJECT_NAME})

## Setting Unit Test
##catkin_add_gtest(testsuite_rqt_cpp_common test/test_bridge_ros_marble.cpp)
##target_link_libraries(testsuite_rqt_cpp_common ${PROJECT_NAME_LIBRARY})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/rqt_cpp_common
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(PROGRAMS testsuite_rqt_cpp_common
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
