cmake_minimum_required(VERSION 2.8.3)
project(rqt_cpp_common)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS message_generation sensor_msgs)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

add_message_files(DIRECTORY msg FILES RouteGps.msg)
generate_messages(DEPENDENCIES sensor_msgs)

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
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME} 
	CATKIN_DEPENDS message_runtime sensor_msgs
)

qt4_wrap_cpp(rqt_cpp_common_MOCS ${rqt_cpp_common_HDRS})
qt4_wrap_ui(rqt_cpp_common_UIS_H ${rqt_cpp_common_UIS})

include_directories(${rqt_cpp_common_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS} ${MARBLE_INCLUDE_DIR})
add_library(${PROJECT_NAME} ${rqt_cpp_common_SRCS} ${rqt_cpp_common_MOCS} ${rqt_cpp_common_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

find_package(class_loader)
#class_loader_hide_library_symbols(${PROJECT_NAME})

# Setting Unit Test
catkin_add_gtest(${PROJECT_NAME}-testsuite test/test_widgets_list.cpp)
target_link_libraries(${PROJECT_NAME}-testsuite ${PROJECT_NAME})

#install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#install(PROGRAMS testsuite_rqt_cpp_common
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
