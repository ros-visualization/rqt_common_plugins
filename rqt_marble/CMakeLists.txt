cmake_minimum_required(VERSION 2.8.3)
project(rqt_marble)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rqt_cpp_common rqt_gui rqt_gui_cpp)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

SET( CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/kde4/apps/cmake/modules/" )
find_package(Marble REQUIRED)

#catkin_python_setup()

set(rqt_marble_HDRS
  include/rqt_marble/marble_plugin.h
  include/rqt_marble/bridge_ros_marble.h
)

set(rqt_marble_SRCS
  src/rqt_marble/marble_plugin.cpp
  src/rqt_marble/bridge_ros_marble.cpp
)

set(rqt_marble_UIS
  resource/marble_plugin.ui
)

set(rqt_marble_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

#set(rqt_marble_EXPORTED_TARGETS ${rqt_marble_HDRS} ${rqt_marble_SRCS}
#    ${rqt_marble_UIS} ${rqt_marble_INCLUDE_DIRECTORIES}
#)

catkin_package(
	INCLUDE_DIRS ${rqt_marble_INCLUDE_DIRECTORIES}
	LIBRARIES ${PROJECT_NAME} 
	CATKIN_DEPENDS rqt_gui rqt_gui_cpp
)

qt4_wrap_cpp(rqt_marble_MOCS ${rqt_marble_HDRS})
qt4_wrap_ui(rqt_marble_UIS_H ${rqt_marble_UIS})

include_directories(${rqt_marble_INCLUDE_DIRECTORIES} 
                    ${catkin_INCLUDE_DIRS} ${MARBLE_INCLUDE_DIR})
add_library(${PROJECT_NAME}
            ${rqt_marble_SRCS} ${rqt_marble_MOCS} ${rqt_marble_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY}
                      ${QT_QTGUI_LIBRARY} ${MARBLE_LIBRARIES})
#add_dependencies(${PROJECT_NAME} INCLUDE_DIRS LIBRARIES CATKIN_DEPENDS)
add_dependencies(${PROJECT_NAME} rqt_cpp_common_gencpp)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

## Setting Unit Test
#catkin_add_gtest(${PROJECT_NAME}-testsuite test/test_bridge_ros_marble.cpp)
#target_link_libraries(${PROJECT_NAME}-testsuite ${PROJECT_NAME_LIBRARIES} ${MARBLE_LIBRARIES})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/rqt_marble
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(PROGRAMS testsuite_rqt_marble
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
