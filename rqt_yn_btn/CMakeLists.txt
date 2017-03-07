cmake_minimum_required(VERSION 2.8.3)
project(rqt_yn_btn)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS message_generation)
catkin_package()
catkin_python_setup()

add_service_files(FILES YesNo.srv)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/rqt_yn_btn
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
