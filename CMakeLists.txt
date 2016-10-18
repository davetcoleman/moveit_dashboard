cmake_minimum_required(VERSION 2.8.12)
project(moveit_dashboard)

# C++ 11
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rviz
  sensor_msgs
  rosparam_shortcuts
)

#link_directories(${catkin_LIBRARY_DIRS})

# Qt Stuff
if(rviz_QT_VERSION VERSION_LESS "5")
  find_package(Qt4 ${rviz_QT_VERSION} REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
  macro(qt_wrap_ui)
    qt4_wrap_ui(${ARGN})
  endmacro()
  macro(qt_wrap_cpp)
    qt4_wrap_cpp(${ARGN})
  endmacro()
else()
  find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
  macro(qt_wrap_ui)
    qt5_wrap_ui(${ARGN})
  endmacro()
  macro(qt_wrap_cpp)
    qt5_wrap_cpp(${ARGN})
  endmacro()
endif()

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    sensor_msgs
    rosparam_shortcuts
  LIBRARIES
    ${PROJECT_NAME}_remote_control
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt_wrap_cpp(MOC_FILES
  src/moveit_panel.h
  src/key_tool.h
)

## specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/moveit_panel.cpp
  src/key_tool.cpp
  #${MOC_FILES}
)

add_library(${PROJECT_NAME} ${SOURCE_FILES} src/remote_control.cpp)
target_link_libraries(${PROJECT_NAME} ${rviz_DEFAULT_PLUGIN_LIBRARIES} ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_remote_control src/remote_control.cpp)
target_link_libraries(${PROJECT_NAME}_remote_control ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_remote_control
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

# Mark config files for installation
install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
