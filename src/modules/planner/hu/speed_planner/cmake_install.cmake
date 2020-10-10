# Install script for directory: /home/autolab/tiev2019/src/modules/planner/speed_planner

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/autolab/tiev2019/src/modules/planner/hu/speed_planner/path_time_heuristic/cmake_install.cmake")
  include("/home/autolab/tiev2019/src/modules/planner/hu/speed_planner/speed/cmake_install.cmake")
  include("/home/autolab/tiev2019/src/modules/planner/hu/speed_planner/path_time_graph/cmake_install.cmake")
  include("/home/autolab/tiev2019/src/modules/planner/hu/speed_planner/math/cmake_install.cmake")
  include("/home/autolab/tiev2019/src/modules/planner/hu/speed_planner/common/cmake_install.cmake")
  include("/home/autolab/tiev2019/src/modules/planner/hu/speed_planner/speed_view/cmake_install.cmake")
  include("/home/autolab/tiev2019/src/modules/planner/hu/speed_planner/qp_spline_speed_optimizer/cmake_install.cmake")

endif()

