cmake_minimum_required(VERSION 3.10)
project(CONTROL_CENTER)
set (CMAKE_CXX_STANDARD 11)

include_directories(${PROJECT_SOURCE_DIR}/../../msg/include)
include_directories(${PROJECT_SOURCE_DIR}/../../common)

set(SOURCES EHBControl.cpp ROEWECenterControl.cpp pidController.cpp messageControl.cpp SerialPort.cpp ESRControl.cpp main.cpp  ${PROJECT_SOURCE_DIR}/../../common/nature/timestamp.cpp)
add_executable(CONTROL ${SOURCES})
add_executable(ZCM_TEST zcmUnitTest.cpp)

set(LIBS libusbcan.so libzcm.so libpthread.so)
TARGET_LINK_LIBRARIES(CONTROL ${LIBS}) 
TARGET_LINK_LIBRARIES(ZCM_TEST ${LIBS}) 
