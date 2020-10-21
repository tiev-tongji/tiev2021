project(CONTROL_CENTER)
set (CMAKE_CXX_STANDARD 11)

include_directories(${PROJECT_SOURCE_DIR}/../../msg/include)

set(SOURCES main.cpp EHBControl.cpp ROEWECenterControl.cpp pidController.cpp messageControl.cpp sys_logger.cpp)
add_executable(CONTROL ${SOURCES})
add_executable(ZCM_TEST zcmUnitTest.cpp)

set(LIBS libzcm.so libpthread.so)
TARGET_LINK_LIBRARIES(CONTROL ${LIBS}) 
TARGET_LINK_LIBRARIES(ZCM_TEST ${LIBS}) 
