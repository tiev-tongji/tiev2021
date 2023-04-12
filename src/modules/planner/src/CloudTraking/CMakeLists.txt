find_package(Protobuf REQUIRED)
if(PROTOBUF_FOUND)
    message(STATUS "protobuf library found")
else()
    message(FATAL_ERROR "protobuf library is needed but cant be found")
endif()

#include_directories(${PROTOBUF_INCLUDE_DIRS})
#include_directories(${SRC_ROOT_PATH})
#include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${MOSQUITTO_INCLUDE_DIR})
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS proto/TJ_v2x.proto)


#add_executable(sub src/test_sub.cpp ${PAHO_CPP_LIB} ${PROTO_SRCS} ${PROTO_HDRS})
#add_executable(CloudSystem src/CloudSystem.cpp ${PAHO_CPP_LIB} ${PROTO_SRCS} ${PROTO_HDRS})

add_library(CloudSystem STATIC "")
target_sources(CloudSystem PRIVATE
        src/CloudSystem.cpp
        ${PROTO_SRCS}

        )

target_include_directories(CloudSystem PUBLIC
        ./src
        ./proto
        ../planner_context
        )
#target_link_libraries(sub ${PROTOBUF_LIBRARIES} ${MOSQUITTO_LIBRARIES})
#target_link_libraries(sub paho-mqtt3a paho-mqttpp3)
target_link_libraries(CloudSystem ${PROTOBUF_LIBRARIES} ${MOSQUITTO_LIBRARIES})
target_link_libraries(CloudSystem paho-mqtt3a paho-mqttpp3)
target_link_libraries(CloudSystem  planner_context )
target_link_libraries(CloudSystem  message_manager)




#[[

cmake_minimum_required(VERSION 3.5)

## project name
project("mqtt-test")

## --- C++11 build flags ---

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Protobuf REQUIRED)
if(PROTOBUF_FOUND)
    message(STATUS "protobuf library found")
else()
    message(FATAL_ERROR "protobuf library is needed but cant be found")
endif()

include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${SRC_ROOT_PATH})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${MOSQUITTO_INCLUDE_DIR})

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS proto/TJ_v2x.proto)

add_executable(sub src/test_sub.cpp ${PAHO_CPP_LIB} ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(sub ${PROTOBUF_LIBRARIES} ${MOSQUITTO_LIBRARIES})
target_link_libraries(sub paho-mqtt3a paho-mqttpp3)
]]