
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

find_package(Protobuf REQUIRED)
if(PROTOBUF_FOUND)
    message(STATUS "protobuf library found")
else()
    message(FATAL_ERROR "protobuf library is needed but cant be found")
endif()


PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ../proto/TJ_v2x.proto)

add_executable(sub test_sub.cpp ${PAHO_CPP_LIB} ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(sub ${PROTOBUF_LIBRARIES} ${MOSQUITTO_LIBRARIES})
target_link_libraries(sub paho-mqtt3a paho-mqttpp3)
add_executable(pub test_pub.cpp ${PAHO_CPP_LIB} ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(pub ${PROTOBUF_LIBRARIES} ${MOSQUITTO_LIBRARIES})
target_link_libraries(pub paho-mqtt3a paho-mqttpp3)