find_package(Protobuf REQUIRED)
find_package(gRPC REQUIRED)
if(GRPC_FOUND)
    message("-------grpc include dir: ${GRPC_INCLUDE_DIRS}")
else()
    message("do not find grpc......")
endif()
# 输出文件到源文件目录
add_custom_command(
    OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/routing_service.pb.cc" "${CMAKE_CURRENT_SOURCE_DIR}/routing_service.pb.h" 
    COMMAND protoc --proto_path=${CMAKE_CURRENT_SOURCE_DIR} --cpp_out=${CMAKE_CURRENT_SOURCE_DIR} routing_service.proto
    DEPENDS "routing_service.proto"
)

add_custom_target(
    gen_pb
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/routing_service.pb.cc
)

add_custom_command(
    OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/routing_service.grpc.pb.cc" "${CMAKE_CURRENT_SOURCE_DIR}/routing_service.grpc.pb.h" 
    COMMAND protoc --proto_path=${CMAKE_CURRENT_SOURCE_DIR} --grpc_out=${CMAKE_CURRENT_SOURCE_DIR}
    --plugin=protoc-gen-grpc=`which grpc_cpp_plugin` routing_service.proto
    DEPENDS "routing_service.proto"
)

add_custom_target(
    gen_pb_grpc
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/routing_service.grpc.pb.cc
)

add_custom_target(gen_all)
add_dependencies(gen_all
    gen_pb gen_pb_grpc
)

# 打包成一个库
add_library(routing_service
    ${CMAKE_CURRENT_SOURCE_DIR}/routing_service.pb.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/routing_service.grpc.pb.cc
)

add_dependencies(routing_service gen_all)

target_include_directories(routing_service
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_include_directories(routing_service PRIVATE ${GRPC_INCLUDE_DIRS})