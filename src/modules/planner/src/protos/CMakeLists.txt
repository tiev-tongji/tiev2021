cmake_minimum_required(VERSION 3.0.0)
project(protos)

# 输出文件到源文件目录
add_custom_command(
    OUTPUT "${PROJECT_SOURCE_DIR}/routing_service.pb.cc" "${PROJECT_SOURCE_DIR}/routing_service.pb.h" 
    COMMAND protoc --proto_path=${PROJECT_SOURCE_DIR} --cpp_out=${PROJECT_SOURCE_DIR} routing_service.proto
    DEPENDS "routing_service.proto"
)

add_custom_target(
    gen_pb
    DEPENDS ${PROJECT_SOURCE_DIR}/routing_service.pb.cc
)

add_custom_command(
    OUTPUT "${PROJECT_SOURCE_DIR}/routing_service.grpc.pb.cc" "${PROJECT_SOURCE_DIR}/routing_service.grpc.pb.h" 
    COMMAND protoc --proto_path=${PROJECT_SOURCE_DIR} --grpc_out=${PROJECT_SOURCE_DIR}
    --plugin=protoc-gen-grpc=`which grpc_cpp_plugin` routing_service.proto
    DEPENDS "routing_service.proto"
)

add_custom_target(
    gen_pb_grpc
    DEPENDS ${PROJECT_SOURCE_DIR}/routing_service.grpc.pb.cc
)

add_custom_target(gen_all)
add_dependencies(gen_all
    gen_pb gen_pb_grpc
)

# 打包成一个库
add_library(protos
    ${PROJECT_SOURCE_DIR}/routing_service.pb.cc
    ${PROJECT_SOURCE_DIR}/routing_service.grpc.pb.cc
)

add_dependencies(protos gen_all)

target_include_directories(protos
    PUBLIC
    ${PROJECT_SOURCE_DIR}
)