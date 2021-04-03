add_subdirectory(protos)

set(SRC Routing.cpp)
add_library(routing ${SRC})
#target_link_libraries(routing pqxx -L/usr/lib/x86_64-linux-gnu)
target_link_libraries(routing pqxx pq protos)

# 增加可执行文件
# add_executable(${PROJECT_NAME} main.cpp)
# target_link_libraries(${PROJECT_NAME} librouting.so)
