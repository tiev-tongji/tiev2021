add_library(message_manager SHARED "")

target_sources(message_manager PRIVATE 
    message_manager.cpp 
)

target_include_directories(message_manager PRIVATE 
    ${CMAKE_SOURCE_DIR}/include/message_manager
)