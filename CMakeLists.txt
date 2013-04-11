# Project Description
cmake_minimum_required(VERSION 2.6)
project(hubo-manipulation CXX C)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Include hubo-motion stuff
link_libraries(ach)
link_libraries(huboctrl daemonizer rt huboparams)

# Include the project files
include_directories("include")

file(GLOB scripts_source "src/*.cpp")
LIST(SORT scripts_source)

add_executable(hubo-manipulation ${scripts_source})
