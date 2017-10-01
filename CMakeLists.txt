project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

#add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 11)

#set(sources src/main.cpp src/Play_ground.cpp)
set(sources src/main.cpp src/JMT.cpp src/COST.cpp src/OPTPATH.cpp)
#set(sources src/main.cpp src/JMT.cpp)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

FIND_PATH(MathGL_INCLUDE_DIRS NAMES mgl2/mgl.h
        PATHS
        /opt/local/include
        /usr/include
        /usr/local/include
        )

FIND_LIBRARY(MathGL_LIB NAMES mgl
        PATHS
        /opt/local/lib
        /usr/local/lib
        /usr/lib
        )

add_executable(path_planning ${sources})

target_link_libraries(path_planning
        z
        ssl
        uv
        uWS
        ${MathGL_LIB})
