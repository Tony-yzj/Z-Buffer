# 设置 CMake 最低版本和项目名称
cmake_minimum_required(VERSION 3.10)
project(zbuffer)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

# 在 MacOS 上使用 clang 的一些配置
if(APPLE)
    message(STATUS "Compiling on macOS")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra")
endif()

# 添加 include 和 src 目录
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)

# 查找 OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# 查找 src 文件夹下的所有 .cpp 文件
file(GLOB SRC_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)

# 定义可执行文件
add_executable(zbuffer ${SRC_FILES})

# 链接 OpenCV 库
target_link_libraries(zbuffer ${OpenCV_LIBS})
