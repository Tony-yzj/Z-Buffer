# 设置 CMake 最低版本和项目名称
cmake_minimum_required(VERSION 3.10)
project(zbuffer)
project(hierachyZbuffer)
project(hierachyZbufferBVH)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

# 在 MacOS 上使用 clang 的一些配置
if(APPLE)
    message(STATUS "Compiling on macOS")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra")
endif()

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(zbuffer PUBLIC OpenMP::OpenMP_CXX)
    target_link_libraries(hierachyZbuffer PUBLIC OpenMP::OpenMP_CXX)
endif()

# 添加 include 和 src 目录
include_directories(${PROJECT_SOURCE_DIR}/src/include)
include_directories(${PROJECT_SOURCE_DIR}/src/thirdparty)
include_directories(${PROJECT_SOURCE_DIR}/src)

# 查找 OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# 查找 src 文件夹下的所有 .cpp 文件
file(GLOB SRC_FILES ${PROJECT_SOURCE_DIR}/src/scanlineZbuffer.cpp ${PROJECT_SOURCE_DIR}/src/model.cpp ${PROJECT_SOURCE_DIR}/src/scanline.cpp ${PROJECT_SOURCE_DIR}/src/hierachy.cpp  ${PROJECT_SOURCE_DIR}/src/include/*.h)
file(GLOB H_FILES ${PROJECT_SOURCE_DIR}/src/hierachyZbuffer.cpp  ${PROJECT_SOURCE_DIR}/src/model.cpp ${PROJECT_SOURCE_DIR}/src/scanlineHZB.cpp ${PROJECT_SOURCE_DIR}/src/BVH.cpp ${PROJECT_SOURCE_DIR}/src/hierachy.cpp ${PROJECT_SOURCE_DIR}/src/include/*.h)
file(GLOB BVH_FILES ${PROJECT_SOURCE_DIR}/src/hierachyZbufferBVH.cpp  ${PROJECT_SOURCE_DIR}/src/model.cpp ${PROJECT_SOURCE_DIR}/src/scanlineHZB.cpp ${PROJECT_SOURCE_DIR}/src/hierachy.cpp ${PROJECT_SOURCE_DIR}/src/BVH.cpp ${PROJECT_SOURCE_DIR}/src/include/*.h)

# 定义可执行文件
add_executable(zbuffer ${SRC_FILES})
add_executable(hierachyZbuffer ${H_FILES})
add_executable(hierachyZbufferBVH ${BVH_FILES})

# 链接 OpenCV 库
target_link_libraries(zbuffer ${OpenCV_LIBS})
target_link_libraries(hierachyZbuffer ${OpenCV_LIBS})
target_link_libraries(hierachyZbufferBVH ${OpenCV_LIBS})
