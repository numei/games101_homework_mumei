cmake_minimum_required(VERSION 3.5)
project(Assignment2)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. 查找 Eigen (这个目前看来没问题了)
find_package(Eigen3 REQUIRED)

# 2. 查找 OpenCV
find_package(OpenCV REQUIRED)

# 3. 添加头文件路径
include_directories(${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(main main.cpp rasterizer.hpp rasterizer.cpp Triangle.hpp Triangle.cpp)

# 4. 【关键修改】不再依赖 ${OpenCV_LIBS} 变量，手动链接 Homebrew 下的核心库
# 这能强制程序使用你刚才 reinstall 的那个 OpenCV
target_link_libraries(main 
    "/opt/homebrew/opt/opencv/lib/libopencv_core.dylib"
    "/opt/homebrew/opt/opencv/lib/libopencv_highgui.dylib"
    "/opt/homebrew/opt/opencv/lib/libopencv_imgcodecs.dylib"
    "/opt/homebrew/opt/opencv/lib/libopencv_imgproc.dylib"
    Eigen3::Eigen
)