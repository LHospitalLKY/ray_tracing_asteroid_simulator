cmake_minimum_required(VERSION 3.12)
project(G2O_Asteroid)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
# 设置编译选项为debug
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS_DEBUG} -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS_RELEASE} -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} -O0")
# 设置生成compile_command
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# tiny-obj
find_package(tinyobjloader REQUIRED)

# eigen
find_package(Eigen3 REQUIRED)
include_directories(
    /usr/include/eigen3
)
# opencv
# find_package(OpenCV REQUIRED)

# g2o库
find_package(g2o REQUIRED)
link_directories(/usr/local/lib)
# ceres库
find_package(Ceres REQUIRED)
# glog
find_package(Glog REQUIRED)

# Kissfft
include_directories(
    ./3dpart
    ./3dpart/kissfft
    ./3dpart/kissfft/tools
)
# CCFits
include_directories(
    ./3dpart/CCFtis/include
    ./3dpart/CCFtis/include/CCfits
    ./3dpart/CCFtis/include/cfitsio
)
link_directories(
    ./3dpart/CCFtis/lib/CCfits
    ./3dpart/CCFtis/lib/cfitsio
)

# 添加源文件
set(SOURCES
    # include/csv.h
    # include/gnuplot.h
    src/hello_g2o.cpp
)

# 添加库文件
# fft库
add_library(kiss_fft
    ./3dpart/Kissfft/tools/kiss_fftndr.h
    ./3dpart/Kissfft/tools/kiss_fftndr.h
    ./3dpart/Kissfft/tools/kiss_fftr.h
    ./3dpart/Kissfft/tools/kiss_fftnd.h
    ./3dpart/Kissfft/kiss_fft.h
    ./3dpart/Kissfft/tools/kiss_fftndr.c
    ./3dpart/Kissfft/tools/kiss_fftndr.c
    ./3dpart/Kissfft/tools/kiss_fftr.c
    ./3dpart/Kissfft/tools/kiss_fftnd.c
    ./3dpart/Kissfft/kiss_fft.c
)
# 亮度计算库
add_library(bright_calcu
    include/common.h
    include/bright_calcu.h
    src/common.cpp
    src/bright_calcu.cpp
)
# AO图像处理库
# add_library(AO_calcu
#     include/common.h
#     include/AO_calcu.h
#     src/common.cpp
#     src/AO_calcu.cpp
# )


# ========测试程序========
# hello world
add_executable(hello_demo test/hello_g2o.cpp)
target_link_libraries(hello_demo
    # libg2o_stuff.so
    g2o::core
    g2o::solver_dense
)
# bright_calcu功能测试
add_executable(bright_calcu_test test/bright_calcu_test.cpp)
target_link_libraries(bright_calcu_test
    bright_calcu
)
# damit2obj功能测试
add_executable(damit_obj_converter_test
    include/common.h
    include/obj_loader.h
    include/damit_obj_converter.h
    src/common.cpp
    # src/obj_loader.cpp
    src/damit_obj_converter.cpp
    test/damit_obj_converter_test.cpp
)
# 可见性测试
add_executable(visibility_test
    # include/common.h
    # include/bright_calcu.h
    include/obj_loader.h
    src/common.cpp
    test/visibility_test.cpp
)
target_link_libraries(visibility_test
    bright_calcu
)

# 基于g2o的周期计算
add_executable(period_scan
    test/period_scan.cpp
)
target_link_libraries(period_scan
    bright_calcu
    g2o::core
    g2o::solver_dense
)

# 基于ceres的周期计算
add_executable(ceres_test
    test/ceres_test.cpp
)
target_link_libraries(ceres_test
    bright_calcu
    ${GLOG_LIBRARIES}
    Ceres::ceres
)

# AO图像处理计算
add_executable(AO_calcu
    include/common.h
    include/AO_calcu.h
    src/common.cpp
    src/AO_calcu.cpp
)
target_link_libraries(AO_calcu
    # bright_calcu
    # ${GLOG_LIBRARIES}
    # Ceres::ceres
    # ./3dpart/CCFtis/lib/CCfits/libCCfits.so
    CCfits
    cfitsio
    kiss_fft
)

# add_executable(jet_test
#     include/bright_calcu_jet.h
#     test/test_jet.cpp
# )
# target_link_libraries(jet_test
#     ${GLOG_LIBRARIES}
#     Ceres::ceres
# )

# ========可执行程序========