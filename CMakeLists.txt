cmake_minimum_required(VERSION 3.16)
project(autoaim002 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Path to the installed Daedalus contest SDK package. The directory must
# contain  include/daedalus_sim_sdk/  and  lib/cmake/DaedalusSimSdk/ .
# Override for a different install, e.g.:
#   -DDAEDALUS_SDK_ROOT=/path/to/linux-x86_64/sdk
set(DAEDALUS_SDK_ROOT "/home/xqy/下载/linux-x86_64(1)/sdk" CACHE PATH
    "Root of the installed Daedalus Sim SDK (1.3.1-contest-r4 with getLatestArmorHit)")

if(NOT EXISTS "${DAEDALUS_SDK_ROOT}/lib/cmake/DaedalusSimSdk/DaedalusSimSdkConfig.cmake")
  message(FATAL_ERROR
      "DaedalusSimSdk was not found below DAEDALUS_SDK_ROOT=${DAEDALUS_SDK_ROOT}.\n"
      "Point DAEDALUS_SDK_ROOT at the 'sdk' directory of the contest release.")
endif()
set(DaedalusSimSdk_DIR "${DAEDALUS_SDK_ROOT}/lib/cmake/DaedalusSimSdk")

find_package(DaedalusSimSdk 1.3.1 REQUIRED CONFIG)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

# 项目根目录加入 include 路径，使各功能包头文件可以按
# "tasks/<package>/<file>.hpp" 的方式互相引用（与 tongji 工程一致）。
include_directories(${PROJECT_SOURCE_DIR})

#################### 功能包（每个 add_subdirectory 对应一个静态库目标） ####################
add_subdirectory(tasks/vision)      # 纯视觉：形态学 / 检测 / PnP / 坐标链 / 卡尔曼 / HUD
add_subdirectory(tasks/detection)   # 检测器抽象：传统视觉 / 神经网络（--detector 切换）
add_subdirectory(tasks/planning)    # 规划器：弹道解算 + 提前量预测

#################### 可执行程序（src/） ####################
# 主程序：SDK 读帧 + 调用视觉模块 + 显示/打印。
add_executable(autoaim002 src/main.cpp)
target_link_libraries(autoaim002 PRIVATE
    vision
    detection
    planning
    DaedalusSimSdk::DaedalusSimSdk
    Threads::Threads)

# 校内赛自瞄测试工具：按规则 6 工况驱动靶车运动并自动瞄准/开火计数。
add_executable(autoaim002_test src/contest_test.cpp)
target_link_libraries(autoaim002_test PRIVATE
    vision
    detection
    planning
    DaedalusSimSdk::DaedalusSimSdk
    Threads::Threads)

# 识别 + PnP 曲线测试工具：仅检测/PnP，输出 CSV 供画图。
add_executable(pnp_curve_test src/pnp_curve_test.cpp)
target_link_libraries(pnp_curve_test PRIVATE
    vision
    detection
    planning
    DaedalusSimSdk::DaedalusSimSdk
    Threads::Threads)

#################### 单元测试（tests/，无需模拟器） ####################
add_executable(planning_test tests/planning_test.cpp)
target_link_libraries(planning_test PRIVATE planning)

add_executable(vision_test tests/vision_test.cpp)
target_link_libraries(vision_test PRIVATE vision)

#################### 安装 ####################
install(TARGETS autoaim002 autoaim002_test RUNTIME DESTINATION bin)
