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

add_executable(autoaim002 src/main.cpp src/vision_processing.cpp src/planning.cpp src/detector.cpp)
target_link_libraries(autoaim002 PRIVATE
    DaedalusSimSdk::DaedalusSimSdk
    ${OpenCV_LIBS}
    Threads::Threads)

# 校内赛自瞄测试工具：按规则 6 工况驱动靶车运动并自动瞄准/开火计数。
add_executable(autoaim002_test src/contest_test.cpp src/vision_processing.cpp src/planning.cpp src/detector.cpp)
target_link_libraries(autoaim002_test PRIVATE
    DaedalusSimSdk::DaedalusSimSdk
    ${OpenCV_LIBS}
    Threads::Threads)

install(TARGETS autoaim002 autoaim002_test RUNTIME DESTINATION bin)
