# 功能包：vision —— 纯图像处理（形态学 / 灯条-装甲板识别 / PnP / 坐标链 / 卡尔曼 / HUD）
#
# 只依赖 OpenCV，不含任何模拟器 SDK 依赖，可独立测试或复用到其他工程。
# 头文件以 <tasks/vision/vision_processing.hpp> 的方式被其他包引用（顶层
# CMakeLists 已 include_directories 项目根目录）。

add_library(vision STATIC
    vision_processing.cpp
)

target_link_libraries(vision PUBLIC ${OpenCV_LIBS})
