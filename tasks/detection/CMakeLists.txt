# 功能包：detection —— 装甲板检测器抽象（传统形态学 / 神经网络）
#
# 定义 detect::IDetector 接口与工厂 makeDetector()。
# - VisionDetector：形态学 + 灯条配对（基于 vision 包）。
# - NNDetector：通过 Python 子进程（tools/ov_armor_service.py）跑 OpenVINO YOLO。
# 依赖 vision 包。

add_library(detection STATIC
    detector.cpp
)

target_link_libraries(detection PUBLIC vision)
