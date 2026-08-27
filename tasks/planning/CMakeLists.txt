# 功能包：planning —— 射击规划（弹道解算 + 提前量预测）
#
# 无空气阻力的弹道求解（solveBallistic）与"预测时刻 = 开火延迟 + 飞行时间"的
# 迭代瞄准点规划（planAimPoint）。只依赖 OpenCV 核心数据类型。

add_library(planning STATIC
    planning.cpp
)

target_link_libraries(planning PUBLIC ${OpenCV_LIBS})
