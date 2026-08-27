# 架构文档（ARCHITECTURE）

本文档说明 `autoaim002` 的**功能包（package）结构**：代码如何按功能拆分、各包之间的
依赖关系、构建系统如何组织，以及如何扩展/新增一个功能包。目录结构参考了
`tongji` 工程（`tasks/` 功能包 + `src/` 可执行程序 + `tests/` 测试 + `tools/` 工具）。

## 1. 目录总览

```
autoaim002/
├── CMakeLists.txt              # 顶层构建：找依赖 → 添加功能包 → 生成可执行程序与测试
├── README.md                   # 主文档（功能、参数、算法说明）
├── QUICKSTART.md               # 5 分钟快速开始
├── docs/
│   ├── ARCHITECTURE.md         # 本文档：功能包结构与扩展指南
│   └── CODE_WALKTHROUGH.md     # 代码逐段讲解
├── configs/                    # 运行时配置（相机标定等）
├── src/                        # 可执行程序入口（main），只做组装与命令行交互
│   ├── main.cpp                # 主程序：SDK 读帧 + 调用功能包 + 显示/打印
│   ├── contest_test.cpp        # 校内赛自瞄测试工具（6 工况）
│   └── pnp_curve_test.cpp      # 识别 + PnP 曲线测试（输出 CSV）
├── tasks/                      # ★ 功能包（每个子目录是一个可复用的静态库）
│   ├── vision/                 # 纯视觉：形态学 / 灯条-装甲板识别 / PnP / 坐标链 / 卡尔曼 / HUD
│   │   ├── CMakeLists.txt
│   │   ├── vision_processing.hpp
│   │   └── vision_processing.cpp
│   ├── detection/              # 检测器抽象：传统视觉 / 神经网络（--detector 切换）
│   │   ├── CMakeLists.txt
│   │   ├── detector.hpp
│   │   └── detector.cpp
│   └── planning/               # 规划器：弹道解算 + 提前量预测
│       ├── CMakeLists.txt
│       ├── planning.hpp
│       └── planning.cpp
├── tests/                      # 单元测试（无需模拟器，直接 ./build/xxx_test 运行）
│   ├── planning_test.cpp
│   └── vision_test.cpp
└── tools/                      # 工具脚本
    ├── ov_armor_service.py     # OpenVINO 神经网络检测子进程服务
    ├── plot_pnp.py             # 单次 PnP 曲线画图
    └── plot_pnp_compare.py     # 多份 CSV 对比画图
```

## 2. 功能包（`tasks/<package>`）

每个功能包是一个**可独立编译、测试、复用的静态库**，自带 `CMakeLists.txt`。
包内约定：

- **对外接口只通过头文件暴露**，头文件命名与包名一致（如 `vision_processing.hpp`）。
- 包内源码用相对路径包含**同包头文件**：`#include "vision_processing.hpp"`。
- **跨包引用**用项目根目录相对路径：`#include "tasks/vision/vision_processing.hpp"`。
  （顶层 CMakeLists 已 `include_directories(${PROJECT_SOURCE_DIR})`，与 tongji 一致。）
- 包之间用 `target_link_libraries(... PUBLIC ...)` 表达依赖，依赖关系自动传递。

### 2.1 `tasks/vision` —— 纯视觉

只依赖 OpenCV，**不含任何模拟器 SDK 依赖**。这是"纯算法"层，可独立测试、移植：

- 形态学滤波：`splitColorMask` / `applyMorphology`。
- 灯条/装甲板识别：`detect`（含候选调试信息）。
- PnP 位姿解算：`solveArmorPose`。
- 坐标链：`cameraToGimbal` / `gimbalToCamera` / `gimbalToWorld` / `worldToGimbal` /
  `projectPoint` / `pixelToCamera` / `pixelToWorld` / 旋转链与欧拉角。
- 瞄准角：`worldToAimAngles` / `absoluteAimAngles` / `aimWithGravity`。
- 状态估计：`KalmanFilter3D`（9 维恒加速度模型）。
- 可视化：`drawResult` / `makeGrid` / `drawAimHud`。

### 2.2 `tasks/detection` —— 检测器抽象

定义 `detect::IDetector` 接口与工厂 `makeDetector()`，上层**不需要关心后端**：

- `VisionDetector`：形态学 + 灯条配对（调用 `vision` 包）。
- `NNDetector`：通过 Python 子进程（`tools/ov_armor_service.py`）跑 OpenVINO YOLO。
- 工厂：NN 加载失败自动回退传统检测。

依赖：`vision`。

### 2.3 `tasks/planning` —— 射击规划

- `solveBallistic`：无空气阻力弹道求解（参考 tongji `tools/trajectory.cpp`）。
- `predictAhead`：恒加速度位置预测。
- `planAimPoint`：迭代求解"预测时刻 = 开火延迟 + 飞行时间"的瞄准点（参考 tongji
  `Aimer::aim`）。

只依赖 OpenCV 核心类型。

## 3. 可执行程序（`src/`）

`src/` 下的程序**只做组装**：解析命令行 → 连接 SDK → 调用功能包 → 输出/可视化。
公共逻辑（读帧、标定加载、`frameToBgr` 等）在各自的 `.cpp` 内，未跨程序共享；
如需进一步去重，可考虑抽到 `tools/`（参考 tongji 的 `tools` 工具库）。

## 4. 构建系统

顶层 `CMakeLists.txt`：

```cmake
include_directories(${PROJECT_SOURCE_DIR})   # 全局 include 根目录
add_subdirectory(tasks/vision)               # 生成静态库 vision
add_subdirectory(tasks/detection)            # 生成静态库 detection（依赖 vision）
add_subdirectory(tasks/planning)             # 生成静态库 planning

add_executable(autoaim002      src/main.cpp)
add_executable(autoaim002_test src/contest_test.cpp)
add_executable(pnp_curve_test  src/pnp_curve_test.cpp)

add_executable(planning_test tests/planning_test.cpp)   # 单元测试
add_executable(vision_test   tests/vision_test.cpp)
```

每个功能包的 `CMakeLists.txt` 形如：

```cmake
add_library(<package> STATIC <sources>...)
target_link_libraries(<package> PUBLIC <依赖的包/库>)
```

依赖图：

```
src/main.cpp ──────────────┐
src/contest_test.cpp ──────┼──▶ tasks/detection ──▶ tasks/vision
src/pnp_curve_test.cpp ────┤       │
                          └──▶ tasks/planning
```

`tasks/detection` 依赖 `tasks/vision`；`tasks/planning` 独立；`tasks/vision` 只依赖
OpenCV。

## 5. 测试

`tests/` 下是对各功能包的**离线单元测试**（不连模拟器、不需要图形环境）：

```bash
./build/vision_test      # vision 包：掩膜/形态学/识别/坐标/PnP/卡尔曼
./build/planning_test    # planning 包：弹道/预测/瞄准点
```

新增功能包时建议同步添加对应 `tests/xxx_test.cpp` 并在顶层 CMakeLists 注册。

## 6. 如何新增一个功能包

以新增 `tasks/trajectory/` 为例：

1. 创建目录并放置 `trajectory.hpp` / `trajectory.cpp`（同包内引用用相对路径）。
2. 编写 `tasks/trajectory/CMakeLists.txt`：
   ```cmake
   add_library(trajectory STATIC trajectory.cpp)
   target_link_libraries(trajectory PUBLIC ${OpenCV_LIBS})
   ```
3. 在顶层 `CMakeLists.txt` 加 `add_subdirectory(tasks/trajectory)`。
4. 其他包/程序用 `#include "tasks/trajectory/trajectory.hpp"` 引用，并在对应
   `target_link_libraries` 中加入 `trajectory`。
5. （可选）在 `tests/` 增加 `trajectory_test.cpp` 并注册。

## 7. 与 tongji 的对应关系

| tongji | autoaim002（本工程） | 说明 |
| --- | --- | --- |
| `tasks/auto_aim/`、`tasks/auto_buff/`、`tasks/omniperception/` | `tasks/vision/`、`tasks/detection/`、`tasks/planning/` | 功能包（静态库 + 自带 CMakeLists） |
| `src/standard.cpp` 等 | `src/main.cpp` 等 | 可执行程序入口 |
| `tests/` | `tests/` | 测试 |
| `tools/` | `tools/` | 工具库 / 脚本 |
| `configs/*.yaml` | `configs/` | 运行时配置 |
| 顶层 `include_directories(PROJECT_SOURCE_DIR)` | 同 | 跨包按项目根相对路径引用头文件 |

相比 tongji，本工程规模小、无 ROS/IO/标定等模块，因此只拆出三个与自瞄链路直接相关的
功能包；后续如接入真实相机（io）、云台/底盘通信等，可继续按同样模式扩展。
