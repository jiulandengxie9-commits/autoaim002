# autoaim002 — Daedalus 模拟器 SDK 图像读取与形态学滤波

通过 Daedalus 竞赛版模拟器（`linux-x86_64` 发行版）的 C++17 SDK 读取相机图像与模拟器
状态信息，使用 OpenCV 显示图像，并把读取到的信息实时打印到终端。图像会送入独立的
**纯视觉处理模块**（`src/vision_processing.*`）做形态学滤波（腐蚀、膨胀、开运算、
闭运算）、灯条/装甲板识别（外接矩形框），并利用 `camera-calibration.json` 的相机
内参/外参结合 **PnP** 求出装甲板与灯条的**云台系（世界系）三维坐标**，可视化各阶段
结果。配套 `autoaim002_test` 按校内赛规则 6 工况自动驱动靶车并自瞄开火。

## 功能

- 连接模拟器（TCP 图像 + UDP 云台 + UDP 场景控制 三合一 `ContestClient`）。
- 读取曝光同步的相机帧（默认 RGBA32，1440×1080）并显示在 OpenCV 窗口中。
- 终端实时输出每一帧的信息：
  - 图像：`source_sequence`、`producer_epoch`、时间戳、分辨率、像素格式、载荷大小、帧率。
  - 云台：`yaw` / `pitch` 绝对角、角速度、最后应用的命令号（与曝光同帧同步）。
  - 能量机关场景下：红/蓝大符得分统计（`arms` / `avg_ring` / `last_ring` 等）。
  - 形态学滤波：滤波前后掩膜像素数、噪声去除百分比、检测到的轮廓数。
  - 灯条/装甲板识别：检测到的灯条数、装甲板数。
  - **PnP 世界坐标**：对每个装甲板用 `solvePnP` 结合 `camera-calibration.json`
    的相机内参解算位姿，再用外参把相机系坐标变换到云台系（世界系），并打印
    装甲板中心与左右灯条的三维坐标及距离（米）。
  - **卡尔曼滤波**：将恒加速度系统模型与上述 PnP 相机坐标融合，对装甲板位置
    做平滑/预测（`KalmanFilter3D`），终端打印原始 vs 滤波后的云台系坐标与
    速度，窗口/保存图中用绿色十字标注滤波位置。
  - **像素→世界坐标**：`pixelToWorld()` 把任意像素坐标 + 深度换算成云台系
    （世界系）三维坐标（`pixelToCamera` 反投影 + 外参变换）。
  - **瞄准角可视化**：`drawAimHud()` 在画面上叠加瞄准 HUD——当前云台
    `yaw/pitch`、目标瞄准角与距离、中心十字线、水平参考线，以及目标相对
    云台的角度偏差指示（`worldToAimAngles()`）。
- 形态学滤波（默认开启，`--no-morph` 关闭）：
  - `splitRedMask`：`R-B` 通道差阈值分割出红色灯条掩膜。
  - `applyMorphology`：腐蚀 → 膨胀 → 开运算 → 闭运算 → 开+闭级联，输出最终掩膜。
  - `findContours` 提取轮廓，`--max-area` 可滤除面积超限的干扰区域。
- 灯条与装甲板识别（在滤波后的轮廓上执行）：
  - `findLights`：用最小外接旋转矩形描述灯条，检验面积、凸度、长宽比。
  - `matchArmor`：对灯条两两配对，检验角度差、长度差、位置关系、装甲板比例，
    生成四顶点装甲板框（外接矩形框）。
  - 结果图绘制：绿色=轮廓，黄色=灯条外接矩形，蓝色=装甲板四顶点框。
  - 窗口同时显示 8 个阶段：原图/阈值/腐蚀/膨胀/开/闭/最终掩膜/结果。
  - 主窗口叠加瞄准 HUD：当前云台角、目标瞄准角/距离、十字线与角度偏差指示。
- PnP 三维位姿估计（世界坐标）：
  - 加载发行版自带的 `camera-calibration.json`（内参 + 云台→相机外参）。
  - 每个装甲板用 4 个图像角点 + 标准装甲板尺寸做 `solvePnP`（IPPE），得到相机系
    位姿 `t_cam/r_cam` 与距离。
  - 用外参把相机系坐标变换到云台（世界）系，得到装甲板中心、四角点以及左右灯条
    中心的三维世界坐标。
  - 终端打印 `pnp armor[i]: distance=… gimbal=(x, y, z)`，并在主窗口/保存图中
    于装甲板旁标注距离与云台系坐标。
- 卡尔曼滤波（默认开启，`--no-kalman` 关闭）：
  - `KalmanFilter3D`：9 维状态 `[x, y, z, vx, vy, vz, ax, ay, az]`，恒加速度
    运动模型（状态转移矩阵同 `big_homework.cpp` 的分轴卡尔曼，但合并在 3D 中）。
  - `predict(dt)`：用曝光时间戳算出帧间隔 `dt` 推进运动模型；`update(z)` 把
    PnP 测得的云台系坐标作为观测值融合，得到更平滑的后验位置与速度。
  - 单目标跟踪：初始选择最近的装甲板，后续把预测位置最近的观测关联到滤波
    器（门限 2 m），丢失超过 60 帧自动重置。
  - 终端打印 `kf: raw=(…) filtered=(…) vel=(…)`；主窗口/保存图中用绿色十字
    （含 `KF` 标注）显示滤波后的位置。
- 启动时输出额外信息：SDK 版本、模拟器健康状态（`health()`）、运行时 GPU 能力
  （`readRuntimeCapabilities`）、共享内存元数据、相机内参、底盘观测。
- 可选：把收到的帧与各阶段掩膜保存为 PNG；指定接收帧数上限后自动退出。

## 目录结构

```
autoaim002/
├── CMakeLists.txt              # CMake 构建脚本，链接 SDK 与 OpenCV
├── README.md                   # 本文档
├── QUICKSTART.md               # 快速开始
├── docs/
│   └── CODE_WALKTHROUGH.md     # 代码逐段讲解
└── src/
    ├── main.cpp                # 主程序：SDK 读帧 + 调用视觉模块 + 显示/打印
    ├── contest_test.cpp        # 校内赛自瞄测试工具（6 工况自动驱动靶车 + 自瞄开火）
    ├── detector.hpp/.cpp       # 检测器抽象：传统视觉 / 神经网络（--detector 切换）
    ├── planning.hpp/.cpp       # 规划器：弹道解算 + 提前量预测
    ├── vision_processing.hpp   # 纯视觉处理模块声明（形态学 + 灯条/装甲板识别 + PnP + 卡尔曼）
    └── vision_processing.cpp   # 纯视觉处理模块实现
└── tools/
    ├── ov_armor_service.py     # OpenVINO 神经网络检测子进程服务
    ├── plot_pnp.py             # 单次 PnP 曲线画图（camera/gimbal/world/距离/框/数量）
    └── plot_pnp_compare.py     # 多份 CSV 对比画图（raw vs Kalman、距离、抖动）
```

`src/vision_processing.*` 是**纯图像处理**模块：只接收 `cv::Mat` 与标定参数，
不含任何模拟器/SDK 依赖，可独立测试或复用到其他工程。

## 依赖

- 已安装的 Daedalus 竞赛版 SDK（本机位于 `/home/xqy/autoaim/linux-x86_64/sdk`，
  包含 `include/`、`lib/libdaedalus_sim_sdk.a` 与 CMake 配置）。
- 编译器与构建工具：`g++`（支持 C++17）、`cmake`。
- OpenCV（4.x，需要 `core`、`highgui`、`imgproc` 模块；`pkg-config opencv4` 可检测）。
- 运行时需要图形环境显示窗口（OpenCV `highgui`）。

## 构建

```bash
cd /home/xqy/桌面/autoaim002
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

生成的程序位于 `build/autoaim002`。如果 SDK 不在默认路径，请通过 `-DDAEDALUS_SDK_ROOT=`
指定，例如：

```bash
cmake -S . -B build -DDAEDALUS_SDK_ROOT=/path/to/linux-x86_64/sdk
```

## 运行

先启动模拟器（任选其一）：

```bash
# 使用发行版脚本（IPC 目录 = $XDG_RUNTIME_DIR/daedalus-contest-<uid>，桌面会话下即
# /run/user/<uid>/daedalus-contest-<uid>；脚本启动时会打印 runtime_dir=...）
/home/xqy/autoaim/linux-x86_64/daedalus-contest.sh start --scene shooting-range

# 或直接使用底层启动脚本，指定自定义 IPC 目录
/home/xqy/autoaim/linux-x86_64/start-simulator.sh --visible --ipc-dir /tmp/my-ipc
```

再运行本程序。默认 IPC 目录与 `daedalus-contest.sh` 保持一致（优先 `$XDG_RUNTIME_DIR`，
否则 `/tmp`），所以一般无需传参；只有模拟器用了自定义目录时才需要显式指定：

```bash
./build/autoaim002 --scene shooting-range
# 自定义目录时：
./build/autoaim002 --ipc-dir /run/user/1000/daedalus-contest-1000 --scene shooting-range
```

按 `q` 或 ESC 关闭窗口退出；关闭窗口本身也会退出。

### 常用参数

| 参数 | 说明 | 默认值 |
| --- | --- | --- |
| `--ipc-dir PATH` | 模拟器使用的 IPC 目录 | `$XDG_RUNTIME_DIR`/`/tmp` 下的 `daedalus-contest-<uid>` |
| `--scene NAME` | `shooting-range` / `energy` / `large-energy` | `shooting-range` |
| `--max-frames N` | 收到 N 帧后自动退出，0 表示不限制 | `0` |
| `--save-dir PATH` | 同时把每帧（以及形态学各阶段 PNG）保存到该目录 | 不保存 |
| `--window NAME` | 主窗口标题 | `Daedalus Simulator` |
| `--morph` / `--no-morph` | 开启 / 关闭形态学滤波 | 开启 |
| `--kernel N` | 形态学结构元素尺寸（正奇数） | `5` |
| `--rb-threshold N` | 掩膜的颜色通道差阈值 | `30` |
| `--color NAME` | 灯条颜色：`red`（R-B）或 `blue`（B-R） | `red` |
| `--detector NAME` | 检测后端：`nn`（默认，OpenVINO）或 `traditional`（形态学） | `nn` |
| `--nn-model PATH` | `--detector nn` 的 OpenVINO/ONNX 模型路径 | tongji `yolo11.xml` |
| `--max-area N` | 滤除面积大于 N 像素的轮廓，0 表示保留全部 | `0` |
| `--grid-width N` | 形态学对比网格中每个单元格的宽度（≥160），越大文字越清晰 | `480` |
| `--min-light-area N` | 灯条最小轮廓面积 | `100` |
| `--min-solidity N` | 灯条最小凸度（轮廓面积/凸包面积） | `0.3` |
| `--max-light-ratio N` | 灯条最大宽/长比 | `0.8` |
| `--max-angle-diff N` | 两灯条最大角度差（度） | `15` |
| `--max-height-diff N` | 两灯条最大长度差比率 | `0.2` |
| `--max-y-diff N` | 两灯条中心最大 y 差比率 | `0.5` |
| `--min-x-diff N` | 两灯条中心最小 x 差比率 | `0.5` |
| `--max-armor-ratio N` | 装甲板最大距离/高度比 | `2.5` |
| `--min-armor-ratio N` | 装甲板最小距离/高度比 | `1.0` |
| `--det-debug` | 每帧打印所有灯条候选装甲板的约束值（角差/长度差/y差/x差/装甲比例） | 关 |
| `--calibration PATH` | 相机标定文件（内参 + 外参）路径 | `/home/xqy/autoaim/linux-x86_64/camera-calibration.json` |
| `--armor-width M` | 装甲板实际宽度（米），PnP 用 | `0.135` |
| `--armor-height M` | 装甲板实际高度（米），PnP 用 | `0.056` |
| `--kalman` / `--no-kalman` | 开启 / 关闭卡尔曼平滑 | 开启 |
| `--kf-pos-sigma N` | 卡尔曼位置过程噪声标准差（米） | `0.01` |
| `--kf-vel-sigma N` | 卡尔曼速度过程噪声标准差（米/秒） | `0.1` |
| `--kf-acc-sigma N` | 卡尔曼加速度过程噪声标准差（米/秒²） | `0.2` |
| `--kf-meas-sigma N` | 卡尔曼观测噪声标准差（米） | `0.03` |
| `-h` / `--help` | 显示帮助 | — |

示例：读取能量机关大符画面，开启形态学滤波、滤除面积 > 5000 的干扰区域、放大调试网格
并把各阶段结果保存下来：

```bash
./build/autoaim002 --scene energy --max-area 5000 --grid-width 640 \
                   --save-dir ./frames --max-frames 300
# 只读帧不做滤波：
./build/autoaim002 --no-morph --save-dir ./frames --max-frames 300
```

## 检测后端（传统视觉 / 神经网络）

识别装甲板支持两种后端，用 `--detector` 运行时切换（`main.cpp` 与
`autoaim002_test` 都支持），**默认神经网络**：

| 后端 | 实现 | 说明 |
| --- | --- | --- |
| `nn`（默认） | `NNDetector`：OpenVINO（Python 子进程服务 `tools/ov_armor_service.py`）加载 YOLO，letterbox 640×640 | 参考 `tongji/tasks/auto_aim` 的 yolo11.xml（38 类四点模型）；输出 xywh+类+4 关键点 |
| `traditional` | `VisionDetector`：`splitColorMask` → `applyMorphology` → `detect` | 形态学滤波 + 灯条/装甲板配对，纯 OpenCV |

`src/detector.{hpp,cpp}` 定义 `detect::IDetector` 抽象接口与工厂
`makeDetector()`。神经网络后端通过 C++ 启动一个常驻 Python 子进程（OpenVINO
推理），C++ 每帧发送 BGR 原始帧、回收 JSON 检测结果；子进程无法启动时自动回退
`traditional` 并打印原因。

```bash
./build/autoaim002 --scene shooting-range            # 默认 nn（OpenVINO）
./build/autoaim002 --detector traditional --scene shooting-range  # 形态学
./build/autoaim002 --nn-model /path/to/yolo11.xml    # 指定模型
```

> 注意：C++ 直连 OpenVINO 运行时对本机该 INT8 模型推理结果异常，故采用 Python
> 子进程方式（已验证可靠）。运行前需 `python3 -m pip install --break-system-packages openvino`。

## 形态学滤波说明

本程序的视觉处理部分参考了 `~/桌面/big_homework/big_homework.cpp` 的灯条检测思路，
并拆分为独立的纯视觉模块 `src/vision_processing.*`，由 `main.cpp` 调用。处理流程：

1. **阈值分割**（`splitRedMask`）：BGR 三通道分离后计算 `R - B`，`threshold` 得到二值
   红色掩膜——红色灯条（R 高、B 低）亮起，其余为背景。
2. **腐蚀**（`erode`）：收缩前景，去掉细小的孤立噪点。
3. **膨胀**（`dilate`）：扩大前景，补上灯条内部的空洞与断缝。
4. **开运算**（`MORPH_OPEN` = 先腐蚀后膨胀）：整体去小噪点、断开粘连。
5. **闭运算**（`MORPH_CLOSE` = 先膨胀后腐蚀）：填充小孔、连接邻近区域。
6. **级联滤波**：对开运算结果再做一次闭运算作为最终掩膜（开+闭组合最常用）。
7. **轮廓提取**（`findContours`）：在最终掩膜上提取轮廓。
8. **面积滤除**（`--max-area N`）：丢弃面积大于 `N` 像素的轮廓（如过大的背景/干扰
   红色区域），终端会额外打印 `(filtered=N)`。

## 灯条与装甲板识别说明

在滤波后的轮廓上执行（参考 `~/桌面/big_homework/big_homework.cpp` 的
`LightDescriptor` / `ArmorDescriptor` / `findlight` / `matchArmor`）：

### 灯条约束（`detect` → `findLights`）

对每个轮廓用 `minAreaRect` 取最小外接矩形，再经 `adjustLightRect` 归一化（保证宽 ≤ 长、
角度归一化到 `[-45°, 45°]`），`LightDescriptor` 记录宽/长/中心/角度/面积。然后依次检查：

| 约束 | 参数 | 默认值 | 含义 |
| --- | --- | --- | --- |
| 最小面积 | `--min-light-area` | `100` | 轮廓面积必须大于该值，滤掉小噪点 |
| 最少点数 | — | `5` | 轮廓点数 ≥ 5 才可做 `minAreaRect` |
| 最小凸度 | `--min-solidity` | `0.3` | 轮廓面积/凸包面积，滤掉形状破碎区域 |
| 最大宽长比 | `--max-light-ratio` | `0.8` | 归一化后 `width/length`，滤掉横置的块 |

### 装甲板约束（`detect` → `matchArmor`）

按 `center.x` 升序排列灯条后两两配对，左右灯条满足以下全部条件才构成装甲板：

| 约束 | 参数 | 默认值 | 含义 |
| --- | --- | --- | --- |
| 角度差 | `--max-angle-diff` | `15`° | 两灯条倾斜角之差，装甲板灯条应近似平行 |
| 长度差比率 | `--max-height-diff` | `0.2` | `|len1-len2|/max(len1,len2)`，高度应相近 |
| y 差比率 | `--max-y-diff` | `0.5` | `|y1-y2|/meanLen`，中心 y 应近似对齐 |
| x 差比率 | `--min-x-diff` | `0.5` | `|x1-x2|/meanLen`，左右必须分开 |
| 装甲板比例 | `--min/max-armor-ratio` | `1.0~2.5` | `距离/meanLen`，装甲板长宽比范围 |

通过的配对用左右灯条中心与平均角度构造 `RotatedRect`，取四个顶点（
`adjustVertexOrder` 归一为左上/右上/右下/左下）作为装甲板外接矩形框。

结果图颜色约定：**绿色**=掩膜轮廓，**黄色**=灯条外接矩形，**蓝色**=装甲板四顶点框。
终端每帧打印 `detect: lights=N armors=M`。

上述所有阈值都可命令行覆盖，便于针对不同距离/光照调参。

## PnP 三维位姿估计（世界坐标）

识别到装甲板后，程序对每个装甲板求解其在**云台系（世界系）**中的三维坐标。实现位于
`src/vision_processing.{hpp,cpp}` 的 `solveArmorPose()`，纯 OpenCV，不依赖 SDK：

1. **加载标定**：启动时解析发行版自带的 `camera-calibration.json`
   （可用 `--calibration` 覆盖），得到：
   - 内参：`fx/fy/cx/cy` 与畸变系数（本发行版为无畸变 `plumb_bob`，全 0）。
   - 外参：云台系→相机光学系的平移 `translation_m` 与旋转 `quaternion_xyzw`。
2. **solvePnP（IPPE）**：以装甲板 4 个图像角点（`ArmorDescriptor.vertex`，
   顺序 左上/右上/右下/左下）为 `image_points`，以真实装甲板尺寸
   `--armor-width × --armor-height`（默认 0.135 × 0.056 m，即小装甲板）构造
   局部系 3D 角点为 `object_points`，用内参求解 `rvec/tvec`。
   - `t_cam`：装甲板中心在相机光学系中的坐标（米）。
   - `distance_m`：相机到装甲板中心的距离。
3. **外参变换**：`cameraToGimbal()` 计算 `p_gimbal = Rᵀ·(p_cam − t)`，把相机系
   坐标变换到云台系（世界系）。装甲板中心、四角点以及左右灯条中心（用
   `solvePnP` 得到的装甲板平面 + 像素光线求交获得三维坐标）都输出到云台系。
4. **旋转链变换**（参考 `tongji/tasks/auto_aim/solver.cpp` 的 `solve()`）：
   - `rotationCameraToGimbal()`：`R_armor2gimbal = R_extᵀ · R_armor2camera`。
   - `rotationGimbalToWorld()`：`R_armor2world = R_gimbal2world · R_armor2gimbal`。
   - `rotationMatrixToYpr()`：提取 ZYX 欧拉角 `(yaw, pitch, roll)`。
   - `xyzToYpd()`：直角坐标→球坐标 `(yaw, pitch, distance)`。
   - `TargetPose` 新增 `r_gimbal`（云台系旋转向量）；`solveArmorPose()` 计算它。
5. **输出**：终端每帧打印 `pnp armor[i]: distance=… cam=(…) gimbal=(…) ypr_gimbal=(…)`
   以及（有世界位姿时）`world=(…) ypr_world=(…) ypd_world=(…)`；主窗口与
   `--save-dir` 保存的结果图中在装甲板旁标注距离与云台系坐标。

坐标系说明：`camera-calibration.json` 中 `extrinsics.from_frame = gimbal`、
`to_frame = camera_optical`，即外参描述的是相机相对云台的固定安装位姿。相机系为
OpenCV 光学系（+Z 朝前、+X 朝右、+Y 朝下），云台系为世界系（+Z 朝前），两者由
`cameraToGimbal()` 互转。

## 卡尔曼滤波（运动模型 + 相机坐标融合）

`big_homework.cpp` 用三个独立的一维卡尔曼分别滤波旋转中心的 x/y/z；`rmcs_auto_aim_v2`
用 `EKF` 维护整个机器人状态。本程序把两者结合为**单个 9 维线性卡尔曼**
（`src/vision_processing.{hpp,cpp}` 的 `KalmanFilter3D`），状态为
`[x, y, z, vx, vy, vz, ax, ay, az]`，直接对 PnP 求出的云台系（世界系）坐标做
平滑，供后续瞄准/预测使用：

1. **系统模型（恒加速度）**：状态转移矩阵
   ```
   A = [ I   dt·I  dt²/2·I ]
       [ 0    I     dt·I   ]
       [ 0    0      I     ]
   ```
   其中 `dt` 由相邻两帧的曝光时间戳差计算（`capture_timestamp_ns`，上限 0.5 s），
   模拟装甲板在云台系中的匀速/匀加速运动。
2. **观测模型**：`H = [I, 0, 0]`，观测为 `solveArmorPose()` 得到的云台系位置
   `t_gimbal`。预测协方差 `P_pred = A·P·Aᵀ + Q`，创新 `y = z − H·x_pred`，
   增益 `K = P·Hᵀ·(H·P·Hᵀ + R)⁻¹`，后验 `x = x_pred + K·y`。
3. **噪声参数**：过程噪声 `Q` 由 `--kf-pos-sigma/--kf-vel-sigma/--kf-acc-sigma`
   控制（位置/速度/加速度分块），观测噪声 `R` 由 `--kf-meas-sigma` 控制。
4. **关联与跟踪**：启动时选最近的装甲板初始化；后续每帧 `predict(dt)` 后，把与
   预测位置最近（< 2 m）的装甲板观测关联并 `update()`；连续 > 60 帧无关联则
   重置跟踪。这也为瞄准提供了速度估计 `(vx, vy, vz)` 以做提前量。

滤波结果比原始 PnP 抖动更小（合成噪声测试中平均误差约降低 20–40%），并能在目标
短暂丢失时给出更稳定的预测位置。

## 像素坐标 → 世界坐标与瞄准角

参考 `big_homework.cpp` 的坐标链（图像角点 → `solvePnP` → 相机系 → 世界系）与
`rmcs_auto_aim_v2` 的 `reproject_point` / `xyz2ypd` 思路，本程序提供一条完整的
双向坐标链：

```
像素 (u,v) ──pixelToCamera──▶ 相机光学系 (X,Y,Z) ──cameraToGimbal──▶ 云台系（世界系）
     ◀──projectPoint───────────  (Y,X,Z)  ◀───────gimbalToCamera─────
```

- **`pixelToCamera(uv, depth)`**：把像素 + 深度反投影到相机光学系（带 plumb-bob
  畸变逆变换，本发行版畸变全 0，退化为一阶透视）。
- **`pixelToWorld(uv, depth)`**：`pixelToCamera` 后再用外参 `cameraToGimbal` 得到
  世界系坐标。`solveArmorPose()` 内部即用该链把装甲板角点/灯条中心换算到世界系。
- **`worldToAimAngles(p_cam)`**：把目标相机系坐标换算成**云台系相对**瞄准角——
  `yaw = atan2(x, z)`，`pitch = 90° + atan2(y, √(x²+y²))`（与 SDK 语义一致：
  `pitch=90` 为水平）。
- **`absoluteAimAngles(p_cam, gimbal_yaw, gimbal_pitch, camera_tilt)`**：**绝对**云台
  瞄准角 = 当前云台角 + 目标相对相机光轴的偏差（yaw 分量按相机倾斜角 `cos(tilt)`
  缩放、pitch 分量沿倾斜轴直接叠加）。目标在世界中不动时该值恒定，即使只转动云台
  也不变；这就是把云台转到该 `yaw/pitch` 即可对准目标的命令角。终端 `kf:` 行打印
  `aim_yaw/aim_pitch`，HUD 的 `target yaw/pitch` 也是该绝对角。
- **`gimbalToWorld(p_gimbal, world_pose)`**：把云台系坐标变换到**绝对世界系（odom）**。
  `world_pose` 由 SDK 的 `readExposureStateForFrame()` 每帧提供（云台在世界系中的
  位置 + wxyz 四元数）。终端 `pnp armor[i]` 行同时打印 `gimbal=(…)` 与 `world=(…)`。
- **旋转链**（参考 `tongji/tasks/auto_aim/solver.cpp` 的 `solve()`）：
  - `rotationCameraToGimbal(r_cam, extrinsics)`：`R_armor2gimbal = R_extᵀ·R_armor2cam`。
  - `rotationGimbalToWorld(r_gimbal, world_pose)`：`R_armor2world = R_gimbal2world·R_armor2gimbal`。
  - `rotationMatrixToYpr(R)`：ZYX 欧拉角（yaw/pitch/roll）。
  - `xyzToYpd(xyz)`：直角→球坐标 `(yaw, pitch, distance)`。
  - 终端 `pnp armor[i]` 打印 `ypr_gimbal=(…)`（恒有）与 `ypr_world=(…)`、
    `ypd_world=(…)`（有世界位姿时）。

> **坐标系区别（重要）**：`gimbal=` 是**云台系**——相机/云台一起转动，因此仅转动
> 云台时该坐标会变化（这是瞄准计算所需的相对位置）；`world=` 是**绝对世界系
> （odom）**——目标在世界中不动时该坐标保持不变。
- **`drawAimHud()`**：在显示/保存图上叠加瞄准 HUD：
  - 左上角显示当前云台 `gimbal yaw/pitch`（来自曝光同步的 `cf.gimbal`）。
  - 有目标时显示 `target yaw/pitch dist`——这里的 `target` 是**绝对云台瞄准角**
    （`absoluteAimAngles`，目标不动时恒定），绿色圆环 + 连线标出它与当前云台角的
    偏差（每度约 8 像素）。

主窗口与 `--save-dir` 保存的 `result.png` 均包含该 HUD。

## 校内赛自瞄测试工具（autoaim002_test）

`src/contest_test.cpp` 编译为 `build/autoaim002_test`，按《RoboMaster 机甲大师校内赛
规则手册》装甲板自瞄任务（60 分）的 6 个工况自动驱动模拟器靶车并自瞄开火：

| # | 工况 | 模式 | 低速档 | 高速档 | 满分 |
| --- | --- | --- | --- | --- | --- |
| 1 | 原地旋转 | `Spin` | ω=4 rad/s | — | 6 |
| 2 | 原地旋转 | `Spin` | — | ω=9 rad/s | 8 |
| 3 | 横向平移 | `Linear` | v=0.6 m/s | — | 8 |
| 4 | 横向平移 | `Linear` | — | v=1.5 m/s | 10 |
| 5 | 组合运动 | `LinearAndSpin` | v=0.6, ω=4 | — | 12 |
| 6 | 组合运动 | `LinearAndSpin` | — | v=1.5, ω=9 | 16 |

每个工况：`SceneControlClient` 设置靶车运动（`setRangeTargetMotion`）→ 稳定 3 s →
进入计分窗口（≤30 s）读帧、检测、PnP、`sendAim` 瞄准开火，最多发射 `--rounds` 发
（默认 50）。

```bash
./build/autoaim002_test [--ipc-dir DIR] [--rounds 50] [--only N] [--no-fire] [--color red|blue] [--no-window] [--bullet-speed 25] [--fire-delay 0.1]
```

- 默认检测**红色**灯条（模拟器敌方装甲板灯条为红色）；`--color blue` 切蓝色。
- 默认弹出 **AutoAim Test** 可视化窗口，实时显示检测框、目标中心十字、瞄准 HUD
  （当前云台角/目标瞄准角/距离）与工况进度；`--no-window` 关闭（无图形环境自动降级）。
- **规划器提前量**：`planAimPoint()` 预测目标在 `开火延迟 + 弹丸飞行时间` 时刻的
  位置并解算弹道俯仰（参考 `tongji Aimer::aim` 的迭代）。`--bullet-speed` 设弹速
  （默认 25 m/s，规则固定值）；`--fire-delay SEC` 设命令到实际发射的固定延迟
  （云台响应 + 发弹机构，参考 tongji `low/high_speed_delay_time`），用于提前量。
- 输出每工况 `locked=`（锁定帧数）、`rounds_fired=N/50`、`score_estimate`。
- 由于 SDK 为只读数据源、不提供靶场装甲板命中真值，`score_estimate` 用
  `rounds_fired/50 × 满分` 作命中率代理；实际命中需裁判系统接入。
- 模拟器对 `Linear` 低速档有最小速度 clamp（实测 0.6 → 1.0 m/s），低速档参数以
  模拟器实际生效值为准。

**与主程序的关系**：两者共用 `vision_processing.*` 的检测/PnP/瞄准逻辑；主程序
`autoaim002` 侧重可视化调试，测试工具侧重自动跑分。

## 识别 + PnP 曲线测试（pnp_curve_test）

`src/pnp_curve_test.cpp` 编译为 `build/pnp_curve_test`：**仅做识别与 PnP 解算**，
不做云台控制/开火，把每帧检测到的装甲板 PnP 结果写入 CSV，供脚本画曲线图评估
识别稳定性、PnP 抖动与 Kalman 平滑效果。

```bash
./build/pnp_curve_test --frames 200 --out /tmp/pnp_curve.csv      # 默认 NN 检测
./build/pnp_curve_test --show --out /tmp/pnp_curve.csv            # 实时窗口显示识别框
./build/pnp_curve_test --detector traditional --out out.csv       # 传统视觉对比
```

- `--show`：弹出 OpenCV 窗口实时绘制检测框（蓝）+ 最近目标中心（绿）与 PnP
  距离/世界坐标标签；按 `q`/ESC 提前结束。
- CSV 每帧一行：`seq, t_ns, n_armors, best_cam_xyz, best_g_xyz, best_w_xyz,
  best_dist, kf_xyz, box_cx/cy/w/h`（best=最近目标，kf=世界系 Kalman 平滑）。

**画图脚本**：

```bash
python3 tools/plot_pnp.py /tmp/pnp_curve.csv /tmp/pnp_curve_plots   # 单份曲线
python3 tools/plot_pnp_compare.py [csv1 csv2 ...] [out_dir]         # 多份对比
```

- `plot_pnp.py`：camera/gimbal/world 坐标、距离、检测框、每帧装甲板数。
- `plot_pnp_compare.py`：raw vs Kalman 世界坐标对比、多工况距离/抖动对比。

## 说明与限制

- SDK 为只读数据源：不提供目标真值、检测框、PnP、预测器或模型管理，标定也是只读的。
  PnP 位姿估计是本程序自行实现的（参考 `big_homework.cpp` 与 `rmcs_auto_aim_v2` 的
  `solvePnP` 思路），属于自瞄消费者自身逻辑。
- `camera-calibration.json` 的**外参**描述的是云台系→相机光学系的固定安装位姿
  （`from_frame = gimbal, to_frame = camera_optical`），云台系即本程序输出的世界系。
- TCP 图像默认 RGBA32（1440×1080），alpha 通道为不透明，显示时丢弃第 4 通道。
- 云台 `pitch_deg = 90` 表示水平瞄准，与 `UdpGimbalCommand` 的语义一致。
- 大符环数读取仅在能量机关**大符规则驱动**状态下有效，超时/完成后数值按规则重置为
  零值，这是正常行为而非读取失败。
