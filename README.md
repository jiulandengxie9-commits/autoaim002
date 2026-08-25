# autoaim002 — Daedalus 模拟器 SDK 图像读取与形态学滤波

通过 Daedalus 竞赛版模拟器（`linux-x86_64` 发行版）的 C++17 SDK 读取相机图像与模拟器
状态信息，使用 OpenCV 显示图像，并把读取到的信息实时打印到终端。图像会送入独立的
**纯视觉处理模块**（`src/vision_processing.*`）做形态学滤波（腐蚀、膨胀、开运算、
闭运算）与灯条/装甲板识别（外接矩形框），并可视化各阶段结果。

## 功能

- 连接模拟器（TCP 图像 + UDP 云台 + UDP 场景控制 三合一 `ContestClient`）。
- 读取曝光同步的相机帧（默认 RGBA32，1440×1080）并显示在 OpenCV 窗口中。
- 终端实时输出每一帧的信息：
  - 图像：`source_sequence`、`producer_epoch`、时间戳、分辨率、像素格式、载荷大小、帧率。
  - 云台：`yaw` / `pitch` 绝对角、角速度、最后应用的命令号（与曝光同帧同步）。
  - 能量机关场景下：红/蓝大符得分统计（`arms` / `avg_ring` / `last_ring` 等）。
  - 形态学滤波：滤波前后掩膜像素数、噪声去除百分比、检测到的轮廓数。
  - 灯条/装甲板识别：检测到的灯条数、装甲板数。
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
    ├── vision_processing.hpp   # 纯视觉处理模块声明（形态学滤波）
    └── vision_processing.cpp   # 纯视觉处理模块实现
```

`src/vision_processing.*` 是**纯图像处理**模块：只接收 `cv::Mat`，不含任何模拟器/SDK
依赖，可独立测试或复用到其他工程。

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
| `--rb-threshold N` | 红色掩膜的 R-B 阈值 | `30` |
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
| `-h` / `--help` | 显示帮助 | — |

示例：读取能量机关大符画面，开启形态学滤波、滤除面积 > 5000 的干扰区域、放大调试网格
并把各阶段结果保存下来：

```bash
./build/autoaim002 --scene energy --max-area 5000 --grid-width 640 \
                   --save-dir ./frames --max-frames 300
# 只读帧不做滤波：
./build/autoaim002 --no-morph --save-dir ./frames --max-frames 300
```

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

## 说明与限制

- SDK 为只读数据源：不提供目标真值、检测框、PnP、预测器或模型管理，标定也是只读的。
- TCP 图像默认 RGBA32（1440×1080），alpha 通道为不透明，显示时丢弃第 4 通道。
- 云台 `pitch_deg = 90` 表示水平瞄准，与 `UdpGimbalCommand` 的语义一致。
- 大符环数读取仅在能量机关**大符规则驱动**状态下有效，超时/完成后数值按规则重置为
  零值，这是正常行为而非读取失败。
