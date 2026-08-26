# 快速开始

从零到看到模拟器画面 + 终端信息，约 5 分钟。

## 1. 确认依赖

```bash
g++ --version          # 需要支持 C++17
cmake --version        # >= 3.16
pkg-config --modversion opencv4   # 有输出即已安装 OpenCV 4.x
```

本机均已满足。SDK 位于 `/home/xqy/autoaim/linux-x86_64/sdk`。

## 2. 构建

```bash
cd /home/xqy/桌面/autoaim002
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

成功后在 `build/` 下生成 `autoaim002`。

## 3. 启动模拟器

```bash
/home/xqy/autoaim/linux-x86_64/daedalus-contest.sh start --scene shooting-range
```

脚本会打印 `runtime_dir=...`。该目录就是程序的 **IPC 目录**：桌面会话下通常是
`/run/user/<uid>/daedalus-contest-<uid>`（`$XDG_RUNTIME_DIR` 存在时），否则是
`/tmp/daedalus-contest-<uid>`。

## 4. 运行程序

打开另一个终端。程序默认会自动使用和 `daedalus-contest.sh` 相同的 IPC 目录，无需
额外参数：

```bash
cd /home/xqy/桌面/autoaim002
./build/autoaim002 --scene shooting-range
```

如果模拟器是用自定义目录启动的（`start-simulator.sh --ipc-dir PATH` 或
`daedalus-contest.sh --runtime-dir PATH`），请显式传 `--ipc-dir`：

```bash
./build/autoaim002 --ipc-dir /run/user/1000/daedalus-contest-1000 --scene shooting-range
```

看到两个 OpenCV 窗口：主窗口显示检测结果图（绿色=轮廓，黄色=灯条外接矩形，蓝色=
装甲板框），识别到装甲板时其旁标注 PnP 距离与云台系坐标（黄色文字），若卡尔曼滤波
生效还会在滤波位置画一个绿色十字（带 `KF` 标注）；主窗口左上角还叠加**瞄准 HUD**
——当前云台 `yaw/pitch`、目标瞄准角与距离（绿色）、画面中央十字线与水平参考线，
以及目标相对云台的角度偏差圆环。**Morphology Stages** 窗口显示形态学滤波的 8 个
阶段（原图/阈值/腐蚀/膨胀/开/闭/最终掩膜/结果）。终端持续打印图像、云台、滤波/
检测统计、PnP 世界坐标与卡尔曼滤波结果信息：

```
Daedalus SDK 1.3.1  IPC dir: /run/user/1000/daedalus-contest-1000
connected to simulator.
simulator health: schema=1 product=1.3.1-contest backend=vulkan adapter=...
scene 'shooting-range' selected: status=Ok command_id=1 applied_frame_seq=...
metadata: magic=0x54414c05 version=7 abi_rev=2 image=1440x1080
camera: fx=1303.68 fy=1303.68 cx=720 cy=540 1440x1080
calibration: fx=1303.68 fy=1303.68 cx=720 cy=540 (from /home/xqy/autoaim/linux-x86_64/camera-calibration.json)
morph: enabled kernel=5 rb_threshold=30 max_area=0 grid_width=480 det(...)

===== frame 1 =====  (60.0 fps)
image: seq=42 producer_epoch=1 timestamp_ns=123456789
       1440x1080 format=RGBA32 payload=6220800 bytes
gimbal: yaw=0.0 deg pitch=90.0 deg yaw_vel=0.0 deg/s pitch_vel=0.0 deg/s last_cmd=0
morph: kernel=5 rb_threshold=30 mask_before=12983 mask_after=12960 noise_removed=0.2% contours=1
detect: lights=2 armors=1
pnp armor[0]: distance=2.75 m cam=(0.02, 0.01, 2.74) gimbal=(0.00, 0.00, 3.00) ypr_gimbal=(0.00, 15.00, 0.00) deg world=(0.00, 0.00, 3.00) ypr_world=(0.00, 15.00, 0.00) deg ypd_world=(0.00, 90.00, 3.00)
  light[0]: gimbal=(-0.07, 0.00, 3.01) world=(-0.07, 0.00, 3.01)
  light[1]: gimbal=(0.07, 0.00, 2.99) world=(0.07, 0.00, 2.99)
kf: raw=(0.01, 0.01, 3.02) filtered=(0.00, 0.00, 3.00) vel=(0.10, 0.02, -0.01) m/s lost=0 aim_yaw=3.81 aim_pitch=92.86
```

> **`ypr_gimbal` / `ypr_world`**：装甲板姿态的 ZYX 欧拉角（yaw/pitch/roll，度），
> 分别在其云台系与世界（odom）系下；**`ypd_world`** 是世界系球坐标
> （yaw/pitch/distance）。这些来自与位置同链的旋转变换（相机→云台→世界）。

> **`gimbal=` 与 `world=` 的区别**：`gimbal=` 是云台系坐标（随云台一起转动，仅动
> 云台时该值会变，用于瞄准）；`world=` 是绝对世界系（odom）坐标（目标在世界中
> 不动时保持不变）。若发现只转云台而 `gimbal=` 变化，属正常现象，看 `world=` 即可。
>
> **`aim_yaw/aim_pitch`（HUD 的 `target`）是绝对云台瞄准角**：把云台转到该角度即
> 可对准目标。目标在世界中不动时它恒定，即使只转动云台也不变。

主窗口的瞄准 HUD 效果（示例）：

```
┌──────────────────────────────────────────────┐
│ gimbal yaw=    0.00  pitch=   90.00 deg  ◄ 当前云台角（黄色）
│ target yaw=    3.81  pitch=   92.86 deg  dist= 3.01 m  ◄ 绝对瞄准角（绿色）
│            ══════════════════════════════════  ◄ 水平参考线
│                   +  ●                       ◄ 十字线 + 目标与当前云台角偏差圆环
└──────────────────────────────────────────────┘
```

`target` 为绝对云台瞄准角：目标在世界中不动时，无论云台转到哪，它都保持不变
（= 当前云台角 + 相对偏差）。

只读帧、不做滤波时加 `--no-morph`。常用调试参数：

- `--kernel N`：结构元素尺寸（正奇数），调节滤波强度。
- `--rb-threshold N`：红色分割阈值。
- `--max-area N`：滤除面积大于 N 像素的干扰轮廓，终端会打印 `(filtered=N)`。
- `--grid-width N`：放大/缩小形态学网格单元格（默认 480，越大文字越清晰，例如
  `--grid-width 640`）。
- `--min-light-area N`、`--max-light-ratio N`、`--min-solidity N`：灯条约束（面积、
  长宽比、凸度）。
- `--max-angle-diff N`、`--max-y-diff N`、`--min-x-diff N`、`--max-armor-ratio N` 等：
  装甲板配对约束。
- `--det-debug`：调试灯条配对。开启后每帧打印所有候选装甲板（灯条两两配对）的
  约束值，便于观察哪条约束拒绝了配对：
  `cand L0+L1 [OK]/[rej] angle_diff=… len_diff_ratio=… y_diff_ratio=… x_diff_ratio=… ratio=…`。
- `--calibration PATH`：相机标定文件路径（默认发行版 `camera-calibration.json`）。
- `--armor-width M`、`--armor-height M`：PnP 使用的装甲板真实尺寸（米）。
- `--detector nn|traditional`：切换检测后端——`nn`（默认，OpenVINO 神经网络，
  需 `python3 -m pip install --break-system-packages openvino`）、`traditional`
  （形态学+灯条配对）。NN 后端启动 Python 子进程推理。
- `--no-kalman`：关闭卡尔曼滤波（默认开启）。调参示例：
  - 目标抖动明显 → 增大 `--kf-meas-sigma`（观测噪声，默认 0.03）。
  - 目标快速变速 → 增大 `--kf-acc-sigma`（加速度过程噪声，默认 0.2）。
  - `--kf-pos-sigma` / `--kf-vel-sigma` 分别控制位置/速度过程噪声。

完整参数见 `./build/autoaim002 --help`。

## 5. 跑校内赛自瞄测试（autoaim002_test）

模拟器保持运行，另开终端：

```bash
cd /home/xqy/桌面/autoaim002
./build/autoaim002_test --rounds 50            # 全部 6 工况，每工况 50 发
./build/autoaim002_test --only 0 --rounds 50   # 只跑工况 1（原地旋转低速）
./build/autoaim002_test --no-fire              # 只瞄准不开火
./build/autoaim002_test --no-window            # 关闭可视化窗口（纯命令行）
```

运行时会弹出一个 **AutoAim Test** 窗口，实时显示：检测结果（绿色=轮廓、黄色=灯条框、
蓝色=装甲板框）、目标装甲板中心十字、左上角瞄准 HUD（当前云台角/目标瞄准角/距离）、
顶部工况信息（`cond X/6 … fired=N/50 locked=M`）。按 `q` 或 ESC 可提前结束当前工况。

输出示例：

```
========== 工况 1: spin-low  原地旋转 低速  满分=6 ==========
  set motion: mode=Spin ... -> Ok applied_seq=...
  stabilizing 3 s...
  [spin-low ...] frames=1777 locked=1776 rounds_fired=50/50 window=... s
  score_estimate=6.00 / 6.00  (命中率代理=开火数/50；实际命中需裁判系统)
===== 装甲板自瞄任务汇总 =====  total_score_estimate=... / 60
```

默认检测红色灯条（模拟器敌方装甲板）；如目标为蓝色用 `--color blue`。
命中真值由裁判系统提供，工具以 `rounds_fired/50 × 满分` 作得分估算。

## 6. 退出

在任意图像窗口按 `q` 或 ESC，或直接关闭主窗口。程序同时关闭 SDK 连接。

## 常见问题

- **could not connect**：模拟器未启动，或 IPC 目录不匹配。先执行第 3 步，确认
  `daedalus-contest status` 显示 running，并核对程序打印的 `IPC dir` 与脚本打印的
  `runtime_dir` 是否一致。
- **IPC 目录不一致**：程序打印的 `IPC dir` 和模拟器实际目录不同（例如脚本用的是
  `$XDG_RUNTIME_DIR` 而程序默认了 `/tmp`，或者用了 `--runtime-dir`/`--ipc-dir` 自定义），
  用 `--ipc-dir` 显式指定即可，例如
  `./build/autoaim002 --ipc-dir /run/user/1000/daedalus-contest-1000`。
- **窗口不显示 / 报错**：无图形环境时 `cv::imshow` 会失败，程序自动降级为“只打印信息、
  不显示窗口”，仍会继续读帧并打印信息（可配合 `--save-dir` 保存帧）。
- **detect: lights=0 / armors=0**：先确认装甲板颜色——模拟器敌方装甲板灯条为**红色**，
  默认 `--color red` 即正确；若目标实际为蓝色用 `--color blue`。远处的大装甲板两侧
  灯条渲染时会连成一块（宽长比超灯条约束），已由 `detect_armor_blobs` 回退路径当作
  单个装甲板整体识别；若仍为 0，用 `--det-debug` 查看候选与约束值。
