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
装甲板框），**Morphology Stages** 窗口显示形态学滤波的 8 个阶段（原图/阈值/腐蚀/膨胀/
开/闭/最终掩膜/结果）。终端持续打印图像、云台与滤波/检测统计信息：

```
Daedalus SDK 1.3.1  IPC dir: /run/user/1000/daedalus-contest-1000
connected to simulator.
simulator health: schema=1 product=1.3.1-contest backend=vulkan adapter=...
scene 'shooting-range' selected: status=Ok command_id=1 applied_frame_seq=...
metadata: magic=0x54414c05 version=7 abi_rev=2 image=1440x1080
camera: fx=1303.68 fy=1303.68 cx=720 cy=540 1440x1080
morph: enabled kernel=5 rb_threshold=30 max_area=0 grid_width=480 det(...)

===== frame 1 =====  (60.0 fps)
image: seq=42 producer_epoch=1 timestamp_ns=123456789
       1440x1080 format=RGBA32 payload=6220800 bytes
gimbal: yaw=0.0 deg pitch=90.0 deg yaw_vel=0.0 deg/s pitch_vel=0.0 deg/s last_cmd=0
morph: kernel=5 rb_threshold=30 mask_before=12983 mask_after=12960 noise_removed=0.2% contours=1
detect: lights=2 armors=1
```

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

完整参数见 `./build/autoaim002 --help`。

## 5. 退出

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
