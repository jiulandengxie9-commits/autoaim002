# 代码讲解（CODE WALKTHROUGH）

程序由两部分组成：`src/main.cpp`（主程序：SDK 读帧 + 显示/打印 + 调用视觉模块）与
`src/vision_processing.{hpp,cpp}`（纯视觉处理模块：形态学滤波，不依赖 SDK）。
下面按功能拆解它做了什么、为什么这么做。

## 0. 整体流程

```
main()
 ├─ parseArgs()          解析命令行参数（含形态学开关与参数）
 ├─ ContestClient 连接    TCP 图像 + UDP 云台 + UDP 场景控制
 ├─ health() / runtime capabilities / scene / metadata   启动时一次性信息
 └─ 主循环 while(true)
     ├─ nextFrame()      阻塞等到一帧新图像（含曝光同步云台姿态）
     ├─ 打印图像与云台信息
     ├─ frameToBgr()      RGBA32/RGB24 -> OpenCV BGR Mat
     ├─ vision::splitRedMask + applyMorphology   形态学滤波（--no-morph 可关）
     ├─ imshow() + waitKey()   显示结果图与 Morphology Stages 网格
     └─ 可选 imwrite() 保存 PNG / max_frames 退出
```

## 1. 头文件与命名空间

```cpp
#include <daedalus_sim_sdk/contest_client.hpp>        // 竞赛首选入口
#include <daedalus_sim_sdk/runtime_capabilities.hpp>  // 读取 GPU/后端能力
#include <daedalus_sim_sdk/talos_metadata_reader.hpp> // 共享内存元数据读取
#include <opencv2/core.hpp> <highgui.hpp> <imgproc.hpp>
using namespace daedalus::sim::sdk::v1;
```

SDK 只支持 C++17，所有公开接口在命名空间 `daedalus::sim::sdk::v1` 中。

## 2. 参数解析（`parseArgs`）

支持 `--ipc-dir`、`--scene`、`--max-frames`、`--save-dir`、`--window`、`--help`。
IPC 目录默认取 `$XDG_RUNTIME_DIR`（未设置时回退 `/tmp`）下的 `daedalus-contest-<uid>`，
与发行版 `daedalus-contest.sh` 的默认运行目录一致。

## 3. 连接模拟器

```cpp
ContestClientOptions opts;
opts.ipc_directory = o.ipc_directory;   // 关键：告诉 SDK 去哪里找共享内存/套接字
ContestClient sim(opts);
if (!sim.connect()) { ... }
```

`ContestClient` 把三类传输合成一个对象：

| 传输 | 端点 | 用途 |
| --- | --- | --- |
| TCP 图像 | `127.0.0.1:5602` | 最新完整相机帧（latest-only） |
| UDP 云台 | `127.0.0.1:5601` | 下发 yaw/pitch 命令 |
| UDP 场景控制 | `127.0.0.1:5603` | 切换靶场/能量机关、查询大符得分 |

`ipc_directory` 里放着 `talos_ipc_meta`（共享内存元数据）、
`talos_ipc_image_pool`（旧 SHM 图像池）和 `daedalus-runtime-capabilities-v1.json`。

## 4. 启动时一次性信息

- `sim.health()` → `RuntimeCapabilities`（schema 版本、产品版本、渲染后端、GPU 名、
  驱动）。
- `readRuntimeCapabilities(ipc_directory)` → 从 JSON 文件读取实际选中的 wgpu 适配器。
- `sim.selectScene(ContestScene)` → 切换场景，返回 `SceneControlResponse`
  （`command_id`、`applied_frame_seq`、`status`）。竞赛版只接受
  `ShootingRange` 与 `Energy`（`LargeEnergy == Energy`）。
- `printMetadata()` → 用 `TalosMetadataMapping` 打开 `talos_ipc_meta`，读出：
  - `ShmHeader`：魔数、SHM 版本、ABI 版本、图像尺寸（用于兼容性检查）。
  - `CameraInfo`：固定标定内参 `fx/fy/cx/cy`。
  - `GimbalState`：最新实际云台角（yaw/pitch）。
  - `ChassisObservation`：底盘线速度/角速度。

## 5. 主循环：读帧

```cpp
auto frame_res = sim.nextFrame(last_seq);
```

`nextFrame(after_source_sequence)` 会阻塞直到出现 `source_sequence` 更大的完整帧。
返回 `ContestFrame`：

```cpp
struct ContestFrame {
  TcpImageFrame image;   // header + 原始像素 payload
  GimbalState   gimbal;  // 与 image 同一曝光的实际云台姿态
};
```

图像头结构见 `tcp_image::FrameHeader`（魔数、版本、格式、宽高、载荷字节、生产者纪元、
源序号、时间戳）。云台与图像由同一 `source_sequence` 同步，这是自瞄计算角度的正确数据源。

## 6. 图像转换（`frameToBgr`）

```cpp
cv::Mat rgba(h.height, h.width, CV_8UC4, payload.data());
cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);
```

`frameToBgr` 用一个不复制数据的 Mat 包装 `payload` 指针，再 `cvtColor` 复制转换为 BGR
（OpenCV `imshow`/`imwrite` 需要 BGR）。`header.format` 是权威格式，代码同时支持
RGBA32 与 RGB24。

## 7. 显示与终端输出

```cpp
cv::imshow(o.window_title, image);
int key = cv::waitKey(1);
if (key == 'q' || key == 27) break;
if (cv::getWindowProperty(title, cv::WND_PROP_VISIBLE) < 1) break;
```

- 每帧打印：序号、生产者纪元、时间戳、宽高、格式、载荷、帧率、云台 yaw/pitch、
  角速度、最后应用命令号。
- 能量机关场景额外打印 `printBigRuneScore()`：红/蓝两侧 `run_id`、`run_active`、
  `arms`、`avg_ring`、`last_ring`、`last_radius_mm`、`last_target`。
- `cv::imshow` 在无图形环境会抛异常，代码捕获后把 `show_window` 置为 `false`，
  降级为纯信息打印（仍可配合 `--save-dir` 保存帧）。

## 8. 退出与资源释放

```cpp
sim.close();
```

主循环退出条件：按 `q`/ESC、关闭窗口、达到 `--max-frames`。退出后调用
`ContestClient::close()` 关闭套接字与共享内存映射。

## 9. 纯视觉处理模块（vision_processing）

`src/vision_processing.hpp/.cpp` 只依赖 OpenCV，不含 SDK，便于独立测试与复用。
`main.cpp` 通过 `#include "vision_processing.hpp"` 调用：

```cpp
// 1) R-B 通道差阈值分割红色灯条
cv::Mat mask = vision::splitRedMask(bgr, rb_threshold);

// 2) 形态学滤波（腐蚀/膨胀/开/闭/级联 + 面积滤除）
vision::MorphologyParams params;
params.kernel_size = kernel_size;          // 结构元素尺寸（椭圆核）
params.rb_threshold = rb_threshold;
params.max_contour_area = max_contour_area;  // >0 时滤除面积超限的轮廓
vision::MorphologyResult m = vision::applyMorphology(mask, params);
// m.eroded / m.dilated / m.opened / m.closed / m.final_mask / m.contours
// m.contours_filtered：被面积滤除掉的轮廓数

// 3) 灯条与装甲板识别（在滤波后轮廓上执行）
vision::DetectionResult det = vision::detect(m.contours, det_params);
// det.lights：LightDescriptor 列表（宽/长/中心/角度/面积）
// det.armors：ArmorDescriptor 列表（左右灯条 + 四顶点外接框）

// 4) 绘制结果：绿色=轮廓，黄色=灯条外接矩形，蓝色=装甲板框
cv::Mat result = vision::drawResult(bgr, m, det);

// 5) 拼出 8 阶段对比网格（原图/阈值/腐蚀/膨胀/开/闭/最终/结果）
cv::Mat grid = vision::makeGrid(bgr, m, det, cell_width);
```

`applyMorphology` 内部依次执行：

| 步骤 | 调用 | 作用 |
| --- | --- | --- |
| 腐蚀 | `cv::erode` | 收缩前景，去孤立噪点 |
| 膨胀 | `cv::dilate` | 扩大前景，补孔洞/断缝 |
| 开运算 | `cv::morphologyEx(MORPH_OPEN)` | 先腐蚀后膨胀，去小噪点 |
| 闭运算 | `cv::morphologyEx(MORPH_CLOSE)` | 先膨胀后腐蚀，连邻近区域 |
| 级联 | 开结果再做闭运算 | 得到最终掩膜 |
| 轮廓 | `cv::findContours` | 提取外轮廓 |
| 面积滤除 | `cv::contourArea` 比较 | `area > max_contour_area` 的轮廓剔除 |

面积滤除在 `findContours` 之后逐轮廓执行：超过 `params.max_contour_area` 的轮廓跳过，
`contours_filtered` 记录剔除个数（`--max-area N` 对应此参数，0 表示不过滤）。

### detect()：灯条识别

`detect` 内部的灯条部分参考 `big_homework.cpp` 的 `findlight()`：

1. 逐轮廓检查：`contourArea > min_light_area` 且点数 ≥ 5。
2. `cv::minAreaRect` 取最小外接旋转矩形，`adjustLightRect` 归一化——保证宽 ≤ 长、
   角度落在 `[-45°, 45°]`，得到 `LightDescriptor{width, length, center, angle, area}`。
3. 凸度检查：`solidity = 轮廓面积 / 凸包面积 >= min_contour_solidity`，滤掉形状破碎的
   区域。
4. 长宽比检查：`width / length <= max_light_ratio`，滤掉横置/偏方的红色块。

### detect()：装甲板配对

参考 `big_homework.cpp` 的 `matchArmor()`。灯条按 `center.x` 升序排列后两两配对，
同时满足以下条件才构成装甲板（任一不满足即跳过）：

| 条件 | 表达式 | 默认 |
| --- | --- | --- |
| 角度差 | `|a1-a2| <= max_angle_diff` | 15° |
| 长度差比率 | `|len1-len2|/max <= max_height_diff_ratio` | 0.2 |
| y 差比率 | `|y1-y2|/meanLen <= max_y_diff_ratio` | 0.5 |
| x 差比率 | `|x1-x2|/meanLen >= min_x_diff_ratio` | 0.5 |
| 装甲比例 | `dis/meanLen` 落在 `[min_armor_ratio, max_armor_ratio]` | 1.0~2.5 |

通过的配对以 `(center1+center2)/2` 为中心、两灯条平均角度为倾角、`dis` 为宽、
`meanLen` 为高构造 `RotatedRect`，取四点并 `adjustVertexOrder` 归一为
左上/右上/右下/左下顺序，即装甲板外接矩形框。

`drawResult` 绘制顺序：轮廓（绿）→ 灯条旋转矩形（黄）→ 装甲板四顶点框（蓝）。

`makeGrid(bgr, m, det, cell_width)` 中每个单元格会先统一为 BGR 三通道再放大/缩小到
指定宽度（默认 480px），文字字号随单元格宽度等比放大，因此用 `--grid-width` 调大后
调试文字更清晰。

## 扩展建议

- 控制云台：构造 `UdpGimbalCommand{ yaw_deg, pitch_deg }` 调用 `sim.sendAim()`；
  用 `sendTracked()` 拿到 `command_id`，再通过 `readGimbalStateForFrame()` 核对曝光时角度。
- 切换大符场景：`setRuneScenario()`，`RuneMotion::RuleDriven` 或 `Static`。
- 帧率优化：`nextFrame()` 返回最新完整帧，慢消费者不会看到半帧，可自行加入
  drop-frame 策略。
