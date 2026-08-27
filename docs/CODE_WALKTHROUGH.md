# 代码讲解（CODE WALKTHROUGH）

程序按功能拆分为几个包（详见 `ARCHITECTURE.md`）：`src/main.cpp`（主程序：SDK 读帧 +
显示/打印 + 调用功能包）与 `tasks/vision/vision_processing.{hpp,cpp}`（纯视觉功能包：
形态学滤波、灯条/装甲板识别、PnP 位姿估计与卡尔曼滤波，不依赖 SDK），另有
`tasks/detection/`（检测器抽象）与 `tasks/planning/`（弹道/提前量）。
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

`tasks/vision/vision_processing.hpp/.cpp` 只依赖 OpenCV，不含 SDK，便于独立测试与复用。
`main.cpp` 通过 `#include "tasks/vision/vision_processing.hpp"` 调用：

```cpp
// 1) 通道差阈值分割目标色灯条（red: B-R；blue: R-B）
cv::Mat mask = vision::splitColorMask(bgr, LightColor::Red, rb_threshold);
// 兼容旧接口：splitRedMask(bgr, thr) == splitColorMask(bgr, Red, thr)

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

### 大装甲板整体回退（detect_armor_blobs）

模拟器靶场的敌方**大装甲板**在稍远距离/侧视角下，两侧灯条会与装甲板底色连成一个
连通区域（实测为一个 122×76 px 的矩形，宽长比约 1.6），不满足灯条"竖条、宽/长<0.8"
约束。`detect()` 在灯条配对之后增加回退路径：

- 对每个轮廓取 `minAreaRect`，归一化使 `w ≤ h`，若 `min_blob_ratio ≤ w/h ≤
  max_blob_ratio`（默认 0.4~4.0）且面积 ≥ `min_blob_area`，视为单个装甲板。
- 若该轮廓中心已被灯条配对识别过（距离 < 0.5×高），跳过避免重复。
- 以矩形左右边缘构造两条合成灯条 + 四顶点 `ArmorDescriptor`，交给后续 PnP 解算。

`DetectionParams` 新增 `detect_armor_blobs / min_blob_area / min_blob_ratio /
max_blob_ratio` 控制。

`makeGrid(bgr, m, det, cell_width)` 中每个单元格会先统一为 BGR 三通道再放大/缩小到
指定宽度（默认 480px），文字字号随单元格宽度等比放大，因此用 `--grid-width` 调大后
调试文字更清晰。

## 10. PnP 三维位姿估计（`solveArmorPose`）

`tasks/vision/vision_processing.{hpp,cpp}` 新增的纯 OpenCV 世界坐标模块：

- `CameraIntrinsics`：`fx/fy/cx/cy` + 畸变系数（`camera-calibration.json` 内参）。
- `CameraExtrinsics`：云台→相机光学系的静态外参（平移 + 四元数 xyzw）。
- `TargetPose`：`valid`、相机系 `t_cam/r_cam`、距离 `distance_m`、云台系
  `t_gimbal`、装甲板四角点与左右灯条中心的三维坐标。

流程：

1. **标定加载**：`loadCalibration()` 读取发行版 `camera-calibration.json`。
   `readJsonDouble` / `readJsonArray` 是极简 JSON 提取器（文件结构固定，无需第三方库）。
2. **solvePnP（IPPE）**：`image_points` 取 `armor.vertex[0..3]`（左上/右上/右下/左下），
   `object_points` 用真实装甲板尺寸（`--armor-width × --armor-height`，默认
   0.135 × 0.056 m）构造的局部系平面角点，得到相机系 `rvec/tvec`。
3. **外参变换**：`cameraToGimbal()` 对 `p_cam` 求 `p_gimbal = Rᵀ·(p_cam − t)`，
   把装甲板中心与四角点变换到云台（世界）系。
4. **灯条三维**：左右灯条中心像素与装甲板平面（由 `solvePnP` 的位姿决定）做
   光线求交，得到其在相机系的三维坐标，再变换到云台系。
5. **输出**：`main.cpp` 的 `printPoses()` 打印 `pnp armor[i]: distance=… gimbal=(…)`
   与 `light[j]: gimbal=(…)`；显示/保存结果图时在装甲板旁叠加黄色距离与坐标标注。

坐标系约定：相机光学系为 OpenCV 约定（+Z 前、+X 右、+Y 下）；外参
`from_frame = gimbal, to_frame = camera_optical`，故云台系（世界系）坐标 =
`cameraToGimbal(相机系坐标)`。

## 11. 卡尔曼滤波（`KalmanFilter3D`）

`tasks/vision/vision_processing.{hpp,cpp}` 的 `KalmanFilter3D` 参考 `big_homework.cpp` 的分轴
卡尔曼（`A` 取位置/速度/加速度，`H = [1 0 0]`）与 `rmcs_auto_aim_v2` 的
predict/update 流程，合并为单个 9 维线性卡尔曼：

- 状态 `x`：`[x, y, z, vx, vy, vz, ax, ay, az]`。
- 状态转移 `A`：恒加速度模型，`x += v·dt + a·dt²/2`，`v += a·dt`；`dt` 由帧曝光
  时间戳差给出。
- 观测 `H`：只取位置三项；观测值 `z` = `solveArmorPose().t_gimbal`（云台系）。
- `init(pos, dt)`：用首个观测初始化状态、`Q/R`（由 `process_*_sigma` 与
  `measurement_sigma` 构造）与初始协方差 `P`。
- `predict(dt)`：`x_pred = A·x`，`P_pred = A·P·Aᵀ + Q`。
- `update(z)`：`K = P·Hᵀ·(H·P·Hᵀ+R)⁻¹`，`x += K·(z−H·x)`，`P = (I−K·H)·P`。

`main.cpp` 的 `KalmanTracker` 封装单目标跟踪：

1. 无目标时用 `computePoses()` 的最近装甲板 `init()`。
2. 每帧 `predict(dt)` 后，在 `poses` 中找离预测位置最近（门限 2 m）的观测
   `update()`；`lost_frames` 连续 > 60 则清空跟踪。
3. 打印 `kf: raw=… filtered=… vel=… lost=…`；用 `gimbalToCamera()` + `projectPoint()`
   把滤波位置投回图像画绿色十字（`KF` 标注）。

`gimbalToCamera` 是 `cameraToGimbal` 的逆（`p_cam = R·p_gimbal + t`）；
`projectPoint` 用针孔模型（带 plumb-bob 畸变，本发行版畸变全 0）。

## 12. 像素→世界坐标与瞄准角可视化

参考 `big_homework.cpp`（图像角点→PnP→世界系）与 `rmcs_auto_aim_v2` 的
`reproject_point`/`xyz2ypd`，`vision_processing` 补齐了双向坐标链：

```cpp
// 像素 (u,v) + 深度 → 相机光学系（带畸变逆变换）
cv::Vec3d p_cam = vision::pixelToCamera(uv, depth, intrinsics);
// 相机光学系 → 云台（世界）系
cv::Vec3d p_world = vision::cameraToGimbal(p_cam, extrinsics);
// 一步到位：像素 → 世界系
cv::Vec3d p_world = vision::pixelToWorld(uv, depth, intrinsics, extrinsics);
// 世界系 → 云台瞄准角（yaw/pitch，度；pitch=90 水平）
cv::Vec2d aim = vision::worldToAimAngles(p_world);
```

- `pixelToCamera`：由 `(u-cx)/fx、(v-cy)/fy` 归一化坐标乘深度得到三维点；若畸变
  非零则做 5 次迭代逆畸变（plumb-bob）。
- `worldToAimAngles`：`yaw = atan2(x, z)`，`pitch = 90° + atan2(y, √(x²+y²))`，
  与 SDK `UdpGimbalCommand.pitch_deg`（90=水平）一致。注意这是**云台系相对**角，
  会随云台转动而变。
- `absoluteAimAngles(p_cam, gimbal_yaw, gimbal_pitch, camera_tilt)`：**绝对**云台
  瞄准角 = 当前云台角 + 目标相对相机光轴的偏差（yaw 分量按 `cos(camera_tilt)`
  缩放、pitch 分量沿倾斜轴直接叠加）。目标在世界中不动时恒定（即使只转云台），
  这是真正要发给云台的命令角。HUD 的 `target yaw/pitch` 与终端 `kf:` 行的
  `aim_yaw/aim_pitch` 都输出该绝对角。
- `gimbalToWorld(p_gimbal, world_pose)`：云台系→绝对世界系（odom）。`world_pose`
  来自 `readGimbalWorldPose()`（封装 SDK `readExposureStateForFrame()` 返回的
  `gimbal_position_world` 与 wxyz 四元数），`p_world = R(q_world)·p_gimbal + t_world`。
  `main.cpp` 的 `printPoses()` 同时打印 `gimbal=` 与 `world=`，便于区分"随云台转的
  相对坐标"与"绝对世界坐标"。

### 旋转链（相机→云台→世界，参考 `tongji/tasks/auto_aim/solver.cpp`）

`tongji` 的 `Solver::solve()` 不仅变换位置，还把 `solvePnP` 得到的**旋转**沿同一链
变换并提取欧拉角。本程序在 `vision_processing` 补齐：

```cpp
// R_armor2gimbal = R_ext^T * R_armor2camera        （相机系旋转向量 → 云台系）
cv::Vec3d r_gimbal = vision::rotationCameraToGimbal(pose.r_cam, extrinsics);
// R_armor2world   = R_gimbal2world * R_armor2gimbal（云台系旋转向量 → 世界系）
cv::Vec3d r_world  = vision::rotationGimbalToWorld(r_gimbal, world_pose);
// ZYX 欧拉角（yaw/pitch/roll，rad），同 tongji tools::eulers(R, 2, 1, 0)
cv::Vec3d ypr = vision::rotationMatrixToYpr(R_armor2world);
// 直角坐标 → 球坐标 (yaw, pitch, distance)
cv::Vec3d ypd = vision::xyzToYpd(world_xyz);
```

- `solveArmorPose()` 在内部计算 `r_gimbal` 存入 `TargetPose`。
- `printPoses()` 输出 `ypr_gimbal=(yaw,pitch,roll) deg`（恒有）与
  `ypr_world=(…) deg`、`ypd_world=(yaw,pitch,distance)`（有世界位姿时）。
- `rotationMatrixToYpr` 采用 ZYX 内旋（先绕 z 后绕 y 后绕 x）：`yaw=atan2(R10,R00)`、
  `pitch=atan2(-R20,hypot(R21,R22))`、`roll=atan2(R21,R22)`。
- `xyzToYpd`：`yaw=atan2(y,x)`、`pitch=atan2(z,hypot(x,y))`、`distance=norm`。
- `drawAimHud(bgr, gimbal_yaw, gimbal_pitch, has_target, target_aim, dist)`：
  - 左上角 HUD 文本：`gimbal yaw/pitch`（来自曝光同步 `cf.gimbal`）。
  - 有目标时：`target yaw/pitch dist` + 绿色圆环/连线标记目标相对云台的角度偏差
    （每度约 8 像素），无目标时显示 `target: none`。
  - 画面中央画十字线 + 蓝色水平参考线。
- `main.cpp`：目标角取卡尔曼滤波位置（`tracker.filtered`），无跟踪时取最近装甲板
  PnP 坐标；主窗口与保存图都叠加该 HUD。

## 扩展建议

- 控制云台：构造 `UdpGimbalCommand{ yaw_deg, pitch_deg }` 调用 `sim.sendAim()`；
  用 `sendTracked()` 拿到 `command_id`，再通过 `readGimbalStateForFrame()` 核对曝光时角度。
- 切换大符场景：`setRuneScenario()`，`RuneMotion::RuleDriven` 或 `Static`。
- 帧率优化：`nextFrame()` 返回最新完整帧，慢消费者不会看到半帧，可自行加入
  drop-frame 策略。
