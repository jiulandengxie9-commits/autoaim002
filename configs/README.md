# configs —— 运行时配置

各功能包的算法参数目前通过命令行（`--xxx`）传入，见各可执行程序的 `--help`。
本目录存放**固定的运行时数据**（相机标定等），供 `--calibration` 等参数引用。

## 文件

| 文件 | 说明 |
| --- | --- |
| `camera-calibration.json` | 模拟器发行版自带的相机标定（内参 + 云台→相机外参），PnP 位姿解算使用。 |

## 用法

程序默认从发行版 SDK 目录读取标定文件；若需要显式指定，可引用本目录副本：

```bash
./build/autoaim002 --calibration configs/camera-calibration.json
./build/autoaim002_test --calibration configs/camera-calibration.json
./build/pnp_curve_test --calibration configs/camera-calibration.json
```

> 标定文件为只读数据源（`read_only: true`），不要修改；如需自定义相机参数，
> 复制一份后通过 `--calibration` 传入即可。
