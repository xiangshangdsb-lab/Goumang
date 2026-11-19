# C30D 固件源码

- `APP/control` 和 `APP/bluetooth` 实现句芒的底盘控制与板间命令。
- `APP/protocol`、`APP/line`、`APP/pan`、`APP/storage` 保存协议、巡线、角度和断电数据逻辑。
- `HARDWARE` 包含项目补充的 CCD、OLED 和 Flash 驱动。
- `gcc` 保存本项目的构建清单、链接脚本和 GCC 适配。应用限于 Flash 扇区 0–2，扇区 3 保存参数与云台位置。

## 构建说明

桌面仿真可直接从仓库根目录运行 `python source/S31/tests/sim/run_simulation.py`。

硬件联编需要单独取得轮趣 C30D STM32F407 底板工程。项目适配以 L150Pro C30D 霍尔编码器版为基线，使用其系统启动、标准外设库、FreeRTOS 端口与电机／编码器／IMU 驱动。将本仓库的路径对照 `gcc/Makefile` 与本地工程整合，保留 C30D GCC 启动文件、板级 FreeRTOS 初始化、GCC Cortex-M4F 移植和 OLED 字库等本地依赖。使用已有的句芒 C30D 完整工程时，可以在该工程顶部覆盖本仓库的对应源码路径，然后执行下面的构建命令。

```shell
make -f gcc/Makefile -j4
```

在 PATH 上安装 arm-none-eabi-gcc 和 GNU Make，或用 `TOOLCHAIN=/path/to/toolchain/bin` 指定目录。此公开树包含项目源码，单独检出可跑仿真。C30D 实板镜像要在补齐本地底板工程后构建。
