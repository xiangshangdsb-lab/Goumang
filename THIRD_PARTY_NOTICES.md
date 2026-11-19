# 第三方说明

- `source/S31/components/esp32_s31_korvo_1/` 基于 Espressif BSP，按目录内的 Apache-2.0 许可和原版权声明分发。
- `source/S31/main/perception/app_video.c` 与 `app_video.h` 保留原 CC0-1.0 声明。
- `source/S31/main/interface/panel/fonts/td_font_16.c` 与 `td_font_24.c` 为 Noto Sans SC 2.002 的字形子集，保留 [SIL Open Font License 1.1](source/S31/main/interface/panel/fonts/OFL.txt)。
- ESP-IDF 组件管理器可在构建时下载各自授权的依赖，包括 LVGL、相机驱动和端侧模型。版本和来源见 [依赖锁文件](source/S31/dependencies.lock)。
- 轮趣底板 SDK、手册及照片依原出版方条款另取。Windows 语音合成器及本机生成的 WAV 资源通过用户环境提供。
