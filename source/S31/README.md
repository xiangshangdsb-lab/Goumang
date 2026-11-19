# S31 固件

硬件为 ESP32-S31-Korvo-1。使用支持 esp32s31 的 ESP-IDF 6.1 环境，把仓库放在简短的 ASCII 路径下。项目已保存 esp32s31 的 sdkconfig，组件管理器按 dependencies.lock 获取 ESP Component Registry 依赖。中文界面字形随源码提供。

在已加载 ESP-IDF 环境的终端运行。

```shell
cd source/S31
idf.py build
```

应用镜像为 `build/goumang_s31.bin`。完整构建会生成分区镜像。已有设备应按当前分区表和更新范围选择烧录内容。

## 生成离线语音

需要播报时，在 S31 目录使用装有 Microsoft Huihui Desktop 和 Python 3 的 Windows，先生成 WAV，再构建。

```powershell
powershell.exe -ExecutionPolicy Bypass -File tools/generate_voice.ps1
idf.py build
```

语音文本在 `voice/prompts.json` 及生成脚本中。没有 WAV 时仍可构建，语音播放需生成并烧录 voice 分区。

## 配置云端 AI

打开设备网页，用触屏「网络设置」显示的控制码授权。在 AI 设置填入自己的 DeepSeek API Key。Key 由设备写入 NVS。设备未设置 Key 时，触屏、网页控制、本地视觉和巡线可独立使用。
