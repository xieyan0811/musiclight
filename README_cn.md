# 音乐律动灯

Arduino音乐律动灯项目，通过FFT音频分析实现RGB灯带随音乐节拍变化。

## 功能特点

- 音频信号采集与FFT频谱分析
- 自动节拍检测
- RGB灯带颜色循环变化（红→绿→蓝→黑）
- 噪声过滤与信号增强

## 硬件要求

- Arduino开发板（如Arduino Uno）
- 麦克风模块（连接到A0引脚）
- WS2812B RGB灯带（10个灯珠，连接到9号引脚）

## 依赖库

- [arduinoFFT](https://github.com/kosme/arduinoFFT) - FFT音频分析库
- [Adafruit_NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel) - RGB灯带控制库

## 文件说明

- `base.ino` - 基础版本，包含详细的节拍检测和频率分析
- `demo2.ino` - 简化版本，使用峰值检测和平滑处理

## 使用方法

1. 在Arduino IDE中安装所需依赖库
2. 连接硬件：麦克风→A0，RGB灯带→Pin 9
3. 上传代码到Arduino开发板
4. 播放音乐，灯带将随节拍闪烁变色

## 参数调整

可根据实际效果调整以下参数：
- 采样频率：`samplingFrequency`（4000-5000 Hz）
- 节拍阈值：代码中的`180`（灵敏度调节）
- 灯色切换间隔：`minLightChangeInterval`（300ms）

## 许可证

本项目采用开源许可证。
