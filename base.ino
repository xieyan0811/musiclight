/*
 * 本程序通过采集音频信号并进行FFT变换，实现音乐节奏检测与灯光联动。
 *
 * 工作流程说明：
 * 1. 每次 loop() 循环时，先从模拟输入通道（CHANNEL）以设定的采样周期（sampling_period_us）采集一段音频数据。
 * 2. 对采集到的数据进行FFT变换，得到频域数据，幅值存储在 vReal 数组中。
 * 3. 将频域数据划分为7个频率段（分箱），分别计算每个频段的信号强度。
 * 4. 对每个频段的幅值进行去噪和放大处理，滤除底噪并平衡各频段信号。
 * 5. 若某个频段信号强度超过阈值，则认为检测到节拍（beat），并进行计数。
 * 6. 若本周期内检测到节拍，则触发灯光变化，实现音乐与灯光的联动响应。
 *
 * 主要目标：判断当前采样周期内的音频信号是否包含强拍，并通过灯光进行可视化反馈。
 * 
 * 当前问题：每116ms只“取”16ms，其余时间“闭耳不听”，所以有些节拍会漏掉。
 */

#include "arduinoFFT.h"
#include <Adafruit_NeoPixel.h>

// 模拟输入通道
#define CHANNEL A0
// FFT采样点数，必须是2的幂次方（为了节省Uno内存而设置为64）
const uint16_t samples = 64; // Must be a power of 2, reduced for Uno RAM
// 采样频率：4000Hz，适合音频信号处理
const double samplingFrequency = 4000; // Hz, suitable for audio signals (better than 9600 for music)
// 采样周期（微秒）
unsigned int sampling_period_us;
// 用于记录时间的微秒计数器
unsigned long microseconds;

// FFT实部数组
float vReal[samples];
// FFT虚部数组
float vImag[samples];

// 初始化FFT对象
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, samples, samplingFrequency);

// RGB灯带初始化
Adafruit_NeoPixel rgb_display_9(10); // 10个RGB灯珠，控制端口为9
// 灯色阶段：0=红, 1=绿, 2=蓝, 3=空闲（黑）
int lightStage = 3;
// 上次灯色变化时间
unsigned long lastLightChangeTime = 0;
// 灯色变化最小间隔（毫秒），防止切换太快
const unsigned long minLightChangeInterval = 300;

// 噪声阈值数组（用于不同频率段的噪声过滤）
int noise[] = {204, 198, 100, 85, 85, 80, 80, 80};
// 调整后的噪声因子数组
float noise_fact_adj[] = {15, 7, 1.5, 1, 1.2, 1.4, 1.7, 3};

// 节拍计数器
int beat = 0;
// 循环计数器
int counter = 0;

// 量级标记定义
#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02

void setup()
{
    // 初始化RGB灯带
    rgb_display_9.begin();
    rgb_display_9.setPin(9);

    // 计算采样周期（单位微秒）
    sampling_period_us = round(1000000 * (1.0 / samplingFrequency));
    // 初始化串口通信，波特率9600
    Serial.begin(9600);
    delay(1000);
    Serial.println("Baseline - Beat Detection FFT Started");
    Serial.println("Ready");
}

// 设置所有RGB灯珠的颜色
void setAllPixels(int r, int g, int b)
{
    for (int i = 0; i < 10; i++)
    {
        rgb_display_9.setPixelColor(i, r, g, b);
    }
    rgb_display_9.show();
}

void updateLED()
{
    unsigned long currentTime = millis();

    // 如果距离上次灯色变化时间太短，则忽略此次调用
    if (currentTime - lastLightChangeTime < minLightChangeInterval)
    {
        Serial.println("Light change ignored - too fast");
        return;
    }

    // 灯色循环变化：红 -> 绿 -> 蓝 -> 黑（空闲）
    if (lightStage == 3 || lightStage == 2)
    {
        setAllPixels(255, 0, 0); // 红灯
        lightStage = 0;
    }
    else if (lightStage == 0)
    {
        setAllPixels(0, 255, 0); // 绿灯
        lightStage = 1;
    }
    else if (lightStage == 1)
    {
        setAllPixels(0, 0, 255); // 蓝灯
        lightStage = 2;
    }
    else if (lightStage == 2)
    {
        setAllPixels(0, 0, 0); // 黑灯（空闲）
        lightStage = 3;
    }

    // 更新上次灯色变化时间
    lastLightChangeTime = currentTime;
}

void loop()
{
    // 记录开始采样时间
    microseconds = micros();
    // 采集FFT样本点
    for (int i = 0; i < samples; i++) // 250us * 64 = 16000us = 16ms
    {
        // 读取模拟输入
        vReal[i] = analogRead(CHANNEL);
        // 虚部初始化为0
        vImag[i] = 0;
        // 等待直到达到采样周期
        while (micros() - microseconds < sampling_period_us)
        {
            // empty loop for timing
        }
        // 更新时间戳
        microseconds += sampling_period_us;
    }

    // 应用汉宁窗口函数
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);

    // 计算FFT变换
    FFT.compute(FFTDirection::Forward);

    // 将复数转换为幅值
    FFT.complexToMagnitude();

    // 循环计数增加
    counter++;

    // 打印频率箱和检测节拍
    Serial.print("Magnitudes: ");
    for (int i = 1; i < 8; i++)
    {
        // 计算频率箱的索引
        int bin = i * (samples / 16);
        // 从原始幅值中减去噪声阈值
        int j = (int)vReal[bin] - noise[i];

        // 噪声过滤：如果幅值低于10，认为是噪声
        if (j < 10)
        {
            j = 0;
        }
        else
        {
            // 应用噪声因子放大处理后的信号
            j = j * noise_fact_adj[i];
            // 检测节拍：如果幅值超过阈值180，则计数增加
            if (j > 180)
            {
                // 高频段（i>=7）的节拍权重更高
                beat += (i >= 7) ? 2 : 1;
            }
            // 将结果量化为30的倍数
            j = (j / 30) * 30;
        }

        // 打印处理后的幅值
        Serial.print(j);
        Serial.print(" ");
    }

    // 打印节拍计数
    Serial.print("| Beat: ");
    Serial.println(beat);

    // 如果检测到拍子，调用亮灯函数
    if (beat > 0)
    {
        Serial.print("DEBUG: beat = ");
        Serial.println(beat);
        updateLED(); // 调用亮灯函数
    }

    // 打印主频率
    float dominantFreq = FFT.majorPeak();
    Serial.print("Dominant Freq: ");
    Serial.print(dominantFreq, 2);
    Serial.println(" Hz");

    // 复位节拍计数器
    beat = 0;
    // 延时100毫秒，避免串口输出过快
    delay(100); // Small delay to avoid overwhelming serial output
}
