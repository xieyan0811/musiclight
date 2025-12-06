/*
 * 音乐律动彩灯 - 频域定颜色，时域定节奏
 * 
 * 实现思路：
 * 1. 每次 loop 连续采样音频数据并做 FFT，得到频域能量分布。
 * 2. 将频域划分为 3 个频段（低频/中频/高频），分别对应红/绿/蓝颜色。
 * 3. 找出当前能量最高的频段，决定灯光颜色。
 * 4. 同时在时域检测音量突变（节拍），只有检测到节拍时才切换灯光颜色。
 * 5. 这样既能根据音乐风格显示不同颜色，又能跟着节奏律动。
 */

#include "arduinoFFT.h"
#include "Adafruit_NeoPixel.h"

// ========== 硬件配置 ==========
#define CHANNEL A0           // 音频输入通道
#define LED_PIN 9            // RGB灯带控制引脚
#define LED_COUNT 10         // 灯珠数量
#define MAGNET_PIN_1 10      // 电磁铁1控制引脚
#define MAGNET_PIN_2 11      // 电磁铁2控制引脚

// ========== FFT 参数 ==========
const uint16_t samples = 64;              // 采样点数（2的幂）
const double samplingFrequency = 4000;    // 采样频率 4kHz
unsigned int sampling_period_us;          // 采样周期（微秒）

float vReal[samples];
float vImag[samples];
float vRawAudio[samples];  // 存储 FFT 前的原始音频数据
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, samples, samplingFrequency);

// ========== RGB 灯带 ==========
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// ========== 节拍检测参数 ==========
const int beatThreshold = 80;              // 节拍检测阈值（音量波动）- 降低以提高灵敏度
unsigned long lastBeatTime = 0;           // 上次节拍时间
const unsigned long minBeatInterval = 300; // 最小节拍间隔（ms），两次打拍子的最小时间间隔
const unsigned long lightDuration = 150;   // 灯光持续时间（ms），节拍后亮灯的时长
const unsigned long magnetDuration = 200; // 磁铁持续时间（ms），节拍后吸引磁铁的时长
const unsigned long magnetCooldown = 500;  // 磁铁冷却时间（ms），两次吸引之间的最小间隔
bool lightOn = false;              // 当前灯光状态
bool magnetOn1 = false;            // 电磁铁1状态
bool magnetOn2 = false;            // 电磁铁2状态
unsigned long lastMagnet1Time = 0; // 电磁铁1上次启动时间
unsigned long lastMagnet2Time = 0; // 电磁铁2上次启动时间
const int magnetPower = 200;

// ========== 频段能量存储 ==========
float lowEnergy = 0;     // 低频能量（红色）
float midEnergy = 0;     // 中频能量（绿色）
float highEnergy = 0;    // 高频能量（蓝色）

// ========== 噪声降低和加权参数（参考 base.ino） ==========
// 各频段的噪声阈值，用于滤除无声时的低频底噪
const float noiseFloor[] = {3000, 0, 0};  // [低频, 中频, 高频]
// 各频段的加权因子，用于平衡不同频段的强度差异
const float bandWeights[] = {1.0, 2.0, 3.5};   // [低频, 中频, 高频] - 高频加权更多
// 处理后能量的最小阈值，低于此值认为无音乐信号
const float minEnergyThreshold = 50;

void setup()
{
    Serial.begin(9600);
    
    // 初始化 RGB 灯带
    strip.begin();
    strip.show(); // 初始化为全灭
    strip.setBrightness(100); // 设置亮度（0-255）
    
    // 计算采样周期
    sampling_period_us = round(1000000 * (1.0 / samplingFrequency));
    
    Serial.println("Music Rhythm Light - Started");
    Serial.println("Frequency -> Color, Beat -> Trigger");
}

// 设置所有灯珠为指定颜色
void setAllPixels(uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < LED_COUNT; i++)
    {
        strip.setPixelColor(i, strip.Color(r, g, b));
    }
    strip.show();
}

// 采样并计算 FFT
void sampleAndFFT()
{
    unsigned long microseconds = micros();
    
    // 采样
    for (int i = 0; i < samples; i++)
    {
        vRawAudio[i] = analogRead(CHANNEL);  // 保存原始音频数据
        vReal[i] = vRawAudio[i];             // 复制到 vReal 供 FFT 使用
        vImag[i] = 0;
        
        while (micros() - microseconds < sampling_period_us)
        {
            // 等待采样周期
        }
        microseconds += sampling_period_us;
    }
    
    // FFT 计算
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();
}

// 计算三个频段的能量
void calculateFrequencyBands()
{
    // 重置能量
    lowEnergy = 0;
    midEnergy = 0;
    highEnergy = 0;
    
    // 分频段累加能量
    // 低频：bin 1-8（约 62-500 Hz，鼓、贝斯）
    for (int i = 1; i <= 8; i++)
    {
        lowEnergy += vReal[i];
    }
    
    // 中频：bin 9-20（约 560-1250 Hz，人声、吉他）
    for (int i = 9; i <= 20; i++)
    {
        midEnergy += vReal[i];
    }
    
    // 高频：bin 21-31（约 1310-1940 Hz，镲片、高音）
    for (int i = 21; i < samples / 2; i++)
    {
        highEnergy += vReal[i];
    }
    
    // ========== 降噪处理：减去噪声阈值 ==========
    lowEnergy = (lowEnergy > noiseFloor[0]) ? (lowEnergy - noiseFloor[0]) : 0;
    midEnergy = (midEnergy > noiseFloor[1]) ? (midEnergy - noiseFloor[1]) : 0;
    highEnergy = (highEnergy > noiseFloor[2]) ? (highEnergy - noiseFloor[2]) : 0;
    
    // ========== 加权处理：使高频更容易被检测 ==========
    lowEnergy *= bandWeights[0];
    midEnergy *= bandWeights[1];
    highEnergy *= bandWeights[2];
    
    // 打印调试信息（含处理后的能量值）
    Serial.print("Low: ");
    Serial.print(lowEnergy);
    Serial.print(" | Mid: ");
    Serial.print(midEnergy);
    Serial.print(" | High: ");
    Serial.println(highEnergy);
}


/*
// 根据频段能量决定颜色
void getColorFromFrequency(uint8_t &r, uint8_t &g, uint8_t &b)
{
    // 找出能量最大的频段
    if (lowEnergy > midEnergy && lowEnergy > highEnergy)
    {
        // 低频 -> 红色
        r = 255; g = 0; b = 0;
        Serial.println("Color: RED (Low Freq)");
    }
    else if (midEnergy > lowEnergy && midEnergy > highEnergy)
    {
        // 中频 -> 绿色（人声、吉他）
        r = 0; g = 255; b = 0;
        Serial.println("Color: GREEN (Mid Freq - Voice/Guitar)");
    }
    else if (highEnergy > lowEnergy && highEnergy > midEnergy)
    {
        // 高频 -> 蓝色（女声、高音）
        r = 0; g = 0; b = 255;
        Serial.println("Color: BLUE (High Freq - Female Voice/Treble)");
    }
    else
    {
        // 默认：当所有能量都很低时，保持黑色或上一个颜色
        r = 0; g = 0; b = 0;
        Serial.println("Color: BLACK (No signal)");
    }
}*/

// 根据频段能量决定颜色
void getColorFromFrequency(uint8_t &r, uint8_t &g, uint8_t &b)
{
    // 找出能量最大的频段
    if (lowEnergy > midEnergy && lowEnergy > highEnergy)
    {
        // 低频 -> 白色
        r = 255; g = 255; b = 255;
        Serial.println("Color: RED (Low Freq)");
    }
    else if (midEnergy > lowEnergy && midEnergy > highEnergy)
    {
        // 中频 -> 黄色（人声、吉他）
        r = 255; g = 215; b = 0;
        Serial.println("Color: GREEN (Mid Freq - Voice/Guitar)");
    }
    else if (highEnergy > lowEnergy && highEnergy > midEnergy)
    {
        // 高频 -> 粉色（女声、高音）
        r = 255; g = 0; b = 255;
        Serial.println("Color: BLUE (High Freq - Female Voice/Treble)");
    }
    else
    {
        // 默认：当所有能量都很低时，保持黑色或上一个颜色
        r = 0; g = 0; b = 0;
        Serial.println("Color: BLACK (No signal)");
    }
}

// 检测节拍（基于原始音频数据的 peakToPeak）
bool detectBeat()
{
    // 使用原始音频数据计算峰值到谷值的差
    float signalMax = 0;
    float signalMin = 1024;
    
    for (int i = 0; i < samples; i++)
    {
        if (vRawAudio[i] > signalMax)
        {
            signalMax = vRawAudio[i];
        }
        if (vRawAudio[i] < signalMin)
        {
            signalMin = vRawAudio[i];
        }
    }
    float peakToPeak = signalMax - signalMin;
    
    // 检查是否超过阈值且间隔足够
    unsigned long currentTime = millis();
    /*
    Serial.print("signalMax: ");
    Serial.print(signalMax);
    Serial.print(" | signalMin: ");
    Serial.print(signalMin);
    Serial.print(" | PeakToPeak: ");
    Serial.println(peakToPeak);
    Serial.print("beatThreshold: ");
    Serial.println(beatThreshold);
    */

    if (peakToPeak > beatThreshold && 
        (currentTime - lastBeatTime) > minBeatInterval)
    {
        lastBeatTime = currentTime;
        Serial.print("@@@@@@@@@@@@@@@@@@@@ BEAT detected! PeakToPeak: ");
        Serial.println(peakToPeak);
        return true;
    }
    
    return false;
}

// 控制电磁铁1（10号引脚）
void magnetOn1_control(bool on)
{
    if (on)
    {
        // 检查冷却时间（从上次关闭到现在的时间间隔）
        unsigned long currentTime = millis();
        if ((currentTime - lastMagnet1Time) < magnetCooldown)
        {
            // 间隔不足，扔掉这个 on 命令
            Serial.println(">>> Magnet 1 ON rejected - cooldown not ready <<<");
            return;
        }
        
        analogWrite(MAGNET_PIN_1, magnetPower);
        magnetOn1 = true;
        Serial.println(">>> Magnet 1 ON <<<");
    }
    else
    {
        analogWrite(MAGNET_PIN_1, 0);
        magnetOn1 = false;
        lastMagnet1Time = millis();  // 记录关闭时间
        Serial.println(">>> Magnet 1 OFF <<<");
    }
}

// 控制电磁铁2（11号引脚）
void magnetOn2_control(bool on)
{
    if (on)
    {
        // 检查冷却时间（从上次关闭到现在的时间间隔）
        unsigned long currentTime = millis();
        if ((currentTime - lastMagnet2Time) < magnetCooldown)
        {
            // 间隔不足，扔掉这个 on 命令
            Serial.println(">>> Magnet 2 ON rejected - cooldown not ready <<<");
            return;
        }
        
        analogWrite(MAGNET_PIN_2, magnetPower);
        magnetOn2 = true;
        Serial.println(">>> Magnet 2 ON <<<");
    }
    else
    {
        analogWrite(MAGNET_PIN_2, 0);
        magnetOn2 = false;
        lastMagnet2Time = millis();  // 记录关闭时间
        Serial.println(">>> Magnet 2 OFF <<<");
    }
}

void loop()
{
    // 检查是否有电磁铁工作，如果有则跳过采样和节拍检测（防止干扰）
    if (magnetOn1 || magnetOn2)
    {
        // 只处理磁铁的关闭逻辑，不进行采样和节拍检测
        if ((millis() - lastBeatTime) > magnetDuration)
        {
            magnetOn1_control(false);  // 关闭电磁铁1
            magnetOn2_control(false);  // 关闭电磁铁2
            Serial.println(">>> Magnets OFF after duration <<<");
        }
        delay(50);
        return;  // 跳过本次 loop 的其他处理
    }
    
    // 1. 采样并做 FFT
    sampleAndFFT();
    
    // 2. 计算频段能量
    calculateFrequencyBands();
    
    // 3. 根据频段决定颜色
    uint8_t r, g, b;
    getColorFromFrequency(r, g, b);
    
    // 4. 检测节拍
    bool beatDetected = detectBeat();
    
    // 5. 检测到节拍时点亮灯光和启动磁铁
    if (beatDetected)
    {
        setAllPixels(r, g, b);
        lightOn = true;

        if (highEnergy > lowEnergy && highEnergy > midEnergy)
        {
            // 高频 -> 头和手都动
            magnetOn1_control(true);   // 启动电磁铁1（头）
            magnetOn2_control(true);   // 启动电磁铁2（手）
            Serial.println(">>> Light ON + Magnet 1&2 ON (High Freq)! <<<");
        }
        else if (lowEnergy > midEnergy && lowEnergy > highEnergy)
        {
            // 低频 -> 只动手
            //magnetOn1_control(false);  // 关闭电磁铁1（头） xieyan
            magnetOn1_control(true);
            magnetOn2_control(true);   // 启动电磁铁2（手）
            Serial.println(">>> Light ON + Magnet 2 ON (Low Freq)! <<<");
        }
        else
        {
            // 中频 -> 头和手都动
            magnetOn1_control(true);   // 启动电磁铁1（头）
            magnetOn2_control(true);   // 启动电磁铁2（手）
            Serial.println(">>> Light ON + Magnet 1&2 ON (Mid Freq)! <<<");
        }
    }
    
    // 6. 检查是否需要熄灭灯光（节拍后经过 lightDuration）
    if (lightOn && (millis() - lastBeatTime) > lightDuration)
    {
        setAllPixels(0, 0, 0); // 熄灭所有灯
        lightOn = false;
        Serial.println(">>> Light OFF <<<");
    }
    
    // 小延时，避免处理过快
    delay(50);
    //delay(300);
}
