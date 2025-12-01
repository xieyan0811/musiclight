/*
 * 本程序实现了基于音频信号的灯光控制，以下为功能说明与讨论总结：
 *
 * 1. 主要流程：
 *    - 每次 loop() 先采集一段音频数据，对其做 FFT 变换，得到主频（peak），但主频仅用于串口输出参考，未参与实际控制。
 *    - 随后在 sampleWindow（单位：毫秒，例50ms）内采集原始音频信号，记录最大值和最小值，计算音量波动（peakToPeak）。
 *    - 通过 map() 函数将 peakToPeak 映射为亮度值 brightnessValue，若大于180则触发灯光变化。
 *    - 代码中还实现了主频的滑动平均（average），但同样仅用于输出参考。
 *
 * 2. 采样与控制逻辑：
 *    - FFT 相关计算结果（主频）未参与灯光控制，实际控制只依赖音量波动（peakToPeak）。
 *    - 采样窗口外有 delay(100)，导致每次 loop 只分析很短的音频片段（如16ms），其余时间为“盲区”，可能漏检节拍。
 *    - 若需更精确的节拍检测或频率响应，可考虑用 FFT 结果的幅值或能量参与判断。
 *
 * 3. 结论：
 *    - 当前代码结构下，FFT 仅作演示和参考，实际灯光响应只与音量波动相关。
 *    - 若只需音量响应，可省略 FFT 部分，简化程序。
 *    - 若需频率响应或节奏检测，建议充分利用 FFT 结果。
 */

#include "arduinoFFT.h"
#include <Adafruit_NeoPixel.h>

#define CHANNEL A0
#define SAMPLES 128
#define SAMPLING_FREQUENCY 5000
#define NUM_LEDS 10
#define LED_PIN 9

const int sampleWindow = 50; // 50ms
unsigned int sample;

const int numReadings = 10;
int readings[numReadings];
int readIndex = 0;
int total = 0;
int average = 0;

unsigned int sampling_period_us;
unsigned long microseconds;

float vReal[SAMPLES];
float vImag[SAMPLES];

ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

Adafruit_NeoPixel rgb_display(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
int lightStage = 3;
unsigned long lastLightChangeTime = 0;
const unsigned long minLightChangeInterval = 300;

void setup()
{
    Serial.begin(9600);
    delay(1000);
    Serial.println("Demo2 - Beat Detection FFT Started");

    rgb_display.begin();
    sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
    randomSeed(98155);

    for (int thisReading = 0; thisReading < numReadings; thisReading++)
    {
        readings[thisReading] = 0;
    }
}

void setAllPixels(int r, int g, int b)
{
    for (int i = 0; i < NUM_LEDS; i++)
    {
        rgb_display.setPixelColor(i, r, g, b);
    }
    rgb_display.show();
}

void updateLED()
{
    unsigned long currentTime = millis();

    if (currentTime - lastLightChangeTime < minLightChangeInterval)
    {
        return;
    }

    if (lightStage == 3 || lightStage == 2)
    {
        setAllPixels(255, 0, 0);
        lightStage = 0;
    }
    else if (lightStage == 0)
    {
        setAllPixels(0, 255, 0);
        lightStage = 1;
    }
    else if (lightStage == 1)
    {
        setAllPixels(0, 0, 255);
        lightStage = 2;
    }
    else if (lightStage == 2)
    {
        setAllPixels(0, 0, 0);
        lightStage = 3;
    }

    lastLightChangeTime = currentTime;
}

void loop()
{
    microseconds = micros();
    for (int i = 0; i < SAMPLES; i++)
    {
        vReal[i] = analogRead(CHANNEL);
        vImag[i] = 0;

        while (micros() - microseconds < sampling_period_us)
        {
        }
        microseconds += sampling_period_us;
    }

    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();

    double peak = FFT.majorPeak(); // 经过傅立叶变换后，幅值最大的那个频率分量的频率值

    unsigned long startMillis = millis();
    unsigned int peakToPeak = 0;
    unsigned int signalMax = 0; // 最大音量
    unsigned int signalMin = 1024; // 最小音量(模拟信号范围0-1023)

    while (millis() - startMillis < sampleWindow)
    {
        sample = analogRead(A0);
        if (sample < 1024)
        {
            if (sample > signalMax)
            {
                signalMax = sample;
            }
            else if (sample < signalMin)
            {
                signalMin = sample;
            }
        }
    }
    peakToPeak = signalMax - signalMin;
    // 移动平均滤波，readings数组存储最近numReadings次的峰值
    total = total - readings[readIndex];
    readings[readIndex] = peak;
    total = total + readings[readIndex];
    readIndex = readIndex + 1;

    if (readIndex >= numReadings)
    {
        readIndex = 0;
    }

    average = total / numReadings; // 计算了，但没用上

    int brightnessValue = map(peakToPeak, 1, 400, 0, 255);

    if (brightnessValue > 180)
    {
        updateLED();
    }

    Serial.print("Average: ");
    Serial.print(average);
    Serial.print(" | Peak: ");
    Serial.print(peakToPeak);
    Serial.print(" | Brightness: ");
    Serial.print(brightnessValue);
    Serial.print(" | Freq: ");
    Serial.print(peak, 2);
    Serial.println(" Hz");

    delay(100);
}