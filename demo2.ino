#include "arduinoFFT.h"
#include <Adafruit_NeoPixel.h>

#define CHANNEL A0
#define SAMPLES 128
#define SAMPLING_FREQUENCY 5000
#define NUM_LEDS 10
#define LED_PIN 9

const int sampleWindow = 50;
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

    double peak = FFT.majorPeak();

    unsigned long startMillis = millis();
    unsigned int peakToPeak = 0;
    unsigned int signalMax = 0;
    unsigned int signalMin = 1024;

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
    double volts = (peakToPeak * 5.0) / 1024;

    total = total - readings[readIndex];
    readings[readIndex] = peak;
    total = total + readings[readIndex];
    readIndex = readIndex + 1;

    if (readIndex >= numReadings)
    {
        readIndex = 0;
    }

    average = total / numReadings;

    int hueValue = map(average, 200, 1500, 0, 150);
    int brightnessValue = map(peakToPeak, 1, 400, 0, 255);

    if (brightnessValue > 180)
    {
        updateLED();
    }

    Serial.print("Average: ");
    Serial.print(average);
    Serial.print(" | Peak: ");
    Serial.print(peakToPeak);
    Serial.print(" | Hue: ");
    Serial.print(hueValue);
    Serial.print(" | Brightness: ");
    Serial.print(brightnessValue);
    Serial.print(" | Freq: ");
    Serial.print(peak, 2);
    Serial.println(" Hz");

    delay(100);
}