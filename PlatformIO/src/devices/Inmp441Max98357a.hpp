#pragma once
#include <Arduino.h>
#include <device.h>

#include <driver/i2s.h>
#include "IndicatorLight.h"

// I2S pins for MIC
#define MIC_I2S_BCLK    17
#define MIC_I2S_FS      16
#define MIC_I2S_DIN     18
#define MIC_I2S_PORT    I2S_NUM_0
#define MIC_SAMPLE_RATE   (16000)

// I2S pins for speaker
#define SPK_I2S_FS      27
#define SPK_I2S_BCLK    14
#define SPK_I2S_DOUT    12
#define SPK_I2S_PORT    I2S_NUM_1
#define SPK_SAMPLE_RATE (22050)

#define I2S_SAMPLE_BITS   (16)
#define I2S_READ_LEN     512

// LEDs
#define LED_FLASH 4

class Inmp441Max98357a : public Device
{
  public:
    Inmp441Max98357a();
    void init();
    void updateColors(int colors);
    bool readAudio(uint8_t *data, size_t size);
    
    void setWriteMode(int sampleRate, int bitDepth, int numChannels);
    void writeAudio(uint8_t *data, size_t size, size_t *bytes_written);
    void setGain(uint16_t gain);

    IndicatorLight* indicator_light = new IndicatorLight(LED_FLASH);

    int numAmpOutConfigurations() { return 1; };
    void updateBrightness(int brightness);
    
  private:
    char* i2s_read_buff = (char*) calloc(I2S_READ_LEN, sizeof(char));
    uint16_t m_gain;
};


Inmp441Max98357a::Inmp441Max98357a() {};


void Inmp441Max98357a::init() {

  esp_err_t err = ESP_OK;
  Serial.printf("Connect to Inmp441... \n");

  //----- Speaker

  i2s_config_t i2s_config_spk = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = SPK_SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,  
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S ), 
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 2,
      .dma_buf_len = 512,
      .tx_desc_auto_clear = true,
  };

  i2s_pin_config_t pin_config_spk = {
      .bck_io_num = SPK_I2S_BCLK, 
      .ws_io_num = SPK_I2S_FS, 
      .data_out_num = SPK_I2S_DOUT, 
      .data_in_num = -1 
  };

  err += i2s_driver_install(SPK_I2S_PORT, &i2s_config_spk, 0, NULL);
  if (err != ESP_OK) {
      Serial.printf("Failed installing headphone driver: %d\n", err);
      while (true);
  }

  err += i2s_set_pin(SPK_I2S_PORT, &pin_config_spk);
  if (err != ESP_OK) {
    Serial.printf("Failed setting headphone pin: %d\n", err);
    while (true);
  }
  Serial.println("I2S headphone driver installed.\n");

  //----- Microphone

  /*
    - the INMP441 MEMS microphone has a specified sensitivity of -26 dBFS @ 1 Pa,
      or 120 dB SPL @ FS. 
    - Even a loud speech signal won't exceed 80 dB, so we "waste" the top 40 dB 
      of dynamic range. 
    - The microphone sends 24 bit samples (which is not to say it has a true 
       24 bit dynamic range). 
    - If we read 16 bit samples (nominal 96 dB dynamic range), and waste the top 
      40 dB, then we get a ~56 dB dynamic range speech signal.
    - Might bet better to read 24 bit samples, and amplify by 40 dB
  */
  i2s_config_t i2s_config_mic = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = MIC_SAMPLE_RATE,
    .bits_per_sample = i2s_bits_per_sample_t(I2S_SAMPLE_BITS),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S, 
    .intr_alloc_flags = 0,
    .dma_buf_count = 2,
    .dma_buf_len = 512,
    .use_apll = 1
  };

  i2s_pin_config_t pin_config_mic = {
    .bck_io_num = MIC_I2S_BCLK,
    .ws_io_num = MIC_I2S_FS,
    .data_out_num = -1,
    .data_in_num = MIC_I2S_DIN
  };

  err += i2s_driver_install(MIC_I2S_PORT, &i2s_config_mic, 0, NULL);
  if (err != ESP_OK) {
     Serial.printf("Failed installing mic driver: %d\n", err);
     while (true);
  }

  err += i2s_set_pin(MIC_I2S_PORT, &pin_config_mic);
  if (err != ESP_OK) {
    Serial.printf("Failed setting mic pin: %d\n", err);
    while (true);
  }
  Serial.println("I2S driver mic installed.");

  return;
}


void Inmp441Max98357a::updateColors(int colors)
{
    switch (colors)
    {
    case COLORS_HOTWORD:
        // very slow pulsing of LED
        indicator_light->setPulseTime(4000);
        indicator_light->setState(PULSING);
        break;
    case COLORS_WIFI_CONNECTED:
        // quick flashing of LED
        indicator_light->setDutyPercent(50);
        indicator_light->setPulseTime(1000);
        indicator_light->setState(BLINKING);
        break;
    case COLORS_WIFI_DISCONNECTED:
        // slower flashing of LED
        indicator_light->setDutyPercent(25);
        indicator_light->setPulseTime(2000);
        indicator_light->setState(BLINKING);
        break;    
    case COLORS_IDLE:
        // all lights are off
        indicator_light->setState(OFF);
        break;
    case COLORS_ERROR:
        // very quick blinking of LED
        indicator_light->setDutyPercent(25);
        indicator_light->setPulseTime(500);
        indicator_light->setState(BLINKING);
        break;
    case COLORS_OTA:
        // very quick pulsing of LED
        indicator_light->setPulseTime(500);
        indicator_light->setState(PULSING);
        break;
    case COLORS_TTS:
        indicator_light->setState(ON);
        break;

    }
};


void Inmp441Max98357a::setWriteMode(int sampleRate, int bitDepth, int numChannels)
{
    if (sampleRate > 0) {
        i2s_set_clk(SPK_I2S_PORT, sampleRate, static_cast<i2s_bits_per_sample_t>(bitDepth), static_cast<i2s_channel_t>(numChannels));
    }
}


void Inmp441Max98357a::writeAudio(uint8_t *data, size_t size, size_t *bytes_written) 
{
    i2s_write(SPK_I2S_PORT, data, size, bytes_written, portMAX_DELAY);
}


bool Inmp441Max98357a::readAudio(uint8_t *data, size_t size) 
{
    size_t bytes_read;
    i2s_read(MIC_I2S_PORT, (void*) i2s_read_buff, size, &bytes_read, portMAX_DELAY);
    
    int16_t* from = (int16_t*) i2s_read_buff;
    int16_t* to   = (int16_t*) data;
    size_t nsamples = bytes_read / sizeof(int16_t);
    size_t i;
    int32_t y;
    /*
      according to https://www.esp32.com/viewtopic.php?t=11023 
      incoming samples are in flipped order, i.e. 1,0,3,2,5,4,...
    */
    for (i=0; i<nsamples; i+=2) {
        y = from[1];
        y *= m_gain;
        *to++ = y;
        y = from[0];
        y *= m_gain;
        *to++ = y;
        from += 2;        
    }
    return true;
}


void Inmp441Max98357a::updateBrightness(int brightness)
{
    indicator_light->setMaxBrightness((brightness*indicator_light->limit)/100);
} 


void Inmp441Max98357a::setGain(uint16_t gain) 
{
    m_gain = gain;
}