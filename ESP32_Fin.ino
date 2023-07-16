#include <WiFi.h>
#include <driver/i2s.h>
#include <soc/syscon_struct.h>
#include "util.h"
#include <vector>
using namespace std;

// #define DEBUG

// I2S
#define I2S_SAMPLE_RATE (8000) // Max sampling frequency = 277.777 kHz
#define ADC_INPUT (ADC1_CHANNEL_4) //pin 32s
#define I2S_DMA_BUF_LEN (512)
#define MAX_WAIT_BUF_LEN (24000)  // Max wait time 1.5s for wifi to connect

// timeStamp for recording time elapsed
#define TIMEOUT (10000)
uint32_t timeStamp;

// Use WiFiClient class to create TCP connections
WiFiClient client;
const char * host = "192.168.0.162"; 
const uint16_t port = 2333;  // the port for service
WIFIinfo mWifi;
TaskHandle_t wifiHandler;

// buffering data before wifi initiated
vector<uint8_t>* growBuffer;
bool bufferFailed = false;

// i2s Initialization
// Using dual channel format since it does not have swapping behavior
// Use fixed 1M MCLK to get correct sampling rate
void i2sInit(){
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate =  I2S_SAMPLE_RATE,              // The format of the signal using ADC_BUILT_IN
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // is fixed at 12bit, stereo, Most Significant Bit first
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,  
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = I2S_DMA_BUF_LEN,
    .use_apll = true,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 1000000
  };
  #ifdef DEBUG
  Serial.printf("Attempting to setup I2S ADC with sampling frequency %d Hz\n", I2S_SAMPLE_RATE);
  #endif
  if(ESP_OK != i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL)){
    #ifdef DEBUG
    Serial.printf("Error installing I2S. Halt!");
    #endif
    while(1);  // reset instead!
  }
  if(ESP_OK != i2s_set_adc_mode(ADC_UNIT_1, ADC_INPUT)){
    #ifdef DEBUG
    Serial.printf("Error setting up ADC. Halt!");
    #endif
    while(1);
  }
  if(ESP_OK != adc1_config_channel_atten(ADC_INPUT, ADC_ATTEN_DB_11)){
    #ifdef DEBUG        
    Serial.printf("Error setting up ADC attenuation. Halt!");
    #endif
    while(1);
  }
  // SYSCON.saradc_ctrl2.meas_num_limit = 0;
  if(ESP_OK != i2s_adc_enable(I2S_NUM_0)){
    #ifdef DEBUG
    Serial.printf("Error enabling ADC. Halt!");
    #endif
    while(1);
  }
  #ifdef DEBUG  
  Serial.printf("I2S ADC setup ok\n");
  #endif
}

// fast join the network with hardcoded information!!
void wifiInit(){
  while(WiFi.status() != WL_CONNECTED){
    WiFi.mode(WIFI_STA);
    WiFi.config(*mWifi.local_IP, *mWifi.gateway, *mWifi.subnet);
    WiFi.begin(mWifi.ssid, mWifi.pwd, mWifi.channel, mWifi.bssid, true);

    #ifdef DEBUG
    Serial.println("Waiting for WiFi... ");
    #endif  

    // 500ms attempt to connect otherwise restart!
    uint32_t timeout = millis() + 500;
    while(WiFi.status() != WL_CONNECTED && millis() < timeout) {
        delay(10);
    }

    #ifdef DEBUG
    if(WiFi.status() == WL_CONNECTED){
      Serial.println("");
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
    }
    else{
      Serial.println("Join network failed, restarting...");
    }
    #endif
  }
}
// check host connection status
// if not, attempt to reconnect
void wifiHostReconnect(){
  if(!client.connected() && WiFi.status() == WL_CONNECTED){
    // then we try to connect with our host
    #ifdef DEBUG
    Serial.print("Connecting to ");
    Serial.println(host);
    #endif

    while(!client.connect(host, port)) {
        #ifdef DEBUG
        Serial.println("Connecting to host failed.");
        Serial.println("Waiting 1 sec before retrying...");
        #endif
        delay(1000);
    }
    #ifdef DEBUG
    Serial.println("connected!");
    #endif
  }
}
// code for core 0 to run
// you can either run this in an infinite loop
// or call vTaskDelete(NULL) after it's done the job
void codeForCore0(void * parameter)
{
  for(;;){
    wifiInit();
    wifiHostReconnect();
    // vTaskDelete(NULL);
  }
}

void setup()
{
  #ifdef DEBUG
  Serial.begin(115200);
  delay(100);
  #endif
  
  // init I2S and ADC  
  i2sInit();
  // setup wakeup
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 1); //1 = High, 0 = Low
  // Kickout a thread for wifi init
  // tskIDLE_PRIORITY=0, idle task won't trigger watchdog
  xTaskCreatePinnedToCore(codeForCore0, "Wifi_init", 5000, NULL, tskIDLE_PRIORITY,  &wifiHandler, 0);
  growBuffer = new vector<uint8_t>();
  timeStamp = millis();

  #ifdef DEBUG
  Serial.println("entering loop!");
  #endif
}


void loop()
{  
  // // work for some time then go to sleep
  if(millis() - timeStamp < TIMEOUT) {
    size_t bytes_read;
    uint16_t buffer[I2S_DMA_BUF_LEN] = {0};

    i2s_read(I2S_NUM_0, &buffer, sizeof(buffer), &bytes_read, 15);
    
    // wifiHostReconnect();
    uint8_t alt_buffer[bytes_read / 2] = {0};  // 2 repeatitions
    for(int i = 0; i < bytes_read / 2; i=i+2){
      // alter bytes
      alt_buffer[i] = (buffer[i] >> 8) & 0x0f;  // first 4 bits not making sense
      alt_buffer[i + 1] = buffer[i] & 0xff;
    }

    if(client.connected()){
      if(growBuffer->size() > 0){
        // vector to uint8_t array
        uint8_t* waitBuffer = new uint8_t[MAX_WAIT_BUF_LEN];
        size_t waitSize;
        vector2array(waitBuffer, &waitSize, growBuffer);
        growBuffer->clear();
        client.write(waitBuffer, waitSize);
        free(waitBuffer);
      }
      client.write(alt_buffer, bytes_read / 2);
    }
    else{
      // buffering data in vector when data does not exceed the limit
      if(!bufferFailed) append2vector(alt_buffer, sizeof(alt_buffer), growBuffer);
      if(growBuffer->size() > MAX_WAIT_BUF_LEN){
        Serial.println("data buffering timeout!");
        growBuffer->clear();
        bufferFailed = true;
      }
    }

    // TODOs: input detection
    // we detect on host, here only receive sleep cmd
  }
  else{
    #ifdef DEBUG
    Serial.println("disconnecting..");
    #endif
    client.stop();
    delay(100);
    esp_deep_sleep_start();
  }
}

