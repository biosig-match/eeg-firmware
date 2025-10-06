#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string.h>
#include <math.h>

// --- Configuration ---
#define NUM_CHANNELS 8
#define SAMPLE_RATE 256
#define TIMER_INTERVAL_US (1000000 / SAMPLE_RATE)
#define TRIGGER_INTERVAL_SAMPLES (SAMPLE_RATE * 2)
#define SAMPLES_PER_CHUNK 12

// --- BLE Configuration ---
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define DEVICE_NAME "EEG-Device"

// --- Data Structures ---
struct __attribute__((packed)) SampleData
{
    uint16_t signals[NUM_CHANNELS];
    int16_t accel[3];
    int16_t gyro[3];
    uint8_t trigger_state; // 0: OFF, 1-255: イベントIDなど
};

struct __attribute__((packed)) ChunkedSamplePacket
{
    uint8_t packet_type;     // パケット種別 (このパケットは 0x66)
    uint16_t start_index;    // このチャンクの開始サンプルインデックス
    uint8_t num_samples;     // このチャンクに含まれるサンプル数
    SampleData samples[SAMPLES_PER_CHUNK];
};

struct __attribute__((packed)) ElectrodeConfig {
    char name[8];
    uint8_t type;
    uint8_t reserved;
};

struct __attribute__((packed)) DeviceConfigPacket {
    uint8_t packet_type;    // パケット種別 (このパケットは 0xDD)
    uint8_t num_channels;
    uint8_t reserved[6];
    ElectrodeConfig configs[NUM_CHANNELS];
};

// --- Global Variables ---
Adafruit_MPU6050 mpu;
hw_timer_t *timer = nullptr;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool sampleFlag = false;
BLEServer *pServer = nullptr;
BLECharacteristic *pTxCharacteristic = nullptr;
bool deviceConnected = false;
volatile bool isStreaming = false;

uint32_t samplesSinceLastTrigger = 0;
uint32_t sampleIndexCounter = 0;

SampleData sampleBuffer[SAMPLES_PER_CHUNK];
volatile int bufferCounter = 0;

const ElectrodeConfig electrodeConfigs[NUM_CHANNELS] = {
    {"Fp1", 0, 0}, {"Fp2", 0, 0}, {"F7", 0, 0}, {"F8", 0, 0},
    {"T5", 0, 0}, {"T6", 0, 0}, {"O1", 0, 0}, {"O2", 0, 0}
};

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println(">>> デバイスが接続されました");
    }

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      isStreaming = false;
      BLEDevice::startAdvertising();
    }
};

class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            uint8_t command = value[0];
            if (command == 0xAA) { // ストリーミング開始コマンド
                isStreaming = true;
                sampleIndexCounter = 0;
                bufferCounter = 0;
                samplesSinceLastTrigger = 0;

                // デバイス設定情報を送信
                DeviceConfigPacket configPacket;
                configPacket.packet_type = 0xDD;
                configPacket.num_channels = NUM_CHANNELS;
                memcpy(configPacket.configs, electrodeConfigs, sizeof(electrodeConfigs));
                pTxCharacteristic->setValue((uint8_t*)&configPacket, sizeof(configPacket));
                pTxCharacteristic->notify();
                Serial.println("デバイス設定情報を送信しました");

            } else if (command == 0x5B) { // ストリーミング停止コマンド
                isStreaming = false;
                Serial.println("### ストリーミングを停止します ###");
            }
        }
    }
};


// --- ISR, Sensor Functions ---
void IRAM_ATTR onTimer()
{
    portENTER_CRITICAL_ISR(&timerMux);
    sampleFlag = true;
    portEXIT_CRITICAL_ISR(&timerMux);
}

void generate_high_quality_dummy_data(SampleData *data_ptr)
{
    float time = micros() / 1000000.0f;
    const float baseline = 32768.0f;

    for (int ch = 0; ch < NUM_CHANNELS; ch++)
    {
        float delta = 800.0f * sin(2.0 * PI * 2.0 * time + ch * 1.5);
        float theta = 1600.0f * sin(2.0 * PI * 6.0 * time + ch * 0.2);
        float alpha = 12800.0f * sin(2.0 * PI * 10.0 * time + ch * 0.5);
        float beta = 3200.0f * sin(2.0 * PI * 20.0 * time + ch * 0.3);
        float drift = 2400.0f * sin(2.0 * PI * 0.5 * time + ch * 0.8);
        float noise = 800.0f * (rand() % 100 - 50) / 50.0;
        float combined_signal = baseline + delta + theta + alpha + beta + drift + noise;
        if(combined_signal < 0) combined_signal = 0;
        if(combined_signal > 65535) combined_signal = 65535;
        data_ptr->signals[ch] = (uint16_t)combined_signal;
    }

    if (samplesSinceLastTrigger >= TRIGGER_INTERVAL_SAMPLES)
    {
        data_ptr->trigger_state = 1;
    } else {
        data_ptr->trigger_state = 0;
    }

    data_ptr->accel[0] = (int16_t)(sin(2.0 * PI * 0.3 * time) * 2000.0);
    data_ptr->accel[1] = (int16_t)(cos(2.0 * PI * 0.5 * time) * 2000.0);
    data_ptr->accel[2] = (int16_t)(-9.8 * 819.2 + sin(2.0 * PI * 0.2 * time) * 1000.0);
    data_ptr->gyro[0] = (int16_t)(sin(2.0 * PI * 1.5 * time) * 300.0);
    data_ptr->gyro[1] = (int16_t)(cos(2.0 * PI * 1.0 * time) * 300.0);
    data_ptr->gyro[2] = (int16_t)(sin(2.0 * PI * 2.0 * time) * 300.0);
}

// --- Setup & Loop ---
void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n--- EEG Device Booting (v4 - 16bit Precision & Dedicated Trigger) ---");

    // 1. BLEデバイスを初期化 (デバイス名を設定)
    BLEDevice::init(DEVICE_NAME);
    Serial.println("BLEデバイス初期化完了");

    // 2. BLEサーバーを作成
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    Serial.println("BLEサーバー作成完了");

    // 3. BLEサービスを作成
    BLEService *pService = pServer->createService(SERVICE_UUID);
    Serial.println("BLEサービス作成完了");

    // 4. TXキャラクタリスティックを作成 (Notify: サーバー→クライアント)
    pTxCharacteristic = pService->createCharacteristic(
                                CHARACTERISTIC_UUID_TX,
                                BLECharacteristic::PROPERTY_NOTIFY
                            );
    pTxCharacteristic->addDescriptor(new BLE2902());
    Serial.println("TXキャラクタリスティック作成完了");

    // 5. RXキャラクタリスティックを作成 (Write: クライアント→サーバー)
    BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
                                            CHARACTERISTIC_UUID_RX,
                                            BLECharacteristic::PROPERTY_WRITE
                                        );
    pRxCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
    Serial.println("RXキャラクタリスティック作成完了");

    // 6. サービスを開始
    pService->start();
    Serial.println("サービス開始");

    // 7. アドバタイズを開始
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    BLEDevice::startAdvertising();
    Serial.println("★★★ アドバタイズ開始: 他のデバイスから 'EEG-Device' が見えます ★★★");

    // タイマー設定
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, TIMER_INTERVAL_US, true);
    timerAlarmEnable(timer);
    Serial.println("サンプリングタイマー起動完了");
}


void loop()
{
    if (sampleFlag)
    {
        portENTER_CRITICAL(&timerMux);
        sampleFlag = false;
        portEXIT_CRITICAL(&timerMux);

        if (deviceConnected && isStreaming)
        {
            if (bufferCounter < SAMPLES_PER_CHUNK)
            {
                SampleData currentSample;
                generate_high_quality_dummy_data(&currentSample);

                if (currentSample.trigger_state == 1) {
                    samplesSinceLastTrigger = 0;
                }
                samplesSinceLastTrigger++;

                memcpy(&sampleBuffer[bufferCounter], &currentSample, sizeof(SampleData));
                bufferCounter++;
                sampleIndexCounter++;
            }

            if (bufferCounter >= SAMPLES_PER_CHUNK)
            {
                ChunkedSamplePacket packet;
                packet.packet_type = 0x66;
                packet.num_samples = SAMPLES_PER_CHUNK;
                packet.start_index = sampleIndexCounter - SAMPLES_PER_CHUNK;
                memcpy(packet.samples, sampleBuffer, sizeof(sampleBuffer));

                pTxCharacteristic->setValue((uint8_t*)&packet, sizeof(packet));
                pTxCharacteristic->notify();
                
                // Serial.printf("--- Sent BLE Chunk (Start Index: %u) ---\n", packet.start_index);
                
                bufferCounter = 0;
            }
        }
    }
}