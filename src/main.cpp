#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "zstd.h"
#include <string.h>
#include <math.h>

// --- 設定 ---
#define NUM_EEG_CHANNELS 8
#define USE_DUMMY_DATA 1
const int EEG_PINS[NUM_EEG_CHANNELS] = {A0, A1, A2, A3, A4, A5, 7, 8};
#define MPU1_AD0_PIN 3
#define MPU2_AD0_PIN 4
#define SAMPLE_RATE 256
#define SAMPLES_PER_PACKET (SAMPLE_RATE / 2)
#define TIMER_INTERVAL_US (1000000 / SAMPLE_RATE)
#define TRIGGER_INTERVAL_SAMPLES (SAMPLE_RATE * 2)

// --- BLE設定 ---
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

// --- データ構造 ---
struct __attribute__((packed)) PacketHeader
{
    char deviceId[18];
};
struct __attribute__((packed)) SensorData
{
    uint16_t eeg[NUM_EEG_CHANNELS];
    float accel[3];
    float gyro[3];
    uint8_t trigger;
    int8_t impedance[NUM_EEG_CHANNELS];
    uint32_t timestamp_us;
};

// --- グローバル変数 ---
Adafruit_MPU6050 mpu;
hw_timer_t *timer = nullptr;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool sampleFlag = false;
BLEServer *pServer = nullptr;
BLECharacteristic *pTxCharacteristic = nullptr;
bool deviceConnected = false;
volatile bool canSendData = true;
PacketHeader packetHeader;
SensorData sensorDataBuffer[SAMPLES_PER_PACKET];
volatile int sampleCounter = 0;
const size_t RAW_PAYLOAD_SIZE = sizeof(sensorDataBuffer);
const size_t TOTAL_RAW_SIZE = sizeof(PacketHeader) + RAW_PAYLOAD_SIZE;
uint32_t samplesSinceLastTrigger = 0;
uint8_t *compressedBuffer = nullptr;
ZSTD_CCtx *cctx = NULL;

// --- BLEコールバック ---
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        // ★★★ログフォーマット変更★★★
        Serial.println("➡️  ACK received. Ready for next packet.");
        canSendData = true;
    }
};
class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        deviceConnected = true;
        canSendData = true;
        // ★★★ログフォーマット変更★★★
        Serial.println("✅ Client Connected!");
    }
    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = false;
        // ★★★ログフォーマット変更★★★
        Serial.println("❌ Client Disconnected. Restarting advertising...");
        pServer->getAdvertising()->start();
    }
};

// (ISR, MPU切り替え, データ生成は変更なし)
void IRAM_ATTR onTimer()
{
    portENTER_CRITICAL_ISR(&timerMux);
    if (sampleCounter < SAMPLES_PER_PACKET)
    {
        sampleFlag = true;
    }
    portEXIT_CRITICAL_ISR(&timerMux);
}
void switchMPU(bool selectMPU1)
{
    digitalWrite(MPU1_AD0_PIN, selectMPU1 ? LOW : HIGH);
    digitalWrite(MPU2_AD0_PIN, selectMPU1 ? HIGH : LOW);
    delayMicroseconds(100);
}
void generate_dummy_sensor_data(SensorData *data_ptr)
{
    data_ptr->timestamp_us = micros();
    float time = data_ptr->timestamp_us / 1000000.0f;
    for (int ch = 0; ch < NUM_EEG_CHANNELS; ch++)
    {
        data_ptr->eeg[ch] = (uint16_t)(2048 + 150.0 * sin(2.0 * PI * 10.0 * time + ch));
    }
    data_ptr->accel[0] = sin(2.0 * PI * 2.0 * time);
    data_ptr->accel[1] = cos(2.0 * PI * 2.0 * time);
    data_ptr->accel[2] = -1.0 * sin(2.0 * PI * 2.0 * time);
    data_ptr->gyro[0] = sin(2.0 * PI * 5.0 * time) * 10.0;
    data_ptr->gyro[1] = cos(2.0 * PI * 5.0 * time) * 10.0;
    data_ptr->gyro[2] = sin(2.0 * PI * 5.0 * time) * -10.0;
    for (int ch = 0; ch < NUM_EEG_CHANNELS; ch++)
    {
        data_ptr->impedance[ch] = (int8_t)(ch + 1);
    }
}

void sendData()
{
    canSendData = false;
    // ★★★ログフォーマット変更★★★
    Serial.printf("Compressing %d samples (%u bytes)...\n", SAMPLES_PER_PACKET, (unsigned int)TOTAL_RAW_SIZE);

    uint8_t *rawBuffer = new uint8_t[TOTAL_RAW_SIZE];
    if (rawBuffer == NULL)
    {
        Serial.println("Error: Failed to allocate rawBuffer for compression!");
        canSendData = true;
        return;
    }
    memcpy(rawBuffer, &packetHeader, sizeof(PacketHeader));
    memcpy(rawBuffer + sizeof(PacketHeader), sensorDataBuffer, RAW_PAYLOAD_SIZE);

    size_t const compressedSize = ZSTD_compress2(cctx, compressedBuffer, ZSTD_compressBound(TOTAL_RAW_SIZE), rawBuffer, TOTAL_RAW_SIZE);

    delete[] rawBuffer;

    if (ZSTD_isError(compressedSize))
    {
        Serial.printf("Compression failed: %s\n", ZSTD_getErrorName(compressedSize));
        canSendData = true;
        return;
    }

    // ★★★ログフォーマット変更★★★
    Serial.printf("Compression OK: %u bytes -> %u bytes (%.1f%%)\n",
                  (unsigned int)TOTAL_RAW_SIZE, (unsigned int)compressedSize,
                  (float)compressedSize / TOTAL_RAW_SIZE * 100.0);

    uint32_t header = (uint32_t)compressedSize;
    size_t totalSize = sizeof(header) + compressedSize;
    uint8_t *sendBuffer = new uint8_t[totalSize];
    if (sendBuffer != NULL)
    {
        memcpy(sendBuffer, &header, sizeof(header));
        memcpy(sendBuffer + sizeof(header), compressedBuffer, compressedSize);
        const int max_chunk_size = 500;
        size_t bytes_sent = 0;
        while (bytes_sent < totalSize)
        {
            size_t chunk_size = totalSize - bytes_sent;
            if (chunk_size > max_chunk_size)
            {
                chunk_size = max_chunk_size;
            }
            pTxCharacteristic->setValue(sendBuffer + bytes_sent, chunk_size);
            pTxCharacteristic->notify();
            bytes_sent += chunk_size;
        }
        delete[] sendBuffer;
        sampleCounter = 0;
    }
    else
    {
        Serial.println("Error: Failed to allocate sendBuffer!");
        canSendData = true;
    }
}

void setup()
{
    Serial.begin(115200);
    delay(2000);
    // ★★★ログフォーマット変更★★★
    Serial.println("\n--- EEG Device Booting ---");

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(packetHeader.deviceId, sizeof(packetHeader.deviceId), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    char bleName[32];
    snprintf(bleName, sizeof(bleName), "EEG-Device-%02X%02X", mac[4], mac[5]);
    Serial.printf("Device ID (MAC): %s\n", packetHeader.deviceId);

    BLEDevice::init(bleName);
    Serial.printf("BLE Advertising as: %s\n", bleName);

    // 圧縮関連の初期化
    size_t const compressedBufferSize = ZSTD_compressBound(TOTAL_RAW_SIZE);
    compressedBuffer = new uint8_t[compressedBufferSize];
    if (compressedBuffer == NULL)
    {
        Serial.println("FATAL: Failed to allocate compressedBuffer");
        while (1)
            ;
    }
    cctx = ZSTD_createCCtx();
    if (cctx == NULL)
    {
        Serial.println("FATAL: Failed to create ZSTD_CCtx");
        while (1)
            ;
    }

    // BLEサービスの初期化
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
    pTxCharacteristic->addDescriptor(new BLE2902());
    BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    pRxCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
    pService->start();

    // アドバタイジング開始
    pServer->getAdvertising()->addServiceUUID(SERVICE_UUID);
    pServer->startAdvertising();
    Serial.println("BLE advertising started. Waiting for a client...");

    // タイマー割り込みの開始
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, TIMER_INTERVAL_US, true);
    timerAlarmEnable(timer);

    // ★★★ログフォーマット変更★★★
    Serial.println("-------------------------");
    Serial.println("✅ System Ready.");
}

void loop()
{
    if (sampleFlag)
    {
        portENTER_CRITICAL(&timerMux);
        sampleFlag = false;
        portEXIT_CRITICAL(&timerMux);

        if (sampleCounter < SAMPLES_PER_PACKET)
        {
            if (samplesSinceLastTrigger >= TRIGGER_INTERVAL_SAMPLES)
            {
                sensorDataBuffer[sampleCounter].trigger = 1;
                samplesSinceLastTrigger = 0;
            }
            else
            {
                sensorDataBuffer[sampleCounter].trigger = 0;
            }
            samplesSinceLastTrigger++;
            generate_dummy_sensor_data(&sensorDataBuffer[sampleCounter]);
            sampleCounter++;
        }
    }

    if (sampleCounter >= SAMPLES_PER_PACKET)
    {
        if (deviceConnected && canSendData)
        {
            sendData();
        }
        else if (!deviceConnected)
        {
            sampleCounter = 0;
            // ★★★ログフォーマット変更★★★
            Serial.println("🔹 Packet full. No client connected, discarding data.");
        }
    }
}
