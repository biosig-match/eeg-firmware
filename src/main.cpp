#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string.h>
#include <math.h>

// ========= ADS1299 実装と互換の設定 =========
#define DEVICE_NAME "ADS1299_EEG_NUS"
#define CH_MAX 8
#define SAMPLE_RATE_HZ 250
#define SAMPLES_PER_CHUNK 25 // 250SPS / 10Hz = 25

// ========= BLE (NUS-like) UUIDs (ADS1299実装と同一) =========
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E" // Notify
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E" // Write

// ========= パケット種別 (ADS1299実装と同一) =========
#define PKT_TYPE_DATA_CHUNK 0x66
#define PKT_TYPE_DEVICE_CFG 0xDD

// ========= 制御コマンド (ADS1299実装と同一) =========
#define CMD_START_STREAMING 0xAA
#define CMD_STOP_STREAMING 0x5B

// ========= データ構造 (ADS1299実装と同一) =========
struct __attribute__((packed)) ElectrodeConfig
{
    char name[8];
    uint8_t type;
    uint8_t reserved;
};

// IMUなし、符号付き16bit信号に修正
struct __attribute__((packed)) SampleData
{
    int16_t signals[CH_MAX]; // 符号付き16bit, little-endian
    uint8_t trigger_state;   // GPIO下位4bitを模倣 (0..15)
    uint8_t reserved[3];     // 予約領域
};

struct __attribute__((packed)) ChunkedSamplePacket
{
    uint8_t packet_type;  // 0x66
    uint16_t start_index; // LE
    uint8_t num_samples;  // 25
    SampleData samples[SAMPLES_PER_CHUNK];
};

struct __attribute__((packed)) DeviceConfigPacket
{
    uint8_t packet_type;  // 0xDD
    uint8_t num_channels; // 実使用ch数（今回は8ch固定のダミー）
    uint8_t reserved[6];
    ElectrodeConfig configs[CH_MAX];
};

// ========= グローバル変数 =========
BLEServer *pServer = nullptr;
BLECharacteristic *pTxCharacteristic = nullptr;

bool deviceConnected = false;
volatile bool isStreaming = false; // volatile: 割り込みから変更されるため

// 送信パケットをグローバルに確保 (スタックオーバーフロー防止)
ChunkedSamplePacket chunkPacket;
DeviceConfigPacket deviceConfigPacket;

// BLEコールバックからメインループへ処理を依頼するためのフラグ
volatile bool g_send_config_packet = false;

// サンプリング用タイマー
hw_timer_t *timer = nullptr;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool sampleReady = false;

// データバッファとカウンタ
SampleData sampleBuffer[SAMPLES_PER_CHUNK];
volatile int sampleBufferIndex = 0;
uint16_t sampleIndexCounter = 0;

// ADS1299互換の電極設定
const ElectrodeConfig defaultElectrodes[CH_MAX] = {
    {"CH1", 0, 0}, {"CH2", 0, 0}, {"CH3", 0, 0}, {"CH4", 0, 0}, {"CH5", 0, 0}, {"CH6", 0, 0}, {"CH7", 0, 0}, {"CH8", 0, 0}};

// ========= BLEコールバック (ADS1299実装と同一) =========
class ServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *s) override
    {
        deviceConnected = true;
        Serial.println(">>> [BLE] Client connected");
    }
    void onDisconnect(BLEServer *s) override
    {
        deviceConnected = false;
        isStreaming = false;
        BLEDevice::startAdvertising();
        Serial.println(">>> [BLE] Client DISCONNECTED. Streaming stopped. Advertising restarted.");
    }
};

class RxCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *ch) override
    {
        std::string v = ch->getValue();
        if (v.empty())
            return;
        uint8_t cmd = static_cast<uint8_t>(v[0]);

        if (cmd == CMD_START_STREAMING)
        {
            isStreaming = true;
            sampleIndexCounter = 0;
            sampleBufferIndex = 0;
            g_send_config_packet = true; // メインループに設定情報送信を依頼
        }
        else if (cmd == CMD_STOP_STREAMING)
        {
            isStreaming = false;
            Serial.println("[CMD] Stop streaming");
        }
    }
};

// ========= タイマー割り込み処理 =========
void IRAM_ATTR onTimer()
{
    portENTER_CRITICAL_ISR(&timerMux);
    sampleReady = true;
    portEXIT_CRITICAL_ISR(&timerMux);
}

// ========= ダミーデータ生成 (ADS1299互換) =========
void generateDummyAds1299Sample(SampleData &outSample)
{
    float time = (float)micros() / 1000000.0f;

    // 信号を int16_t (-32768 to 32767) の範囲で生成
    for (int ch = 0; ch < CH_MAX; ch++)
    {
        float alpha = 8000.0f * sin(2.0 * PI * 10.0 * time + ch * 0.5);
        float beta = 2000.0f * sin(2.0 * PI * 20.0 * time + ch * 0.3);
        float noise = 500.0f * (rand() % 100 - 50) / 50.0;
        float combined_signal = alpha + beta + noise;

        // 範囲内にクリッピング
        if (combined_signal < -32767.0)
            combined_signal = -32767.0;
        if (combined_signal > 32767.0)
            combined_signal = 32767.0;
        outSample.signals[ch] = (int16_t)combined_signal;
    }

    // 1秒ごとにトリガー状態が変化するダミー (0-15の範囲)
    outSample.trigger_state = (millis() / 250) % 16;

    // 予約領域を0で埋める
    memset(outSample.reserved, 0, sizeof(outSample.reserved));
}

// ========= Setup =========
void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n--- ADS1299-Compatible Dummy Data Streamer ---");

    // BLEデバイス初期化
    BLEDevice::init(DEVICE_NAME);
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // TX Characteristic (Notify)
    pTxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
    pTxCharacteristic->addDescriptor(new BLE2902());

    // RX Characteristic (Write)
    BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
    pRxCharacteristic->setCallbacks(new RxCallbacks());

    pService->start();
    BLEAdvertising *adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(SERVICE_UUID);
    adv->setScanResponse(true);
    BLEDevice::startAdvertising();
    Serial.println("BLE advertising started (ADS1299-NUS compatible)");

    // サンプリング用タイマー設定
    const int timer_id = 0;
    const uint32_t prescaler = 80; // 80MHz / 80 = 1MHz
    const uint64_t alarm_value = 1000000 / SAMPLE_RATE_HZ;
    timer = timerBegin(timer_id, prescaler, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, alarm_value, true);
    timerAlarmEnable(timer);
    Serial.printf("Sampling timer started for %d Hz\n", SAMPLE_RATE_HZ);
}

// ========= Loop =========
void loop()
{
    // --- [1] BLEコールバックからの設定情報送信要求を処理 ---
    if (g_send_config_packet)
    {
        g_send_config_packet = false;
        if (deviceConnected)
        {
            deviceConfigPacket.packet_type = PKT_TYPE_DEVICE_CFG;
            deviceConfigPacket.num_channels = CH_MAX; // 8chのダミーデバイスとして通知
            memset(deviceConfigPacket.reserved, 0, sizeof(deviceConfigPacket.reserved));
            memcpy(deviceConfigPacket.configs, defaultElectrodes, sizeof(defaultElectrodes));

            pTxCharacteristic->setValue((uint8_t *)&deviceConfigPacket, sizeof(deviceConfigPacket));
            pTxCharacteristic->notify();

            Serial.println("[CMD] Start streaming -> Sent DeviceConfigPacket");
            delay(10); // 送信処理のための短い待機
        }
    }

    // --- [2] ストリーミング中のデータ生成とバッファリング ---
    if (isStreaming && deviceConnected)
    {
        if (sampleReady)
        {
            portENTER_CRITICAL(&timerMux);
            sampleReady = false;
            portEXIT_CRITICAL(&timerMux);

            // ダミーデータを生成してバッファに格納
            generateDummyAds1299Sample(sampleBuffer[sampleBufferIndex]);
            sampleBufferIndex++;
            sampleIndexCounter++;
        }

        // --- [3] バッファが満たされたらBLEで送信 ---
        if (sampleBufferIndex >= SAMPLES_PER_CHUNK)
        {
            chunkPacket.packet_type = PKT_TYPE_DATA_CHUNK;
            chunkPacket.start_index = (uint16_t)(sampleIndexCounter - SAMPLES_PER_CHUNK);
            chunkPacket.num_samples = SAMPLES_PER_CHUNK;
            memcpy(chunkPacket.samples, sampleBuffer, sizeof(sampleBuffer));

            pTxCharacteristic->setValue((uint8_t *)&chunkPacket, sizeof(chunkPacket));
            pTxCharacteristic->notify();

            sampleBufferIndex = 0; // バッファインデックスをリセット
        }
    }
    else
    {
        // ストリーミング中でない場合はCPUを少し休ませる
        delay(10);
    }
}
