/*
  ESP32 + R60ABD1 毫米波雷達 正確協定解碼程式
*/

#include <HardwareSerial.h>

// ESP32 串口腳位設定 (若為 ESP32-C3 請改為 2 與 3)
#define RADAR_RX_PIN 16  // ESP32 RX2 (接雷達 TX)
#define RADAR_TX_PIN 17  // ESP32 TX2 (接雷達 RX)

HardwareSerial RadarSerial(2);

// 協定長度上限
#define MAX_PACKET_LEN 32

uint8_t packetBuf[MAX_PACKET_LEN];
uint8_t bufIndex = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("\n=============================================");
  Serial.println("  ESP32 + R60ABD1 雷達心率 (BPM) 協定校正解析");
  Serial.println("=============================================\n");

  // R60ABD1 波特率為 115200
  RadarSerial.begin(115200, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
}

// 解析完整的 R60ABD1 封包
void parseCompletePacket(uint8_t* p, uint8_t len) {
  // R60ABD1 封包結構：
  // [0]=0x53, [1]=0x59 (Header)
  // [2]=Control Byte, [3]=Command Byte
  // [4]=Length High, [5]=Length Low
  // [6...]=Data Payload
  // [len-2]=0x54, [len-1]=0x43 (Tail)

  if (len < 8) return; // 格式不完整忽略

  uint8_t controlCmd = p[2];
  uint8_t commandID  = p[3];
  
  // 數據長度
  uint16_t dataLen = ((uint16_t)p[4] << 8) | p[5];

  // 1. 生理參數 - 心率數據 (Control: 0x85)
  if (controlCmd == 0x85) {
    if (commandID == 0x02 && dataLen >= 1) { // 0x85 0x02 代表即時心率
      uint8_t heartRate = p[6]; // 正確的心率 Data 位置落在 p[6]
      
      // 心率合理範圍過濾 (40 ~ 180 BPM)
      if (heartRate >= 40 && heartRate <= 180) {
        Serial.print("❤ 【即時心率】: ");
        Serial.print(heartRate);
        Serial.println(" BPM");
      }
    }
  } 
  // 2. 人體狀態/體動資訊 (Control: 0x80)
  else if (controlCmd == 0x80) {
    // 0x80 0x01: 人體存在判斷 (0x00: 無人, 0x01: 有人)
    if (commandID == 0x01 && dataLen >= 1) {
      uint8_t presence = p[6];
      if (presence == 0x00) {
        Serial.println("⚠ [狀態] 雷達感應區內無人");
      }
    }
    // 0x80 0x02: 體動/活動狀態 (0x01: 較大體動, 0x02: 微幅靜止)
    else if (commandID == 0x02 && dataLen >= 1) {
      uint8_t motion = p[6];
      if (motion == 0x01) {
        Serial.println("🏃 [狀態] 偵測到較大體動（心率數據暫停更新/估測中）");
      } else if (motion == 0x02) {
        Serial.println(" [狀態] 微幅靜止/進入心率穩定量測區");
      }
    }
  }
}

void loop() {
  while (RadarSerial.available() > 0) {
    uint8_t b = RadarSerial.read();

    // 檢查 Header 1: 0x53
    if (bufIndex == 0) {
      if (b == 0x53) {
        packetBuf[bufIndex++] = b;
      }
      continue;
    }

    // 檢查 Header 2: 0x59
    if (bufIndex == 1) {
      if (b == 0x59) {
        packetBuf[bufIndex++] = b;
      } else {
        bufIndex = 0; // 同步失敗重置
      }
      continue;
    }

    // 收集封包資料
    packetBuf[bufIndex++] = b;

    // 檢查 Tail: 0x54 0x43
    if (bufIndex >= 8 && packetBuf[bufIndex - 2] == 0x54 && packetBuf[bufIndex - 1] == 0x43) {
      // 收到完整 Header 53 59 ... 54 43 封包，進行解析
      parseCompletePacket(packetBuf, bufIndex);
      bufIndex = 0; // 重置準備下一次接收
    }

    // 防禦性程式碼：防止緩衝區溢位
    if (bufIndex >= MAX_PACKET_LEN) {
      bufIndex = 0;
    }
  }
}
