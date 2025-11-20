#include <Wire.h>
#include "MS5837.h"
#include "ping1d.h"

// ---- Ping1D #1 (Serial1) ----
static Ping1D ping1 { Serial1 };

// ---- Ping1D #2 (Serial2) ----
static Ping1D ping2 { Serial2 };

// ---- Bar02 Sensor ----
MS5837 bar02;

// ---- Output packet ----
// [d1, c1, d2, c2, depth_mm, temp_cX100]
int32_t packet[6];

static const uint8_t ledPin = 13;

void setup() {
  Serial.begin(115200);     // PC / Raspberry Pi

  Serial1.begin(115200);    // Ping1D #1
  Serial2.begin(115200);    // Ping1D #2

  pinMode(ledPin, OUTPUT);

  // ---- Ping1D init ----
  while (!ping1.initialize()) {
    Serial.println("Ping1 init...");
    delay(500);
  }
  while (!ping2.initialize()) {
    Serial.println("Ping2 init...");
    delay(500);
  }

  // ---- Bar02 init ----
  Wire.begin();
  while (!bar02.init()) {
    Serial.println("Bar02 init...");
    delay(500);
  }

  bar02.setModel(MS5837::MS5837_02BA);
  bar02.setFluidDensity(997);  // freshwater

  Serial.println("=== All Sensors Ready ===");
}

void loop() {

  // ---- Ping1 ----
  if (ping1.update()) {
    packet[0] = ping1.distance();    // mm
    packet[1] = ping1.confidence();  // %
  }

  // ---- Ping2 ----
  if (ping2.update()) {
    packet[2] = ping2.distance();
    packet[3] = ping2.confidence();
  }

  // ---- Bar02 ----
  bar02.read();

  float depth_m = bar02.depth();       // m
  float temp_c = bar02.temperature();  // °C

  packet[4] = (int32_t)(depth_m * 1000.0f);   // depth(mm)
  packet[5] = (int32_t)(temp_c * 100.0f);     // temp x100

  // ---- デバッグ表示（テキスト） ----
  Serial.print("D1:");
  Serial.print(packet[0]);
  Serial.print("  C1:");
  Serial.print(packet[1]);
  Serial.print("  D2:");
  Serial.print(packet[2]);
  Serial.print("  C2:");
  Serial.print(packet[3]);
  Serial.print("  Depth(mm):");
  Serial.print(packet[4]);
  Serial.print("  Temp(x100):");
  Serial.println(packet[5]);

  // ---- バイナリ送信（必要なときにON） ----
  Serial.write((uint8_t*)packet, sizeof(packet));

  digitalWrite(ledPin, !digitalRead(ledPin));
  delay(100);
}
