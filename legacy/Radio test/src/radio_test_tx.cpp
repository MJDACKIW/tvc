#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS   10
#define RFM95_RST  24
#define RFM95_INT  8
#define RF95_FREQ  915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println("RF95 init failed");
    while (1) {}
  }
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1) {}
  }
  rf95.setTxPower(23, false);
  rf95.setSpreadingFactor(7);
  rf95.setSignalBandwidth(125000);
  rf95.setCodingRate4(5);

  Serial.println("TX ready");
}

uint32_t packetNum = 0;

void loop() {
  char msg[32];
  snprintf(msg, sizeof(msg), "PING %lu", packetNum++);

  rf95.send((uint8_t*)msg, strlen(msg) + 1);
  rf95.waitPacketSent();
  Serial.print("Sent: "); Serial.println(msg);

  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  if (rf95.waitAvailableTimeout(1000)) {
    if (rf95.recv(buf, &len)) {
      Serial.print("Reply: "); Serial.println((char*)buf);
      Serial.print("RSSI: "); Serial.println(rf95.lastRssi());
    }
  } else {
    Serial.println("No reply — timeout");
  }

  delay(1000);
}