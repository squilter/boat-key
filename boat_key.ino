// These libraries were installed automatically after adding espressif esp32 2.0.9
// in the package manager. Set board to ESP32C3 dev module (for my board), flash mode DIO.
// Many esp32 boards don't support bluetooth, and these libraries won't exist if you select them.
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <SoftwareSerial.h>

#define TX_PIN 6

BLEScan* pBLEScan;

// Write your own into here.
esp_bd_addr_t addr{0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
BLEAddress cube{addr};

// I had trouble with the uarts on my board, so using soft uart
// second arg is RX, which isn't used
SoftwareSerial pixhawk(255, TX_PIN);

struct __attribute__((packed)) MavlinkHeartbeatPayload
{
  uint8_t type = 6; // MAV_TYPE_GCS = 6
  uint8_t autopilot = 8; // MAV_AUTOPILOT_INVALID = 8
  uint8_t base_mode = 0;
  uint32_t custom_mode = 0;
  uint8_t system_status = 0;
  uint8_t mavlink_version = 3;
} ;

struct __attribute__((packed)) MavlinkHeartbeat
{
  uint8_t start_of_frame = 0xfe;
  uint8_t payload_length = sizeof(MavlinkHeartbeatPayload);
  uint8_t packet_sequence; // increment this manually every time!
  uint8_t system_id = 0;
  uint8_t component_id = 0; // see readme
  uint8_t message_id = 0; // HEARTBEAT = 0
  MavlinkHeartbeatPayload payload;
  uint16_t crc;
};

void crc_accumulate(uint8_t data, uint16_t *crcAccum)
{
        /*Accumulate one byte of data into the CRC*/
        uint8_t tmp;

        tmp = data ^ (uint8_t)(*crcAccum &0xff);
        tmp ^= (tmp<<4);
        *crcAccum = (*crcAccum>>8) ^ (tmp<<8) ^ (tmp <<3) ^ (tmp>>4);
}

uint16_t crc_calculate(const uint8_t* pBuffer, uint16_t length)
{
        uint16_t ret = 0xffff;
        while (length--) {
                crc_accumulate(*pBuffer++, &ret);
        }
        crc_accumulate(50, &ret); // add the magic number 50 for "crc extra"
        return ret;
}

void sendHeartbeat()
{
  static MavlinkHeartbeat heartbeat;
  uint8_t* raw = reinterpret_cast<uint8_t*>(&heartbeat);
  heartbeat.packet_sequence++;
  // crc skips start-of-frame and skips the crc field itself
  heartbeat.crc = crc_calculate(raw+1, sizeof(MavlinkHeartbeat) - 1 - 2);
  pixhawk.write(raw, sizeof(MavlinkHeartbeat));  
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      Serial.printf("Found Device: %s \n", advertisedDevice.toString().c_str());
      if(advertisedDevice.getAddress().equals(cube)) {
        Serial.printf("Sending heartbeat!\n");
        sendHeartbeat();
      }
    }
};

void setup() {
  Serial.begin(9600);
  Serial.println("Scanning...");

  pinMode(TX_PIN, OUTPUT);
  pixhawk.begin(115200);

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);  // less or equal setInterval value
}

void loop() {
  BLEScanResults foundDevices = pBLEScan->start(3, false); // 3 seconds
}
