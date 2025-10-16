#include "driver/twai.h"
#include <WiFi.h>
#include <WiFiUDP.h>
#include "NetworkUdp.h"

#define RX_PIN 27
#define TX_PIN 26

#define RDATA_SIZE 256

const char *ssid = "DoradoraPC";
const char *pass = "12345678";

IPAddress ip(192, 168, 137, 5);
IPAddress gateway(192, 168, 137, 1);
IPAddress subnet(255, 255, 255, 0);

int kLocalPort = 8000;

uint8_t recdata[RDATA_SIZE];

WiFiUDP udp;

static void can_transmit(uint32_t id, const uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote) {
  twai_message_t message;
  message.identifier = id;
  message.data_length_code = dlc;
  message.extd = isExtended ? 1 : 0;
  message.rtr = isRemote ? 1 : 0;
  for (int i = 0; i < dlc; i++) {
    message.data[i] = data[i];
  }
  twai_transmit(&message, pdMS_TO_TICKS(100));
}

void setup() {
  Serial.begin(115200);

  WiFi.config(ip, gateway, subnet);
  WiFi.begin(ssid, pass);
  Serial.print("Wi-Fi Connecting...");
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  udp.begin(kLocalPort);
  Serial.println();
  Serial.println("Wi-Fi Connected");

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();  //Look in the api-reference for other speed sets.
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("Failed to install driver");
    while (1) delay(1000);
  }
  if (twai_start() != ESP_OK) {
    Serial.println("Failed to start driver");
    while (1) delay(1000);
  }
  Serial.println("OK");
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(recdata, RDATA_SIZE);
    if (len > 0) {
      if (recdata[0] == 1) {
        if (len >= 9) {
          can_transmit(1, &recdata[1], 8, false, false);
        }
      } else if (recdata[0] == 2) {
        if (len >= 9) {
          can_transmit(2, &recdata[1], 8, false, false);
        }
      }
    }
  }
}
