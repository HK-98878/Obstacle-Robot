// WiFiComm.cpp

#include "WiFiComm.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "globals.h"

static bool WiFi_is_setup = false;
static WiFiUDP Udp;
static const unsigned int localUdpPort = 55500;

static constexpr size_t UDP_BUF_SZ = 128;
char incomingPacket[UDP_BUF_SZ];
const char KILL_COMMAND[] = "Stop";

void setupWiFi(const char* ssid, const char* password, unsigned long WIFI_TIMEOUT_MS) {
  unsigned long wifi_start = millis();
  CONDITIONAL_SERIAL_PRINT("Connecting to WiFi");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {

    if (millis() - wifi_start < WIFI_TIMEOUT_MS) {
      delay(500);
      CONDITIONAL_SERIAL_PRINT(".");
    } else break;
  }

  uint8_t status = WiFi.status();

// Check status - exit without starting UDP and setting the "WiFi_is_setup" variable if the
//  connection fails
#ifdef SERIAL_LOGGING
  switch (status) {
    case WL_CONNECTED:
      Serial.println("Connected to WiFi!");
      Serial.println("IP Address: ");
      Serial.println(WiFi.localIP());
      break;

    case WL_NO_SSID_AVAIL:
      Serial.println("SSID not found.");
      return;
    case WL_CONNECT_FAILED:
      Serial.println("Connection failed.");
      return;
    case WL_DISCONNECTED:
      Serial.println("Disconnected.");
      return;

    case WL_SCAN_COMPLETED:
      Serial.println("WiFi Status: SCAN COMPLETED - Networks scanned.");
      return;

    case WL_CONNECTION_LOST:
      Serial.println("WiFi Status: CONNECTION LOST - Was connected but lost it.");
      return;
  }
#else
  switch (status) {
    case WL_CONNECTED:
      break;

    case WL_NO_SSID_AVAIL:
    case WL_CONNECT_FAILED:
    case WL_DISCONNECTED:
    case WL_SCAN_COMPLETED:
    case WL_CONNECTION_LOST:
      return;
  }
#endif

  delay(1000);

  // Begin listening for UDP
  Udp.begin(localUdpPort);
  CONDITIONAL_SERIAL_PRINT("Listening for UDP on port: ");
  CONDITIONAL_SERIAL_PRINTLN(localUdpPort);

  WiFi_is_setup = true;
}


void handleWiFiCommands(bool* kill_switch_active) {
  if (!WiFi_is_setup) return;  // Wifi has not been initialised

  int packetSize = Udp.parsePacket();
  if (packetSize <= 0) return;  // no packet

  if (packetSize >= UDP_BUF_SZ) {
    // flush and ignore
    Udp.flush();
    CONDITIONAL_SERIAL_PRINTLN("UDP packet too big, dropped");
    return;
  }

  int len = Udp.read(incomingPacket, UDP_BUF_SZ - 1);  // limit 255? sizeof(incomingPacket)
  if (len <= 0) return;                                // read error or empty

  incomingPacket[len] = '\0';  // Null‑terminate
  CONDITIONAL_SERIAL_PRINT("Received UDP: ");
  CONDITIONAL_SERIAL_PRINTLN(incomingPacket);

  // Compare C‑strings directly
  if (strcmp(incomingPacket, KILL_COMMAND) == 0) {
    *kill_switch_active = true;
    CONDITIONAL_SERIAL_PRINT("Robot Eliminated");
  }
}
