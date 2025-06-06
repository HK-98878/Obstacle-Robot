#ifndef WIFICOMM_H
#define WIFICOMM_H
#include <Arduino.h>

/*
WiFi Code:
- Setup
- Kill command handling
*/

void setupWiFi(const char* ssid, const char* password, unsigned long WIFI_TIMEOUT_MS);
void handleWiFiCommands(bool *kill_switch_active);

#endif