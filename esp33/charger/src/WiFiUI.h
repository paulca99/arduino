#ifndef WIFI_UI_H
#define WIFI_UI_H

// ============================================================================
// WiFi and Web Interface Module
// ============================================================================
// Manages WiFi connection, HTTP status page (port 80), and CSV endpoint (port 8080).
// Allows runtime configuration of power limits and calibration values.

void wifiSetup();
void wifiLoop();
void connectToWifi();

#endif // WIFI_UI_H
