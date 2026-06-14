#include "udp_telemetry.h"
#include <WiFi.h>
#include <stdio.h>

UdpTelemetry Telemetry;

void UdpTelemetry::begin(const char* host, uint16_t port) {
    _host = host;
    _port = port;
    _udp.begin(0);  // bind to any local port
}

void UdpTelemetry::_sendPacket(const char* json, size_t len) {
    if (!_host) return;
    _udp.beginPacket(_host, _port);
    _udp.write((const uint8_t*)json, len);
    _udp.endPacket();
}

static float _ts() { return millis() / 1000.0f; }

void UdpTelemetry::send(const char* k1, float v1) {
    char buf[128];
    int n = snprintf(buf, sizeof(buf),
        "{\"ts\":%.3f,\"%s\":%.4f}", _ts(), k1, v1);
    _sendPacket(buf, n);
}

void UdpTelemetry::send(const char* k1, float v1, const char* k2, float v2) {
    char buf[192];
    int n = snprintf(buf, sizeof(buf),
        "{\"ts\":%.3f,\"%s\":%.4f,\"%s\":%.4f}",
        _ts(), k1, v1, k2, v2);
    _sendPacket(buf, n);
}

void UdpTelemetry::send(const char* k1, float v1, const char* k2, float v2,
                         const char* k3, float v3) {
    char buf[256];
    int n = snprintf(buf, sizeof(buf),
        "{\"ts\":%.3f,\"%s\":%.4f,\"%s\":%.4f,\"%s\":%.4f}",
        _ts(), k1, v1, k2, v2, k3, v3);
    _sendPacket(buf, n);
}

void UdpTelemetry::send(const char* k1, float v1, const char* k2, float v2,
                         const char* k3, float v3, const char* k4, float v4) {
    char buf[320];
    int n = snprintf(buf, sizeof(buf),
        "{\"ts\":%.3f,\"%s\":%.4f,\"%s\":%.4f,\"%s\":%.4f,\"%s\":%.4f}",
        _ts(), k1, v1, k2, v2, k3, v3, k4, v4);
    _sendPacket(buf, n);
}
