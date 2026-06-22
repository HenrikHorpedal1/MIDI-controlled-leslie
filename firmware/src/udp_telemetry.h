#pragma once
#include <Arduino.h>
#include <WiFiUdp.h>

// Sends JSON telemetry packets to PlotJuggler via UDP.
// Usage:
//   Telemetry.begin("192.168.x.x", 9870);
//   Telemetry.send("rpm", 400.0f);
//   Telemetry.send("ref", ref, "actual", actual);  // up to 4 pairs per call
//   Telemetry.flush();  // or call send() which auto-flushes

class UdpTelemetry {
public:
    void begin(const char* host, uint16_t port = 9870);

    void send(const char* k1, float v1);
    void send(const char* k1, float v1, const char* k2, float v2);
    void send(const char* k1, float v1, const char* k2, float v2,
              const char* k3, float v3);
    void send(const char* k1, float v1, const char* k2, float v2,
              const char* k3, float v3, const char* k4, float v4);

    // Batched-packet builder used by telemetryLogTask: open a packet, append
    // pairs until batchAdd() returns false (won't fit), then flush.
    void batchBegin(float ts);
    bool batchAdd(const char* key, float value); // false if it won't fit
    void batchFlush();

private:
    WiFiUDP _udp;
    const char* _host = nullptr;
    uint16_t _port = 9870;

    char _batchBuf[512];
    int  _batchLen = 0;
    bool _batchOpen = false;

    void _sendPacket(const char* json, size_t len);
};

extern UdpTelemetry Telemetry;

// The multi-producer logging bus that feeds this sender lives in telemetry_log.h
// (a lightweight, WiFi-free producer header). telemetryLogTask (implemented in
// udp_telemetry.cpp) drains it and sends via Telemetry's batch builder above.
