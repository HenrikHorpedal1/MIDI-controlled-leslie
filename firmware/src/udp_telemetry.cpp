#include "udp_telemetry.h"
#include "telemetry_log.h"
#include <WiFi.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

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

// --- Batched packet builder -------------------------------------------------
void UdpTelemetry::batchBegin(float ts) {
    _batchLen = snprintf(_batchBuf, sizeof(_batchBuf), "{\"ts\":%.3f", ts);
    _batchOpen = true;
}

bool UdpTelemetry::batchAdd(const char* key, float value) {
    if (!_batchOpen) return false;
    // Reserve 1 byte for the closing '}'. Probe into a scratch first so a pair
    // that doesn't fit leaves the buffer untouched (caller flushes and retries).
    char frag[96];
    int fn = snprintf(frag, sizeof(frag), ",\"%s\":%.4f", key, value);
    if (fn < 0 || _batchLen + fn + 1 >= (int)sizeof(_batchBuf)) return false;
    memcpy(_batchBuf + _batchLen, frag, fn);
    _batchLen += fn;
    return true;
}

void UdpTelemetry::batchFlush() {
    if (!_batchOpen) return;
    if (_batchLen + 1 < (int)sizeof(_batchBuf)) _batchBuf[_batchLen++] = '}';
    _sendPacket(_batchBuf, _batchLen);
    _batchOpen = false;
    _batchLen = 0;
}

// --- Shared logging bus -----------------------------------------------------
struct LogEntry {
    const char* key; // string literal (stored by pointer)
    float       value;
    float       ts;  // seconds, stamped at publish time
};

static QueueHandle_t s_logQueue = nullptr;
static constexpr int LOG_QUEUE_DEPTH = 128;

void telemetryLogInit() {
    if (!s_logQueue)
        s_logQueue = xQueueCreate(LOG_QUEUE_DEPTH, sizeof(LogEntry));
}

void telemetryLog(const char* key, float value) {
    if (!s_logQueue) return;
    LogEntry e{key, value, millis() / 1000.0f};
    (void)xQueueSend(s_logQueue, &e, 0); // non-blocking; drop if full
}

void telemetryLogTask(void* /*pvParameters*/) {
    LogEntry e;
    for (;;) {
        // Block for the first entry, then drain whatever else is queued right now
        // into as few packets as fit, so a burst (one control tick) batches.
        if (xQueueReceive(s_logQueue, &e, portMAX_DELAY) != pdTRUE) continue;
        Telemetry.batchBegin(e.ts);
        Telemetry.batchAdd(e.key, e.value);
        while (xQueueReceive(s_logQueue, &e, 0) == pdTRUE) {
            if (!Telemetry.batchAdd(e.key, e.value)) {
                Telemetry.batchFlush();
                Telemetry.batchBegin(e.ts);
                Telemetry.batchAdd(e.key, e.value);
            }
        }
        Telemetry.batchFlush();
    }
}
