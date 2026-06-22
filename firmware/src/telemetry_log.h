// telemetry_log.h
#pragma once

// Lightweight producer API for the shared telemetry bus. Any task can publish a
// named scalar; a single low-priority task (telemetryLogTask) drains the queue
// and does all the UDP, so no producer ever touches the network or pulls in the
// WiFi headers. Each module logs the data it owns, where it computes it.
//
// Keys are stored BY POINTER (not copied), so they MUST be string literals or
// otherwise have static lifetime.
void telemetryLogInit();                          // create the queue (call in setup())
void telemetryLog(const char* key, float value);  // non-blocking; drops if full
void telemetryLogTask(void* pvParameters);        // drains the queue, sends batched
