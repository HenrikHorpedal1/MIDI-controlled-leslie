#include "moteus-config.h"

#include <SPI.h>

namespace {
constexpr int PIN_SCK  = 48;
constexpr int PIN_MOSI = 38;
constexpr int PIN_MISO = 47;
constexpr int PIN_CS   = 21;
constexpr int PIN_INT  = 18;

ACAN2517FD g_can(PIN_CS, SPI, PIN_INT);

Moteus g_horn(g_can, []() {
  Moteus::Options o;
  o.id = 2;
  o.disable_brs = true;
  return o;
}());

bool g_initialized = false;
}

Moteus& hornMoteus() { return g_horn; }

bool configureMoteus(Print& debug) {
  if (g_initialized) return true;

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_INT, INPUT_PULLUP);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  SPI.setFrequency(10'000'000);

  ACAN2517FDSettings settings(
      ACAN2517FDSettings::OSC_40MHz,
      1'000'000,
      DataBitRateFactor::x1);

  settings.mDriverTransmitFIFOSize = 8;
  settings.mDriverReceiveFIFOSize = 16;

  const uint32_t err = g_can.begin(settings, []() {
    g_can.isr();
  });

  if (err != 0) {
    debug.print("CAN begin error 0x");
    debug.println(err, HEX);
    return false;
  }

  if (!g_horn.SetStop()) {
    debug.println("No reply from horn moteus");
    return false;
  }

  // Configure PID gains (RAM only — survives until power-off)
  char pidBuf[64];
  snprintf(pidBuf, sizeof(pidBuf), "conf set servo.pid_position.kp %.4f", MOTEUS_BASE_KP);
  g_horn.DiagnosticCommand(pidBuf); delay(20);
  snprintf(pidBuf, sizeof(pidBuf), "conf set servo.pid_position.ki %.4f", MOTEUS_BASE_KI);
  g_horn.DiagnosticCommand(pidBuf); delay(20);
  snprintf(pidBuf, sizeof(pidBuf), "conf set servo.pid_position.kd %.4f", MOTEUS_BASE_KD);
  g_horn.DiagnosticCommand(pidBuf); delay(20);
  debug.printf("Moteus PID gains set: kp=%.4f ki=%.4f kd=%.4f\n",
               MOTEUS_BASE_KP, MOTEUS_BASE_KI, MOTEUS_BASE_KD);

  g_initialized = true;
  return true;
}
