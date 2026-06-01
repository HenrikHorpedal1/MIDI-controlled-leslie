#include "moteus-config.h"

#include <SPI.h>

namespace {
constexpr int PIN_SCK = 48;
constexpr int PIN_MOSI = 38;
constexpr int PIN_MISO = 47;
constexpr int PIN_CS = 21;
constexpr int PIN_INT = 18;

ACAN2517FD g_can(PIN_CS, SPI, PIN_INT);

Moteus g_horn(g_can, []() {
  Moteus::Options o;
  o.id = 2;
  o.disable_brs = true;
  return o;
}());

Moteus g_drum(g_can, []() {
  Moteus::Options o;
  o.id = 1;
  o.disable_brs = true;
  return o;
}());

bool g_initialized = false;
} // namespace

Moteus &hornMoteus() { return g_horn; }
Moteus &drumMoteus() { return g_drum; }

bool configureMoteus(Print &debug) {
  if (g_initialized)
    return true;

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_INT, INPUT_PULLUP);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  SPI.setFrequency(10'000'000);

  ACAN2517FDSettings settings(ACAN2517FDSettings::OSC_40MHz, 1'000'000,
                              DataBitRateFactor::x1);

  settings.mDriverTransmitFIFOSize = 8;
  settings.mDriverReceiveFIFOSize = 16;

  const uint32_t err = g_can.begin(settings, []() { g_can.isr(); });

  if (err != 0) {
    debug.print("CAN begin error 0x");
    debug.println(err, HEX);
    return false;
  }

  if (!g_horn.SetStop()) {
    debug.println("WARN: No reply from horn moteus");
  }

  // Read encoder 1 (rotor) position and align the moteus output position to it
  mm::Query::Format enc1Qf;
  enc1Qf.extra[0].register_number = mm::Register(0x052);
  enc1Qf.extra[0].resolution      = mm::kFloat;

  if (g_drum.SetStop(&enc1Qf)) {
    const float enc1Pos = g_drum.last_result().values.extra[0].value;
    mm::OutputExact::Command oeCmd;
    oeCmd.position = enc1Pos;
    g_drum.SetOutputExact(oeCmd);
    delay(20);
    debug.printf("Drum output aligned to enc1=%.4f\n", enc1Pos);
  } else {
    debug.println("WARN: No reply from drum moteus");
  }

  g_initialized = true;
  return true;
}
