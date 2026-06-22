#include "moteus-config.h"
#include "leslie_config.h"

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

namespace {
mm::PositionMode::Format makePositionFmt() {
  mm::PositionMode::Format f;
  f.feedforward_torque = mm::kFloat;
  f.accel_limit        = mm::kFloat;
  f.velocity_limit     = mm::kFloat;
  return f;
}

mm::Query::Format makeQueryFmt() {
  mm::Query::Format f;
  f.position = mm::kFloat;
  f.velocity = mm::kFloat;
  f.torque   = mm::kFloat;                          // measured torque (0x00a)
  // Internal control-loop references (planner output) for the position-
  // controller test. Extras MUST stay in ascending register order: the moteus
  // protocol reads them as one contiguous block (min = extra[0], max = last).
  f.extra[0].register_number = mm::Register(0x038); // Control position
  f.extra[0].resolution = mm::kFloat;
  f.extra[1].register_number = mm::Register(0x039); // Control velocity
  f.extra[1].resolution = mm::kFloat;
  f.extra[2].register_number = mm::Register(0x03a); // Control torque
  f.extra[2].resolution = mm::kFloat;
  f.extra[3].register_number = mm::Register(0x052); // Encoder 1 position
  f.extra[3].resolution = mm::kFloat;
  f.extra[4].register_number = mm::Register(0x053); // Encoder 1 velocity
  f.extra[4].resolution = mm::kFloat;
  return f;
}

const mm::PositionMode::Format g_positionFmt = makePositionFmt();
const mm::Query::Format g_queryFmt = makeQueryFmt();
} // namespace

const mm::PositionMode::Format &lesliePositionFmt() { return g_positionFmt; }
const mm::Query::Format &leslieQueryFmt() { return g_queryFmt; }

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

  // Read encoder 1 (rotor) position and align the moteus output position to it
  mm::Query::Format enc1Qf;
  enc1Qf.extra[0].register_number = mm::Register(0x052);
  enc1Qf.extra[0].resolution = mm::kFloat;

  if (g_horn.SetStop(&enc1Qf)) {
    const float enc1Pos = g_horn.last_result().values.extra[0].value;
    mm::OutputExact::Command oeCmd;
    // rotor_to_output_ratio=1: output counter is in motor revs.
    // Scale enc1Pos (load revs) by belt ratio so motorPos/i == enc1Pos at startup.
    oeCmd.position = enc1Pos * static_cast<float>(HORN_BELT_RATIO);
    g_horn.SetOutputExact(oeCmd);
    delay(20);
    debug.printf("Horn output aligned to enc1=%.4f (motor=%.4f)\n",
                 enc1Pos, oeCmd.position);
  } else {
    debug.println("WARN: No reply from horn moteus");
  }

  if (g_drum.SetStop(&enc1Qf)) {
    const float enc1Pos = g_drum.last_result().values.extra[0].value;
    mm::OutputExact::Command oeCmd;
    oeCmd.position = enc1Pos * static_cast<float>(DRUM_BELT_RATIO);
    g_drum.SetOutputExact(oeCmd);
    delay(20);
    debug.printf("Drum output aligned to enc1=%.4f (motor=%.4f)\n",
                 enc1Pos, oeCmd.position);
  } else {
    debug.println("WARN: No reply from drum moteus");
  }

  g_initialized = true;
  return true;
}
