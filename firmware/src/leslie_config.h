#pragma once

// Measured belt ratios (motor revs per load rev) for each drive.
// Identified with measure_belt_slip.py at rotor_to_output_ratio = 1.
static constexpr double HORN_BELT_RATIO = 3.675;
static constexpr double DRUM_BELT_RATIO = 4.156;

// ===== RPM targets ============================================================

// MIDI / expression-pedal chorale & tremolo speeds.
static constexpr float HORN_CHORALE_RPM      = 40.0f;
static constexpr float HORN_TREMOLO_RPM      = 420.0f;
static constexpr float DRUM_CHORALE_RPM      = 35.0f;
static constexpr float DRUM_TREMOLO_RPM      = 350.0f;

// Footswitch chorale runs a touch faster than the MIDI-triggered chorale.
static constexpr float FOOT_HORN_CHORALE_RPM = 50.0f;
static constexpr float FOOT_DRUM_CHORALE_RPM = 44.0f;

// Beat-sync: hard speed cap — subdivisions that would exceed this are denied.
static constexpr double BEAT_MAX_RPM         = 450.0;

// ===== Acceleration limits (rev/s²) ==========================================

// Velocity mode: independent up/down ramps per rotor.
static constexpr double ACCEL_UP_HORN_REV_S2   = 3.50;   // 420 RPM in ~2 s
static constexpr double ACCEL_DOWN_HORN_REV_S2 = 2.50;   // ~2.7 s down from tremolo
static constexpr double ACCEL_UP_DRUM_REV_S2   = 1.458;  // 350 RPM in ~4 s
static constexpr double ACCEL_DOWN_DRUM_REV_S2 = 1.17;  // ~5 s from tremolo to stop

// BeatSync cam planner spin-up limits (seeds beatCamInit).
static constexpr double BEAT_ACCEL_UP_REV_S2   = 3.50;
static constexpr double BEAT_ACCEL_DOWN_REV_S2 = 2.50;

// Park: gentle hold limits and the speed below which a rotor snaps to a mark.
static constexpr double ACCEL_LIMIT_STOP_REV_S2  = 0.50;
static constexpr double VELOCITY_LIMIT_STOP       = 0.5;
static constexpr double HORN_STOP_THRESH_REV_S    = 0.333; // 20 RPM
static constexpr double DRUM_STOP_THRESH_REV_S    = 0.333; // 20 RPM

// ===== Beat-sync phase home offsets (fraction of one revolution) ==============

// Mechanical mounting offset of the rotor face on the beat [rev] (0 = encoder/
// face zero physically faces the mic on the beat). This is a fixed ANGLE; use it
// only for the mounting orientation, NOT for latency (see BEAT_PHASE_LEAD_US).
static constexpr double HORN_BEAT_HOME_FRAC = -20.0 / 360.0;  // delay horn swell 20 deg relative to the mics/beat
static constexpr double DRUM_BEAT_HOME_FRAC = -30.0 / 360.0;  // delay drum swell 30 deg relative to the mics/beat

// Global beat-phase lead compensation [us]. The rotors physically lead the beat
// by a roughly constant TIME (downstream actuation/acoustic latency; the PLL
// residual is zero-mean, so it is not a clock bias). Retarding the phase
// reference by this many microseconds delays both rotors equally at EVERY
// tempo/subdivision (unlike a fixed home-frac angle, which only cancels the lead
// at one speed). Positive = retard (cures a lead). Tune until the lead is gone,
// then confirm it stays gone across tempos.
static constexpr double BEAT_PHASE_LEAD_US = 0.0;  // zeroed while tuning HORN_BEAT_HOME_FRAC (geometric offset) first; re-add only residual tempo-scaling latency afterward
