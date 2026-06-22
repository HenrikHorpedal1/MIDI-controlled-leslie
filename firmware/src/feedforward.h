#pragma once

// Torque feedforward for the Leslie drum and horn rotors.
//
// Given the current velocity reference and acceleration reference (both in the
// motor frame), returns the feedforward torque [Nm] to pass as
// feedforward_torque in the moteus position command.
//
//   T_ff = J * a_ref  +  sign(v_ref) * (c + b|v_ref| + a*v_ref^2)
//          ^^^^^^^^^      ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//          inertia                     friction
//
// The friction term is small at typical Leslie speeds and is OFF by default.
// Enable it with withFriction=true once the friction model is validated at
// the operating speed of interest (>~10 rev/s).
//
// Usage per tick in the controller:
//   cmd.feedforward_torque = ffDrum(v_ref, a_ref);   // drum rotor
//   cmd.feedforward_torque = ffHorn(v_ref, a_ref);   // horn rotor
//
// All units: rev/s, rev/s², Nm — motor frame throughout.

// ---- Identified physical constants ----------------------------------------

// Drum (moteus id=1)
static constexpr double DRUM_J = 7.30e-3;  // reflected inertia [Nm·s²/rev]
static constexpr double DRUM_C = 2.82e-3;  // Coulomb friction  [Nm]
static constexpr double DRUM_B = 2.28e-4;  // viscous friction  [Nm·s/rev]
static constexpr double DRUM_A = 2.70e-5;  // quadratic term    [Nm·s²/rev²]

// Horn (moteus id=2)
static constexpr double HORN_J = 2.29e-3;  // lumped reflected inertia (FRF Jbar)
static constexpr double HORN_C = 7.59e-3;
static constexpr double HORN_B = 7.05e-4;
static constexpr double HORN_A = 3.08e-5;

// ---------------------------------------------------------------------------

double ffDrum(double vRef, double aRef, bool withFriction = false);
double ffHorn(double vRef, double aRef, bool withFriction = false);
