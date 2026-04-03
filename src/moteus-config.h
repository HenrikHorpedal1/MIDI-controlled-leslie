#pragma once

#include <Arduino.h>
#include <ACAN2517FD.h>
#include <Moteus.h>

using Moteus = MoteusController<ACAN2517FD>;

bool configureMoteus(Print& debug = Serial);
Moteus& hornMoteus();
