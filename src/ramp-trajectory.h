
enum class SpeedCommand : uint8_t {
    CHORALE,
    STOP,
    TREMOLO
};


void rampTrajectoryCommand(SpeedCommand cmd);
