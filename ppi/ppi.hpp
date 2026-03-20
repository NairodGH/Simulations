#include "main.hpp"

// farthest distance the sonar can detect anything, edge of the green circle on screen
static constexpr float maximumRangeKilometers = 100.f;
// warmer surface water directly affects how fast sound travels through it (~3.5 m/s per °C)
// yes, 21°C surface temperature is what we've reached to with global warming :(
static constexpr float surfaceTemperature = 21.f;
// passive sonar display is split into 720 angular "slices" around the full 360 degree circle, so 0.5° each
static constexpr int passiveBinCount = 720;
// one distinct color per simulated target (up to 10), no bright green so passive rays not confused with sonar ray
static const Color targetColors[10] = {
    { 255, 70, 70, 255 }, // red
    { 70, 160, 255, 255 }, // blue
    { 255, 210, 50, 255 }, // yellow
    { 190, 80, 255, 255 }, // purple
    { 60, 235, 160, 255 }, // green
    { 255, 140, 50, 255 }, // orange
    { 180, 255, 80, 255 }, // lime
    { 255, 80, 200, 255 }, // pink
    { 80, 225, 255, 255 }, // cyan
    { 255, 230, 120, 255 }, // gold
};

// represents a single simulated underwater body with a 2D position in km (x = east/west, y = north/south),
// a velocity in km/s on each axis (how fast and in what direction it is drifting), a color index to look up in targetColors[]
// the last time the rotating sweep arm detected it (so we don't redetect if it's moving along the sweep direction, default -999 for not detected yet)
struct Target {
    float xKilometers, yKilometers;
    float velocityX, velocityY;
    int colorIndex;
    float lastDetectionTime = -999.f;
};

// a "blip" is the bright dot that briefly flashes on the PPI when the sweep arm passes over a target and the echo comes back
// it stores its screen pixel position, how bright it currently is (fades from 1.0 to 0.0 over time) and the color it inherited from its target
struct Blip {
    float xPixels, yPixels;
    float alpha;
    Color color;
};

// passive sonar does not emit any ping it just listens, bins carry no range only bearing but it's stealthier because you don't reveal your position
struct PassiveBin {
    float alpha = 0.f;
    Color color;
};

// hold everything dynamic
struct SonarState {
    float elapsedSeconds = 0.f; // seconds since program start, used as a simulation clock
    float sweepAngleDegrees = 0.f; // current angle of the rotating arm (0=N, 90=E, 180=S, 270=W)
    float previousSweepDegrees = 0.f; // arm angle from last frame, the arc between this and sweepAngleDegrees
    float sweepSpeedDegreesPerSecond = 90.f; // rotation speed: 90 dps default = one full revolution every 4 seconds
    bool activeMode = true; // true = active (ping and listen), false = passive (listen only)
    // the thermocline is a sharp temperature boundary layer in the ocean where warm surface water meets cold deep water,
    // sound waves bend away from it creating a "shadow zone" on the far side where signals are attenuated,
    // like a glass pane that refracts light, some gets through, some bounces off
    // stored as a 0 to 1 fraction of the total depth range (0.4 = 40% of the way down)
    float thermoclineNormalized = 0.4f;
    // real oceans have a "SOFAR channel" (Sound Fixing And Ranging) where sound speeds back up at great depth
    // after slowing down at the thermocline, this simulates that speed recovery
    // 0.0 = no recovery (sound stays slow in the deep), 1.0 = maximum speed recovery
    float deepSpeedBoost = 0.3f;
    // updated every draw frame and read by the UI panel to show what the cursor is pointing at on the PPI
    float mouseBearing = 0.f; // compass angle from our ship to the cursor (see sweepAngleDegrees)
    float mouseRange = 0.f; // how far from our ship that cursor point represents in real-world kilometers
    bool mouseOnPPI = false; // true only when the cursor is inside the green sonar circle

    int targetCount = 5;
    std::vector<Target> targets;
    std::vector<Blip> blips;
    std::array<PassiveBin, passiveBinCount> passiveBins {};
};