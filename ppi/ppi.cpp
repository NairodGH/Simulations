#include "ppi.hpp"

#pragma region update utils
// converts bearing (angle) to unit 2D vector pointing in that direction so we can scale it by a distance and get a position,
// ex: bearing 0° (north) -> (0, -1) in screen space because screen y increases downward, so "up" on screen is negative y,
// ex: bearing 90° (east) -> (1, 0),  bearing 270° (west) -> (-1, 0)
// sin gives the east component (x), -cos gives the screen-up component (y), circle but rotated so 0° points up instead of right
static Vector2 bearingToDirection(float bearingDegrees)
{
    float radians = bearingDegrees * DEG2RAD;
    return { std::sin(radians), -std::cos(radians) };
}

// checks whether a target's bearing falls inside the "slice of pie" the sweep arm just rotated through,
// we need this because at high frame rates the arm moves only a tiny arc per frame, but we still need to detect
// every target the arm passes over, so instead of checking one exact angle we check an arc,
// all three angles are normalised into [0°, 360°) first to handle wrap-around (ex: 365° -> 5°),
// then we measure the clockwise arc length from arcStart to arcEnd, and check whether the clockwise distance from arcStart to bearing fits inside that arc
// ex: arcStart=355°, arcEnd=5°, bearing=2° -> arcLength=10°, offsetToBearing=7° -> 7<=10 -> true (inside)
static bool bearingInArc(float bearing, float arcStart, float arcEnd)
{
    bearing = std::fmod(bearing + 360.f, 360.f);
    arcStart = std::fmod(arcStart + 360.f, 360.f);
    arcEnd = std::fmod(arcEnd + 360.f, 360.f);
    float arcLength = std::fmod(arcEnd - arcStart + 360.f, 360.f);
    float offsetToBearing = std::fmod(bearing - arcStart + 360.f, 360.f);
    return offsetToBearing <= arcLength;
}

// returns the sound speed at a given normalizedDepth (0.0 = surface, 1.0 = deepest),
// above the thermocline: speed linearly drops from surfaceSpeed down to minimumSpeed as you go deeper,
// below the thermocline: speed linearly rises from minimumSpeed back up toward deepSpeed (the SOFAR channel effect),
// the 1e-4 added to each denominator prevents a divide-by-zero when the thermocline is at exactly 0 or 1
static float soundSpeedMetersPerSecond(float normalizedDepth, float thermoclineNormalized, float deepSpeedBoost)
{
    float surfaceSpeed = 1450.f + 3.5f * surfaceTemperature; // warmer surface = faster sound
    float minimumSpeed = surfaceSpeed - 80.f; // the slowest point, right at the thermocline boundary
    float deepSpeed = minimumSpeed + deepSpeedBoost * 100.f; // how much speed recovers in the deep layer
    if (normalizedDepth <= thermoclineNormalized)
        return surfaceSpeed - (surfaceSpeed - minimumSpeed) * (normalizedDepth / (thermoclineNormalized + 1e-4f));
    return minimumSpeed + (deepSpeed - minimumSpeed) * ((normalizedDepth - thermoclineNormalized) / (1.f - thermoclineNormalized + 1e-4f));
}

// the thermocline creates a "shadow zone", when a ping or noise crosses it, some of the sound energy is
// refracted (bent) sideways and never reaches the other side, this returns extra signal loss in decibels (dB) for a target beyond the shadow boundary,
// decibels are logarithmic: +10 dB ≈ 10x more power lost, so even 18 dB of extra loss cuts signal severely,
// no loss at all if the target is closer than the shadow edge, or if the thermocline is negligibly shallow
static float shadowLossDecibels(float rangeKilometers, float thermoclineNormalized, float deepSpeedBoost)
{
    float shadowKilometers = thermoclineNormalized * maximumRangeKilometers; // where the shadow zone begins
    if (rangeKilometers <= shadowKilometers || shadowKilometers < 1.f)
        return 0.f;
    // log10(range/shadow) grows slowly: at 2x the shadow range -> ~5 dB, at 10x -> ~18 dB
    return 18.f * (1.f - deepSpeedBoost) * std::log10(rangeKilometers / shadowKilometers);
}

// sound spreading in 3D (spherical) follows 20·log10(r), but because the sound travels to target then back to us,
// the signal degrades much faster than passive (40 dB per decade vs 20),
// the 78 dB Figure of Merit (FOM) means a ping that travels far enough loses all detectable energy,
// 40·log10(100) = 80 dB active loss, so FOM needs to be just above 80 to detect at 100km, 78 means you start losing 100km contacts first
// returns a 0 to 1 fraction where 1.0 = perfect echo, 0.0 = completely lost in noise,
// the 40*log10 term is called "two-way transmission loss", doubling the range loses ~12 dB in this model,
// (real active sonar also accounts for sea-state noise, target strength, and reverberation, but this is a clean sim)
static float activeSignalStrength(float rangeKilometers, float thermoclineNormalized, float deepSpeedBoost)
{
    if (rangeKilometers < 0.1f)
        return 1.f; // essentially on top of us, perfect signal
    float transmissionLoss = 40.f * std::log10(rangeKilometers) + shadowLossDecibels(rangeKilometers, thermoclineNormalized, deepSpeedBoost);
    return std::max(0.f, (78.f - transmissionLoss) / 78.f); // clamp to 0 if loss exceeds budget
}

// since sound only travels one way (target to us), the loss is half as severe (20 dB per decade vs 40),
// and the shadow loss is halved too, less total path through the thermocline,
// the 62 dB budget reflects that we are only fighting one-way spreading loss, 20·log10(100) = 40 dB, so 62 gives comfortable margin
// that passive reaches a bit further than active, which is realistic (passive has no two-way loss penalty)
static float passiveSignalStrength(float rangeKilometers, float thermoclineNormalized, float deepSpeedBoost)
{
    if (rangeKilometers < 0.1f)
        return 1.f;
    // passive having half the path through the thermocline means half the refraction loss, hence the 0.5 factor
    float transmissionLoss = 20.f * std::log10(rangeKilometers) + shadowLossDecibels(rangeKilometers, thermoclineNormalized, deepSpeedBoost) * 0.5f;
    return std::max(0.f, (62.f - transmissionLoss) / 62.f);
}

// spawns a target at a random position and heading somewhere in the operational area
static Target makeTarget(int colorId)
{
    float angle = (std::rand() % 3600) * (float)M_PI / 1800.f; // 0 to 2pi, random spawn direction
    float distance = 15.f + std::rand() % 70; // 15 to 85 km from center
    float speed = 1.2f + (std::rand() % 1000) * 0.001f * 3.2f; // 1.2 to 4.4 km/s
    float course = (std::rand() % 3600) * (float)M_PI / 1800.f; // random initial heading
    return { std::sin(angle) * distance, std::cos(angle) * distance, // initial position
        std::sin(course) * speed, std::cos(course) * speed, // initial velocity
        colorId, -999.f };
}

// adjusts the live target list to match the desired count without rebuilding everything
static void setTargetCount(SonarState& sonarState, int count)
{
    count = std::clamp(count, 1, 10);
    while ((int)sonarState.targets.size() < count)
        sonarState.targets.push_back(makeTarget((int)sonarState.targets.size()));
    while ((int)sonarState.targets.size() > count)
        sonarState.targets.pop_back();
    sonarState.targetCount = count;
}

#pragma region updates
// moves every target forward by one time step using dead-reckoning (position += velocity * time),
// "dead reckoning" is the nautical term for estimating current position based on last known position plus speed and heading
// speedScale ties target movement to the sweep speed so faster sweeps feel like a more active simulation,
// if a target drifts past 88% of the maximum range it is near the edge of the display,
// so we redirect its heading back toward center with up to 30° of random wobble to keep contacts visible
static void updateTargets(SonarState& sonarState, float deltaTime)
{
    float speedScale = sonarState.sweepSpeedDegreesPerSecond / 90.f; // 1.0 at default 90 dps
    for (auto& target : sonarState.targets) {
        target.xKilometers += target.velocityX * speedScale * deltaTime;
        target.yKilometers += target.velocityY * speedScale * deltaTime;
        float distance = std::hypot(target.xKilometers, target.yKilometers); // straight-line dist from center
        if (distance > maximumRangeKilometers * 0.88f) {
            // atan2 of the negated position gives the inward-pointing bearing (back toward center),
            // then add up to 30 degrees of random wobble so targets don't all funnel to the same spot
            float angle = std::atan2(-target.xKilometers, -target.yKilometers) + ((std::rand() % 60) - 30) * DEG2RAD;
            float speed = std::hypot(target.velocityX, target.velocityY); // preserve the target's current speed
            target.velocityX = std::sin(angle) * speed;
            target.velocityY = std::cos(angle) * speed;
        }
    }
}

// for each target, checks whether the sweep arm just crossed its bearing, if so, records an echo (active mode) or a directional hit (passive mode)
// detection only registers if the arm's arc this frame covered that bearing + enough time has passed since the last detection on this target
// (prevents re-firing every frame),
// active mode -> compute screen pixel position from bearing + range, push a fading blip dot at that location,
// passive mode -> map bearing to one of the 720 angular bins and light that bin up,
static void updateDetections(SonarState& sonarState, Vector2 center, float radius)
{
    // one full revolution takes 360/sweepSpeed seconds, require 85% of that before re-detecting the same target
    // this mimics real PPI behavior where a contact appears once per sweep, not continuously on every frame
    float revolutionSeconds = 360.f / sonarState.sweepSpeedDegreesPerSecond;
    float minimumInterval = revolutionSeconds * 0.85f;

    for (auto& target : sonarState.targets) {
        float rangeKilometers = std::hypot(target.xKilometers, target.yKilometers);
        // too close (inside own-ship noise floor) or too far (off the display)
        if (rangeKilometers < 0.5f || rangeKilometers > maximumRangeKilometers)
            continue;

        // reverse of bearingToDirection, given a position in (x km east, y km north) relative to our ship, compute the compass bearing to it,
        // atan2(x, y) gives the clockwise-from-north angle in radians (x first then y is deliberate, standard atan2 is from-east,
        // we want from-north so we swap), the +360 then fmod(360) ensures the result is always in [0°, 360°) with no negative values
        // ex: a target at (1, 0) due east -> bearing 90°,  target at (0, -1) due south -> bearing 180°
        float bearing = std::fmod(std::atan2(target.xKilometers, target.yKilometers) * RAD2DEG + 360.f, 360.f);

        if (!bearingInArc(bearing, sonarState.previousSweepDegrees, sonarState.sweepAngleDegrees))
            continue;
        if (sonarState.elapsedSeconds - target.lastDetectionTime < minimumInterval)
            continue; // too soon since the last ping on this target
        target.lastDetectionTime = sonarState.elapsedSeconds;

        Color color = targetColors[target.colorId];

        if (sonarState.activeMode) {
            float signalStrength = activeSignalStrength(rangeKilometers, sonarState.thermoclineNormalized, sonarState.deepSpeedBoost);
            if (signalStrength <= 0.f)
                continue; // echo too weak to register on the display
            Vector2 direction = bearingToDirection(bearing);
            // scale range in km to screen pixels: 100 km maps to the full disc radius in pixels
            float rangePixels = (rangeKilometers / maximumRangeKilometers) * radius;
            sonarState.blips.push_back({ center.x + direction.x * rangePixels, center.y + direction.y * rangePixels, 1.0f, color });
        } else {
            float signalStrength = passiveSignalStrength(rangeKilometers, sonarState.thermoclineNormalized, sonarState.deepSpeedBoost);
            if (signalStrength <= 0.f)
                continue;
            // map bearing (0-360) to one of 720 bins by simple proportion, ex: bearing 180 deg -> bin 360
            int binId = (int)(bearing / 360.f * passiveBinCount) % passiveBinCount;
            sonarState.passiveBins[binId].alpha = 1.0f;
            sonarState.passiveBins[binId].color = color;
        }
    }
}

// advances the sweep arm angle, fades out old blips and passive bins, then moves targets and checks for new detections,
// fade rate is proportional to sweep speed: faster sweep -> contacts fade faster, keeping the display
// consistent regardless of rotation speed (a full revolution always clears the previous contacts),
// the erase-remove_if pattern is the standard C++ idiom for deleting items from a vector in one pass
// mark the ones to remove (alpha < 0.01), shift everything else forward, then truncate the tail
static void updateSonar(SonarState& sonarState, float deltaTime, Vector2 center, float radius)
{
    sonarState.elapsedSeconds += deltaTime;
    sonarState.previousSweepDegrees = sonarState.sweepAngleDegrees;
    sonarState.sweepAngleDegrees = std::fmod(sonarState.sweepAngleDegrees + sonarState.sweepSpeedDegreesPerSecond * deltaTime, 360.f);

    // at 90 dps: fadeRate = 0.25/s -> a blip at alpha 1.0 fully disappears after 4 seconds (one revolution),
    // so the natural "sweep lifetime" of a contact exactly matches one full rotation
    float fadeRate = sonarState.sweepSpeedDegreesPerSecond / 360.f;

    for (auto& blip : sonarState.blips)
        blip.alpha = std::max(0.f, blip.alpha - fadeRate * deltaTime);
    sonarState.blips.erase(
        std::remove_if(sonarState.blips.begin(), sonarState.blips.end(), [](const Blip& blip) { return blip.alpha < 0.01f; }), sonarState.blips.end());

    for (auto& passiveBin : sonarState.passiveBins)
        passiveBin.alpha = std::max(0.f, passiveBin.alpha - fadeRate * deltaTime);

    updateTargets(sonarState, deltaTime);
    updateDetections(sonarState, center, radius);
}

#pragma region draw utils

// horizontal clickable slider: a label above it, a dark bar with a colored fill proportional to value,
// and the current numeric value to the right, if the user clicks or drags inside the bar, we read the mouse x position,
// map it linearly into [valueMin, valueMax] and return the new value, otherwise we return the unchanged value,
static float drawSlider(float x, float y, float width, const char* label, float value, float valueMin, float valueMax, Color color)
{
    DrawText(label, (int)x, (int)(y - 15), 12, ColorAlpha(WHITE, 0.7f));
    Rectangle sliderRect = { x, y, width, 14.f };
    DrawRectangleRec(sliderRect, ColorAlpha(BLACK, 0.45f));
    // filled portion: totalWidth * normalizedValue, where normalizedValue = (value-min)/(max-min) in [0,1]
    DrawRectangle((int)sliderRect.x, (int)sliderRect.y, (int)(sliderRect.width * ((value - valueMin) / (valueMax - valueMin))), (int)sliderRect.height, color);
    DrawRectangleLinesEx(sliderRect, 1, ColorAlpha(WHITE, 0.2f));
    char valueText[16];
    std::snprintf(valueText, sizeof(valueText), "%.1f", value);
    DrawText(valueText, (int)(x + width + 6), (int)y, 11, ColorAlpha(WHITE, 0.6f));
    if (CheckCollisionPointRec(GetMousePosition(), sliderRect) && IsMouseButtonDown(MOUSE_BUTTON_LEFT))
        return std::clamp(valueMin + (GetMousePosition().x - sliderRect.x) / sliderRect.width * (valueMax - valueMin), valueMin, valueMax);
    return value;
}

// read-only data display row: a label above and the value right-aligned inside a dark box,
// used for the bearing and range readouts that reflect what the cursor points at on the PPI
static void drawReadout(float x, float y, float width, const char* label, const char* value)
{
    DrawText(label, (int)x, (int)(y - 15), 12, ColorAlpha(WHITE, 0.7f));
    Rectangle readoutRect = { x, y, width + 20.f, 14.f };
    DrawRectangleRec(readoutRect, ColorAlpha(BLACK, 0.35f));
    DrawRectangleLinesEx(readoutRect, 1, ColorAlpha(WHITE, 0.12f));
    int textWidth = MeasureText(value, 12);
    DrawText(value, (int)(readoutRect.x + readoutRect.width - textWidth - 6), (int)(readoutRect.y + 1), 12, { 0, 220, 0, 220 });
}

#pragma region draws
// PPI = Plan Position Indicator, the classic round green sonar scope, ship is always at the exact center,
// the sweep arm rotates clockwise, whatever the arm "illuminates" on each pass gets painted on screen and then slowly fades
// this function draws in order: the dark green disc background, the fading contacts (blips or passive lines),
// the concentric range rings (25/50/75/100 km), the cardinal direction labels (N/S/E/W),
// the faint crosshair, the bright green sweep arm, and then updates the mouse cursor readout for the UI panel
static void drawPPI(SonarState& sonarState, Vector2 center, float radius)
{
    DrawCircleV(center, radius, { 0, 15, 0, 255 }); // dark green phosphor CRT screen effect

    // passive mode: draw a radial line from center to the disc edge for each lit bin,
    // the line goes all the way to the edge because passive sonar has no range information
    if (!sonarState.activeMode) {
        for (int binId = 0; binId < passiveBinCount; ++binId) {
            if (sonarState.passiveBins[binId].alpha < 0.01f)
                continue;
            float bearing = binId * 360.f / passiveBinCount;
            Vector2 direction = bearingToDirection(bearing);
            Vector2 endpoint = { center.x + direction.x * radius, center.y + direction.y * radius };
            Color color = sonarState.passiveBins[binId].color;
            color.a = (unsigned char)(sonarState.passiveBins[binId].alpha * 235);
            DrawLineV(center, endpoint, color);
        }
    }

    // active mode: draw each echo as a small circle whose radius shrinks as it fades,
    if (sonarState.activeMode) {
        for (const auto& blip : sonarState.blips) {
            Color color = blip.color;
            color.a = (unsigned char)(blip.alpha * 240);
            // size: starts at 6px (4.5+1.5) when fresh, shrinks to 1.5px when nearly invisible
            DrawCircleV({ blip.xPixels, blip.yPixels }, 4.5f * blip.alpha + 1.5f, color);
        }
    }

    // range rings: concentric circles at 25, 50, 75, 100 km
    for (int rangeKilometers : { 25, 50, 75, 100 }) {
        float rangePixels = (rangeKilometers / maximumRangeKilometers) * radius;
        DrawCircleLines((int)center.x, (int)center.y, rangePixels, { 0, 80, 0, 170 });
        char label[8];
        std::snprintf(label, sizeof(label), "%dkm", rangeKilometers);
        int textWidth = MeasureText(label, 11);
        // label sits just inside the ring on the east side, shifted left by its own width so it does not overlap
        DrawText(label, (int)(center.x + rangePixels) - textWidth - 5, (int)center.y - 15, 11, { 0, 130, 0, 200 });
    }

    // cardinal compass labels just outside the disc edge (N/S/E/W),
    // cardinalMargin pushes them beyond the radius so they sit clearly outside the green circle
    float cardinalMargin = 16.f;
    DrawText("N", center.x - 6, center.y - radius - cardinalMargin, 13, { 0, 200, 0, 255 });
    DrawText("S", center.x - 5, center.y + radius + cardinalMargin - 13, 13, { 0, 200, 0, 255 });
    DrawText("E", center.x + radius + cardinalMargin, center.y - 6, 13, { 0, 200, 0, 255 });
    DrawText("W", center.x - radius - cardinalMargin - 12, center.y - 6, 13, { 0, 200, 0, 255 });

    // faint crosshair lines through center for orientation (north-south and east-west axes)
    DrawLine(center.x, center.y - radius, center.x, center.y + radius, { 0, 50, 0, 100 });
    DrawLine(center.x - radius, center.y, center.x + radius, center.y, { 0, 50, 0, 100 });

    // the rotating sweep arm: a bright green line from center out to the disc edge at the current angle
    Vector2 sweepDirection = bearingToDirection(sonarState.sweepAngleDegrees);
    Vector2 sweepTip = { center.x + sweepDirection.x * radius, center.y + sweepDirection.y * radius };
    DrawLineV(center, sweepTip, { 0, 255, 80, 210 });

    // convert the cursor's screen pixel offset from center into bearing and range,
    // atan2(dx, -dy) gives clockwise-from-north angle in screen space (dy is negated because screen y is flipped
    // positive y goes down, but north is up), result wrapped to [0, 360) with fmod+offset,
    // this is updated here so the UI panel can read the values without extra passing
    Vector2 mousePosition = GetMousePosition();
    float mouseDeltaX = mousePosition.x - center.x;
    float mouseDeltaY = mousePosition.y - center.y;
    float mouseDistancePixels = std::hypot(mouseDeltaX, mouseDeltaY);
    sonarState.mouseOnPPI = mouseDistancePixels <= radius;
    if (sonarState.mouseOnPPI) {
        sonarState.mouseRange = (mouseDistancePixels / radius) * maximumRangeKilometers;
        sonarState.mouseBearing = std::fmod(std::atan2(mouseDeltaX, -mouseDeltaY) * RAD2DEG + 360.f, 360.f);
    }
}

// scientific chart showing how fast sound travels at each depth for the current ocean settings,
// horizontal axis = speed (1420–1630 m/s), vertical axis = depth (top = surface, bottom = deep),
// the curve is built from 100 short connected line segments computed by sampling soundSpeedMetersPerSecond,
// the orange horizontal line shows the thermocline, the kink in the curve appears right at that depth,
// helps the operator understand why distant contacts are harder to detect
// the dip in the curve at the thermocline is exactly where sound bends away and creates the shadow zone
static void drawSoundSpeedProfile(const SonarState& sonarState, float x, float y, float width, float height)
{
    DrawText("SOUND SPEED PROFILE", (int)x, (int)(y - 15), 12, ColorAlpha(WHITE, 0.7f));
    DrawRectangle((int)x, (int)y, (int)width, (int)height, ColorAlpha(BLACK, 0.5f));
    DrawRectangleLinesEx({ x, y, width, height }, 1, ColorAlpha(SKYBLUE, 0.35f));

    constexpr float minimumSpeedAxis = 1420.f;
    constexpr float maximumSpeedAxis = 1630.f;
    constexpr float speedAxisRange = maximumSpeedAxis - minimumSpeedAxis;

    float leftEdge = x + 36.f, rightEdge = x + width - 8.f;
    float topEdge = y + 20.f, bottomEdge = y + height - 18.f;

    DrawText("1420", (int)(leftEdge - 4), (int)(bottomEdge - 8), 9, ColorAlpha(WHITE, 0.35f));
    DrawText("1630", (int)(rightEdge - 22), (int)(bottomEdge - 8), 9, ColorAlpha(WHITE, 0.35f));
    DrawText("m/s", (int)(x + 4), (int)(bottomEdge - 8), 9, ColorAlpha(WHITE, 0.3f));
    DrawText("shallow", (int)(x + 4), (int)topEdge, 9, ColorAlpha(WHITE, 0.35f));
    DrawText("deep", (int)(x + 4), (int)(bottomEdge - 18), 9, ColorAlpha(WHITE, 0.35f));

    // piecewise linear curve: sample depth at 100 evenly spaced points, compute sound speed at each,
    // map both depth (0 to 1 -> topEdge to bottomEdge) and speed (1420 to 1630 -> leftEdge to rightEdge) to pixels,
    // then draw each consecutive pair as a short line, 100 segments is visually smooth enough
    constexpr int segmentCount = 100;
    for (int segmentId = 1; segmentId <= segmentCount; ++segmentId) {
        float depthStart = (segmentId - 1) / (float)segmentCount;
        float depthEnd = segmentId / (float)segmentCount;
        float speedStart = soundSpeedMetersPerSecond(depthStart, sonarState.thermoclineNormalized, sonarState.deepSpeedBoost);
        float speedEnd = soundSpeedMetersPerSecond(depthEnd, sonarState.thermoclineNormalized, sonarState.deepSpeedBoost);
        // (speed - axisMin) / axisRange maps m/s value to a 0 to 1 position, then scale to pixel width
        float pixelXStart = leftEdge + (speedStart - minimumSpeedAxis) / speedAxisRange * (rightEdge - leftEdge);
        float pixelYStart = topEdge + depthStart * (bottomEdge - topEdge);
        float pixelXEnd = leftEdge + (speedEnd - minimumSpeedAxis) / speedAxisRange * (rightEdge - leftEdge);
        float pixelYEnd = topEdge + depthEnd * (bottomEdge - topEdge);
        DrawLineV({ pixelXStart, pixelYStart }, { pixelXEnd, pixelYEnd }, SKYBLUE);
    }

    // orange line marking the thermocline depth, the curve kinks visibly right here
    float thermoclineY = topEdge + sonarState.thermoclineNormalized * (bottomEdge - topEdge);
    DrawLine((int)(x + 2), (int)thermoclineY, (int)(x + width - 2), (int)thermoclineY, ColorAlpha(ORANGE, 0.65f));
    DrawText("thermo", (int)(x + 4), (int)(thermoclineY - 10), 9, ColorAlpha(ORANGE, 0.85f));
}

static void drawUI(SonarState& sonarState, int panelWidth, int screenHeight)
{
    DrawRectangle(0, 0, panelWidth, screenHeight, ColorAlpha(BLACK, 0.65f)); // translucent dark panel background

    float x = 15.f;
    float y = 18.f;
    float sliderWidth = (float)panelWidth - 50.f;
    // each row = 15px label + 14px bar + 19px gap = 48px
    constexpr float rowHeight = 48.f;

    // bearing readout
    {
        char valueText[16];
        if (sonarState.mouseOnPPI)
            std::snprintf(valueText, sizeof(valueText), "%06.2f deg", sonarState.mouseBearing);
        else
            std::strcpy(valueText, "---.-- deg");
        drawReadout(x, y, sliderWidth, "BEARING", valueText);
        y += rowHeight;
    }

    // range readout
    {
        char valueText[16];
        if (sonarState.mouseOnPPI)
            std::snprintf(valueText, sizeof(valueText), "%06.2f km", sonarState.mouseRange);
        else
            std::strcpy(valueText, "---.-- km");
        drawReadout(x, y, sliderWidth, "RANGE", valueText);
        y += rowHeight;
    }

    // mode toggle button
    y += 4.f;
    Rectangle modeToggle = { x, y, sliderWidth + 20.f, 28.f };
    DrawRectangleRec(modeToggle, sonarState.activeMode ? Color { 25, 100, 25, 255 } : Color { 40, 40, 90, 255 });
    DrawRectangleLinesEx(modeToggle, 1, ColorAlpha(WHITE, 0.25f));
    DrawText(sonarState.activeMode ? "MODE: ACTIVE" : "MODE: PASSIVE", (int)(modeToggle.x + 10), (int)(modeToggle.y + 8), 13, WHITE);
    if (CheckCollisionPointRec(GetMousePosition(), modeToggle) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
        sonarState.activeMode = !sonarState.activeMode;
    y += 28.f + rowHeight - 14.f;

    // sweep speed slider from 10 degrees to a full revolution per sec
    sonarState.sweepSpeedDegreesPerSecond
        = drawSlider(x, y, sliderWidth, "SWEEP  deg/s", sonarState.sweepSpeedDegreesPerSecond, 10.f, 360.f, { 30, 175, 30, 255 });
    y += rowHeight;

    // targets slider from 1 to 10 (rounded)
    {
        DrawText("TARGETS", (int)x, (int)(y - 15), 12, ColorAlpha(WHITE, 0.7f));
        Rectangle targetRect = { x, y, sliderWidth, 14.f };
        DrawRectangleRec(targetRect, ColorAlpha(BLACK, 0.45f));
        DrawRectangle(
            (int)targetRect.x, (int)targetRect.y, (int)(targetRect.width * (sonarState.targetCount - 1) / 9.f), (int)targetRect.height, { 180, 100, 30, 255 });
        DrawRectangleLinesEx(targetRect, 1, ColorAlpha(WHITE, 0.2f));
        char valueText[4];
        std::snprintf(valueText, sizeof(valueText), "%d", sonarState.targetCount);
        DrawText(valueText, (int)(x + sliderWidth + 6), (int)y, 11, ColorAlpha(WHITE, 0.6f));
        if (CheckCollisionPointRec(GetMousePosition(), targetRect) && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            int newCount = std::clamp(1 + (int)std::round((GetMousePosition().x - targetRect.x) / targetRect.width * 9.f), 1, 10);
            if (newCount != sonarState.targetCount)
                setTargetCount(sonarState, newCount);
        }
        y += rowHeight;
    }

    // thermocline depth slider, moves the temperature boundary layer up or down the water column,
    // 0.05 = very shallow thermocline (shadow zone starts close, most of the range area is affected),
    // 0.95 = very deep (nearly the entire depth is warm surface water, almost no shadow zone)
    sonarState.thermoclineNormalized
        = drawSlider(x, y, sliderWidth, "THERMOCLINE DEPTH", sonarState.thermoclineNormalized, 0.05f, 0.95f, { 255, 160, 30, 255 });
    y += rowHeight;

    // deep speed boost slider, how strongly sound speeds back up below the thermocline (SOFAR channel strength),
    // 0.0 = no recovery (deep water stays slow, shadow zone loss is severe),
    // 1.0 = strong SOFAR channel (deep speed nearly recovers to surface level, shadow loss nearly cancels out)
    sonarState.deepSpeedBoost = drawSlider(x, y, sliderWidth, "DEEP SPEED BOOST", sonarState.deepSpeedBoost, 0.f, 1.f, { 100, 200, 255, 255 });
    y += rowHeight + 8.f;

    // fills whatever vertical space remains at the bottom of the panel,
    // minimum 60px to remain legible, capped at 150px so it does not crowd the sliders above
    float chartHeight = std::min(150.f, (float)screenHeight - y - 12.f);
    if (chartHeight > 60.f)
        drawSoundSpeedProfile(sonarState, x, y, (float)(panelWidth - 25), chartHeight);
}

#pragma region main
int main()
{
    const int panelWidth = 220;
    const int screenHeight = 900;
    const float radius = (screenHeight - 80.f) / 2.f; // 410 px
    const int screenWidth = panelWidth + (int)(2.f * radius) + 60; // panel + disc diameter + margins
    const Vector2 center = { panelWidth + (screenWidth - panelWidth) / 2.f, screenHeight / 2.f };

    InitWindow(screenWidth, screenHeight, "SONAR PPI");
    SetTargetFPS(120);

    SonarState sonarState;
    setTargetCount(sonarState, sonarState.targetCount); // spawn the initial 5 targets

    while (!WindowShouldClose()) {
        float deltaTime = GetFrameTime();
        updateSonar(sonarState, deltaTime, center, radius);

        BeginDrawing();
        ClearBackground(BLACK);
        drawPPI(sonarState, center, radius); // drawn first so UI panel overlays on top
        drawUI(sonarState, panelWidth, screenHeight);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}