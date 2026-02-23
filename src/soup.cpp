#include "soup.hpp"

// NDC (Normalized Device Coordinates) is the GPU's native coordinate system:
// X and Y both go from -1 to +1, covering the entire screen regardless of actual resolution
// top-left corner is (-1, +1), bottom-right is (+1, -1), center is (0, 0)
// by placing our quad corners at the NDC extremes we cover every pixel, making the fragment
// shader run on the whole screen in one draw call (bypasses all raylib cameras and projections entirely)
// each rlTexCoord2f/rlVertex2f pair defines one corner, becomes fragCoord in the vertex shader
static void drawFullscreenQuad()
{
    rlBegin(RL_QUADS);
    {
        rlTexCoord2f(0.0f, 0.0f);
        rlVertex2f(-1.0f, 1.0f);
        rlTexCoord2f(0.0f, 1.0f);
        rlVertex2f(-1.0f, -1.0f);
        rlTexCoord2f(1.0f, 1.0f);
        rlVertex2f(1.0f, -1.0f);
        rlTexCoord2f(1.0f, 0.0f);
        rlVertex2f(1.0f, 1.0f);
    }
    rlEnd();
}

// random particles spawn using https://en.wikipedia.org/wiki/Mersenne_Twister
void initSimulation(Particles& particles, int screenWidth, int screenHeight)
{
    std::mt19937 rng(std::random_device {}());
    std::uniform_real_distribution<float> random(0.0f, static_cast<float>(screenWidth));

    for (int index = 0; index < numParticles; index++) {
        particles.posX[index] = random(rng);
        particles.posY[index] = random(rng);
        particles.velX[index] = particles.velY[index] = 0;
        particles.species[index] = index / perSpecies;
    }
}

// for each particle , we compute forces from all other particles simultaneously
// using Eigen array operations on the inner dimension (Eigen's ArrayXf maps
// directly onto SIMD registers, AVX2 = 8 floats per instruction so the
// compiler processes 8 particles per cycle instead of 1
//  toroidal wrap: particles see an infinite world as exiting on one edge makes you appear on the other
// ("toroidal" for torus/donut shape, opposite edges are connected)
void updateSimulation(Particles& particles, float deltaTime, float screenWidth, float screenHeight)
{
    // "where, at which %, is rMin (particle edge) from 0 (particle center) to rMax (detection field edge)"
    const float beta = rMin / rMax;
    // calculate const values before the loop for perf
    const float inverseBeta = 1.0f / beta;
    const float triangleDenominator = 1.0f - beta;
    const float rMaxSquared = rMax * rMax;
    const float drag = 1.0f - friction;

    // will contain the total force pushing each particle left/right or up/down from all its neighbors combined
    Eigen::ArrayXf forceX = Eigen::ArrayXf::Zero(numParticles);
    Eigen::ArrayXf forceY = Eigen::ArrayXf::Zero(numParticles);

    for (int i = 0; i < numParticles; i++) {
        // distance from self to every other particle, all at once (SIMD)
        Eigen::ArrayXf deltaX = particles.posX - particles.posX[i];
        Eigen::ArrayXf deltaY = particles.posY - particles.posY[i];

        // find the shortest-path displacement through periodic boundaries, for example in a world of width 100
        // particle A at 5 and B at 95, distance 90, but since it wraps it's actually distance 10 so
        // delta -= round(delta / worldSize) * worldSize with delta 90 and worldSize 100, delta become -10 (good)
        // basically if delta is bigger than half of worldSize then they'll take the wrap way (hence round)
        deltaX -= (deltaX / screenWidth).round() * screenWidth;
        deltaY -= (deltaY / screenHeight).round() * screenHeight;

        // https://en.wikipedia.org/wiki/Pythagorean_theorem combine X and Y into real distance
        // keep it squared because comparing avoids square root (which is slow, comparing squared distances is faster)
        Eigen::ArrayXf distanceSquared = deltaX * deltaX + deltaY * deltaY;

        // hard minimum distance prevents force from dividing by 0 (since self has distance 0 from itself)
        // calculate, for later since division is expensive, the reciprocal of every distance and
        // "at which % is distance from 0 (particle center) to rMax (detection field edge)"
        Eigen::ArrayXf distance = distanceSquared.sqrt().max(1e-6f);
        Eigen::ArrayXf inverseDistance = 1.0f / distance;
        Eigen::ArrayXf normalizedDistance = distance / rMax;

        // active mask, 0 or 1 for considering only particles within rMax and not self
        Eigen::ArrayXf activeMask = (distanceSquared < rMaxSquared && distanceSquared > 1e-6f).cast<float>();

        // inner mask, 0 or 1 for considering only particles within rMin
        Eigen::ArrayXf innerMask = (normalizedDistance < beta).cast<float>();
        // if normalizedDistance = 0 then repulsionForce is -repulsionScale, if normalizedDistance = beta (rMin) then 0
        // inbetween is linear, closer you are, harder it pushes back
        Eigen::ArrayXf repulsionForce = (normalizedDistance * inverseBeta - 1.0f) * repulsionScale;

        // outer mask, 0 or 1 for considering only particles between rMin and rMax
        Eigen::ArrayXf outerMask = (normalizedDistance >= beta && normalizedDistance < 1.0f).cast<float>();

        // for each neighbor, look up how self's species reacts to that neighbor's species
        Eigen::ArrayXf matrixValues(numParticles);
        for (int j = 0; j < numParticles; j++) {
            matrixValues[j] = forceMatrix[particles.species[i]][particles.species[j]] * forceScale;
        }

        // https://en.wikipedia.org/wiki/Triangle_wave, shapes how strongly the matrix force applies
        // across the outer zone. at the inner edge (normalizedDistance = beta), wave = 0 (force fades to zero smoothly)
        // at the midpoint, wave = 1 (full matrix force), at the outer edge (normalizedDistance = 1.0), wave = 0 again
        // force builds up as particles approach from far away, peaks at medium range, then fades as they get very close
        // (where the repulsion zone takes over instead), this prevents abrupt force jumps at zone boundaries
        Eigen::ArrayXf triangleWave = 1.0f - (1.0f + beta - 2.0f * normalizedDistance).abs() / triangleDenominator;
        Eigen::ArrayXf interactionForce = matrixValues * triangleWave;

        // put it all together (so either innerMask or outerMask is 1, the other 0, unless both 0 if activeMask 0)
        Eigen::ArrayXf totalForce = activeMask * (innerMask * repulsionForce + outerMask * interactionForce);

        // convert magnitude into direction, deltaX * inverseDistance = deltaX / distance = the unit vector pointing from self
        // toward the neighbor multiply by totalForce magnitude = force vector, sum over all neighbors = total force on
        // particle i positive forceX means net push to the right, negative means left (same for up/down)
        forceX[i] = (totalForce * deltaX * inverseDistance).sum();
        forceY[i] = (totalForce * deltaY * inverseDistance).sum();
    }

    // shrinks existing velocity then new force adds to it, for all particles at once (SIMD)
    // without friction particles would accelerate forever, without force friction would bring everything to a stop
    // the balance between the two creates the perpetual-motion-without-explosion feel
    particles.velX = particles.velX * (1.0f - friction) + forceX * deltaTime;
    particles.velY = particles.velY * (1.0f - friction) + forceY * deltaTime;

    // branchless (no if block since expensive) maxSpeed cap with pythagorean, 1e-6f clamp to prevent divide by 0
    // and clamp 1.0 to not touch particles with valid speed
    Eigen::ArrayXf speed = (particles.velX * particles.velX + particles.velY * particles.velY).sqrt().max(1e-6f);
    Eigen::ArrayXf speedScale = (maxSpeed / speed).min(1.0f);
    particles.velX *= speedScale;
    particles.velY *= speedScale;

    // move every particle, velocity is in pixels/second, deltaTime is seconds elapsed since last frame (≈0.016 at
    // 60fps) so a particle moving at 300px/s moves 300 * 0.016 = 4.8px per frame
    particles.posX += particles.velX * deltaTime;
    particles.posY += particles.velY * deltaTime;

    // toroidal position wrap, in the same previous example, if particle A is at 107 then it'll reappear at 7
    // basically if position is bigger than worldSize or negative then it'll take the wrap way (hence floor)
    particles.posX -= (particles.posX / screenWidth).floor() * screenWidth;
    particles.posY -= (particles.posY / screenHeight).floor() * screenHeight;
}

int main()
{
    SetConfigFlags(FLAG_FULLSCREEN_MODE | FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
    InitWindow(0, 0, "Soup");
    SetTargetFPS(60);

    char fragmentSource[1024 * 20]; //20KB
    snprintf(fragmentSource, sizeof(fragmentSource), fragShaderTemplate, numParticles, discRadius, discRadius, discRadius);
    Shader shader = LoadShaderFromMemory(vertexShader, fragmentSource);

    // inform shader of screenSize as an uniform, can't be injected like above because not a constant
    const float screenWidth = GetScreenWidth();
    const float screenHeight = GetScreenHeight();
    float screenSize[2] = { screenWidth, screenHeight };
    SetShaderValue(shader, GetShaderLocation(shader, "screenSize"), screenSize, SHADER_UNIFORM_VEC2);

    // particleData sampler reads from texture unit 0 (raylib's default slot)
    int textureUnit = 0;
    SetShaderValue(shader, GetShaderLocation(shader, "particleData"), &textureUnit, SHADER_UNIFORM_INT);

    int timeLoc = GetShaderLocation(shader, "time");

    Particles particles(numParticles);
    initSimulation(particles, screenWidth, screenHeight);

    // one RGBA32F texture, numParticles wide, 2 rows tall
    // each channel is a 32-bit float, 4 channels per texel, we need 4 channels even though we only use 2-3
    // because RGBA32F is the float format rlgl exposes
    static float particleData[numParticles * 4 * 2];

    // upload row 1 (colors) once, species don't change at runtime
    for (int index = 0; index < numParticles; index++) {
        int species = particles.species[index];
        int colorRow = (numParticles + index) * 4; // row 1 starts at offset numParticles*4
        particleData[colorRow + 0] = speciesColor[species][0];
        particleData[colorRow + 1] = speciesColor[species][1];
        particleData[colorRow + 2] = speciesColor[species][2];
        particleData[colorRow + 3] = 1.0f;
    }

    unsigned int particleTexId
        = rlLoadTexture(particleData, numParticles, 2, RL_PIXELFORMAT_UNCOMPRESSED_R32G32B32A32, 1);

    // MIN_FILTER = what to do when the texture is displayed smaller than its actual size
    // MAG_FILTER = what to do when displayed larger (magnification)
    // NEAREST = snap to the exact nearest texel, no blending (important for our texel-encoded positions)
    rlTextureParameters(particleTexId, RL_TEXTURE_MIN_FILTER, RL_TEXTURE_FILTER_NEAREST);
    rlTextureParameters(particleTexId, RL_TEXTURE_MAG_FILTER, RL_TEXTURE_FILTER_NEAREST);

    static float positionRowData[numParticles * 4];

    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_ESCAPE))
            break;

        // if the window loses focus/alt-tab/system lags, GetFrameTime() could return more than it should and
        // physics would then simulate too much in one step and break, so clamp from 60 to 30fps just in case
        float deltaTime = std::min(GetFrameTime(), 1.0f / 30.0f);
        updateSimulation(particles, deltaTime, screenWidth, screenHeight);

        // pack eigen SoA positions into row 0 of the texture data
        for (int index = 0; index < numParticles; index++) {
            positionRowData[index * 4 + 0] = particles.posX[index];
            positionRowData[index * 4 + 1] = particles.posY[index];
            positionRowData[index * 4 + 2] = 0.0f;
            positionRowData[index * 4 + 3] = 0.0f;
        }

        // upload row 0 (positions) only, row 1 (colors) never changes
        rlUpdateTexture(
            particleTexId, 0, 0, numParticles, 1, RL_PIXELFORMAT_UNCOMPRESSED_R32G32B32A32, positionRowData);

        float currentTime = GetTime();
        SetShaderValue(shader, timeLoc, &currentTime, SHADER_UNIFORM_FLOAT);

        BeginDrawing();
        {
            BeginShaderMode(shader);
            {
                // rlSetTexture registers our data texture with raylib's batch renderer
                // and binds it in OpenGL, ithout this, the batch renderer would
                // overwrite our binding with its own default texture when it flushes
                rlSetTexture(particleTexId);
                drawFullscreenQuad();
                rlSetTexture(0);
            }
            EndShaderMode();
        }
        EndDrawing();
    }
    rlUnloadTexture(particleTexId);
    UnloadShader(shader);
    CloseWindow();
    return 0;
}