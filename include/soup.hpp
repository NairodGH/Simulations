#include "main.hpp"

static constexpr int numSpecies = 3;
static constexpr int perSpecies = 500;
static constexpr int numParticles = numSpecies * perSpecies;

// radius of particles body in pixels
static constexpr float discRadius = 3.0f;

// 0 -> rMin = repulsion zone (species-blind spring pushback), force = (distNorm/beta - 1) * repulsionScale
// distNorm is "what fraction of the maximum range (rMax) am I at ?"
// at distNorm=0 (contact) full -repulsionScale push, at distNorm=rMin force = 0, zone ends
// low (5) = soft/particles pass through
// high (80) = hard collision/particles bounce
// should be about discRadius*2 + ~glowSigma so there's 1 glow gap between particles so you can always visually distinguish them
static constexpr float rMin = 15.0f;
// rMin -> rMax = interaction zone (cyclic dominance matrix), + is attract (predator), - is repel (prey)
// low (60) = particles only interact when close/isolated small clusters
// high (300) = particles see far/more global behavior/heavy on perf
static constexpr float rMax = 200.0f;

// velocity damping applied every frame as vel *= (1 - friction)
// low (0.01) = particles slide like ice/clusters orbit endlessly
// high (0.15) = particles stop quickly/clusters freeze
static constexpr float friction = 0.035f;

// scales all matrix forces for convenience (forceMatrix encodes relationships, this encodes their intensity)
// low (5) = weak pull/particles drift loosely
// high (40) = violent snap together/clusters form instantly and lock up
static constexpr float forceScale = 25.0f;

// inner-zone repulsion multiplier, larger than forceScale so high-speed particles cant phase through each other
static constexpr float repulsionScale = 250.0f;

// hard speed cap in pixels/second, without this a particle attracted by
// 20 neighbors simultaneously accelerates to infinity
// low (100) = sluggish, high (600) = chaotic
static constexpr float maxSpeed = 300.0f;

// hunt > |flee| so predators chase faster than preys escape = perpetual motion (as long as friction low enough)
// self is for same species cohesion
static constexpr float hunt = 0.75f;
static constexpr float flee = -0.25f;
static constexpr float self = 1.0f;

static constexpr float forceMatrix[numSpecies][numSpecies] = {
    { self, hunt, flee }, // red:   hunts green, flees blue
    { flee, self, hunt }, // green: hunts blue,  flees red
    { hunt, flee, self }, // blue:  hunts red,   flees green
};

// RGB per species
static constexpr float speciesColor[numSpecies][3] = {
    { 1.0f,  0.2f, 0.2f }, // red
    { 0.2f, 1.0f,  0.2f }, // green
    { 0.2f, 0.2f, 1.0f  }, // blue
};

// store particles as StructOfArrays instead of ArraysOfStructs so that Eigen can vectorize them since they need to be
// contiguous same-type values, each particle has a position/velocity (position += velocity * deltaTime)/species (0, 1, 2...)
struct Particles {
    Eigen::ArrayXf posX;
    Eigen::ArrayXf posY;
    Eigen::ArrayXf velX;
    Eigen::ArrayXf velY;
    Eigen::ArrayXi species;

    explicit Particles(int count)
        : posX(count), posY(count),
          velX(count), velY(count),
          species(count) {}
};

// OpenGL's pipeline is inherently 3D with vec 4 xyzw, in 2D we just ignore z (0) and w (1 = a point, not a direction)
// runs once per corner (4 times total per frame) of the fullscreen quad (2 triangles making the fullscreen rectangle)
// transform the point's 3D position into screen coordinates in NDC unchanged (see drawFullscreenQuad)
static const char* vertexShader = R"GLSL(
#version 330
in vec3 vertexPosition; // XYZ corner of the quad sent from CPU
in vec2 vertexTexCoord; // 2D coordinate (0 to 1) for this corner from CPU
out vec2 fragCoord; // texture coordinate result to the fragment shader
void main() {
    fragCoord   = vertexTexCoord;
    gl_Position = vec4(vertexPosition, 1.0); // tells the GPU where this corner is on screen
}
)GLSL";

// runs once per pixel on screen, each pixel calculates in parallel "I am at screen position X,Y, what color am I ?"
// numParticles and discRadius are constants so injected at runtime (see buildShader) so no constexpr
// we can't use uniform arrays since they live in a tiny on-chip constant register file with hardware limit
// which would heavily reduce the amount of particles, instead we "abuse" textures having "unlimited" space on the VRAM
// by using one (array of texels aka pixels in a texture, rgba so4 floats) for position using rg only
// and another for the rgb colors, both numParticles wide
// interpolation normally blends neighboring texels smoothly (useful for images) but for out particleData it would corrupt
// positions so we disable it RL_TEXTURE_FILTER_NEAREST
static const char* fragShaderTemplate = R"GLSL(
#version 330
in vec2 fragCoord; // from vertexShader
out vec4 outColor; // color for each pixels
#define numParticles %d

// from main
uniform vec2 screenSize;
uniform sampler2D particleData;
uniform float time;

void main() {
    // fragCoord is 0 to 1 across the screen, multiply by screenSize to get actual pixel coordinates
    // flip Y because OpenGL UV origin is bottom-left but our screen coordinates are top-left
    vec2 pixelPos = vec2(fragCoord.x, 1.0 - fragCoord.y) * screenSize;

    // start with the background dark blue color, every particle's light adds on top
    vec3 light = vec3(0.0, 0.0, 0.01);

    for (int i = 0; i < numParticles; i++) {
        // extract this particle's position from row 0 of the data texture
        vec2 particlePos = texelFetch(particleData, ivec2(i, 0), 0).rg;

        // extract this particle's color from row 1 of the data texture
        vec3 particleColor = texelFetch(particleData, ivec2(i, 1), 0).rgb;

        // how far is this pixel from the particle center
        float dist = length(pixelPos - particlePos);

        // just a cool effect to make particles feel a bit alive
        // time * x is pulse speed: sin completes one cycle over 2π, so x / (2π) = y Hz then convert y to seconds
        // float(i) * 0.381966 is phase offset: https://en.wikipedia.org/wiki/Golden_angle in normalized degrees,
        // picture advancing over a circle but never ending up on the same spot you started at because it's mathematically the
        // "worst approximable" rational (nature's choice !) so consecutive particles have maximally spread phases
        // abs(sin()) folds the sine to always 0->1 positive, scale to x-1.0 with x + (1-x) * abs(sin())
        float pulse = 0.25 + 0.75 * abs(sin(time * 3.0 + float(i) * 0.381966));

        // hard neon circle edge, anti-aliased (smoothed) over 2px with smoothstep
        // smoothstep(a, b, x) = 0 when x<a, 1 when x>b, smooth S-curve between so (1 - smoothstep) = 1 inside the disc, 0 outside
        // 5.0 is intentionally above 1.0 (HDR) so tone-mapping keeps the center near bright-white compared to edge
        float disc = 5.0 * (1.0 - smoothstep(%.1f - 1.0, %.1f + 1.0, dist));

        // glow is measured from the disc surface outward, not from the center
        float surfaceDist = max(dist - %.1f, 0.0);

        // gaussian glow: A·e^(-x²/σ²) bright at the surface, fades exponentially outward
        // A: peak glow intensity multiplier, for us pulse
        // x: distance from disc edge (outward), starts at surface=0
        // σ (sigma): the bigger it gets the bigger the halo
        float sigma = 8.0;
        float glow = pulse * exp(-(surfaceDist * surfaceDist) / (sigma * sigma));

        // additive accumulation/colors mixing, dense clusters become HDR bright, tune intensity with multipliers
        light += particleColor * (disc + glow * 0.1);
    }

    // tone-mapping: squashes HDR values (1+) into display range (0 to 1)
    // 1 - exp(-x) approaches 1 as x grows so bright clusters saturate to white
    light = 1.0 - exp(-light * 1.0);
    // gamma correction: 2.2 comes from old CRT monitors where doubling the input voltage gave roughly 4x the brightness (a power law with exponent ~2.2)
    // so to make it linear (^1 instead of ^2.2) we cancel it out with value^(1/2.2)
    // if not the monitor would apply gamma curve on top of already-linear values aka double-darkening
    light = pow(light, vec3(1.0 / 2.2));
    // basically convert "physics brightness" to "display brightness"

    // alpha is always 1, transluscency comes from our total control of brightnesses over background
    outColor = vec4(light, 1.0);
}
)GLSL";