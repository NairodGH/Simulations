#include "toolpath.hpp"

struct State {
    Input input;

    float cube_size;

    Camera3D camera;

    bool linear;
    Rectangle toggle;
    
    struct Slider {
        Rectangle rect;
        double min;
        double max;
        const char* label;
        bool sliding;
    };
    Slider width;
    Slider height;
    Slider spacing;
    Slider factor;
    Slider length;
    Slider step;
};

State init_state() {
    State s = {};

    s.cube_size = 6.f;
    
    s.camera = {0};
    s.camera.position = {1.5f, 4.f, 36.f};
    s.camera.up = {0.f, 1.f, 0.f};

    float angle = 45.0f * DEG2RAD;
    float cos_a = cosf(angle);
    float sin_a = sinf(angle);
    // rotate position
    float x = s.camera.position.x;
    float y = s.camera.position.y;
    s.camera.position.x = x * cos_a - y * sin_a;
    s.camera.position.y = x * sin_a + y * cos_a;
    // rotate up vector
    x = s.camera.up.x;
    y = s.camera.up.y;
    s.camera.up.x = x * cos_a - y * sin_a;
    s.camera.up.y = x * sin_a + y * cos_a;
    
    s.camera.fovy = s.cube_size * 10.f;
    s.camera.projection = CAMERA_PERSPECTIVE;
    
    s.linear = true;
    s.toggle = {20, 20, 120, 30};
    
    s.width = {{20, 80, 200, 20}, 1.0, 20.0, "Width"};
    s.height = {{20, 140, 200, 20}, 1.0, 10.0, "Height"};
    s.spacing = {{20, 200, 200, 20}, 0.2, 2.0, "Spacing"};
    s.factor = {{20, 80, 200, 20}, 0.1, 1.0, "Factor"};
    s.length = {{20, 140, 200, 20}, 5.0, 15.0, "Length"};
    s.step = {{20, 200, 200, 20}, 0.1, 1.0, "Step"};

    const float hSize = s.cube_size / 2.f;

    // define the cube vertices around its origin by simply going through every x y z halfSize configurations
    // order doesn't really matter as long as their respective faces are referenced correctly
    s.input.V.resize(8, 3);
    s.input.V <<
        hSize, hSize, hSize,
        -hSize, -hSize, -hSize,
        -hSize, -hSize, hSize,
        -hSize, hSize, -hSize,
        hSize, -hSize, -hSize,
        -hSize, hSize, hSize,
        hSize, -hSize, hSize,
        hSize, hSize, -hSize;

    // define the cube's 6 faces with 2 triangles each, referencing their vertices accordingly
    s.input.F.resize(12, 3);
    s.input.F <<
        0, 6, 2, 0, 2, 5,
        7, 4, 1, 7, 1, 3,
        0, 7, 6, 0, 6, 4,
        5, 2, 1, 5, 1, 3,
        0, 5, 3, 0, 3, 7,
        6, 4, 1, 6, 1, 2;

    // create the local to world transform matrix that applies the same Y 225° rotation and corner translation as hardcoded in draw_3d
    Eigen::Matrix3d rotation = Eigen::AngleAxisd(270.0 * EIGEN_PI / 180.0, Eigen::Vector3d(1, 1, 0).normalized()).toRotationMatrix();
    s.input.mesh_to_world = (rotation * Eigen::Translation3d(hSize, hSize, hSize)).matrix();
    s.input.slicing_plane_normal = Eigen::Vector3d(0, 1, 0);
    s.input.kind = Input::Kind::Linear;
    
    return s;
}

void handle_mouse(State& s) {
    Vector2 mouse_pos = GetMousePosition();
    
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        // s.linear is just syntactic sugar for s.input.kind == Input::Kind::Linear
        if (CheckCollisionPointRec(mouse_pos, s.toggle)) {
            s.linear = !s.linear;
            s.input.kind = s.linear ? Input::Kind::Linear : Input::Kind::Spiral;
        }

        for (auto* slider : {&s.width, &s.height, &s.spacing, &s.factor, &s.length, &s.step}) {
            if (CheckCollisionPointRec(mouse_pos, slider->rect)) {
                slider->sliding = true;
            }
        }
    }
    
    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
        for (auto* slider : {&s.width, &s.height, &s.spacing, &s.factor, &s.length, &s.step}) {
            slider->sliding = false;
        }
    }
    
    auto updateSlider = [&](auto& slider, double& value, bool condition) {
        if (slider.sliding && condition) {
            value = slider.min + Clamp((mouse_pos.x - slider.rect.x) / slider.rect.width, 0.f, 1.f) * (slider.max - slider.min);
        }
    };
    updateSlider(s.width, s.input.width, s.linear);
    updateSlider(s.height, s.input.height, s.linear);
    updateSlider(s.spacing, s.input.height_spacing, s.linear);
    updateSlider(s.factor, s.input.spiralizing_out_factor, !s.linear);
    updateSlider(s.length, s.input.spiral_length, !s.linear);
    updateSlider(s.step, s.input.spiral_step, !s.linear);

    // manually handle X panning
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && GetMousePosition().x > 240) {
        s.camera.position = Vector3RotateByAxisAngle(s.camera.position, s.camera.up, -GetMouseDelta().x * 0.01f);
    }
}

// create the same scene (objects, rotations, translations, colors) as in the provided schemas
void draw_3d(const State& s) {
    const Output ramp = ramping(s.input);

    ClearBackground(DARKGRAY);
    BeginMode3D(s.camera);
        DrawCubeWires({0, 0, 0}, 30.f, 0.01f, 30.f, ORANGE);
        DrawLine3D({0, 0, 0}, {0, s.cube_size, 0}, s.linear ? BLUE : PURPLE);

        for (size_t i = 0; i < ramp.points.size() - 1; i++) {
            Vector3 p1 = {(float)ramp.points[i].x(),
                        (float)ramp.points[i].y(),
                        (float)ramp.points[i].z()};
            Vector3 p2 = {(float)ramp.points[i + 1].x(),
                        (float)ramp.points[i + 1].y(),
                        (float)ramp.points[i + 1].z()};
            DrawLine3D(p1, p2, s.linear ? BLUE : PURPLE);
        }

        rlBegin(RL_LINES);
            for (int i = 0; i < s.input.F.rows(); i++) {
                for (int j = 0; j < 3; j++) {
                    int v1 = s.input.F(i, j);
                    int v2 = s.input.F(i, (j + 1) % 3);
                    
                    Eigen::Vector4d p1_h(s.input.V(v1, 0), s.input.V(v1, 1), s.input.V(v1, 2), 1.0);
                    Eigen::Vector4d p2_h(s.input.V(v2, 0), s.input.V(v2, 1), s.input.V(v2, 2), 1.0);
                    Eigen::Vector4d p1_world = s.input.mesh_to_world * p1_h;
                    Eigen::Vector4d p2_world = s.input.mesh_to_world * p2_h;
                    
                    rlColor4ub(255, 165, 0, 255);
                    rlVertex3f(p1_world.x(), p1_world.y(), p1_world.z());
                    rlVertex3f(p2_world.x(), p2_world.y(), p2_world.z());
                }
            }
        rlEnd();
    EndMode3D();
}

void draw_ui(const State& s) {
    DrawRectangle(0, 0, 240, 720, ColorAlpha(BLACK, 0.5f));

    DrawRectangleRec(s.toggle, s.linear ? BLUE : PURPLE);
    DrawText(s.linear ? "Linear" : "Spiral",
             s.toggle.x + 15, s.toggle.y + 6, 20, WHITE);

    auto draw_slider = [&](const State::Slider& config, double value) {
        DrawText(TextFormat("%s: %.2f", config.label, value), config.rect.x, config.rect.y - 20, 16, WHITE);
        Rectangle fill = config.rect;
        fill.width *= (value - config.min) / (config.max - config.min);
        DrawRectangleRec(fill, s.linear ? BLUE : PURPLE);
        DrawRectangleLinesEx(config.rect, 2, WHITE);
    };
    
    if (s.linear) {
        draw_slider(s.width, s.input.width);
        draw_slider(s.height, s.input.height);
        draw_slider(s.spacing, s.input.height_spacing);
    } else {
        draw_slider(s.factor, s.input.spiralizing_out_factor);
        draw_slider(s.length, s.input.spiral_length);
        draw_slider(s.step, s.input.spiral_step);
    }
}

int main() {
    SetTraceLogLevel(LOG_NONE);
    InitWindow(1280, 720, "Toolpath");
    State state = init_state();
    SetTargetFPS(60);
    while (!WindowShouldClose()) {
        handle_mouse(state);
        BeginDrawing();
            draw_3d(state);
            draw_ui(state);
        EndDrawing();
    }
    CloseWindow();
    return 0;
}