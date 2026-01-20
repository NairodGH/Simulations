#include "kinematic.hpp"

struct State {
    Robot robot;
    std::array<float, 6> joint_values;

    Target target;

    Camera3D camera;

    float link_height = 1.5f;
    float link_width = 0.7f;
};

State init_state() {
    State s;
    s.robot.transform = Eigen::Matrix4d::Identity();
    
    // replicate some kind of practical arm with a moving base, then rotating base, then alternate left-right and forward-backward
    for (int i = 0; i < 6; i++) {
        s.robot.axes[i].kind = (i == 0) ? Axis::Kind::Linear : Axis::Kind::Rotary;
        s.robot.axes[i].pivot = (i == 0) ? Eigen::Vector3d(0, 0, 0) : Eigen::Vector3d(0, s.link_height, 0);
        
        if (i == 1) {
            s.robot.axes[i].pivotNormal = Eigen::Vector3d(0, 1, 0); 
        } else {
            s.robot.axes[i].pivotNormal = (i % 2 == 0) ? Eigen::Vector3d(1, 0, 0) : Eigen::Vector3d(0, 0, 1);
        }
        s.joint_values[i] = 0.f;
    }

    s.target.position = Eigen::Vector3d(0, s.link_height, 0);
    s.target.align = Eigen::Vector3d(0, 0, 1);
    s.target.roll = 0.0;

    s.camera = {0};
    s.camera.position = {-30.f, 20.f, 0};
    s.camera.up = {0.f, 1.f, 0.f};
    s.camera.fovy = 30.f;
    s.camera.projection = CAMERA_PERSPECTIVE;

    return s;
}

void handle_mouse(State& s) {
    // manually handle panning, this time in all directions + mousewheel zoom
    if (GetMousePosition().x > 240) {
        Vector3 pos = s.camera.position;
        Vector3 target = s.camera.target;
        Vector3 up  = s.camera.up;
        if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
            Vector2 delta = GetMouseDelta();
            Vector3 to_camera  = Vector3Subtract(pos, target);
            Vector3 right = Vector3Normalize(Vector3CrossProduct(up, to_camera));
            pos = Vector3RotateByAxisAngle(pos, up, -delta.x * 0.01f);
            pos = Vector3RotateByAxisAngle(pos, right, -delta.y * 0.01f);
        }
        Vector3 to_target = Vector3Normalize(Vector3Subtract(target, pos));
        s.camera.position = Vector3Add(pos, Vector3Scale(to_target, GetMouseWheelMove() * 1.5f));
    }
}

void draw_3d(State& s) {
    // not used since we can't pass the joint values
    const Target target = forward(s.robot, s.target);

    ClearBackground(DARKGRAY);
    BeginMode3D(s.camera);
        DrawGrid(20, 1.f);
        rlPushMatrix();
            float robot_space[16];
            for(int i = 0; i < 16; i++)
                robot_space[i] = (float)s.robot.transform.data()[i];
            rlMultMatrixf(robot_space);
            // in robot space, create the axes with each being in the previous' space for the kinematic chain
            for (int i = 0; i < 6; i++) {
                rlPushMatrix();
                rlTranslatef(s.robot.axes[i].pivot.x(), s.robot.axes[i].pivot.y(), s.robot.axes[i].pivot.z());
                // translate if linear, rotate along the pivotNormal if rotary, both using the joint_value, then draw
                if (s.robot.axes[i].kind == Axis::Kind::Linear) {
                    rlTranslatef(s.joint_values[i], 0, 0);
                    DrawCube({0, s.link_height / 2.f, 0}, s.link_width, s.link_height, s.link_width, ColorAlpha(BLUE, 0.4f));
                    DrawCubeWires({0, s.link_height / 2.f, 0}, s.link_width, s.link_height, s.link_width, BLUE);
                } else {
                    Vector3 axis_normal = {(float)s.robot.axes[i].pivotNormal.x(), (float)s.robot.axes[i].pivotNormal.y(), (float)s.robot.axes[i].pivotNormal.z()};
                    rlRotatef(s.joint_values[i] * RAD2DEG, axis_normal.x, axis_normal.y, axis_normal.z);
                    DrawCapsule({0, 0, 0}, {0, s.link_height, 0}, s.link_width / 2.f, 8, 8, ColorAlpha(PURPLE, 0.4f));
                    DrawCapsuleWires({0, 0, 0}, {0, s.link_height, 0}, s.link_width / 2.f, 8, 8, PURPLE);
                }
            }

            rlTranslatef(s.target.position.x(), s.target.position.y(), s.target.position.z());
            // rotate to an angle axis from align and roll then draw
            Eigen::AngleAxisd angle_axis(fromAlignRoll(s.target.align, s.target.roll));
            rlRotatef(angle_axis.angle() * RAD2DEG, (float)angle_axis.axis().x(), (float)angle_axis.axis().y(), (float)angle_axis.axis().z());
            DrawCylinderWiresEx({0, 0, 0}, {0, 0, 1.f}, 0.3f, 0.f, 8, ORANGE);

            for (int i = 0; i < 6; i++)
                rlPopMatrix();
        rlPopMatrix();
    EndMode3D();
}

void draw_ui(State& s) {
    DrawRectangle(0, 0, 240, 720, ColorAlpha(BLACK, 0.5f));

    Rectangle toggle_rect = {20, 20, 200, 30};
    bool linear = (s.robot.axes[0].kind == Axis::Kind::Linear);
    DrawRectangleRec(toggle_rect, linear ? BLUE : PURPLE);
    DrawText(linear ? "Axis 0 linear" : "Axis 0 rotary", toggle_rect.x + 15, toggle_rect.y + 6, 20, WHITE);
    if (CheckCollisionPointRec(GetMousePosition(), toggle_rect) && IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
        s.robot.axes[0].kind = linear ? Axis::Kind::Rotary : Axis::Kind::Linear;
        s.joint_values[0] = 0.f; 
    }

    for (int i = 0; i < 6; i++) {
        float y = 80.f + i * 60.f;
        DrawText(TextFormat("Axis %d: %.2f", i, s.joint_values[i]), 20, y, 16, WHITE);
        Rectangle slider_rect = {20, y + 20, 200, 20};
        DrawRectangleLinesEx(slider_rect, 2, WHITE);
        DrawRectangle(
            slider_rect.x,
            slider_rect.y,
            slider_rect.width * (s.joint_values[i] + PI) / (2.f * PI),
            slider_rect.height,
            (i == 0 && linear) ? BLUE : PURPLE
        );
        if (CheckCollisionPointRec(GetMousePosition(), slider_rect) && IsMouseButtonDown(MOUSE_LEFT_BUTTON))
            s.joint_values[i] = -PI + (GetMousePosition().x - slider_rect.x) / slider_rect.width * (2.f * PI);
    }

    DrawText(TextFormat("Target roll: %.2f rad", s.target.roll), 20, 430, 16, WHITE);
    Rectangle roll_rect = {20, 450, 200, 15};
    DrawRectangleLinesEx(roll_rect, 2, WHITE);
    DrawRectangle(
        roll_rect.x,
        roll_rect.y,
        roll_rect.width * ((s.target.roll + PI) / (2.f * PI)),
        roll_rect.height,
        ORANGE
    );
    if (CheckCollisionPointRec(GetMousePosition(), roll_rect) && IsMouseButtonDown(MOUSE_LEFT_BUTTON))
        s.target.roll = -PI + (GetMousePosition().x - roll_rect.x) / roll_rect.width * (2.f * PI);
}

int main() {
    SetTraceLogLevel(LOG_NONE);
    InitWindow(1280, 720, "Kinematics");
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