//
// gyro_space.cpp - orientation-space conversion (Steam-Input-style).
//
// Conventions:
//   - q is the body -> world orientation (world +Y = up).
//   - controller axes: +X = right, +Y = forward, +Z = up (right-handed).
//   - gyro[] arrives in this same body frame: main.cpp maps the raw DS5
//     sensor axes (sensor +X = left, +Y = up, +Z = back; byte15 = pitch,
//     byte17 = yaw, byte19 = roll) as body = (-sensorX, -sensorZ, +sensorY),
//     so pitch-up / roll-right / yaw-left are positive and gyro and accel
//     share one consistent right-handed frame in any grip.
//   - At the neutral flat grip the controller +Y (forward) points along
//     world -Z, so rotating "right" (positive yaw) maps to +world X.
//   - WORLD_SPACE output is grip-independent: it uses the world yaw axis for
//     horizontal and the controller-right axis projected onto the world
//     horizontal plane for vertical (GamepadMotionHelpers convention).
//
#include "gyro_space.h"

#include <cmath>

namespace {
constexpr float kAxisRight[3] = {1.0f, 0.0f, 0.0f};
constexpr float kAxisFwd[3]   = {0.0f, 1.0f, 0.0f};
constexpr float kWorldUp[3]   = {0.0f, 1.0f, 0.0f};
}

void gyro_space_init(GyroSpace *s, GyroMode mode) {
    s->mode = mode;
    s->q_ref = quat_identity();
    s->q_ref_valid = false;
    s->was_active = false;
    s->lp_sx = s->lp_sy = 0.0f;
    s->lp_valid = false;
}

void gyro_space_capture_reference(GyroSpace *s, const Quat &q) {
    s->q_ref = quat_normalize(q);
    s->q_ref_valid = true;
    s->lp_sx = s->lp_sy = 0.0f;
    s->lp_valid = false;
}

void gyro_space_tick(GyroSpace *s, bool active, const Quat &q) {
    if (active && !s->was_active) {
        gyro_space_capture_reference(s, q);
    }
    s->was_active = active;
}

void gyro_space_output(GyroSpace *s, const Quat &q, const float gyro[3],
                       uint8_t gyro_axis, float dt, GyroOutput *out) {
    out->x = 0.0f;
    out->y = 0.0f;

    // World-frame angular velocity (body -> world), deg/s. Computed for modes
    // that need quaternion-space projections; skipped for raw body-frame modes.
    float omega[3];
    quat_rotate(q, gyro, omega);

    switch (s->mode) {
    case GYRO_YAW: {
        // Traditional mode (matches artzox original). gyro_axis=0 selects yaw
        // (body Z) for horizontal, gyro_axis=1 selects roll (body Y). Both
        // output pitch (body X) to vertical. Raw body-frame, no quaternion.
        const float horiz = (gyro_axis == 0) ? gyro[2] : gyro[1];
        out->x = -horiz;
        out->y =  gyro[0];
        break;
    }

    case GYRO_ROLL:
        // Traditional mode: roll (body Y) -> horizontal, pitch (body X) -> vertical.
        // Raw body-frame, no quaternion, matches artzox gyro_axis=1 behaviour.
        out->x =  gyro[1];
        out->y =  gyro[0];
        break;

    case GYRO_YAW_ROLL: {
        // Steam "Yaw + Roll" (GamepadMotionHelpers / Steam Deck 2023 rework):
        // vertical = local pitch (as usual), horizontal = rotation about the
        // gravity axis driven by local yaw AND roll. Grip-independent: flat
        // grip reduces to raw yaw; tilted, roll contributes so horizontal
        // aiming still tracks the world horizon instead of disappearing.
        float up_body[3]; // world up (gravity) axis, expressed in body frame
        quat_rotate(quat_conjugate(q), kWorldUp, up_body);
        const float world_yaw = up_body[2] * gyro[2] + up_body[1] * gyro[1];
        out->x = -world_yaw;
        out->y =  gyro[0];
        break;
    }

    case GYRO_LOCAL_SPACE:
        // Raw body-frame: yaw -> X, pitch -> Y. Signs match artzox exactly
        // (dx = -horiz, dy = -pitch). Axes follow the controller wherever it
        // points, no quaternion, no gravity correction.
        out->x = -gyro[2];
        out->y =  gyro[0];
        break;

    case GYRO_PLAYER_SPACE: {
        // Steam-style Player Space: world yaw + local pitch (GamepadMotionHelpers
        // CalculatePlayerSpaceGyro). Horizontal = rotation about the gravity
        // axis, derived from the body yaw/roll rates projected onto the current
        // world-up axis expressed in body frame. The pitch component is
        // deliberately excluded so errors in the estimated gravity direction
        // cannot leak into the yaw output. Vertical = local pitch, which is
        // immune to fusion yaw drift and gives the "world yaw + local pitch"
        // hybrid feel.
        float up_body[3]; // world up (gravity) axis, expressed in body frame
        quat_rotate(quat_conjugate(q), kWorldUp, up_body);
        const float world_yaw = up_body[2] * gyro[2] + up_body[1] * gyro[1];
        out->x = -world_yaw;
        out->y =  gyro[0];
        break;
    }

    case GYRO_WORLD_SPACE: {
        // Grip-independent aiming (GamepadMotionHelpers TransformToWorldSpace).
        // Horizontal = world yaw (rotation about gravity, world +Y).
        // Vertical = rotation about the controller-right axis projected onto
        // the world horizontal plane (normalized). Unlike the fixed world right
        // axis, the projected axis follows the grip, so aiming up/down works
        // at any yaw; unlike cross(fwd, worldUp), it never flips sign when the
        // forward vector sweeps through world-up.
        out->x = -omega[1];
        float right_w[3];
        quat_rotate(q, kAxisRight, right_w);
        const float ax = right_w[0];
        const float az = right_w[2];
        const float al = std::sqrt(ax * ax + az * az);
        if (al > 0.001f) {
            out->y = (omega[0] * ax + omega[2] * az) / al;
        }
        break;
    }

    case GYRO_LASER_POINTER: {
        // Perspective projection of the controller's forward vector onto a
        // virtual screen plane fixed at activation (VR-controller-pointer
        // style). Using the orientation relative to q_ref (captured on the
        // activation edge) makes the pointer independent of the fusion's yaw
        // drift and re-centres it every time the gyro engages (mouse-lift).
        // Frame-to-frame screen deltas are far smaller than deg/s gyro rates,
        // so the scale below converts the per-report delta into a deg/s rate.
        const Quat qrel = s->q_ref_valid ? quat_mult(quat_conjugate(s->q_ref), q) : q;
        float fr[3];
        quat_rotate(qrel, kAxisFwd, fr);
        const float depth = fr[1]; // forward component toward the screen
        if (depth > 0.1f) {
            const float sx = fr[0] / depth;
            const float sy = fr[2] / depth;
            if (!s->lp_valid) {
                s->lp_sx = sx;
                s->lp_sy = sy;
                s->lp_valid = true;
            }
            // d(screen)/dt (rad/s) * 57.2958 = deg/s. Using the actual report
            // period keeps the pointer rate identical at 250 Hz and 500 Hz
            // polling (the old fixed 28 600 constant was correct only at 500 Hz).
            const float kPointerScale = (dt > 1e-5f) ? (57.2958f / dt) : 28600.0f;
            out->x = (sx - s->lp_sx) * kPointerScale;
            out->y = (sy - s->lp_sy) * kPointerScale;
            s->lp_sx = sx;
            s->lp_sy = sy;
        } else {
            s->lp_valid = false; // not pointing at the screen: hold output
        }
        break;
    }

    default:
        out->x = -omega[1];
        break;
    }
}
