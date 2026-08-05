//
// gyro_space.cpp - orientation-space conversion (Steam-Input-style).
//
// Conventions:
//   - q is the body -> world orientation (world +Y = up).
//   - controller axes: +X = right, +Y = forward, +Z = up.
//   - At the neutral flat grip the controller +Y (forward) points along
//     world -Z, so rotating "right" (positive yaw) maps to +world X.
//   - WORLD_SPACE output is grip-independent: it uses the world yaw axis for
//     horizontal and the forward-vector sweep for vertical.
//
#include "gyro_space.h"

#include <cmath>

namespace {
constexpr float kAxisUp[3]    = {0.0f, 0.0f, 1.0f};
constexpr float kAxisRight[3] = {1.0f, 0.0f, 0.0f};
constexpr float kAxisFwd[3]   = {0.0f, 1.0f, 0.0f};
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
                       GyroOutput *out) {
    out->x = 0.0f;
    out->y = 0.0f;

    // World-frame angular velocity (body -> world).
    float omega[3];
    quat_rotate(q, gyro, omega);

    // World-frame direction vectors of the controller.
    float fwd[3];
    quat_rotate(q, kAxisFwd, fwd);

    switch (s->mode) {
    case GYRO_YAW:
        // Yaw only: world rotation about the vertical axis. Roll and pitch
        // do not contribute, so tilting the controller sideways does nothing.
        out->x = -omega[1];
        break;

    case GYRO_ROLL:
        // Roll only: rotation about the controller's forward axis.
        out->x = -(omega[0] * fwd[0] + omega[1] * fwd[1] + omega[2] * fwd[2]);
        break;

    case GYRO_YAW_ROLL:
        out->x = -omega[1];
        out->y = -(omega[0] * fwd[0] + omega[1] * fwd[1] + omega[2] * fwd[2]);
        break;

    case GYRO_LOCAL_SPACE:
        // Raw sensor-frame rates: yaw (about controller up) -> X, pitch
        // (about controller right) -> Y. The output follows the controller's
        // own coordinates wherever it is pointing.
        out->x = -gyro[2];
        out->y =  gyro[0];
        break;

    case GYRO_PLAYER_SPACE: {
        // Axes locked to the grip captured when the gyro activated. Uses
        // world-rate projections onto the captured frame.
        if (!s->q_ref_valid) { out->x = -omega[1]; break; }
        float up0[3], right0[3];
        quat_rotate(s->q_ref, kAxisUp, up0);
        quat_rotate(s->q_ref, kAxisRight, right0);
        out->x = -(omega[0] * up0[0] + omega[1] * up0[1] + omega[2] * up0[2]);
        out->y =  (omega[0] * right0[0] + omega[1] * right0[1] + omega[2] * right0[2]);
        break;
    }

    case GYRO_WORLD_SPACE: {
        // Grip-independent. Horizontal = world yaw. Vertical = rotation in the
        // world-vertical plane that sweeps the controller's forward vector up
        // and down. Works however the controller is held (flat, vertical,
        // tilted 45 deg, upside down).
        out->x = -omega[1];
        // Pitch axis in the horizontal world plane, perpendicular to forward:
        //   P = normalize(cross(fwd, worldUp)) = normalize(-fwd.z, 0, fwd.x)
        const float ax = -fwd[2];
        const float az =  fwd[0];
        const float al = std::sqrt(ax * ax + az * az);
        if (al > 0.05f) {
            out->y = (omega[0] * ax + omega[2] * az) / al;
        } else {
            // Controller pointing straight up/down: the sweep axis is degenerate,
            // fall back to the controller's local pitch.
            out->y = omega[0];
        }
        break;
    }

    case GYRO_LASER_POINTER: {
        // Perspective projection of the controller's forward vector onto a
        // virtual screen plane (VR-controller-pointer style). The pointer moves
        // on the screen; its velocity becomes the aim rate.
        const float depth = -fwd[2]; // forward along world -Z
        if (depth > 0.1f) {
            const float sx = fwd[0] / depth;
            const float sy = fwd[1] / depth;
            if (!s->lp_valid) {
                s->lp_sx = sx;
                s->lp_sy = sy;
                s->lp_valid = true;
            }
            constexpr float kPointerScale = 100.0f; // screen units -> aim rate
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
