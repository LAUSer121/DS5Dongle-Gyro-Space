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
                       uint8_t gyro_axis, GyroOutput *out) {
    out->x = 0.0f;
    out->y = 0.0f;

    // World-frame angular velocity (body -> world) — computed for modes that
    // need quaternion-space projections; skipped for raw body-frame modes.
    float omega[3];
    quat_rotate(q, gyro, omega);

    // World-frame direction vectors of the controller.
    float fwd[3];
    quat_rotate(q, kAxisFwd, fwd);

    switch (s->mode) {
    case GYRO_YAW: {
        // Traditional mode (matches artzox original). gyro_axis=0 selects yaw
        // (body Z) for horizontal, gyro_axis=1 selects roll (body Y). Both
        // output pitch (body X) to vertical. Raw body-frame — no quaternion.
        const float horiz = (gyro_axis == 0) ? gyro[2] : gyro[1];
        out->x = -horiz;
        out->y = -gyro[0];
        break;
    }

    case GYRO_ROLL:
        // Traditional mode: roll (body Y) → horizontal, pitch (body X) → vertical.
        // Raw body-frame — no quaternion, matches artzox gyro_axis=1 behaviour.
        out->x = -gyro[1];
        out->y = -gyro[0];
        break;

    case GYRO_YAW_ROLL:
        // World-frame: yaw (about world up) → X, roll (about controller forward) → Y.
        out->x = -omega[1];
        out->y = -(omega[0] * fwd[0] + omega[1] * fwd[1] + omega[2] * fwd[2]);
        break;

    case GYRO_LOCAL_SPACE:
        // Raw body-frame: yaw → X, pitch → Y. Signs match artzox exactly
        // (dx = -horiz, dy = -pitch). Axes follow the controller wherever it
        // points — no quaternion, no gravity correction.
        out->x = -gyro[2];
        out->y = -gyro[0];
        break;

    case GYRO_PLAYER_SPACE: {
        // Axes locked to the grip captured at activation. Uses world-rate
        // projections onto the captured controller frame so that yaw/pitch
        // follow the initial grip, not the current controller orientation.
        // When q_ref is the flat-grip identity-rotation quaternion, this
        // reduces to: X = -omega[1] (world yaw), Y = -omega[0] (world pitch).
        // Y sign matches artzox convention: nose-up → negative Y (invertable).
        float up0[3], right0[3];
        if (s->q_ref_valid) {
            quat_rotate(s->q_ref, kAxisUp, up0);
            quat_rotate(s->q_ref, kAxisRight, right0);
        } else {
            // No reference yet — fall back to world axes (same as WORLD_SPACE).
            up0[0] = 0.0f; up0[1] = 1.0f; up0[2] = 0.0f;    // world +Y = gravity
            right0[0] = 1.0f; right0[1] = 0.0f; right0[2] = 0.0f; // world +X
        }
        out->x = -(omega[0] * up0[0] + omega[1] * up0[1] + omega[2] * up0[2]);
        out->y = -(omega[0] * right0[0] + omega[1] * right0[1] + omega[2] * right0[2]);
        break;
    }

    case GYRO_WORLD_SPACE: {
        // Grip-independent. Horizontal = world yaw (about gravity). Vertical =
        // rotation that sweeps the controller's forward vector up/down in the
        // world-vertical plane. Works for any grip: flat, vertical, tilted,
        // upside down.
        out->x = -omega[1];
        // Pitch axis: perpendicular to forward in the world horizontal plane.
        // P = normalize(cross(fwd, worldUp)) = normalize(-fwd.z, 0, fwd.x)
        //   (world +Y = up, so cross(fwd, {0,1,0}) → on the XZ plane).
        const float ax = -fwd[2];
        const float az =  fwd[0];
        const float al = std::sqrt(ax * ax + az * az);
        if (al > 0.05f) {
            out->y = -(omega[0] * ax + omega[2] * az) / al;
        } else {
            // Controller pointing straight up/down: the sweep axis is degenerate.
            // Fall back to controller-local pitch, same sign convention.
            out->y = -omega[0];
        }
        // Body-magnitude normalisation: in non-flat grips the world yaw axis
        // and forward-sweep axis are not orthonormal in body-frame, which makes
        // a controller-space circle map to an output ellipse (up to 3x gain
        // variation at 60 deg roll, >5x at vertical).  Scale the output by
        // |body_yaw+pitch| / |output| so the intended hand-speed magnitude is
        // preserved regardless of grip angle — circles stay circles.
        const float bodyMag = std::sqrt(gyro[0] * gyro[0] + gyro[2] * gyro[2]);
        const float outMag  = std::sqrt(out->x * out->x + out->y * out->y);
        if (outMag > 0.001f && bodyMag > 0.001f) {
            const float scale = bodyMag / outMag;
            out->x *= scale;
            out->y *= scale;
        }
        break;
    }

    case GYRO_LASER_POINTER: {
        // Perspective projection of the controller's forward vector onto a
        // virtual screen plane (VR-controller-pointer style). Frame-to-frame
        // screen deltas are ~500× smaller than deg/s gyro rates at 500 Hz
        // (because the delta comes from forward-vector dp not angular rate).
        // kPointerScale compensates for this discretization so the pointer
        // velocity lands in the same stick-scaling range as other modes.
        const float depth = -fwd[2]; // forward component along world -Z (toward screen)
        if (depth > 0.1f) {
            const float sx = fwd[0] / depth;
            const float sy = fwd[1] / depth;
            if (!s->lp_valid) {
                s->lp_sx = sx;
                s->lp_sy = sy;
                s->lp_valid = true;
            }
            // ~28 600 converts a 1-rad/s screen delta at 500 Hz into a ~100 deg/s
            // equivalent (500 Hz * 57.3 deg/rad) so the per-report stick offset
            // matches the magnitude other modes produce from direct gyro rates.
            constexpr float kPointerScale = 28600.0f;
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
