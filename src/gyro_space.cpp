//
// gyro_space.cpp - orientation-space conversion (Steam-Input-style).
//
// Conventions:
//   - q is the body -> world orientation (world +Y = up).
//   - controller axes: +X = right, +Y = forward, +Z = up.
//   - At the neutral flat grip the controller +Y (forward) points along
//     world -Z, so rotating "right" (positive yaw) maps to +world X.
//   - WORLD_SPACE output is grip-independent: it uses the world yaw axis for
//     horizontal and the fixed world right axis for vertical.
//
#include "gyro_space.h"

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
        // Axes locked to the grip captured at activation. Projects body-frame
        // gyro onto the captured controller axes expressed in body space.
        // This avoids using the potentially drifted current quaternion for the
        // omega (world-frame angular velocity) conversion — the raw gyro is
        // always in body frame and the captured axes (inverse-rotated from
        // world to body via q_ref_inv) are computed once at capture time.
        float up_body[3];   // captured world-up axis, expressed in body frame
        float rt_body[3];   // captured right axis, expressed in body frame
        if (s->q_ref_valid) {
            const Quat qref_inv = quat_conjugate(s->q_ref);
            quat_rotate(qref_inv, kAxisUp, up_body);
            quat_rotate(qref_inv, kAxisRight, rt_body);
        } else {
            // No reference yet — fall back to body Z/Y (flat-grip yaw/roll).
            up_body[0] = 0.0f; up_body[1] = 0.0f; up_body[2] = 1.0f;   // body Z
            rt_body[0] = 1.0f; rt_body[1] = 0.0f; rt_body[2] = 0.0f;   // body X
        }
        // X = yaw (about captured up)  → dot(gyro, up_body)
        // Y = pitch (about captured right) → dot(gyro, rt_body)
        out->x = -(gyro[0]*up_body[0]  + gyro[1]*up_body[1]  + gyro[2]*up_body[2]);
        out->y = -(gyro[0]*rt_body[0] + gyro[1]*rt_body[1] + gyro[2]*rt_body[2]);
        break;
    }

    case GYRO_WORLD_SPACE:
        // Grip-independent aiming (Steam Input World Space / GamepadMotionHelpers
        // TransformToWorldSpace). Horizontal = world yaw (rotation about gravity,
        // world +Y). Vertical = world pitch (rotation about the fixed world right
        // axis, world +X). Both axes are world-frame angular velocities of the
        // controller.
        //
        // The fixed world right axis is used instead of the dynamic axis
        // (cross(fwd, worldUp) = (-fwd.z, 0, fwd.x)) because the dynamic axis
        // flips sign when the forward vector sweeps through world-up: a
        // continuous upward sweep would suddenly report downward motion (the
        // "boundary" flip). The fixed axis is also linear — its magnitude does
        // not shrink as the controller tilts away from horizontal, so circles
        // stay round at any grip orientation.
        out->x = -omega[1];
        out->y = -omega[0];
        break;

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
