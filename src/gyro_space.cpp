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
        // Player Space: axes locked to the grip captured at activation.
        // q_rel = conj(q_ref) * q maps current body → reference body.
        // Applying quat_rotate(q_rel, gyro_body) expresses body-frame gyro
        // in the reference body frame, then we extract yaw (Z) and pitch (X)
        // from that reference-frame gyro.  Axes stay aligned with the initial
        // grip regardless of how far the controller has rotated since capture.
        if (s->q_ref_valid) {
            const Quat q_rel = quat_mult(quat_conjugate(s->q_ref), q);
            float gyro_ref[3];
            quat_rotate(q_rel, gyro, gyro_ref);
            out->x = -gyro_ref[2];  // yaw in reference frame
            out->y = -gyro_ref[0];  // pitch in reference frame
        } else {
            // No reference yet — fall back to raw body-frame yaw/pitch.
            out->x = -gyro[2];
            out->y = -gyro[0];
        }
        break;
    }

    case GYRO_WORLD_SPACE: {
        // Grip-independent World Space: horizontal = world yaw (about gravity);
        // vertical = rotation about the body-right axis projected to the world
        // horizontal plane.  Unlike cross(fwd, worldUp) which flips sign when
        // the forward vector crosses the world-up direction (gimbal-like
        // singularity), the body-right axis never flips — the projected axis
        // stays continuous through all orientations.
        out->x = -omega[1];                // world yaw

        float right_w[3];
        quat_rotate(q, kAxisRight, right_w);  // controller X axis in world

        // Pitch axis = body right projected to the world horizontal (XZ) plane.
        // This axis is the direction in the world that "pitch" sweeps the
        // forward vector up/down, and it never reverses direction across any
        // orientation (unlike a forward-vector cross-product).
        const float ax = right_w[0];
        const float az = right_w[2];
        const float al = std::sqrt(ax * ax + az * az);
        if (al > 0.05f) {
            out->y = -(omega[0] * ax + omega[2] * az) / al;
        } else {
            // Body right is nearly vertical (extreme roll): pitch about body X
            // is effectively about world Y, which is already captured as yaw.
            out->y = -omega[0];
        }

        // Body-magnitude normalisation: in non-flat grips the world yaw axis
        // and pitch axis are not orthonormal in body-frame, which makes a
        // controller-space circle map to an output ellipse.  Scale output by
        // |body_yaw+pitch| / |output| so hand-speed magnitude is preserved.
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
