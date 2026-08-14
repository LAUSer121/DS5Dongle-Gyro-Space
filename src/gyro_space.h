//
// gyro_space.h - orientation-space conversion for gyro aiming.
//
// Takes the fused controller orientation quaternion (body -> world) and turns
// it into a unified 2D output (gyro_x, gyro_y) in one of 7 spaces, mirroring
// Steam Input Gyro Space. Output axes are aim-space: +x = aim right,
// +y = aim up. The caller integrates gyro_x/gyro_y over time onto the stick.
//
// Coordinate convention (body frame): +X = right, +Y = forward, +Z = up
// (right-handed). main.cpp maps the raw DS5 sensor axes (sensor +X = left,
// +Y = up, +Z = back; byte15 = pitch, byte17 = yaw, byte19 = roll) into this
// frame as body = (-sensorX, -sensorZ, +sensorY) before calling in, so both
// the fusion and every mode below operate on one consistent right-handed body
// frame in any grip.
//
#ifndef DS5_GYRO_SPACE_H
#define DS5_GYRO_SPACE_H

#include <cstdint>
#include "quaternion.h"

enum GyroMode : uint8_t {
    GYRO_YAW = 0,           // yaw only -> horizontal
    GYRO_ROLL = 1,          // roll only -> horizontal
    GYRO_YAW_ROLL = 2,      // yaw -> horizontal, roll -> vertical
    GYRO_LOCAL_SPACE = 3,   // axes follow the controller's own coordinates
    GYRO_PLAYER_SPACE = 4,  // world yaw + local pitch (Steam-style player space)
    GYRO_WORLD_SPACE = 5,   // world yaw + world pitch (gravity-aligned, grip-independent)
    GYRO_LASER_POINTER = 6, // screen projection of the forward vector (relative to activation)
};

struct GyroOutput {
    float x; // deg/s or pointer-rate, + = aim right
    float y; // deg/s or pointer-rate, + = aim up
};

struct GyroSpace {
    GyroMode mode;
    Quat     q_ref;        // laser-pointer reference orientation (captured at activation)
    bool     q_ref_valid;
    bool     was_active;   // activation edge detection
    // laser pointer projection state
    float    lp_sx, lp_sy;
    bool     lp_valid;
};

void gyro_space_init(GyroSpace *s, GyroMode mode);

// Capture the current orientation as the laser-pointer reference.
void gyro_space_capture_reference(GyroSpace *s, const Quat &q);

// Call every sample with `active` = whether the gyro is currently gated on.
// Captures the laser-pointer reference on the activation rising edge.
void gyro_space_tick(GyroSpace *s, bool active, const Quat &q);

// Convert the fused orientation + body-frame angular velocity into aim-space
// output for the configured mode. `gyro_degps` must be in the body frame
// (X=right, Y=forward, Z=up); the quaternion is the authoritative state.
// `dt` is the report period in seconds and is used by LASER_POINTER to scale
// the per-frame pointer delta into a deg/s-equivalent rate.
// `gyro_axis` controls which body axis drives horizontal in GYRO_YAW mode
// (0=yaw=body Z, 1=roll=body Y), matching the original artzox behaviour.
void gyro_space_output(GyroSpace *s, const Quat &q, const float gyro_degps[3],
                       uint8_t gyro_axis, float dt, GyroOutput *out);

#endif // DS5_GYRO_SPACE_H
