//
// gyro_fusion.h - Mahony-style complementary AHRS for the DualSense IMU.
//
// Fuses the gyroscope (fast, drifts) with the accelerometer (slow, absolute)
// into a single controller orientation quaternion. The accelerometer's measured
// gravity vector continuously corrects gyro drift and the initial posture, so
// the controller works in ANY grip - there is no fixed horizon / horizontal
// reference anywhere in this pipeline.
//
// All math is single-precision (RP2350 FPU), no allocation.
//
#ifndef DS5_GYRO_FUSION_H
#define DS5_GYRO_FUSION_H

#include <cstdint>
#include "quaternion.h"

// DualSense gyro scale: int16 spans +/-2000 deg/s (verified on hardware).
constexpr float GYRO_DEG_PER_LSB = 2000.0f / 32768.0f;

struct GyroFusion {
    Quat  q;              // body -> world orientation (world +Y = gravity up)
    float bias[3];        // gyro bias estimate (deg/s), slow-drift compensation
    float ierr[3];        // Mahony integral error (deg/s)
    float rest_len;       // learned |accel| at rest (raw units)
    bool  has_rest;       // rest_len has been learned
};

// Initialize fusion. `accel_raw` is the first accelerometer sample (raw int16
// units); its direction seeds the orientation so any starting grip is correct.
void gyro_fusion_init(GyroFusion *f, const float accel_raw[3]);

// Run one fusion step.
//   gyro_degps : angular velocity in deg/s, sensor frame (X=pitch, Y=roll, Z=yaw)
//   accel_raw  : accelerometer in raw int16 units, sensor frame
//   dt         : elapsed time in seconds since the previous update
//   gain       : [0-100] gravity-correction strength (50 = default)
void gyro_fusion_update(GyroFusion *f, const float gyro_degps[3],
                        const float accel_raw[3], float dt, float gain);

#endif // DS5_GYRO_FUSION_H
