//
// gyro_fusion.cpp - Mahony-style complementary AHRS.
//
// At each step:
//   1. compensate gyro bias (stationary-averaged, drift-compensated),
//   2. build the gyro-integration correction from the accelerometer's measured
//      gravity vector vs the orientation's predicted one (cross product error,
//      PI feedback - the classic Mahony algorithm),
//   3. integrate body-frame angular velocity into the orientation quaternion,
//   4. re-normalize.
//
#include "gyro_fusion.h"

#include <cmath>

namespace {
// World up vector (gravity) in the world frame. World +Y is up.
constexpr float kWorldUp[3] = {0.0f, 1.0f, 0.0f};
// Mahony proportional gain baseline and slope (deg/s per unit error).
constexpr float kKpBase = 8.0f;
constexpr float kKpPerGain = 0.8f;
// Integral gain is a small fraction of the proportional gain.
constexpr float kKiRatio = 0.04f;
// Integral wind-up clamp (deg/s).
constexpr float kIerrMax = 5.0f;
}

void gyro_fusion_init(GyroFusion *f, const float accel_raw[3]) {
    f->q = quat_identity();
    f->bias[0] = f->bias[1] = f->bias[2] = 0.0f;
    f->ierr[0] = f->ierr[1] = f->ierr[2] = 0.0f;
    const float len = std::sqrt(accel_raw[0] * accel_raw[0] +
                                accel_raw[1] * accel_raw[1] +
                                accel_raw[2] * accel_raw[2]);
    f->rest_len = len;
    f->has_rest = len > 1.0f;

    // Seed the orientation from gravity: the first accel sample points UP in the
    // sensor frame. Aligning it with world +Y immediately corrects any grip.
    if (len > 1.0f) {
        const float a[3] = {accel_raw[0] / len, accel_raw[1] / len, accel_raw[2] / len};
        f->q = quat_from_unit_vectors(a, kWorldUp);
    }
}

void gyro_fusion_update(GyroFusion *f, const float gyro[3],
                        const float accel_raw[3], float dt, float gain) {
    const float kp = kKpBase + gain * kKpPerGain;
    const float ki = kp * kKiRatio;

    // --- 1. Gyro bias drift compensation --------------------------------
    // When the sensor is at rest (|accel| ~ learned rest length AND angular
    // rates are tiny), blend the gyro reading into the bias estimate with a
    // ~1.5 s time constant. This tracks slow sensor drift without reacting to
    // actual motion, and is posture-independent (no fixed horizon).
    const float alen = std::sqrt(accel_raw[0] * accel_raw[0] +
                                 accel_raw[1] * accel_raw[1] +
                                 accel_raw[2] * accel_raw[2]);
    if (f->has_rest) {
        const bool at_rest = std::fabs(alen - f->rest_len) < 0.06f * f->rest_len;
        const bool slow = std::fabs(gyro[0]) < 3.0f && std::fabs(gyro[1]) < 3.0f &&
                          std::fabs(gyro[2]) < 3.0f;
        if (at_rest && slow) {
            const float blend = dt / (dt + 1.5f); // ~1.5 s time constant
            f->bias[0] += (gyro[0] - f->bias[0]) * blend;
            f->bias[1] += (gyro[1] - f->bias[1]) * blend;
            f->bias[2] += (gyro[2] - f->bias[2]) * blend;
            // Gently re-learn the rest magnitude too.
            f->rest_len += (alen - f->rest_len) * (0.1f * dt);
        }
    } else {
        f->rest_len = alen;
        f->has_rest = alen > 1.0f;
    }

    float w[3];
    w[0] = gyro[0] - f->bias[0];
    w[1] = gyro[1] - f->bias[1];
    w[2] = gyro[2] - f->bias[2];

    // --- 2. Gravity-vector correction (Mahony) --------------------------
    if (alen > 1.0f) {
        const float ax = accel_raw[0] / alen;
        const float ay = accel_raw[1] / alen;
        const float az = accel_raw[2] / alen;

        // Predicted gravity direction in the body frame.
        float gb[3];
        const Quat qinv = quat_conjugate(f->q);
        quat_rotate(qinv, kWorldUp, gb);

        // Error = measured_accel x predicted_gravity (cross product).
        const float ex = ay * gb[2] - az * gb[1];
        const float ey = az * gb[0] - ax * gb[2];
        const float ez = ax * gb[1] - ay * gb[0];

        // Integrate (with wind-up clamp) and feed back.
        f->ierr[0] += ex * ki * dt;
        f->ierr[1] += ey * ki * dt;
        f->ierr[2] += ez * ki * dt;
        const float ilen = std::sqrt(f->ierr[0] * f->ierr[0] +
                                     f->ierr[1] * f->ierr[1] +
                                     f->ierr[2] * f->ierr[2]);
        if (ilen > kIerrMax) {
            const float s = kIerrMax / ilen;
            f->ierr[0] *= s; f->ierr[1] *= s; f->ierr[2] *= s;
        }
        w[0] += kp * ex + f->ierr[0];
        w[1] += kp * ey + f->ierr[1];
        w[2] += kp * ez + f->ierr[2];
    }

    // --- 3. Integrate body-frame angular velocity -----------------------
    // dq/dt = 0.5 * q (x) (0, w), w in rad/s. First-order integration with
    // re-normalization is plenty accurate at 100-1000 Hz sample rates.
    constexpr float kDegToRad = 0.01745329252f;
    const float hdt = 0.5f * dt * kDegToRad;
    const Quat dq{0.0f, w[0] * hdt, w[1] * hdt, w[2] * hdt};
    const Quat qd = quat_mult(f->q, dq);
    f->q.w += qd.w;
    f->q.x += qd.x;
    f->q.y += qd.y;
    f->q.z += qd.z;
    f->q = quat_normalize(f->q);
}
