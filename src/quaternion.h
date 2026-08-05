//
// quaternion.h - minimal single-precision quaternion math for the RP2350
// gyro-aiming pipeline. No dynamic memory, no external math library.
// Convention: a quaternion q represents the rotation that maps the SENSOR
// (controller) frame onto the WORLD frame (world +Y = gravity up).
//
#ifndef DS5_QUATERNION_H
#define DS5_QUATERNION_H

#include <cmath>

struct Quat {
    float w, x, y, z;
};

static inline Quat quat_identity() { return Quat{1.0f, 0.0f, 0.0f, 0.0f}; }

// Normalize a quaternion to unit length (guards near-zero input).
static inline Quat quat_normalize(Quat q) {
    const float n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n < 1e-8f) return quat_identity();
    const float inv = 1.0f / n;
    return Quat{q.w * inv, q.x * inv, q.y * inv, q.z * inv};
}

// Hamilton product. q = a * b applies b first, then a.
static inline Quat quat_mult(Quat a, Quat b) {
    return Quat{
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
}

// Inverse of a unit quaternion is its conjugate.
static inline Quat quat_conjugate(Quat q) { return Quat{q.w, -q.x, -q.y, -q.z}; }

// Rotate a 3-vector by a UNIT quaternion (body -> world when q is the body->world
// orientation). out may alias v.
void quat_rotate(const Quat &q, const float v[3], float out[3]);

// Unit quaternion representing `angle` radians about axis (ax,ay,az).
Quat quat_from_axis_angle(float ax, float ay, float az, float angle);

// Shortest-arc quaternion rotating unit vector `from` onto unit vector `to`.
Quat quat_from_unit_vectors(const float from[3], const float to[3]);

#endif // DS5_QUATERNION_H
