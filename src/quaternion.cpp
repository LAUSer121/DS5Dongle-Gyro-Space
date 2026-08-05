//
// quaternion.cpp - minimal single-precision quaternion math (RP2350).
//
#include "quaternion.h"

namespace {
constexpr float kPi = 3.14159265358979f;
}

void quat_rotate(const Quat &q, const float v[3], float out[3]) {
    // v' = v + 2w(u x v) + 2u x (u x v), with u = (q.x,q.y,q.z)
    const float t0 = 2.0f * (q.y * v[2] - q.z * v[1]);
    const float t1 = 2.0f * (q.z * v[0] - q.x * v[2]);
    const float t2 = 2.0f * (q.x * v[1] - q.y * v[0]);
    out[0] = v[0] + q.w * t0 + (q.y * t2 - q.z * t1);
    out[1] = v[1] + q.w * t1 + (q.z * t0 - q.x * t2);
    out[2] = v[2] + q.w * t2 + (q.x * t1 - q.y * t0);
}

Quat quat_from_axis_angle(float ax, float ay, float az, float angle) {
    const float h = angle * 0.5f;
    const float s = std::sin(h);
    return quat_normalize(Quat{std::cos(h), ax * s, ay * s, az * s});
}

Quat quat_from_unit_vectors(const float from[3], const float to[3]) {
    const float dot = from[0] * to[0] + from[1] * to[1] + from[2] * to[2];
    if (dot > 0.9999f) return quat_identity();
    if (dot < -0.9999f) {
        // 180 degrees about any axis perpendicular to `from`
        float ax = 1.0f, ay = 0.0f, az = 0.0f;
        if (std::fabs(from[0]) < 0.9f) { ax = 0.0f; ay = 1.0f; az = 0.0f; }
        return quat_from_axis_angle(ax, ay, az, kPi);
    }
    const float cx = from[1] * to[2] - from[2] * to[1];
    const float cy = from[2] * to[0] - from[0] * to[2];
    const float cz = from[0] * to[1] - from[1] * to[0];
    const float cl = std::sqrt(cx * cx + cy * cy + cz * cz);
    if (cl < 1e-8f) return quat_identity();
    const float half = std::sin(std::acos(dot) * 0.5f);
    return Quat{std::cos(std::acos(dot) * 0.5f), (cx / cl) * half, (cy / cl) * half, (cz / cl) * half};
}
