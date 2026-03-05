#pragma once
#include <cmath>

constexpr double L_HIP   = 0.083;
constexpr double L_THIGH = 0.213;
constexpr double L_CALF  = 0.213;

namespace GO1_LIMIT {
constexpr double HIP_MIN   = -1.047;
constexpr double HIP_MAX   =  1.047;
constexpr double THIGH_MIN = -0.663;
constexpr double THIGH_MAX =  2.966;
constexpr double CALF_MIN  = -2.721;
constexpr double CALF_MAX  = -0.837;
}

static const Vec3 HIP_FR(0.1881, -0.04675, 0.0);
static const Vec3 CAM_BASE(0.21, 0.0, 0.25);

inline Vec3 camToBase(const Vec3& p_cam)
{
    return p_cam + CAM_BASE;
}

inline Vec3 baseToLegFR(const Vec3& p_base)
{
    return p_base - HIP_FR;
}

inline bool inWorkspace(double x, double y, double z)
{
    double r = std::sqrt(x*x + y*y);
    double z2   = z - L_HIP;
    double d    = std::sqrt(r*r + z2*z2);

    return (d >= std::fabs(L_THIGH - L_CALF)) &&
           (d <= (L_THIGH + L_CALF));
}

inline bool checkJointLimit(double q_hip, double q_thigh, double q_calf)
{
    using namespace GO1_LIMIT;
    return (q_hip >= HIP_MIN     && q_hip <= HIP_MAX) &&
           (q_thigh >= THIGH_MIN && q_thigh <= THIGH_MAX) &&
           (q_calf >= CALF_MIN   && q_calf <= CALF_MAX);
}

inline Vec3 solveFK_FR(double q_hip, double q_thigh, double q_calf)
{
    double r =
        L_THIGH * std::cos(q_thigh)
      + L_CALF  * std::cos(q_thigh + q_calf);

    Vec3 p;
    p.x() = std::cos(q_hip) * r;
    p.y() = std::sin(q_hip) * r;
    p.z() = L_HIP
          + L_THIGH * std::sin(q_thigh)
          + L_CALF  * std::sin(q_thigh + q_calf);

    return p;
}

inline bool solveIK_FR(
    double x, double y, double z,
    double& q_hip,
    double& q_thigh,
    double& q_calf)
{
    // Hip
    q_hip = std::atan2(y, x);

    double r  = std::sqrt(x*x + y*y);
    double z2 = z - L_HIP;
    double d2 = r*r + z2*z2;

    // Calf
    double c = (d2 - L_THIGH*L_THIGH - L_CALF*L_CALF)
             / (2.0 * L_THIGH * L_CALF);

    if (c >  1.0) c =  1.0;
    if (c < -1.0) c = -1.0;

    double s_sq = 1.0 - c*c;
    if (s_sq < 0.0) s_sq = 0.0;

    double s = -std::sqrt(s_sq); 
    q_calf = std::atan2(s, c);

    // Thigh
    double k1 = L_THIGH + L_CALF * std::cos(q_calf);
    double k2 = L_CALF  * std::sin(q_calf);

    q_thigh = std::atan2(z2, r)
            - std::atan2(k2, k1);

    return true;
}

inline Vec3 path(const Vec3& p0, const Vec3& p1, double t)
{
    return (1.0 - t) * p0 + t * p1;
}
