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
    double r_xy = std::sqrt(x*x + y*y);
    double z2   = z - L_HIP;
    double d    = std::sqrt(r_xy*r_xy + z2*z2);

    return (d >= std::fabs(L_THIGH - L_CALF)) &&
           (d <= (L_THIGH + L_CALF));
}

inline bool checkJointLimit(double q1, double q2, double q3)
{
    using namespace GO1_LIMIT;
    return (q1 >= HIP_MIN   && q1 <= HIP_MAX) &&
           (q2 >= THIGH_MIN && q2 <= THIGH_MAX) &&
           (q3 >= CALF_MIN  && q3 <= CALF_MAX);
}

inline Vec3 solveFK_FR(double q_hip, double q_thigh, double q_calf)
{
    double r =
        L_THIGH * cos(q_thigh)
      + L_CALF  * cos(q_thigh + q_calf);

    Vec3 p;
    p.x() = cos(q_hip) * r;
    p.y() = sin(q_hip) * r;
    p.z() = L_HIP
          + L_THIGH * sin(q_thigh)
          + L_CALF  * sin(q_thigh + q_calf);

    return p;
}


inline bool solveIK_FR(
    double x, double y, double z,
    double& q_hip,
    double& q_thigh,
    double& q_calf)
{
    q_hip = std::atan2(y, x);

    double r_xy = std::sqrt(x*x + y*y);

    double z2 = z - L_HIP;
    double d  = std::sqrt(r_xy*r_xy + z2*z2);

    if(d > (L_THIGH + L_CALF)) return false;
    if(d < std::fabs(L_THIGH - L_CALF)) return false;

    double c = (L_THIGH*L_THIGH + L_CALF*L_CALF - d*d) /
               (2.0 * L_THIGH * L_CALF);

    if(c < -1.0 || c > 1.0) return false;

    double theta = std::acos(c);
    q_calf = -(M_PI - theta);

    double alpha = std::atan2(z2, r_xy);
    double beta  = std::acos(
        (L_THIGH*L_THIGH + d*d - L_CALF*L_CALF) /
        (2.0 * L_THIGH * d)
    );

    q_thigh = alpha - beta;

    return true;
}

inline Vec3 path(const Vec3& p0, const Vec3& p1, double t)
{
    return (1.0 - t) * p0 + t * p1;
}
