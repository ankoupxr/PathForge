// CamPoint.cpp
#include "CamPoint.h"
#include "CamPoint2D.h"

namespace PathForge {

CamPoint::CamPoint(const CamPoint2D& p2d, double z_)
    : x(p2d.u), y(p2d.v), z(z_) {}

CamPoint CamPoint::cross(const CamPoint& other) const {
    return CamPoint(
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x
    );
}

} // namespace PathForge