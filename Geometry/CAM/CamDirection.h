// CamDirection.h
#pragma once

#include "CamVector.h"

namespace PathForge {

class CamDirection {
public:
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    CamDirection() = default;

    explicit CamDirection(const CamVector& v) {
        double mag = v.magnitude();
        if (mag < 1e-10) {
            x = 0; y = 0; z = 1;
        } else {
            x = v.x / mag;
            y = v.y / mag;
            z = v.z / mag;
        }
    }

    explicit CamDirection(const CamPoint& start, const CamPoint& end) {
        CamVector v(end.x - start.x, end.y - start.y, end.z - start.z);
        *this = CamDirection(v);
    }

    CamDirection(double x_, double y_, double z_) {
        double mag = std::sqrt(x_ * x_ + y_ * y_ + z_ * z_);
        if (mag < 1e-10) {
            x = 0; y = 0; z = 1;
        } else {
            x = x_ / mag;
            y = y_ / mag;
            z = z_ / mag;
        }
    }

    static CamDirection axisX() { return CamDirection(1, 0, 0); }
    static CamDirection axisY() { return CamDirection(0, 1, 0); }
    static CamDirection axisZ() { return CamDirection(0, 0, 1); }

    static CamDirection fromDegrees(double rx, double ry, double rz) {
        double dr = 3.14159265358979 / 180.0;
        return CamDirection(rx * dr, ry * dr, rz * dr);
    }

    double magnitude() const { return 1.0; }

    CamDirection operator+(const CamDirection& other) const {
        return CamDirection(CamVector(*this) + CamVector(other));
    }

    CamDirection operator-(const CamDirection& other) const {
        return CamDirection(CamVector(*this) - CamVector(other));
    }

    CamDirection operator*(double scalar) const {
        return CamDirection(CamVector(*this) * scalar);
    }

    CamDirection operator-() const {
        return CamDirection(-x, -y, -z);
    }

    double dot(const CamDirection& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    double dot(const CamVector& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    CamVector cross(const CamDirection& other) const {
        return CamVector(*this).cross(CamVector(other));
    }

    CamVector cross(const CamVector& other) const {
        return CamVector(*this).cross(other);
    }

    double angle(const CamDirection& other) const {
        double cosAngle = dot(other);
        cosAngle = std::max(-1.0, std::min(1.0, cosAngle));
        return std::acos(cosAngle);
    }

    CamDirection reversed() const {
        return CamDirection(-x, -y, -z);
    }

    bool isParallel(const CamDirection& other, double tol = 1e-6) const {
        return std::abs(dot(other)) > (1.0 - tol);
    }

    bool isOpposite(const CamDirection& other, double tol = 1e-6) const {
        return dot(other) < (-1.0 + tol);
    }

    bool isPerpendicular(const CamDirection& other, double tol = 1e-6) const {
        return std::abs(dot(other)) < tol;
    }

    explicit operator CamVector() const {
        return CamVector(x, y, z);
    }
};

inline CamDirection operator*(double scalar, const CamDirection& d) {
    return d * scalar;
}

} // namespace PathForge