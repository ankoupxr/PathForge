// CamVector.h
#pragma once

#include "CamPoint.h"

namespace PathForge {

class CamDirection;

class CamVector {
public:
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    CamVector() = default;

    explicit CamVector(const CamPoint& p)
        : x(p.x), y(p.y), z(p.z) {}

    explicit CamVector(double x_, double y_, double z_)
        : x(x_), y(y_), z(z_) {}

    static CamVector zero() { return CamVector(0, 0, 0); }

    static CamVector axisX() { return CamVector(1, 0, 0); }
    static CamVector axisY() { return CamVector(0, 1, 0); }
    static CamVector axisZ() { return CamVector(0, 0, 1); }

    double magnitude() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    double length() const { return magnitude(); }

    double lengthSq() const {
        return x * x + y * y + z * z;
    }

    CamVector normalized() const {
        double mag = magnitude();
        if (mag < 1e-10) return CamVector::zero();
        return CamVector(x / mag, y / mag, z / mag);
    }

    CamVector& normalize() {
        *this = normalized();
        return *this;
    }

    CamVector operator+(const CamVector& other) const {
        return CamVector(x + other.x, y + other.y, z + other.z);
    }

    CamVector operator-(const CamVector& other) const {
        return CamVector(x - other.x, y - other.y, z - other.z);
    }

    CamVector operator*(double scalar) const {
        return CamVector(x * scalar, y * scalar, z * scalar);
    }

    CamVector operator/(double scalar) const {
        return CamVector(x / scalar, y / scalar, z / scalar);
    }

    CamVector operator-() const {
        return CamVector(-x, -y, -z);
    }

    CamVector& operator+=(const CamVector& other) {
        x += other.x; y += other.y; z += other.z;
        return *this;
    }

    CamVector& operator-=(const CamVector& other) {
        x -= other.x; y -= other.y; z -= other.z;
        return *this;
    }

    CamVector& operator*=(double scalar) {
        x *= scalar; y *= scalar; z *= scalar;
        return *this;
    }

    double dot(const CamVector& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    CamVector cross(const CamVector& other) const {
        return CamVector(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }

    CamVector& crossAssign(const CamVector& other) {
        double nx = y * other.z - z * other.y;
        double ny = z * other.x - x * other.z;
        double nz = x * other.y - y * other.x;
        x = nx; y = ny; z = nz;
        return *this;
    }

    double angle(const CamVector& other) const {
        double dotVal = dot(other);
        double mag1 = magnitude();
        double mag2 = other.magnitude();
        if (mag1 < 1e-10 || mag2 < 1e-10) return 0.0;
        double cosAngle = std::max(-1.0, std::min(1.0, dotVal / (mag1 * mag2)));
        return std::acos(cosAngle);
    }

    CamVector projectOnto(const CamVector& other) const {
        double dotVal = dot(other);
        double magSq = other.lengthSq();
        if (magSq < 1e-10) return CamVector::zero();
        return other * (dotVal / magSq);
    }

    CamVector reflect(const CamVector& normal) const {
        return *this - normal * (2.0 * this->dot(normal));
    }

    bool isParallel(const CamVector& other, double tol = 1e-6) const {
        return cross(other).magnitude() < tol;
    }

    bool isPerpendicular(const CamVector& other, double tol = 1e-6) const {
        return std::abs(dot(other)) < tol;
    }

    CamVector lerp(const CamVector& target, double t) const {
        return *this + (target - *this) * t;
    }

    explicit operator CamPoint() const {
        return CamPoint(x, y, z);
    }

    explicit operator CamDirection() const;
};

inline CamVector operator*(double scalar, const CamVector& v) {
    return v * scalar;
}

} // namespace PathForge