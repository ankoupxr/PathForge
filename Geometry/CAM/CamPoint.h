// CamPoint.h
#pragma once

#include <cmath>
#include <ostream>

namespace PathForge {

class CamPoint2D;

class CamPoint {
public:
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    CamPoint() = default;

    constexpr CamPoint(double x_, double y_, double z_) noexcept
        : x(x_), y(y_), z(z_) {}

    explicit CamPoint(const CamPoint2D& p2d, double z_ = 0.0);

    static CamPoint origin() { return CamPoint(0, 0, 0); }

    double magnitude() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    double distanceTo(const CamPoint& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        double dz = z - other.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    CamPoint operator+(const CamPoint& other) const {
        return CamPoint(x + other.x, y + other.y, z + other.z);
    }

    CamPoint operator-(const CamPoint& other) const {
        return CamPoint(x - other.x, y - other.y, z - other.z);
    }

    CamPoint operator*(double scalar) const {
        return CamPoint(x * scalar, y * scalar, z * scalar);
    }

    CamPoint operator/(double scalar) const {
        return CamPoint(x / scalar, y / scalar, z / scalar);
    }

    CamPoint& operator+=(const CamPoint& other) {
        x += other.x; y += other.y; z += other.z;
        return *this;
    }

    CamPoint& operator-=(const CamPoint& other) {
        x -= other.x; y -= other.y; z -= other.z;
        return *this;
    }

    CamPoint& operator*=(double scalar) {
        x *= scalar; y *= scalar; z *= scalar;
        return *this;
    }

    bool operator==(const CamPoint& other) const {
        return std::abs(x - other.x) < 1e-10 &&
               std::abs(y - other.y) < 1e-10 &&
               std::abs(z - other.z) < 1e-10;
    }

    bool isEqual(const CamPoint& other, double tol = 1e-6) const {
        return distanceTo(other) < tol;
    }

    CamPoint lerp(const CamPoint& target, double t) const {
        return *this + (target - *this) * t;
    }

    CamPoint2D to2D() const {
        return CamPoint2D(x, y);
    }

    double dot(const CamPoint& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    CamPoint cross(const CamPoint& other) const;

    CamPoint normalized() const {
        double mag = magnitude();
        if (mag < 1e-10) return *this;
        return *this / mag;
    }

    CamPoint& normalize() {
        *this = normalized();
        return *this;
    }

    void set(double x_, double y_, double z_) {
        x = x_; y = y_; z = z_;
    }

    void setX(double x_) { x = x_; }
    void setY(double y_) { y = y_; }
    void setZ(double z_) { z = z_; }

    double getX() const { return x; }
    double getY() const { return y; }
    double getZ() const { return z; }

    struct Hash {
        size_t operator()(const CamPoint& p) const {
            size_t h1 = std::hash<double>{}(p.x);
            size_t h2 = std::hash<double>{}(p.y);
            size_t h3 = std::hash<double>{}(p.z);
            return h1 ^ (h2 << 1) ^ (h3 << 2);
        }
    };
};

inline CamPoint operator*(double scalar, const CamPoint& p) {
    return p * scalar;
}

inline std::ostream& operator<<(std::ostream& os, const CamPoint& p) {
    os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
    return os;
}

} // namespace PathForge