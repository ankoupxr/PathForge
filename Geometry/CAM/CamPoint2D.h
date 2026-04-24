// CamPoint2D.h
#pragma once

#include <cmath>
#include <ostream>

namespace PathForge {

class CamPoint2D {
public:
    double u = 0.0;
    double v = 0.0;

    CamPoint2D() = default;

    constexpr CamPoint2D(double u_, double v_) noexcept
        : u(u_), v(v_) {}

    static CamPoint2D origin() { return CamPoint2D(0, 0); }

    double magnitude() const {
        return std::sqrt(u * u + v * v);
    }

    double distanceTo(const CamPoint2D& other) const {
        double du = u - other.u;
        double dv = v - other.v;
        return std::sqrt(du * du + dv * dv);
    }

    CamPoint2D operator+(const CamPoint2D& other) const {
        return CamPoint2D(u + other.u, v + other.v);
    }

    CamPoint2D operator-(const CamPoint2D& other) const {
        return CamPoint2D(u - other.u, v - other.v);
    }

    CamPoint2D operator*(double scalar) const {
        return CamPoint2D(u * scalar, v * scalar);
    }

    CamPoint2D operator/(double scalar) const {
        return CamPoint2D(u / scalar, v / scalar);
    }

    CamPoint2D& operator+=(const CamPoint2D& other) {
        u += other.u; v += other.v;
        return *this;
    }

    CamPoint2D& operator-=(const CamPoint2D& other) {
        u -= other.u; v -= other.v;
        return *this;
    }

    bool operator==(const CamPoint2D& other) const {
        return std::abs(u - other.u) < 1e-10 &&
               std::abs(v - other.v) < 1e-10;
    }

    bool isEqual(const CamPoint2D& other, double tol = 1e-6) const {
        return distanceTo(other) < tol;
    }

    CamPoint2D normalized() const {
        double mag = magnitude();
        if (mag < 1e-10) return *this;
        return *this / mag;
    }

    CamPoint2D& normalize() {
        *this = normalized();
        return *this;
    }

    double dot(const CamPoint2D& other) const {
        return u * other.u + v * other.v;
    }

    double cross(const CamPoint2D& other) const {
        return u * other.v - v * other.u;
    }

    void set(double u_, double v_) {
        u = u_; v = v_;
    }

    struct Hash {
        size_t operator()(const CamPoint2D& p) const {
            size_t h1 = std::hash<double>{}(p.u);
            size_t h2 = std::hash<double>{}(p.v);
            return h1 ^ (h2 << 1);
        }
    };
};

inline std::ostream& operator<<(std::ostream& os, const CamPoint2D& p) {
    os << "(" << p.u << ", " << p.v << ")";
    return os;
}

} // namespace PathForge