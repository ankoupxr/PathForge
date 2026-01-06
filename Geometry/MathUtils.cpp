#include "MathUtils.h"
#include <cmath>

namespace PathForge {
    namespace Geometry {

        double MathUtils::angleBetween(const gp_Vec& a, const gp_Vec& b) {
            double na = a.Magnitude();
            double nb = b.Magnitude();
            if (na <= 0.0 || nb <= 0.0) return 0.0;
            double dot = a.Dot(b) / (na * nb);
            if (dot > 1.0) dot = 1.0;
            if (dot < -1.0) dot = -1.0;
            return std::acos(dot);
        }

        double MathUtils::distance(const gp_Pnt& a, const gp_Pnt& b) {
            return a.Distance(b);
        }

        bool MathUtils::isParallel(const gp_Vec& a, const gp_Vec& b, double tol) {
            double ang = angleBetween(a, b);
            return (std::abs(ang) < tol) || (std::abs(ang - M_PI) < tol);
        }

        bool MathUtils::isPerpendicular(const gp_Vec& a, const gp_Vec& b, double tol) {
            double ang = angleBetween(a, b);
            return std::abs(ang - M_PI / 2.0) < tol;
        }

    } // namespace Geometry
} // namespace PathForge
