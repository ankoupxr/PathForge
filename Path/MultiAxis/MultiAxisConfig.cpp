#include "MultiAxisConfig.h"

#include <cmath>

namespace PathForge::MultiAxis {

bool AxisLimits::isWithinLimits(double x, double y, double z, double a, double b, double c) const {
    if (x < minX || x > maxX) return false;
    if (y < minY || y > maxY) return false;
    if (z < minZ || z > maxZ) return false;
    if (a < minA || a > maxA) return false;
    if (b < minB || b > maxB) return false;
    if (c < minC || c > maxC) return false;
    return true;
}

bool MultiAxisConfig::isValid() const {
    if (configuration == AxisConfiguration::Axis3) {
        return !outputA && !outputB && !outputC;
    }
    return true;
}

gp_Ax1 ToolOrientation::getToolAxis() const {
    return gp_Ax1(position, axis);
}

}
