#include "DrivePatternGenerator.h"

#include <cmath>
#include <algorithm>

#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

namespace PathForge::Path::Core {

DrivePatternGenerator::DrivePatternGenerator()
{
}

void DrivePatternGenerator::setPatternType(DrivePatternType type)
{
    m_patternType = type;
}

void DrivePatternGenerator::setBoundingBox(double xMin, double xMax, double yMin, double yMax)
{
    m_xMin = xMin;
    m_xMax = xMax;
    m_yMin = yMin;
    m_yMax = yMax;
}

void DrivePatternGenerator::setStep(double step)
{
    m_step = step;
}

void DrivePatternGenerator::setAngle(double angle)
{
    m_angle = angle;
}

void DrivePatternGenerator::setCenter(const gp_Pnt& center)
{
    m_center = center;
}

void DrivePatternGenerator::setRadius(double innerRadius, double outerRadius)
{
    m_innerRadius = innerRadius;
    m_outerRadius = outerRadius;
}

std::vector<DrivePoint> DrivePatternGenerator::generate()
{
    std::vector<DrivePoint> result;

    switch (m_patternType) {
        case DrivePatternType::ParallelLines:
        case DrivePatternType::Zigzag: {
            bool zigzag = (m_patternType == DrivePatternType::Zigzag);
            auto points = generateParallelLines(m_xMin, m_xMax, m_yMin, m_yMax, m_step, m_angle, zigzag);
            for (size_t i = 0; i < points.size(); ++i) {
                double u = i / static_cast<double>(std::max(points.size(), 1));
                result.emplace_back(points[i], u, 0.0);
            }
            break;
        }
        case DrivePatternType::Spiral: {
            auto points = generateSpiral(m_center, m_innerRadius, m_outerRadius, m_step, 3);
            for (size_t i = 0; i < points.size(); ++i) {
                double u = i / static_cast<double>(std::max(points.size(), 1));
                result.emplace_back(points[i], u, 0.0);
            }
            break;
        }
        case DrivePatternType::Circular: {
            auto points = generateCircular(m_center, m_innerRadius, m_outerRadius, m_step);
            for (size_t i = 0; i < points.size(); ++i) {
                double u = i / static_cast<double>(std::max(points.size(), 1));
                result.emplace_back(points[i], u, 0.0);
            }
            break;
        }
        case DrivePatternType::Contour:
        default:
            break;
    }

    return result;
}

std::vector<gp_Pnt> DrivePatternGenerator::generateParallelLines(
    double xMin, double xMax,
    double yMin, double yMax,
    double step,
    double angle,
    bool zigzag)
{
    std::vector<gp_Pnt> result;

    if (step <= 0.0) {
        return result;
    }

    double rotAngle = angle * M_PI / 180.0;
    double cosA = std::cos(rotAngle);
    double sinA = std::sin(rotAngle);

    int numLines = static_cast<int>((yMax - yMin) / step) + 1;

    for (int i = 0; i <= numLines; ++i) {
        double y = yMin + i * step;
        if (y > yMax) break;

        double x1 = xMin;
        double x2 = xMax;

        double rx1 = x1 * cosA - y * sinA;
        double ry = x1 * sinA + y * cosA;

        double rx2 = x2 * cosA - y * sinA;
        double ry2 = x2 * sinA + y * cosA;

        if (zigzag && (i % 2 == 1)) {
            result.emplace_back(rx2, ry2, 0.0);
            result.emplace_back(rx1, ry, 0.0);
        } else {
            result.emplace_back(rx1, ry, 0.0);
            result.emplace_back(rx2, ry2, 0.0);
        }
    }

    return result;
}

std::vector<gp_Pnt> DrivePatternGenerator::generateSpiral(
    const gp_Pnt& center,
    double innerRadius,
    double outerRadius,
    double step,
    int turns)
{
    std::vector<gp_Pnt> result;

    if (step <= 0.0 || outerRadius <= innerRadius) {
        return result;
    }

    int numPoints = static_cast<int>((outerRadius - innerRadius) / step) * 10;
    if (numPoints < 20) numPoints = 20;

    double angleStep = turns * 2 * M_PI / numPoints;

    for (int i = 0; i <= numPoints; ++i) {
        double angle = i * angleStep;
        double radius = innerRadius + (outerRadius - innerRadius) * i / numPoints;

        double x = center.X() + radius * std::cos(angle);
        double y = center.Y() + radius * std::sin(angle);

        result.emplace_back(x, y, center.Z());
    }

    return result;
}

std::vector<gp_Pnt> DrivePatternGenerator::generateCircular(
    const gp_Pnt& center,
    double innerRadius,
    double outerRadius,
    double step)
{
    std::vector<gp_Pnt> result;

    if (step <= 0.0 || outerRadius <= innerRadius) {
        return result;
    }

    int numRings = static_cast<int>((outerRadius - innerRadius) / step) + 1;

    for (int r = 0; r < numRings; ++r) {
        double radius = innerRadius + r * step;

        int numPoints = static_cast<int>(2 * M_PI * radius / step);
        if (numPoints < 8) numPoints = 8;

        double angleStep = 2 * M_PI / numPoints;

        for (int i = 0; i < numPoints; ++i) {
            double angle = i * angleStep;
            double x = center.X() + radius * std::cos(angle);
            double y = center.Y() + radius * std::sin(angle);
            result.emplace_back(x, y, center.Z());
        }
    }

    return result;
}

}