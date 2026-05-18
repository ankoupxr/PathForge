#include "Axis5Interpolator.h"

#include "ToolAxisController.h"

#include <BRep_Tool.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <Geom_Surface.hxx>
#include <GeomAbs_SurfaceType.hxx>

#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Vec.hxx>
#include <gp_Ax1.hxx>
#include <gp_Circ.hxx>

#include <algorithm>
#include <cmath>

namespace PathForge::MultiAxis {

Axis5Interpolator::Axis5Interpolator() {
    m_config.configuration = AxisConfiguration::Axis5;
    m_config.interpolationType = InterpolationType::Linear;
    m_config.outputA = true;
    m_config.outputB = true;
    m_config.outputC = false;
    m_config.smoothAxisTransitions = true;
}

void Axis5Interpolator::setConfiguration(const MultiAxisConfig& config) {
    m_config = config;
}

void Axis5Interpolator::setToolAxisController(ToolAxisController* controller) {
    m_toolAxisController = controller;
}

void Axis5Interpolator::setSurface(const TopoDS_Shape& surface) {
    m_surface = surface;
}

void Axis5Interpolator::setMachiningFace(const TopoDS_Face& face) {
    m_machiningFace = face;
}

void Axis5Interpolator::setFeedrate(double feedrate) {
    m_feedrate = feedrate;
}

void Axis5Interpolator::setToolDiameter(double diameter) {
    m_toolDiameter = diameter;
}

void Axis5Interpolator::setTolerance(double tolerance) {
    m_tolerance = tolerance;
}

MultiAxisPath Axis5Interpolator::interpolateLinearPath(const gp_Pnt& start, const gp_Pnt& end,
                                                        const gp_Dir& startNormal, 
                                                        const gp_Dir& endNormal) {
    MultiAxisPath path;

    double distance = start.Distance(end);
    InterpolationParameters params;
    params.pointCount = std::max(2, static_cast<int>(distance / m_tolerance));
    params.maxAngleVariation = 5.0 * M_PI / 180.0;

    auto points = generateLinearInterpolation(start, end, params);

    for (size_t i = 0; i < points.size(); ++i) {
        double t = static_cast<double>(i) / (points.size() - 1);
        gp_Dir normal = interpolateNormal(startNormal, endNormal, t);

        auto point = interpolatePoint(points[i], normal, t);
        path.addPoint(point);
    }

    m_interpolatedPointCount = static_cast<int>(path.pointCount());
    return path;
}

MultiAxisPath Axis5Interpolator::interpolateArcPath(const gp_Pnt& start, const gp_Pnt& end,
                                                      const gp_Pnt& center, const gp_Dir& normal,
                                                      bool isClockwise) {
    MultiAxisPath path;

    double radius = start.Distance(center);

    double angle1 = std::atan2(start.Y() - center.Y(), start.X() - center.X());
    double angle2 = std::atan2(end.Y() - center.Y(), end.X() - center.X());

    if (isClockwise && angle2 < angle1) {
        angle2 += 2.0 * M_PI;
    } else if (!isClockwise && angle2 > angle1) {
        angle2 -= 2.0 * M_PI;
    }

    double totalAngle = std::abs(angle2 - angle1);
    int numPoints = std::max(4, static_cast<int>(totalAngle * radius / m_tolerance));

    for (int i = 0; i <= numPoints; ++i) {
        double t = static_cast<double>(i) / numPoints;
        double angle = angle1 + t * (angle2 - angle1);

        gp_Pnt position = calculateHelixPoint(center, radius, angle);
        gp_Dir normalDir = calculateHelixNormal(angle);

        auto point = interpolatePoint(position, normalDir, t);
        path.addPoint(point);
    }

    m_interpolatedPointCount = static_cast<int>(path.pointCount());
    return path;
}

MultiAxisPath Axis5Interpolator::interpolateCurvePath(const std::vector<gp_Pnt>& controlPoints,
                                                        const std::vector<gp_Dir>& normals) {
    MultiAxisPath path;

    if (controlPoints.size() < 2 || controlPoints.size() != normals.size()) {
        m_lastError = "Invalid control points or normals";
        return path;
    }

    for (size_t i = 1; i < controlPoints.size(); ++i) {
        auto segmentPath = interpolateLinearPath(
            controlPoints[i - 1],
            controlPoints[i],
            normals[i - 1],
            normals[i]
        );

        if (i == 1) {
            path.addPoints(segmentPath.getPoints());
        } else {
            auto points = segmentPath.getPoints();
            if (points.size() > 1) {
                path.addPoints(std::vector<MultiAxisPoint>(points.begin() + 1, points.end()));
            }
        }
    }

    m_interpolatedPointCount = static_cast<int>(path.pointCount());
    return path;
}

bool Axis5Interpolator::isValid() const {
    if (!m_toolAxisController) {
        m_lastError = "Tool axis controller not set";
        return false;
    }
    return true;
}

MultiAxisPoint Axis5Interpolator::interpolatePoint(const gp_Pnt& position, 
                                                    const gp_Dir& normal, double t) {
    MultiAxisPoint point(position);
    point.surfaceNormal = normal;
    point.toolAxis = normal;
    point.feedrate = m_feedrate;
    point.pointIndex = static_cast<int>(t * 1000);

    if (m_config.smoothAxisTransitions && m_toolAxisController) {
        m_toolAxisController->calculateTiltAngles(normal, point.axisA, point.axisB);
        point.setTiltedAxis(point.axisA, point.axisB);
    } else {
        calculateAxisAngles(normal, point.axisA, point.axisB);
        point.setTiltedAxis(point.axisA, point.axisB);
    }

    return point;
}

gp_Dir Axis5Interpolator::interpolateNormal(const gp_Dir& startNormal, 
                                             const gp_Dir& endNormal, double t) {
    gp_Vec start(startNormal.X(), startNormal.Y(), startNormal.Z());
    gp_Vec end(endNormal.X(), endNormal.Y(), endNormal.Z());

    gp_Vec result = start * (1 - t) + end * t;
    result.Normalize();

    return gp_Dir(result.X(), result.Y(), result.Z());
}

void Axis5Interpolator::calculateAxisAngles(const gp_Dir& normal, 
                                             double& aAngle, double& bAngle) {
    aAngle = std::asin(std::max(-1.0, std::min(1.0, -normal.Y()))) * 180.0 / M_PI;
    bAngle = std::asin(std::max(-1.0, std::min(1.0, -normal.X()))) * 180.0 / M_PI;

    if (std::abs(normal.Z()) > 0.001) {
        double xyLen = std::sqrt(normal.X() * normal.X() + normal.Y() * normal.Y());
        bAngle = std::atan2(-normal.X(), -normal.Y()) * 180.0 / M_PI;
    }
}

gp_Pnt Axis5Interpolator::calculateHelixPoint(const gp_Pnt& center, double radius, double angle) {
    double x = center.X() + radius * std::cos(angle);
    double y = center.Y() + radius * std::sin(angle);
    double z = center.Z();
    return gp_Pnt(x, y, z);
}

gp_Dir Axis5Interpolator::calculateHelixNormal(double angle) {
    return gp_Dir(0, 0, 1);
}

std::vector<gp_Pnt> Axis5Interpolator::generateLinearInterpolation(
    const gp_Pnt& start, const gp_Pnt& end, 
    const InterpolationParameters& params) {

    std::vector<gp_Pnt> points;

    for (int i = 0; i <= params.pointCount; ++i) {
        double t = static_cast<double>(i) / params.pointCount;
        double x = start.X() + t * (end.X() - start.X());
        double y = start.Y() + t * (end.Y() - start.Y());
        double z = start.Z() + t * (end.Z() - start.Z());
        points.push_back(gp_Pnt(x, y, z));
    }

    return points;
}

std::vector<gp_Pnt> Axis5Interpolator::generateArcInterpolation(
    const gp_Pnt& start, const gp_Pnt& end,
    const gp_Pnt& center, double radius,
    const InterpolationParameters& params) {

    std::vector<gp_Pnt> points;

    double angle1 = std::atan2(start.Y() - center.Y(), start.X() - center.X());
    double angle2 = std::atan2(end.Y() - center.Y(), end.X() - center.X());
    double totalAngle = std::abs(angle2 - angle1);

    int numPoints = std::max(4, static_cast<int>(totalAngle * radius / m_tolerance));

    for (int i = 0; i <= numPoints; ++i) {
        double t = static_cast<double>(i) / numPoints;
        double angle = angle1 + t * (angle2 - angle1);
        points.push_back(calculateHelixPoint(center, radius, angle));
    }

    return points;
}

}
