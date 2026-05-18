#include "MultiAxisEngine.h"

#include <algorithm>
#include <cmath>

namespace PathForge::MultiAxis {

MultiAxisEngine::MultiAxisEngine() {
    m_config.configuration = AxisConfiguration::Axis5;
    m_config.outputA = true;
    m_config.outputB = true;
    m_config.outputC = false;
    m_config.checkInterference = true;
    m_config.collisionClearance = 1.0;
    m_config.tiltedHead.maxTiltAngle = 30.0;
    m_config.tiltedHead.minTiltAngle = -30.0;

    setupComponents();
}

void MultiAxisEngine::setupComponents() {
    m_toolAxisController = std::make_unique<ToolAxisController>();
    m_toolAxisController->setConfiguration(m_config);

    m_axis3Plus2 = std::make_unique<Axis3Plus2Positioning>();
    m_axis3Plus2->setConfiguration(m_config);
    m_axis3Plus2->setToolAxisController(m_toolAxisController.get());

    m_axis5Interpolator = std::make_unique<Axis5Interpolator>();
    m_axis5Interpolator->setConfiguration(m_config);
    m_axis5Interpolator->setToolAxisController(m_toolAxisController.get());
    m_axis5Interpolator->setTolerance(m_interpolationTolerance);
    m_axis5Interpolator->setFeedrate(m_feedrate);

    m_interferenceChecker = std::make_unique<InterferenceChecker>();
    m_interferenceChecker->setConfiguration(m_config);
    m_interferenceChecker->setInterferenceTolerance(m_interferenceTolerance);
}

void MultiAxisEngine::setConfiguration(const MultiAxisConfig& config) {
    m_config = config;
    setupComponents();
}

void MultiAxisEngine::setWorkpiece(const TopoDS_Shape& workpiece) {
    m_workpiece = workpiece;
    m_interferenceChecker->setWorkpiece(workpiece);
    m_toolAxisController->setSurface(nullptr);
}

void MultiAxisEngine::setStockShape(const TopoDS_Shape& stock) {
    m_stockShape = stock;
    m_interferenceChecker->setStockShape(stock);
}

void MultiAxisEngine::setMachiningFace(const TopoDS_Face& face) {
    m_machiningFace = face;
    m_axis5Interpolator->setMachiningFace(face);
    if (!face.IsNull()) {
        m_axis3Plus2->addMachiningPlane(face);
    }
}

void MultiAxisEngine::setToolGeometry(double diameter, double length,
                                        double holderDiameter, double holderLength) {
    m_interferenceChecker->setToolGeometry(diameter, length, holderDiameter, holderLength);
    m_axis5Interpolator->setToolDiameter(diameter);
}

void MultiAxisEngine::setControlMode(AxisControlMode mode) {
    m_toolAxisController->setControlMode(mode);

    switch (mode) {
        case AxisControlMode::Normal:
            m_config.configuration = AxisConfiguration::Axis3Plus2;
            break;
        case AxisControlMode::Tilted:
        case AxisControlMode::Interpolated:
            m_config.configuration = AxisConfiguration::Axis5;
            break;
        case AxisControlMode::Fixed:
            m_config.configuration = AxisConfiguration::Axis3Plus2;
            break;
    }
}

MultiAxisResult MultiAxisEngine::generate5AxisPath(const std::vector<gp_Pnt>& points,
                                                     const std::vector<gp_Dir>& normals) {
    MultiAxisResult result = createResult();

    if (points.size() < 2 || points.size() != normals.size()) {
        result.success = false;
        result.errorMessage = "Invalid input points or normals";
        return result;
    }

    m_lastResult.clear();

    MultiAxisPath fullPath;

    for (size_t i = 1; i < points.size(); ++i) {
        auto segmentPath = m_axis5Interpolator->interpolateLinearPath(
            points[i - 1],
            points[i],
            normals[i - 1],
            normals[i]
        );

        if (i == 1) {
            fullPath.addPoints(segmentPath.getPoints());
        } else {
            auto segPoints = segmentPath.getPoints();
            if (segPoints.size() > 1) {
                fullPath.addPoints(std::vector<MultiAxisPoint>(segPoints.begin() + 1, segPoints.end()));
            }
        }
    }

    if (m_config.checkInterference) {
        result.interferences = m_interferenceChecker->checkPath(fullPath);
        result.hasInterferences = !result.interferences.empty();

        if (result.hasInterferences) {
            result.riskLevel = std::min(1.0, static_cast<double>(result.interferences.size()) / 10.0);
        }
    }

    result.totalPoints = static_cast<int>(fullPath.pointCount());
    result.interpolatedPoints = m_axis5Interpolator->getInterpolatedPointCount();
    result.totalPathLength = fullPath.totalLength();
    result.estimatedTime = fullPath.estimatedTime();
    result.success = true;

    m_lastResult = fullPath;

    return result;
}

MultiAxisResult MultiAxisEngine::generate3Plus2Path(const std::vector<gp_Pnt>& points) {
    MultiAxisResult result = createResult();

    if (points.empty()) {
        result.success = false;
        result.errorMessage = "No input points";
        return result;
    }

    m_lastResult.clear();

    if (points.size() >= 2) {
        auto positioningPath = m_axis3Plus2->generatePositioningPath(points.front(), points.back());
        result.positioningPoints = static_cast<int>(positioningPath.pointCount());
        m_lastResult = positioningPath;
    } else {
        for (const auto& pt : points) {
            MultiAxisPoint point(pt);
            point.setAxisAngles(0.0, 0.0, 0.0);
            point.feedrate = m_feedrate;
            m_lastResult.addPoint(point);
        }
    }

    if (m_config.checkInterference) {
        result.interferences = m_interferenceChecker->checkPath(m_lastResult);
        result.hasInterferences = !result.interferences.empty();
    }

    result.totalPoints = static_cast<int>(m_lastResult.pointCount());
    result.totalPathLength = m_lastResult.totalLength();
    result.estimatedTime = m_lastResult.estimatedTime();
    result.success = true;

    return result;
}

void MultiAxisEngine::setFeedrate(double feedrate) {
    m_feedrate = feedrate;
    m_axis5Interpolator->setFeedrate(feedrate);
}

void MultiAxisEngine::setSpindleSpeed(double speed) {
    m_spindleSpeed = speed;
}

void MultiAxisEngine::setInterpolationTolerance(double tolerance) {
    m_interpolationTolerance = tolerance;
    m_axis5Interpolator->setTolerance(tolerance);
}

void MultiAxisEngine::setInterferenceTolerance(double tolerance) {
    m_interferenceTolerance = tolerance;
    m_interferenceChecker->setInterferenceTolerance(tolerance);
}

bool MultiAxisEngine::validateConfiguration() const {
    if (m_workpiece.IsNull()) {
        m_lastError = "Workpiece not set";
        return false;
    }

    if (m_config.configuration == AxisConfiguration::Axis5) {
        if (!m_config.outputA && !m_config.outputB) {
            m_lastError = "5-axis requires A or B axis output";
            return false;
        }
    }

    return true;
}

MultiAxisResult MultiAxisEngine::createResult() {
    MultiAxisResult result;
    result.success = false;
    result.hasInterferences = false;
    result.riskLevel = 0.0;
    result.totalPoints = 0;
    result.positioningPoints = 0;
    result.interpolatedPoints = 0;
    result.totalPathLength = 0.0;
    result.estimatedTime = 0.0;
    return result;
}

}
