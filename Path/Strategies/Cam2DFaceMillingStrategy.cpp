// Cam2DFaceMillingStrategy.cpp
#include "Cam2DFaceMillingStrategy.h"
#include "CamFace.h"
#include "CamWire.h"
#include <cmath>
#include <algorithm>

namespace PathForge {
namespace CAM {

Cam2DFaceMillingStrategy::Cam2DFaceMillingStrategy()
    : CamPathStrategy() {}

Cam2DFaceMillingStrategy::Cam2DFaceMillingStrategy(CamFacePtr face)
    : CamPathStrategy(face) {}

CamStrategyType Cam2DFaceMillingStrategy::getType() const {
    return CamStrategyType::FaceMilling2D;
}

std::string Cam2DFaceMillingStrategy::getName() const {
    return "2D Face Milling";
}

std::string Cam2DFaceMillingStrategy::getDescription() const {
    return "2D face milling strategy with zigzag or one-direction patterns";
}

bool Cam2DFaceMillingStrategy::validate() const {
    if (!m_face) {
        m_lastError = "Face is not set";
        return false;
    }
    if (m_stepover <= 0) {
        m_lastError = "Stepover must be positive";
        return false;
    }
    return true;
}

CamToolpathPtr Cam2DFaceMillingStrategy::generate() {
    if (!validate()) {
        return nullptr;
    }

    auto toolpath = std::make_shared<CamToolpath>();
    toolpath->setName(getName());

    const auto& bbox = m_face->bbox;
    BoundingBox2D bbox2D;
    bbox2D.minX = bbox.min.x;
    bbox2D.maxX = bbox.max.x;
    bbox2D.minY = bbox.min.y;
    bbox2D.maxY = bbox.max.y;

    auto boundary = m_face->outerBoundary.getAllPoints();
    if (boundary.empty()) {
        m_lastError = "Face boundary is empty";
        return nullptr;
    }

    std::vector<CamPoint> pathPoints;

    if (m_cuttingDirection == CamCuttingDirection::Zigzag) {
        pathPoints = generateZigzagLines(bbox2D, m_stepover, m_cuttingAngle);
    } else {
        bool forward = (m_cuttingDirection == CamCuttingDirection::Forward);
        pathPoints = generateOneDirectionLines(bbox2D, m_stepover, m_cuttingAngle, forward);
    }

    std::vector<CamPoint> filteredPoints;
    for (const auto& p : pathPoints) {
        if (pointInPolygon(p, boundary)) {
            filteredPoints.push_back(p);
        }
    }

    double z = m_context.modelTop.z - m_context.depth;
    for (const auto& p : filteredPoints) {
        CamPathPoint pathPoint;
        pathPoint.position = CamPoint(p.x, p.y, z);
        pathPoint.moveType = CamMoveType::Linear;
        pathPoint.motionType = CamMotionType::Cutting;
        pathPoint.feedrate = m_context.feedrate;
        toolpath->addPoint(pathPoint);
    }

    toolpath->setDepth(m_context.depth);

    if (!filteredPoints.empty()) {
        toolpath->setStartPoint(filteredPoints.front());
        toolpath->setEndPoint(filteredPoints.back());
    }

    return toolpath;
}

Cam2DFaceMillingStrategy::BoundingBox2D
Cam2DFaceMillingStrategy::computeBoundingBox(const CamWire& wire) {
    BoundingBox2D bbox;
    auto points = wire.getAllPoints();

    if (points.empty()) {
        bbox.minX = bbox.minY = 0;
        bbox.maxX = bbox.maxY = 0;
        return bbox;
    }

    bbox.minX = bbox.maxX = points[0].x;
    bbox.minY = bbox.maxY = points[0].y;

    for (const auto& p : points) {
        bbox.minX = std::min(bbox.minX, p.x);
        bbox.maxX = std::max(bbox.maxX, p.x);
        bbox.minY = std::min(bbox.minY, p.y);
        bbox.maxY = std::max(bbox.maxY, p.y);
    }

    return bbox;
}

std::vector<CamPoint> Cam2DFaceMillingStrategy::generateZigzagLines(
    const BoundingBox2D& bbox,
    double stepover,
    double angle
) {
    std::vector<CamPoint> lines;

    double cosA = std::cos(angle * M_PI / 180.0);
    double sinA = std::sin(angle * M_PI / 180.0);

    double width = bbox.maxX - bbox.minX;
    double height = bbox.maxY - bbox.minY;

    double diagonal = std::sqrt(width * width + height * height);
    int numPasses = static_cast<int>(diagonal / stepover) + 1;

    double offsetMin = -diagonal / 2;
    double offsetMax = diagonal / 2;

    for (int i = 0; i <= numPasses; ++i) {
        double offset = offsetMin + i * stepover;

        CamPoint p1(bbox.minX + offset * cosA, bbox.minY + offset * sinA, 0);
        CamPoint p2(bbox.maxX + offset * cosA, bbox.maxY + offset * sinA, 0);

        lines.push_back(p1);
        lines.push_back(p2);
    }

    return lines;
}

std::vector<CamPoint> Cam2DFaceMillingStrategy::generateOneDirectionLines(
    const BoundingBox2D& bbox,
    double stepover,
    double angle,
    bool forward
) {
    std::vector<CamPoint> lines;

    double cosA = std::cos(angle * M_PI / 180.0);
    double sinA = std::sin(angle * M_PI / 180.0);

    double width = bbox.maxX - bbox.minX;
    double height = bbox.maxY - bbox.minY;

    double diagonal = std::sqrt(width * width + height * height);
    int numPasses = static_cast<int>(diagonal / stepover) + 1;

    double offsetMin = -diagonal / 2;
    double offsetMax = diagonal / 2;

    for (int i = 0; i <= numPasses; ++i) {
        double offset = offsetMin + i * stepover;

        CamPoint start(bbox.minX + offset * cosA, bbox.minY + offset * sinA, 0);
        CamPoint end(bbox.maxX + offset * cosA, bbox.maxY + offset * sinA, 0);

        if ((i % 2 == 0) != forward) {
            lines.push_back(end);
            lines.push_back(start);
        } else {
            lines.push_back(start);
            lines.push_back(end);
        }
    }

    return lines;
}

bool Cam2DFaceMillingStrategy::pointInPolygon(
    const CamPoint& p,
    const std::vector<CamPoint>& polygon
) {
    int n = static_cast<int>(polygon.size());
    if (n < 3) return false;

    bool inside = false;

    for (int i = 0, j = n - 1; i < n; j = i++) {
        const CamPoint& pi = polygon[i];
        const CamPoint& pj = polygon[j];

        if (((pi.y > p.y) != (pj.y > p.y)) &&
            (p.x < (pj.x - pi.x) * (p.y - pi.y) / (pj.y - pi.y) + pi.x)) {
            inside = !inside;
        }
    }

    return inside;
}

CamPoint Cam2DFaceMillingStrategy::findEntryPoint(
    const CamBoundingBox& bbox,
    double angle
) {
    CamPoint center = bbox.center();
    CamPoint entry = center - CamDirection(angle * M_PI / 180.0) * (bbox.width() / 2);
    return entry;
}

} // namespace CAM
} // namespace PathForge