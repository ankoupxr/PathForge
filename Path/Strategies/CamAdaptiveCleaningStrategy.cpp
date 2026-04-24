// CamAdaptiveCleaningStrategy.cpp
#include "CamAdaptiveCleaningStrategy.h"
#include "CamFace.h"
#include "CamWire.h"
#include "CamEdge.h"
#include <cmath>
#include <algorithm>

namespace PathForge {
namespace CAM {

CamAdaptiveCleaningStrategy::CamAdaptiveCleaningStrategy()
    : CamPathStrategy() {}

CamAdaptiveCleaningStrategy::CamAdaptiveCleaningStrategy(CamFacePtr face)
    : CamPathStrategy(face) {}

CamStrategyType CamAdaptiveCleaningStrategy::getType() const {
    return CamStrategyType::PocketMilling;
}

std::string CamAdaptiveCleaningStrategy::getName() const {
    return "Adaptive Cleaning";
}

std::string CamAdaptiveCleaningStrategy::getDescription() const {
    return "Adaptive offset-based cleaning strategy for pocket milling";
}

bool CamAdaptiveCleaningStrategy::validate() const {
    if (!m_face) {
        m_lastError = "Face is not set";
        return false;
    }
    if (m_initialStepover <= 0) {
        m_lastError = "Stepover must be positive";
        return false;
    }
    if (m_face->outerBoundary.edgeCount() == 0) {
        m_lastError = "Face has no boundary";
        return false;
    }
    return true;
}

CamToolpathPtr CamAdaptiveCleaningStrategy::generate() {
    if (!validate()) {
        return nullptr;
    }

    auto toolpath = std::make_shared<CamToolpath>();
    toolpath->setName(getName());

    const CamWire& boundary = m_face->outerBoundary;
    double depth = m_context.depth;

    m_offsetLayers = performOffset(boundary);

    std::vector<CamLinkSegment> allLinks;

    for (size_t i = 0; i < m_offsetLayers.size(); ++i) {
        const auto& layer = m_offsetLayers[i];

        for (size_t j = 0; j < layer.discretizedPoints.size(); ++j) {
            CamPathPoint point;
            point.position = layer.discretizedPoints[j];
            point.position.z = m_context.modelTop.z - depth;
            point.moveType = CamMoveType::Linear;
            point.motionType = CamMotionType::Cutting;
            point.feedrate = m_context.feedrate;
            toolpath->addPoint(point);
        }

        if (i < m_offsetLayers.size() - 1) {
            CamLinkSegment link = findShortestLink(
                m_offsetLayers[i].discretizedPoints,
                m_offsetLayers[i + 1].discretizedPoints
            );
            if (link.distance > 0) {
                allLinks.push_back(link);
            }
        }
    }

    addLinkSegments(toolpath, allLinks);

    toolpath->setDepth(depth);

    return toolpath;
}

std::vector<CamOffsetLayer> CamAdaptiveCleaningStrategy::performOffset(const CamWire& boundary) {
    std::vector<CamOffsetLayer> layers;

    double currentOffset = m_toolRadius;
    int depth = 0;

    while (currentOffset < 2.0 * m_toolRadius && depth < m_maxRecursionDepth) {
        CamOffsetLayer layer;
        layer.offsetDistance = currentOffset;
        layer.discretizedPoints = discretizeWire(boundary, m_chordLength);

        if (layer.discretizedPoints.empty()) {
            break;
        }

        layers.push_back(layer);

        currentOffset += m_initialStepover;
        depth++;
    }

    return layers;
}

std::vector<CamPoint> CamAdaptiveCleaningStrategy::discretizeWire(
    const CamWire& wire,
    double chordLength
) {
    std::vector<CamPoint> points;

    for (const auto& edge : wire.edges) {
        if (edge.points.size() < 2) {
            if (!edge.points.empty()) {
                points.push_back(edge.points.front());
            }
            continue;
        }

        double edgeLen = edge.length();
        int numSamples = std::max(2, static_cast<int>(edgeLen / chordLength) + 1);

        for (int i = 0; i <= numSamples; ++i) {
            double t = static_cast<double>(i) / numSamples;
            const auto& pts = edge.points;

            size_t idx1 = edge.isReversed ? (pts.size() - 1 - i) : i;
            if (edge.isReversed && i > 0) {
                idx1 = pts.size() - 1 - std::min(i, pts.size() - 1);
            }

            if (idx1 < pts.size()) {
                points.push_back(pts[idx1]);
            } else if (!edge.isReversed && i >= pts.size()) {
                points.push_back(pts.back());
            } else if (edge.isReversed && pts.size() - 1 - i < 0) {
                points.push_back(pts.front());
            }
        }
    }

    return points;
}

CamLinkSegment CamAdaptiveCleaningStrategy::findShortestLink(
    const std::vector<CamPoint>& pts1,
    const std::vector<CamPoint>& pts2
) {
    CamLinkSegment link;
    link.distance = 1e100;

    if (pts1.empty() || pts2.empty()) {
        return link;
    }

    for (const auto& p1 : pts1) {
        for (const auto& p2 : pts2) {
            double dist = p1.distanceTo(p2);
            if (dist < link.distance) {
                link.distance = dist;
                link.startPoint = p1;
                link.endPoint = p2;
            }
        }
    }

    return link;
}

std::tuple<double, double, double, double>
CamAdaptiveCleaningStrategy::computeBBox2D(const std::vector<CamPoint>& points) {
    if (points.empty()) {
        return {0, 0, 0, 0};
    }

    double minX = points[0].x, maxX = points[0].x;
    double minY = points[0].y, maxY = points[0].y;

    for (const auto& p : points) {
        minX = std::min(minX, p.x);
        maxX = std::max(maxX, p.x);
        minY = std::min(minY, p.y);
        maxY = std::max(maxY, p.y);
    }

    return {minX, minY, maxX, maxY};
}

CamPoint CamAdaptiveCleaningStrategy::wireCentroid(const CamWire& wire) {
    auto points = wire.getAllPoints();
    if (points.empty()) {
        return CamPoint::origin();
    }

    double cx = 0, cy = 0, cz = 0;
    for (const auto& p : points) {
        cx += p.x;
        cy += p.y;
        cz += p.z;
    }

    size_t n = points.size();
    return CamPoint(cx / n, cy / n, cz / n);
}

double CamAdaptiveCleaningStrategy::wireArea(const CamWire& wire) {
    auto points = wire.getAllPoints();
    if (points.size() < 3) {
        return 0.0;
    }

    double area = 0.0;
    size_t n = points.size();

    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        area += points[i].x * points[j].y;
        area -= points[j].x * points[i].y;
    }

    return std::abs(area) / 2.0;
}

CamToolpathPtr CamAdaptiveCleaningStrategy::generateSpiralPath(
    const CamWire& boundary,
    double depth
) {
    auto toolpath = std::make_shared<CamToolpath>();
    toolpath->setName("Spiral Path");

    return toolpath;
}

void CamAdaptiveCleaningStrategy::addLinkSegments(
    CamToolpathPtr path,
    const std::vector<CamLinkSegment>& links
) {
    for (const auto& link : links) {
        CamPathPoint liftPoint;
        liftPoint.position = link.startPoint;
        liftPoint.position.z = m_context.safeZ;
        liftPoint.moveType = CamMoveType::Rapid;
        liftPoint.motionType = CamMotionType::Lift;
        path->addPoint(liftPoint);

        CamPathPoint plungePoint;
        plungePoint.position = link.endPoint;
        plungePoint.position.z = m_context.modelTop.z - depth;
        plungePoint.moveType = CamMoveType::Linear;
        plungePoint.motionType = CamMotionType::Plunge;
        plungePoint.feedrate = m_context.plungeFeedrate;
        path->addPoint(plungePoint);
    }
}

} // namespace CAM
} // namespace PathForge