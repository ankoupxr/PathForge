#include "PathStrategy.h"
#include <cmath>
#include <algorithm>

namespace PathForge::Path {

ContourPathStrategy::ContourPathStrategy() = default;

void ContourPathStrategy::setBoundary(const std::vector<gp_Pnt>& boundary) {
    m_boundary = boundary;
    m_boundarySet = !boundary.empty();
}

void ContourPathStrategy::setOffset(double offset) {
    m_offset = offset;
}

void ContourPathStrategy::setStartPoint(const gp_Pnt& point) {
    m_startPoint = point;
}

void ContourPathStrategy::setDirection(bool clockwise) {
    m_clockwise = clockwise;
}

ZigzagStrategy::ZigzagStrategy() {
    m_offset = 0.0;
}

ZigzagStrategy::ZigzagStrategy(double angle) : m_angle(angle) {}

void ZigzagStrategy::setAngle(double degrees) {
    m_angle = degrees;
}

double ZigzagStrategy::angle() const {
    return m_angle;
}

void ZigzagStrategy::setEntryType(const std::string& type) {
    m_entryType = type;
}

const std::string& ZigzagStrategy::entryType() const {
    return m_entryType;
}

std::string ZigzagStrategy::strategyName() const {
    return "Zigzag";
}

gp_Pnt ZigzagStrategy::calculateEntryPoint(double z) {
    if (!m_boundary.empty()) {
        return m_boundary.front();
    }
    return gp_Pnt(0, 0, z);
}

ToolpathPtr ZigzagStrategy::generate(
    const std::vector<gp_Pnt>& boundary,
    const MachiningParameters& params
) {
    auto path = std::make_shared<Toolpath>("Zigzag");
    setBoundary(boundary);
    generateZigzag(path, params);
    return path;
}

void ZigzagStrategy::generateZigzag(ToolpathPtr path, const MachiningParameters& params) {
    if (m_boundary.size() < 3) return;

    double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
    for (const auto& p : m_boundary) {
        minX = std::min(minX, p.X());
        maxX = std::max(maxX, p.X());
        minY = std::min(minY, p.Y());
        maxY = std::max(maxY, p.Y());
    }

    double width = maxX - minX;
    double height = maxY - minY;
    double stepover = params.toolDiameter() * params.stepover();
    double depth = params.depth();
    double stepdown = params.stepdown();
    double feedrate = params.feedrate();
    double safeZ = params.safeZ();
    double clearance = params.clearance();

    double angleRad = m_angle * M_PI / 180.0;
    double cosA = std::cos(angleRad);
    double sinA = std::sin(angleRad);

    for (double z = 0; z > -depth; z += stepdown) {
        bool forward = true;
        
        for (double t = 0; t <= height + stepover; t += stepover) {
            double yPos = minY + t;
            if (yPos > maxY) yPos = maxY;

            double yLocal = forward ? yPos : (height - (yPos - minY)) + minY;
            double x1 = minX;
            double x2 = maxX;

            if (!forward) {
                std::swap(x1, x2);
            }

            double xRotated1 = (x1 - minX) * cosA - (yLocal - minY) * sinA + minX;
            double yRotated1 = (x1 - minX) * sinA + (yLocal - minY) * cosA + minY;
            double xRotated2 = (x2 - minX) * cosA - (yLocal - minY) * sinA + minX;
            double yRotated2 = (x2 - minX) * sinA + (yLocal - minY) * cosA + minY;

            if (t == 0) {
                PathPoint rapidPt(gp_Pnt(xRotated1, yRotated1, safeZ), MoveType::Rapid);
                rapidPt.motionType = MotionType::Lift;
                path->addPoint(rapidPt);

                if (m_entryType == "helix") {
                    for (int i = 0; i <= 36; ++i) {
                        double angle = M_PI * 2 * i / 36;
                        double r = params.toolDiameter() / 2.0;
                        double h = clearance - clearance * i / 36;
                        PathPoint helixPt(gp_Pnt(xRotated1 + r * std::cos(angle), 
                                                 yRotated1 + r * std::sin(angle), 
                                                 clearance - h), MoveType::Helix);
                        helixPt.motionType = MotionType::Plunge;
                        helixPt.feedrate = params.plungeFeedrate();
                        path->addPoint(helixPt);
                    }
                } else {
                    PathPoint plungePt(gp_Pnt(xRotated1, yRotated1, clearance), MoveType::Linear);
                    plungePt.motionType = MotionType::Plunge;
                    plungePt.feedrate = params.plungeFeedrate();
                    path->addPoint(plungePt);
                }
            } else {
                PathPoint rapidPt(gp_Pnt(xRotated1, yRotated1, safeZ), MoveType::Rapid);
                rapidPt.motionType = MotionType::Lift;
                path->addPoint(rapidPt);

                PathPoint rapidPt2(gp_Pnt(xRotated1, yRotated1, clearance), MoveType::Rapid);
                rapidPt2.motionType = MotionType::Plunge;
                path->addPoint(rapidPt2);
            }

            PathPoint cutPt(gp_Pnt(xRotated2, yRotated2, z), MoveType::Linear);
            cutPt.motionType = MotionType::Cutting;
            cutPt.feedrate = feedrate;
            path->addPoint(cutPt);

            forward = !forward;
        }
    }
}

SpiralStrategy::SpiralStrategy() = default;

void SpiralStrategy::setSpiralType(const std::string& type) {
    m_spiralType = type;
}

const std::string& SpiralStrategy::spiralType() const {
    return m_spiralType;
}

void SpiralStrategy::setOverlap(double overlap) {
    m_overlap = overlap;
}

double SpiralStrategy::overlap() const {
    return m_overlap;
}

std::string SpiralStrategy::strategyName() const {
    return "Spiral";
}

ToolpathPtr SpiralStrategy::generate(
    const std::vector<gp_Pnt>& boundary,
    const MachiningParameters& params
) {
    auto path = std::make_shared<Toolpath>("Spiral");
    setBoundary(boundary);

    if (m_spiralType == "offset") {
        generateOffsetSpiral(path, params);
    } else {
        generateClassicSpiral(path, params);
    }
    return path;
}

void SpiralStrategy::generateClassicSpiral(ToolpathPtr path, const MachiningParameters& params) {
    if (m_boundary.empty()) return;

    double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
    for (const auto& p : m_boundary) {
        minX = std::min(minX, p.X());
        maxX = std::max(maxX, p.X());
        minY = std::min(minY, p.Y());
        maxY = std::max(maxY, p.Y());
    }

    double centerX = (minX + maxX) / 2.0;
    double centerY = (minY + maxY) / 2.0;
    double maxRadius = std::min(maxX - minX, maxY - minY) / 2.0;
    double toolRadius = params.toolDiameter() / 2.0;
    double depth = params.depth();
    double stepdown = params.stepdown();
    double feedrate = params.feedrate();
    double safeZ = params.safeZ();
    double clearance = params.clearance();
    double stepover = params.toolDiameter() * params.stepover();

    double currentRadius = maxRadius - toolRadius + m_overlap * toolRadius;

    for (double z = 0; z > -depth; z += stepdown) {
        if (currentRadius < toolRadius) currentRadius = toolRadius;

        int segments = static_cast<int>(currentRadius * 20);
        
        for (int i = 0; i <= segments; ++i) {
            double angle = 2.0 * M_PI * i / segments;
            double x = centerX + currentRadius * std::cos(angle);
            double y = centerY + currentRadius * std::sin(angle);

            if (i == 0 && z == 0) {
                PathPoint rapidPt(gp_Pnt(x, y, safeZ), MoveType::Rapid);
                rapidPt.motionType = MotionType::Lift;
                path->addPoint(rapidPt);

                PathPoint plungePt(gp_Pnt(x, y, clearance), MoveType::Linear);
                plungePt.motionType = MotionType::Plunge;
                plungePt.feedrate = params.plungeFeedrate();
                path->addPoint(plungePt);
            }

            PathPoint cutPt(gp_Pnt(x, y, z), MoveType::Linear);
            cutPt.motionType = MotionType::Cutting;
            cutPt.feedrate = feedrate;
            path->addPoint(cutPt);
        }

        currentRadius -= stepover;
    }
}

void SpiralStrategy::generateOffsetSpiral(ToolpathPtr path, const MachiningParameters& params) {
    generateClassicSpiral(path, params);
}

OffsetStrategy::OffsetStrategy() = default;

void OffsetStrategy::setCompensationType(const std::string& type) {
    m_compensationType = type;
}

const std::string& OffsetStrategy::compensationType() const {
    return m_compensationType;
}

void OffsetStrategy::setCornerStyle(const std::string& style) {
    m_cornerStyle = style;
}

const std::string& OffsetStrategy::cornerStyle() const {
    return m_cornerStyle;
}

std::string OffsetStrategy::strategyName() const {
    return "Offset";
}

ToolpathPtr OffsetStrategy::generate(
    const std::vector<gp_Pnt>& boundary,
    const MachiningParameters& params
) {
    auto path = std::make_shared<Toolpath>("Offset");
    setBoundary(boundary);

    double depth = params.depth();
    double stepdown = params.stepdown();
    double feedrate = params.feedrate();
    double safeZ = params.safeZ();
    double clearance = params.clearance();
    double toolRadius = params.toolDiameter() / 2.0;

    double currentOffset = toolRadius;
    int pass = 0;

    while (currentOffset < toolRadius * 10) {
        auto offsetResult = offsetBoundary(currentOffset);
        
        if (offsetResult.size() < 3) break;

        for (double z = 0; z > -depth; z += stepdown) {
            for (size_t i = 0; i <= offsetResult.size(); ++i) {
                const auto& p = offsetResult[i % offsetResult.size()];

                if (i == 0 && z == 0 && pass == 0) {
                    PathPoint rapidPt(gp_Pnt(p.X(), p.Y(), safeZ), MoveType::Rapid);
                    rapidPt.motionType = MotionType::Lift;
                    path->addPoint(rapidPt);

                    PathPoint plungePt(gp_Pnt(p.X(), p.Y(), clearance), MoveType::Linear);
                    plungePt.motionType = MotionType::Plunge;
                    plungePt.feedrate = params.plungeFeedrate();
                    path->addPoint(plungePt);
                }

                PathPoint cutPt(gp_Pnt(p.X(), p.Y(), z), MoveType::Linear);
                cutPt.motionType = MotionType::Cutting;
                cutPt.feedrate = feedrate;
                path->addPoint(cutPt);
            }
        }

        currentOffset += params.toolDiameter() * params.stepover();
        pass++;
    }

    return path;
}

std::vector<gp_Pnt> OffsetStrategy::offsetBoundary(double distance) {
    if (m_boundary.size() < 3) return {};

    std::vector<gp_Pnt> result;
    size_t n = m_boundary.size();

    for (size_t i = 0; i < n; ++i) {
        const gp_Pnt& p0 = m_boundary[(i + n - 1) % n];
        const gp_Pnt& p1 = m_boundary[i];
        const gp_Pnt& p2 = m_boundary[(i + 1) % n];

        gp_Vec v1(p1.X() - p0.X(), p1.Y() - p0.Y(), 0);
        gp_Vec v2(p2.X() - p1.X(), p2.Y() - p1.Y(), 0);
        
        v1.Normalize();
        v2.Normalize();

        gp_Vec normal(-v1.Y(), v1.X(), 0);
        
        if (!m_clockwise) {
            normal.Reverse();
        }

        gp_Pnt offsetP(
            p1.X() + normal.X() * distance,
            p1.Y() + normal.Y() * distance,
            p1.Z()
        );

        result.push_back(offsetP);
    }

    return result;
}

gp_Pnt OffsetStrategy::calculateCorner(
    const gp_Pnt& p1, 
    const gp_Pnt& p2, 
    const gp_Pnt& p3, 
    double offset
) {
    return p2;
}

AdaptiveStrategy::AdaptiveStrategy() = default;

void AdaptiveStrategy::setMaxStepover(double stepover) {
    m_maxStepover = stepover;
}

double AdaptiveStrategy::maxStepover() const {
    return m_maxStepover;
}

void AdaptiveStrategy::setMinimumRadius(double radius) {
    m_minimumRadius = radius;
}

double AdaptiveStrategy::minimumRadius() const {
    return m_minimumRadius;
}

void AdaptiveStrategy::setSmoothFactor(double factor) {
    m_smoothFactor = factor;
}

double AdaptiveStrategy::smoothFactor() const {
    return m_smoothFactor;
}

std::string AdaptiveStrategy::strategyName() const {
    return "Adaptive";
}

ToolpathPtr AdaptiveStrategy::generate(
    const std::vector<gp_Pnt>& boundary,
    const MachiningParameters& params
) {
    auto path = std::make_shared<Toolpath>("Adaptive");
    setBoundary(boundary);

    double stepover = m_maxStepover > 0 ? m_maxStepover : params.toolDiameter() * params.stepover();
    
    SpiralStrategy spiral;
    spiral.setSpiralType("classic");
    auto spiralPath = spiral.generate(boundary, params);
    
    for (const auto& pt : spiralPath->points()) {
        path->addPoint(pt);
    }

    optimizeStepover(path, params);

    return path;
}

void AdaptiveStrategy::optimizeStepover(ToolpathPtr path, const MachiningParameters& params) {
    if (path->pointCount() < 3) return;

    const auto& points = path->points();
    double threshold = m_minimumRadius;

    for (size_t i = 1; i + 1 < points.size(); ++i) {
        double curvature = calculateCurvature(
            points[i - 1].position,
            points[i].position,
            points[i + 1].position
        );

        if (curvature > 1.0 / threshold) {
            double adjustedStepover = params.toolDiameter() * params.stepover() * m_smoothFactor;
            (void)adjustedStepover;
        }
    }
}

double AdaptiveStrategy::calculateCurvature(
    const gp_Pnt& p1, 
    const gp_Pnt& p2, 
    const gp_Pnt& p3
) {
    gp_Vec v1(p2.X() - p1.X(), p2.Y() - p1.Y(), p2.Z() - p1.Z());
    gp_Vec v2(p3.X() - p2.X(), p3.Y() - p2.Y(), p3.Z() - p2.Z());
    
    double len1 = v1.Magnitude();
    double len2 = v2.Magnitude();
    
    if (len1 < 1e-9 || len2 < 1e-9) return 0.0;
    
    v1.Normalize();
    v2.Normalize();
    
    gp_Vec cross = v1.Crossed(v2);
    double crossLen = cross.Magnitude();
    
    if (crossLen < 1e-9) return 0.0;
    
    double angle = std::asin(crossLen);
    double chord = (p1.Distance(p3)) / 2.0;
    
    if (chord < 1e-9) return 0.0;
    
    return angle / chord;
}

}
