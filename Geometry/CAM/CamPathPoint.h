// CamPathPoint.h
#pragma once

#include "CamPoint.h"
#include "CamDirection.h"

namespace PathForge {

enum class CamMoveType {
    Rapid,
    Linear,
    ArcCW,
    ArcCCW,
    Helix,
    Dwell
};

enum class CamMotionType {
    Cutting,
    LeadIn,
    LeadOut,
    Lift,
    Plunge,
    Rapid
};

struct CamPathPoint {
    CamPoint position;
    CamDirection normal;
    double feedrate = 0.0;
    double spindleSpeed = 0.0;
    int toolIndex = 0;
    CamMoveType moveType = CamMoveType::Linear;
    CamMotionType motionType = CamMotionType::Cutting;

    CamPathPoint() = default;

    explicit CamPathPoint(const CamPoint& p, CamMoveType mt = CamMoveType::Linear)
        : position(p), moveType(mt) {}
};

class CamToolpath {
public:
    CamToolpath() = default;
    explicit CamToolpath(const std::string& name) : m_name(name) {}

    void addPoint(const CamPathPoint& point) {
        m_points.push_back(point);
    }

    void addPoints(const std::vector<CamPathPoint>& points) {
        m_points.insert(m_points.end(), points.begin(), points.end());
    }

    const std::vector<CamPathPoint>& points() const { return m_points; }
    std::vector<CamPathPoint>& points() { return m_points; }

    const std::string& name() const { return m_name; }
    void setName(const std::string& name) { m_name = name; }

    double totalLength() const {
        if (m_points.size() < 2) return 0.0;
        double len = 0.0;
        for (size_t i = 1; i < m_points.size(); ++i) {
            len += m_points[i].position.distanceTo(m_points[i - 1].position);
        }
        return len;
    }

    double cuttingLength() const {
        double len = 0.0;
        for (size_t i = 1; i < m_points.size(); ++i) {
            if (m_points[i].motionType == CamMotionType::Cutting) {
                len += m_points[i].position.distanceTo(m_points[i - 1].position);
            }
        }
        return len;
    }

    void setDepth(double depth) { m_depth = depth; }
    double depth() const { return m_depth; }

    void setStartPoint(const CamPoint& p) { m_startPoint = p; }
    CamPoint startPoint() const { return m_startPoint; }

    void setEndPoint(const CamPoint& p) { m_endPoint = p; }
    CamPoint endPoint() const { return m_endPoint; }

    void reverse() {
        std::reverse(m_points.begin(), m_points.end());
    }

    void clear() {
        m_points.clear();
    }

    bool isEmpty() const { return m_points.empty(); }
    size_t pointCount() const { return m_points.size(); }

private:
    std::string m_name;
    std::vector<CamPathPoint> m_points;
    double m_depth = 0.0;
    CamPoint m_startPoint;
    CamPoint m_endPoint;
};

using CamToolpathPtr = std::shared_ptr<CamToolpath>;
using CamToolpathList = std::vector<CamToolpathPtr>;

} // namespace PathForge