#include "MultiAxisPoint.h"

#include <cmath>

namespace PathForge::MultiAxis {

MultiAxisPoint::MultiAxisPoint() {
    feedrate = 0.0;
    spindleSpeed = 0.0;
    axisA = 0.0;
    axisB = 0.0;
    axisC = 0.0;
    pointIndex = 0;
    isKeyPoint = false;
}

MultiAxisPoint::MultiAxisPoint(const gp_Pnt& pos) : position(pos) {
    toolAxis = gp_Dir(0, 0, 1);
    surfaceNormal = gp_Dir(0, 0, 1);
    feedrate = 0.0;
    spindleSpeed = 0.0;
    axisA = 0.0;
    axisB = 0.0;
    axisC = 0.0;
    pointIndex = 0;
    isKeyPoint = false;
}

void MultiAxisPoint::setAxisAngles(double a, double b, double c) {
    axisA = a;
    axisB = b;
    axisC = c;
}

void MultiAxisPoint::setTiltedAxis(double a, double b) {
    setAxisAngles(a, b, 0.0);
}

bool MultiAxisPoint::hasTiltedAxis() const {
    return std::abs(axisA) > 0.01 || std::abs(axisB) > 0.01;
}

bool MultiAxisPoint::hasRotatedAxis() const {
    return std::abs(axisC) > 0.01;
}

void MultiAxisPath::addPoint(const MultiAxisPoint& point) {
    m_points.push_back(point);
}

void MultiAxisPath::addPoints(const MultiAxisPointList& points) {
    m_points.insert(m_points.end(), points.begin(), points.end());
}

void MultiAxisPath::clear() {
    m_points.clear();
}

void MultiAxisPath::reverse() {
    std::reverse(m_points.begin(), m_points.end());
}

double MultiAxisPath::totalLength() const {
    if (m_points.size() < 2) return 0.0;

    double length = 0.0;
    for (size_t i = 1; i < m_points.size(); ++i) {
        length += m_points[i].position.Distance(m_points[i-1].position);
    }
    return length;
}

double MultiAxisPath::estimatedTime() const {
    if (m_points.empty()) return 0.0;

    double time = 0.0;
    for (size_t i = 1; i < m_points.size(); ++i) {
        const auto& p0 = m_points[i-1];
        const auto& p1 = m_points[i];

        if (p1.feedrate > 0) {
            double dist = p1.position.Distance(p0.position);
            time += dist / p1.feedrate * 60.0;
        }
    }
    return time;
}

const MultiAxisPoint& MultiAxisPath::firstPoint() const {
    return m_points.front();
}

const MultiAxisPoint& MultiAxisPath::lastPoint() const {
    return m_points.back();
}

}
