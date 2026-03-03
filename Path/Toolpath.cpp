#include "Toolpath.h"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <TopoDS_Wire.hxx>

#include <cmath>

namespace PathForge::Path {

Toolpath::Toolpath() = default;

Toolpath::Toolpath(const std::string& name) : m_name(name) {}

void Toolpath::addPoint(const PathPoint& point) {
    m_points.push_back(point);
    if (m_points.size() == 1) {
        m_startPoint = point.position;
    }
    m_endPoint = point.position;
}

void Toolpath::addPoints(const std::vector<PathPoint>& points) {
    for (const auto& p : points) {
        addPoint(p);
    }
}

const std::vector<PathPoint>& Toolpath::points() const { return m_points; }
std::vector<PathPoint>& Toolpath::points() { return m_points; }

const std::string& Toolpath::name() const { return m_name; }
void Toolpath::setName(const std::string& name) { m_name = name; }

double Toolpath::totalLength() const {
    if (m_points.size() < 2) return 0.0;
    double length = 0.0;
    for (size_t i = 1; i < m_points.size(); ++i) {
        length += m_points[i].position.Distance(m_points[i - 1].position);
    }
    return length;
}

double Toolpath::cuttingLength() const {
    if (m_points.size() < 2) return 0.0;
    double length = 0.0;
    for (size_t i = 1; i < m_points.size(); ++i) {
        if (m_points[i].motionType == MotionType::Cutting) {
            length += m_points[i].position.Distance(m_points[i - 1].position);
        }
    }
    return length;
}

double Toolpath::duration() const {
    if (m_points.empty()) return 0.0;
    double time = 0.0;
    for (size_t i = 1; i < m_points.size(); ++i) {
        const auto& p0 = m_points[i - 1];
        const auto& p1 = m_points[i];
        if (p1.feedrate > 0) {
            double dist = p1.position.Distance(p0.position);
            time += dist / p1.feedrate * 60.0;
        }
    }
    return time;
}

void Toolpath::setDepth(double depth) { m_depth = depth; }
double Toolpath::depth() const { return m_depth; }

void Toolpath::setStartPoint(const gp_Pnt& p) { m_startPoint = p; }
gp_Pnt Toolpath::startPoint() const { return m_startPoint; }

void Toolpath::setEndPoint(const gp_Pnt& p) { m_endPoint = p; }
gp_Pnt Toolpath::endPoint() const { return m_endPoint; }

void Toolpath::reverse() {
    std::reverse(m_points.begin(), m_points.end());
    for (auto& p : m_points) {
        if (p.motionType == MotionType::LeadIn) p.motionType = MotionType::LeadOut;
        else if (p.motionType == MotionType::LeadOut) p.motionType = MotionType::LeadIn;
    }
}

void Toolpath::clear() {
    m_points.clear();
    m_startPoint = gp_Pnt(0, 0, 0);
    m_endPoint = gp_Pnt(0, 0, 0);
}

bool Toolpath::isEmpty() const { return m_points.empty(); }
size_t Toolpath::pointCount() const { return m_points.size(); }

}
