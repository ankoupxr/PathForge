#pragma once

#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>

#include <vector>

namespace PathForge::MultiAxis {

struct MultiAxisPoint {
    gp_Pnt position;
    gp_Dir toolAxis;
    gp_Dir surfaceNormal;
    double feedrate;
    double spindleSpeed;

    double axisA;
    double axisB;
    double axisC;

    int pointIndex;
    bool isKeyPoint;

    MultiAxisPoint();
    MultiAxisPoint(const gp_Pnt& pos);

    void setAxisAngles(double a, double b, double c = 0.0);
    void setTiltedAxis(double a, double b);

    bool hasTiltedAxis() const;
    bool hasRotatedAxis() const;
};

using MultiAxisPointList = std::vector<MultiAxisPoint>;

class MultiAxisPath {
public:
    MultiAxisPath() = default;

    void addPoint(const MultiAxisPoint& point);
    void addPoints(const MultiAxisPointList& points);

    const MultiAxisPointList& getPoints() const { return m_points; }
    MultiAxisPointList& getPoints() { return m_points; }

    size_t pointCount() const { return m_points.size(); }
    bool isEmpty() const { return m_points.empty(); }

    void clear();
    void reverse();

    double totalLength() const;
    double estimatedTime() const;

    const MultiAxisPoint& firstPoint() const;
    const MultiAxisPoint& lastPoint() const;

private:
    MultiAxisPointList m_points;
};

}
