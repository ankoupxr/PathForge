// CamEdge.h
#pragma once

#include <vector>
#include "CamPoint.h"

namespace PathForge {

enum class CamEdgeType {
    Line,
    Arc,
    Circle,
    Bezier,
    BSpline
};

struct CamEdge {
    std::vector<CamPoint> points;
    CamEdgeType type = CamEdgeType::Line;
    bool isReversed = false;

    CamEdge() = default;

    explicit CamEdge(const std::vector<CamPoint>& pts, CamEdgeType t = CamEdgeType::Line)
        : points(pts), type(t) {}

    explicit CamEdge(CamPoint p1, CamPoint p2)
        : points({p1, p2}), type(CamEdgeType::Line) {}

    size_t pointCount() const { return points.size(); }

    CamPoint firstPoint() const {
        return isReversed ? points.back() : points.front();
    }

    CamPoint lastPoint() const {
        return isReversed ? points.front() : points.back();
    }

    double length() const {
        double len = 0.0;
        for (size_t i = 1; i < points.size(); ++i) {
            const CamPoint& p1 = isReversed ? points[points.size() - i] : points[i - 1];
            const CamPoint& p2 = isReversed ? points[points.size() - i - 1] : points[i];
            len += p1.distanceTo(p2);
        }
        return len;
    }

    CamEdge reversed() const {
        CamEdge rev;
        rev.points = points;
        std::reverse(rev.points.begin(), rev.points.end());
        rev.type = type;
        rev.isReversed = !isReversed;
        return rev;
    }

    CamPoint midpoint() const {
        if (points.empty()) return CamPoint::origin();
        if (points.size() == 1) return points.front();
        size_t mid = points.size() / 2;
        return points[mid];
    }
};

} // namespace PathForge