// CamFace.h
#pragma once

#include <vector>
#include <memory>
#include <string>
#include <functional>
#include "CamWire.h"
#include "CamPoint.h"
#include "CamPoint2D.h"
#include "CamDirection.h"

namespace PathForge {

enum class CamSurfaceType {
    Plane,
    Cylinder,
    Cone,
    Sphere,
    Bezier,
    BSpline,
    Unknown
};

struct CamUVRange {
    CamPoint2D umin;
    CamPoint2D umax;

    CamUVRange() : umin(0, 0), umax(1, 1) {}

    CamUVRange(double u1, double v1, double u2, double v2)
        : umin(u1, v1), umax(u2, v2) {}

    double uSpan() const { return umax.u - umin.u; }
    double vSpan() const { return umax.v - umin.v; }
    double uCenter() const { return (umin.u + umax.u) * 0.5; }
    double vCenter() const { return (umin.v + umax.v) * 0.5; }
};

struct CamBoundingBox {
    CamPoint min;
    CamPoint max;

    CamBoundingBox() : min(0, 0, 0), max(0, 0, 0) {}

    CamBoundingBox(const CamPoint& min_, const CamPoint& max_)
        : min(min_), max(max_) {}

    double width() const { return max.x - min.x; }
    double height() const { return max.y - min.y; }
    double depth() const { return max.z - min.z; }

    CamPoint center() const {
        return CamPoint(
            (min.x + max.x) * 0.5,
            (min.y + max.y) * 0.5,
            (min.z + max.z) * 0.5
        );
    }

    bool contains(const CamPoint& p) const {
        return p.x >= min.x && p.x <= max.x &&
               p.y >= min.y && p.y <= max.y &&
               p.z >= min.z && p.z <= max.z;
    }

    bool intersects(const CamBoundingBox& other) const {
        return !(max.x < other.min.x || min.x > other.max.x ||
                 max.y < other.min.y || min.y > other.max.y ||
                 max.z < other.min.z || min.z > other.max.z);
    }
};

class CamFace {
public:
    int id = -1;
    std::string name;

    CamSurfaceType surfaceType = CamSurfaceType::Plane;
    CamDirection normal;
    CamUVRange uvRange;

    std::function<CamPoint(double u, double v)> surfaceEvaluator;
    std::function<CamDirection(double u, double v)> normalEvaluator;

    CamWire outerBoundary;
    std::vector<CamWire> holeBoundaries;

    CamBoundingBox bbox;

    double area = 0.0;
    CamPoint centroid;

    bool hasSurfaceEvaluator() const { return static_cast<bool>(surfaceEvaluator); }
    bool hasNormalEvaluator() const { return static_cast<bool>(normalEvaluator); }

    std::vector<CamPoint> sampleGridPoints(size_t uSamples, size_t vSamples) const;

    std::vector<CamPoint> getAllBoundaryPoints() const {
        std::vector<CamPoint> result = outerBoundary.getAllPoints();
        for (const auto& hole : holeBoundaries) {
            auto holePoints = hole.getAllPoints();
            result.insert(result.end(), holePoints.begin(), holePoints.end());
        }
        return result;
    }

    bool hasHoles() const { return !holeBoundaries.empty(); }
    size_t holeCount() const { return holeBoundaries.size(); }

    CamDirection getNormalAt(const CamPoint& point) const {
        if (normalEvaluator) {
            CamPoint2D uv = projectToUV(point);
            return normalEvaluator(uv.u, uv.v);
        }
        return normal;
    }

    CamPoint evaluate(double u, double v) const;

    CamPoint2D projectToUV(const CamPoint& point3D) const;

    bool isPointInside(const CamPoint& point) const;

    double getZRange(std::pair<double, double>& range) const {
        if (surfaceType == CamSurfaceType::Plane) {
            range.first = range.second = normal.z != 0 ? centroid.z : bbox.min.z;
        } else {
            range.first = bbox.min.z;
            range.second = bbox.max.z;
        }
        return range.second - range.first;
    }

    CamPoint closestPoint(const CamPoint& point) const;

    double distanceTo(const CamPoint& point) const {
        return closestPoint(point).distanceTo(point);
    }
};

using CamFacePtr = std::shared_ptr<CamFace>;
using CamFaceList = std::vector<CamFacePtr>;

} // namespace PathForge