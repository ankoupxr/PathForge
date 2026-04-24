// CamWire.h
#pragma once

#include <vector>
#include <memory>
#include "CamEdge.h"
#include "CamPoint.h"

namespace PathForge {

enum class CamWireType {
    Outer,
    Hole,
    Unknown
};

class CamWire {
public:
    std::vector<CamEdge> edges;
    CamWireType type = CamWireType::Outer;
    bool isClosed = true;

    CamWire() = default;

    explicit CamWire(const std::vector<CamEdge>& edges_, CamWireType t = CamWireType::Outer)
        : edges(edges_), type(t) {
        checkClosed();
    }

    explicit CamWire(const std::vector<CamPoint>& points, CamWireType t = CamWireType::Outer)
        : type(t), isClosed(true) {
        for (size_t i = 0; i < points.size(); ++i) {
            size_t next = (i + 1) % points.size();
            if (next == 0 && !isClosed) break;
            edges.emplace_back(points[i], points[next]);
        }
    }

    void addEdge(const CamEdge& edge) {
        edges.push_back(edge);
        checkClosed();
    }

    void addEdges(const std::vector<CamEdge>& newEdges) {
        edges.insert(edges.end(), newEdges.begin(), newEdges.end());
        checkClosed();
    }

    size_t edgeCount() const { return edges.size(); }

    size_t pointCount() const {
        if (edges.empty()) return 0;
        size_t count = 0;
        for (const auto& edge : edges) {
            count += edge.pointCount();
        }
        return count;
    }

    std::vector<CamPoint> getAllPoints() const {
        std::vector<CamPoint> result;
        for (const auto& edge : edges) {
            for (const auto& p : edge.points) {
                result.push_back(p);
            }
        }
        return result;
    }

    CamPoint firstPoint() const {
        if (edges.empty()) return CamPoint::origin();
        return edges.front().firstPoint();
    }

    CamPoint lastPoint() const {
        if (edges.empty()) return CamPoint::origin();
        return edges.back().lastPoint();
    }

    bool isClosedLoop(double tolerance = 1e-6) const {
        return isClosed && !edges.empty() &&
               firstPoint().distanceTo(lastPoint()) < tolerance;
    }

    double perimeter() const {
        double peri = 0.0;
        for (const auto& edge : edges) {
            peri += edge.length();
        }
        return peri;
    }

    CamPoint centroid() const {
        auto points = getAllPoints();
        if (points.empty()) return CamPoint::origin();

        double cx = 0, cy = 0, cz = 0;
        for (const auto& p : points) {
            cx += p.x;
            cy += p.y;
            cz += p.z;
        }
        size_t n = points.size();
        return CamPoint(cx / n, cy / n, cz / n);
    }

    bool containsPoint(const CamPoint& point, double tolerance = 1e-6) const;

    CamWire reversed() const {
        CamWire rev;
        rev.type = type;
        rev.isClosed = isClosed;
        for (auto it = edges.rbegin(); it != edges.rend(); ++it) {
            rev.edges.push_back(it->reversed());
        }
        return rev;
    }

    std::vector<CamPoint> samplePoints(double interval) const;

    CamWire offset(double distance) const;

    CamWire simplify(double tolerance = 0.01) const;

private:
    void checkClosed() {
        if (edges.size() >= 2) {
            isClosed = firstPoint().distanceTo(lastPoint()) < 1e-6;
        }
    }
};

using CamWirePtr = std::shared_ptr<CamWire>;
using CamWireList = std::vector<CamWirePtr>;

} // namespace PathForge