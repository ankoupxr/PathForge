// CamWire.cpp
#include "CamWire.h"
#include <algorithm>
#include <cmath>

namespace PathForge {

bool CamWire::containsPoint(const CamPoint& point, double tolerance) const {
    if (!isClosedLoop()) {
        return false;
    }

    const auto& points = getAllPoints();
    if (points.size() < 3) {
        return false;
    }

    int windingNumber = 0;
    size_t n = points.size();

    for (size_t i = 0; i < n; ++i) {
        const CamPoint& p1 = points[i];
        const CamPoint& p2 = points[(i + 1) % n];

        if (p1.y <= point.y) {
            if (p2.y > point.y) {
                if (isLeft(p1, p2, point) > 0) {
                    ++windingNumber;
                }
            }
        } else {
            if (p2.y <= point.y) {
                if (isLeft(p1, p2, point) < 0) {
                    --windingNumber;
                }
            }
        }
    }

    return windingNumber != 0;
}

double CamWire::isLeft(const CamPoint& p1, const CamPoint& p2, const CamPoint& p0) const {
    return (p2.x - p1.x) * (p0.y - p1.y) - (p0.x - p1.x) * (p2.y - p1.y);
}

std::vector<CamPoint> CamWire::samplePoints(double interval) const {
    std::vector<CamPoint> sampledPoints;

    for (const auto& edge : edges) {
        double edgeLen = edge.length();
        if (edgeLen < 1e-10) continue;

        int numSamples = std::max(2, static_cast<int>(edgeLen / interval));

        for (int i = 0; i <= numSamples; ++i) {
            double t = static_cast<double>(i) / numSamples;
            const auto& pts = edge.points;

            if (pts.size() >= 2) {
                if (edge.isReversed) {
                    size_t idx = pts.size() - 1 - i;
                    if (idx < pts.size()) {
                        sampledPoints.push_back(pts[idx]);
                    }
                } else {
                    size_t idx = i;
                    if (idx < pts.size()) {
                        sampledPoints.push_back(pts[idx]);
                    }
                }
            }
        }
    }

    return sampledPoints;
}

CamWire CamWire::offset(double distance) const {
    CamWire offsetWire;
    offsetWire.type = type;
    offsetWire.isClosed = isClosed;

    const auto& points = getAllPoints();
    if (points.size() < 3) {
        return *this;
    }

    std::vector<CamPoint> offsetPoints;
    size_t n = points.size();

    for (size_t i = 0; i < n; ++i) {
        const CamPoint& prev = points[(i + n - 1) % n];
        const CamPoint& curr = points[i];
        const CamPoint& next = points[(i + 1) % n];

        CamVector v1 = (curr - prev).normalized();
        CamVector v2 = (next - curr).normalized();

        CamVector normal(-v1.y, v1.x, v1.z);

        CamVector bisector = (v1 + v2).normalized();

        double cosAngle = v1.dot(v2);
        double sinAngle = std::sqrt(1 - cosAngle * cosAngle);

        double offsetAmount = distance / sinAngle;

        CamPoint offsetPoint = curr + bisector * offsetAmount;

        offsetPoints.push_back(offsetPoint);
    }

    for (size_t i = 0; i < offsetPoints.size(); ++i) {
        size_t next = (i + 1) % offsetPoints.size();
        if (next == 0 && !isClosed) break;
        offsetWire.edges.emplace_back(offsetPoints[i], offsetPoints[next]);
    }

    return offsetWire;
}

CamWire CamWire::simplify(double tolerance) const {
    CamWire simplified;
    simplified.type = type;
    simplified.isClosed = isClosed;

    auto points = getAllPoints();
    if (points.size() <= 2) {
        simplified.edges = edges;
        return simplified;
    }

    std::vector<CamPoint> simplifiedPoints = douglasPeucker(points, tolerance);

    for (size_t i = 0; i < simplifiedPoints.size(); ++i) {
        size_t next = (i + 1) % simplifiedPoints.size();
        if (next == 0 && !isClosed) break;
        simplified.edges.emplace_back(simplifiedPoints[i], simplifiedPoints[next]);
    }

    return simplified;
}

std::vector<CamPoint> CamWire::douglasPeucker(const std::vector<CamPoint>& points, double tolerance) {
    if (points.size() <= 2) {
        return points;
    }

    double maxDist = 0;
    size_t maxIndex = 0;

    CamPoint first = points.front();
    CamPoint last = points.back();

    for (size_t i = 1; i < points.size() - 1; ++i) {
        double dist = perpendicularDistance(points[i], first, last);
        if (dist > maxDist) {
            maxDist = dist;
            maxIndex = i;
        }
    }

    if (maxDist > tolerance) {
        auto left = douglasPeucker(
            std::vector<CamPoint>(points.begin(), points.begin() + maxIndex + 1),
            tolerance
        );
        auto right = douglasPeucker(
            std::vector<CamPoint>(points.begin() + maxIndex, points.end()),
            tolerance
        );

        left.insert(left.end(), right.begin() + 1, right.end());
        return left;
    } else {
        return {first, last};
    }
}

double CamWire::perpendicularDistance(const CamPoint& point,
                                       const CamPoint& lineStart,
                                       const CamPoint& lineEnd) {
    CamVector line = lineEnd - lineStart;
    double lineLen = line.magnitude();
    if (lineLen < 1e-10) {
        return point.distanceTo(lineStart);
    }

    CamVector v = point - lineStart;
    double t = v.dot(line) / (lineLen * lineLen);
    t = std::max(0.0, std::min(1.0, t));

    CamPoint projection = lineStart + line * t;
    return point.distanceTo(projection);
}

} // namespace PathForge