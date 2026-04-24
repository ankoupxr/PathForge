// CamFace.cpp
#include "CamFace.h"
#include "CamWire.h"
#include <cmath>
#include <algorithm>

namespace PathForge {

std::vector<CamPoint> CamFace::sampleGridPoints(size_t uSamples, size_t vSamples) const {
    std::vector<CamPoint> gridPoints;

    if (surfaceType != CamSurfaceType::Plane) {
        return gridPoints;
    }

    for (size_t i = 0; i <= uSamples; ++i) {
        for (size_t j = 0; j <= vSamples; ++j) {
            double u = umin.u + (umax.u - umin.u) * static_cast<double>(i) / uSamples;
            double v = umin.v + (umax.v - umin.v) * static_cast<double>(j) / vSamples;
            gridPoints.push_back(evaluate(u, v));
        }
    }

    return gridPoints;
}

CamPoint CamFace::evaluate(double u, double v) const {
    if (surfaceEvaluator) {
        return surfaceEvaluator(u, v);
    }

    if (surfaceType == CamSurfaceType::Plane) {
        double uSpan = umax.u - umin.u;
        double vSpan = umax.v - umin.v;

        auto boundaryPoints = outerBoundary.getAllPoints();
        if (boundaryPoints.empty()) {
            return centroid;
        }

        double minX = 1e100, maxX = -1e100;
        double minY = 1e100, maxY = -1e100;
        for (const auto& p : boundaryPoints) {
            minX = std::min(minX, p.x);
            maxX = std::max(maxX, p.x);
            minY = std::min(minY, p.y);
            maxY = std::max(maxY, p.y);
        }

        if (std::abs(normal.z) > 0.9) {
            double x = minX + (maxX - minX) * u;
            double y = minY + (maxY - minY) * v;
            double z = centroid.z;
            return CamPoint(x, y, z);
        } else if (std::abs(normal.y) > 0.9) {
            double x = minX + (maxX - minX) * u;
            double z = minY + (maxY - minY) * v;
            double y = centroid.y;
            return CamPoint(x, y, z);
        } else {
            double y = minX + (maxX - minX) * u;
            double z = minY + (maxY - minY) * v;
            double x = centroid.x;
            return CamPoint(x, y, z);
        }
    }

    return centroid;
}

CamPoint2D CamFace::projectToUV(const CamPoint& point3D) const {
    auto boundaryPoints = outerBoundary.getAllPoints();
    if (boundaryPoints.empty()) {
        return CamPoint2D(0.5, 0.5);
    }

    double minX = 1e100, maxX = -1e100;
    double minY = 1e100, maxY = -1e100;
    for (const auto& p : boundaryPoints) {
        minX = std::min(minX, p.x);
        maxX = std::max(maxX, p.x);
        minY = std::min(minY, p.y);
        maxY = std::max(maxY, p.y);
    }

    double u, v;
    if (std::abs(normal.z) > 0.9) {
        u = (point3D.x - minX) / (maxX - minX);
        v = (point3D.y - minY) / (maxY - minY);
    } else if (std::abs(normal.y) > 0.9) {
        u = (point3D.x - minX) / (maxX - minX);
        v = (point3D.z - minY) / (maxY - minY);
    } else {
        u = (point3D.y - minX) / (maxX - minX);
        v = (point3D.z - minY) / (maxY - minY);
    }

    return CamPoint2D(u, v);
}

bool CamFace::isPointInside(const CamPoint& point) const {
    if (!bbox.contains(point)) {
        return false;
    }

    return outerBoundary.containsPoint(point);
}

CamPoint CamFace::closestPoint(const CamPoint& point) const {
    if (surfaceType == CamSurfaceType::Plane) {
        double dist = (point - centroid).dot(CamVector(normal));
        return point - normal * dist;
    }
    return centroid;
}

} // namespace PathForge