#include "Projector.h"
#include <cmath>
#include <algorithm>
#include <limits>

namespace PathForge::Path {

Projector::Projector()
    : m_projectionDirection(0, 0, -1) {}

void Projector::setProjectionDirection(const gp_Dir& direction) {
    m_projectionDirection = direction;
}

void Projector::setTolerance(double tolerance) {
    m_tolerance = tolerance;
}

SurfaceProjector::SurfaceProjector() = default;

SurfaceProjector::SurfaceProjector(const std::vector<gp_Pnt>& surfacePoints)
    : m_surfacePoints(surfacePoints) {}

void SurfaceProjector::setSurfacePoints(const std::vector<gp_Pnt>& points) {
    m_surfacePoints = points;
}

void SurfaceProjector::setSurfaceNormals(const std::vector<gp_Vec>& normals) {
    m_surfaceNormals = normals;
}

void SurfaceProjector::enableNormalSmoothing(bool enable) {
    m_useSmoothing = enable;
}

void SurfaceProjector::setNormalSmoothingFactor(double factor) {
    m_smoothingFactor = factor;
}

gp_Pnt SurfaceProjector::findClosestSurfacePoint(const gp_Pnt& point) {
    if (m_surfacePoints.empty()) return point;

    double minDist = std::numeric_limits<double>::max();
    gp_Pnt closest = m_surfacePoints[0];

    for (const auto& surfPoint : m_surfacePoints) {
        double dist = point.Distance(surfPoint);
        if (dist < minDist) {
            minDist = dist;
            closest = surfPoint;
        }
    }

    return closest;
}

gp_Vec SurfaceProjector::interpolateNormal(size_t index) {
    if (m_surfaceNormals.empty() || index >= m_surfaceNormals.size()) {
        return gp_Vec(0, 0, 1);
    }
    return m_surfaceNormals[index];
}

std::vector<gp_Pnt> SurfaceProjector::project(
    const std::vector<gp_Pnt>& path2D,
    const gp_Pnt& referencePoint
) {
    std::vector<gp_Pnt> result;

    for (const auto& point : path2D) {
        gp_Pnt projected = findClosestSurfacePoint(point);
        result.push_back(projected);
    }

    return result;
}

std::vector<gp_Pnt> SurfaceProjector::projectOnSurface(
    const std::vector<gp_Pnt>& path2D,
    const std::vector<gp_Vec>& surfaceNormals
) {
    std::vector<gp_Pnt> result;

    for (size_t i = 0; i < path2D.size(); ++i) {
        const auto& point = path2D[i];
        
        gp_Vec normal = (i < surfaceNormals.size()) 
            ? surfaceNormals[i] 
            : calculateNormal(point);
        
        if (m_useSmoothing && i > 0) {
            const auto& prevNormal = (i - 1 < surfaceNormals.size()) 
                ? surfaceNormals[i - 1] 
                : gp_Vec(0, 0, 1);
            normal = gp_Vec(
                prevNormal.X() * (1 - m_smoothingFactor) + normal.X() * m_smoothingFactor,
                prevNormal.Y() * (1 - m_smoothingFactor) + normal.Y() * m_smoothingFactor,
                prevNormal.Z() * (1 - m_smoothingFactor) + normal.Z() * m_smoothingFactor
            );
        }

        gp_Vec offset = normal * 0.1;
        gp_Pnt projected(point.X() + offset.X(), point.Y() + offset.Y(), point.Z() + offset.Z());
        result.push_back(projected);
    }

    return result;
}

gp_Vec SurfaceProjector::calculateNormal(const gp_Pnt& point) {
    if (!m_surfaceNormals.empty()) {
        return interpolateNormal(0);
    }
    return gp_Vec(0, 0, 1);
}

UVProjector::UVProjector() = default;

void UVProjector::setUVBounds(double uMin, double uMax, double vMin, double vMax) {
    m_uMin = uMin;
    m_uMax = uMax;
    m_vMin = vMin;
    m_vMax = vMax;
}

void UVProjector::setSurfaceFunction(
    std::function<gp_Pnt(double u, double v)> func,
    std::function<gp_Vec(double u, double v)> normalFunc
) {
    m_surfaceFunc = func;
    m_normalFunc = normalFunc;
}

gp_Pnt UVProjector::evaluateSurface(double u, double v) {
    if (m_surfaceFunc) {
        return m_surfaceFunc(u, v);
    }
    return gp_Pnt(u, v, 0);
}

gp_Vec UVProjector::evaluateNormal(double u, double v) {
    if (m_normalFunc) {
        return m_normalFunc(u, v);
    }
    return gp_Vec(0, 0, 1);
}

std::vector<gp_Pnt> UVProjector::project(
    const std::vector<gp_Pnt>& path2D,
    const gp_Pnt& referencePoint
) {
    std::vector<gp_Pnt> result;

    for (const auto& point : path2D) {
        double u = (point.X() - referencePoint.X()) / 100.0;
        double v = (point.Y() - referencePoint.Y()) / 100.0;
        
        u = std::clamp(u, m_uMin, m_uMax);
        v = std::clamp(v, m_vMin, m_vMax);

        gp_Pnt projected = evaluateSurface(u, v);
        result.push_back(projected);
    }

    return result;
}

std::vector<gp_Pnt> UVProjector::projectOnSurface(
    const std::vector<gp_Pnt>& path2D,
    const std::vector<gp_Vec>& surfaceNormals
) {
    return project(path2D, gp_Pnt(0, 0, 0));
}

gp_Vec UVProjector::calculateNormal(const gp_Pnt& point) {
    return evaluateNormal(0.5, 0.5);
}

MultiSurfaceProjector::MultiSurfaceProjector() = default;

void MultiSurfaceProjector::setTransitionTolerance(double tolerance) {
    m_transitionTolerance = tolerance;
}

void MultiSurfaceProjector::enableSmoothTransition(bool enable) {
    m_smoothTransition = enable;
}

void MultiSurfaceProjector::addSurface(
    const std::vector<gp_Pnt>& points,
    const std::vector<gp_Vec>& normals
) {
    SurfaceData data;
    data.points = points;
    data.normals = normals;
    m_surfaces.push_back(data);
}

size_t MultiSurfaceProjector::findNearestSurface(const gp_Pnt& point) {
    if (m_surfaces.empty()) return 0;

    double minDist = std::numeric_limits<double>::max();
    size_t nearest = 0;

    for (size_t i = 0; i < m_surfaces.size(); ++i) {
        for (const auto& surfPoint : m_surfaces[i].points) {
            double dist = point.Distance(surfPoint);
            if (dist < minDist) {
                minDist = dist;
                nearest = i;
            }
        }
    }

    return nearest;
}

std::vector<gp_Pnt> MultiSurfaceProjector::transitionPath(
    const std::vector<gp_Pnt>& path,
    size_t fromSurface,
    size_t toSurface
) {
    if (!m_smoothTransition) return path;

    std::vector<gp_Pnt> result;
    
    if (path.empty()) return result;

    result.push_back(path.front());
    
    if (fromSurface == toSurface || m_surfaces.size() <= 1) {
        result = path;
        return result;
    }

    const auto& fromPoints = m_surfaces[fromSurface].points;
    const auto& toPoints = m_surfaces[toSurface].points;

    if (fromPoints.empty() || toPoints.empty()) {
        result = path;
        return result;
    }

    gp_Pnt transitionStart = fromPoints.back();
    gp_Pnt transitionEnd = toPoints.front();

    int transitionSteps = 10;
    for (int i = 1; i <= transitionSteps; ++i) {
        double t = static_cast<double>(i) / transitionSteps;
        gp_Pnt interp(
            transitionStart.X() * (1 - t) + transitionEnd.X() * t,
            transitionStart.Y() * (1 - t) + transitionEnd.Y() * t,
            transitionStart.Z() * (1 - t) + transitionEnd.Z() * t
        );
        result.push_back(interp);
    }

    for (size_t i = 1; i < path.size(); ++i) {
        result.push_back(path[i]);
    }

    return result;
}

std::vector<gp_Pnt> MultiSurfaceProjector::project(
    const std::vector<gp_Pnt>& path2D,
    const gp_Pnt& referencePoint
) {
    if (m_surfaces.empty()) return path2D;

    std::vector<gp_Pnt> result;
    size_t currentSurface = 0;

    for (size_t i = 0; i < path2D.size(); ++i) {
        const auto& point = path2D[i];
        
        size_t nearestSurface = findNearestSurface(point);
        
        if (nearestSurface != currentSurface && i > 0) {
            auto transitionResult = transitionPath(
                std::vector<gp_Pnt>(path2D.begin() + i, path2D.end()),
                currentSurface,
                nearestSurface
            );
            
            for (const auto& t : transitionResult) {
                result.push_back(t);
            }
            break;
        }

        const auto& surface = m_surfaces[nearestSurface];
        gp_Pnt closestPoint = surface.points[0];
        double minDist = point.Distance(closestPoint);

        for (const auto& surfPoint : surface.points) {
            double dist = point.Distance(surfPoint);
            if (dist < minDist) {
                minDist = dist;
                closestPoint = surfPoint;
            }
        }

        result.push_back(closestPoint);
        currentSurface = nearestSurface;
    }

    return result;
}

std::vector<gp_Pnt> MultiSurfaceProjector::projectOnSurface(
    const std::vector<gp_Pnt>& path2D,
    const std::vector<gp_Vec>& surfaceNormals
) {
    return project(path2D, gp_Pnt(0, 0, 0));
}

gp_Vec MultiSurfaceProjector::calculateNormal(const gp_Pnt& point) {
    if (m_surfaces.empty()) return gp_Vec(0, 0, 1);

    size_t nearest = findNearestSurface(point);
    const auto& normals = m_surfaces[nearest].normals;

    if (!normals.empty()) {
        return normals[0];
    }

    return gp_Vec(0, 0, 1);
}

}
