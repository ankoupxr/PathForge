#pragma once

#include <vector>
#include <memory>

#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gp_Dir.hxx>

#include "../Toolpath.h"

namespace PathForge::Path {

class Projector {
public:
    virtual ~Projector() = default;

    virtual std::vector<gp_Pnt> project(
        const std::vector<gp_Pnt>& path2D,
        const gp_Pnt& referencePoint
    ) = 0;

    virtual std::vector<gp_Pnt> projectOnSurface(
        const std::vector<gp_Pnt>& path2D,
        const std::vector<gp_Vec>& surfaceNormals
    ) = 0;

    virtual gp_Vec calculateNormal(const gp_Pnt& point) = 0;

    virtual void setProjectionDirection(const gp_Dir& direction);
    virtual void setTolerance(double tolerance);

protected:
    Projector();

    gp_Dir m_projectionDirection;
    double m_tolerance = 0.01;
};

class SurfaceProjector : public Projector {
public:
    SurfaceProjector();
    explicit SurfaceProjector(const std::vector<gp_Pnt>& surfacePoints);

    std::vector<gp_Pnt> project(
        const std::vector<gp_Pnt>& path2D,
        const gp_Pnt& referencePoint
    ) override;

    std::vector<gp_Pnt> projectOnSurface(
        const std::vector<gp_Pnt>& path2D,
        const std::vector<gp_Vec>& surfaceNormals
    ) override;

    gp_Vec calculateNormal(const gp_Pnt& point) override;

    void setSurfacePoints(const std::vector<gp_Pnt>& points);
    void setSurfaceNormals(const std::vector<gp_Vec>& normals);

    void enableNormalSmoothing(bool enable);
    void setNormalSmoothingFactor(double factor);

private:
    gp_Pnt findClosestSurfacePoint(const gp_Pnt& point);
    gp_Vec interpolateNormal(size_t index);

    std::vector<gp_Pnt> m_surfacePoints;
    std::vector<gp_Vec> m_surfaceNormals;
    bool m_useSmoothing = true;
    double m_smoothingFactor = 0.2;
};

class UVProjector : public Projector {
public:
    UVProjector();

    std::vector<gp_Pnt> project(
        const std::vector<gp_Pnt>& path2D,
        const gp_Pnt& referencePoint
    ) override;

    std::vector<gp_Pnt> projectOnSurface(
        const std::vector<gp_Pnt>& path2D,
        const std::vector<gp_Vec>& surfaceNormals
    ) override;

    gp_Vec calculateNormal(const gp_Pnt& point) override;

    void setUVBounds(double uMin, double uMax, double vMin, double vMax);
    void setSurfaceFunction(
        std::function<gp_Pnt(double u, double v)> func,
        std::function<gp_Vec(double u, double v)> normalFunc
    );

private:
    gp_Pnt evaluateSurface(double u, double v);
    gp_Vec evaluateNormal(double u, double v);

    double m_uMin = 0.0, m_uMax = 1.0;
    double m_vMin = 0.0, m_vMax = 1.0;
    std::function<gp_Pnt(double u, double v)> m_surfaceFunc;
    std::function<gp_Vec(double u, double v)> m_normalFunc;
};

class MultiSurfaceProjector : public Projector {
public:
    MultiSurfaceProjector();

    std::vector<gp_Pnt> project(
        const std::vector<gp_Pnt>& path2D,
        const gp_Pnt& referencePoint
    ) override;

    std::vector<gp_Pnt> projectOnSurface(
        const std::vector<gp_Pnt>& path2D,
        const std::vector<gp_Vec>& surfaceNormals
    ) override;

    gp_Vec calculateNormal(const gp_Pnt& point) override;

    void addSurface(
        const std::vector<gp_Pnt>& points,
        const std::vector<gp_Vec>& normals
    );

    void setTransitionTolerance(double tolerance);
    void enableSmoothTransition(bool enable);

private:
    size_t findNearestSurface(const gp_Pnt& point);
    std::vector<gp_Pnt> transitionPath(
        const std::vector<gp_Pnt>& path,
        size_t fromSurface,
        size_t toSurface
    );

    struct SurfaceData {
        std::vector<gp_Pnt> points;
        std::vector<gp_Vec> normals;
    };

    std::vector<SurfaceData> m_surfaces;
    double m_transitionTolerance = 1.0;
    bool m_smoothTransition = true;
};

using ProjectorPtr = std::shared_ptr<Projector>;

}
