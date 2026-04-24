// CamFaceBuilder.h
#pragma once

#include <memory>
#include <vector>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>

#include "CamFace.h"

namespace PathForge {

class CamFaceBuilder {
public:
    CamFaceBuilder() = default;
    ~CamFaceBuilder() = default;

    CamFacePtr build(const TopoDS_Face& face);
    CamFaceList buildAll(const TopoDS_Shape& shape);

    void setDiscretizationTolerance(double tol) { m_chordTolerance = tol; }
    void setMinEdgeSize(double minSize) { m_minEdgeSize = minSize; }
    void setSimplifyTolerance(double tol) { m_simplifyTolerance = tol; }
    void setPreserveAllPoints(bool preserve) { m_preserveAllPoints = preserve; }
    void setMaxDiscretizationPoints(int maxPoints) { m_maxPoints = maxPoints; }

private:
    struct EdgeInfo {
        std::vector<CamPoint> points;
        bool isReversed = false;
    };

    CamWire extractWire(const TopoDS_Wire& wire);
    EdgeInfo extractEdge(const TopoDS_Edge& edge);
    CamSurfaceType getSurfaceType(const TopoDS_Face& face);
    CamDirection computeNormal(const TopoDS_Face& face);
    CamUVRange computeUVRange(const TopoDS_Face& face);
    CamBoundingBox computeBoundingBox(const TopoDS_Face& face);
    CamPoint computeCentroid(const TopoDS_Face& face);
    double computeArea(const TopoDS_Face& face);
    bool isOuterBoundary(const TopoDS_Wire& wire);
    bool isOuterBoundary(const TopoDS_Wire& wire, const TopoDS_Face& face);
    std::vector<CamEdge> extractEdgesFromWire(const TopoDS_Wire& wire);
    std::vector<CamPoint> sampleEdgePoints(const TopoDS_Edge& edge);

    double m_chordTolerance = 0.01;
    double m_minEdgeSize = 0.001;
    double m_simplifyTolerance = 0.0;
    bool m_preserveAllPoints = true;
    int m_maxPoints = 10000;
};

using CamFaceBuilderPtr = std::shared_ptr<CamFaceBuilder>;

} // namespace PathForge