// CamFaceBuilder.cpp
#include "CamFaceBuilder.h"

#include <TopoDS.hxx>
#include <TopoDS_Iterator.hxx>
#include <BRep_Tool.hxx>
#include <BRepTools.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GCPnts_UniformAbscissa.hxx>
#include <GCPnts_QuasiUniformAbscissa.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Surface.hxx>
#include <Geom_Plane.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom2dAdaptor_HCurve.hxx>
#include <ElCLib.hxx>
#include <Precision.hxx>

#include "Geometry/CAM/CamEdge.h"
#include "Geometry/CAM/CamWire.h"

namespace PathForge {

CamFacePtr CamFaceBuilder::build(const TopoDS_Face& face) {
    auto camFace = std::make_shared<CamFace>();

    camFace->surfaceType = getSurfaceType(face);
    camFace->normal = computeNormal(face);
    camFace->uvRange = computeUVRange(face);
    camFace->bbox = computeBoundingBox(face);
    camFace->centroid = computeCentroid(face);
    camFace->area = computeArea(face);
    camFace->surfaceEvaluator = [&face](double u, double v) {
        BRepAdaptor_Surface surface(face);
        gp_Pnt p = surface.Value(u, v);
        return CamPoint(p.X(), p.Y(), p.Z());
    };
    camFace->normalEvaluator = [&face](double u, double v) {
        BRepAdaptor_Surface surface(face);
        gp_Pnt p;
        gp_Vec d1u, d1v;
        surface.D1(u, v, p, d1u, d1v);
        gp_Vec n = d1u.Crossed(d1v);
        if (n.Magnitude() > Precision::SquareAngular()) {
            n.Normalize();
        }
        return CamDirection(n.X(), n.Y(), n.Z());
    };

    std::vector<TopoDS_Wire> outerWires;
    std::vector<TopoDS_Wire> holeWires;

    TopoDS_Iterator itWires(face);
    for (; itWires.More(); itWires.Next()) {
        const TopoDS_Wire& wire = TopoDS::Wire(itWires.Value());
        if (isOuterBoundary(wire)) {
            outerWires.push_back(wire);
        } else {
            holeWires.push_back(wire);
        }
    }

    if (outerWires.empty()) {
        itWires.Initialize(face);
        if (itWires.More()) {
            outerWires.push_back(TopoDS::Wire(itWires.Value()));
        }
    }

    if (!outerWires.empty()) {
        camFace->outerBoundary = extractWire(outerWires[0]);
    }

    for (const auto& holeWire : holeWires) {
        camFace->holeBoundaries.push_back(extractWire(holeWire));
    }

    return camFace;
}

CamFaceList CamFaceBuilder::buildAll(const TopoDS_Shape& shape) {
    CamFaceList faces;

    TopoDS_Iterator itFaces(shape);
    for (; itFaces.More(); itFaces.Next()) {
        const TopoDS_Shape& faceShape = itFaces.Value();
        if (faceShape.ShapeType() == TopAbs_FACE) {
            TopoDS_Face occFace = TopoDS::Face(faceShape);
            CamFacePtr camFace = build(occFace);
            camFace->id = static_cast<int>(faces.size());
            faces.push_back(camFace);
        }
    }

    return faces;
}

CamWire CamFaceBuilder::extractWire(const TopoDS_Wire& wire) {
    CamWire camWire;

    std::vector<CamEdge> edges = extractEdgesFromWire(wire);
    camWire.edges = edges;
    camWire.isClosed = camWire.isClosedLoop();
    camWire.type = CamWireType::Outer;

    return camWire;
}

std::vector<CamEdge> CamFaceBuilder::extractEdgesFromWire(const TopoDS_Wire& wire) {
    std::vector<CamEdge> edges;

    TopoDS_Iterator itEdges(wire);
    for (; itEdges.More(); itEdges.Next()) {
        const TopoDS_Edge& edge = TopoDS::Edge(itEdges.Value());
        EdgeInfo edgeInfo = extractEdge(edge);

        CamEdge camEdge;
        camEdge.points = edgeInfo.points;
        camEdge.isReversed = edgeInfo.isReversed;

        if (!edgeInfo.points.empty()) {
            Standard_Real first, last;
            Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);
            if (curve) {
                if (curve->IsKind(STANDARD_TYPE(Geom_Circle))) {
                    camEdge.type = CamEdgeType::Circle;
                } else if (curve->IsKind(STANDARD_TYPE(Geom_BezierCurve))) {
                    camEdge.type = CamEdgeType::Bezier;
                } else if (curve->IsKind(STANDARD_TYPE(Geom_BSplineCurve))) {
                    camEdge.type = CamEdgeType::BSpline;
                } else {
                    camEdge.type = CamEdgeType::Line;
                }
            } else {
                camEdge.type = CamEdgeType::Line;
            }
            edges.push_back(camEdge);
        }
    }

    return edges;
}

CamFaceBuilder::EdgeInfo CamFaceBuilder::extractEdge(const TopoDS_Edge& edge) {
    EdgeInfo info;

    Standard_Real first, last;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);

    if (curve.IsNull()) {
        TopoDS_Iterator itEV(edge);
        if (itEV.More()) {
            const TopoDS_Shape& geom = itEV.Value();
            if (geom.ShapeType() == TopAbs_EDGE) {
            }
        }
        return info;
    }

    double edgeLen = curve->Length(first, last);
    int numPoints = std::max(2, static_cast<int>(edgeLen / m_chordTolerance));

    GCPnts_QuasiUniformAbscissa discretizer;
    discretizer.Initialize(curve, first, last, numPoints);

    if (discretizer.IsDone() && discretizer.NbPoints() > 0) {
        int nbPoints = discretizer.NbPoints();
        info.points.reserve(nbPoints);

        for (int i = 1; i <= nbPoints; ++i) {
            Standard_Real param = discretizer.Parameter(i);
            gp_Pnt p = curve->Value(param);
            info.points.push_back(CamPoint(p.X(), p.Y(), p.Z()));
        }
    } else {
        gp_Pnt p1 = curve->Value(first);
        gp_Pnt p2 = curve->Value(last);
        info.points.push_back(CamPoint(p1.X(), p1.Y(), p1.Z()));
        info.points.push_back(CamPoint(p2.X(), p2.Y(), p2.Z()));
    }

    info.isReversed = edge.Orientation() == TopAbs_REVERSED;

    return info;
}

CamSurfaceType CamFaceBuilder::getSurfaceType(const TopoDS_Face& face) {
    BRepAdaptor_Surface surface(face);
    GeomAbs_SurfaceType type = surface.GetType();

    switch (type) {
        case GeomAbs_Plane: return CamSurfaceType::Plane;
        case GeomAbs_Cylinder: return CamSurfaceType::Cylinder;
        case GeomAbs_Cone: return CamSurfaceType::Cone;
        case GeomAbs_Sphere: return CamSurfaceType::Sphere;
        case GeomAbs_BezierSurface: return CamSurfaceType::Bezier;
        case GeomAbs_BSplineSurface: return CamSurfaceType::BSpline;
        default: return CamSurfaceType::Unknown;
    }
}

CamDirection CamFaceBuilder::computeNormal(const TopoDS_Face& face) {
    BRepAdaptor_Surface surface(face);

    if (surface.GetType() == GeomAbs_Plane) {
        gp_Pln plane = surface.Plane();
        gp_Dir dir = plane.Axis().Direction();
        CamDirection camDir(dir.X(), dir.Y(), dir.Z());
        if (face.Orientation() == TopAbs_REVERSED) {
            return camDir.reversed();
        }
        return camDir;
    }

    Standard_Real u = surface.URange().First + surface.URange().Length() / 2;
    Standard_Real v = surface.VRange().First + surface.VRange().Length() / 2;

    gp_Pnt p;
    gp_Vec d1u, d1v;
    surface.D1(u, v, p, d1u, d1v);

    gp_Vec normal = d1u.Crossed(d1v);
    if (normal.Magnitude() > Precision::SquareAngular()) {
        normal.Normalize();
        CamDirection camDir(normal.X(), normal.Y(), normal.Z());
        if (face.Orientation() == TopAbs_REVERSED) {
            return camDir.reversed();
        }
        return camDir;
    }

    return CamDirection::axisZ();
}

CamUVRange CamFaceBuilder::computeUVRange(const TopoDS_Face& face) {
    BRepAdaptor_Surface surface(face);
    Standard_Real umin = surface.URange().First;
    Standard_Real umax = surface.URange().Last;
    Standard_Real vmin = surface.VRange().First;
    Standard_Real vmax = surface.VRange().Last;

    return CamUVRange(umin, vmin, umax, vmax);
}

CamBoundingBox CamFaceBuilder::computeBoundingBox(const TopoDS_Face& face) {
    CamBoundingBox bbox;
    bool initialized = false;

    TopoDS_Iterator itWires(face);
    for (; itWires.More(); itWires.Next()) {
        const TopoDS_Wire& wire = TopoDS::Wire(itWires.Value());
        TopoDS_Iterator itEdges(wire);
        for (; itEdges.More(); itEdges.Next()) {
            const TopoDS_Edge& edge = TopoDS::Edge(itEdges.Value());
            EdgeInfo edgeInfo = extractEdge(edge);

            for (const auto& p : edgeInfo.points) {
                if (!initialized) {
                    bbox.min = p;
                    bbox.max = p;
                    initialized = true;
                } else {
                    bbox.min.x = std::min(bbox.min.x, p.x);
                    bbox.min.y = std::min(bbox.min.y, p.y);
                    bbox.min.z = std::min(bbox.min.z, p.z);
                    bbox.max.x = std::max(bbox.max.x, p.x);
                    bbox.max.y = std::max(bbox.max.y, p.y);
                    bbox.max.z = std::max(bbox.max.z, p.z);
                }
            }
        }
    }

    if (!initialized) {
        BRepAdaptor_Surface surface(face);
        Standard_Real u = surface.URange().First + surface.URange().Length() / 2;
        Standard_Real v = surface.VRange().First + surface.VRange().Length() / 2;
        gp_Pnt center = surface.Value(u, v);
        bbox.min = CamPoint(center.X(), center.Y(), center.Z());
        bbox.max = bbox.min;
    }

    return bbox;
}

CamPoint CamFaceBuilder::computeCentroid(const TopoDS_Face& face) {
    std::vector<CamPoint> allPoints;

    TopoDS_Iterator itWires(face);
    for (; itWires.More(); itWires.Next()) {
        const TopoDS_Wire& wire = TopoDS::Wire(itWires.Value());
        TopoDS_Iterator itEdges(wire);
        for (; itEdges.More(); itEdges.Next()) {
            const TopoDS_Edge& edge = TopoDS::Edge(itEdges.Value());
            EdgeInfo edgeInfo = extractEdge(edge);
            allPoints.insert(allPoints.end(), edgeInfo.points.begin(), edgeInfo.points.end());
        }
    }

    if (allPoints.empty()) {
        BRepAdaptor_Surface surface(face);
        Standard_Real u = surface.URange().First + surface.URange().Length() / 2;
        Standard_Real v = surface.VRange().First + surface.VRange().Length() / 2;
        gp_Pnt center = surface.Value(u, v);
        return CamPoint(center.X(), center.Y(), center.Z());
    }

    double cx = 0, cy = 0, cz = 0;
    for (const auto& p : allPoints) {
        cx += p.x;
        cy += p.y;
        cz += p.z;
    }

    size_t n = allPoints.size();
    return CamPoint(cx / n, cy / n, cz / n);
}

double CamFaceBuilder::computeArea(const TopoDS_Face& face) {
    double area = 0.0;

    TopoDS_Iterator itWires(face);
    for (; itWires.More(); itWires.Next()) {
        const TopoDS_Wire& wire = TopoDS::Wire(itWires.Value());
        std::vector<CamPoint> points;

        TopoDS_Iterator itEdges(wire);
        for (; itEdges.More(); itEdges.Next()) {
            const TopoDS_Edge& edge = TopoDS::Edge(itEdges.Value());
            EdgeInfo edgeInfo = extractEdge(edge);
            for (const auto& p : edgeInfo.points) {
                points.push_back(p);
            }
        }

        if (points.size() >= 3) {
            double polyArea = 0.0;
            for (size_t i = 0; i < points.size(); ++i) {
                size_t j = (i + 1) % points.size();
                polyArea += points[i].x * points[j].y;
                polyArea -= points[j].x * points[i].y;
            }
            area += std::abs(polyArea) / 2.0;
        }
    }

    return area;
}

bool CamFaceBuilder::isOuterBoundary(const TopoDS_Wire& wire) {
    TopoDS_Iterator itOuter(wire);
    if (itOuter.More()) {
        const TopoDS_Shape& firstShape = itOuter.Value();
        if (firstShape.ShapeType() == TopAbs_EDGE) {
            TopoDS_Edge firstEdge = TopoDS::Edge(firstShape);
            Standard_Real first, last;
            Handle(Geom_Curve) curve = BRep_Tool::Curve(firstEdge, first, last);
            return !curve.IsNull();
        }
    }
    return true;
}

} // namespace PathForge