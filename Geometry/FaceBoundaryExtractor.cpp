// FaceBoundaryExtractor.cpp
#include "FaceBoundaryExtractor.h"

#include <TopoDS.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Edge.hxx>
#include <BRep_Tool.hxx>
#include <BRepTools.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Pln.hxx>
#include <algorithm>
#include <cmath>
#include <TopoDS_Iterator.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Surface.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>

namespace PathForge {

std::vector<gp_Pnt> FaceBoundaryExtractor::extractOuterBoundary(const TopoDS_Face& face) {
    std::vector<gp_Pnt> outerBoundary;

    TopoDS_Iterator itWires(face);
    for (; itWires.More(); itWires.Next()) {
        const TopoDS_Wire& wire = TopoDS::Wire(itWires.Value());

        if (isOuterBoundary(wire)) {
            outerBoundary = extractEdgeLoop(wire);
            break;
        }
    }

    if (outerBoundary.empty()) {
        itWires.Initialize(face);
        if (itWires.More()) {
            outerBoundary = extractEdgeLoop(TopoDS::Wire(itWires.Value()));
        }
    }

    return simplifyBoundary(outerBoundary);
}

std::vector<std::vector<gp_Pnt>> FaceBoundaryExtractor::extractAllBoundaries(
    const TopoDS_Face& face
) const {
    std::vector<std::vector<gp_Pnt>> allBoundaries;

    TopoDS_Iterator itWires(face);
    for (; itWires.More(); itWires.Next()) {
        const TopoDS_Wire& wire = TopoDS::Wire(itWires.Value());
        auto boundary = extractEdgeLoop(wire);
        if (!boundary.empty()) {
            allBoundaries.push_back(simplifyBoundary(boundary));
        }
    }

    return allBoundaries;
}

bool FaceBoundaryExtractor::hasHoles(const TopoDS_Face& face) const {
    int wireCount = 0;
    TopoDS_Iterator itWires(face);
    for (; itWires.More(); itWires.Next()) {
        wireCount++;
    }
    return wireCount > 1;
}

std::pair<double, double> FaceBoundaryExtractor::getZRange(const TopoDS_Face& face) const {
    double minZ = 1e100;
    double maxZ = -1e100;

    BRepAdaptor_Surface surface(face);

    if (surface.GetType() == GeomAbs_Plane) {
        gp_Pln plane = surface.Plane();
        minZ = maxZ = plane.Location().Z();
    } else {
        auto boundaries = extractAllBoundaries(face);
        for (const auto& boundary : boundaries) {
            for (const auto& p : boundary) {
                minZ = std::min(minZ, p.Z());
                maxZ = std::max(maxZ, p.Z());
            }
        }
    }

    return {minZ, maxZ};
}

gp_Pnt FaceBoundaryExtractor::getCentroid(const TopoDS_Face& face) const {
    auto boundary = const_cast<FaceBoundaryExtractor*>(this)->extractOuterBoundary(const_cast<TopoDS_Face&>(face));
    if (boundary.empty()) {
        return gp_Pnt(0, 0, 0);
    }

    double cx = 0, cy = 0, cz = 0;
    for (const auto& p : boundary) {
        cx += p.X();
        cy += p.Y();
        cz += p.Z();
    }
    cx /= boundary.size();
    cy /= boundary.size();
    cz /= boundary.size();

    return gp_Pnt(cx, cy, cz);
}

double FaceBoundaryExtractor::getArea(const TopoDS_Face& face) const {
    BRepAdaptor_Surface surface(face);

    if (surface.GetType() == GeomAbs_Plane) {
        auto boundary = const_cast<FaceBoundaryExtractor*>(this)->extractOuterBoundary(const_cast<TopoDS_Face&>(face));

        double area = 0.0;
        int n = static_cast<int>(boundary.size());
        for (int i = 0; i < n; ++i) {
            const auto& p1 = boundary[i];
            const auto& p2 = boundary[(i + 1) % n];
            area += (p1.X() * p2.Y() - p2.X() * p1.Y());
        }
        return std::abs(area) / 2.0;
    }

    return 0.0;
}

std::vector<gp_Pnt> FaceBoundaryExtractor::simplifyBoundary(
    const std::vector<gp_Pnt>& points,
    double tolerance
) const {
    if (points.size() <= 2) return points;

    std::vector<gp_Pnt> simplified;
    simplified.push_back(points.front());

    for (size_t i = 1; i < points.size() - 1; ++i) {
        const auto& prev = simplified.back();
        const auto& curr = points[i];

        double dist = prev.Distance(curr);
        if (dist > tolerance) {
            simplified.push_back(curr);
        }
    }

    simplified.push_back(points.back());

    if (simplified.size() > 2) {
        if (simplified.front().Distance(simplified.back()) < tolerance) {
            simplified.pop_back();
        }
    }

    return simplified;
}

bool FaceBoundaryExtractor::isPointInside(
    const gp_Pnt& point,
    const std::vector<gp_Pnt>& boundary
) const {
    if (boundary.empty()) return false;

    int crossings = 0;
    size_t n = boundary.size();

    for (size_t i = 0; i < n; ++i) {
        const auto& p1 = boundary[i];
        const auto& p2 = boundary[(i + 1) % n];

        if (((p1.Y() <= point.Y()) && (p2.Y() > point.Y())) ||
            ((p2.Y() <= point.Y()) && (p1.Y() > point.Y()))) {
            double xIntersect = p1.X() + (point.Y() - p1.Y()) / (p2.Y() - p1.Y()) * (p2.X() - p1.X());
            if (point.X() < xIntersect) {
                crossings++;
            }
        }
    }

    return (crossings % 2) == 1;
}

std::vector<gp_Pnt> FaceBoundaryExtractor::extractEdgeLoop(const TopoDS_Shape& edge) const {
    std::vector<gp_Pnt> points;

    TopoDS_Iterator itEdges(edge);
    for (; itEdges.More(); itEdges.Next()) {
        const TopoDS_Edge& currentEdge = TopoDS::Edge(itEdges.Value());

        if (BRep_Tool::Degenerated(currentEdge)) continue;

        double first, last;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(currentEdge, first, last);

        if (curve.IsNull()) continue;

        int segments = 20;
        for (int i = 0; i <= segments; ++i) {
            double t = first + (last - first) * i / segments;
            gp_Pnt p = curve->Value(t);
            points.push_back(p);
        }
    }

    return points;
}

bool FaceBoundaryExtractor::isOuterBoundary(const TopoDS_Shape& wire) const {
    if (wire.ShapeType() != TopAbs_WIRE) return false;

    TopoDS_Iterator itEdges(wire);
    double totalLength = 0.0;

    for (; itEdges.More(); itEdges.Next()) {
        const TopoDS_Edge& edge = TopoDS::Edge(itEdges.Value());
        if (BRep_Tool::Degenerated(edge)) continue;

        double first, last;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);
        if (!curve.IsNull()) {
            totalLength += curve->Value(last).Distance(curve->Value(first));
        }
    }

    int edgeCount = 0;
    TopoDS_Iterator it(wire);
    for (; it.More(); it.Next()) {
        edgeCount++;
    }

    return edgeCount > 0;
}

std::vector<gp_Pnt2d> FaceBoundaryExtractor::unfoldTo2D(const TopoDS_Face& face) const {
    std::vector<gp_Pnt2d> points2d;

    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
    if (surface.IsNull()) {
        return points2d;
    }

    auto uvRange = getUVRange(face);
    double umin = uvRange.first.X();
    double umax = uvRange.second.X();
    double vmin = uvRange.first.Y();
    double vmax = uvRange.second.Y();

    int uSamples = 20;
    int vSamples = 20;
    double du = (umax - umin) / uSamples;
    double dv = (vmax - vmin) / vSamples;

    for (int i = 0; i <= uSamples; ++i) {
        for (int j = 0; j <= vSamples; ++j) {
            double u = umin + i * du;
            double v = vmin + j * dv;
            gp_Pnt2d uv(u, v);
            points2d.push_back(uv);
        }
    }

    return points2d;
}

gp_Pnt FaceBoundaryExtractor::mapTo3D(const TopoDS_Face& face, const gp_Pnt2d& uv) const {
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
    if (surface.IsNull()) {
        return gp_Pnt(0, 0, 0);
    }

    gp_Pnt p = surface->Value(uv.X(), uv.Y());
    return p;
}

std::pair<gp_Pnt2d, gp_Pnt2d> FaceBoundaryExtractor::getUVRange(const TopoDS_Face& face) const {
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
    if (surface.IsNull()) {
        return {gp_Pnt2d(0, 0), gp_Pnt2d(1, 1)};
    }

    Standard_Real umin, umax, vmin, vmax;
    surface->Bounds(umin, umax, vmin, vmax);

    return {gp_Pnt2d(umin, vmin), gp_Pnt2d(umax, vmax)};
}

gp_Pnt2d FaceBoundaryExtractor::mapTo2D(const TopoDS_Face& face, const gp_Pnt& point) const {
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
    if (surface.IsNull()) {
        return gp_Pnt2d(0, 0);
    }

    Standard_Real u = 0, v = 0;
    GeomAPI_ProjectPointOnSurf proj(point, surface);
    if (proj.IsDone()) {
        proj.Parameter(u, v);
    }

    return gp_Pnt2d(u, v);
}

std::vector<gp_Pnt2d> FaceBoundaryExtractor::offset2D(
    const std::vector<gp_Pnt2d>& points,
    double offsetDistance
) const {
    if (points.size() < 2) {
        return points;
    }

    std::vector<gp_Pnt2d> offsetPoints;

    for (size_t i = 0; i < points.size(); ++i) {
        const auto& prev = points[(i + points.size() - 1) % points.size()];
        const auto& curr = points[i];
        const auto& next = points[(i + 1) % points.size()];

        double dx1 = curr.X() - prev.X();
        double dy1 = curr.Y() - prev.Y();
        double len1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
        if (len1 < 1e-10) continue;

        double dx2 = next.X() - curr.X();
        double dy2 = next.Y() - curr.Y();
        double len2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
        if (len2 < 1e-10) continue;

        double nx1 = -dy1 / len1;
        double ny1 = dx1 / len1;
        double nx2 = -dy2 / len2;
        double ny2 = dx2 / len2;

        double nx = (nx1 + nx2) / 2.0;
        double ny = (ny1 + ny2) / 2.0;
        double len = std::sqrt(nx * nx + ny * ny);
        if (len < 1e-10) continue;

        nx /= len;
        ny /= len;

        double newX = curr.X() + nx * offsetDistance;
        double newY = curr.Y() + ny * offsetDistance;

        offsetPoints.push_back(gp_Pnt2d(newX, newY));
    }

    return offsetPoints;
}

} // namespace PathForge
