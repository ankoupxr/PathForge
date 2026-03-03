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
#include <gp_Pln.hxx>
#include <algorithm>
#include <cmath>
#include <TopoDS_Iterator.hxx>
#include <Geom_Curve.hxx>

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

} // namespace PathForge
