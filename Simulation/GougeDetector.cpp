#include "GougeDetector.h"

#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>

#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepExtrema_ExtPN.hxx>

#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>

#include <GeomAbs_SurfaceType.hxx>
#include <Geom_Plane.hxx>
#include <Geom_CylindricalSurface.hxx>

#include <algorithm>
#include <cmath>

namespace PathForge::Simulation {

GougeDetector::GougeDetector() = default;

void GougeDetector::setStockShape(const TopoDS_Shape& stock) {
    m_stockShape = stock;
}

void GougeDetector::setToolGeometry(double diameter, double cornerRadius) {
    m_toolDiameter = diameter;
    m_toolCornerRadius = cornerRadius;
}

void GougeDetector::setGougeTolerance(double tolerance) {
    m_gougeTolerance = tolerance;
}

void GougeDetector::setAllowableStock(double stock) {
    m_allowableStock = stock;
}

std::vector<GougeInfo> GougeDetector::detectGouges(const Path::Toolpath& toolpath) {
    m_gouges.clear();

    if (m_stockShape.IsNull()) {
        return m_gouges;
    }

    const auto& points = toolpath.points();

    for (size_t i = 0; i < points.size(); ++i) {
        const auto& pt = points[i];
        const auto& pos = pt.position;

        if (isPointBelowStock(pos)) {
            double distToStock = calculateDistanceToStock(pos);

            double toolRadius = m_toolDiameter / 2.0;
            double effectiveGouge = distToStock + toolRadius - m_allowableStock;

            if (effectiveGouge > m_gougeTolerance) {
                GougeInfo gouge;
                gouge.pointIndex = static_cast<int>(i);
                gouge.position = pos;
                gouge.gougeDepth = effectiveGouge;
                gouge.gougeArea = M_PI * m_toolDiameter * std::abs(distToStock);
                gouge.severity = evaluateGougeSeverity(effectiveGouge);
                gouge.description = getGougeDescription(gouge.severity);

                TopoDS_Face nearestFace = findNearestFace(pos);
                if (!nearestFace.IsNull()) {
                    gouge.face = nearestFace;
                }

                m_gouges.push_back(gouge);
            }
        }
    }

    return m_gouges;
}

bool GougeDetector::isPointBelowStock(const gp_Pnt& point) {
    if (m_stockShape.IsNull()) {
        return false;
    }

    Bnd_Box bbox;
    BRepBndLib::Add(m_stockShape, bbox);
    Standard_Real xMin, yMin, zMin, xMax, yMax, zMax;
    bbox.Get(xMin, yMin, zMin, xMax, yMax, zMax);

    if (point.X() < xMin || point.X() > xMax ||
        point.Y() < yMin || point.Y() > yMax ||
        point.Z() > zMax) {
        return false;
    }

    return point.Z() < zMin - m_gougeTolerance;
}

double GougeDetector::calculateDistanceToStock(const gp_Pnt& point) {
    if (m_stockShape.IsNull()) {
        return 1e10;
    }

    try {
        BRepExtrema_ExtPN extrema(BRepBuilderAPI_MakeVertex(point).Vertex(), m_stockShape);
        if (extrema.IsDone() && extrema.NbSolution() > 0) {
            return std::sqrt(extrema.SquareDistance(1));
        }
    } catch (...) {
    }

    Bnd_Box bbox;
    BRepBndLib::Add(m_stockShape, bbox);
    Standard_Real xMin, yMin, zMin, xMax, yMax, zMax;
    bbox.Get(xMin, yMin, zMin, xMax, yMax, zMax);

    double dz = zMin - point.Z();
    if (point.X() >= xMin && point.X() <= xMax &&
        point.Y() >= yMin && point.Y() <= yMax) {
        return dz;
    }

    return 1e10;
}

TopoDS_Face GougeDetector::findNearestFace(const gp_Pnt& point) {
    if (m_stockShape.IsNull()) {
        return TopoDS_Face();
    }

    TopoDS_Face nearestFace;
    double minDist = 1e10;

    TopExp_Explorer expFace(m_stockShape, TopAbs_FACE);
    while (expFace.More()) {
        TopoDS_Face face = TopoDS::Face(expFace.Current());

        BRepExtrema_ExtPN extrema(BRepBuilderAPI_MakeVertex(point).Vertex(), face);
        if (extrema.IsDone() && extrema.NbSolution() > 0) {
            double dist = std::sqrt(extrema.SquareDistance(1));
            if (dist < minDist) {
                minDist = dist;
                nearestFace = face;
            }
        }

        expFace.Next();
    }

    return nearestFace;
}

double GougeDetector::calculateGougeDepth(const gp_Pnt& point, const TopoDS_Face& face) {
    if (face.IsNull()) {
        return 0.0;
    }

    BRepAdaptor_Surface adaptor(face);
    if (adaptor.GetType() != GeomAbs_Plane) {
        return 0.0;
    }

    Handle(Geom_Plane) plane = adaptor.Plane();
    if (plane.IsNull()) {
        return 0.0;
    }

    gp_Pnt p1, p2;
    adaptor.D0(adaptor.FirstUParameter(), adaptor.FirstVParameter(), p1);
    adaptor.D0(adaptor.LastUParameter(), adaptor.FirstVParameter(), p2);

    gp_Vec vec(p1, p2);
    vec.Normalize();

    gp_Pnt projected = plane->Project(point);
    double depth = projected.Distance(point);

    return depth;
}

GougeSeverity GougeDetector::evaluateGougeSeverity(double depth) const {
    if (depth < 0.1) {
        return GougeSeverity::Minor;
    } else if (depth < 0.5) {
        return GougeSeverity::Moderate;
    } else {
        return GougeSeverity::Severe;
    }
}

std::string GougeDetector::getGougeDescription(GougeSeverity severity) const {
    switch (severity) {
        case GougeSeverity::Minor:
            return "轻微过切";
        case GougeSeverity::Moderate:
            return "中等过切";
        case GougeSeverity::Severe:
            return "严重过切";
        default:
            return "未知";
    }
}

void GougeDetector::clear() {
    m_gouges.clear();
}

}
