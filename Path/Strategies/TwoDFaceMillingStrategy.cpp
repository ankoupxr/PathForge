#include "TwoDFaceMillingStrategy.h"

#include <TopoDS.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Edge.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs.hxx>
#include <BRep_Tool.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Line.hxx>
#include <Geom_Circle.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Circ.hxx>
#include <Standard_Integer.hxx>

#include <cmath>
#include <algorithm>
#include <numeric>
#include <BRepOffsetAPI_MakeOffset.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <Geom_Plane.hxx>
#include <gp_Pln.hxx>
#include <gp_Ax3.hxx>

namespace PathForge {
namespace Path {

TwoDFaceMillingStrategy::TwoDFaceMillingStrategy()
    : PathStrategy(PathStrategyContext())
{
}

TwoDFaceMillingStrategy::TwoDFaceMillingStrategy(const PathStrategyContext& context)
    : PathStrategy(context)
{
}

StrategyType TwoDFaceMillingStrategy::getType() const
{
    return StrategyType::FaceMilling2D;
}

std::string TwoDFaceMillingStrategy::getName() const
{
    return "2D Face Milling";
}

std::string TwoDFaceMillingStrategy::getDescription() const
{
    return "2D Face milling strategy - parallel linear tool paths for planar surface roughing";
}

bool TwoDFaceMillingStrategy::validate() const
{
    if (getContext().getBoundaryWire().IsNull()) {
        m_lastError = "Boundary wire is null";
        return false;
    }

    if (getContext().getStepover() <= 0) {
        m_lastError = "Stepover must be positive";
        return false;
    }

    if (getContext().getDepth() < 0) {
        m_lastError = "Depth must be non-negative (stock top must be above model top)";
        return false;
    }

    return true;
}

ToolpathPtr TwoDFaceMillingStrategy::generate()
{
    if (!validate()) {
        return nullptr;
    }

    auto toolpath = std::make_shared<Toolpath>("2D Face Milling");

    const auto& boundary = getContext().getBoundaryWire();
    double angleRad = getContext().getCuttingAngle() * M_PI / 180.0;
    double effectiveStepover = getContext().getStepover();

    if (m_toolRadiusCompensation) {
        effectiveStepover = getContext().getStepover() - getContext().getToolDiameter() * 0.4;
        if (effectiveStepover <= 0) {
            effectiveStepover = getContext().getStepover();
        }
    }

    BRepOffsetAPI_MakeOffset offsetMaker(boundary, GeomAbs_Arc);
    offsetMaker.Perform(5.0);
    TopoDS_Shape offsetShape = offsetMaker.Shape();
    m_offsetWire = TopoDS::Wire(offsetShape);

    BoundingBox bbox = computeBoundingBox(boundary);

    double depth = getContext().getDepth();
    double currentZ = getContext().getStockTop();
    double modelTop = getContext().getModelTop();

    std::vector<gp_Pnt> lines;

    if (getContext().getCuttingDirection() == CuttingDirection::Zigzag) {
        lines = generateZigzagLines(bbox, effectiveStepover, angleRad);
    }
    else {
        bool forward = true;
        lines = generateOneDirectionLines(bbox, effectiveStepover, angleRad, forward);
        if (getContext().getCuttingDirection() == CuttingDirection::Reverse) {
            std::reverse(lines.begin(), lines.end());
        }
    }


    for (size_t i = 0; i < lines.size(); i++) {
        const auto& p1 = lines[i];
        PathPoint pathPt1(gp_Pnt(p1.X(), p1.Y(), p1.Z()), MoveType::Linear);
        pathPt1.feedrate = getContext().getFeedrate();
        pathPt1.motionType = MotionType::Cutting;
        toolpath->addPoint(pathPt1);
    }

    return toolpath;
}

void TwoDFaceMillingStrategy::setToolRadiusCompensation(bool enabled)
{
    m_toolRadiusCompensation = enabled;
}

bool TwoDFaceMillingStrategy::isToolRadiusCompensationEnabled() const
{
    return m_toolRadiusCompensation;
}

void TwoDFaceMillingStrategy::setOverlap(double overlap)
{
    m_overlap = overlap;
}

double TwoDFaceMillingStrategy::getOverlap() const
{
    return m_overlap;
}

void TwoDFaceMillingStrategy::setMultiplePasses(bool enabled)
{
    m_multiplePasses = enabled;
}

bool TwoDFaceMillingStrategy::isMultiplePassesEnabled() const
{
    return m_multiplePasses;
}

void TwoDFaceMillingStrategy::setStepDown(double stepDown)
{
    m_stepDown = stepDown;
}

double TwoDFaceMillingStrategy::getStepDown() const
{
    return m_stepDown;
}

TwoDFaceMillingStrategy::BoundingBox TwoDFaceMillingStrategy::computeBoundingBox(const TopoDS_Wire& wire)
{
    BoundingBox bbox;
    bbox.minX = bbox.minY = bbox.minZ = 1e10;
    bbox.maxX = bbox.maxY = bbox.maxZ = -1e10;

    TopExp_Explorer explorer(wire, TopAbs_EDGE);
    while (explorer.More()) {
        const TopoDS_Edge& edge = TopoDS::Edge(explorer.Current());
        Standard_Real first, last;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);

        if (!curve.IsNull()) {
            gp_Pnt p1 = curve->Value(first);
            gp_Pnt p2 = curve->Value(last);

            bbox.minX = std::min({bbox.minX, p1.X(), p2.X()});
            bbox.maxX = std::max({bbox.maxX, p1.X(), p2.X()});
            bbox.minY = std::min({bbox.minY, p1.Y(), p2.Y()});
            bbox.maxY = std::max({bbox.maxY, p1.Y(), p2.Y()});
            bbox.minZ = std::min({bbox.minZ, p1.Z(), p2.Z()});
            bbox.maxZ = std::max({bbox.maxZ, p1.Z(), p2.Z()});
        }
        explorer.Next();
    }

    return bbox;
}

std::vector<gp_Pnt> TwoDFaceMillingStrategy::extractPoints2D(const TopoDS_Wire& wire)
{
    std::vector<gp_Pnt> points;

    TopExp_Explorer explorer(wire, TopAbs_EDGE);
    while (explorer.More()) {
        const TopoDS_Edge& edge = TopoDS::Edge(explorer.Current());
        Standard_Real first, last;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);

        if (!curve.IsNull()) {
            Handle(Geom_Curve) baseCurve = curve;
            if (curve->DynamicType() == STANDARD_TYPE(Geom_TrimmedCurve)) {
                Handle(Geom_TrimmedCurve) trimmed =
                    Handle(Geom_TrimmedCurve)::DownCast(curve);
                baseCurve = trimmed->BasisCurve();
            }

            if (baseCurve->DynamicType() == STANDARD_TYPE(Geom_Line)) {
                gp_Pnt p1 = curve->Value(first);
                gp_Pnt p2 = curve->Value(last);
                points.push_back(p1);
                points.push_back(p2);
            }
            else if (baseCurve->DynamicType() == STANDARD_TYPE(Geom_Circle)) {
                int numSegments = 36;
                for (int i = 0; i <= numSegments; ++i) {
                    double t = first + (last - first) * i / numSegments;
                    gp_Pnt p = curve->Value(t);
                    points.push_back(p);
                }
            }
            else if (baseCurve->DynamicType() == STANDARD_TYPE(Geom_BSplineCurve)) {
                int numSegments = 20;
                for (int i = 0; i <= numSegments; ++i) {
                    double t = first + (last - first) * i / numSegments;
                    gp_Pnt p = curve->Value(t);
                    points.push_back(p);
                }
            }
            else {
                gp_Pnt p1 = curve->Value(first);
                gp_Pnt p2 = curve->Value(last);
                points.push_back(p1);
                points.push_back(p2);
            }
        }
        explorer.Next();
    }

    if (points.size() > 1) {
        std::vector<gp_Pnt> uniquePoints;
        uniquePoints.push_back(points[0]);
        double tolerance = getContext().getTolerance();
        for (size_t i = 1; i < points.size(); ++i) {
            double dx = points[i].X() - uniquePoints.back().X();
            double dy = points[i].Y() - uniquePoints.back().Y();
            if (std::sqrt(dx*dx + dy*dy) > tolerance) {
                uniquePoints.push_back(points[i]);
            }
        }
        return uniquePoints;
    }

    return points;
}

std::vector<gp_Pnt> TwoDFaceMillingStrategy::generateZigzagLines(
    const BoundingBox& bbox, double stepover, double angle)
{
    std::vector<gp_Pnt> lines;

    // 使用 OCC 计算 m_offsetWire 的包围盒
    Bnd_Box offsetWireBbox;
    if (!m_offsetWire.IsNull()) {
        BRepBndLib::Add(m_offsetWire, offsetWireBbox);
    }

    Standard_Real xMin, yMin, zMin, xMax, yMax, zMax;
    offsetWireBbox.Get(xMin, yMin, zMin, xMax, yMax, zMax);


    double vMin, vMax;
    int numPasses;

    if (abs(bbox.maxZ - bbox.minZ) < 1E-10) {
        // XY 平面
        vMin = bbox.minX;
        vMax = bbox.maxX;
        numPasses = static_cast<int>((vMax - vMin) / stepover) + 1;
    }
    else if (abs(bbox.minY - bbox.maxY) < 1E-10) {
        // XZ 平面
        vMin = bbox.minZ;
        vMax = bbox.maxZ;
        numPasses = static_cast<int>((vMax - vMin) / stepover) + 1;
    }
    else if (abs(bbox.minX - bbox.maxX) < 1E-10) {
        // YZ 平面
        vMin = bbox.minY;
        vMax = bbox.maxY;
        numPasses = static_cast<int>((vMax - vMin) / stepover) + 1;
    }
    else {
        // 默认 XY 平面
        vMin = bbox.minX;
        vMax = bbox.maxX;
        numPasses = static_cast<int>((vMax - vMin) / stepover) + 1;
    }

    // 生成往复式路径
    bool forward = true;
    for (int i = 0; i < numPasses; ++i) {
        double v = vMin + i * stepover;

        gp_Pnt p1, p2;

        if (abs(bbox.maxZ - bbox.minZ) < 1E-10) 
        {
            // XY 平面
            p1 = gp_Pnt(v, yMax, bbox.minZ);
            p2 = gp_Pnt(v, yMin, bbox.minZ);
        }
        else if (abs(bbox.minY - bbox.maxY) < 1E-10) 
        {
            // XZ 平面
            p1 = gp_Pnt(xMax, bbox.maxY, v);
            p2 = gp_Pnt(xMin, bbox.maxY, v);
        }
        else if (abs(bbox.minX - bbox.maxX) < 1E-10) 
        {
            // YZ 平面
            p1 = gp_Pnt(bbox.maxX, v, zMax);
            p2 = gp_Pnt(bbox.maxX, v, zMin);
        }
        else 
        {
            // 默认 XY 平面
            p1 = gp_Pnt(v, yMax, bbox.minZ);
            p2 = gp_Pnt(v, yMin, bbox.minZ);
        }

        //if (i != 0 && i != numPasses - 1)
        //{
        //    if (forward) {
        //        lines.push_back(lines.back());
        //        lines.push_back(p2);
        //    }
        //    else {
        //        lines.push_back(lines.back());
        //        lines.push_back(p1);
        //    }
        //}

        // 根据方向添加点
        if (forward) {
            lines.push_back(p1);
            lines.push_back(p2);
        } else {
            lines.push_back(p2);
            lines.push_back(p1);
        }


        // 切换方向
        forward = !forward;
    }

    return lines;
}

std::vector<gp_Pnt> TwoDFaceMillingStrategy::generateOneDirectionLines(
    const BoundingBox& bbox, double stepover, double angle, bool forward)
{
    auto lines = generateZigzagLines(bbox, stepover, angle);

    if (!forward) {
        for (size_t i = 0; i < lines.size(); i += 2) {
            std::swap(lines[i], lines[i + 1]);
        }
    }

    return lines;
}


gp_Pnt TwoDFaceMillingStrategy::lineIntersection(
    const gp_Pnt& p1, const gp_Pnt& p2,
    const gp_Pnt& p3, const gp_Pnt& p4)
{
    double denom = (p1.X() - p2.X()) * (p3.Y() - p4.Y()) - (p1.Y() - p2.Y()) * (p3.X() - p4.X());

    if (std::abs(denom) < 1e-10) {
        return p1;
    }

    double t = ((p1.X() - p3.X()) * (p3.Y() - p4.Y()) - (p1.Y() - p3.Y()) * (p3.X() - p4.X())) / denom;

    return gp_Pnt(
        p1.X() + t * (p2.X() - p1.X()),
        p1.Y() + t * (p2.Y() - p1.Y()),
        p1.Z()
    );
}

bool TwoDFaceMillingStrategy::pointInPolygon(
    const gp_Pnt& p, const std::vector<gp_Pnt>& polygon)
{
    if (polygon.size() < 3) return false;

    int crossings = 0;
    size_t n = polygon.size();

    for (size_t i = 0; i < n; ++i) {
        const gp_Pnt& p1 = polygon[i];
        const gp_Pnt& p2 = polygon[(i + 1) % n];

        if (((p1.Y() <= p.Y() && p.Y() < p2.Y()) || (p2.Y() <= p.Y() && p.Y() < p1.Y())) &&
            (p.X() < (p2.X() - p1.X()) * (p.Y() - p1.Y()) / (p2.Y() - p1.Y()) + p1.X())) {
            crossings = 1 - crossings;
        }
    }

    return crossings == 1;
}

void TwoDFaceMillingStrategy::addLeadInOut(
    PathPoint& startPoint, PathPoint& endPoint,
    const gp_Dir& direction, bool addLeadIn, bool addLeadOut)
{
    if (addLeadIn && getContext().isLeadInEnabled()) {
        double length = getContext().getLeadInLength();
        gp_Vec leadInVec(-direction.X() * length, -direction.Y() * length, 0);

        gp_XYZ newStartPos = startPoint.position.XYZ() - leadInVec.XYZ();
        startPoint.position.SetX(newStartPos.X());
        startPoint.position.SetY(newStartPos.Y());
        startPoint.position.SetZ(newStartPos.Z());
        startPoint.motionType = MotionType::LeadIn;
    }

    if (addLeadOut && getContext().isLeadOutEnabled()) {
        double length = getContext().getLeadOutLength();
        gp_Vec leadOutVec(direction.X() * length, direction.Y() * length, 0);

        gp_XYZ newEndPos = endPoint.position.XYZ() + leadOutVec.XYZ();
        endPoint.position.SetX(newEndPos.X());
        endPoint.position.SetY(newEndPos.Y());
        endPoint.position.SetZ(newEndPos.Z());
        endPoint.motionType = MotionType::LeadOut;
    }
}

}
}
