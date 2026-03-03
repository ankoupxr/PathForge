#include "TwoDFaceMillingStrategy.h"
#include "PathStrategy.h"

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

namespace PathForge {
namespace Path {

TwoDFaceMillingStrategy::TwoDFaceMillingStrategy()
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
    if (m_context.getBoundaryWire().IsNull()) {
        m_lastError = "Boundary wire is null";
        return false;
    }

    if (m_context.getStepover() <= 0) {
        m_lastError = "Stepover must be positive";
        return false;
    }

    if (m_context.getDepth() < 0) {
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

    const auto& boundary = m_context.getBoundaryWire();
    double angleRad = m_context.getCuttingAngle() * M_PI / 180.0;
    double effectiveStepover = m_context.getStepover();
    
    if (m_toolRadiusCompensation) {
        effectiveStepover = m_context.getStepover() - m_context.getToolDiameter() * 0.4;
        if (effectiveStepover <= 0) {
            effectiveStepover = m_context.getStepover();
        }
    }

    BoundingBox bbox = computeBoundingBox(boundary);
    std::vector<Point2D> boundaryPoints = extractPoints2D(boundary);

    double depth = m_context.getDepth();
    double currentZ = m_context.getStockTop();
    double modelTop = m_context.getModelTop();

    while (currentZ > modelTop) {
        double passDepth = std::min(m_stepDown, currentZ - modelTop);
        currentZ -= passDepth;
        double cuttingZ = currentZ;

        std::vector<Point2D> lines;
        
        if (m_context.getCuttingDirection() == CuttingDirection::Zigzag) {
            lines = generateZigzagLines(bbox, effectiveStepover, angleRad);
        } else {
            bool forward = true;
            lines = generateOneDirectionLines(bbox, effectiveStepover, angleRad, forward);
            if (m_context.getCuttingDirection() == CuttingDirection::Reverse) {
                std::reverse(lines.begin(), lines.end());
            }
        }

        auto clippedLines = clipLinesToBoundary(lines, boundary);

        // clippedLines is a vector of Point2D, where each pair of points forms a line segment
        for (size_t i = 0; i + 1 < clippedLines.size(); i += 2) {
            const auto& p1 = clippedLines[i];
            const auto& p2 = clippedLines[i + 1];
            
            PathPoint pathPt1(gp_Pnt(p1.x, p1.y, cuttingZ), MoveType::Linear);
            pathPt1.feedrate = m_context.getFeedrate();
            pathPt1.motionType = MotionType::Cutting;
            toolpath->addPoint(pathPt1);

            PathPoint pathPt2(gp_Pnt(p2.x, p2.y, cuttingZ), MoveType::Linear);
            pathPt2.feedrate = m_context.getFeedrate();
            pathPt2.motionType = MotionType::Cutting;
            toolpath->addPoint(pathPt2);
        }
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
    bbox.minX = bbox.minY = 1e10;
    bbox.maxX = bbox.maxY = -1e10;

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
        }
        explorer.Next();
    }

    return bbox;
}

std::vector<TwoDFaceMillingStrategy::Point2D> TwoDFaceMillingStrategy::extractPoints2D(const TopoDS_Wire& wire)
{
    std::vector<Point2D> points;

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
                points.push_back(Point2D(p1.X(), p1.Y()));
                points.push_back(Point2D(p2.X(), p2.Y()));
            }
            else if (baseCurve->DynamicType() == STANDARD_TYPE(Geom_Circle)) {
                int numSegments = 36;
                for (int i = 0; i <= numSegments; ++i) {
                    double t = first + (last - first) * i / numSegments;
                    gp_Pnt p = curve->Value(t);
                    points.push_back(Point2D(p.X(), p.Y()));
                }
            }
            else if (baseCurve->DynamicType() == STANDARD_TYPE(Geom_BSplineCurve)) {
                int numSegments = 20;
                for (int i = 0; i <= numSegments; ++i) {
                    double t = first + (last - first) * i / numSegments;
                    gp_Pnt p = curve->Value(t);
                    points.push_back(Point2D(p.X(), p.Y()));
                }
            }
            else {
                gp_Pnt p1 = curve->Value(first);
                gp_Pnt p2 = curve->Value(last);
                points.push_back(Point2D(p1.X(), p1.Y()));
                points.push_back(Point2D(p2.X(), p2.Y()));
            }
        }
        explorer.Next();
    }
    
    if (points.size() > 1) {
        std::vector<Point2D> uniquePoints;
        uniquePoints.push_back(points[0]);
        double tolerance = m_context.getTolerance();
        for (size_t i = 1; i < points.size(); ++i) {
            double dx = points[i].x - uniquePoints.back().x;
            double dy = points[i].y - uniquePoints.back().y;
            if (std::sqrt(dx*dx + dy*dy) > tolerance) {
                uniquePoints.push_back(points[i]);
            }
        }
        return uniquePoints;
    }
    
    return points;
}

std::vector<TwoDFaceMillingStrategy::Point2D> TwoDFaceMillingStrategy::generateZigzagLines(
    const BoundingBox& bbox, double stepover, double angle)
{
    std::vector<Point2D> lines;
    
    double diag = std::sqrt(std::pow(bbox.maxX - bbox.minX, 2) + std::pow(bbox.maxY - bbox.minY, 2));
    double extend = diag * 0.1;
    
    double cosA = std::cos(angle);
    double sinA = std::sin(angle);
    
    double ux = cosA;
    double uy = sinA;
    double vx = -sinA;
    double vy = cosA;
    
    double vMin = -extend;
    double vMax = (bbox.maxX - bbox.minX) * std::abs(vx) + (bbox.maxY - bbox.minY) * std::abs(vy) + extend;
    
    for (double v = vMin; v <= vMax; v += stepover) {
        Point2D p1, p2;
        
        if (std::abs(sinA) > 0.001) {
            double t1 = (bbox.minX - extend - v * vx) / ux;
            double t2 = (bbox.maxX + extend - v * vx) / ux;
            
            p1 = Point2D(bbox.minX - extend + ux * t1, v + vx * t1);
            p2 = Point2D(bbox.minX - extend + ux * t2, v + vx * t2);
        } else {
            double t1 = (bbox.minY - extend - v * vy) / uy;
            double t2 = (bbox.maxY + extend - v * vy) / uy;
            
            p1 = Point2D(v + vx * t1, bbox.minY - extend + uy * t1);
            p2 = Point2D(v + vx * t2, bbox.minY - extend + uy * t2);
        }
        
        lines.push_back(p1);
        lines.push_back(p2);
    }
    
    return lines;
}

std::vector<TwoDFaceMillingStrategy::Point2D> TwoDFaceMillingStrategy::generateOneDirectionLines(
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

std::vector<TwoDFaceMillingStrategy::Point2D> TwoDFaceMillingStrategy::clipLinesToBoundary(
    const std::vector<Point2D>& lines, const TopoDS_Wire& boundary)
{
    std::vector<Point2D> boundaryPoints = extractPoints2D(boundary);
    
    if (boundaryPoints.empty()) {
        return lines;
    }
    
    std::vector<Point2D> clippedLines;
    double tolerance = m_context.getTolerance();
    
    for (size_t i = 0; i + 1 < lines.size(); i += 2) {
        Point2D p1 = lines[i];
        Point2D p2 = lines[i + 1];
        
        if (pointInPolygon(p1, boundaryPoints) && pointInPolygon(p2, boundaryPoints)) {
            clippedLines.push_back(p1);
            clippedLines.push_back(p2);
        } else if (pointInPolygon(p1, boundaryPoints) || pointInPolygon(p2, boundaryPoints)) {
            clippedLines.push_back(p1);
            clippedLines.push_back(p2);
        }
    }
    
    return clippedLines;
}

TwoDFaceMillingStrategy::Point2D TwoDFaceMillingStrategy::lineIntersection(
    const Point2D& p1, const Point2D& p2,
    const Point2D& p3, const Point2D& p4)
{
    double denom = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x);
    
    if (std::abs(denom) < 1e-10) {
        return p1;
    }
    
    double t = ((p1.x - p3.x) * (p3.y - p4.y) - (p1.y - p3.y) * (p3.x - p4.x)) / denom;
    
    return Point2D(
        p1.x + t * (p2.x - p1.x),
        p1.y + t * (p2.y - p1.y)
    );
}

bool TwoDFaceMillingStrategy::pointInPolygon(
    const Point2D& p, const std::vector<Point2D>& polygon)
{
    if (polygon.size() < 3) return false;
    
    int crossings = 0;
    size_t n = polygon.size();
    
    for (size_t i = 0; i < n; ++i) {
        const Point2D& p1 = polygon[i];
        const Point2D& p2 = polygon[(i + 1) % n];
        
        if (((p1.y <= p.y && p.y < p2.y) || (p2.y <= p.y && p.y < p1.y)) &&
            (p.x < (p2.x - p1.x) * (p.y - p1.y) / (p2.y - p1.y) + p1.x)) {
            crossings = 1 - crossings;
        }
    }
    
    return crossings == 1;
}

void TwoDFaceMillingStrategy::addLeadInOut(
    PathPoint& startPoint, PathPoint& endPoint,
    const Point2D& direction, bool addLeadIn, bool addLeadOut)
{
    if (addLeadIn && m_context.isLeadInEnabled()) {
        double length = m_context.getLeadInLength();
        gp_Vec leadInVec(-direction.x * length, -direction.y * length, 0);

        gp_XYZ newStartPos = startPoint.position.XYZ() - leadInVec.XYZ();
        startPoint.position.SetX(newStartPos.X());
        startPoint.position.SetY(newStartPos.Y());
        startPoint.position.SetZ(newStartPos.Z());
        startPoint.motionType = MotionType::LeadIn;
    }

    if (addLeadOut && m_context.isLeadOutEnabled()) {
        double length = m_context.getLeadOutLength();
        gp_Vec leadOutVec(direction.x * length, direction.y * length, 0);

        gp_XYZ newEndPos = endPoint.position.XYZ() + leadOutVec.XYZ();
        endPoint.position.SetX(newEndPos.X());
        endPoint.position.SetY(newEndPos.Y());
        endPoint.position.SetZ(newEndPos.Z());
        endPoint.motionType = MotionType::LeadOut;
    }
}

}
}
