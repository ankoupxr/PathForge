#include "ContourMillingStrategy.h"

#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>

#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>

#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Circ.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>

namespace PathForge::Path {

ContourMillingStrategy::ContourMillingStrategy() = default;

ContourMillingStrategy::ContourMillingStrategy(const PathStrategyContext& context) : PathStrategy(context) {}

StrategyType ContourMillingStrategy::getType() const {
    return StrategyType::ContourMilling;
}

std::string ContourMillingStrategy::getName() const {
    return "Contour Milling";
}

std::string ContourMillingStrategy::getDescription() const {
    return "轮廓铣削策略 - 用于侧壁和外形轮廓精加工";
}

bool ContourMillingStrategy::validate() const {
    if (m_context.getBoundaryWire().IsNull()) {
        m_lastError = "边界线无效";
        return false;
    }
    return true;
}

ToolpathPtr ContourMillingStrategy::generate() {
    auto toolpath = std::make_shared<Toolpath>("Contour Milling-" + getName());

    const auto& wire = m_context.getBoundaryWire();

    double toolRadius = m_context.getToolDiameter() / 2.0;
    double offset = toolRadius + m_finishStock;

    double depth = m_context.getModelTop();
    double totalDepth = m_context.getDepth();
    double safeZ = m_context.getSafeZ();

    if (m_allowMultiplePasses) {
        double currentZ = depth + m_stepDepth;

        while (currentZ > depth - totalDepth) {
            currentZ = std::max(depth - totalDepth, currentZ - m_stepDepth);

            PathPoint safePoint;
            safePoint.position = gp_Pnt(0, 0, safeZ);
            safePoint.moveType = MoveType::Rapid;
            safePoint.feedrate = 5000;
            toolpath->addPoint(safePoint);

            PathPoint clearPoint;
            clearPoint.position = gp_Pnt(0, 0, currentZ + 5.0);
            clearPoint.moveType = MoveType::Linear;
            clearPoint.motionType = MotionType::LeadIn;
            clearPoint.feedrate = m_context.getFeedrate();
            toolpath->addPoint(clearPoint);

            auto roughingPath = generateRoughingPass(wire);
            if (roughingPath) {
                for (const auto& pt : roughingPath->points()) {
                    PathPoint newPt = pt;
                    newPt.position = gp_Pnt(pt.position.X(), pt.position.Y(), currentZ);
                    newPt.motionType = MotionType::Cutting;
                    newPt.feedrate = m_context.getFeedrate();
                    toolpath->addPoint(newPt);
                }
            }

            PathPoint retractPoint;
            retractPoint.position = gp_Pnt(0, 0, currentZ + 5.0);
            retractPoint.moveType = MoveType::Rapid;
            retractPoint.motionType = MotionType::Lift;
            retractPoint.feedrate = 5000;
            toolpath->addPoint(retractPoint);
        }
    }

    if (m_finishPassEnabled) {
        PathPoint safePoint;
        safePoint.position = gp_Pnt(0, 0, safeZ);
        safePoint.moveType = MoveType::Rapid;
        safePoint.motionType = MotionType::LeadIn;
        safePoint.feedrate = 5000;
        toolpath->addPoint(safePoint);

        PathPoint startPoint;
        startPoint.position = gp_Pnt(0, 0, depth - totalDepth);
        startPoint.moveType = MoveType::Linear;
        startPoint.motionType = MotionType::LeadIn;
        startPoint.feedrate = m_context.getFeedrate();
        toolpath->addPoint(startPoint);

        auto finishPath = generateFinishPass(wire);
        if (finishPath) {
            toolpath->addPoints(finishPath->points());
        }

        PathPoint retractPoint;
        retractPoint.position = gp_Pnt(0, 0, safeZ);
        retractPoint.moveType = MoveType::Rapid;
        retractPoint.motionType = MotionType::LeadOut;
        retractPoint.feedrate = 5000;
        toolpath->addPoint(retractPoint);
    }

    return toolpath;
}

void ContourMillingStrategy::setCompensation(ContourCompensation comp) {
    m_compensation = comp;
}

ContourCompensation ContourMillingStrategy::getCompensation() const {
    return m_compensation;
}

void ContourMillingStrategy::setContourSide(ContourSide side) {
    m_contourSide = side;
}

ContourSide ContourMillingStrategy::getContourSide() const {
    return m_contourSide;
}

void ContourMillingStrategy::setAllowMultiplePasses(bool enabled) {
    m_allowMultiplePasses = enabled;
}

bool ContourMillingStrategy::isAllowMultiplePassesEnabled() const {
    return m_allowMultiplePasses;
}

void ContourMillingStrategy::setStepDepth(double depth) {
    m_stepDepth = depth;
}

double ContourMillingStrategy::getStepDepth() const {
    return m_stepDepth;
}

void ContourMillingStrategy::setFinishPassEnabled(bool enabled) {
    m_finishPassEnabled = enabled;
}

bool ContourMillingStrategy::isFinishPassEnabled() const {
    return m_finishPassEnabled;
}

void ContourMillingStrategy::setFinishStock(double stock) {
    m_finishStock = stock;
}

double ContourMillingStrategy::getFinishStock() const {
    return m_finishStock;
}

void ContourMillingStrategy::setCornerRadius(double radius) {
    m_cornerRadius = radius;
}

double ContourMillingStrategy::getCornerRadius() const {
    return m_cornerRadius;
}

void ContourMillingStrategy::setSmoothCorners(bool enabled) {
    m_smoothCorners = enabled;
}

bool ContourMillingStrategy::isSmoothCornersEnabled() const {
    return m_smoothCorners;
}

void ContourMillingStrategy::setMergeEnabled(bool enabled) {
    m_mergeEnabled = enabled;
}

bool ContourMillingStrategy::isMergeEnabled() const {
    return m_mergeEnabled;
}

ToolpathPtr ContourMillingStrategy::generateRoughingPass(const TopoDS_Wire& boundary) {
    auto toolpath = std::make_shared<Toolpath>("Contour Roughing");

    Bnd_Box bbox;
    BRepBndLib::Add(boundary, bbox);
    Standard_Real xMin, yMin, zMin, xMax, yMax, zMax;
    bbox.Get(xMin, yMin, zMin, xMax, yMax, zMax);

    auto points = extractWirePoints(boundary);

    double toolRadius = m_context.getToolDiameter() / 2.0;
    double roughingOffset = toolRadius + m_finishStock + 0.5;

    for (const auto& pt : points) {
        PathPoint point;
        point.position = pt;
        point.moveType = MoveType::Linear;
        point.motionType = MotionType::Cutting;
        point.feedrate = m_context.getFeedrate();
        point.spindleSpeed = 3000;
        toolpath->addPoint(point);
    }

    if (!points.empty()) {
        toolpath->addPoint(points.front());
    }

    return toolpath;
}

ToolpathPtr ContourMillingStrategy::generateFinishPass(const TopoDS_Wire& boundary) {
    auto toolpath = std::make_shared<Toolpath>("Contour Finish");

    auto points = extractWirePoints(boundary);

    double toolRadius = m_context.getToolDiameter() / 2.0;
    double finishOffset = toolRadius;

    for (const auto& pt : points) {
        PathPoint point;
        point.position = pt;
        point.moveType = MoveType::Linear;
        point.motionType = MotionType::Cutting;
        point.feedrate = m_context.getFeedrate() * 0.8;
        point.spindleSpeed = 3000;
        toolpath->addPoint(point);
    }

    if (!points.empty()) {
        toolpath->addPoint(points.front());
    }

    return toolpath;
}

std::vector<gp_Pnt> ContourMillingStrategy::extractWirePoints(const TopoDS_Wire& wire) {
    std::vector<gp_Pnt> points;

    TopExp_Explorer expEdge(wire, TopAbs_EDGE);
    while (expEdge.More()) {
        TopoDS_Edge edge = TopoDS::Edge(expEdge.Current());

        BRepAdaptor_Curve curve(edge);
        double uStart = curve.FirstParameter();
        double uEnd = curve.LastParameter();

        int numSamples = 20;
        for (int i = 0; i <= numSamples; ++i) {
            double u = uStart + (uEnd - uStart) * i / numSamples;
            gp_Pnt pt = curve.Value(u);
            points.push_back(pt);
        }

        expEdge.Next();
    }

    return points;
}

gp_Pnt ContourMillingStrategy::offsetPoint(const gp_Pnt& point, const gp_Dir& normal, double offset) {
    gp_Vec offsetVec(normal.X() * offset, normal.Y() * offset, normal.Z() * offset);
    return gp_Pnt(point.X() + offsetVec.X(), point.Y() + offsetVec.Y(), point.Z() + offsetVec.Z());
}

}
