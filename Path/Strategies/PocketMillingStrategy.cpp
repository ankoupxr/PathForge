#include "PocketMillingStrategy.h"

#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepAdaptor_Surface.hxx>

#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepExtrema_ExtPF.hxx>

#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Circ.hxx>

#include <algorithm>
#include <cmath>

namespace PathForge::Path {

PocketMillingStrategy::PocketMillingStrategy() = default;

PocketMillingStrategy::PocketMillingStrategy(const PathStrategyContext& context) : PathStrategy(context) {}

StrategyType PocketMillingStrategy::getType() const {
    return StrategyType::PocketMilling;
}

std::string PocketMillingStrategy::getName() const {
    return "Pocket Milling";
}

std::string PocketMillingStrategy::getDescription() const {
    return "型腔铣削策略 - 支持螺旋线、平行线、往复式粗加工";
}

bool PocketMillingStrategy::validate() const {
    if (m_context.getBoundaryWire().IsNull()) {
        m_lastError = "边界线无效";
        return false;
    }
    return true;
}

ToolpathPtr PocketMillingStrategy::generate() {
    auto toolpath = std::make_shared<Toolpath>("Pocket Milling-" + getName());

    const auto& wire = m_context.getBoundaryWire();

    if (m_helicalEntry) {
        gp_Pnt entryPoint = findEntryPoint(wire);

        PathPoint safePoint;
        safePoint.position = gp_Pnt(entryPoint.X(), entryPoint.Y(), m_context.getSafeZ());
        safePoint.moveType = MoveType::Rapid;
        safePoint.feedrate = 5000;
        toolpath->addPoint(safePoint);

        PathPoint clearancePoint;
        clearancePoint.position = gp_Pnt(entryPoint.X(), entryPoint.Y(), m_context.getModelTop() + m_context.getStockTop());
        clearancePoint.moveType = MoveType::Linear;
        clearancePoint.motionType = MotionType::LeadIn;
        clearancePoint.feedrate = m_context.getFeedrate();
        toolpath->addPoint(clearancePoint);

        double depth = m_context.getModelTop() + m_context.getStockTop() - m_context.getDepth();
        double currentZ = m_context.getModelTop() + m_context.getStockTop();

        while (currentZ > depth) {
            currentZ = std::max(depth, currentZ - m_stepDepth);

            double helixRadius = m_helixRadius;
            int numTurns = 5;
            double angleStep = 2.0 * M_PI / numTurns;

            for (int i = 0; i <= numTurns; ++i) {
                double angle = i * angleStep;
                double x = entryPoint.X() + helixRadius * std::cos(angle);
                double y = entryPoint.Y() + helixRadius * std::sin(angle);

                PathPoint helixPoint;
                helixPoint.position = gp_Pnt(x, y, currentZ);
                helixPoint.moveType = MoveType::Helix;
                helixPoint.motionType = MotionType::Cutting;
                helixPoint.feedrate = m_context.getFeedrate();
                helixPoint.spindleSpeed = 3000;
                toolpath->addPoint(helixPoint);
            }

            PathPoint retractPoint;
            retractPoint.position = gp_Pnt(entryPoint.X(), entryPoint.Y(), currentZ + 2.0);
            retractPoint.moveType = MoveType::Rapid;
            retractPoint.motionType = MotionType::Lift;
            retractPoint.feedrate = 5000;
            toolpath->addPoint(retractPoint);
        }
    }

    switch (m_pattern) {
        case PocketRoughingPattern::螺旋线:
        {
            auto roughingPath = generateSpiralRoughing(wire);
            if (roughingPath) toolpath->addPoints(roughingPath->points());
            break;
        }
        case PocketRoughingPattern::平行线:
        {
            auto roughingPath = generateParallelRoughing(wire);
            if (roughingPath) toolpath->addPoints(roughingPath->points());
            break;
        }
        case PocketRoughingPattern::往复式:
        {
            auto roughingPath = generateZigzagRoughing(wire);
            if (roughingPath) toolpath->addPoints(roughingPath->points());
            break;
        }
        default:
        {
            auto roughingPath = generateSpiralRoughing(wire);
            if (roughingPath) toolpath->addPoints(roughingPath->points());
            break;
        }
    }

    if (m_finishPassEnabled) {
        PathPoint finishStart;
        finishStart.position = gp_Pnt(0, 0, m_context.getSafeZ());
        finishStart.moveType = MoveType::Rapid;
        finishStart.motionType = MotionType::LeadIn;
        finishStart.feedrate = 5000;
        toolpath->addPoint(finishStart);
    }

    return toolpath;
}

void PocketMillingStrategy::setRoughingPattern(PocketRoughingPattern pattern) {
    m_pattern = pattern;
}

PocketRoughingPattern PocketMillingStrategy::getRoughingPattern() const {
    return m_pattern;
}

void PocketMillingStrategy::setRoughingClearance(double clearance) {
    m_roughingClearance = clearance;
}

double PocketMillingStrategy::getRoughingClearance() const {
    return m_roughingClearance;
}

void PocketMillingStrategy::setMinimumPocketRadius(double radius) {
    m_minimumPocketRadius = radius;
}

double PocketMillingStrategy::getMinimumPocketRadius() const {
    return m_minimumPocketRadius;
}

void PocketMillingStrategy::setHelicalEntry(bool enabled) {
    m_helicalEntry = enabled;
}

bool PocketMillingStrategy::isHelicalEntryEnabled() const {
    return m_helicalEntry;
}

void PocketMillingStrategy::setHelixRadius(double radius) {
    m_helixRadius = radius;
}

double PocketMillingStrategy::getHelixRadius() const {
    return m_helixRadius;
}

void PocketMillingStrategy::setStepDepth(double depth) {
    m_stepDepth = depth;
}

double PocketMillingStrategy::getStepDepth() const {
    return m_stepDepth;
}

void PocketMillingStrategy::setFinishPassEnabled(bool enabled) {
    m_finishPassEnabled = enabled;
}

bool PocketMillingStrategy::isFinishPassEnabled() const {
    return m_finishPassEnabled;
}

void PocketMillingStrategy::setFinishStepover(double stepover) {
    m_finishStepover = stepover;
}

double PocketMillingStrategy::getFinishStepover() const {
    return m_finishStepover;
}

ToolpathPtr PocketMillingStrategy::generateSpiralRoughing(const TopoDS_Wire& boundary) {
    auto toolpath = std::make_shared<Toolpath>("Spiral Roughing");

    Bnd_Box bbox;
    BRepBndLib::Add(boundary, bbox);
    Standard_Real xMin, yMin, zMin, xMax, yMax, zMax;
    bbox.Get(xMin, yMin, zMin, xMax, yMax, zMax);

    double centerX = (xMin + xMax) / 2.0;
    double centerY = (yMin + yMax) / 2.0;
    double maxRadius = std::max((xMax - xMin) / 2.0, (yMax - yMin) / 2.0);

    double stepover = m_context.getStepover();
    double startRadius = m_helixRadius;
    double endRadius = maxRadius - m_roughingClearance;

    auto spiralPoints = generateSpiralPath(centerX, centerY, startRadius, endRadius, stepover, 1);

    double depth = m_context.getModelTop() - m_context.getDepth();
    double currentZ = m_context.getModelTop();

    while (currentZ > depth) {
        currentZ = std::max(depth, currentZ - m_stepDepth);

        for (size_t i = 0; i < spiralPoints.size(); ++i) {
            const auto& pt = spiralPoints[i];
            PathPoint point;
            point.position = gp_Pnt(pt.X(), pt.Y(), currentZ);
            point.moveType = MoveType::Linear;
            point.motionType = MotionType::Cutting;
            point.feedrate = m_context.getFeedrate();
            point.spindleSpeed = 3000;
            toolpath->addPoint(point);
        }

        PathPoint retractPoint;
        retractPoint.position = gp_Pnt(centerX, centerY, currentZ + 2.0);
        retractPoint.moveType = MoveType::Rapid;
        retractPoint.motionType = MotionType::Lift;
        retractPoint.feedrate = 5000;
        toolpath->addPoint(retractPoint);
    }

    return toolpath;
}

ToolpathPtr PocketMillingStrategy::generateParallelRoughing(const TopoDS_Wire& boundary) {
    auto toolpath = std::make_shared<Toolpath>("Parallel Roughing");

    Bnd_Box bbox;
    BRepBndLib::Add(boundary, bbox);
    Standard_Real xMin, yMin, zMin, xMax, yMax, zMax;
    bbox.Get(xMin, yMin, zMin, xMax, yMax, zMax);

    double stepover = m_context.getStepover();
    double angle = m_context.getCuttingAngle();

    double lengthY = yMax - yMin;
    int numLines = static_cast<int>(lengthY / stepover) + 1;

    double depth = m_context.getModelTop() - m_context.getDepth();
    double currentZ = m_context.getModelTop();

    while (currentZ > depth) {
        currentZ = std::max(depth, currentZ - m_stepDepth);

        for (int i = 0; i <= numLines; ++i) {
            double y = yMin + i * stepover;
            bool forward = (i % 2 == 0);

            double x1 = forward ? xMin : xMax;
            double x2 = forward ? xMax : xMin;

            PathPoint point1;
            point1.position = gp_Pnt(x1, y, currentZ);
            point1.moveType = MoveType::Linear;
            point1.motionType = MotionType::Cutting;
            point1.feedrate = m_context.getFeedrate();
            toolpath->addPoint(point1);

            PathPoint point2;
            point2.position = gp_Pnt(x2, y, currentZ);
            point2.moveType = MoveType::Linear;
            point2.motionType = MotionType::Cutting;
            point2.feedrate = m_context.getFeedrate();
            toolpath->addPoint(point2);
        }

        PathPoint retractPoint;
        retractPoint.position = gp_Pnt((xMin + xMax) / 2.0, yMax, currentZ + 2.0);
        retractPoint.moveType = MoveType::Rapid;
        retractPoint.motionType = MotionType::Lift;
        retractPoint.feedrate = 5000;
        toolpath->addPoint(retractPoint);
    }

    return toolpath;
}

ToolpathPtr PocketMillingStrategy::generateZigzagRoughing(const TopoDS_Wire& boundary) {
    return generateParallelRoughing(boundary);
}

std::vector<gp_Pnt> PocketMillingStrategy::generateSpiralPath(double centerX, double centerY,
                                                              double startRadius, double endRadius,
                                                              double stepover, int direction) {
    std::vector<gp_Pnt> points;

    double currentRadius = startRadius;
    double angle = 0.0;
    double angleStep = 0.1;

    while (currentRadius < endRadius) {
        double x = centerX + currentRadius * std::cos(angle);
        double y = centerY + currentRadius * std::sin(angle);
        points.push_back(gp_Pnt(x, y, 0));

        angle += direction * angleStep;
        currentRadius += stepover * angleStep / (2.0 * M_PI);
    }

    return points;
}

gp_Pnt PocketMillingStrategy::findEntryPoint(const TopoDS_Wire& wire) {
    Bnd_Box bbox;
    BRepBndLib::Add(wire, bbox);
    Standard_Real xMin, yMin, zMin, xMax, yMax, zMax;
    bbox.Get(xMin, yMin, zMin, xMax, yMax, zMax);

    return gp_Pnt(xMin, (yMin + yMax) / 2.0, zMax);
}

double PocketMillingStrategy::calculatePocketArea(const TopoDS_Wire& wire) {
    GProp_GProps props;
    BRepGProp::LinearProperties(wire, props);
    return 0.0;
}

}
