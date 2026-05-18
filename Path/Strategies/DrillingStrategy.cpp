#include "DrillingStrategy.h"

#include "../StrategyFactory.h"

#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <Geom_Surface.hxx>
#include <Geom_CylindricalSurface.hxx>

#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>

#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Circ.hxx>

#include <algorithm>
#include <cmath>

namespace PathForge::Path {

DrillingStrategy::DrillingStrategy() = default;

DrillingStrategy::DrillingStrategy(const PathStrategyContext& context) : PathStrategy(context) {}

StrategyType DrillingStrategy::getType() const {
    return StrategyType::DrillCenter;
}

std::string DrillingStrategy::getName() const {
    return "Drilling";
}

std::string DrillingStrategy::getDescription() const {
    return "钻孔加工策略 - 支持啄钻、深孔钻、铰孔、镗孔等";
}

bool DrillingStrategy::validate() const {
    if (m_context.getBoundaryFace().IsNull()) {
        m_lastError = "钻孔面无效";
        return false;
    }
    return true;
}

ToolpathPtr DrillingStrategy::generate() {
    auto toolpath = std::make_shared<Toolpath>("Drilling-" + getName());

    const auto& face = m_context.getBoundaryFace();
    BRepAdaptor_Surface adaptor(face);

    if (adaptor.GetType() != GeomAbs_Cylinder) {
        m_lastError = "当前仅支持圆柱面孔";
        return nullptr;
    }

    GProp_GProps props;
    BRepGProp::SurfaceProperties(face, props);
    gp_Pnt centroid = props.CentreOfMass();

    BRep_Tool::Surface(face);
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
    Handle(Geom_CylindricalSurface) cylinder = Handle(Geom_CylindricalSurface)::DownCast(surface);

    if (cylinder.IsNull()) {
        m_lastError = "无法获取圆柱面";
        return nullptr;
    }

    double safeZ = m_context.getSafeZ();
    double clearance = 2.0;
    double drillDepth = m_context.getDepth();

    if (drillDepth <= 0) {
        drillDepth = 10.0;
    }

    switch (m_drillPattern) {
        case DrillPattern::啄钻:
        {
            auto path = generatePeckDrilling(safeZ, drillDepth, m_peckDepth);
            if (path) toolpath->addPoints(path->points());
            break;
        }
        case DrillPattern::深孔钻:
        {
            auto path = generateDeepDrilling(safeZ, drillDepth, m_peckDepth);
            if (path) toolpath->addPoints(path->points());
            break;
        }
        case DrillPattern::中心钻:
        {
            auto path = generateCenterDrilling(safeZ, drillDepth);
            if (path) toolpath->addPoints(path->points());
            break;
        }
        default:
        {
            auto path = generateNormalDrilling(safeZ, drillDepth);
            if (path) toolpath->addPoints(path->points());
            break;
        }
    }

    toolpath->setStartPoint(gp_Pnt(centroid.X(), centroid.Y(), safeZ));
    toolpath->setEndPoint(gp_Pnt(centroid.X(), centroid.Y(), safeZ));
    toolpath->setDepth(drillDepth);

    return toolpath;
}

void DrillingStrategy::setDrillPattern(DrillPattern pattern) {
    m_drillPattern = pattern;
}

DrillPattern DrillingStrategy::getDrillPattern() const {
    return m_drillPattern;
}

void DrillingStrategy::setPeckDepth(double depth) {
    m_peckDepth = depth;
}

double DrillingStrategy::getPeckDepth() const {
    return m_peckDepth;
}

void DrillingStrategy::setDwellTime(double seconds) {
    m_dwellTime = seconds;
}

double DrillingStrategy::getDwellTime() const {
    return m_dwellTime;
}

void DrillingStrategy::setRetractDistance(double distance) {
    m_retractDistance = distance;
}

double DrillingStrategy::getRetractDistance() const {
    return m_retractDistance;
}

void DrillingStrategy::setSpindleSpeed(int rpm) {
    m_spindleSpeed = rpm;
}

int DrillingStrategy::getSpindleSpeed() const {
    return m_spindleSpeed;
}

void DrillingStrategy::setDrillCycleType(const std::string& cycle) {
    m_drillCycleType = cycle;
}

const std::string& DrillingStrategy::getDrillCycleType() const {
    return m_drillCycleType;
}

ToolpathPtr DrillingStrategy::generatePeckDrilling(double startZ, double depth, double peckDepth) {
    auto toolpath = std::make_shared<Toolpath>("Peck Drilling");

    PathPoint safePoint;
    safePoint.position = gp_Pnt(0, 0, startZ);
    safePoint.moveType = MoveType::Rapid;
    safePoint.feedrate = 5000;
    toolpath->addPoint(safePoint);

    double currentZ = startZ - clearance;
    PathPoint retractPoint;
    retractPoint.position = gp_Pnt(0, 0, currentZ);
    retractPoint.moveType = MoveType::Rapid;
    retractPoint.feedrate = 5000;
    toolpath->addPoint(retractPoint);

    while (currentZ > startZ - depth) {
        double targetZ = std::max(startZ - depth, currentZ - peckDepth);

        PathPoint plunge;
        plunge.position = gp_Pnt(0, 0, targetZ);
        plunge.moveType = MoveType::Linear;
        plunge.motionType = MotionType::Plunge;
        plunge.feedrate = m_context.getPlungeFeedrate();
        plunge.spindleSpeed = m_spindleSpeed;
        toolpath->addPoint(plunge);

        if (m_dwellTime > 0) {
            PathPoint dwell;
            dwell.position = gp_Pnt(0, 0, targetZ);
            dwell.moveType = MoveType::Dwell;
            dwell.feedrate = m_dwellTime;
            toolpath->addPoint(dwell);
        }

        PathPoint retract;
        retract.position = gp_Pnt(0, 0, currentZ);
        retract.moveType = MoveType::Rapid;
        retract.motionType = MotionType::Lift;
        retract.feedrate = 5000;
        toolpath->addPoint(retract);

        currentZ -= peckDepth;
    }

    PathPoint endRetract;
    endRetract.position = gp_Pnt(0, 0, startZ);
    endRetract.moveType = MoveType::Rapid;
    endRetract.motionType = MotionType::Lift;
    endRetract.feedrate = 5000;
    toolpath->addPoint(endRetract);

    return toolpath;
}

ToolpathPtr DrillingStrategy::generateDeepDrilling(double startZ, double depth, double peckDepth) {
    return generatePeckDrilling(startZ, depth, peckDepth);
}

ToolpathPtr DrillingStrategy::generateNormalDrilling(double startZ, double depth) {
    auto toolpath = std::make_shared<Toolpath>("Normal Drilling");

    PathPoint safePoint;
    safePoint.position = gp_Pnt(0, 0, startZ);
    safePoint.moveType = MoveType::Rapid;
    safePoint.feedrate = 5000;
    toolpath->addPoint(safePoint);

    PathPoint clearancePoint;
    clearancePoint.position = gp_Pnt(0, 0, startZ - 2.0);
    clearancePoint.moveType = MoveType::Linear;
    clearancePoint.feedrate = m_context.getFeedrate();
    toolpath->addPoint(clearancePoint);

    PathPoint plunge;
    plunge.position = gp_Pnt(0, 0, startZ - depth);
    plunge.moveType = MoveType::Linear;
    plunge.motionType = MotionType::Cutting;
    plunge.feedrate = m_context.getPlungeFeedrate();
    plunge.spindleSpeed = m_spindleSpeed;
    toolpath->addPoint(plunge);

    if (m_dwellTime > 0) {
        PathPoint dwell;
        dwell.position = gp_Pnt(0, 0, startZ - depth);
        dwell.moveType = MoveType::Dwell;
        dwell.feedrate = m_dwellTime;
        toolpath->addPoint(dwell);
    }

    PathPoint retract;
    retract.position = gp_Pnt(0, 0, startZ);
    retract.moveType = MoveType::Rapid;
    retract.motionType = MotionType::Lift;
    retract.feedrate = 5000;
    toolpath->addPoint(retract);

    return toolpath;
}

ToolpathPtr DrillingStrategy::generateCenterDrilling(double startZ, double depth) {
    return generateNormalDrilling(startZ, depth);
}

}
