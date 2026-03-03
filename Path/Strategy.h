#pragma once

#include <string>
#include <memory>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>

#include "Toolpath.h"

namespace PathForge::Path {

enum class StrategyType {
    FaceMilling2D,
    PocketMilling,
    ContourMilling,
    DrillCenter,
    Engrave
};

enum class CuttingDirection {
    Zigzag,
    Forward,
    Reverse
};

// Forward declaration
class PathStrategy;

using ToolpathPtr = std::shared_ptr<Toolpath>;
using PathStrategyPtr = std::shared_ptr<PathStrategy>;

// ============== 加工参数 ==============
class MachiningParameters {
public:
    MachiningParameters() = default;

    // Tool parameters
    double toolDiameter() const { return m_toolDiameter; }
    void setToolDiameter(double d) { m_toolDiameter = d; }

    double cornerRadius() const { return m_cornerRadius; }
    void setCornerRadius(double r) { m_cornerRadius = r; }

    double toolLength() const { return m_toolLength; }
    void setToolLength(double l) { m_toolLength = l; }

    // Cutting parameters
    double stepover() const { return m_stepover; }
    void setStepover(double s) { m_stepover = s; }

    double stepdown() const { return m_stepdown; }
    void setStepdown(double s) { m_stepdown = s; }

    double depth() const { return m_depth; }
    void setDepth(double d) { m_depth = d; }

    double feedrate() const { return m_feedrate; }
    void setFeedrate(double f) { m_feedrate = f; }

    double plungeFeedrate() const { return m_plungeFeedrate; }
    void setPlungeFeedrate(double f) { m_plungeFeedrate = f; }

    double spindleSpeed() const { return m_spindleSpeed; }
    void setSpindleSpeed(double s) { m_spindleSpeed = s; }

    // Height parameters
    double safeZ() const { return m_safeZ; }
    void setSafeZ(double z) { m_safeZ = z; }

    double clearance() const { return m_clearance; }
    void setClearance(double z) { m_clearance = z; }

    // Lead in/out
    double leadInLength() const { return m_leadInLength; }
    void setLeadInLength(double l) { m_leadInLength = l; }

    double leadOutLength() const { return m_leadOutLength; }
    void setLeadOutLength(double l) { m_leadOutLength = l; }

    // Tolerance
    double tolerance() const { return m_tolerance; }
    void setTolerance(double t) { m_tolerance = t; }

private:
    double m_toolDiameter = 10.0;
    double m_cornerRadius = 0.0;
    double m_toolLength = 50.0;
    double m_stepover = 5.0;
    double m_stepdown = 2.0;
    double m_depth = 5.0;
    double m_feedrate = 1000.0;
    double m_plungeFeedrate = 300.0;
    double m_spindleSpeed = 3000.0;
    double m_safeZ = 10.0;
    double m_clearance = 5.0;
    double m_leadInLength = 2.0;
    double m_leadOutLength = 2.0;
    double m_tolerance = 0.01;
};

// ============== 策略上下文 ==============
class PathStrategyContext {
public:
    PathStrategyContext() = default;

    // Boundary
    const TopoDS_Wire& getBoundaryWire() const { return m_boundaryWire; }
    void setBoundaryWire(const TopoDS_Wire& wire) { m_boundaryWire = wire; }

    // Face
    const TopoDS_Face& getBoundaryFace() const { return m_boundaryFace; }
    void setBoundaryFace(const TopoDS_Face& face) { m_boundaryFace = face; }

    // Cutting parameters
    double getStepover() const { return m_stepover; }
    void setStepover(double s) { m_stepover = s; }

    double getDepth() const { return m_depth; }
    void setDepth(double d) { m_depth = d; }

    double getStockTop() const { return m_stockTop; }
    void setStockTop(double z) { m_stockTop = z; }

    double getModelTop() const { return m_modelTop; }
    void setModelTop(double z) { m_modelTop = z; }

    double getCuttingAngle() const { return m_cuttingAngle; }
    void setCuttingAngle(double angle) { m_cuttingAngle = angle; }

    CuttingDirection getCuttingDirection() const { return m_cuttingDirection; }
    void setCuttingDirection(CuttingDirection dir) { m_cuttingDirection = dir; }

    // Tool parameters
    double getToolDiameter() const { return m_toolDiameter; }
    void setToolDiameter(double d) { m_toolDiameter = d; }

    double getFeedrate() const { return m_feedrate; }
    void setFeedrate(double f) { m_feedrate = f; }

    double getPlungeFeedrate() const { return m_plungeFeedrate; }
    void setPlungeFeedrate(double f) { m_plungeFeedrate = f; }

    double getSafeZ() const { return m_safeZ; }
    void setSafeZ(double z) { m_safeZ = z; }

    bool isLeadInEnabled() const { return m_leadInEnabled; }
    void setLeadInEnabled(bool enabled) { m_leadInEnabled = enabled; }

    bool isLeadOutEnabled() const { return m_leadOutEnabled; }
    void setLeadOutEnabled(bool enabled) { m_leadOutEnabled = enabled; }

    double getLeadInLength() const { return m_leadInLength; }
    void setLeadInLength(double l) { m_leadInLength = l; }

    double getLeadOutLength() const { return m_leadOutLength; }
    void setLeadOutLength(double l) { m_leadOutLength = l; }

    double getTolerance() const { return m_tolerance; }
    void setTolerance(double t) { m_tolerance = t; }

protected:
    TopoDS_Wire m_boundaryWire;
    TopoDS_Face m_boundaryFace;
    double m_stepover = 5.0;
    double m_depth = 5.0;
    double m_stockTop = 5.0;
    double m_modelTop = 0.0;
    double m_cuttingAngle = 0.0;
    CuttingDirection m_cuttingDirection = CuttingDirection::Zigzag;
    double m_toolDiameter = 10.0;
    double m_feedrate = 1000.0;
    double m_plungeFeedrate = 300.0;
    double m_safeZ = 10.0;
    bool m_leadInEnabled = false;
    bool m_leadOutEnabled = false;
    double m_leadInLength = 2.0;
    double m_leadOutLength = 2.0;
    double m_tolerance = 0.01;
};

// ============== 抽象策略基类 ==============
class PathStrategy {
public:
    virtual ~PathStrategy() = default;

    virtual StrategyType getType() const = 0;
    virtual std::string getName() const = 0;
    virtual std::string getDescription() const = 0;

    virtual bool validate() const = 0;
    virtual ToolpathPtr generate() = 0;

    void setContext(const PathStrategyContext& ctx) { m_context = ctx; }
    const PathStrategyContext& getContext() const { return m_context; }

    std::string getLastError() const { return m_lastError; }

protected:
    PathStrategy() = default;
    explicit PathStrategy(const PathStrategyContext& ctx) : m_context(ctx) {}

    PathStrategyContext m_context;
    mutable std::string m_lastError;
};

} // namespace PathForge::Path
