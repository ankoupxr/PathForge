#pragma once

#include <vector>
#include <memory>
#include <string>

#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include "../Toolpath.h"
#include "../Strategy.h"

namespace PathForge::Path {

// 注意：这是一个独立的策略类，用于 2D 轮廓加工
// 与 Strategy.h 中的 PathStrategy 基类不同

class ContourPathStrategy {
public:
    virtual ~ContourPathStrategy() = default;

    virtual ToolpathPtr generate(
        const std::vector<gp_Pnt>& boundary,
        const MachiningParameters& params
    ) = 0;

    virtual void setBoundary(const std::vector<gp_Pnt>& boundary);
    virtual void setOffset(double offset);
    virtual void setStartPoint(const gp_Pnt& point);
    virtual void setDirection(bool clockwise);

    virtual std::string strategyName() const = 0;

protected:
    ContourPathStrategy();

    std::vector<gp_Pnt> m_boundary;
    double m_offset = 0.0;
    gp_Pnt m_startPoint;
    bool m_clockwise = true;
    bool m_boundarySet = false;
};

class ZigzagStrategy : public ContourPathStrategy {
public:
    ZigzagStrategy();
    explicit ZigzagStrategy(double angle);

    ToolpathPtr generate(
        const std::vector<gp_Pnt>& boundary,
        const MachiningParameters& params
    ) override;

    std::string strategyName() const override;

    void setAngle(double degrees);
    double angle() const;

    void setEntryType(const std::string& type);
    const std::string& entryType() const;

private:
    void generateZigzag(ToolpathPtr path, const MachiningParameters& params);
    gp_Pnt calculateEntryPoint(double z);

    double m_angle = 0.0;
    std::string m_entryType = "plunge";
    double m_leadInLength = 2.0;
    double m_leadOutLength = 2.0;
};

class SpiralStrategy : public ContourPathStrategy {
public:
    SpiralStrategy();

    ToolpathPtr generate(
        const std::vector<gp_Pnt>& boundary,
        const MachiningParameters& params
    ) override;

    std::string strategyName() const override;

    void setSpiralType(const std::string& type);
    const std::string& spiralType() const;

    void setOverlap(double overlap);
    double overlap() const;

private:
    void generateClassicSpiral(ToolpathPtr path, const MachiningParameters& params);
    void generateOffsetSpiral(ToolpathPtr path, const MachiningParameters& params);

    std::string m_spiralType = "classic";
    double m_overlap = 0.0;
};

class OffsetStrategy : public ContourPathStrategy {
public:
    OffsetStrategy();

    ToolpathPtr generate(
        const std::vector<gp_Pnt>& boundary,
        const MachiningParameters& params
    ) override;

    std::string strategyName() const override;

    void setCompensationType(const std::string& type);
    const std::string& compensationType() const;

    void setCornerStyle(const std::string& style);
    const std::string& cornerStyle() const;

private:
    std::vector<gp_Pnt> offsetBoundary(double distance);
    gp_Pnt calculateCorner(const gp_Pnt& p1, const gp_Pnt& p2, const gp_Pnt& p3, double offset);

    std::string m_compensationType = "computer";
    std::string m_cornerStyle = "round";
};

class AdaptiveStrategy : public ContourPathStrategy {
public:
    AdaptiveStrategy();

    ToolpathPtr generate(
        const std::vector<gp_Pnt>& boundary,
        const MachiningParameters& params
    ) override;

    std::string strategyName() const override;

    void setMaxStepover(double stepover);
    double maxStepover() const;

    void setMinimumRadius(double radius);
    double minimumRadius() const;

    void setSmoothFactor(double factor);
    double smoothFactor() const;

private:
    void optimizeStepover(ToolpathPtr path, const MachiningParameters& params);
    double calculateCurvature(const gp_Pnt& p1, const gp_Pnt& p2, const gp_Pnt& p3);

    double m_maxStepover = 0.0;
    double m_minimumRadius = 0.5;
    double m_smoothFactor = 0.1;
};

using ContourPathStrategyPtr = std::shared_ptr<ContourPathStrategy>;

}
