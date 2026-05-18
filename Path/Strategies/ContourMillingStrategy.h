#pragma once

#include "../Strategy.h"

namespace PathForge::Path {

enum class ContourCompensation {
    计算机补偿,  // Computer compensation
    机床补偿,    // Machine compensation
    无补偿       // No compensation
};

enum class ContourSide {
    左补偿,      // Left compensation
    右补偿,      // Right compensation
    在线中       // On line
};

class ContourMillingStrategy : public PathStrategy {
public:
    ContourMillingStrategy();
    explicit ContourMillingStrategy(const PathStrategyContext& context);

    StrategyType getType() const override;
    std::string getName() const override;
    std::string getDescription() const override;

    bool validate() const override;
    ToolpathPtr generate() override;

    void setCompensation(ContourCompensation comp);
    ContourCompensation getCompensation() const;

    void setContourSide(ContourSide side);
    ContourSide getContourSide() const;

    void setAllowMultiplePasses(bool enabled);
    bool isAllowMultiplePassesEnabled() const;

    void setStepDepth(double depth);
    double getStepDepth() const;

    void setFinishPassEnabled(bool enabled);
    bool isFinishPassEnabled() const;

    void setFinishStock(double stock);
    double getFinishStock() const;

    void setCornerRadius(double radius);
    double getCornerRadius() const;

    void setSmoothCorners(bool enabled);
    bool isSmoothCornersEnabled() const;

    void setMergeEnabled(bool enabled);
    bool isMergeEnabled() const;

private:
    ToolpathPtr generateRoughingPass(const TopoDS_Wire& boundary);
    ToolpathPtr generateFinishPass(const TopoDS_Wire& boundary);

    std::vector<gp_Pnt> extractWirePoints(const TopoDS_Wire& wire);
    gp_Pnt offsetPoint(const gp_Pnt& point, const gp_Dir& normal, double offset);

    ContourCompensation m_compensation = ContourCompensation::计算机补偿;
    ContourSide m_contourSide = ContourSide::左补偿;
    bool m_allowMultiplePasses = true;
    double m_stepDepth = 2.0;
    bool m_finishPassEnabled = true;
    double m_finishStock = 0.5;
    double m_cornerRadius = 0.0;
    bool m_smoothCorners = false;
    bool m_mergeEnabled = false;
};

}
