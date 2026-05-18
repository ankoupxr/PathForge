#pragma once

#include "../Strategy.h"

namespace PathForge::Path {

enum class PocketRoughingPattern {
   螺旋线,      // Classic spiral
    平行线,      // Parallel lines
    往复式,      // Zigzag
    轮廓偏移      // Contour offset
};

class PocketMillingStrategy : public PathStrategy {
public:
    PocketMillingStrategy();
    explicit PocketMillingStrategy(const PathStrategyContext& context);

    StrategyType getType() const override;
    std::string getName() const override;
    std::string getDescription() const override;

    bool validate() const override;
    ToolpathPtr generate() override;

    void setRoughingPattern(PocketRoughingPattern pattern);
    PocketRoughingPattern getRoughingPattern() const;

    void setRoughingClearance(double clearance);
    double getRoughingClearance() const;

    void setMinimumPocketRadius(double radius);
    double getMinimumPocketRadius() const;

    void setHelicalEntry(bool enabled);
    bool isHelicalEntryEnabled() const;

    void setHelixRadius(double radius);
    double getHelixRadius() const;

    void setStepDepth(double depth);
    double getStepDepth() const;

    void setFinishPassEnabled(bool enabled);
    bool isFinishPassEnabled() const;

    void setFinishStepover(double stepover);
    double getFinishStepover() const;

private:
    ToolpathPtr generateSpiralRoughing(const TopoDS_Wire& boundary);
    ToolpathPtr generateParallelRoughing(const TopoDS_Wire& boundary);
    ToolpathPtr generateZigzagRoughing(const TopoDS_Wire& boundary);

    std::vector<gp_Pnt> generateSpiralPath(double centerX, double centerY,
                                          double startRadius, double endRadius,
                                          double stepover, int direction);

    gp_Pnt findEntryPoint(const TopoDS_Wire& wire);
    double calculatePocketArea(const TopoDS_Wire& wire);

    PocketRoughingPattern m_pattern = PocketRoughingPattern::螺旋线;
    double m_roughingClearance = 0.5;
    double m_minimumPocketRadius = 2.0;
    bool m_helicalEntry = true;
    double m_helixRadius = 5.0;
    double m_stepDepth = 2.0;
    bool m_finishPassEnabled = false;
    double m_finishStepover = 1.0;
};

}
