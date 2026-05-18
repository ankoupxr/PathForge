#pragma once

#include "../Strategy.h"

namespace PathForge::Path {

enum class DrillPattern {
   啄钻,        // Peck drilling
    深孔钻,      // Deep drilling
    铰孔,        // Reaming
    镗孔,        // Boring
    螺纹钻,      // Thread drilling
    中心钻       // Center drilling
};

class DrillingStrategy : public PathStrategy {
public:
    DrillingStrategy();
    explicit DrillingStrategy(const PathStrategyContext& context);

    StrategyType getType() const override;
    std::string getName() const override;
    std::string getDescription() const override;

    bool validate() const override;
    ToolpathPtr generate() override;

    void setDrillPattern(DrillPattern pattern);
    DrillPattern getDrillPattern() const;

    void setPeckDepth(double depth);
    double getPeckDepth() const;

    void setDwellTime(double seconds);
    double getDwellTime() const;

    void setRetractDistance(double distance);
    double getRetractDistance() const;

    void setSpindleSpeed(int rpm);
    int getSpindleSpeed() const;

    void setDrillCycleType(const std::string& cycle);
    const std::string& getDrillCycleType() const;

private:
    ToolpathPtr generatePeckDrilling(double startZ, double depth, double peckDepth);
    ToolpathPtr generateDeepDrilling(double startZ, double depth, double peckDepth);
    ToolpathPtr generateNormalDrilling(double startZ, double depth);
    ToolpathPtr generateCenterDrilling(double startZ, double depth);

    gp_Pnt m_drillStartPosition;
    std::vector<gp_Pnt> m_holePositions;
    DrillPattern m_drillPattern = DrillPattern::啄钻;
    double m_peckDepth = 2.0;
    double m_dwellTime = 0.0;
    double m_retractDistance = 1.0;
    int m_spindleSpeed = 3000;
    std::string m_drillCycleType = "G83";
};

}
