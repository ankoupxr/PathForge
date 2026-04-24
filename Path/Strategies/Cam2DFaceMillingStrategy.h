// Cam2DFaceMillingStrategy.h
#pragma once

#include "CamPathStrategy.h"

namespace PathForge {
namespace CAM {

class Cam2DFaceMillingStrategy : public CamPathStrategy {
public:
    Cam2DFaceMillingStrategy();
    explicit Cam2DFaceMillingStrategy(CamFacePtr face);

    CamStrategyType getType() const override;
    std::string getName() const override;
    std::string getDescription() const override;

    bool validate() const override;
    CamToolpathPtr generate() override;

    void setStepover(double s) { m_stepover = s; }
    double getStepover() const { return m_stepover; }

    void setCuttingAngle(double angle) { m_cuttingAngle = angle; }
    double getCuttingAngle() const { return m_cuttingAngle; }

    void setCuttingDirection(CamCuttingDirection dir) { m_cuttingDirection = dir; }
    CamCuttingDirection getCuttingDirection() const { return m_cuttingDirection; }

private:
    struct BoundingBox2D {
        double minX, maxX, minY, maxY;
    };

    BoundingBox2D computeBoundingBox(const CamWire& wire);
    std::vector<CamPoint> generateZigzagLines(const BoundingBox2D& bbox,
                                                double stepover,
                                                double angle);
    std::vector<CamPoint> generateOneDirectionLines(const BoundingBox2D& bbox,
                                                       double stepover,
                                                       double angle,
                                                       bool forward);
    bool pointInPolygon(const CamPoint& p, const std::vector<CamPoint>& polygon);
    CamPoint findEntryPoint(const CamBoundingBox& bbox, double angle);

    double m_stepover = 5.0;
    double m_cuttingAngle = 0.0;
    CamCuttingDirection m_cuttingDirection = CamCuttingDirection::Zigzag;
};

} // namespace CAM
} // namespace PathForge