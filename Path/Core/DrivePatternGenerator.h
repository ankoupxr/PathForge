#pragma once

#include <vector>
#include <memory>
#include <functional>

#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gp_Dir.hxx>

#include "SurfaceToolpathCore.h"

namespace PathForge::Path::Core {

struct DrivePoint {
    gp_Pnt position;
    double parameterU;
    double parameterV;
    
    DrivePoint() = default;
    DrivePoint(const gp_Pnt& p, double u = 0.0, double v = 0.0)
        : position(p), parameterU(u), parameterV(v) {}
};

class DrivePatternGenerator {
public:
    DrivePatternGenerator();
    ~DrivePatternGenerator() = default;

    void setPatternType(DrivePatternType type);
    void setBoundingBox(double xMin, double xMax, double yMin, double yMax);
    void setStep(double step);
    void setAngle(double angle);
    void setCenter(const gp_Pnt& center);
    void setRadius(double innerRadius, double outerRadius);

    std::vector<DrivePoint> generate();

    static std::vector<gp_Pnt> generateParallelLines(
        double xMin, double xMax,
        double yMin, double yMax,
        double step,
        double angle = 0.0,
        bool zigzag = false);

    static std::vector<gp_Pnt> generateSpiral(
        const gp_Pnt& center,
        double innerRadius,
        double outerRadius,
        double step,
        int turns = 3);

    static std::vector<gp_Pnt> generateCircular(
        const gp_Pnt& center,
        double innerRadius,
        double outerRadius,
        double step);

private:
    DrivePatternType m_patternType = DrivePatternType::ParallelLines;
    double m_xMin = 0.0, m_xMax = 100.0;
    double m_yMin = 0.0, m_yMax = 100.0;
    double m_step = 5.0;
    double m_angle = 0.0;
    gp_Pnt m_center;
    double m_innerRadius = 0.0;
    double m_outerRadius = 50.0;
};

using DrivePatternGeneratorPtr = std::shared_ptr<DrivePatternGenerator>;

}