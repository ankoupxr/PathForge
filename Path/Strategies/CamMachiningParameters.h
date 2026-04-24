// CamMachiningParameters.h
#pragma once

#include "CamPoint.h"
#include "CamDirection.h"

namespace PathForge {

struct CamMachiningParameters {
    double toolDiameter = 10.0;
    double cornerRadius = 0.0;
    double stepover = 5.0;
    double stepdown = 2.0;
    double depth = 5.0;
    double feedrate = 1000.0;
    double plungeFeedrate = 300.0;
    double spindleSpeed = 3000.0;
    double safeZ = 10.0;
    double clearance = 5.0;
    double leadInLength = 2.0;
    double leadOutLength = 2.0;
    double tolerance = 0.01;
};

enum class CamCuttingDirection {
    Zigzag,
    Forward,
    Reverse
};

struct CamPathStrategyContext {
    CamPoint stockTop = CamPoint(0, 0, 5);
    CamPoint modelTop = CamPoint(0, 0, 0);
    double stepover = 5.0;
    double depth = 5.0;
    double cuttingAngle = 0.0;
    CamCuttingDirection cuttingDirection = CamCuttingDirection::Zigzag;
    double toolDiameter = 10.0;
    double feedrate = 1000.0;
    double plungeFeedrate = 300.0;
    double safeZ = 10.0;
    bool leadInEnabled = false;
    bool leadOutEnabled = false;
    double leadInLength = 2.0;
    double leadOutLength = 2.0;
    double tolerance = 0.01;
};

} // namespace PathForge