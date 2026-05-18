#pragma once

#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax1.hxx>

#include <string>
#include <memory>

namespace PathForge::MultiAxis {

enum class AxisConfiguration {
    Axis3Plus2,
    Axis5,
    Axis4,
    Axis3
};

enum class AxisControlMode {
    Tilted,
    Normal,
    Fixed,
    Interpolated
};

enum class InterpolationType {
    Linear,
    Spline,
    Nurbs
};

struct AxisLimits {
    double minX, maxX;
    double minY, maxY;
    double minZ, maxZ;
    double minA, maxA;
    double minB, maxB;
    double minC, maxC;

    bool isWithinLimits(double x, double y, double z, double a, double b, double c) const;
};

struct TiltedHeadSettings {
    double maxTiltAngle;
    double minTiltAngle;
    bool allowContinuousTilt;
    double tiltStep;

    double primaryTiltAxis;
    double secondaryTiltAxis;
};

struct MultiAxisConfig {
    AxisConfiguration configuration;

    bool outputA;
    bool outputB;
    bool outputC;

    bool allowToolLengthCompensation;
    bool useDynamicToolLength;

    InterpolationType interpolationType;
    int splineDegree;

    AxisLimits limits;
    TiltedHeadSettings tiltedHead;

    double collisionClearance;
    bool checkInterference;

    bool smoothAxisTransitions;
    double maxAxisAcceleration;
    double maxAxisVelocity;

    std::string machineConfigFile;

    bool isValid() const;
};

struct ToolOrientation {
    gp_Dir axis;
    gp_Dir normal;
    gp_Pnt position;
    double feedrate;
    double spindleSpeed;

    gp_Ax1 getToolAxis() const;
};

struct MultiAxisPoint {
    gp_Pnt position;
    gp_Dir toolAxis;
    gp_Dir normal;
    double feedrate;
    double spindleSpeed;
    int lineNumber;

    bool hasTiltedAxis;
    double tiltAngleA;
    double tiltAngleB;
};

}
