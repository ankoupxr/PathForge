#include "ToolAxisController.h"

#include <gp_Vec.hxx>
#include <gp_Trsf.hxx>
#include <gp_Ax3.hxx>
#include <gp_Pnt.hxx>

#include <cmath>
#include <algorithm>

namespace PathForge::MultiAxis {

ToolAxisController::ToolAxisController() {
    m_config.configuration = AxisConfiguration::Axis5;
    m_config.outputA = true;
    m_config.outputB = true;
    m_config.outputC = false;
    m_config.tiltedHead.maxTiltAngle = 30.0;
    m_config.tiltedHead.minTiltAngle = -30.0;
    m_config.tiltedHead.allowContinuousTilt = true;
    m_fixedAxis = gp_Dir(0, 0, 1);
}

void ToolAxisController::setConfiguration(const MultiAxisConfig& config) {
    m_config = config;
}

void ToolAxisController::setControlMode(AxisControlMode mode) {
    m_controlMode = mode;
}

void ToolAxisController::setSurface(const SurfaceGeometry* surface) {
    m_surface = surface;
}

gp_Dir ToolAxisController::calculateToolAxis(const gp_Pnt& position, const gp_Vec& surfaceNormal) {
    switch (m_controlMode) {
        case AxisControlMode::Normal:
            return calculateNormalControl(position);
        case AxisControlMode::Tilted:
            return calculateTiltedControl(position, 0.0, 0.0);
        case AxisControlMode::Fixed:
            return calculateFixedControl(position);
        case AxisControlMode::Interpolated:
            return calculateInterpolatedControl(position);
        default:
            return calculateNormalControl(position);
    }
}

gp_Dir ToolAxisController::calculateTiltedAxis(const gp_Pnt& position, const gp_Dir& normal,
                                                 double aAngle, double bAngle) {
    gp_Trsf trsfA, trsfB, trsfTotal;

    gp_Dir xAxis(1, 0, 0);
    gp_Dir zAxis(0, 0, 1);

    trsfA.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), xAxis), aAngle * M_PI / 180.0);
    trsfB.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), zAxis), bAngle * M_PI / 180.0);

    trsfTotal.Multiply(trsfB);
    trsfTotal.Multiply(trsfA);

    gp_Dir result = normal.Transformed(trsfTotal);
    result.Normalize();

    return result;
}

void ToolAxisController::calculateTiltAngles(const gp_Dir& targetNormal, double& aAngle, double& bAngle) {
    double z = targetNormal.Z();

    bAngle = std::asin(std::max(-1.0, std::min(1.0, -targetNormal.X()))) * 180.0 / M_PI;

    double projLen = std::sqrt(1.0 - z * z);
    if (projLen > 0.001) {
        aAngle = std::asin(std::max(-1.0, std::min(1.0, -targetNormal.Y() / projLen))) * 180.0 / M_PI;
    } else {
        aAngle = 0.0;
    }

    aAngle = std::max(m_config.tiltedHead.minTiltAngle,
                       std::min(m_config.tiltedHead.maxTiltAngle, aAngle));
    bAngle = std::max(m_config.tiltedHead.minTiltAngle,
                      std::min(m_config.tiltedHead.maxTiltAngle, bAngle));
}

gp_Dir ToolAxisController::projectAxisOntoLimit(const gp_Dir& axis, const gp_Dir& limitNormal) {
    gp_Vec axisVec(axis.X(), axis.Y(), axis.Z());
    gp_Vec limitVec(limitNormal.X(), limitNormal.Y(), limitNormal.Z());

    gp_Vec proj = axisVec - limitVec.Dot(axisVec) * limitVec;
    proj.Normalize();

    return gp_Dir(proj.X(), proj.Y(), proj.Z());
}

bool ToolAxisController::isWithinLimits(const gp_Dir& axis) const {
    if (!m_config.tiltedHead.allowContinuousTilt) {
        double angle = std::acos(std::abs(axis.Z())) * 180.0 / M_PI;
        return angle >= m_config.tiltedHead.minTiltAngle &&
               angle <= m_config.tiltedHead.maxTiltAngle;
    }
    return true;
}

void ToolAxisController::setLeadCompensation(double lead) {
    m_leadCompensation = lead;
}

void ToolAxisController::setLagCompensation(double lag) {
    m_lagCompensation = lag;
}

gp_Dir ToolAxisController::calculateNormalControl(const gp_Pnt& position) {
    if (m_surface) {
        return m_surface->getNormalAt(position);
    }
    return m_fixedAxis;
}

gp_Dir ToolAxisController::calculateTiltedControl(const gp_Pnt& position, double aAngle, double bAngle) {
    gp_Dir normal = calculateNormalControl(position);
    return calculateTiltedAxis(position, normal, aAngle, bAngle);
}

gp_Dir ToolAxisController::calculateFixedControl(const gp_Pnt& position) {
    return m_fixedAxis;
}

gp_Dir ToolAxisController::calculateInterpolatedControl(const gp_Pnt& position) {
    return calculateNormalControl(position);
}

double ToolAxisController::calculatePivotLength(const gp_Pnt& position, const gp_Dir& axis) {
    return m_pivotHeight;
}

}
