#pragma once

#include "MultiAxisConfig.h"
#include "MultiAxisPoint.h"

#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax1.hxx>

#include <vector>
#include <memory>

namespace PathForge::MultiAxis {

class SurfaceGeometry;

class ToolAxisController {
public:
    ToolAxisController();
    ~ToolAxisController() = default;

    void setConfiguration(const MultiAxisConfig& config);
    const MultiAxisConfig& getConfiguration() const { return m_config; }

    void setControlMode(AxisControlMode mode);
    AxisControlMode getControlMode() const { return m_controlMode; }

    void setSurface(const SurfaceGeometry* surface);
    const SurfaceGeometry* getSurface() const { return m_surface; }

    gp_Dir calculateToolAxis(const gp_Pnt& position, const gp_Vec& surfaceNormal);
    gp_Dir calculateTiltedAxis(const gp_Pnt& position, const gp_Dir& normal, 
                                double aAngle, double bAngle);

    void calculateTiltAngles(const gp_Dir& targetNormal, double& aAngle, double& bAngle);

    gp_Dir projectAxisOntoLimit(const gp_Dir& axis, const gp_Dir& limitNormal);
    bool isWithinLimits(const gp_Dir& axis) const;

    void setLeadCompensation(double lead);
    void setLagCompensation(double lag);

    double getLeadCompensation() const { return m_leadCompensation; }
    double getLagCompensation() const { return m_lagCompensation; }

private:
    gp_Dir calculateNormalControl(const gp_Pnt& position);
    gp_Dir calculateTiltedControl(const gp_Pnt& position, double aAngle, double bAngle);
    gp_Dir calculateFixedControl(const gp_Pnt& position);
    gp_Dir calculateInterpolatedControl(const gp_Pnt& position);

    double calculatePivotLength(const gp_Pnt& position, const gp_Dir& axis);

    MultiAxisConfig m_config;
    AxisControlMode m_controlMode = AxisControlMode::Normal;
    const SurfaceGeometry* m_surface = nullptr;

    double m_leadCompensation = 0.0;
    double m_lagCompensation = 0.0;

    gp_Dir m_fixedAxis;
    double m_pivotHeight = 100.0;
};

}
