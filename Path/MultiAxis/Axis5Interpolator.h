#pragma once

#include "MultiAxisConfig.h"
#include "MultiAxisPoint.h"

#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>

#include <vector>
#include <memory>

namespace PathForge::MultiAxis {

class ToolAxisController;

class Axis5Interpolator {
public:
    Axis5Interpolator();
    ~Axis5Interpolator() = default;

    void setConfiguration(const MultiAxisConfig& config);
    void setToolAxisController(ToolAxisController* controller);

    void setSurface(const TopoDS_Shape& surface);
    void setMachiningFace(const TopoDS_Face& face);

    void setFeedrate(double feedrate);
    double getFeedrate() const { return m_feedrate; }

    void setToolDiameter(double diameter);
    double getToolDiameter() const { return m_toolDiameter; }

    void setTolerance(double tolerance);
    double getTolerance() const { return m_tolerance; }

    MultiAxisPath interpolateLinearPath(const gp_Pnt& start, const gp_Pnt& end,
                                       const gp_Dir& startNormal, const gp_Dir& endNormal);

    MultiAxisPath interpolateArcPath(const gp_Pnt& start, const gp_Pnt& end, 
                                    const gp_Pnt& center, const gp_Dir& normal,
                                    bool isClockwise);

    MultiAxisPath interpolateCurvePath(const std::vector<gp_Pnt>& controlPoints,
                                       const std::vector<gp_Dir>& normals);

    int getInterpolatedPointCount() const { return m_interpolatedPointCount; }

    bool isValid() const;
    std::string getLastError() const { return m_lastError; }

private:
    struct InterpolationParameters {
        int pointCount;
        double chordHeight;
        double angularTolerance;
        double maxAngleVariation;
    };

    MultiAxisPoint interpolatePoint(const gp_Pnt& position, const gp_Dir& normal, double t);
    gp_Dir interpolateNormal(const gp_Dir& startNormal, const gp_Dir& endNormal, double t);
    void calculateAxisAngles(const gp_Dir& normal, double& aAngle, double& bAngle);

    gp_Pnt calculateHelixPoint(const gp_Pnt& center, double radius, double angle);
    gp_Dir calculateHelixNormal(double angle);

    std::vector<gp_Pnt> generateLinearInterpolation(const gp_Pnt& start, const gp_Pnt& end,
                                                     const InterpolationParameters& params);
    std::vector<gp_Pnt> generateArcInterpolation(const gp_Pnt& start, const gp_Pnt& end,
                                                  const gp_Pnt& center, double radius,
                                                  const InterpolationParameters& params);

    MultiAxisConfig m_config;
    ToolAxisController* m_toolAxisController = nullptr;

    TopoDS_Shape m_surface;
    TopoDS_Face m_machiningFace;

    double m_feedrate = 1500.0;
    double m_toolDiameter = 10.0;
    double m_tolerance = 0.01;

    int m_interpolatedPointCount = 0;
    std::string m_lastError;
};

}
