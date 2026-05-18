#pragma once

#include "MultiAxisConfig.h"
#include "MultiAxisPoint.h"
#include "ToolAxisController.h"
#include "Axis3Plus2Positioning.h"
#include "Axis5Interpolator.h"
#include "InterferenceChecker.h"

#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>

#include <memory>
#include <vector>
#include <string>

namespace PathForge::MultiAxis {

struct MultiAxisResult {
    bool success;
    bool hasInterferences;
    double riskLevel;
    std::string errorMessage;
    std::vector<InterferenceInfo> interferences;

    int totalPoints;
    int positioningPoints;
    int interpolatedPoints;
    double totalPathLength;
    double estimatedTime;
};

class MultiAxisEngine {
public:
    MultiAxisEngine();
    ~MultiAxisEngine() = default;

    void setConfiguration(const MultiAxisConfig& config);
    const MultiAxisConfig& getConfiguration() const { return m_config; }

    void setWorkpiece(const TopoDS_Shape& workpiece);
    void setStockShape(const TopoDS_Shape& stock);
    void setMachiningFace(const TopoDS_Face& face);

    void setToolGeometry(double diameter, double length, 
                        double holderDiameter, double holderLength);

    void setControlMode(AxisControlMode mode);

    MultiAxisResult generate5AxisPath(const std::vector<gp_Pnt>& points,
                                      const std::vector<gp_Dir>& normals);

    MultiAxisResult generate3Plus2Path(const std::vector<gp_Pnt>& points);

    const MultiAxisPath& getLastResult() const { return m_lastResult; }

    void setFeedrate(double feedrate);
    void setSpindleSpeed(double speed);

    void setInterpolationTolerance(double tolerance);
    void setInterferenceTolerance(double tolerance);

    bool validateConfiguration() const;
    std::string getLastError() const { return m_lastError; }

private:
    MultiAxisResult createResult();
    void setupComponents();

    MultiAxisConfig m_config;
    TopoDS_Shape m_workpiece;
    TopoDS_Shape m_stockShape;
    TopoDS_Face m_machiningFace;

    std::unique_ptr<ToolAxisController> m_toolAxisController;
    std::unique_ptr<Axis3Plus2Positioning> m_axis3Plus2;
    std::unique_ptr<Axis5Interpolator> m_axis5Interpolator;
    std::unique_ptr<InterferenceChecker> m_interferenceChecker;

    MultiAxisPath m_lastResult;
    std::string m_lastError;

    double m_feedrate = 1500.0;
    double m_spindleSpeed = 3000.0;
    double m_interpolationTolerance = 0.01;
    double m_interferenceTolerance = 0.5;
};

}
