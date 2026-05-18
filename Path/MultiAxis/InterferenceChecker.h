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

enum class InterferenceType {
    None,
    HolderCollision,
    SpindleCollision,
    TailStockCollision,
    WorkpieceCollision
};

struct InterferenceInfo {
    int pointIndex;
    gp_Pnt position;
    gp_Dir toolAxis;
    InterferenceType type;
    double severity;
    std::string description;
    gp_Pnt interferencePoint;
};

class InterferenceChecker {
public:
    InterferenceChecker();
    ~InterferenceChecker() = default;

    void setConfiguration(const MultiAxisConfig& config);
    const MultiAxisConfig& getConfiguration() const { return m_config; }

    void setWorkpiece(const TopoDS_Shape& workpiece);
    void setStockShape(const TopoDS_Shape& stock);

    void setToolGeometry(double diameter, double length, double holderDiameter, double holderLength);
    void setPivotHeight(double height);

    std::vector<InterferenceInfo> checkPath(const MultiAxisPath& path);
    bool hasInterferences() const { return !m_interferences.empty(); }
    const std::vector<InterferenceInfo>& getInterferences() const { return m_interferences; }

    bool checkPointInterference(const gp_Pnt& position, const gp_Dir& axis);

    double calculateMinimumClearance(const gp_Pnt& position, const gp_Dir& axis);

    void setInterferenceTolerance(double tolerance);
    double getInterferenceTolerance() const { return m_interferenceTolerance; }

    void clear();

private:
    bool checkHolderInterference(const gp_Pnt& position, const gp_Dir& axis);
    bool checkSpindleInterference(const gp_Pnt& position, const gp_Dir& axis);
    bool checkWorkpieceInterference(const gp_Pnt& position, const gp_Dir& axis);

    gp_Pnt calculateHolderEndPosition(const gp_Pnt& toolTip, const gp_Dir& axis);
    gp_Pnt calculateSpindlePosition(const gp_Pnt& toolTip, const gp_Dir& axis);

    double calculateDistanceToWorkpiece(const gp_Pnt& point);
    bool isPointInsideWorkpiece(const gp_Pnt& point);

    MultiAxisConfig m_config;
    TopoDS_Shape m_workpiece;
    TopoDS_Shape m_stockShape;

    double m_toolDiameter = 10.0;
    double m_toolLength = 50.0;
    double m_holderDiameter = 30.0;
    double m_holderLength = 40.0;
    double m_pivotHeight = 100.0;

    double m_interferenceTolerance = 0.5;

    std::vector<InterferenceInfo> m_interferences;
};

}
