#pragma once

#include "SimulationResults.h"

#include <PathForge/Path/Toolpath.h>

#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <vector>
#include <memory>

namespace PathForge::Simulation {

class GougeDetector {
public:
    GougeDetector();
    ~GougeDetector() = default;

    void setStockShape(const TopoDS_Shape& stock);
    void setToolGeometry(double diameter, double cornerRadius = 0.0);

    void setGougeTolerance(double tolerance);
    double getGougeTolerance() const { return m_gougeTolerance; }

    void setAllowableStock(double stock);
    double getAllowableStock() const { return m_allowableStock; }

    std::vector<GougeInfo> detectGouges(const Path::Toolpath& toolpath);
    bool hasGouges() const { return !m_gouges.empty(); }
    const std::vector<GougeInfo>& getGouges() const { return m_gouges; }

    double calculateGougeDepth(const gp_Pnt& point, const TopoDS_Face& face);
    GougeSeverity evaluateGougeSeverity(double depth) const;

    void clear();

private:
    bool isPointBelowStock(const gp_Pnt& point);
    double calculateDistanceToStock(const gp_Pnt& point);
    TopoDS_Face findNearestFace(const gp_Pnt& point);

    TopoDS_Shape m_stockShape;
    double m_toolDiameter = 10.0;
    double m_toolCornerRadius = 0.0;
    double m_gougeTolerance = 0.01;
    double m_allowableStock = 0.0;

    std::vector<GougeInfo> m_gouges;
};

}
