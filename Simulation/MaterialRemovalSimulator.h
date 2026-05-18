#pragma once

#include "SimulationResults.h"

#include <PathForge/Path/Toolpath.h>

#include <TopoDS_Shape.hxx>
#include <TopoDS_Solid.hxx>
#include <gp_Pnt.hxx>

#include <memory>
#include <vector>

namespace PathForge::Simulation {

class MaterialRemovalSimulator {
public:
    MaterialRemovalSimulator();
    ~MaterialRemovalSimulator() = default;

    void setStockShape(const TopoDS_Shape& stock);
    const TopoDS_Shape& getStockShape() const { return m_stockShape; }

    void setToolGeometry(double diameter, double cornerRadius = 0.0);
    void setLayerHeight(double height);
    void setSimulationResolution(int resolution);

    TopoDS_Shape simulate(const Path::Toolpath& toolpath);
    TopoDS_Shape simulateWithProgress(const Path::Toolpath& toolpath);

    double getRemovedVolume() const { return m_removedVolume; }
    double getRemainingVolume() const { return m_remainingVolume; }
    double getRemovalPercentage() const { return m_removalPercentage; }

    std::vector<double> getLayerVolumes() const { return m_layerVolumes; }

    void clear();
    bool isEmpty() const { return m_stockShape.IsNull(); }

private:
    TopoDS_Shape calculateToolSweptVolume(const gp_Pnt& p1, const gp_Pnt& p2, double radius);
    double calculateCylinderVolume(const gp_Pnt& center, double radius, double height);

    void updateStatistics();
    void calculateVolumes();

    TopoDS_Shape m_stockShape;
    TopoDS_Shape m_simulatedShape;

    double m_toolDiameter = 10.0;
    double m_toolCornerRadius = 0.0;
    double m_layerHeight = 0.5;
    int m_simulationResolution = 50;

    double m_removedVolume = 0.0;
    double m_remainingVolume = 0.0;
    double m_removalPercentage = 0.0;
    std::vector<double> m_layerVolumes;
};

}
