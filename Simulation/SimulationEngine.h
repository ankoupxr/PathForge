#pragma once

#include "SimulationResults.h"
#include "CollisionDetector.h"
#include "GougeDetector.h"
#include "MaterialRemovalSimulator.h"

#include <PathForge/Path/Toolpath.h>

#include <TopoDS_Shape.hxx>
#include <TopoDS_Solid.hxx>

#include <memory>
#include <chrono>

namespace PathForge::Simulation {

class SimulationEngine {
public:
    SimulationEngine();
    ~SimulationEngine() = default;

    void setWorkpiece(const TopoDS_Shape& workpiece);
    void setStockShape(const TopoDS_Shape& stock);

    void setToolGeometry(double diameter, double length, double holderDiameter = 30.0, double holderLength = 40.0);

    void setCollisionCheckEnabled(bool enabled);
    void setGougeCheckEnabled(bool enabled);
    void setMaterialRemovalEnabled(bool enabled);

    SimulationResults runSimulation(const Path::Toolpath& toolpath);
    SimulationResults runSimulationWithProgress(const Path::Toolpath& toolpath);

    const CollisionDetector& getCollisionDetector() const { return *m_collisionDetector; }
    const GougeDetector& getGougeDetector() const { return *m_gougeDetector; }
    const MaterialRemovalSimulator& getMaterialSimulator() const { return *m_materialSimulator; }

    bool isSimulationValid() const { return m_lastResults.isValid; }
    const SimulationResults& getLastResults() const { return m_lastResults; }

    double getProgress() const { return m_progress; }

    void cancelSimulation();
    void clear();

private:
    void initializeDetectors();
    SimulationResults createResults();
    void calculateStatistics(const Path::Toolpath& toolpath);
    double calculateOverallRisk();
    std::chrono::milliseconds getSimulationTime() const;

    TopoDS_Shape m_workpiece;
    TopoDS_Shape m_stockShape;

    std::unique_ptr<CollisionDetector> m_collisionDetector;
    std::unique_ptr<GougeDetector> m_gougeDetector;
    std::unique_ptr<MaterialRemovalSimulator> m_materialSimulator;

    bool m_collisionCheckEnabled = true;
    bool m_gougeCheckEnabled = true;
    bool m_materialRemovalEnabled = true;

    bool m_cancelled = false;
    double m_progress = 0.0;

    SimulationResults m_lastResults;
    std::chrono::steady_clock::time_point m_startTime;
    std::chrono::steady_clock::time_point m_endTime;
};

}
