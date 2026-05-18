#include "SimulationEngine.h"

#include <algorithm>
#include <cmath>
#include <chrono>

namespace PathForge::Simulation {

SimulationEngine::SimulationEngine() {
    m_collisionDetector = std::make_unique<CollisionDetector>();
    m_gougeDetector = std::make_unique<GougeDetector>();
    m_materialSimulator = std::make_unique<MaterialRemovalSimulator>();
}

void SimulationEngine::setWorkpiece(const TopoDS_Shape& workpiece) {
    m_workpiece = workpiece;
    m_collisionDetector->setWorkpiece(workpiece);
}

void SimulationEngine::setStockShape(const TopoDS_Shape& stock) {
    m_stockShape = stock;
    m_collisionDetector->setStockShape(stock);
    m_gougeDetector->setStockShape(stock);
    m_materialSimulator->setStockShape(stock);
}

void SimulationEngine::setToolGeometry(double diameter, double length, double holderDiameter, double holderLength) {
    ToolGeometry geo;
    geo.diameter = diameter;
    geo.length = length;
    geo.holderDiameter = holderDiameter;
    geo.holderLength = holderLength;

    m_collisionDetector->setToolGeometry(geo);
    m_gougeDetector->setToolGeometry(diameter);
    m_materialSimulator->setToolGeometry(diameter);
}

void SimulationEngine::setCollisionCheckEnabled(bool enabled) {
    m_collisionCheckEnabled = enabled;
}

void SimulationEngine::setGougeCheckEnabled(bool enabled) {
    m_gougeCheckEnabled = enabled;
}

void SimulationEngine::setMaterialRemovalEnabled(bool enabled) {
    m_materialRemovalEnabled = enabled;
}

SimulationResults SimulationEngine::runSimulation(const Path::Toolpath& toolpath) {
    m_startTime = std::chrono::steady_clock::now();
    m_cancelled = false;
    m_progress = 0.0;

    m_lastResults = SimulationResults();
    m_lastResults.isValid = true;

    if (toolpath.isEmpty()) {
        m_lastResults.isValid = false;
        m_lastResults.lastError = "Toolpath is empty";
        return m_lastResults;
    }

    if (m_stockShape.IsNull() && m_workpiece.IsNull()) {
        m_lastResults.lastError = "No workpiece or stock shape defined";
        return m_lastResults;
    }

    m_lastResults.collisions = m_collisionDetector->detectCollisions(toolpath);
    m_progress = 0.3;

    m_lastResults.gouges = m_gougeDetector->detectGouges(toolpath);
    m_progress = 0.6;

    if (m_materialRemovalEnabled) {
        m_materialSimulator->simulate(toolpath);
        m_progress = 0.9;
    }

    calculateStatistics(toolpath);

    m_lastResults.hasCollisions = !m_lastResults.collisions.empty();
    m_lastResults.hasGouges = !m_lastResults.gouges.empty();
    m_lastResults.overallRisk = calculateOverallRisk();

    m_endTime = std::chrono::steady_clock::now();
    m_lastResults.simulationTime = std::chrono::duration_cast<std::chrono::milliseconds>(m_endTime - m_startTime);

    m_progress = 1.0;

    return m_lastResults;
}

SimulationResults SimulationEngine::runSimulationWithProgress(const Path::Toolpath& toolpath) {
    return runSimulation(toolpath);
}

void SimulationEngine::cancelSimulation() {
    m_cancelled = true;
}

void SimulationEngine::clear() {
    m_collisionDetector->clear();
    m_gougeDetector->clear();
    m_materialSimulator->clear();
    m_lastResults = SimulationResults();
    m_progress = 0.0;
    m_cancelled = false;
}

void SimulationEngine::initializeDetectors() {
}

SimulationResults SimulationEngine::createResults() {
    return SimulationResults();
}

void SimulationEngine::calculateStatistics(const Path::Toolpath& toolpath) {
    const auto& points = toolpath.points();

    m_lastResults.statistics.totalPoints = static_cast<int>(points.size());
    m_lastResults.statistics.totalPathLength = toolpath.totalLength();
    m_lastResults.statistics.cuttingLength = toolpath.cuttingLength();

    int cuttingCount = 0;
    int rapidCount = 0;
    double rapidLength = 0.0;

    for (size_t i = 1; i < points.size(); ++i) {
        const auto& pt = points[i];
        if (pt.motionType == Path::MotionType::Cutting) {
            cuttingCount++;
        } else if (pt.moveType == Path::MoveType::Rapid) {
            rapidCount++;
            rapidLength += points[i].position.Distance(points[i - 1].position);
        }
    }

    m_lastResults.statistics.cuttingPoints = cuttingCount;
    m_lastResults.statistics.rapidPoints = rapidCount;
    m_lastResults.statistics.rapidLength = rapidLength;

    m_lastResults.statistics.collisionCount = static_cast<int>(m_lastResults.collisions.size());
    m_lastResults.statistics.gougeCount = static_cast<int>(m_lastResults.gouges.size());

    double totalTime = toolpath.duration();
    m_lastResults.statistics.estimatedTime = totalTime / 60.0;

    m_lastResults.statistics.materialRemovalVolume = m_materialSimulator->getRemovedVolume();
}

double SimulationEngine::calculateOverallRisk() {
    double collisionRisk = m_lastResults.collisions.empty() ? 0.0 :
        std::min(1.0, m_lastResults.collisions.size() / 10.0);

    double gougeRisk = m_lastResults.gouges.empty() ? 0.0 :
        std::min(1.0, m_lastResults.gouges.size() / 20.0);

    double severeGougeCount = 0.0;
    for (const auto& gouge : m_lastResults.gouges) {
        if (gouge.severity == GougeSeverity::Severe) {
            severeGougeCount += 0.5;
        } else if (gouge.severity == GougeSeverity::Moderate) {
            severeGougeCount += 0.2;
        }
    }
    gougeRisk = std::max(gougeRisk, std::min(1.0, severeGougeCount / 10.0));

    double overallRisk = std::max(collisionRisk, gougeRisk * 0.8);

    return overallRisk;
}

std::chrono::milliseconds SimulationEngine::getSimulationTime() const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(m_endTime - m_startTime);
}

}
