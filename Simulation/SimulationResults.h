#pragma once

#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <string>
#include <vector>
#include <memory>
#include <chrono>

namespace PathForge::Simulation {

enum class CollisionType {
    None,
    ToolHolder,
    ToolFlute,
    Spindle,
    Workpiece,
    Clamp,
    Table,
    AirStrike
};

enum class GougeSeverity {
    None,
    Minor,
    Moderate,
    Severe
};

struct CollisionInfo {
    int pointIndex;
    gp_Pnt position;
    CollisionType type;
    double distance;
    double severity;
    std::string description;
    gp_Pnt collisionPoint;
};

struct GougeInfo {
    int pointIndex;
    gp_Pnt position;
    TopoDS_Face face;
    double gougeDepth;
    double gougeArea;
    GougeSeverity severity;
    std::string description;
};

struct SimulationStatistics {
    int totalPoints;
    int cuttingPoints;
    int rapidPoints;
    int collisionCount;
    int gougeCount;
    double totalPathLength;
    double cuttingLength;
    double rapidLength;
    double estimatedTime;
    double materialRemovalVolume;
};

struct SimulationResults {
    bool isValid;
    bool hasCollisions;
    bool hasGouges;
    double overallRisk;

    std::vector<CollisionInfo> collisions;
    std::vector<GougeInfo> gouges;
    SimulationStatistics statistics;

    std::chrono::milliseconds simulationTime;
    std::string lastError;

    bool exportToFile(const std::string& filename) const;
    std::string generateReport() const;
};

}
