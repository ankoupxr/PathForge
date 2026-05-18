#pragma once

#include "SimulationResults.h"

#include <PathForge/Path/Toolpath.h>
#include <PathForge/Geometry/CAM/CamTypes.h>

#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <vector>
#include <memory>

namespace PathForge::Simulation {

struct ToolGeometry {
    double diameter;
    double length;
    double holderDiameter;
    double holderLength;
    double fluteLength;
    int fluteNumber;
};

class CollisionDetector {
public:
    CollisionDetector();
    ~CollisionDetector() = default;

    void setToolGeometry(const ToolGeometry& geometry);
    const ToolGeometry& getToolGeometry() const { return m_toolGeometry; }

    void setWorkpiece(const TopoDS_Shape& workpiece);
    void setStockShape(const TopoDS_Shape& stock);

    void setCollisionTolerance(double tolerance);
    double getCollisionTolerance() const { return m_collisionTolerance; }

    void setCheckToolHolder(bool check);
    void setCheckWorkpiece(bool check);
    void setCheckAirStrike(bool check);

    std::vector<CollisionInfo> detectCollisions(const Path::Toolpath& toolpath);
    bool hasCollisions() const { return !m_collisions.empty(); }
    const std::vector<CollisionInfo>& getCollisions() const { return m_collisions; }

    void clear();

private:
    bool isPointInCollision(const gp_Pnt& point, double zCoord);
    double calculateMinimumDistance(const gp_Pnt& point, const TopoDS_Shape& shape);
    bool checkToolHolderCollision(const gp_Pnt& point, const TopoDS_Shape& shape);
    bool checkToolFluteCollision(const gp_Pnt& point, const TopoDS_Shape& shape);

    gp_Pnt calculateToolTipPosition(const gp_Pnt& feedPoint, double cutDepth);
    gp_Pnt calculateHolderPosition(const gp_Pnt& feedPoint, double safeZ);

    ToolGeometry m_toolGeometry;
    TopoDS_Shape m_workpiece;
    TopoDS_Shape m_stockShape;
    double m_collisionTolerance = 0.1;
    double m_safeClearance = 1.0;

    bool m_checkToolHolder = true;
    bool m_checkWorkpiece = true;
    bool m_checkAirStrike = false;

    std::vector<CollisionInfo> m_collisions;
};

}
