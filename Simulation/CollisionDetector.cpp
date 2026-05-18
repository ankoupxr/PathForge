#include "CollisionDetector.h"

#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeBox.hxx>

#include <BRepExtrema_ExtPN.hxx>
#include <BRepClass3d_SolidClassifier.hxx>

#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>

#include <gp_Cylinder.hxx>
#include <gp_Circ.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax3.hxx>

#include <algorithm>
#include <cmath>

namespace PathForge::Simulation {

CollisionDetector::CollisionDetector() {
    m_toolGeometry.diameter = 10.0;
    m_toolGeometry.length = 50.0;
    m_toolGeometry.holderDiameter = 30.0;
    m_toolGeometry.holderLength = 40.0;
    m_toolGeometry.fluteLength = 20.0;
    m_toolGeometry.fluteNumber = 4;
}

void CollisionDetector::setToolGeometry(const ToolGeometry& geometry) {
    m_toolGeometry = geometry;
}

void CollisionDetector::setWorkpiece(const TopoDS_Shape& workpiece) {
    m_workpiece = workpiece;
}

void CollisionDetector::setStockShape(const TopoDS_Shape& stock) {
    m_stockShape = stock;
}

void CollisionDetector::setCollisionTolerance(double tolerance) {
    m_collisionTolerance = tolerance;
}

void CollisionDetector::setCheckToolHolder(bool check) {
    m_checkToolHolder = check;
}

void CollisionDetector::setCheckWorkpiece(bool check) {
    m_checkWorkpiece = check;
}

void CollisionDetector::setCheckAirStrike(bool check) {
    m_checkAirStrike = check;
}

std::vector<CollisionInfo> CollisionDetector::detectCollisions(const Path::Toolpath& toolpath) {
    m_collisions.clear();

    if (m_workpiece.IsNull() && m_stockShape.IsNull()) {
        return m_collisions;
    }

    const auto& points = toolpath.points();
    const double safeZ = 100.0;

    for (size_t i = 0; i < points.size(); ++i) {
        const auto& pt = points[i];
        const auto& pos = pt.position;

        if (pos.Z() > safeZ * 0.9) {
            continue;
        }

        double distToWorkpiece = calculateMinimumDistance(pos, m_workpiece);
        double distToStock = calculateMinimumDistance(pos, m_stockShape);
        double minDist = std::min(distToWorkpiece, distToStock);

        if (minDist < m_collisionTolerance) {
            CollisionInfo collision;
            collision.pointIndex = static_cast<int>(i);
            collision.position = pos;
            collision.distance = minDist;
            collision.severity = (m_collisionTolerance - minDist) / m_collisionTolerance;

            if (m_checkToolHolder) {
                bool holderCollision = checkToolHolderCollision(pos, m_workpiece);
                if (holderCollision) {
                    collision.type = CollisionType::ToolHolder;
                    collision.description = "刀具柄部碰撞";
                    m_collisions.push_back(collision);
                    continue;
                }
            }

            if (minDist < m_toolGeometry.diameter / 2.0) {
                collision.type = CollisionType::ToolFlute;
                collision.description = "刀具切削部碰撞";
                collision.severity = 1.0;
                m_collisions.push_back(collision);
            }
        }

        if (m_checkAirStrike && pt.moveType == Path::MoveType::Rapid) {
            if (i > 0) {
                const auto& prevPos = points[i - 1].position;
                if (prevPos.Z() < safeZ && pos.Z() < safeZ) {
                    double rapidDist = pos.Distance(prevPos);
                    if (rapidDist > 20.0) {
                        CollisionInfo collision;
                        collision.pointIndex = static_cast<int>(i);
                        collision.position = pos;
                        collision.type = CollisionType::AirStrike;
                        collision.description = "快速移动碰撞风险";
                        collision.severity = 0.8;
                        m_collisions.push_back(collision);
                    }
                }
            }
        }
    }

    return m_collisions;
}

bool CollisionDetector::isPointInCollision(const gp_Pnt& point, double zCoord) {
    if (!m_workpiece.IsNull()) {
        BRepClass3d_SolidClassifier classifier(m_workpiece);
        classifier.Perform(point, 1e-6);
        if (classifier.State() == TopAbs_IN) {
            return true;
        }
    }
    return false;
}

double CollisionDetector::calculateMinimumDistance(const gp_Pnt& point, const TopoDS_Shape& shape) {
    if (shape.IsNull()) {
        return 1e10;
    }

    try {
        BRepExtrema_ExtPN extrema(BRepBuilderAPI_MakeVertex(point).Vertex(), shape);
        if (extrema.IsDone() && extrema.NbSolution() > 0) {
            return extrema.SquareDistance(1);
        }
    } catch (...) {
    }

    Bnd_Box bbox;
    BRepBndLib::Add(shape, bbox);
    Standard_Real xMin, yMin, zMin, xMax, yMax, zMax;
    bbox.Get(xMin, yMin, zMin, xMax, yMax, zMax);

    double dx = 0.0, dy = 0.0, dz = 0.0;
    if (point.X() < xMin) dx = xMin - point.X();
    else if (point.X() > xMax) dx = point.X() - xMax;
    if (point.Y() < yMin) dy = yMin - point.Y();
    else if (point.Y() > yMax) dy = point.Y() - yMax;
    if (point.Z() < zMin) dz = zMin - point.Z();
    else if (point.Z() > zMax) dz = point.Z() - zMax;

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool CollisionDetector::checkToolHolderCollision(const gp_Pnt& point, const TopoDS_Shape& shape) {
    if (!m_checkToolHolder || shape.IsNull()) {
        return false;
    }

    gp_Pnt holderBase = calculateHolderPosition(point, 0.0);

    double distToCenter = calculateMinimumDistance(holderBase, shape);
    double holderRadius = m_toolGeometry.holderDiameter / 2.0;

    return distToCenter < holderRadius + m_safeClearance;
}

bool CollisionDetector::checkToolFluteCollision(const gp_Pnt& point, const TopoDS_Shape& shape) {
    if (!m_checkWorkpiece || shape.IsNull()) {
        return false;
    }

    double distToWorkpiece = calculateMinimumDistance(point, shape);
    double toolRadius = m_toolGeometry.diameter / 2.0;

    return distToWorkpiece < toolRadius;
}

gp_Pnt CollisionDetector::calculateToolTipPosition(const gp_Pnt& feedPoint, double cutDepth) {
    return feedPoint;
}

gp_Pnt CollisionDetector::calculateHolderPosition(const gp_Pnt& feedPoint, double safeZ) {
    double holderTopZ = safeZ + m_toolGeometry.holderLength;
    return gp_Pnt(feedPoint.X(), feedPoint.Y(), holderTopZ);
}

void CollisionDetector::clear() {
    m_collisions.clear();
}

}
