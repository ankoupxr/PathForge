#include "InterferenceChecker.h"

#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepExtrema_ExtPN.hxx>

#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Vec.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Circ.hxx>

#include <algorithm>
#include <cmath>

namespace PathForge::MultiAxis {

InterferenceChecker::InterferenceChecker() {
    m_config.checkInterference = true;
    m_config.collisionClearance = 1.0;
}

void InterferenceChecker::setConfiguration(const MultiAxisConfig& config) {
    m_config = config;
}

void InterferenceChecker::setWorkpiece(const TopoDS_Shape& workpiece) {
    m_workpiece = workpiece;
}

void InterferenceChecker::setStockShape(const TopoDS_Shape& stock) {
    m_stockShape = stock;
}

void InterferenceChecker::setToolGeometry(double diameter, double length, 
                                          double holderDiameter, double holderLength) {
    m_toolDiameter = diameter;
    m_toolLength = length;
    m_holderDiameter = holderDiameter;
    m_holderLength = holderLength;
}

void InterferenceChecker::setPivotHeight(double height) {
    m_pivotHeight = height;
}

std::vector<InterferenceInfo> InterferenceChecker::checkPath(const MultiAxisPath& path) {
    m_interferences.clear();

    const auto& points = path.getPoints();

    for (size_t i = 0; i < points.size(); ++i) {
        const auto& point = points[i];

        if (checkPointInterference(point.position, point.toolAxis)) {
            InterferenceInfo info;
            info.pointIndex = static_cast<int>(i);
            info.position = point.position;
            info.toolAxis = point.toolAxis;
            info.axisA = point.axisA;
            info.axisB = point.axisB;
            info.axisC = point.axisC;

            double clearance = calculateMinimumClearance(point.position, point.toolAxis);
            info.severity = (m_interferenceTolerance - clearance) / m_interferenceTolerance;

            m_interferences.push_back(info);
        }
    }

    return m_interferences;
}

bool InterferenceChecker::checkPointInterference(const gp_Pnt& position, const gp_Dir& axis) {
    if (checkHolderInterference(position, axis)) {
        return true;
    }

    if (checkSpindleInterference(position, axis)) {
        return true;
    }

    if (checkWorkpieceInterference(position, axis)) {
        return true;
    }

    return false;
}

double InterferenceChecker::calculateMinimumClearance(const gp_Pnt& position, const gp_Dir& axis) {
    if (!m_workpiece.IsNull()) {
        return calculateDistanceToWorkpiece(position);
    }
    return m_interferenceTolerance;
}

void InterferenceChecker::setInterferenceTolerance(double tolerance) {
    m_interferenceTolerance = tolerance;
}

void InterferenceChecker::clear() {
    m_interferences.clear();
}

bool InterferenceChecker::checkHolderInterference(const gp_Pnt& position, const gp_Dir& axis) {
    gp_Pnt holderEnd = calculateHolderEndPosition(position, axis);

    if (!m_workpiece.IsNull()) {
        double dist = calculateDistanceToWorkpiece(holderEnd);
        if (dist < m_holderDiameter / 2.0 + m_config.collisionClearance) {
            return true;
        }
    }

    return false;
}

bool InterferenceChecker::checkSpindleInterference(const gp_Pnt& position, const gp_Dir& axis) {
    gp_Pnt spindlePos = calculateSpindlePosition(position, axis);

    if (!m_workpiece.IsNull()) {
        double dist = calculateDistanceToWorkpiece(spindlePos);
        if (dist < m_holderDiameter / 2.0) {
            return true;
        }
    }

    return false;
}

bool InterferenceChecker::checkWorkpieceInterference(const gp_Pnt& position, const gp_Dir& axis) {
    if (m_workpiece.IsNull()) {
        return false;
    }

    gp_Vec toolVec(axis.X(), axis.Y(), axis.Z());
    gp_Vec negToolVec = -toolVec;

    gp_Pnt toolBase = position.Translated(negToolVec * m_toolLength);

    if (isPointInsideWorkpiece(toolBase)) {
        return true;
    }

    return false;
}

gp_Pnt InterferenceChecker::calculateHolderEndPosition(const gp_Pnt& toolTip, const gp_Dir& axis) {
    gp_Vec axisVec(axis.X(), axis.Y(), axis.Z());
    gp_Vec holderVec = axisVec * (m_toolLength + m_holderLength);
    return toolTip.Translated(holderVec);
}

gp_Pnt InterferenceChecker::calculateSpindlePosition(const gp_Pnt& toolTip, const gp_Dir& axis) {
    gp_Vec axisVec(axis.X(), axis.Y(), axis.Z());
    gp_Vec spindleVec = axisVec * (m_toolLength + m_holderLength + m_pivotHeight);
    return toolTip.Translated(spindleVec);
}

double InterferenceChecker::calculateDistanceToWorkpiece(const gp_Pnt& point) {
    if (m_workpiece.IsNull()) {
        return m_interferenceTolerance;
    }

    try {
        BRepExtrema_ExtPN extrema(BRepBuilderAPI_MakeVertex(point).Vertex(), m_workpiece);
        if (extrema.IsDone() && extrema.NbSolution() > 0) {
            return std::sqrt(extrema.SquareDistance(1));
        }
    } catch (...) {
    }

    Bnd_Box bbox;
    BRepBndLib::Add(m_workpiece, bbox);
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

bool InterferenceChecker::isPointInsideWorkpiece(const gp_Pnt& point) {
    if (m_workpiece.IsNull()) {
        return false;
    }

    BRepClass3d_SolidClassifier classifier(m_workpiece);
    classifier.Perform(point, 1e-6);
    return classifier.State() == TopAbs_IN;
}

}
