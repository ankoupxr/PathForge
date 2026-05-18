#include "Axis3Plus2Positioning.h"

#include "../Strategies/Cam2DFaceMillingStrategy.h"
#include "../../Geometry/FaceClassifier.h"

#include <BRep_Tool.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <Geom_Plane.hxx>

#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Vec.hxx>
#include <gp_Ax1.hxx>
#include <gp_Trsf.hxx>

#include <algorithm>
#include <cmath>

namespace PathForge::MultiAxis {

Axis3Plus2Positioning::Axis3Plus2Positioning() {
    m_config.configuration = AxisConfiguration::Axis3Plus2;
    m_config.outputA = true;
    m_config.outputB = true;
    m_config.outputC = false;
}

void Axis3Plus2Positioning::setConfiguration(const MultiAxisConfig& config) {
    m_config = config;
}

void Axis3Plus2Positioning::setToolAxisController(ToolAxisController* controller) {
    m_toolAxisController = controller;
}

void Axis3Plus2Positioning::addMachiningPlane(const TopoDS_Face& face) {
    auto plane = createMachiningPlane(face);
    if (plane.isMachiningPlane()) {
        m_machiningPlanes.push_back(plane);
    }
}

void Axis3Plus2Positioning::clearMachiningPlanes() {
    m_machiningPlanes.clear();
}

MultiAxisPath Axis3Plus2Positioning::generatePositioningPath(const gp_Pnt& start, const gp_Pnt& end) {
    MultiAxisPath path;

    if (m_machiningPlanes.empty()) {
        MultiAxisPoint startPoint(start);
        startPoint.setAxisAngles(0.0, 0.0, 0.0);
        startPoint.feedrate = m_positioningFeedrate;
        path.addPoint(startPoint);

        MultiAxisPoint endPoint(end);
        endPoint.setAxisAngles(0.0, 0.0, 0.0);
        endPoint.feedrate = m_positioningFeedrate;
        path.addPoint(endPoint);

        return path;
    }

    for (const auto& plane : m_machiningPlanes) {
        double aAngle, bAngle;
        m_toolAxisController->calculateTiltAngles(plane.normal, aAngle, bAngle);

        gp_Pnt safePosition(plane.getCentroid().X(), plane.getCentroid().Y(), 
                          plane.getCentroid().Z() + 50.0);

        MultiAxisPoint retractPoint(safePosition);
        retractPoint.setAxisAngles(0.0, 0.0, 0.0);
        retractPoint.feedrate = m_positioningFeedrate;
        retractPoint.isKeyPoint = true;
        path.addPoint(retractPoint);

        MultiAxisPoint positionedPoint(plane.getCentroid());
        positionedPoint.setAxisAngles(aAngle, bAngle, 0.0);
        positionedPoint.feedrate = m_positioningFeedrate;
        positionedPoint.isKeyPoint = true;
        path.addPoint(positionedPoint);
    }

    return path;
}

void Axis3Plus2Positioning::setPositioningFeedrate(double feedrate) {
    m_positioningFeedrate = feedrate;
}

bool Axis3Plus2Positioning::isValid() const {
    if (!m_toolAxisController) {
        m_lastError = "Tool axis controller not set";
        return false;
    }
    return true;
}

Axis3Plus2Positioning::MachiningPlane Axis3Plus2Positioning::createMachiningPlane(const TopoDS_Face& face) {
    MachiningPlane plane;
    plane.face = face;

    BRepAdaptor_Surface adaptor(face);
    if (adaptor.GetType() == GeomAbs_Plane) {
        Handle(Geom_Plane) geomPlane = adaptor.Plane();
        plane.normal = geomPlane->Axis().Direction();

        if (plane.normal.Z() < 0) {
            plane.normal.Reverse();
        }

        plane.rotationAxis = gp_Ax1(gp_Pnt(0, 0, 0), plane.normal);
        plane.angle = 0.0;
    }

    return plane;
}

bool Axis3Plus2Positioning::MachiningPlane::isMachiningPlane() const {
    return !face.IsNull();
}

gp_Pnt Axis3Plus2Positioning::MachiningPlane::getCentroid() const {
    GProp_GProps props;
    BRepGProp::SurfaceProperties(face, props);
    return props.CentreOfMass();
}

double Axis3Plus2Positioning::calculateRotationAngle(const gp_Dir& currentNormal, 
                                                    const gp_Dir& targetNormal) {
    gp_Vec current(currentNormal.X(), currentNormal.Y(), currentNormal.Z());
    gp_Vec target(targetNormal.X(), targetNormal.Y(), targetNormal.Z());

    double dot = current.Dot(target);
    dot = std::max(-1.0, std::min(1.0, dot));

    return std::acos(dot) * 180.0 / M_PI;
}

gp_Ax1 Axis3Plus2Positioning::calculateRotationAxis(const gp_Dir& currentNormal, 
                                                    const gp_Dir& targetNormal) {
    gp_Vec normal(targetNormal.X(), targetNormal.Y(), targetNormal.Z());
    gp_Vec perp = normal.Crossed(gp_Vec(0, 0, 1));

    if (perp.Magnitude() < 0.001) {
        perp = normal.Crossed(gp_Vec(0, 1, 0));
    }

    return gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(perp.X(), perp.Y(), perp.Z()));
}

MultiAxisPoint Axis3Plus2Positioning::createPositioningPoint(const gp_Pnt& position, 
                                                              const gp_Dir& axis,
                                                              double aAngle, double bAngle) {
    MultiAxisPoint point(position);
    point.toolAxis = axis;
    point.setAxisAngles(aAngle, bAngle, 0.0);
    point.feedrate = m_positioningFeedrate;
    point.isKeyPoint = true;
    return point;
}

}
