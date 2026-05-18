#include "MaterialRemovalSimulator.h"

#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepBuilderAPI_MakeCylinder.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>

#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>

#include <gp_Cylinder.hxx>
#include <gp_Circ.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax3.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>

#include <algorithm>
#include <cmath>

namespace PathForge::Simulation {

MaterialRemovalSimulator::MaterialRemovalSimulator() = default;

void MaterialRemovalSimulator::setStockShape(const TopoDS_Shape& stock) {
    m_stockShape = stock;
    m_simulatedShape = TopoDS_Shape();
}

void MaterialRemovalSimulator::setToolGeometry(double diameter, double cornerRadius) {
    m_toolDiameter = diameter;
    m_toolCornerRadius = cornerRadius;
}

void MaterialRemovalSimulator::setLayerHeight(double height) {
    m_layerHeight = height;
}

void MaterialRemovalSimulator::setSimulationResolution(int resolution) {
    m_simulationResolution = resolution;
}

TopoDS_Shape MaterialRemovalSimulator::simulate(const Path::Toolpath& toolpath) {
    m_simulatedShape = m_stockShape;

    if (m_stockShape.IsNull() || toolpath.isEmpty()) {
        return m_simulatedShape;
    }

    const auto& points = toolpath.points();
    double totalLength = toolpath.totalLength();

    std::map<int, double> layerRemoved;

    for (size_t i = 1; i < points.size(); ++i) {
        const auto& p1 = points[i - 1];
        const auto& p2 = points[i];

        if (p2.motionType == Path::MotionType::Cutting ||
            p2.motionType == Path::MotionType::Plunge) {

            double stepLength = p2.position.Distance(p1.position);
            int numSteps = std::max(1, static_cast<int>(stepLength / 0.5));

            for (int s = 0; s <= numSteps; ++s) {
                double t = static_cast<double>(s) / numSteps;
                double x = p1.position.X() + t * (p2.position.X() - p1.position.X());
                double y = p1.position.Y() + t * (p2.position.Y() - p1.position.Y());
                double z = p1.position.Z() + t * (p2.position.Z() - p1.position.Z());

                int layerZ = static_cast<int>(std::floor(z / m_layerHeight));
                double radius = m_toolDiameter / 2.0;

                layerRemoved[layerZ] += M_PI * radius * radius * stepLength / numSteps;
            }
        }
    }

    double totalRemoved = 0.0;
    for (const auto& [layer, volume] : layerRemoved) {
        totalRemoved += volume;
        m_layerVolumes.push_back(volume);
    }

    m_removedVolume = totalRemoved;

    updateStatistics();

    return m_simulatedShape;
}

TopoDS_Shape MaterialRemovalSimulator::simulateWithProgress(const Path::Toolpath& toolpath) {
    return simulate(toolpath);
}

TopoDS_Shape MaterialRemovalSimulator::calculateToolSweptVolume(const gp_Pnt& p1, const gp_Pnt& p2, double radius) {
    try {
        double height = p1.Distance(p2);
        if (height < 0.001) {
            height = 0.001;
        }

        gp_Pnt center((p1.X() + p2.X()) / 2, (p1.Y() + p2.Y()) / 2, (p1.Z() + p2.Z()) / 2);
        gp_Dir dir(p2.X() - p1.X(), p2.Y() - p1.Y(), p2.Z() - p1.Z());
        if (dir.Magnitude() < 0.001) {
            dir = gp_Dir(0, 0, 1);
        }
        dir.Normalize();

        gp_Ax3 axis(center, dir);
        BRepBuilderAPI_MakeCylinder makeCylinder(gp_Cylinder(axis, radius), height);
        if (makeCylinder.IsDone()) {
            return makeCylinder.Shape();
        }
    } catch (...) {
    }

    return TopoDS_Shape();
}

double MaterialRemovalSimulator::calculateCylinderVolume(const gp_Pnt& center, double radius, double height) {
    return M_PI * radius * radius * height;
}

void MaterialRemovalSimulator::updateStatistics() {
    calculateVolumes();
}

void MaterialRemovalSimulator::calculateVolumes() {
    if (!m_stockShape.IsNull()) {
        GProp_GProps props;
        BRepGProp::VolumeProperties(m_stockShape, props);
        double stockVolume = props.Mass();
        m_remainingVolume = stockVolume - m_removedVolume;

        if (stockVolume > 0.001) {
            m_removalPercentage = (m_removedVolume / stockVolume) * 100.0;
        }
    }
}

void MaterialRemovalSimulator::clear() {
    m_simulatedShape = TopoDS_Shape();
    m_removedVolume = 0.0;
    m_remainingVolume = 0.0;
    m_removalPercentage = 0.0;
    m_layerVolumes.clear();
}

}
