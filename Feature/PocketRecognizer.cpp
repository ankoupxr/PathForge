#include "PocketRecognizer.h"

#include "../Geometry/FaceClassifier.h"
#include "../Topology/FaceCollector.h"
#include "../Topology/AdjacencyGraph.h"

#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <GeomAbs_SurfaceType.hxx>

#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>

#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>

#include <algorithm>
#include <cmath>

namespace PathForge::Feature {

PocketRecognizer::PocketRecognizer() = default;

PocketRecognizer::PocketRecognizer(const TopoDS_Shape& shape) : FeatureRecognizer(shape) {}

bool PocketRecognizer::recognize(const TopoDS_Shape& shape, FeatureList& features) {
    if (shape.IsNull()) {
        m_lastError = "Shape is null";
        return false;
    }

    m_pockets.clear();
    auto faces = Topology::FaceCollector::collectFaces(shape);
    Geometry::FaceClassifier classifier;

    std::vector<TopoDS_Face> bottomFaces;
    std::vector<TopoDS_Face> sideFaces;

    for (const auto& face : faces) {
        auto surfaceType = classifier.classifySurfaceType(face);

        if (surfaceType == GeomAbs_Plane) {
            GProp_GProps props;
            BRepGProp::SurfaceProperties(face, props);
            gp_Vec normal = classifier.faceNormal(face);

            if (normal.Z() < -0.9 || normal.Z() > 0.9) {
                bottomFaces.push_back(face);
            } else {
                sideFaces.push_back(face);
            }
        }
    }

    if (bottomFaces.size() >= 1 && sideFaces.size() >= 3) {
        Bnd_Box bbox;
        for (const auto& face : bottomFaces) {
            BRepBndLib::Add(face, bbox);
        }
        for (const auto& face : sideFaces) {
            BRepBndLib::Add(face, bbox);
        }

        Standard_Real xMin, yMin, zMin, xMax, yMax, zMax;
        bbox.Get(xMin, yMin, zMin, xMax, yMax, zMax);

        double width = xMax - xMin;
        double height = yMax - yMin;
        double depth = zMax - zMin;

        double area = width * height;
        if (area >= m_minPocketArea && depth >= m_minPocketDepth) {
            auto pocket = std::make_shared<PocketData>();
            pocket->id = m_pockets.size();
            pocket->name = "Pocket-" + std::to_string(pocket->id);
            pocket->type = FeatureType::Pocket;
            pocket->machiningType = MachiningType::PocketMilling;
            pocket->faces = bottomFaces;
            pocket->faces.insert(pocket->faces.end(), sideFaces.begin(), sideFaces.end());
            pocket->pocketWidth = width;
            pocket->pocketHeight = height;
            pocket->pocketDepth = depth;
            pocket->bbox = {xMin, xMax, yMin, yMax, zMin, zMax};
            pocket->floorThickness = zMax - zMin;
            pocket->hasFloor = true;
            pocket->hasWall = true;
            pocket->hasOpenSide = false;

            GProp_GProps props;
            BRepGProp::SurfaceProperties(bottomFaces.front(), props);
            pocket->centroid = props.CentreOfMass();

            m_pockets.push_back(pocket);
            features.push_back(pocket);
        }
    }

    return !m_pockets.empty();
}

bool PocketRecognizer::analyzePocketGeometry(const std::vector<TopoDS_Face>& faces,
                                             double& width, double& height, double& depth) {
    if (faces.empty()) return false;

    Bnd_Box bbox;
    for (const auto& face : faces) {
        BRepBndLib::Add(face, bbox);
    }

    Standard_Real xMin, yMin, zMin, xMax, yMax, zMax;
    bbox.Get(xMin, yMin, zMin, xMax, yMax, zMax);

    width = xMax - xMin;
    height = yMax - yMin;
    depth = zMax - zMin;

    return width > 0 && height > 0 && depth > 0;
}

}
