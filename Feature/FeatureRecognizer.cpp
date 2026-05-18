#include "FeatureRecognizer.h"

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

#include <algorithm>
#include <cmath>

namespace PathForge::Feature {

FeatureRecognizer::FeatureRecognizer() = default;

FeatureRecognizer::FeatureRecognizer(const TopoDS_Shape& shape) : m_shape(shape) {}

bool FeatureRecognizer::validate() const {
    return !m_shape.IsNull();
}

PlaneRecognizer::PlaneRecognizer() = default;

PlaneRecognizer::PlaneRecognizer(const TopoDS_Shape& shape) : FeatureRecognizer(shape) {}

bool PlaneRecognizer::recognize(const TopoDS_Shape& shape, FeatureList& features) {
    if (shape.IsNull()) {
        m_lastError = "Shape is null";
        return false;
    }

    auto faces = Topology::FaceCollector::collectFaces(shape);
    Geometry::FaceClassifier classifier;

    int featureId = features.size();

    for (const auto& face : faces) {
        auto surfaceType = classifier.classifySurfaceType(face);

        if (surfaceType == GeomAbs_Plane) {
            auto feature = std::make_shared<FeatureData>();
            feature->id = featureId++;
            feature->name = "Plane-" + std::to_string(feature->id);
            feature->type = FeatureType::Plane;
            feature->machiningType = MachiningType::FaceMilling;
            feature->faces.push_back(face);

            GProp_GProps props;
            BRepGProp::SurfaceProperties(face, props);
            feature->area = props.Mass();
            feature->centroid = props.CentreOfMass();
            feature->normal = classifier.faceNormal(face);

            Bnd_Box bbox;
            BRepBndLib::Add(face, bbox);
            Standard_Real xMin, yMin, zMin, xMax, yMax, zMax;
            bbox.Get(xMin, yMin, zMin, xMax, yMax, zMax);
            feature->bbox = {xMin, xMax, yMin, yMax, zMin, zMax};

            features.push_back(feature);
        }
    }

    return true;
}

CylinderRecognizer::CylinderRecognizer() = default;

CylinderRecognizer::CylinderRecognizer(const TopoDS_Shape& shape) : FeatureRecognizer(shape) {}

bool CylinderRecognizer::recognize(const TopoDS_Shape& shape, FeatureList& features) {
    if (shape.IsNull()) {
        m_lastError = "Shape is null";
        return false;
    }

    auto faces = Topology::FaceCollector::collectFaces(shape);
    Geometry::FaceClassifier classifier;

    int featureId = features.size();

    for (const auto& face : faces) {
        auto surfaceType = classifier.classifySurfaceType(face);

        if (surfaceType == GeomAbs_Cylinder) {
            auto feature = std::make_shared<FeatureData>();
            feature->id = featureId++;
            feature->name = "Cylinder-" + std::to_string(feature->id);
            feature->type = FeatureType::Cylinder;
            feature->machiningType = MachiningType::Drilling;
            feature->faces.push_back(face);

            GProp_GProps props;
            BRepGProp::SurfaceProperties(face, props);
            feature->area = props.Mass();
            feature->centroid = props.CentreOfMass();
            feature->normal = classifier.faceNormal(face);

            BRepAdaptor_Surface adaptor(face);
            feature->sizeParam1 = adaptor.Cylinder().Radius();

            Bnd_Box bbox;
            BRepBndLib::Add(face, bbox);
            Standard_Real xMin, yMin, zMin, xMax, yMax, zMax;
            bbox.Get(xMin, yMin, zMin, xMax, yMax, zMax);
            feature->bbox = {xMin, xMax, yMin, yMax, zMin, zMax};
            feature->depth = zMax - zMin;

            features.push_back(feature);
        }
    }

    return true;
}

}
