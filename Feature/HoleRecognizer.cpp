#include "HoleRecognizer.h"

#include "../Geometry/FaceClassifier.h"
#include "../Topology/FaceCollector.h"
#include "../Topology/AdjacencyGraph.h"

#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>

#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>

#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>

#include <algorithm>
#include <cmath>

namespace PathForge::Feature {

HoleRecognizer::HoleRecognizer() = default;

HoleRecognizer::HoleRecognizer(const TopoDS_Shape& shape) : FeatureRecognizer(shape) {}

bool HoleRecognizer::recognize(const TopoDS_Shape& shape, FeatureList& features) {
    if (shape.IsNull()) {
        m_lastError = "Shape is null";
        return false;
    }

    m_holes.clear();
    auto faces = Topology::FaceCollector::collectFaces(shape);
    Geometry::FaceClassifier classifier;

    std::map<gp_Pnt, std::vector<TopoDS_Face>> holeCandidates;

    for (const auto& face : faces) {
        if (isCylindricalHole(face)) {
            double radius = calculateHoleRadius(face);

            if (radius < m_minHoleRadius || radius > m_maxHoleRadius) {
                continue;
            }

            GProp_GProps props;
            BRepGProp::SurfaceProperties(face, props);
            gp_Pnt centroid = props.CentreOfMass();

            holeCandidates[centroid].push_back(face);
        }
    }

    int holeId = 0;
    for (const auto& [centroid, faceList] : holeCandidates) {
        if (faceList.size() >= 2) {
            auto hole = std::make_shared<HoleData>();
            hole->id = holeId++;
            hole->name = "Hole-" + std::to_string(hole->id);
            hole->type = FeatureType::Hole;
            hole->machiningType = MachiningType::Drilling;
            hole->faces = faceList;
            hole->centroid = centroid;

            double avgRadius = 0.0;
            for (const auto& face : faceList) {
                avgRadius += calculateHoleRadius(face);
            }
            hole->diameter = 2.0 * avgRadius / faceList.size();
            hole->holeDepth = calculateHoleDepth(shape, faceList.front());
            hole->through = isHoleThrough(shape, faceList.front());

            Bnd_Box bbox;
            for (const auto& face : faceList) {
                BRepBndLib::Add(face, bbox);
            }
            Standard_Real xMin, yMin, zMin, xMax, yMax, zMax;
            bbox.Get(xMin, yMin, zMin, xMax, yMax, zMax);
            hole->bbox = {xMin, xMax, yMin, yMax, zMin, zMax};

            hole->drillPoints.push_back(centroid);
            m_holes.push_back(hole);
            features.push_back(hole);
        }
    }

    return !m_holes.empty();
}

bool HoleRecognizer::isCylindricalHole(const TopoDS_Face& face) const {
    Geometry::FaceClassifier classifier;
    auto surfaceType = classifier.classifySurfaceType(face);
    return surfaceType == GeomAbs_Cylinder;
}

double HoleRecognizer::calculateHoleRadius(const TopoDS_Face& face) const {
    BRepAdaptor_Surface adaptor(face);
    if (adaptor.GetType() == GeomAbs_Cylinder) {
        return adaptor.Cylinder().Radius();
    }
    return 0.0;
}

double HoleRecognizer::calculateHoleDepth(const TopoDS_Shape& shape, const TopoDS_Face& face) const {
    Bnd_Box shapeBbox;
    BRepBndLib::Add(shape, shapeBbox);
    Standard_Real xMin, yMin, zMin, xMax, yMax, zMax;
    shapeBbox.Get(xMin, yMin, zMin, xMax, yMax, zMax);

    Bnd_Box faceBbox;
    BRepBndLib::Add(face, faceBbox);
    Standard_Real fxMin, fyMin, fzMin, fxMax, fyMax, fzMax;
    faceBbox.Get(fxMin, fyMin, fzMin, fxMax, fyMax, fzMax);

    double faceZ = (fzMin + fzMax) / 2.0;

    if (faceZ > (zMin + zMax) / 2.0) {
        return zMax - fzMin;
    } else {
        return fzMax - zMin;
    }
}

bool HoleRecognizer::isHoleThrough(const TopoDS_Shape& shape, const TopoDS_Face& face) const {
    Bnd_Box shapeBbox;
    BRepBndLib::Add(shape, shapeBbox);
    Standard_Real xMin, yMin, zMin, xMax, yMax, zMax;
    shapeBbox.Get(xMin, yMin, zMin, xMax, yMax, zMax);

    Bnd_Box faceBbox;
    BRepBndLib::Add(face, faceBbox);
    Standard_Real fzMin, fzMax;
    faceBbox.Get(xMin, yMin, fzMin, xMax, yMax, fzMax);

    double tolerance = 0.1;
    if (std::abs(fzMin - zMin) < tolerance || std::abs(fzMax - zMax) < tolerance) {
        return true;
    }
    return false;
}

}
