#include "SlotRecognizer.h"

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

SlotRecognizer::SlotRecognizer() = default;

SlotRecognizer::SlotRecognizer(const TopoDS_Shape& shape) : FeatureRecognizer(shape) {}

bool SlotRecognizer::recognize(const TopoDS_Shape& shape, FeatureList& features) {
    if (shape.IsNull()) {
        m_lastError = "Shape is null";
        return false;
    }

    m_slots.clear();
    auto faces = Topology::FaceCollector::collectFaces(shape);
    Geometry::FaceClassifier classifier;

    std::vector<TopoDS_Face> planarFaces;
    std::vector<TopoDS_Face> cylindricalFaces;

    for (const auto& face : faces) {
        auto surfaceType = classifier.classifySurfaceType(face);
        if (surfaceType == GeomAbs_Plane) {
            planarFaces.push_back(face);
        } else if (surfaceType == GeomAbs_Cylinder) {
            cylindricalFaces.push_back(face);
        }
    }

    if (cylindricalFaces.size() >= 2) {
        for (size_t i = 0; i < cylindricalFaces.size(); ++i) {
            for (size_t j = i + 1; j < cylindricalFaces.size(); ++j) {
                auto& face1 = cylindricalFaces[i];
                auto& face2 = cylindricalFaces[j];

                BRepAdaptor_Surface adaptor1(face1);
                BRepAdaptor_Surface adaptor2(face2);

                if (adaptor1.GetType() == GeomAbs_Cylinder && adaptor2.GetType() == GeomAbs_Cylinder) {
                    double radius1 = adaptor1.Cylinder().Radius();
                    double radius2 = adaptor2.Cylinder().Radius();

                    if (std::abs(radius1 - radius2) < 0.1) {
                        GProp_GProps props1, props2;
                        BRepGProp::SurfaceProperties(face1, props1);
                        BRepGProp::SurfaceProperties(face2, props2);

                        double width = props1.CentreOfMass().Distance(props2.CentreOfMass());
                        if (width > m_minSlotWidth) {
                            auto slot = std::make_shared<SlotData>();
                            slot->id = m_slots.size();
                            slot->name = "Slot-" + std::to_string(slot->id);
                            slot->type = FeatureType::Slot;
                            slot->machiningType = MachiningType::PocketMilling;
                            slot->faces.push_back(face1);
                            slot->faces.push_back(face2);
                            slot->width = width;
                            slot->slotDepth = radius1;
                            slot->rectangular = true;

                            Bnd_Box bbox;
                            BRepBndLib::Add(face1, bbox);
                            BRepBndLib::Add(face2, bbox);
                            Standard_Real xMin, yMin, zMin, xMax, yMax, zMax;
                            bbox.Get(xMin, yMin, zMin, xMax, yMax, zMax);
                            slot->bbox = {xMin, xMax, yMin, yMax, zMin, zMax};
                            slot->length = xMax - xMin;

                            m_slots.push_back(slot);
                            features.push_back(slot);
                        }
                    }
                }
            }
        }
    }

    return !m_slots.empty();
}

bool SlotRecognizer::detectSlotGeometry(const std::vector<TopoDS_Face>& faces,
                                         double& width, double& depth, double& length) {
    if (faces.size() < 3) return false;

    Bnd_Box bbox;
    for (const auto& face : faces) {
        BRepBndLib::Add(face, bbox);
    }

    Standard_Real xMin, yMin, zMin, xMax, yMax, zMax;
    bbox.Get(xMin, yMin, zMin, xMax, yMax, zMax);

    width = yMax - yMin;
    depth = zMax - zMin;
    length = xMax - xMin;

    return width > m_minSlotWidth && depth > m_minSlotDepth;
}

}
