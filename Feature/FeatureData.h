#pragma once

#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <string>
#include <vector>
#include <memory>

namespace PathForge::Feature {

enum class FeatureType {
    Plane,
    Cylinder,
    Cone,
    Sphere,
    FreeForm,
    Hole,
    Slot,
    Pocket,
    Boss,
    Step,
    Unknown
};

enum class MachiningType {
    FaceMilling,
    ContourMilling,
    PocketMilling,
    Drilling,
    Engraving,
    Unknown
};

struct BoundingBox3D {
    double minX, maxX;
    double minY, maxY;
    double minZ, maxZ;

    double width() const { return maxX - minX; }
    double height() const { return maxY - minY; }
    double depth() const { return maxZ - minZ; }
    double diagonal() const { return std::sqrt(width() * width() + height() * height() + depth() * depth()); }
    gp_Pnt center() const { return gp_Pnt((minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2); }
};

struct FeatureData {
    int id;
    std::string name;
    FeatureType type;
    MachiningType machiningType;

    std::vector<TopoDS_Face> faces;
    std::vector<TopoDS_Edge> edges;

    double area;
    double volume;
    BoundingBox3D bbox;

    gp_Pnt centroid;
    gp_Dir normal;
    gp_Dir depthDirection;

    double sizeParam1;
    double sizeParam2;
    double depth;

    bool hasFloor;
    bool hasWall;
    bool hasOpenSide;

    std::string getTypeName() const;
    std::string getMachiningTypeName() const;
};

struct HoleData : FeatureData {
    double diameter;
    double holeDepth;
    bool through;
    bool tapered;
    double taperAngle;

    std::vector<gp_Pnt> drillPoints;
};

struct SlotData : FeatureData {
    double width;
    double slotDepth;
    double length;
    bool through;
    bool rectangular;
    double cornerRadius;
};

struct PocketData : FeatureData {
    double pocketWidth;
    double pocketHeight;
    double pocketDepth;
    double floorThickness;
    std::vector<gp_Pnt> boundaryPoints;
};

struct StepData : FeatureData {
    double stepHeight;
    double stepWidth;
    double stepDepth;
    bool onExternal;
};

using FeatureDataPtr = std::shared_ptr<FeatureData>;
using HoleDataPtr = std::shared_ptr<HoleData>;
using SlotDataPtr = std::shared_ptr<SlotData>;
using PocketDataPtr = std::shared_ptr<PocketData>;

using FeatureList = std::vector<FeatureDataPtr>;
using HoleList = std::vector<HoleDataPtr>;
using SlotList = std::vector<SlotDataPtr>;
using PocketList = std::vector<PocketDataPtr>;

}
