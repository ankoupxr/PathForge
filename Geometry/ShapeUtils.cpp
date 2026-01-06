#include "ShapeUtils.h"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopAbs.hxx>
#include <TopExp.hxx>

namespace PathForge {
    namespace Geometry {

        double ShapeUtils::faceArea(const TopoDS_Face& face) {
            GProp_GProps props;
            BRepGProp::SurfaceProperties(face, props);
            return props.Mass(); // ±íÃæ»ý
        }

        gp_Pnt ShapeUtils::faceCentroid(const TopoDS_Face& face) {
            GProp_GProps props;
            BRepGProp::SurfaceProperties(face, props);
            return props.CentreOfMass();
        }

        int ShapeUtils::faceEdgeCount(const TopoDS_Face& face) {
            TopTools_IndexedMapOfShape edgeMap;
            TopExp::MapShapes(face, TopAbs_EDGE, edgeMap);
            return edgeMap.Extent();
        }

    } // namespace Geometry
} // namespace PathForge
