#pragma once

#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <vector>
#include <memory>

namespace PathForge::Geometry {

    struct DepthRange {
        double minZ;
        double maxZ;
        
        DepthRange(double min = 0.0, double max = 10.0) : minZ(min), maxZ(max) {}
    };

    class EnvelopeExtractor {
    public:
        EnvelopeExtractor() = default;
        
        TopoDS_Shape extractEnvelope(
            const TopoDS_Shape& shape,
            const DepthRange& depthRange
        );
        
        TopoDS_Shape extractPocketVolume(
            const TopoDS_Shape& shape,
            double pocketDepth,
            double safetyHeight = 0.0
        );
        
        std::vector<TopoDS_Shape> extractMultiplePockets(
            const TopoDS_Shape& shape,
            const std::vector<DepthRange>& depthRanges
        );
        
        TopoDS_Shape projectToXY(const TopoDS_Shape& shape);
        
        TopoDS_Shape extrudeToDepth(
            const TopoDS_Shape& projection,
            double depth,
            double startZ = 0.0
        );
        
    private:
        TopoDS_Shape performBooleanIntersection(
            const TopoDS_Shape& shape1,
            const TopoDS_Shape& shape2
        );
        
        std::vector<TopoDS_Face> getBottomFaces(
            const TopoDS_Shape& shape,
            double maxZ
        );
        
        std::vector<TopoDS_Face> getSideWallFaces(
            const TopoDS_Shape& shape,
            double minZ,
            double maxZ
        );
    };

}
