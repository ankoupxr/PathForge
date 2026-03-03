#include "EnvelopeExtractor.h"
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <gp_Pln.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <BRep_Tool.hxx>
#include <ShapeAnalysis_Surface.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <vector>
#include <algorithm>

namespace PathForge::Geometry {

    TopoDS_Shape EnvelopeExtractor::extractEnvelope(
        const TopoDS_Shape& shape,
        const DepthRange& depthRange)
    {
        if (shape.IsNull()) {
            return TopoDS_Shape();
        }

        TopoDS_Shape projection = projectToXY(shape);
        if (projection.IsNull()) {
            return TopoDS_Shape();
        }

        double extrusionHeight = depthRange.maxZ - depthRange.minZ;
        TopoDS_Shape extruded = extrudeToDepth(projection, extrusionHeight, depthRange.minZ);
        
        if (extruded.IsNull()) {
            return TopoDS_Shape();
        }

        TopoDS_Shape result = performBooleanIntersection(shape, extruded);
        return result;
    }

    TopoDS_Shape EnvelopeExtractor::extractPocketVolume(
        const TopoDS_Shape& shape,
        double pocketDepth,
        double safetyHeight)
    {
        if (shape.IsNull()) {
            return TopoDS_Shape();
        }

        Bnd_Box boundingBox;
        BRepBndLib::Add(shape, boundingBox);
        
        double minZ = boundingBox.CornerMin().Z();
        double maxZ = boundingBox.CornerMax().Z();
        
        double bottomZ = maxZ - pocketDepth;
        
        DepthRange range(bottomZ - safetyHeight, maxZ + safetyHeight);
        
        return extractEnvelope(shape, range);
    }

    std::vector<TopoDS_Shape> EnvelopeExtractor::extractMultiplePockets(
        const TopoDS_Shape& shape,
        const std::vector<DepthRange>& depthRanges)
    {
        std::vector<TopoDS_Shape> results;
        results.reserve(depthRanges.size());
        
        for (const auto& range : depthRanges) {
            TopoDS_Shape envelope = extractEnvelope(shape, range);
            if (!envelope.IsNull()) {
                results.push_back(envelope);
            }
        }
        
        return results;
    }

    TopoDS_Shape EnvelopeExtractor::projectToXY(const TopoDS_Shape& shape)
    {
        if (shape.IsNull()) {
            return TopoDS_Shape();
        }

        Bnd_Box boundingBox;
        BRepBndLib::Add(shape, boundingBox);
        
        double minZ = boundingBox.CornerMin().Z();
        double maxZ = boundingBox.CornerMax().Z();
        
        gp_Pln projectionPlane(gp_Pnt(0, 0, minZ), gp_Dir(0, 0, 1));
        
        TopoDS_Shape result;
        
        TopExp_Explorer exp(shape, TopAbs_FACE);
        std::vector<TopoDS_Face> bottomFaces;
        
        for (; exp.More(); exp.Next()) {
            TopoDS_Face face = TopoDS::Face(exp.Current());
            BRepAdaptor_Surface surface(face);
            
            if (surface.GetType() == GeomAbs_Plane) {
                const gp_Pln& plane = surface.Plane();
                gp_Dir normal = plane.Axis().Direction();
                
                if (std::abs(normal.Z()) > 0.99) {
                    double faceZ = plane.Position().Location().Z();
                    if (std::abs(faceZ - minZ) < 0.001) {
                        bottomFaces.push_back(face);
                    }
                }
            }
        }
        
        if (!bottomFaces.empty()) {
            TopoDS_Compound compound;
            TopoDS_Builder builder;
            builder.MakeCompound(compound);
            
            for (const auto& face : bottomFaces) {
                builder.Add(compound, face);
            }
            
            result = compound;
        }
        else {
            TopoDS_Shape projectedWire;
            
            TopExp_Explorer expEdge(shape, TopAbs_EDGE);
            std::vector<TopoDS_Edge> edges;
            
            for (; expEdge.More(); expEdge.Next()) {
                edges.push_back(TopoDS::Edge(expEdge.Current()));
            }
            
            if (!edges.empty()) {
                TopoDS_Compound compound;
                TopoDS_Builder builder;
                builder.MakeCompound(compound);
                
                for (const auto& edge : edges) {
                    builder.Add(compound, edge);
                }
                
                result = compound;
            }
        }
        
        if (result.IsNull()) {
            TopoDS_Face face = BRepBuilderAPI_MakeFace(projectionPlane);
            result = face;
        }
        
        return result;
    }

    TopoDS_Shape EnvelopeExtractor::extrudeToDepth(
        const TopoDS_Shape& projection,
        double depth,
        double startZ)
    {
        if (projection.IsNull() || depth <= 0) {
            return TopoDS_Shape();
        }
        
        TopoDS_Shape result;
        
        try {
            TopExp_Explorer exp(projection, TopAbs_FACE);
            
            if (exp.More()) {
                TopoDS_Face face = TopoDS::Face(exp.Current());
                
                if (!face.IsNull()) {
                    gp_Dir direction(0, 0, depth);
                    BRepPrimAPI_MakePrism prism(face, direction, true, true);
                    prism.Build();
                    
                    if (prism.IsDone()) {
                        result = prism.Shape();
                    }
                }
            }
            
            if (result.IsNull()) {
                TopExp_Explorer expWire(projection, TopAbs_WIRE);
                
                if (expWire.More()) {
                    TopoDS_Wire wire = TopoDS::Wire(expWire.Current());
                    
                    if (!wire.IsNull()) {
                        TopoDS_Face face = BRepBuilderAPI_MakeFace(wire);
                        
                        if (!face.IsNull()) {
                            gp_Dir direction(0, 0, depth);
                            BRepPrimAPI_MakePrism prism(face, direction, true, true);
                            prism.Build();
                            
                            if (prism.IsDone()) {
                                result = prism.Shape();
                            }
                        }
                    }
                }
            }
            
            if (result.IsNull()) {
                gp_Pln plane(gp_Pnt(0, 0, startZ), gp_Dir(0, 0, 1));
                TopoDS_Face face = BRepBuilderAPI_MakeFace(plane, -1000, 1000, -1000, 1000);
                
                if (!face.IsNull()) {
                    gp_Dir direction(0, 0, depth);
                    BRepPrimAPI_MakePrism prism(face, direction, true, true);
                    prism.Build();
                    
                    if (prism.IsDone()) {
                        result = prism.Shape();
                    }
                }
            }
        }
        catch (...) {
            return TopoDS_Shape();
        }
        
        return result;
    }

    TopoDS_Shape EnvelopeExtractor::performBooleanIntersection(
        const TopoDS_Shape& shape1,
        const TopoDS_Shape& shape2)
    {
        if (shape1.IsNull() || shape2.IsNull()) {
            return TopoDS_Shape();
        }
        
        try {
            BRepAlgoAPI_Common common(shape1, shape2);
            common.Build();
            
            if (common.IsDone()) {
                return common.Shape();
            }
        }
        catch (...) {
        }
        
        return TopoDS_Shape();
    }

    std::vector<TopoDS_Face> EnvelopeExtractor::getBottomFaces(
        const TopoDS_Shape& shape,
        double maxZ)
    {
        std::vector<TopoDS_Face> bottomFaces;
        
        TopExp_Explorer exp(shape, TopAbs_FACE);
        
        for (; exp.More(); exp.Next()) {
            TopoDS_Face face = TopoDS::Face(exp.Current());
            BRepAdaptor_Surface surface(face);
            
            if (surface.GetType() == GeomAbs_Plane) {
                const gp_Pln& plane = surface.Plane();
                gp_Dir normal = plane.Axis().Direction();
                
                if (std::abs(normal.Z()) > 0.99) {
                    double faceZ = plane.Position().Location().Z();
                    if (std::abs(faceZ - maxZ) < 0.001) {
                        bottomFaces.push_back(face);
                    }
                }
            }
        }
        
        return bottomFaces;
    }

    std::vector<TopoDS_Face> EnvelopeExtractor::getSideWallFaces(
        const TopoDS_Shape& shape,
        double minZ,
        double maxZ)
    {
        std::vector<TopoDS_Face> wallFaces;
        
        TopExp_Explorer exp(shape, TopAbs_FACE);
        
        for (; exp.More(); exp.Next()) {
            TopoDS_Face face = TopoDS::Face(exp.Current());
            BRepAdaptor_Surface surface(face);
            
            if (surface.GetType() == GeomAbs_Plane) {
                const gp_Pln& plane = surface.Plane();
                gp_Dir normal = plane.Axis().Direction();
                
                if (std::abs(normal.Z()) < 0.1) {
                    TopExp_Explorer expV(face, TopAbs_VERTEX);
                    
                    bool hasVertexInRange = false;
                    for (; expV.More(); expV.Next()) {
                        TopoDS_Vertex vertex = TopoDS::Vertex(expV.Current());
                        gp_Pnt p = BRep_Tool::Pnt(vertex);
                        
                        if (p.Z() >= minZ && p.Z() <= maxZ) {
                            hasVertexInRange = true;
                            break;
                        }
                    }
                    
                    if (hasVertexInRange) {
                        wallFaces.push_back(face);
                    }
                }
            }
            else if (surface.GetType() == GeomAbs_Cylinder) {
                wallFaces.push_back(face);
            }
        }
        
        return wallFaces;
    }

}
