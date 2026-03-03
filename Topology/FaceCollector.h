// FaceCollector.h
#pragma once

#include <vector>
#include <unordered_map>
#include <cstdint>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>

namespace PathForge {
namespace Topology {

/**
 * @brief Collect faces from Shape objects and provide face indexing
 */
class FaceCollector {
public:
    FaceCollector() = default;
    ~FaceCollector() = default;

    static int ShapeHash(const TopoDS_Shape& s)
    {
        std::uintptr_t ptrValue = reinterpret_cast<std::uintptr_t>(s.TShape().get());
        return static_cast<int>(ptrValue % 1000000);
    }

    /// Collect all faces from shape, keeping original order
    static std::vector<TopoDS_Face> collectFaces(const TopoDS_Shape& shape);

    /// Index faces, build map: faceHash -> index
    static std::unordered_map<int, int> indexFaces(const std::vector<TopoDS_Face>& faces);
};

} // namespace Topology
} // namespace PathForge
