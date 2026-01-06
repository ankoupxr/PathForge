#pragma once

#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

#include <vector>
#include <unordered_map>

namespace PathForge::Topology {

    class AdjacencyGraph
    {
    public:
        explicit AdjacencyGraph(const TopoDS_Shape& shape);

        const std::vector<TopoDS_Face>& getFaces() const { return m_faces; }
        const std::unordered_map<int, std::vector<int>>& getAdjacency() const { return m_adj; }

        std::size_t faceCount() const { return m_faces.size(); }

    private:
        void build(const TopoDS_Shape& shape);

    private:
        std::vector<TopoDS_Face> m_faces;                  // index -> face
        std::unordered_map<int, std::vector<int>> m_adj;  // faceIndex -> neighbor indices
    };

}
