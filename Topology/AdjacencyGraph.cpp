#include "AdjacencyGraph.h"

#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopAbs.hxx>
#include <TopExp_Explorer.hxx>

#include <algorithm>

namespace PathForge::Topology {

    AdjacencyGraph::AdjacencyGraph(const TopoDS_Shape& shape)
    {
        build(shape);
    }

    void AdjacencyGraph::build(const TopoDS_Shape& shape)
    {
        m_faces.clear();
        m_adj.clear();

        // ============================================================
        // 1. 收集 Faces（IndexedMap 保证唯一性）
        // ============================================================
        TopTools_IndexedMapOfShape faceMap;
        TopExp::MapShapes(shape, TopAbs_FACE, faceMap);

        const int faceCount = faceMap.Extent();
        m_faces.reserve(faceCount);

        for (int i = 1; i <= faceCount; ++i) {
            m_faces.push_back(TopoDS::Face(faceMap(i)));
            m_adj[i - 1] = {};   // 显式初始化邻接表
        }

        // ============================================================
        // 2. Edge → Faces 映射
        // ============================================================
        TopTools_IndexedDataMapOfShapeListOfShape edgeToFaces;
        TopExp::MapShapesAndAncestors(
            shape,
            TopAbs_EDGE,
            TopAbs_FACE,
            edgeToFaces
        );

        // ============================================================
        // 3. 构建 Face 邻接关系（双向）
        // ============================================================
        for (int i = 1; i <= faceCount; ++i) {

            int faceIdx = i - 1;
            const TopoDS_Face& face = m_faces[faceIdx];

            for (TopExp_Explorer eexp(face, TopAbs_EDGE); eexp.More(); eexp.Next()) {

                const TopoDS_Shape& edge = eexp.Current();
                if (!edgeToFaces.Contains(edge))
                    continue;

                const TopTools_ListOfShape& owners =
                    edgeToFaces.FindFromKey(edge);

                for (TopTools_ListIteratorOfListOfShape it(owners); it.More(); it.Next()) {

                    int ownerIndex = faceMap.FindIndex(it.Value());
                    if (ownerIndex <= 0)
                        continue;

                    int ownerIdx0 = ownerIndex - 1;
                    if (ownerIdx0 == faceIdx)
                        continue;

                    auto& adjA = m_adj[faceIdx];
                    auto& adjB = m_adj[ownerIdx0];

                    if (std::find(adjA.begin(), adjA.end(), ownerIdx0) == adjA.end())
                        adjA.push_back(ownerIdx0);

                    if (std::find(adjB.begin(), adjB.end(), faceIdx) == adjB.end())
                        adjB.push_back(faceIdx);
                }
            }
        }
    }

} // namespace PathForge::Topology
