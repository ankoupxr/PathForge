#include "FaceCollector.h"

#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopAbs.hxx>
#include <TopoDS_Face.hxx>

namespace PathForge {
    namespace Topology {

        std::vector<TopoDS_Face> FaceCollector::collectFaces(const TopoDS_Shape& shape) const {
            std::vector<TopoDS_Face> faces;
            for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next()) {
                TopoDS_Face f = TopoDS::Face(exp.Current());
                faces.push_back(f);
            }
            return faces;
        }

        std::unordered_map<int, int> FaceCollector::indexFaces(const std::vector<TopoDS_Face>& faces) const {
            std::unordered_map<int, int> map;
            map.reserve(faces.size());
            for (size_t i = 0; i < faces.size(); ++i) {
                int key = ShapeHash(faces[i]); // ÓÃ TShape* identity ×öÎ¨Ò» key
                map.emplace(key, static_cast<int>(i));
            }
            return map;
        }
    } // namespace Topology
} // namespace PathForge
