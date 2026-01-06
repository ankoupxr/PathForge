#pragma once

#include <vector>
#include <unordered_map>
#include <cstdint>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <cstdint>

namespace PathForge {
    namespace Topology {

        /**
         * 收集 Shape 中所有 Face，并提供 Face 索引化函数。
         */
        class FaceCollector {
        public:
            FaceCollector() = default;
            ~FaceCollector() = default;

            static int ShapeHash(const TopoDS_Shape& s)
            {
                std::uintptr_t ptrValue = reinterpret_cast<std::uintptr_t>(s.TShape().get());
                return static_cast<int>(ptrValue % 1000000);   // 取模避免过大
            }

            /// 遍历 shape，收集所有 face（按出现顺序）
            static std::vector<TopoDS_Face> collectFaces(const TopoDS_Shape& shape);

            /// 将 faces 编号（返回 map: faceHash -> index）
            static std::unordered_map<int, int> indexFaces(const std::vector<TopoDS_Face>& faces);
        };

    } // namespace Topology
} // namespace PathForge