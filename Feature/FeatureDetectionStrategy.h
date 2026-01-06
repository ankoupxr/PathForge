#pragma once

#include <TopoDS_Face.hxx>
#include <vector>
#include "AdjacencyGraph.h"
#include "FeatureType.h"


namespace PathForge::Feature {
    enum class FeatureType {
        PlanarFace,
        Cylinder,
        Cone,
        Hole,
        Pocket,
        Slot,
        Chamfer,
        Fillet,
        Unknown
    };

    // 抽象基类，所有的特征识别策略都应该继承自它
    class FeatureDetectionStrategy {
    public:
        virtual ~FeatureDetectionStrategy() = default;

        // 提取特征的纯虚函数
        virtual FeatureType detectFeature(const TopoDS_Face& face,
            const std::vector<int>& neighbors,
            const AdjacencyGraph& graph) const = 0;
    };

}
