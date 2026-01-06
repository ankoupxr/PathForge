#pragma once

#include "FeatureType.h"
#include <vector>

namespace PathForge { namespace Topology { class AdjacencyGraph; } }

// Forward declaration of Open CASCADE face type (defined in TopoDS_Face.hxx)
class TopoDS_Face;

namespace PathForge::Feature {

    // 抽象基类，所有的特征识别策略都应该继承自它
    class FeatureDetectionStrategy {
    public:
        virtual ~FeatureDetectionStrategy() = default;

        // 提取特征的纯虚函数
        virtual FeatureType detectFeature(const TopoDS_Face& face,
            const std::vector<int>& neighbors,
            const PathForge::Topology::AdjacencyGraph& graph) const = 0;
    };

}