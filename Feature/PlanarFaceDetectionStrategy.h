#pragma once

#include "FeatureDetectionStrategy.h"

namespace PathForge::Feature {
    // 平面面的识别策略
    class PlanarFaceDetectionStrategy : public FeatureDetectionStrategy {
    public:
        FeatureType detectFeature(const TopoDS_Face& face,
            const std::vector<int>& neighbors,
            const AdjacencyGraph& graph) const override {
            return FeatureType::PlanarFace;
        }
    };
}
