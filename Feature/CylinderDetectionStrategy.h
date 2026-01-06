#pragma once

#include "FeatureDetectionStrategy.h"


namespace PathForge::Feature {
    // 圆柱面的识别策略
    class CylinderDetectionStrategy : public FeatureDetectionStrategy {
    public:
        FeatureType detectFeature(const TopoDS_Face& face,
            const std::vector<int>& neighbors,
            const AdjacencyGraph& graph) const override {
            // 可以进一步通过邻接关系判断是否为孔、口袋等
            if (neighbors.size() == 2) {
                return FeatureType::Hole;  // 圆柱 + 两个平面 -> 通孔
            }
            else if (neighbors.size() == 1) {
                return FeatureType::Pocket;  // 盲孔
            }
            else if (neighbors.size() > 2) {
                return FeatureType::Slot;  // 多个邻接面 -> 槽
            }
            return FeatureType::Cylinder;
        }
    };
}
