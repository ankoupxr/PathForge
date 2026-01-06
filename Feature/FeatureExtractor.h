#pragma once

#include "FeatureDetectionStrategy.h"
#include <memory>
#include <vector>
#include <map>
#include <AdjacencyGraph.h>
#include "Feature.h"

using namespace PathForge::Topology;

namespace PathForge {
    namespace Feature {
        class FeatureExtractor {
        public:
            FeatureExtractor();

            // 注册特征识别策略
            void registerStrategy(FeatureType type, std::shared_ptr<FeatureDetectionStrategy> strategy);

            // 根据面和邻接关系提取特征
            FeatureType extractFeature(const TopoDS_Face& face,
                const std::vector<int>& neighbors,
                const AdjacencyGraph& graph);

            std::vector<Feature> detectFeatures(const TopoDS_Shape& shape, const AdjacencyGraph& graph);
            
            FeatureType classifyFaceType(const TopoDS_Face& face);

        private:
            // 存储特征识别策略
            std::map<FeatureType, std::shared_ptr<FeatureDetectionStrategy>> strategies_;
        };
    }
}