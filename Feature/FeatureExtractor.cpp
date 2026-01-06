// FeatureExtractor.cpp
#include "FeatureExtractor.h"
#include "PlanarFaceDetectionStrategy.h"
#include "CylinderDetectionStrategy.h"
#include "FaceCollector.h"
#include <BRepAdaptor_Surface.hxx>



using namespace PathForge::Topology;


namespace PathForge {
    namespace Feature {
        FeatureExtractor::FeatureExtractor() {
            // 注册特征识别策略
            registerStrategy(FeatureType::PlanarFace, std::make_shared<PlanarFaceDetectionStrategy>());
            registerStrategy(FeatureType::Cylinder, std::make_shared<CylinderDetectionStrategy>());
            // 注册其他特征的识别策略
        }

        void FeatureExtractor::registerStrategy(FeatureType type, std::shared_ptr<FeatureDetectionStrategy> strategy) {
            strategies_[type] = strategy;
        }

        FeatureType FeatureExtractor::extractFeature(const TopoDS_Face& face,
            const std::vector<int>& neighbors,
            const AdjacencyGraph& graph) {
            // 获取面类型
            FeatureType baseType = classifyFaceType(face);

            // 使用已注册的策略来进行特征提取
            if (strategies_.find(baseType) != strategies_.end()) {
                auto cc = strategies_[baseType]->detectFeature(face, neighbors, graph);;
                return cc;
            }

            return FeatureType::Unknown;
        }

        std::vector<Feature>
            FeatureExtractor::detectFeatures(
                const TopoDS_Shape& shape,
                const AdjacencyGraph& graph)
        {
            std::vector<Feature> result;

            const auto& faces = graph.getFaces();
            const auto& adjacency = graph.getAdjacency();

            result.reserve(faces.size());

            for (std::size_t i = 0; i < faces.size(); ++i) {

                const TopoDS_Face& face = faces[i];

                std::vector<int> neighbors;
                auto it = adjacency.find(static_cast<int>(i));
                if (it != adjacency.end()) {
                    neighbors = it->second;
                }

                FeatureType type = extractFeature(face, neighbors, graph);
                Feature f = Feature::CreateFromFace(type, face);

                result.push_back(f);
            }

            return result;
        }

        FeatureType classifyFaceType(const TopoDS_Face& face)
        {
            if (face.IsNull())
                return FeatureType::Unknown;

            BRepAdaptor_Surface surface(face);
            GeomAbs_SurfaceType stype = surface.GetType();

            switch (stype) {

            case GeomAbs_Plane:
                return FeatureType::PlanarFace;

            case GeomAbs_Cylinder:
                return FeatureType::Cylinder;

            case GeomAbs_Cone:
                return FeatureType::Cone;

                // ===== 预留（后续扩展） =====
            case GeomAbs_Sphere:
                // 当前未支持球面加工语义
                return FeatureType::Unknown;

            case GeomAbs_Torus:
                return FeatureType::Unknown;

            case GeomAbs_BezierSurface:
            case GeomAbs_BSplineSurface:
                return FeatureType::Unknown;

            default:
                return FeatureType::Unknown;
            }
        }

    }
}