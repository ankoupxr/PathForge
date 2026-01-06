#include "VtkViewer.h"
#include "ModelLoader.h"
#include "FaceCollector.h"
#include "AdjacencyGraph.h"
#include <FeatureExtractor.h>

using namespace PathForge::Topology;

int main(int argc, char* argv[])
{
    ModelLoader loader;
    TopoDS_Shape shape;

    //std::string file = "D:\\\\\\\\myself\\\\\\\\OCC\\\\\\\\Atypec.stp";
    std::string file = "D:\\myself\\OCC\\mokuai_waike.stp";
    if (argc >= 2) file = argv[1];

    if (!loader.LoadFile(file, shape) || shape.IsNull())
    {
        ::cerr << "无法加载模型文件: " << file << std::endl;
		return -1;
    }

    //收集几何面
    FaceCollector CollectFaces;
	std::vector<TopoDS_Face> occFaces = CollectFaces.collectFaces(shape);
    std::unordered_map<int, int> faceIndices = CollectFaces.indexFaces(occFaces);

    // 构建面邻接图
    AdjacencyGraph graph(shape);

    // Debug 输出邻接结果
    auto faces = graph.getFaces();
    auto adj = graph.getAdjacency();

    //for (auto& p : adj) {
    //    std::cout << "Face " << p.first << " adjacent: ";
    //    for (auto idx : p.second)
    //        std::cout << idx << ", ";
    //    std::cout << std::endl;
    //}

    FeatureExtractor extractor;

    auto features = extractor.detectFeatures(shape, graph);

    //for (auto& f : features) {
    //    std::cout << f.name << std::endl;
    //}



    // 2. VTK
    VtkViewer viewer;
    viewer.SetWindowTitle("PathForge - Feature Visualization");
    viewer.ShowShapeWithFeatureColor(shape, features);
    viewer.StartInteraction();

    return 0;
}