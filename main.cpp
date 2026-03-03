#include "VtkViewer.h"
#include "ModelLoader.h"
#include "Topology/FaceCollector.h"
#include "Path/StrategyFactory.h"
#include "Path/Toolpath.h"

#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs.hxx>
#include <TopoDS.hxx>

#include <iostream>

using namespace PathForge;
using namespace PathForge::Topology;
using namespace PathForge::Path;


int main(int argc, char* argv[])
{
    ModelLoader loader;
    TopoDS_Shape shape;

    std::string file = "D:\\myself\\PathForge\\kl.step";
    if (argc >= 2) file = argv[1];

    if (!loader.LoadFile(file, shape) || shape.IsNull())
    {
        std::cerr << "Failed to load model: " << file << std::endl;
        return -1;
    }

    FaceCollector collectFaces;
    std::vector<TopoDS_Face> occFaces = collectFaces.collectFaces(shape);

    if (occFaces.empty())
    {
        std::cerr << "No faces found in model" << std::endl;
        return -1;
    }

    TopoDS_Face targetFace = occFaces.front();

    TopoDS_Wire boundaryWire;
    TopExp_Explorer exp(targetFace, TopAbs_WIRE);
    if (exp.More())
    {
        boundaryWire = TopoDS::Wire(exp.Current());
    }

    if (boundaryWire.IsNull())
    {
        std::cerr << "Failed to get boundary wire from face" << std::endl;
        return -1;
    }

    PathStrategyContext ctx;
    ctx.setBoundaryWire(boundaryWire);
    ctx.setStockTop(5.0);
    ctx.setModelTop(0.0);
    ctx.setStepover(8.0);
    ctx.setCuttingAngle(0.0);
    ctx.setCuttingDirection(CuttingDirection::Zigzag);
    ctx.setToolDiameter(10.0);
    ctx.setFeedrate(1500.0);
    ctx.setPlungeFeedrate(300.0);
    ctx.setSafeZ(10.0);
    ctx.setLeadInEnabled(true);
    ctx.setLeadOutEnabled(true);
    ctx.setLeadInLength(5.0);
    ctx.setLeadOutLength(5.0);

    auto strategy = PathStrategyFactory::create(StrategyType::FaceMilling2D);
    if (!strategy)
    {
        std::cerr << "Failed to create strategy" << std::endl;
        return -1;
    }

    strategy->setContext(ctx);

    if (!strategy->validate())
    {
        std::cerr << "Strategy validation failed: " << strategy->getLastError() << std::endl;
        return -1;
    }

    auto toolpath = strategy->generate();


    VtkViewer viewer;
    viewer.SetWindowTitle("PathForge - 2D Face Milling");
    viewer.ShowShapeAndToolpath(shape, *toolpath);
    viewer.StartInteraction();

    return 0;
}
