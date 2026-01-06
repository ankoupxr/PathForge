// VtkViewer.h
#pragma once

#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>

#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkTriangle.h>
#include <vtkProperty.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkLookupTable.h>
#include <vtkIntArray.h>
#include <vtkCellData.h>
#include <vtkNamedColors.h>
#include <vtkPolyDataNormals.h>

#include <vector>
#include <string>
#include "FeatureExtractor.h" // 提供 PathForge::Feature / FeatureType

using namespace PathForge::Topology;
using namespace PathForge::Feature;

// 注意：Feature 类型在 FeatureExtractor.h 中定义，命名空间为 PathForge
class VtkViewer
{
public:
    VtkViewer();
    ~VtkViewer();

    // 显示一个 OCC 模型（核心函数）
    void ShowShape(const TopoDS_Shape& shape);

    // 按 feature 上色显示（必须提供 FeatureExtractor 识别得到的 features）
    void ShowShapeWithFeatureColor(const TopoDS_Shape& shape,
        const std::vector<Feature>& features);

    // 设置窗口标题
    void SetWindowTitle(const std::string& title);

    // 开始交互（阻塞，直到关闭窗口）
    void StartInteraction();

    // 清空当前显示
    void Clear();

    // 颜色表构建（static，方便复用）
    static vtkSmartPointer<vtkLookupTable> BuildFeatureColorTable();


private:
    // OCC → VTK 网格转换（仅模型整体显示）
    void ConvertOccToVtk(const TopoDS_Shape& shape);

    vtkSmartPointer<vtkRenderer>              m_renderer;
    vtkSmartPointer<vtkRenderWindow>          m_renderWindow;
    vtkSmartPointer<vtkRenderWindowInteractor> m_interactor;
    vtkSmartPointer<vtkActor>                 m_actor;
    vtkSmartPointer<vtkPolyDataMapper>        m_mapper;
};
