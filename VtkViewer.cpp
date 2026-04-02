// VtkViewer.cpp
#include "VtkViewer.h"
#include "Toolpath.h"
#include <BRepMesh_IncrementalMesh.hxx>
#include <Poly_Triangulation.hxx>
#include <TopoDS.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs.hxx>
#include <BRep_Tool.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Builder.hxx>
#include <vtkCellData.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkTriangle.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkLookupTable.h>
#include <vtkNamedColors.h>
#include <vtkPolyDataNormals.h>
#include <vtkIntArray.h>
#include <iostream>
#include <vtkPolyLine.h>
#include <cmath>
#include <vtkAxesActor.h>
#include <vtkTransform.h>
#include <vtkBoundingBox.h>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <vtkRendererCollection.h>

using namespace PathForge::Path;

VtkViewer::VtkViewer()
    : m_ownRenderWindow(true)
{
    m_renderer = vtkSmartPointer<vtkRenderer>::New();
    m_renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    m_interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    m_renderWindow->AddRenderer(m_renderer);
    m_interactor->SetRenderWindow(m_renderWindow);
    m_renderer->SetBackground(0.1, 0.1, 0.15);
    m_renderer->SetBackground2(0.3, 0.3, 0.4);
    m_renderer->GradientBackgroundOn();
    m_renderWindow->SetSize(1600, 1000);
    m_renderWindow->SetWindowName("PathForge - VTK Viewer");

    // 创建坐标轴
    m_axesActor = vtkSmartPointer<vtkAxesActor>::New();
    m_axesActor->SetTotalLength(1.0, 1.0, 1.0);
    m_axesActor->SetShaftTypeToCylinder();
    m_axesActor->SetAxisLabels(true);
    //m_axesActor->SetXAxisLabelText("X");
    //m_axesActor->SetYAxisLabelText("Y");
    //m_axesActor->SetZAxisLabelText("Z");

    // 将坐标轴添加到主渲染器
    m_renderer->AddActor(m_axesActor);
}

VtkViewer::~VtkViewer() = default;

void VtkViewer::SetRenderWindow(vtkRenderWindow* renderWindow)
{
    if (!renderWindow)
        return;

    m_ownRenderWindow = false;
    m_renderWindow = renderWindow;
    
    // 如果渲染器未设置，则创建并添加
    if (!m_renderer)
    {
        m_renderer = vtkSmartPointer<vtkRenderer>::New();
        m_renderer->SetBackground(0.1, 0.1, 0.15);
        m_renderer->SetBackground2(0.2, 0.2, 0.3);
        m_renderer->GradientBackgroundOn();
    }
    
    // 直接添加渲染器，不检查是否已存在
    m_renderWindow->AddRenderer(m_renderer);

    // 添加坐标轴
    if (m_axesActor)
    {
        m_renderer->AddActor(m_axesActor);
    }
}

void VtkViewer::SetWindowTitle(const std::string& title)
{
    m_renderWindow->SetWindowName(title.c_str());
}

void VtkViewer::Clear()
{
    // 只移除模型 actor，保留坐标轴
    if (m_actor) {
        m_renderer->RemoveActor(m_actor);
    }
    m_actor = nullptr;
    m_mapper = nullptr;
}

void VtkViewer::StartInteraction()
{
    if (m_interactor) m_interactor->Start();
}

void VtkViewer::ConvertOccToVtk(const TopoDS_Shape& shape)
{
    BRepMesh_IncrementalMesh mesh(shape, 0.05);
    vtkNew<vtkPoints> points;
    vtkNew<vtkCellArray> triangles;
    int vertexOffset = 0;
    TopExp_Explorer faceExp(shape, TopAbs_FACE);
    while (faceExp.More())
    {
        const TopoDS_Face& face = TopoDS::Face(faceExp.Current());
        TopLoc_Location loc;
        Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
        if (tri.IsNull()) { faceExp.Next(); continue; }
        for (int i = 1; i <= tri->NbNodes(); ++i)
        {
            gp_Pnt p = tri->Node(i).Transformed(loc.Transformation());
            points->InsertNextPoint(p.X(), p.Y(), p.Z());
        }
        for (int i = 1; i <= tri->NbTriangles(); ++i)
        {
            Poly_Triangle triangle = tri->Triangle(i);
            int n1 = 0, n2 = 0, n3 = 0;
            triangle.Get(n1, n2, n3);
            vtkIdType ids[3] = { vertexOffset + n1 - 1,
                                 vertexOffset + n2 - 1,
                                 vertexOffset + n3 - 1 };
            triangles->InsertNextCell(3, ids);
        }
        vertexOffset += tri->NbNodes();
        faceExp.Next();
    }
    vtkNew<vtkPolyData> polyData;
    polyData->SetPoints(points);
    polyData->SetPolys(triangles);
    vtkNew<vtkPolyDataNormals> normals;
    normals->SetInputData(polyData);
    normals->ComputePointNormalsOn();
    normals->ComputeCellNormalsOff();
    normals->AutoOrientNormalsOn();
    normals->Update();
    m_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    m_mapper->SetInputData(normals->GetOutput());
    m_actor = vtkSmartPointer<vtkActor>::New();
    m_actor->SetMapper(m_mapper);
    m_actor->GetProperty()->SetColor(0.3, 0.7, 1.0);
}

void VtkViewer::ShowShape(const TopoDS_Shape& shape)
{
    Clear();
    ConvertOccToVtk(shape);
    if (m_actor) m_renderer->AddActor(m_actor);
    
    // 计算边界盒，将坐标轴放到右下角
    if (m_axesActor)
    {
        Bnd_Box bbox;
        BRepBndLib::Add(shape, bbox);
        
        Standard_Real xMin, yMin, zMin, xMax, yMax, zMax;
        bbox.Get(xMin, yMin, zMin, xMax, yMax, zMax);
        
        // 设置坐标轴长度为边界盒尺寸的 1/5
        double axisLength = std::max({xMax - xMin, yMax - yMin, zMax - zMin}) / 5.0;
        m_axesActor->SetTotalLength(axisLength, axisLength, axisLength);
        
        // 将坐标轴放到边界盒的右下角 (X 最大，Y 最小，Z 最小)
        vtkNew<vtkTransform> transform;
        transform->Translate(xMax - axisLength, yMin, zMin);
        m_axesActor->SetUserTransform(transform);
        
        m_axesActor->SetVisibility(true);
    }
    
    m_renderer->ResetCamera();
    m_renderWindow->Render();
}

void VtkViewer::ShowToolpath(const Toolpath& toolpath)
{
    vtkNew<vtkPoints> vtkPoints;
    vtkNew<vtkCellArray> lines;
    const auto& points = toolpath.points();

    // 只有当点数量≥2时才绘制线段（单个点无法形成线段）
    if (points.size() >= 2)
    {
        // 第一步：将所有加工路径点按顺序添加到VTK点集合中
        for (size_t i = 0; i < points.size(); ++i)
        {
            const auto& p = points[i].position;
            vtkPoints->InsertNextPoint(p.X(), p.Y(), p.Z());
        }

        // 第二步：按顺序连接相邻点（i → i+1），形成连续折线
        // 循环终止条件：i < points.size() - 1，避免i+1越界
        for (size_t i = 0; i < points.size() - 1; ++i)
        {
            vtkIdType ids[2] = {
                static_cast<vtkIdType>(i),
                static_cast<vtkIdType>(i + 1)
            };
            // 插入由两个相邻点组成的线段单元
            lines->InsertNextCell(2, ids);
        }
    }

    // 构建PolyData对象，绑定点和线段数据
    vtkNew<vtkPolyData> toolpathData;
    toolpathData->SetPoints(vtkPoints);
    toolpathData->SetLines(lines);

    // 创建Mapper（数据映射）和Actor（渲染实体）
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(toolpathData);

    vtkSmartPointer<vtkActor> toolpathActor = vtkSmartPointer<vtkActor>::New();
    toolpathActor->SetMapper(mapper);
    // 设置线段样式：红色、线宽2
    toolpathActor->GetProperty()->SetColor(1.0, 0.0, 0.0);
    toolpathActor->GetProperty()->SetLineWidth(2.0);

    // 将加工路径Actor添加到渲染器，重置相机并刷新窗口
    m_renderer->AddActor(toolpathActor);
    m_renderer->ResetCamera();
    m_renderWindow->Render();
}

void VtkViewer::ShowToolpaths(const std::vector<std::shared_ptr<Toolpath>>& toolpaths)
{
    vtkNew<vtkNamedColors> colors;
    double colorIndex = 0.0;
    double colorStep = toolpaths.size() > 0 ? 1.0 / toolpaths.size() : 1.0;
    for (const auto& toolpath : toolpaths)
    {
        if (!toolpath) continue;
        vtkNew<vtkPoints> vtkPoints;
        vtkNew<vtkCellArray> lines;
        const auto& points = toolpath->points();
        for (size_t i = 0; i < points.size(); ++i)
        {
            const auto& p = points[i].position;
            vtkPoints->InsertNextPoint(p.X(), p.Y(), p.Z());
        }
        for (size_t i = 0; i < points.size() - 1; ++i)
        {
            vtkIdType ids[2] = { static_cast<vtkIdType>(i), static_cast<vtkIdType>(i + 1) };
            lines->InsertNextCell(2, ids);
        }
        vtkNew<vtkPolyData> toolpathData;
        toolpathData->SetPoints(vtkPoints);
        toolpathData->SetLines(lines);
        vtkNew<vtkPolyDataMapper> mapper;
        mapper->SetInputData(toolpathData);
        vtkSmartPointer<vtkActor> toolpathActor = vtkSmartPointer<vtkActor>::New();
        toolpathActor->SetMapper(mapper);

        // 使用 colorIndex 值设置颜色（蓝到红渐变）
        double r = colorIndex;
        double g = 0.0;
        double b = 1.0 - colorIndex;
        toolpathActor->GetProperty()->SetColor(r, g, b);
        toolpathActor->GetProperty()->SetLineWidth(2.0);
        m_renderer->AddActor(toolpathActor);
        colorIndex += colorStep;
    }
    m_renderer->ResetCamera();
    m_renderWindow->Render();
}

void VtkViewer::ShowShapeAndToolpath(const TopoDS_Shape& shape, const Toolpath& toolpath)
{
    ShowShape(shape);
    ShowToolpath(toolpath);
}

void VtkViewer::ShowAxes(bool visible)
{
    if (m_axesActor)
    {
        m_axesActor->SetVisibility(visible);
    }
    m_renderWindow->Render();
}
