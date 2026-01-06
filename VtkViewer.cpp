// VtkViewer.cpp
#include "VtkViewer.h"

#include <BRepMesh_IncrementalMesh.hxx>
#include <Poly_Triangulation.hxx>
#include <TopoDS.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs.hxx>
#include <BRep_Tool.hxx>
#include <TopoDS_Face.hxx>

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

VtkViewer::VtkViewer()
{
    m_renderer = vtkSmartPointer<vtkRenderer>::New();
    m_renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    m_interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();

    m_renderWindow->AddRenderer(m_renderer);
    m_interactor->SetRenderWindow(m_renderWindow);

    // 美化背景
    m_renderer->SetBackground(0.1, 0.1, 0.15);
    m_renderer->SetBackground2(0.3, 0.3, 0.4);
    m_renderer->GradientBackgroundOn();

    m_renderWindow->SetSize(1600, 1000);
    m_renderWindow->SetWindowName("PathForge - VTK 引擎");
}

VtkViewer::~VtkViewer() = default;

void VtkViewer::SetWindowTitle(const std::string& title)
{
    m_renderWindow->SetWindowName(title.c_str());
}

void VtkViewer::Clear()
{
    if (m_renderer) {
        m_renderer->RemoveAllViewProps();
    }
    m_actor = nullptr;
    m_mapper = nullptr;
}

void VtkViewer::StartInteraction()
{
    if (m_interactor) m_interactor->Start();
}

// ---------- 简单整体显示（不按 feature 上色） ----------
void VtkViewer::ConvertOccToVtk(const TopoDS_Shape& shape)
{
    BRepMesh_IncrementalMesh mesh(shape, 0.05); // 精度可调

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

        // 顶点转换
        for (int i = 1; i <= tri->NbNodes(); ++i)
        {
            gp_Pnt p = tri->Node(i).Transformed(loc.Transformation());
            points->InsertNextPoint(p.X(), p.Y(), p.Z());
        }

        // 三角形转换（注意 orientation）
        for (int i = 1; i <= tri->NbTriangles(); ++i)
        {
            Poly_Triangle triangle = tri->Triangle(i);

            int n1 = 0, n2 = 0, n3 = 0;
            triangle.Get(n1, n2, n3);

            // vtkIdType ids must be in contiguous memory
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

    // 生成法线，避免光照导致黑色
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

// ---------- 颜色表 ----------
vtkSmartPointer<vtkLookupTable> VtkViewer::BuildFeatureColorTable()
{
    vtkNew<vtkLookupTable> lut;
    vtkNew<vtkNamedColors> colors;

    lut->SetNumberOfTableValues(7);
    lut->Build();

    // 颜色顺序需与 FeatureType 枚举值对应（你在 FeatureExtractor 中定义）
    // 假设枚举值顺序为 Unknown=0, PlanarFace=1, Cylinder=2, Cone=3, Hole=4, Pocket=5, Slot=6
    lut->SetTableValue(0, colors->GetColor4d("LightGray").GetData());  // Unknown
    lut->SetTableValue(1, colors->GetColor4d("SkyBlue").GetData());    // Planar
    lut->SetTableValue(2, colors->GetColor4d("Tomato").GetData());     // Cylinder
    lut->SetTableValue(3, colors->GetColor4d("Gold").GetData());       // Cone
    lut->SetTableValue(4, colors->GetColor4d("Green").GetData());      // Hole
    lut->SetTableValue(5, colors->GetColor4d("DeepPink").GetData());   // Pocket
    lut->SetTableValue(6, colors->GetColor4d("Purple").GetData());     // Slot

    return lut;
}

void VtkViewer::ShowShapeWithFeatureColor(
    const TopoDS_Shape& shape,
    const std::vector<Feature>& features)
{
    Clear();
    BRepMesh_IncrementalMesh mesh(shape, 0.05);

    vtkNew<vtkPoints> points;
    vtkNew<vtkCellArray> triangles;
    vtkNew<vtkIntArray> featureIds;
    featureIds->SetName("FeatureType");

    vtkNew<vtkPolyData> polyData;

    int pointOffset = 0;

    // 遍历每个 Feature，渲染其包含的所有面
    for (const auto& f : features)
    {
        const auto& faceList = f.faces();
        if (faceList.empty()) continue;

        for (const auto& face : faceList)
        {
            TopLoc_Location loc;
            Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
            if (tri.IsNull()) continue;

            gp_Trsf trsf = loc.Transformation();

            // 顶点
            for (int i = 1; i <= tri->NbNodes(); ++i)
            {
                gp_Pnt p = tri->Node(i).Transformed(trsf);
                points->InsertNextPoint(p.X(), p.Y(), p.Z());
            }

            // 三角形
            for (int i = 1; i <= tri->NbTriangles(); ++i)
            {
                Poly_Triangle t = tri->Triangle(i);
                int n1, n2, n3;
                t.Get(n1, n2, n3);

                vtkIdType c[3] = { pointOffset + n1 - 1,
                                   pointOffset + n2 - 1,
                                   pointOffset + n3 - 1 };

                triangles->InsertNextCell(3, c);

                // 每个单元标记为该 Feature 的类型
                featureIds->InsertNextValue(static_cast<int>(f.type()));
            }

            pointOffset += tri->NbNodes();
        }
    }

    polyData->SetPoints(points);
    polyData->SetPolys(triangles);
    polyData->GetCellData()->SetScalars(featureIds);

    // ---- Add normals ----
    vtkNew<vtkPolyDataNormals> normals;
    normals->SetInputData(polyData);
    normals->ComputePointNormalsOn();
    normals->AutoOrientNormalsOn();
    normals->SplittingOff();
    normals->Update();

    // ---- Mapper ----
    vtkSmartPointer<vtkLookupTable> lut = BuildFeatureColorTable();

    m_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    m_mapper->SetInputConnection(normals->GetOutputPort());
    m_mapper->SetScalarModeToUseCellData();
    m_mapper->ScalarVisibilityOn();
    //m_mapper->SetLookupTable(lut);
    m_mapper->SetScalarRange(0, 6);

    m_actor = vtkSmartPointer<vtkActor>::New();
    m_actor->SetMapper(m_mapper);
    m_actor->GetProperty()->SetInterpolationToPhong();   // optional
    m_actor->GetProperty()->SetSpecular(0.4);
    m_actor->GetProperty()->SetSpecularPower(20);

    m_renderer->AddActor(m_actor);
    m_renderer->ResetCamera();
    m_renderWindow->Render();
}


// ---------- 简单整体显示接口 ----------
void VtkViewer::ShowShape(const TopoDS_Shape& shape)
{
    Clear();
    ConvertOccToVtk(shape);
    if (m_actor) m_renderer->AddActor(m_actor);
    m_renderer->ResetCamera();
    m_renderWindow->Render();
}