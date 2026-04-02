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
#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkAnnotatedCubeActor.h>

#include <vector>
#include <string>
#include <memory>

#include "Path/Toolpath.h"

class VtkViewer
{
public:
    VtkViewer();
    ~VtkViewer();

    // 设置渲染窗口（用于 Qt 嵌入）
    void SetRenderWindow(vtkRenderWindow* renderWindow);

    void ShowShape(const TopoDS_Shape& shape);

    void SetWindowTitle(const std::string& title);

    void StartInteraction();

    void Clear();

    void ShowToolpath(const PathForge::Path::Toolpath& toolpath);

    void ShowToolpaths(const std::vector<std::shared_ptr<PathForge::Path::Toolpath>>& toolpaths);

    void ShowShapeAndToolpath(const TopoDS_Shape& shape, const PathForge::Path::Toolpath& toolpath);

    void ShowAxes(bool visible = true);

    vtkRenderWindow* GetRenderWindow() const { return m_renderWindow; }
    vtkRenderer* GetRenderer() const { return m_renderer; }

private:
    void ConvertOccToVtk(const TopoDS_Shape& shape);

    vtkSmartPointer<vtkRenderer>              m_renderer;
    vtkSmartPointer<vtkRenderWindow>          m_renderWindow;
    vtkSmartPointer<vtkRenderWindowInteractor> m_interactor;
    vtkSmartPointer<vtkActor>                 m_actor;
    vtkSmartPointer<vtkPolyDataMapper>        m_mapper;
    vtkSmartPointer<vtkAxesActor>             m_axesActor;
    bool                                      m_ownRenderWindow; // 是否拥有 renderWindow
};
