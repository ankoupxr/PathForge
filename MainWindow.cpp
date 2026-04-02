#include "MainWindow.h"
#include "Topology/FaceCollector.h"
#include "Path/StrategyFactory.h"
#include "Path/Toolpath.h"

#include <QApplication>

#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkCamera.h>
#include <vtkProperty.h>

#include <QVTKOpenGLNativeWidget.h>

#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopoDS.hxx>

using namespace PathForge::Topology;
using namespace PathForge::Path;

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , m_centralWidget(nullptr)
    , m_centralLayout(nullptr)
    , m_vtkWidget(nullptr)
    , m_viewer(nullptr)
    , m_featureTree(nullptr)
    , m_logText(nullptr)
    , m_parameterPanel(nullptr)
    , m_hasModel(false)
{
    setupUI();
    createMenus();
    createToolbars();
    createDockWidgets();

    setWindowTitle("PathForge - 特征识别与加工系统");
    resize(1600, 900);

    m_logText->appendLog("应用程序已启动");
    m_logText->appendLog("请导入 STEP/IGES/STL 模型文件开始工作");
}

void MainWindow::setupUI()
{
    m_centralWidget = new QWidget(this);
    setCentralWidget(m_centralWidget);

    m_centralLayout = new QHBoxLayout(m_centralWidget);
    m_centralLayout->setContentsMargins(0, 0, 0, 0);

    // 使用 QVTKOpenGLNativeWidget 作为 VTK 容器
    m_vtkWidget = new QVTKOpenGLNativeWidget(this);
    m_vtkWidget->setMinimumSize(600, 400);
    m_vtkWidget->setStyleSheet("background-color: #1a1a2e;");

    m_centralLayout->addWidget(m_vtkWidget, 1);

    // 创建 VTK 渲染窗口
    m_renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    m_renderer = vtkSmartPointer<vtkRenderer>::New();
    m_renderWindow->AddRenderer(m_renderer);
    m_vtkWidget->setRenderWindow(m_renderWindow);

    // 设置背景颜色
    m_renderer->SetBackground(0.1, 0.1, 0.15);
    m_renderer->SetBackground2(0.2, 0.2, 0.3);
    m_renderer->GradientBackgroundOn();

    // 显示坐标系的 vtk 组件
    vtkSmartPointer<vtkAxesActor> axes_actor = vtkSmartPointer<vtkAxesActor>::New();
    axes_actor->SetPosition(0, 0, 0);
    axes_actor->SetTotalLength(2, 2, 2);
    axes_actor->SetShaftType(0);
    axes_actor->SetCylinderRadius(0.03);
    axes_actor->SetAxisLabels(1);
    axes_actor->SetTipType(0);
    axes_actor->SetXAxisLabelText("X");
    axes_actor->SetYAxisLabelText("Y");
    axes_actor->SetZAxisLabelText("Z");
    axes_actor->GetXAxisShaftProperty()->SetColor(1, 0, 0);
    axes_actor->GetYAxisShaftProperty()->SetColor(0, 1, 0);
    axes_actor->GetZAxisShaftProperty()->SetColor(0, 0, 1);

    // 控制坐标系，使之随视角共同变化
    m_markerWidget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    m_markerWidget->SetOrientationMarker(axes_actor);
    m_markerWidget->SetInteractor(m_vtkWidget->interactor());
    m_markerWidget->SetViewport(0.0, 0.0, 0.2, 0.2);
    m_markerWidget->SetEnabled(1);
    m_markerWidget->SetOutlineColor(1, 0, 0);

    // 创建查看器
    m_viewer = std::make_unique<VtkViewer>();
    m_viewer->SetRenderWindow(m_renderWindow);
}

void MainWindow::createMenus()
{
    // 文件菜单
    auto* fileMenu = menuBar()->addMenu("文件 (&F)");
    
    auto* importAction = fileMenu->addAction("导入模型 (&I)");
    importAction->setShortcut(QKeySequence::Open);
    connect(importAction, &QAction::triggered, this, &MainWindow::onImportModel);

    fileMenu->addSeparator();

    auto* exitAction = fileMenu->addAction("退出 (&X)");
    exitAction->setShortcut(QKeySequence::Quit);
    connect(exitAction, &QAction::triggered, this, &QMainWindow::close);

    // 特征菜单
    auto* featureMenu = menuBar()->addMenu("特征 (&F)");
    
    auto* recognizeAction = featureMenu->addAction("识别特征 (&R)");
    recognizeAction->setShortcut(QKeySequence(Qt::Key_F5));
    connect(recognizeAction, &QAction::triggered, this, &MainWindow::onRecognizeFeatures);

    featureMenu->addSeparator();

    auto* clearAction = featureMenu->addAction("清除模型 (&C)");
    clearAction->setShortcut(QKeySequence::Delete);
    connect(clearAction, &QAction::triggered, this, &MainWindow::onClearModel);

    // 加工菜单
    auto* machiningMenu = menuBar()->addMenu("加工 (&M)");
    
    auto* generateAction = machiningMenu->addAction("生成刀路 (&G)");
    generateAction->setShortcut(QKeySequence(Qt::Key_F9));
    connect(generateAction, &QAction::triggered, this, &MainWindow::onGenerateToolpath);

    // 帮助菜单
    auto* helpMenu = menuBar()->addMenu("帮助 (&H)");
    
    auto* aboutAction = helpMenu->addAction("关于 (&A)");
    connect(aboutAction, &QAction::triggered, this, &MainWindow::onAbout);
}

void MainWindow::createToolbars()
{
    auto* toolbar = addToolBar("主工具栏");
    toolbar->setMovable(false);

    auto* importBtn = new QPushButton("📁 导入模型");
    importBtn->setStyleSheet("padding: 5px 10px;");
    connect(importBtn, &QPushButton::clicked, this, &MainWindow::onImportModel);
    toolbar->addWidget(importBtn);

    toolbar->addSeparator();

    auto* recognizeBtn = new QPushButton("🔍 识别特征");
    recognizeBtn->setStyleSheet("padding: 5px 10px;");
    connect(recognizeBtn, &QPushButton::clicked, this, &MainWindow::onRecognizeFeatures);
    toolbar->addWidget(recognizeBtn);

    toolbar->addSeparator();

    auto* generateBtn = new QPushButton("⚙️ 生成刀路");
    generateBtn->setStyleSheet("padding: 5px 10px; background-color: #4CAF50; color: white; font-weight: bold;");
    connect(generateBtn, &QPushButton::clicked, this, &MainWindow::onGenerateToolpath);
    toolbar->addWidget(generateBtn);

    toolbar->addSeparator();

    auto* clearBtn = new QPushButton("🗑️ 清除");
    clearBtn->setStyleSheet("padding: 5px 10px;");
    connect(clearBtn, &QPushButton::clicked, this, &MainWindow::onClearModel);
    toolbar->addWidget(clearBtn);
}

void MainWindow::createDockWidgets()
{
    // 特征树 Dock
    m_featureDock = new QDockWidget("特征树", this);
    m_featureDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    m_featureTree = new FeatureTreeWidget(m_featureDock);
    m_featureDock->setWidget(m_featureTree);
    addDockWidget(Qt::RightDockWidgetArea, m_featureDock);

    // 参数面板 Dock
    m_parameterDock = new QDockWidget("加工参数", this);
    m_parameterDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    m_parameterPanel = new ParameterPanel(m_parameterDock);
    m_parameterDock->setWidget(m_parameterPanel);
    addDockWidget(Qt::RightDockWidgetArea, m_parameterDock);

    connect(m_parameterPanel->generateButton(), &QPushButton::clicked, this, &MainWindow::onGenerateToolpath);

    // 日志 Dock
    m_logDock = new QDockWidget("日志", this);
    m_logDock->setAllowedAreas(Qt::BottomDockWidgetArea);
    m_logText = new LogTextEdit(m_logDock);
    m_logDock->setWidget(m_logText);
    addDockWidget(Qt::BottomDockWidgetArea, m_logDock);

    // 状态栏
    m_statusLabel = new QLabel("就绪");
    m_modelInfoLabel = new QLabel("无模型");
    m_statusLabel->setMinimumWidth(200);
    m_modelInfoLabel->setMinimumWidth(150);

    statusBar()->addWidget(m_statusLabel);
    statusBar()->addPermanentWidget(m_modelInfoLabel);
}

void MainWindow::onImportModel()
{
    QString filter = "CAD 模型文件 (*.step *.stp *.STEP *.STP *.iges *.igs *.IGES *.IGS *.stl *.STL *.brep *.BRep *.BREP);;所有文件 (*)";
    QString filePath = QFileDialog::getOpenFileName(this, "导入模型", "", filter);

    if (filePath.isEmpty())
        return;

    loadModel(filePath);
}

void MainWindow::loadModel(const QString& filePath)
{
    m_logText->appendLog(QString("正在加载模型：%1").arg(filePath));
    m_statusLabel->setText("正在加载模型...");

    TopoDS_Shape shape;
    if (!m_modelLoader.LoadFile(filePath.toStdString(), shape) || shape.IsNull())
    {
        QMessageBox::critical(this, "错误", 
            QString("无法加载模型:\n%1").arg(QString::fromStdString(m_modelLoader.GetLastError())));
        m_statusLabel->setText("加载失败");
        m_logText->appendLog("模型加载失败");
        return;
    }

    m_currentShape = shape;
    m_hasModel = true;

    // 计算模型信息
    GProp_GProps props;
    BRepGProp::VolumeProperties(shape, props);
    double volume = props.Mass();

    Bnd_Box bbox;
    BRepBndLib::Add(shape, bbox);
    Standard_Real xMin, yMin, zMin, xMax, yMax, zMax;
    bbox.Get(xMin, yMin, zMin, xMax, yMax, zMax);

    double sizeX = xMax - xMin;
    double sizeY = yMax - yMin;
    double sizeZ = zMax - zMin;

    // 统计面数量
    int faceCount = 0;
    TopExp_Explorer exp(shape, TopAbs_FACE);
    while (exp.More()) { faceCount++; exp.Next(); }

    // 显示模型
    m_viewer->ShowShape(shape);

    // 更新 UI
    m_modelInfoLabel->setText(QString("模型：%1 | 面：%2 | 体积:%3 mm³")
        .arg(QFileInfo(filePath).fileName())
        .arg(faceCount)
        .arg(volume, 0, 'f', 2));

    m_statusLabel->setText("模型加载成功");
    m_logText->appendLog(QString("模型加载成功 - 面数：%1, 体积：%2 mm³, 尺寸：%3 x %4 x %5 mm")
        .arg(faceCount)
        .arg(volume, 0, 'f', 2)
        .arg(sizeX, 0, 'f', 2)
        .arg(sizeY, 0, 'f', 2)
        .arg(sizeZ, 0, 'f', 2));

    // 清空特征树
    m_featureTree->clear();
    m_recognizedFaces.clear();

    // 刷新渲染
    m_renderWindow->Render();
}

void MainWindow::onRecognizeFeatures()
{
    if (!m_hasModel || m_currentShape.IsNull())
    {
        QMessageBox::warning(this, "警告", "请先导入模型");
        return;
    }

    m_logText->appendLog("开始特征识别...");
    m_statusLabel->setText("正在识别特征...");

    FaceCollector collector;
    m_recognizedFaces = collector.collectFaces(m_currentShape);

    if (m_recognizedFaces.empty())
    {
        m_logText->appendLog("未识别到任何特征");
        m_statusLabel->setText("未识别到特征");
        QMessageBox::information(this, "提示", "未识别到可加工的特征");
        return;
    }

    displayFeatures();

    m_logText->appendLog(QString("特征识别完成 - 共识别 %1 个特征").arg(m_recognizedFaces.size()));
    m_statusLabel->setText(QString("已识别 %1 个特征").arg(m_recognizedFaces.size()));
}

void MainWindow::displayFeatures()
{
    m_featureTree->clear();

    // 按类型分类统计
    std::map<std::string, int> faceTypes;
    
    for (const auto& face : m_recognizedFaces)
    {
        // 简单分类：根据面的面积
        GProp_GProps props;
        BRepGProp::SurfaceProperties(face, props);
        double area = props.Mass();

        std::string type = "平面";
        if (area < 100) type = "小平面";
        else if (area < 500) type = "中平面";
        else type = "大平面";

        faceTypes[type]++;
    }

    // 添加到树
    auto* rootItem = new QTreeWidgetItem(m_featureTree);
    rootItem->setText(0, "加工特征");
    rootItem->setExpanded(true);

    for (const auto& [type, count] : faceTypes)
    {
        auto* item = new QTreeWidgetItem(rootItem);
        item->setText(0, QString::fromStdString(type));
        item->setText(1, "铣削区域");
        item->setText(2, QString::number(count));
    }

    m_featureTree->expandAll();
}

void MainWindow::onGenerateToolpath()
{
    if (!m_hasModel || m_currentShape.IsNull())
    {
        QMessageBox::warning(this, "警告", "请先导入模型");
        return;
    }

    if (m_recognizedFaces.empty())
    {
        QMessageBox::warning(this, "警告", "请先识别特征");
        onRecognizeFeatures();
        if (m_recognizedFaces.empty())
            return;
    }

    m_logText->appendLog("开始生成刀路...");
    m_statusLabel->setText("正在生成刀路...");

    // 获取参数
    double toolDiameter = m_parameterPanel->toolDiameterSpin()->value();
    double stepover = m_parameterPanel->stepoverSpin()->value();
    double feedrate = m_parameterPanel->feedrateSpin()->value();
    double safeZ = m_parameterPanel->safeZSpin()->value();
    int strategyIndex = m_parameterPanel->strategyCombo()->currentIndex();

    m_logText->appendLog(QString("参数：刀具直径=%1mm, 步距=%2mm, 进给=%3mm/min, 安全高度=%4mm")
        .arg(toolDiameter).arg(stepover).arg(feedrate).arg(safeZ));

    // 选择最后一个面作为目标面
    TopoDS_Face targetFace = m_recognizedFaces.back();

    // 获取边界
    TopoDS_Wire boundaryWire;
    TopExp_Explorer exp(targetFace, TopAbs_WIRE);
    if (exp.More())
    {
        boundaryWire = TopoDS::Wire(exp.Current());
    }

    if (boundaryWire.IsNull())
    {
        m_logText->appendLog("无法获取面的边界");
        m_statusLabel->setText("刀路生成失败");
        return;
    }

    // 设置加工上下文
    PathStrategyContext ctx;
    ctx.setBoundaryWire(boundaryWire);
    ctx.setBoundaryFace(targetFace);
    ctx.setStockTop(5.0);
    ctx.setModelTop(0.0);
    ctx.setStepover(stepover);
    ctx.setCuttingAngle(0.0);
    ctx.setCuttingDirection(CuttingDirection::Zigzag);
    ctx.setToolDiameter(toolDiameter);
    ctx.setFeedrate(feedrate);
    ctx.setPlungeFeedrate(feedrate / 5);
    ctx.setSafeZ(safeZ);
    ctx.setLeadInEnabled(true);
    ctx.setLeadOutEnabled(true);
    ctx.setLeadInLength(5.0);
    ctx.setLeadOutLength(5.0);

    // 创建策略
    auto strategy = PathStrategyFactory::create(StrategyType::FaceMilling2D);
    if (!strategy)
    {
        m_logText->appendLog("无法创建加工策略");
        m_statusLabel->setText("刀路生成失败");
        return;
    }

    strategy->setContext(ctx);

    if (!strategy->validate())
    {
        m_logText->appendLog(QString("策略验证失败：%1").arg(QString::fromStdString(strategy->getLastError())));
        m_statusLabel->setText("刀路生成失败");
        return;
    }

    auto toolpath = strategy->generate();

    if (!toolpath || toolpath->points().empty())
    {
        m_logText->appendLog("刀路生成结果为空");
        m_statusLabel->setText("刀路生成失败");
        return;
    }

    // 显示刀路
    m_viewer->ShowToolpath(*toolpath);

    m_logText->appendLog(QString("刀路生成完成 - 共 %1 个点").arg(toolpath->points().size()));
    m_statusLabel->setText("刀路生成成功");

    // 刷新渲染
    m_renderWindow->Render();
}

void MainWindow::onClearModel()
{
    m_hasModel = false;
    m_currentShape = TopoDS_Shape();
    m_recognizedFaces.clear();
    m_featureTree->clear();
    m_modelInfoLabel->setText("无模型");
    m_statusLabel->setText("已清除模型");
    m_logText->appendLog("模型已清除");

    // 清空 VTK 显示
    if (m_viewer)
    {
        m_viewer->Clear();
        m_renderWindow->Render();
    }
}

void MainWindow::onAbout()
{
    QMessageBox::about(this, "关于 PathForge",
        "<h2>PathForge</h2>"
        "<p>特征识别与加工系统</p>"
        "<p>版本：1.0.0</p>"
        "<p>基于 OpenCASCADE 和 VTK 开发</p>"
        "<p>支持格式：STEP, IGES, STL, BRep</p>");
}

std::string MainWindow::getStrategyType() const
{
    int index = m_parameterPanel->strategyCombo()->currentIndex();
    switch (index)
    {
        case 0: return "FaceMilling2D";
        case 1: return "PocketMilling";
        case 2: return "ProfileMilling";
        case 3: return "Drilling";
        default: return "FaceMilling2D";
    }
}
