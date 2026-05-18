#include "MainWindow.h"
#include "Topology/FaceCollector.h"
#include "Path/StrategyFactory.h"
#include "Path/Toolpath.h"
#include "Path/PostProcess/PostProcessorFactory.h"

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

    setWindowTitle("PathForge - ����ʶ����ӹ�ϵͳ");
    resize(1600, 900);

    m_logText->appendLog("Ӧ�ó��������");
    m_logText->appendLog("�뵼�� STEP/IGES/STL ģ���ļ���ʼ����");
}

void MainWindow::setupUI()
{
    m_centralWidget = new QWidget(this);
    setCentralWidget(m_centralWidget);

    m_centralLayout = new QHBoxLayout(m_centralWidget);
    m_centralLayout->setContentsMargins(0, 0, 0, 0);

    m_vtkWidget = new QVTKOpenGLNativeWidget(this);
    m_vtkWidget->setMinimumSize(600, 400);
    m_vtkWidget->setStyleSheet("background-color: #1a1a2e;");

    m_centralLayout->addWidget(m_vtkWidget, 1);

    m_renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    m_renderer = vtkSmartPointer<vtkRenderer>::New();
    m_renderWindow->AddRenderer(m_renderer);
    m_vtkWidget->setRenderWindow(m_renderWindow);

    m_renderer->SetBackground(0.1, 0.1, 0.15);
    m_renderer->SetBackground2(0.2, 0.2, 0.3);
    m_renderer->GradientBackgroundOn();

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

    m_markerWidget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    m_markerWidget->SetOrientationMarker(axes_actor);
    m_markerWidget->SetInteractor(m_vtkWidget->interactor());
    m_markerWidget->SetViewport(0.0, 0.0, 0.2, 0.2);
    m_markerWidget->SetEnabled(1);
    m_markerWidget->SetOutlineColor(1, 0, 0);

    m_viewer = std::make_unique<VtkViewer>();
    m_viewer->SetRenderWindow(m_renderWindow);
}

void MainWindow::createMenus()
{
    auto* fileMenu = menuBar()->addMenu("�ļ� (&F)");
    
    auto* importAction = fileMenu->addAction("����ģ�� (&I)");
    importAction->setShortcut(QKeySequence::Open);
    connect(importAction, &QAction::triggered, this, &MainWindow::onImportModel);

    fileMenu->addSeparator();

    auto* exitAction = fileMenu->addAction("�˳� (&X)");
    exitAction->setShortcut(QKeySequence::Quit);
    connect(exitAction, &QAction::triggered, this, &QMainWindow::close);

    auto* featureMenu = menuBar()->addMenu("���� (&E)");
    
    auto* recognizeAction = featureMenu->addAction("ʶ������ (&R)");
    recognizeAction->setShortcut(QKeySequence(Qt::Key_F5));
    connect(recognizeAction, &QAction::triggered, this, &MainWindow::onRecognizeFeatures);

    featureMenu->addSeparator();

    auto* clearAction = featureMenu->addAction("���ģ�� (&C)");
    clearAction->setShortcut(QKeySequence::Delete);
    connect(clearAction, &QAction::triggered, this, &MainWindow::onClearModel);

    auto* machiningMenu = menuBar()->addMenu("�ӹ� (&M)");
    
    auto* generateAction = machiningMenu->addAction("���ɵ�· (&G)");
    generateAction->setShortcut(QKeySequence(Qt::Key_F9));
    connect(generateAction, &QAction::triggered, this, &MainWindow::onGenerateToolpath);

    machiningMenu->addSeparator();

    auto* exportAction = machiningMenu->addAction("���� G-code (&E)");
    exportAction->setShortcut(QKeySequence(Qt::Key_F10));
    connect(exportAction, &QAction::triggered, this, &MainWindow::onExportGCode);\n\n    auto* simulationMenu = menuBar()->addMenu("���� (&S)");\n    \n    auto* runSimAction = simulationMenu->addAction("���з��� (&R)");\n    runSimAction->setShortcut(QKeySequence(Qt::Key_F11));\n    connect(runSimAction, &QAction::triggered, this, &MainWindow::onRunSimulation);

    auto* helpMenu = menuBar()->addMenu("���� (&H)");
    
    auto* aboutAction = helpMenu->addAction("���� (&A)");
    connect(aboutAction, &QAction::triggered, this, &MainWindow::onAbout);
}

void MainWindow::createToolbars()
{
    auto* toolbar = addToolBar("��������");
    toolbar->setMovable(false);

    auto* importBtn = new QPushButton("����ģ��");
    importBtn->setStyleSheet("padding: 5px 10px;");
    connect(importBtn, &QPushButton::clicked, this, &MainWindow::onImportModel);
    toolbar->addWidget(importBtn);

    toolbar->addSeparator();

    auto* recognizeBtn = new QPushButton("ʶ������");
    recognizeBtn->setStyleSheet("padding: 5px 10px;");
    connect(recognizeBtn, &QPushButton::clicked, this, &MainWindow::onRecognizeFeatures);
    toolbar->addWidget(recognizeBtn);

    toolbar->addSeparator();

    auto* generateBtn = new QPushButton("���ɵ�·");
    generateBtn->setStyleSheet("padding: 5px 10px; background-color: #4CAF50; color: white; font-weight: bold;");
    connect(generateBtn, &QPushButton::clicked, this, &MainWindow::onGenerateToolpath);
    toolbar->addWidget(generateBtn);

    auto* exportBtn = new QPushButton("����G-code");
    exportBtn->setStyleSheet("padding: 5px 10px; background-color: #2196F3; color: white; font-weight: bold;");
    connect(exportBtn, &QPushButton::clicked, this, &MainWindow::onExportGCode);\n\n    auto* simBtn = new QPushButton("���з���");\n    simBtn->setStyleSheet("padding: 5px 10px; background-color: #FF9800; color: white; font-weight: bold;");\n    connect(simBtn, &QPushButton::clicked, this, &MainWindow::onRunSimulation);\n    toolbar->addWidget(simBtn);
    toolbar->addWidget(exportBtn);

    toolbar->addSeparator();

    auto* clearBtn = new QPushButton("���");
    clearBtn->setStyleSheet("padding: 5px 10px;");
    connect(clearBtn, &QPushButton::clicked, this, &MainWindow::onClearModel);
    toolbar->addWidget(clearBtn);
}

void MainWindow::createDockWidgets()
{
    m_featureDock = new QDockWidget("������", this);
    m_featureDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    m_featureTree = new FeatureTreeWidget(m_featureDock);
    m_featureDock->setWidget(m_featureTree);
    addDockWidget(Qt::RightDockWidgetArea, m_featureDock);

    m_parameterDock = new QDockWidget("�ӹ�����", this);
    m_parameterDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    m_parameterPanel = new ParameterPanel(m_parameterDock);
    m_parameterDock->setWidget(m_parameterPanel);
    addDockWidget(Qt::RightDockWidgetArea, m_parameterDock);

    connect(m_parameterPanel->generateButton(), &QPushButton::clicked, this, &MainWindow::onGenerateToolpath);

    m_logDock = new QDockWidget("��־", this);
    m_logDock->setAllowedAreas(Qt::BottomDockWidgetArea);
    m_logText = new LogTextEdit(m_logDock);
    m_logDock->setWidget(m_logText);
    addDockWidget(Qt::BottomDockWidgetArea, m_logDock);

    m_statusLabel = new QLabel("����");
    m_modelInfoLabel = new QLabel("��ģ��");
    m_statusLabel->setMinimumWidth(200);
    m_modelInfoLabel->setMinimumWidth(150);

    statusBar()->addWidget(m_statusLabel);
    statusBar()->addPermanentWidget(m_modelInfoLabel);
}

void MainWindow::onImportModel()
{
    QString filter = "CAD ģ���ļ� (*.step *.stp *.STEP *.STP *.iges *.igs *.IGES *.IGS *.stl *.STL *.brep *.BRep *.BREP);;�����ļ� (*)";
    QString filePath = QFileDialog::getOpenFileName(this, "����ģ��", "", filter);

    if (filePath.isEmpty())
        return;

    loadModel(filePath);
}

void MainWindow::loadModel(const QString& filePath)
{
    m_logText->appendLog(QString("���ڼ���ģ�ͣ�%1").arg(filePath));
    m_statusLabel->setText("���ڼ���ģ��...");

    TopoDS_Shape shape;
    if (!m_modelLoader.LoadFile(filePath.toStdString(), shape) || shape.IsNull())
    {
        QMessageBox::critical(this, "����", 
            QString("�޷�����ģ��:\n%1").arg(QString::fromStdString(m_modelLoader.GetLastError())));
        m_statusLabel->setText("����ʧ��");
        m_logText->appendLog("ģ�ͼ���ʧ��");
        return;
    }

    m_currentShape = shape;
    m_hasModel = true;

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

    int faceCount = 0;
    TopExp_Explorer exp(shape, TopAbs_FACE);
    while (exp.More()) { faceCount++; exp.Next(); }

    m_viewer->ShowShape(shape);

    m_modelInfoLabel->setText(QString("ģ�ͣ�%1 | �棺%2 | ���:%3 mm3")
        .arg(QFileInfo(filePath).fileName())
        .arg(faceCount)
        .arg(volume, 0, 'f', 2));

    m_statusLabel->setText("ģ�ͼ��سɹ�");
    m_logText->appendLog(QString("ģ�ͼ��سɹ� - ������%1, �����%2 mm3, �ߴ磺%3 x %4 x %5 mm")
        .arg(faceCount)
        .arg(volume, 0, 'f', 2)
        .arg(sizeX, 0, 'f', 2)
        .arg(sizeY, 0, 'f', 2)
        .arg(sizeZ, 0, 'f', 2));

    m_featureTree->clear();
    m_recognizedFaces.clear();

    m_renderWindow->Render();
}

void MainWindow::onRecognizeFeatures()
{
    if (!m_hasModel || m_currentShape.IsNull())
    {
        QMessageBox::warning(this, "����", "���ȵ���ģ��");
        return;
    }

    m_logText->appendLog("��ʼ����ʶ��...");
    m_statusLabel->setText("����ʶ������...");

    FaceCollector collector;
    m_recognizedFaces = collector.collectFaces(m_currentShape);

    if (m_recognizedFaces.empty())
    {
        m_logText->appendLog("δʶ���κ�����");
        m_statusLabel->setText("δʶ������");
        QMessageBox::information(this, "��ʾ", "δʶ�𵽿ɼӹ�������");
        return;
    }

    displayFeatures();

    m_logText->appendLog(QString("����ʶ����� - ��ʶ�� %1 ������").arg(m_recognizedFaces.size()));
    m_statusLabel->setText(QString("��ʶ�� %1 ������").arg(m_recognizedFaces.size()));
}

void MainWindow::displayFeatures()
{
    m_featureTree->clear();

    std::map<std::string, int> faceTypes;
    
    for (const auto& face : m_recognizedFaces)
    {
        GProp_GProps props;
        BRepGProp::SurfaceProperties(face, props);
        double area = props.Mass();

        std::string type = "ƽ��";
        if (area < 100) type = "Сƽ��";
        else if (area < 500) type = "��ƽ��";
        else type = "��ƽ��";

        faceTypes[type]++;
    }

    auto* rootItem = new QTreeWidgetItem(m_featureTree);
    rootItem->setText(0, "�ӹ�����");
    rootItem->setExpanded(true);

    for (const auto& [type, count] : faceTypes)
    {
        auto* item = new QTreeWidgetItem(rootItem);
        item->setText(0, QString::fromStdString(type));
        item->setText(1, "ϳ������");
        item->setText(2, QString::number(count));
    }

    m_featureTree->expandAll();
}

void MainWindow::onGenerateToolpath()
{
    if (!m_hasModel || m_currentShape.IsNull())
    {
        QMessageBox::warning(this, "����", "���ȵ���ģ��");
        return;
    }

    if (m_recognizedFaces.empty())
    {
        QMessageBox::warning(this, "����", "����ʶ������");
        onRecognizeFeatures();
        if (m_recognizedFaces.empty())
            return;
    }

    m_logText->appendLog("��ʼ���ɵ�·...");
    m_statusLabel->setText("�������ɵ�·...");

    double toolDiameter = m_parameterPanel->toolDiameterSpin()->value();
    double stepover = m_parameterPanel->stepoverSpin()->value();
    double feedrate = m_parameterPanel->feedrateSpin()->value();
    double safeZ = m_parameterPanel->safeZSpin()->value();
    int strategyIndex = m_parameterPanel->strategyCombo()->currentIndex();

    m_logText->appendLog(QString("����������ֱ��=%1mm, ����=%2mm, ���=%5mm, ����=%3mm/min, ��ȫ�߶�=%4mm")
        .arg(toolDiameter).arg(stepover).arg(feedrate).arg(safeZ).arg(m_parameterPanel->depthSpin()->value());

    TopoDS_Face targetFace = m_recognizedFaces.back();

    TopoDS_Wire boundaryWire;
    TopExp_Explorer exp(targetFace, TopAbs_WIRE);
    if (exp.More())
    {
        boundaryWire = TopoDS::Wire(exp.Current());
    }

    if (boundaryWire.IsNull())
    {
        m_logText->appendLog("�޷���ȡ��ı߽�");
        m_statusLabel->setText("��·����ʧ��");
        return;
    }

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
    ctx.setDepth(m_parameterPanel->depthSpin()->value());
    ctx.setLeadInEnabled(true);
    ctx.setLeadOutEnabled(true);
    ctx.setLeadInLength(5.0);
    ctx.setLeadOutLength(5.0);

    StrategyType strategyType = StrategyType::FaceMilling2D;
    switch (strategyIndex) {
        case 0: strategyType = StrategyType::FaceMilling2D; break;
        case 1: strategyType = StrategyType::PocketMilling; break;
        case 2: strategyType = StrategyType::ContourMilling; break;
        case 3: strategyType = StrategyType::DrillCenter; break;
        default: strategyType = StrategyType::FaceMilling2D; break;
    }
    auto strategy = PathStrategyFactory::create(strategyType);
    if (!strategy)
    {
        m_logText->appendLog("�޷������ӹ�����");
        m_statusLabel->setText("��·����ʧ��");
        return;
    }

    strategy->setContext(ctx);

    if (!strategy->validate())
    {
        m_logText->appendLog(QString("������֤ʧ�ܣ�%1").arg(QString::fromStdString(strategy->getLastError())));
        m_statusLabel->setText("��·����ʧ��");
        return;
    }

    auto toolpath = strategy->generate();

    if (!toolpath || toolpath->points().empty())
    {
        m_logText->appendLog("��·���ɽ��Ϊ��");
        m_statusLabel->setText("��·����ʧ��");
        return;
    }

    m_currentToolpath = toolpath;
    m_viewer->ShowToolpath(*toolpath);

    m_logText->appendLog(QString("��·������� - �� %1 ����").arg(toolpath->points().size()));
    m_logText->appendLog("��ʾ: �� F10 ���� ����G-code ��ť���� NC ����");
    m_statusLabel->setText("��·���ɳɹ�");

    m_renderWindow->Render();
}

void MainWindow::onExportGCode()
{
    if (!m_currentToolpath || m_currentToolpath->isEmpty())
    {
        QMessageBox::warning(this, "����", "�������ɵ�·");
        return;
    }

    QString filter = "G-code �ļ� (*.nc *.gcode *.ncc);;�����ļ� (*)";
    QString filePath = QFileDialog::getSaveFileName(this, "���� G-code", "", filter);

    if (filePath.isEmpty())
        return;

    m_logText->appendLog("�������� G-code...");
    m_statusLabel->setText("���ں���...");

    try
    {
        m_postConfig.toolDiameter = m_parameterPanel->toolDiameterSpin()->value();
        m_postConfig.defaultFeedrate = m_parameterPanel->feedrateSpin()->value();
        m_postConfig.safeZ = m_parameterPanel->safeZSpin()->value();
        m_postConfig.defaultSpindleSpeed = 3000;
        m_postConfig.toolNumber = 1;
        m_postConfig.programName = m_currentToolpath->name().empty() ? "PathForge" : m_currentToolpath->name();

        auto gcode = PathForge::Post::postProcessToolpath(*m_currentToolpath, m_postConfig);

        if (gcode.empty())
        {
            QMessageBox::critical(this, "����", "G-code ����ʧ��");
            m_statusLabel->setText("G-code ����ʧ��");
            return;
        }

        if (PathForge::Post::saveGCode(gcode, filePath.toStdString()))
        {
            m_logText->appendLog(QString("G-code �ѵ�����: %1").arg(filePath));
            m_statusLabel->setText("G-code �����ɹ�");
            QMessageBox::information(this, "�ɹ�", QString("G-code �ѳɹ�������:\n%1").arg(filePath));
        }
        else
        {
            QMessageBox::critical(this, "����", "�޷������ļ�");
            m_statusLabel->setText("�ļ�����ʧ��");
        }
    }
    catch (const std::exception& e)
    {
        QMessageBox::critical(this, "����", QString("�����쳣: %1").arg(e.what()));
        m_statusLabel->setText("�����쳣");
        m_logText->appendLog(QString("�����쳣: %1").arg(e.what()));
    }
}


void MainWindow::onRunSimulation()
{
    if (!m_currentToolpath || m_currentToolpath->isEmpty())
    {
        QMessageBox::warning(this, "警告", "请先生成刀路");
        return;
    }
    m_logText->appendLog("开始运行仿真...");
    m_statusLabel->setText("正在仿真...");
    try
    {
        m_simulationEngine = std::make_unique<PathForge::Simulation::SimulationEngine>();
        m_simulationEngine->setWorkpiece(m_currentShape);
        m_simulationEngine->setToolGeometry(
            m_parameterPanel->toolDiameterSpin()->value(),
            50.0, 30.0, 40.0);
        auto results = m_simulationEngine->runSimulation(*m_currentToolpath);
        m_logText->appendLog(QString("仿真完成 - 风险等级: %1%").arg(results.overallRisk * 100, 0, 10, 1));
        m_logText->appendLog(QString("碰撞: %1处, 过切: %2处").arg(results.collisions.size()).arg(results.gouges.size()));
        m_logText->appendLog(QString("材料去除: %1mm3").arg(results.statistics.materialRemovalVolume, 0, 10, 2));
        if (results.hasCollisions)
        {
            m_logText->appendLog("警告: 检测到碰撞!");
            m_statusLabel->setText("碰撞警告");
            QMessageBox::warning(this, "碰撞", QString("检测到 %1 处碰撞").arg(results.collisions.size()));
        }
        else if (results.hasGouges)
        {
            m_logText->appendLog("警告: 检测到过切!");
            m_statusLabel->setText("过切警告");
            QMessageBox::warning(this, "过切", QString("检测到 %1 处过切").arg(results.gouges.size()));
        }
        else
        {
            m_logText->appendLog("仿真通过");
            m_statusLabel->setText("仿真通过");
            QMessageBox::information(this, "仿真", "仿真通过 - 刀路安全");
        }
    }
    catch (const std::exception& e)
    {
        QMessageBox::critical(this, "错误", QString("仿真异常: %1").arg(e.what()));
        m_statusLabel->setText("仿真失败");
    }
}void MainWindow::onClearModel()
{
    m_hasModel = false;
    m_currentToolpath.reset();
    m_currentShape = TopoDS_Shape();
    m_recognizedFaces.clear();
    m_featureTree->clear();
    m_modelInfoLabel->setText("��ģ��");
    m_statusLabel->setText("�����ģ��");
    m_logText->appendLog("ģ�������");

    if (m_viewer)
    {
        m_viewer->Clear();
        m_renderWindow->Render();
    }
}

void MainWindow::onAbout()
{
    QMessageBox::about(this, "���� PathForge",
        "<h2>PathForge</h2>"
        "<p>����ʶ����ӹ�ϵͳ</p>"
        "<p>�汾��1.0.0</p>"
        "<p>����ģ�飺֧��ͨ�� G-code ���</p>"
        "<p>���� OpenCASCADE �� VTK ����</p>"
        "<p>֧�ָ�ʽ��STEP, IGES, STL, BRep</p>");
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
