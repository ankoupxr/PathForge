#pragma once

#include <QMainWindow>
#include <QMenuBar>
#include <QToolBar>
#include <QStatusBar>
#include <QDockWidget>
#include <QTreeWidget>
#include <QTextEdit>
#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QFileDialog>
#include <QMessageBox>
#include <QFileInfo>
#include <QTime>
#include <QScrollBar>

#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>

#include "ModelLoader.h"
#include "VtkViewer.h"

#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <memory>

class QVTKOpenGLNativeWidget;

class FeatureTreeWidget : public QTreeWidget
{
    Q_OBJECT
public:
    explicit FeatureTreeWidget(QWidget* parent = nullptr) : QTreeWidget(parent)
    {
        setHeaderLabels({ "特征", "类型", "数量" });
        setAlternatingRowColors(true);
    }
};

class LogTextEdit : public QTextEdit
{
    Q_OBJECT
public:
    explicit LogTextEdit(QWidget* parent = nullptr) : QTextEdit(parent)
    {
        setReadOnly(true);
        setLineWrapMode(QTextEdit::NoWrap);
        setFont(QFont("Consolas", 9));
    }

    void appendLog(const QString& message)
    {
        QString timestamp = QTime::currentTime().toString("HH:mm:ss");
        append(QString("[%1] %2").arg(timestamp, message));
        verticalScrollBar()->setValue(verticalScrollBar()->maximum());
    }

    void clearLog() { clear(); }
};

class ParameterPanel : public QGroupBox
{
    Q_OBJECT
public:
    explicit ParameterPanel(QWidget* parent = nullptr) : QGroupBox("加工参数", parent)
    {
        auto* layout = new QFormLayout(this);

        m_toolDiameterSpin = new QDoubleSpinBox(this);
        m_toolDiameterSpin->setRange(0.1, 100.0);
        m_toolDiameterSpin->setValue(10.0);
        m_toolDiameterSpin->setSuffix(" mm");
        layout->addRow("刀具直径:", m_toolDiameterSpin);

        m_stepoverSpin = new QDoubleSpinBox(this);
        m_stepoverSpin->setRange(0.1, 50.0);
        m_stepoverSpin->setValue(5.0);
        m_stepoverSpin->setSuffix(" mm");
        layout->addRow("步距:", m_stepoverSpin);

        m_feedrateSpin = new QDoubleSpinBox(this);
        m_feedrateSpin->setRange(10, 10000);
        m_feedrateSpin->setValue(1500);
        m_feedrateSpin->setSuffix(" mm/min");
        layout->addRow("进给速度:", m_feedrateSpin);

        m_safeZSpin = new QDoubleSpinBox(this);
        m_safeZSpin->setRange(0.1, 100.0);
        m_safeZSpin->setValue(10.0);
        m_safeZSpin->setSuffix(" mm");
        layout->addRow("安全高度:", m_safeZSpin);

        m_strategyCombo = new QComboBox(this);
        m_strategyCombo->addItems({ "面铣削 2D", "型腔铣削", "轮廓铣削", "钻孔" });
        layout->addRow("加工策略:", m_strategyCombo);

        m_generateBtn = new QPushButton("生成刀路", this);
        m_generateBtn->setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 8px;");
        layout->addRow(m_generateBtn);
    }

    QDoubleSpinBox* toolDiameterSpin() const { return m_toolDiameterSpin; }
    QDoubleSpinBox* stepoverSpin() const { return m_stepoverSpin; }
    QDoubleSpinBox* feedrateSpin() const { return m_feedrateSpin; }
    QDoubleSpinBox* safeZSpin() const { return m_safeZSpin; }
    QComboBox* strategyCombo() const { return m_strategyCombo; }
    QPushButton* generateButton() const { return m_generateBtn; }

private:
    QDoubleSpinBox* m_toolDiameterSpin;
    QDoubleSpinBox* m_stepoverSpin;
    QDoubleSpinBox* m_feedrateSpin;
    QDoubleSpinBox* m_safeZSpin;
    QComboBox* m_strategyCombo;
    QPushButton* m_generateBtn;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() = default;

private slots:
    void onImportModel();
    void onRecognizeFeatures();
    void onGenerateToolpath();
    void onClearModel();
    void onAbout();

private:
    void setupUI();
    void createMenus();
    void createToolbars();
    void createDockWidgets();
    void loadModel(const QString& filePath);
    void displayFeatures();
    std::string getStrategyType() const;

    // UI Components
    QWidget* m_centralWidget;
    QHBoxLayout* m_centralLayout;

    // VTK Viewer
    QVTKOpenGLNativeWidget* m_vtkWidget;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_renderWindow;
    vtkSmartPointer<vtkRenderer> m_renderer;
    vtkSmartPointer<vtkOrientationMarkerWidget> m_markerWidget;
    std::unique_ptr<VtkViewer> m_viewer;

    // Docks
    QDockWidget* m_featureDock;
    QDockWidget* m_logDock;
    QDockWidget* m_parameterDock;

    FeatureTreeWidget* m_featureTree;
    LogTextEdit* m_logText;
    ParameterPanel* m_parameterPanel;

    // Model data
    ModelLoader m_modelLoader;
    TopoDS_Shape m_currentShape;
    std::vector<TopoDS_Face> m_recognizedFaces;
    bool m_hasModel;

    // Status bar
    QLabel* m_statusLabel;
    QLabel* m_modelInfoLabel;
};
