// AdaptiveCleaningStrategy.cpp
#include "AdaptiveCleaningStrategy.h"

#include <BRepBuilderAPI_MakeFace.hxx>
#include <GCPnts_QuasiUniformAbscissa.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopExp_Explorer.hxx>
#include <BRepTools.hxx>
#include <Precision.hxx>
#include <Geom_Circle.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS.hxx>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <functional>

namespace PathForge::Path {

// ==================== 构造函数 ====================
AdaptiveCleaningStrategy::AdaptiveCleaningStrategy()
    : m_offsetJoinType(GeomAbs_Arc)
    , m_initialStepover(5.0)
    , m_minStepover(1.0)
    , m_chordLength(0.5)
    , m_maxDiscretizationPoints(1000)
    , m_linkSegmentOptimization(true)
    , m_maxLiftHeight(2.0)
    , m_spiralModeMaxSpan(50.0)
    , m_maxScallopHeight(0.1)
    , m_maxRecursionDepth(10)
    , m_minRegionArea(1.0)
{
    m_trochoidalParams.loopRadius = 3.0;
    m_trochoidalParams.loopSpacing = 2.0;
    m_trochoidalParams.entryAngle = 0.0;
    m_trochoidalParams.isEnabled = false;
}

AdaptiveCleaningStrategy::AdaptiveCleaningStrategy(const PathStrategyContext& context)
    : AdaptiveCleaningStrategy()
{
    m_context = context;
}

// ==================== 虚函数实现 ====================
StrategyType AdaptiveCleaningStrategy::getType() const
{
    return StrategyType::ContourMilling;
}

std::string AdaptiveCleaningStrategy::getName() const
{
    return "AdaptiveCleaning";
}

std::string AdaptiveCleaningStrategy::getDescription() const
{
    return "2D Adaptive Cleaning Strategy with Multi-level Offsetting and "
           "Topology-adaptive Processing for CAM Area Filling";
}

bool AdaptiveCleaningStrategy::validate() const
{
    if (m_context.getBoundaryWire().IsNull()) {
        m_lastError = "Boundary wire is null";
        return false;
    }

    if (m_initialStepover <= 0.0) {
        m_lastError = "Initial stepover must be positive";
        return false;
    }

    if (m_minStepover <= 0.0 || m_minStepover > m_initialStepover) {
        m_lastError = "Invalid minimum stepover value";
        return false;
    }

    if (m_chordLength <= 0.0) {
        m_lastError = "Chord length must be positive";
        return false;
    }

    if (m_maxRecursionDepth <= 0 || m_maxRecursionDepth > 20) {
        m_lastError = "Recursion depth must be between 1 and 20";
        return false;
    }

    return true;
}

ToolpathPtr AdaptiveCleaningStrategy::generate()
{
    if (!validate()) {
        return nullptr;
    }

    logMessage("=== Starting Adaptive Cleaning Path Generation ===");

    m_offsetLayers.clear();
    m_linkSegments.clear();

    const TopoDS_Wire& boundaryWire = m_context.getBoundaryWire();
    double toolDiameter = m_context.getToolDiameter();
    double depth = m_context.getDepth();

    logMessage("Tool diameter: " + std::to_string(toolDiameter));
    logMessage("Initial stepover: " + std::to_string(m_initialStepover));

    // 检测区域跨度，决定使用何种模式
    std::vector<gp_Pnt> boundaryPoints = discretizeWire(boundaryWire, m_chordLength);
    double regionSpan = detectRegionSpan(boundaryPoints);

    logMessage("Region span: " + std::to_string(regionSpan));

    ToolpathPtr resultPath = nullptr;

    // 根据跨度选择加工模式
    if (regionSpan > m_spiralModeMaxSpan) {
        logMessage("Large region detected, using Spiral mode");
        resultPath = generateSpiralPath(boundaryWire, depth);
    }
    else if (m_trochoidalParams.isEnabled) {
        logMessage("Trochoidal mode enabled");
        resultPath = generateTrochoidalPath(boundaryWire, depth);
    }
    else {
        logMessage("Using standard Multi-level Offset mode");
        resultPath = generateStandardPath(performMultiLevelOffset(boundaryWire));
    }

    if (resultPath) {
        resultPath->setName("AdaptiveCleaning_" + getName());
        resultPath->setDepth(depth);
        logMessage("Path generation completed successfully");
    }

    return resultPath;
}

// ==================== Setter/Getter 方法 ====================
void AdaptiveCleaningStrategy::setOffsetJoinType(GeomAbs_JoinType joinType)
{
    m_offsetJoinType = joinType;
}

GeomAbs_JoinType AdaptiveCleaningStrategy::offsetJoinType() const
{
    return m_offsetJoinType;
}

void AdaptiveCleaningStrategy::setInitialStepover(double stepover)
{
    if (stepover > 0.0) {
        m_initialStepover = stepover;
    }
}

double AdaptiveCleaningStrategy::initialStepover() const
{
    return m_initialStepover;
}

void AdaptiveCleaningStrategy::setMinStepover(double minStepover)
{
    if (minStepover > 0.0) {
        m_minStepover = minStepover;
    }
}

double AdaptiveCleaningStrategy::minStepover() const
{
    return m_minStepover;
}

void AdaptiveCleaningStrategy::setDiscretizationChordLength(double chordLength)
{
    if (chordLength > 0.0) {
        m_chordLength = chordLength;
    }
}

double AdaptiveCleaningStrategy::discretizationChordLength() const
{
    return m_chordLength;
}

void AdaptiveCleaningStrategy::setMaxDiscretizationPoints(int maxPoints)
{
    if (maxPoints > 0) {
        m_maxDiscretizationPoints = maxPoints;
    }
}

int AdaptiveCleaningStrategy::maxDiscretizationPoints() const
{
    return m_maxDiscretizationPoints;
}

void AdaptiveCleaningStrategy::setLinkSegmentOptimization(bool enabled)
{
    m_linkSegmentOptimization = enabled;
}

bool AdaptiveCleaningStrategy::isLinkSegmentOptimizationEnabled() const
{
    return m_linkSegmentOptimization;
}

void AdaptiveCleaningStrategy::setMaxLiftHeight(double height)
{
    if (height > 0.0) {
        m_maxLiftHeight = height;
    }
}

double AdaptiveCleaningStrategy::maxLiftHeight() const
{
    return m_maxLiftHeight;
}

void AdaptiveCleaningStrategy::setTrochoidalParams(const TrochoidalParams& params)
{
    m_trochoidalParams = params;
}

const TrochoidalParams& AdaptiveCleaningStrategy::trochoidalParams() const
{
    return m_trochoidalParams;
}

void AdaptiveCleaningStrategy::setSpiralModeMaxSpan(double maxSpan)
{
    if (maxSpan > 0.0) {
        m_spiralModeMaxSpan = maxSpan;
    }
}

double AdaptiveCleaningStrategy::spiralModeMaxSpan() const
{
    return m_spiralModeMaxSpan;
}

void AdaptiveCleaningStrategy::setMaxScallopHeight(double scallopHeight)
{
    if (scallopHeight > 0.0) {
        m_maxScallopHeight = scallopHeight;
    }
}

double AdaptiveCleaningStrategy::maxScallopHeight() const
{
    return m_maxScallopHeight;
}

void AdaptiveCleaningStrategy::setMaxRecursionDepth(int depth)
{
    if (depth > 0 && depth <= 20) {
        m_maxRecursionDepth = depth;
    }
}

int AdaptiveCleaningStrategy::maxRecursionDepth() const
{
    return m_maxRecursionDepth;
}

void AdaptiveCleaningStrategy::setMinRegionArea(double minArea)
{
    if (minArea > 0.0) {
        m_minRegionArea = minArea;
    }
}

double AdaptiveCleaningStrategy::minRegionArea() const
{
    return m_minRegionArea;
}

const std::vector<OffsetLayer>& AdaptiveCleaningStrategy::getOffsetLayers() const
{
    return m_offsetLayers;
}

const std::vector<LinkSegment>& AdaptiveCleaningStrategy::getLinkSegments() const
{
    return m_linkSegments;
}

const std::vector<std::string>& AdaptiveCleaningStrategy::getAlgorithmLog() const
{
    return m_algorithmLog;
}

void AdaptiveCleaningStrategy::logMessage(const std::string& message)
{
    m_algorithmLog.push_back(message);
}

// ==================== 核心算法：多级等距偏移（使用 WireOffsetter） ====================
std::vector<OffsetLayer> AdaptiveCleaningStrategy::performMultiLevelOffset(
    const TopoDS_Wire& boundaryWire)
{
    logMessage("Performing Multi-level Offset using WireOffsetter...");

    std::vector<OffsetLayer> allLayers;

    // 配置 WireOffsetter
    OffsetOptions options;
    options.stepSize = m_initialStepover;
    options.minArea = m_minRegionArea;
    options.maxIterations = m_maxRecursionDepth;
    options.tolerance = m_chordLength;
    options.keepPoints = true;
    options.discretizationSegments = 50;

    // 设置连接类型
    switch (m_offsetJoinType) {
        case GeomAbs_Arc:
            options.joinType = OffsetOptions::JoinType::Arc;
            break;
        case GeomAbs_Intersection:
            options.joinType = OffsetOptions::JoinType::Intersection;
            break;
        default:
            options.joinType = OffsetOptions::JoinType::Arc;
            break;
    }

    WireOffsetter offsetter(options);

    // 执行多次偏置
    std::vector<OffsetResult> offsetResults = offsetter.performMultiOffset(boundaryWire);

    // 将 OffsetResult 转换为 OffsetLayer
    for (const auto& result : offsetResults) {
        OffsetLayer layer;
        layer.wire = result.wire;
        layer.offsetDistance = result.offsetDistance;
        layer.subRegionIndex = 0;
        layer.isSplit = false;

        // 离散化 Wire 获取点集
        if (result.points.empty()) {
            layer.discretizedPoints = discretizeWire(result.wire, m_chordLength);
        } else {
            // 使用 WireOffsetter 生成的点集，转换为 3D 点
            for (const auto& pt : result.points) {
                layer.discretizedPoints.push_back(pt);
            }
        }

        allLayers.push_back(layer);
    }

    // 记录 WireOffsetter 的日志
    for (const auto& log : offsetter.getLog()) {
        logMessage("[WireOffsetter] " + log);
    }

    logMessage("Generated " + std::to_string(allLayers.size()) + " offset layers");

    m_offsetLayers = allLayers;
    return allLayers;
}

// ==================== 几何处理方法 ====================
TopoDS_Face AdaptiveCleaningStrategy::wireToFace(const TopoDS_Wire& wire)
{
    BRepBuilderAPI_MakeFace makeFace(wire, Precision::Confusion());

    if (makeFace.IsDone()) {
        return makeFace.Face();
    }

    return TopoDS_Face();
}

std::vector<gp_Pnt> AdaptiveCleaningStrategy::discretizeWire(
    const TopoDS_Wire& wire,
    double chordLength)
{
    std::vector<gp_Pnt> points;

    // 遍历 Wire 中的所有边
    for (TopExp_Explorer exp(wire, TopAbs_EDGE); exp.More(); exp.Next()) {
        TopoDS_Edge edge = TopoDS::Edge(exp.Current());
        BRepAdaptor_Curve adaptor(edge);

        // 使用 GCPnts_QuasiUniformAbscissa 进行等弦长离散化
        GCPnts_QuasiUniformAbscissa discretizer(adaptor, chordLength);

        if (discretizer.IsDone()) {
            int nbPoints = discretizer.NbPoints();

            // 限制最大点数
            if (nbPoints > m_maxDiscretizationPoints) {
                double newChordLength = adaptor.LastParameter() - adaptor.FirstParameter();
                if (nbPoints > 1) {
                    newChordLength = newChordLength / (nbPoints - 1);
                }
                GCPnts_QuasiUniformAbscissa newDiscretizer(adaptor, newChordLength);
                if (newDiscretizer.IsDone()) {
                    nbPoints = newDiscretizer.NbPoints();
                    for (int i = 1; i <= nbPoints; ++i) {
                        gp_Pnt pt = adaptor.Value(newDiscretizer.Parameter(i));
                        points.push_back(pt);
                    }
                }
            } else {
                for (int i = 1; i <= nbPoints; ++i) {
                    gp_Pnt pt = adaptor.Value(discretizer.Parameter(i));
                    points.push_back(pt);
                }
            }
        }
    }

    return points;
}

LinkSegment AdaptiveCleaningStrategy::findShortestLinkSegment(
    const std::vector<gp_Pnt>& points1,
    const std::vector<gp_Pnt>& points2)
{
    LinkSegment bestSegment;
    bestSegment.distance = std::numeric_limits<double>::max();

    if (points1.empty() || points2.empty()) {
        return bestSegment;
    }

    // 寻找欧氏距离最短的点对
    for (const auto& pt1 : points1) {
        for (const auto& pt2 : points2) {
            double dist = pt1.Distance(pt2);
            if (dist < bestSegment.distance) {
                bestSegment.distance = dist;
                bestSegment.startPoint = pt1;
                bestSegment.endPoint = pt2;
            }
        }
    }

    return bestSegment;
}

double AdaptiveCleaningStrategy::detectRegionSpan(const std::vector<gp_Pnt>& points)
{
    if (points.empty()) {
        return 0.0;
    }

    auto [minX, maxX, minY, maxY] = computeBoundingBox2D(points);
    double spanX = maxX - minX;
    double spanY = maxY - minY;

    return std::max(spanX, spanY);
}

std::tuple<double, double, double, double> AdaptiveCleaningStrategy::computeBoundingBox2D(
    const std::vector<gp_Pnt>& points)
{
    if (points.empty()) {
        return {0.0, 0.0, 0.0, 0.0};
    }

    double minX = points[0].X();
    double maxX = points[0].X();
    double minY = points[0].Y();
    double maxY = points[0].Y();

    for (const auto& pt : points) {
        minX = std::min(minX, pt.X());
        maxX = std::max(maxX, pt.X());
        minY = std::min(minY, pt.Y());
        maxY = std::max(maxY, pt.Y());
    }

    return {minX, maxX, minY, maxY};
}

double AdaptiveCleaningStrategy::pointToLineDistance(
    const gp_Pnt& point,
    const gp_Pnt& lineStart,
    const gp_Pnt& lineEnd)
{
    gp_Vec lineVec(lineStart.XYZ() - lineEnd.XYZ());
    gp_Vec pointVec(lineStart.XYZ() - point.XYZ());

    double lineLengthSq = lineVec.SquareMagnitude();
    if (lineLengthSq < Precision::SquareConfusion()) {
        return point.Distance(lineStart);
    }

    double t = std::max(0.0, std::min(1.0, pointVec.Dot(lineVec) / lineLengthSq));
    gp_Pnt closestPt(
        lineStart.X() + t * (lineEnd.X() - lineStart.X()),
        lineStart.Y() + t * (lineEnd.Y() - lineStart.Y()),
        lineStart.Z() + t * (lineEnd.Z() - lineStart.Z())
    );

    return point.Distance(closestPt);
}

// ==================== 路径生成方法 ====================
ToolpathPtr AdaptiveCleaningStrategy::generateStandardPath(
    const std::vector<OffsetLayer>& layers)
{
    if (layers.empty()) {
        return nullptr;
    }

    auto path = std::make_shared<Toolpath>("AdaptiveCleaning_Standard");
    path->setDepth(m_context.getDepth());

    double feedrate = m_context.getFeedrate();
    double plungeFeedrate = m_context.getPlungeFeedrate();
    double safeZ = m_context.getSafeZ();

    gp_Pnt lastPoint;
    bool isFirstPoint = true;

    // 遍历所有偏移层
    for (size_t i = 0; i < layers.size(); ++i) {
        const auto& layer = layers[i];

        if (layer.discretizedPoints.empty()) {
            continue;
        }

        // 添加连接段（如果不是第一层）
        if (i > 0 && m_linkSegmentOptimization) {
            LinkSegment link = findShortestLinkSegment(
                layers[i - 1].discretizedPoints,
                layer.discretizedPoints
            );

            if (link.distance > Precision::Confusion()) {
                // 抬刀移动到连接点
                path->addPoint(PathPoint(gp_Pnt(link.startPoint.X(), link.startPoint.Y(), safeZ),
                                         MoveType::Rapid));
                // 下刀
                path->addPoint(PathPoint(gp_Pnt(link.startPoint.X(), link.startPoint.Y(), m_context.getDepth()),
                                         MoveType::Linear));
                // 切削移动到目标层起点
                path->addPoint(PathPoint(gp_Pnt(link.endPoint.X(), link.endPoint.Y(), m_context.getDepth()),
                                         MoveType::Linear));

                m_linkSegments.push_back(link);
                link.fromLayer = static_cast<int>(i - 1);
                link.toLayer = static_cast<int>(i);
            }
        }

        // 添加当前层的切削路径
        for (size_t j = 0; j < layer.discretizedPoints.size(); ++j) {
            const auto& pt = layer.discretizedPoints[j];
            gp_Pnt pathPt(pt.X(), pt.Y(), m_context.getDepth());

            if (isFirstPoint) {
                // 第一点：快速定位 + 下刀
                path->addPoint(PathPoint(gp_Pnt(pt.X(), pt.Y(), safeZ), MoveType::Rapid));
                path->addPoint(PathPoint(pathPt, MoveType::Linear));
                path->points().back().motionType = MotionType::Plunge;
                path->points().back().feedrate = plungeFeedrate;
                isFirstPoint = false;
            } else {
                PathPoint pathPoint(pathPt, MoveType::Linear);
                pathPoint.feedrate = feedrate;
                pathPoint.motionType = MotionType::Cutting;
                path->addPoint(pathPoint);
            }

            lastPoint = pathPt;
        }
    }

    // 抬刀退出
    if (!isFirstPoint && path->pointCount() > 0) {
        path->addPoint(PathPoint(gp_Pnt(lastPoint.X(), lastPoint.Y(), safeZ), MoveType::Rapid));
        path->points().back().motionType = MotionType::Lift;
    }

    return path;
}

ToolpathPtr AdaptiveCleaningStrategy::generateSpiralPath(
    const TopoDS_Wire& boundaryWire,
    double depth)
{
    logMessage("Generating Spiral Path...");

    auto path = std::make_shared<Toolpath>("AdaptiveCleaning_Spiral");
    path->setDepth(depth);

    double feedrate = m_context.getFeedrate();
    double plungeFeedrate = m_context.getPlungeFeedrate();
    double safeZ = m_context.getSafeZ();
    double stepover = m_initialStepover;

    // 计算面中心
    TopoDS_Face face = wireToFace(boundaryWire);
    GProp_GProps props;
    BRepGProp::SurfaceProperties(face, props);
    gp_Pnt center = props.CentreOfMass();

    // 离散化边界
    std::vector<gp_Pnt> boundaryPoints = discretizeWire(boundaryWire, m_chordLength);

    // 从中心开始螺旋向外
    double currentRadius = 0.0;
    double maxRadius = detectRegionSpan(boundaryPoints) / 2.0;

    gp_Pnt startPoint(center.X(), center.Y(), safeZ);
    path->addPoint(PathPoint(startPoint, MoveType::Rapid));

    // 下刀到中心
    gp_Pnt plungePt(center.X(), center.Y(), depth);
    path->addPoint(PathPoint(plungePt, MoveType::Linear));
    path->points().back().motionType = MotionType::Plunge;
    path->points().back().feedrate = plungeFeedrate;

    // 生成螺旋路径
    double angle = 0.0;
    gp_Pnt lastPt = plungePt;

    while (currentRadius < maxRadius) {
        // 计算当前角度上的点
        double x = center.X() + currentRadius * std::cos(angle);
        double y = center.Y() + currentRadius * std::sin(angle);
        gp_Pnt spiralPt(x, y, depth);

        // 检查点是否在边界内
        bool inside = true; // 简化处理，实际应使用点在多边形内算法

        PathPoint pathPoint(spiralPt, MoveType::Linear);
        pathPoint.feedrate = feedrate;
        pathPoint.motionType = MotionType::Cutting;
        path->addPoint(pathPoint);

        angle += 0.1;  // 角度增量
        currentRadius += stepover / (2 * M_PI);  // 半径逐渐增加
    }

    // 沿边界完成最后一圈
    for (const auto& pt : boundaryPoints) {
        gp_Pnt boundaryPathPt(pt.X(), pt.Y(), depth);
        PathPoint pathPoint(boundaryPathPt, MoveType::Linear);
        pathPoint.feedrate = feedrate;
        pathPoint.motionType = MotionType::Cutting;
        path->addPoint(pathPoint);
    }

    // 抬刀退出
    path->addPoint(PathPoint(gp_Pnt(center.X(), center.Y(), safeZ), MoveType::Rapid));
    path->points().back().motionType = MotionType::Lift;

    return path;
}

ToolpathPtr AdaptiveCleaningStrategy::generateTrochoidalPath(
    const TopoDS_Wire& boundaryWire,
    double depth)
{
    logMessage("Generating Trochoidal Path...");

    auto path = std::make_shared<Toolpath>("AdaptiveCleaning_Trochoidal");
    path->setDepth(depth);

    double feedrate = m_context.getFeedrate();
    double plungeFeedrate = m_context.getPlungeFeedrate();
    double safeZ = m_context.getSafeZ();

    double loopRadius = m_trochoidalParams.loopRadius;
    double loopSpacing = m_trochoidalParams.loopSpacing;

    // 离散化边界
    std::vector<gp_Pnt> boundaryPoints = discretizeWire(boundaryWire, m_chordLength);

    if (boundaryPoints.empty()) {
        return nullptr;
    }

    // 计算边界中心
    double sumX = 0, sumY = 0;
    for (const auto& pt : boundaryPoints) {
        sumX += pt.X();
        sumY += pt.Y();
    }
    gp_Pnt center(sumX / boundaryPoints.size(), sumY / boundaryPoints.size(), depth);

    // 快速定位到起点
    gp_Pnt startPoint = boundaryPoints[0];
    path->addPoint(PathPoint(gp_Pnt(startPoint.X(), startPoint.Y(), safeZ), MoveType::Rapid));

    // 下刀
    gp_Pnt plungePt(startPoint.X(), startPoint.Y(), depth);
    path->addPoint(PathPoint(plungePt, MoveType::Linear));
    path->points().back().motionType = MotionType::Plunge;
    path->points().back().feedrate = plungeFeedrate;

    // 生成摆线路径
    // 摆线参数方程：x = r(t - sin(t)), y = r(1 - cos(t))
    int numLoops = static_cast<int>(detectRegionSpan(boundaryPoints) / loopSpacing);

    for (int i = 0; i < numLoops; ++i) {
        double baseAngle = i * 2 * M_PI;

        for (double t = 0; t <= 2 * M_PI; t += 0.1) {
            double angle = baseAngle + t;

            // 摆线环
            double loopX = loopRadius * (t - std::sin(t));
            double loopY = loopRadius * (1 - std::cos(t));

            // 旋转到边界法向
            double rotatedX = loopX * std::cos(angle) - loopY * std::sin(angle);
            double rotatedY = loopX * std::sin(angle) + loopY * std::cos(angle);

            // 沿边界前进
            double progress = static_cast<double>(i) / numLoops;
            size_t boundaryIndex = static_cast<size_t>(progress * (boundaryPoints.size() - 1));
            gp_Pnt boundaryPt = boundaryPoints[boundaryIndex];

            double x = boundaryPt.X() + rotatedX;
            double y = boundaryPt.Y() + rotatedY;

            gp_Pnt trochoidalPt(x, y, depth);
            PathPoint pathPoint(trochoidalPt, MoveType::Linear);
            pathPoint.feedrate = feedrate;
            pathPoint.motionType = MotionType::Cutting;
            path->addPoint(pathPoint);
        }
    }

    // 抬刀退出
    path->addPoint(PathPoint(gp_Pnt(startPoint.X(), startPoint.Y(), safeZ), MoveType::Rapid));
    path->points().back().motionType = MotionType::Lift;

    return path;
}

void AdaptiveCleaningStrategy::addLinkSegmentsToPath(
    ToolpathPtr path,
    const std::vector<LinkSegment>& linkSegments)
{
    if (!path || linkSegments.empty()) {
        return;
    }

    double safeZ = m_context.getSafeZ();
    double feedrate = m_context.getFeedrate();

    for (const auto& link : linkSegments) {
        // 抬刀
        path->addPoint(PathPoint(gp_Pnt(link.startPoint.X(), link.startPoint.Y(), safeZ),
                                 MoveType::Rapid));
        // 移动到目标点上方
        path->addPoint(PathPoint(gp_Pnt(link.endPoint.X(), link.endPoint.Y(), safeZ),
                                 MoveType::Rapid));
        // 下刀
        path->addPoint(PathPoint(gp_Pnt(link.endPoint.X(), link.endPoint.Y(), m_context.getDepth()),
                                 MoveType::Linear));
        path->points().back().feedrate = feedrate;
    }
}

bool AdaptiveCleaningStrategy::checkAndFillScallops(
    std::vector<OffsetLayer>& layers,
    double toolDiameter)
{
    bool needsFill = false;
    double maxAllowedGap = toolDiameter - m_maxScallopHeight;

    // 检查相邻层之间的间隙
    for (size_t i = 0; i < layers.size() - 1; ++i) {
        const auto& currentLayer = layers[i];
        const auto& nextLayer = layers[i + 1];

        if (currentLayer.discretizedPoints.empty() || nextLayer.discretizedPoints.empty()) {
            continue;
        }

        // 采样检查间隙
        for (const auto& pt : nextLayer.discretizedPoints) {
            double minDist = std::numeric_limits<double>::max();

            for (const auto& refPt : currentLayer.discretizedPoints) {
                double dist = pt.Distance(refPt);
                minDist = std::min(minDist, dist);
            }

            if (minDist > maxAllowedGap) {
                logMessage("Scallop detected: gap=" + std::to_string(minDist));
                needsFill = true;
                // 在此处可以添加补充路径
                break;
            }
        }
    }

    return needsFill;
}

} // namespace PathForge::Path
