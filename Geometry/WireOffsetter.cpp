// WireOffsetter.cpp
#include "WireOffsetter.h"

#include <BRep_Tool.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopExp_Explorer.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Line.hxx>
#include <Geom_Circle.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <gp_Vec.hxx>
#include <gp_Lin2d.hxx>
#include <Precision.hxx>
#include <TopoDS.hxx>

#include <algorithm>
#include <cmath>
#include <sstream>

namespace PathForge {

// ==================== 构造函数 ====================
WireOffsetter::WireOffsetter() {
    m_options = OffsetOptions();
}

WireOffsetter::WireOffsetter(const OffsetOptions& options) {
    m_options = options;
}

// ==================== 公共方法 ====================
void WireOffsetter::setOptions(const OffsetOptions& options) {
    m_options = options;
}

const OffsetOptions& WireOffsetter::getOptions() const {
    return m_options;
}

const std::vector<std::string>& WireOffsetter::getLog() const {
    return m_log;
}

void WireOffsetter::clearLog() {
    m_log.clear();
}

void WireOffsetter::log(const std::string& message) {
    m_log.push_back(message);
}

// ==================== 核心偏置算法 ====================
std::vector<OffsetResult> WireOffsetter::performMultiOffset(const TopoDS_Wire& wire) {
    std::vector<OffsetResult> results;
    
    log("=== Starting Multi-Offset Operation ===");
    log("Step size: " + std::to_string(m_options.stepSize));
    log("Max iterations: " + std::to_string(m_options.maxIterations));
    log("Min area: " + std::to_string(m_options.minArea));

    // 添加原始轮廓
    OffsetResult initial;
    initial.wire = wire;
    initial.offsetDistance = 0.0;
    initial.iteration = 0;
    initial.isValid = true;
    initial.message = "Initial wire";

    if (m_options.keepPoints) {
        initial.points = projectWireTo2D(wire, m_options.discretizationSegments, m_options.tolerance);
    }

    results.push_back(initial);

    // 投影到 2D
    std::vector<gp_Pnt> currentPolygon = projectWireTo2D(wire, m_options.discretizationSegments, m_options.tolerance);
    
    if (currentPolygon.size() < 3) {
        log("Error: Input wire has less than 3 points");
        return results;
    }

    // 计算法向（用于确定向内方向）
    gp_Dir normal = computePolygonNormal(currentPolygon);
    log("Polygon normal: (" + std::to_string(normal.X()) + ", " + 
        std::to_string(normal.Y()) + ", " + std::to_string(normal.Z()) + ")");

    double cumulativeOffset = 0.0;
    
    for (int iteration = 1; iteration <= m_options.maxIterations; ++iteration) {
        cumulativeOffset += m_options.stepSize;
        
        log("\n--- Iteration " + std::to_string(iteration) + 
            " (offset=" + std::to_string(cumulativeOffset) + ") ---");

        // 执行偏置
        std::vector<gp_Pnt> offsetPolygon;
        
        switch (m_options.joinType) {
            case OffsetOptions::JoinType::Arc:
                offsetPolygon = offsetWithArcJoin(currentPolygon, m_options.stepSize, normal);
                break;
            case OffsetOptions::JoinType::Intersection:
                offsetPolygon = offsetWithIntersectionJoin(currentPolygon, m_options.stepSize, normal);
                break;
            case OffsetOptions::JoinType::Round:
                offsetPolygon = offsetWithArcJoin(currentPolygon, m_options.stepSize, normal);
                break;
        }

        // 检查偏置结果
        if (offsetPolygon.size() < 3) {
            log("Offset terminated: Result has less than 3 points");
            break;
        }

        // 检查自相交
        if (isPolygonSelfIntersecting(offsetPolygon, m_options.tolerance)) {
            log("Offset terminated: Self-intersection detected");
            break;
        }

        // 检查面积
        double area = computePolygonArea(offsetPolygon);
        log("Offset area: " + std::to_string(area));
        
        if (area < m_options.minArea) {
            log("Offset terminated: Area below minimum threshold");
            break;
        }

        // 简化多边形
        offsetPolygon = simplifyPolygon(offsetPolygon, m_options.tolerance);

        // 创建 Wire
        TopoDS_Wire offsetWire = createWireFrom2DPoints(offsetPolygon, currentPolygon[0].Z());
        
        if (offsetWire.IsNull()) {
            log("Offset terminated: Failed to create wire");
            break;
        }

        // 记录结果
        OffsetResult result;
        result.wire = offsetWire;
        result.offsetDistance = cumulativeOffset;
        result.iteration = iteration;
        result.isValid = true;
        result.message = "Iteration " + std::to_string(iteration) + ", area=" + std::to_string(area);
        
        if (m_options.keepPoints) {
            result.points = offsetPolygon;
        }
        
        results.push_back(result);
        currentPolygon = offsetPolygon;

        log("Iteration " + std::to_string(iteration) + " completed successfully");
    }

    log("\n=== Multi-Offset Completed ===");
    log("Total layers generated: " + std::to_string(results.size()));

    return results;
}

OffsetResult WireOffsetter::performSingleOffset(const TopoDS_Wire& wire, double distance) {
    log("=== Performing Single Offset (distance=" + std::to_string(distance) + ") ===");

    OffsetResult result;
    result.offsetDistance = distance;
    result.iteration = 1;

    // 投影到 2D
    std::vector<gp_Pnt> polygon = projectWireTo2D(wire, m_options.discretizationSegments, m_options.tolerance);
    
    if (polygon.size() < 3) {
        result.isValid = false;
        result.message = "Input wire has less than 3 points";
        return result;
    }

    // 计算法向
    gp_Dir normal = computePolygonNormal(polygon);

    // 执行偏置
    std::vector<gp_Pnt> offsetPolygon;
    
    switch (m_options.joinType) {
        case OffsetOptions::JoinType::Arc:
            offsetPolygon = offsetWithArcJoin(polygon, distance, normal);
            break;
        case OffsetOptions::JoinType::Intersection:
            offsetPolygon = offsetWithIntersectionJoin(polygon, distance, normal);
            break;
        case OffsetOptions::JoinType::Round:
            offsetPolygon = offsetWithArcJoin(polygon, distance, normal);
            break;
    }

    if (offsetPolygon.size() < 3) {
        result.isValid = false;
        result.message = "Offset resulted in less than 3 points";
        return result;
    }

    // 简化
    offsetPolygon = simplifyPolygon(offsetPolygon, m_options.tolerance);

    // 创建 Wire
    result.wire = createWireFrom2DPoints(offsetPolygon, polygon[0].Z());
    
    if (result.wire.IsNull()) {
        result.isValid = false;
        result.message = "Failed to create offset wire";
        return result;
    }

    // 检查面积
    double area = computePolygonArea(offsetPolygon);
    if (area < m_options.minArea) {
        result.isValid = false;
        result.message = "Offset area (" + std::to_string(area) + ") below minimum";
        return result;
    }

    result.isValid = true;
    result.message = "Success, area=" + std::to_string(area);
    
    if (m_options.keepPoints) {
        result.points = offsetPolygon;
    }

    return result;
}

// ==================== 几何处理方法 ====================
std::vector<gp_Pnt> WireOffsetter::projectWireTo2D(
    const TopoDS_Wire& wire,
    int discretizationSegments,
    double tolerance)
{
    std::vector<gp_Pnt> points;

    for (TopoDS_Iterator it(wire); it.More(); it.Next()) {
        TopoDS_Edge edge = TopoDS::Edge(it.Value());

        if (BRep_Tool::Degenerated(edge)) {
            continue;
        }

        double first, last;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);

        if (curve.IsNull()) {
            continue;
        }

        // 离散化边
        for (int i = 0; i <= discretizationSegments; ++i) {
            double t = first + (last - first) * i / discretizationSegments;
            gp_Pnt p = curve->Value(t);
            // 投影到 XY 平面（保留 Z 用于参考）
            points.push_back(gp_Pnt(p.X(), p.Y(), p.Z()));
        }
    }

    // 移除重复点
    if (points.size() > 1) {
        std::vector<gp_Pnt> unique;
        unique.push_back(points[0]);

        for (size_t i = 1; i < points.size(); ++i) {
            if (points[i].Distance(unique.back()) > tolerance) {
                unique.push_back(points[i]);
            }
        }

        // 检查首尾是否重复
        if (unique.size() > 1 &&
            unique.front().Distance(unique.back()) < tolerance) {
            unique.pop_back();
        }

        points = unique;
    }

    return points;
}

TopoDS_Wire WireOffsetter::createWireFrom2DPoints(
    const std::vector<gp_Pnt>& points, 
    double zHeight)
{
    if (points.size() < 3) {
        return TopoDS_Wire();
    }

    BRepBuilderAPI_MakeWire wireMaker;

    for (size_t i = 0; i < points.size(); ++i) {
        size_t nextIdx = (i + 1) % points.size();
        
        gp_Pnt p1(points[i].X(), points[i].Y(), zHeight);
        gp_Pnt p2(points[nextIdx].X(), points[nextIdx].Y(), zHeight);

        BRepBuilderAPI_MakeEdge edgeMaker(p1, p2);
        
        if (edgeMaker.IsDone()) {
            wireMaker.Add(edgeMaker.Edge());
        }
    }

    if (wireMaker.IsDone()) {
        return wireMaker.Wire();
    }

    return TopoDS_Wire();
}

double WireOffsetter::computePolygonArea(const std::vector<gp_Pnt>& points) {
    if (points.size() < 3) {
        return 0.0;
    }

    double area = 0.0;
    size_t n = points.size();

    for (size_t i = 0; i < n; ++i) {
        const auto& p1 = points[i];
        const auto& p2 = points[(i + 1) % n];
        area += (p1.X() * p2.Y() - p2.X() * p1.Y());
    }

    return std::abs(area) / 2.0;
}

gp_Dir WireOffsetter::computePolygonNormal(const std::vector<gp_Pnt>& points) {
    if (points.size() < 3) {
        return gp_Dir(0, 0, 1);
    }

    // 使用 Newell 方法计算法向
    double nx = 0, ny = 0, nz = 0;
    size_t n = points.size();

    for (size_t i = 0; i < n; ++i) {
        const auto& p1 = points[i];
        const auto& p2 = points[(i + 1) % n];

        nx += (p1.Y() - p2.Y()) * (p1.Z() + p2.Z());
        ny += (p1.Z() - p2.Z()) * (p1.X() + p2.X());
        nz += (p1.X() - p2.X()) * (p1.Y() + p2.Y());
    }

    double len = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (len < Precision::Confusion()) {
        return gp_Dir(0, 0, 1);
    }

    return gp_Dir(nx / len, ny / len, nz / len);
}

bool WireOffsetter::isPointInPolygon(
    const gp_Pnt& point,
    const std::vector<gp_Pnt>& polygon)
{
    if (polygon.empty()) {
        return false;
    }

    int crossings = 0;
    size_t n = polygon.size();

    for (size_t i = 0; i < n; ++i) {
        const auto& p1 = polygon[i];
        const auto& p2 = polygon[(i + 1) % n];

        if (((p1.Y() <= point.Y()) && (p2.Y() > point.Y())) ||
            ((p2.Y() <= point.Y()) && (p1.Y() > point.Y()))) {
            double xIntersect = p1.X() + 
                (point.Y() - p1.Y()) / (p2.Y() - p1.Y()) * (p2.X() - p1.X());
            if (point.X() < xIntersect) {
                crossings++;
            }
        }
    }

    return (crossings % 2) == 1;
}

// ==================== 偏置实现 ====================
std::pair<gp_Pnt, gp_Pnt> WireOffsetter::offsetEdge(
    const gp_Pnt& p1,
    const gp_Pnt& p2,
    double offsetDistance,
    const gp_Dir& normal)
{
    // 计算边向量
    gp_Vec edgeVec(p1.XYZ() - p2.XYZ());
    edgeVec.Normalize();

    // 计算法向（在 XY 平面内垂直于边）
    gp_Vec offsetDir = edgeVec ^ normal;
    offsetDir.Normalize();

    // 确保向内偏置（根据多边形顶点顺序）
    // 对于逆时针多边形，向内是右侧
    gp_Pnt offsetP1(
        p1.X() + offsetDir.X() * offsetDistance,
        p1.Y() + offsetDir.Y() * offsetDistance,
        p1.Z() + offsetDir.Z() * offsetDistance
    );
    
    gp_Pnt offsetP2(
        p2.X() + offsetDir.X() * offsetDistance,
        p2.Y() + offsetDir.Y() * offsetDistance,
        p2.Z() + offsetDir.Z() * offsetDistance
    );

    return {offsetP1, offsetP2};
}

bool WireOffsetter::computeLineIntersection(
    const gp_Pnt& p1, const gp_Pnt& p2,
    const gp_Pnt& p3, const gp_Pnt& p4,
    gp_Pnt& outPoint)
{
    // 使用 2D 线交点公式
    double x1 = p1.X(), y1 = p1.Y();
    double x2 = p2.X(), y2 = p2.Y();
    double x3 = p3.X(), y3 = p3.Y();
    double x4 = p4.X(), y4 = p4.Y();

    double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    
    if (std::abs(denom) < Precision::Confusion()) {
        return false; // 平行线
    }

    double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
    
    double x = x1 + t * (x2 - x1);
    double y = y1 + t * (y2 - y1);
    double z = (p1.Z() + p2.Z()) / 2.0; // 取平均 Z

    outPoint.SetX(x);
    outPoint.SetY(y);
    outPoint.SetZ(z);

    return true;
}

std::vector<gp_Pnt> WireOffsetter::offsetPolygon(
    const std::vector<gp_Pnt>& polygon,
    double distance,
    const gp_Dir& normal)
{
    std::vector<gp_Pnt> offsetPolygon;
    size_t n = polygon.size();

    if (n < 3) {
        return offsetPolygon;
    }

    // 为每条边计算偏置边
    std::vector<std::pair<gp_Pnt, gp_Pnt>> offsetEdges;
    
    for (size_t i = 0; i < n; ++i) {
        size_t nextIdx = (i + 1) % n;
        auto offsetEdge = this->offsetEdge(polygon[i], polygon[nextIdx], distance, normal);
        offsetEdges.push_back(offsetEdge);
    }

    // 计算相邻偏置边的交点
    for (size_t i = 0; i < n; ++i) {
        size_t prevIdx = (i == 0) ? n - 1 : i - 1;
        
        gp_Pnt intersection;
        if (computeLineIntersection(
                offsetEdges[prevIdx].first, offsetEdges[prevIdx].second,
                offsetEdges[i].first, offsetEdges[i].second,
                intersection)) {
            offsetPolygon.push_back(intersection);
        } else {
            // 平行时使用端点
            offsetPolygon.push_back(offsetEdges[i].first);
        }
    }

    return offsetPolygon;
}

std::vector<gp_Pnt> WireOffsetter::offsetWithArcJoin(
    const std::vector<gp_Pnt>& polygon,
    double distance,
    const gp_Dir& normal)
{
    // 对于圆弧连接，先计算交点连接，然后在角点处添加圆弧段
    std::vector<gp_Pnt> baseOffset = offsetWithIntersectionJoin(polygon, distance, normal);
    
    // 简化实现：返回基础偏置结果
    // 完整实现需要在每个角点处插入圆弧插值点
    return baseOffset;
}

std::vector<gp_Pnt> WireOffsetter::offsetWithIntersectionJoin(
    const std::vector<gp_Pnt>& polygon,
    double distance,
    const gp_Dir& normal)
{
    return offsetPolygon(polygon, distance, normal);
}

std::vector<gp_Pnt> WireOffsetter::simplifyPolygon(
    const std::vector<gp_Pnt>& points,
    double tolerance)
{
    if (points.size() <= 3) {
        return points;
    }

    std::vector<gp_Pnt> simplified;
    simplified.push_back(points[0]);

    for (size_t i = 1; i < points.size() - 1; ++i) {
        const auto& prev = simplified.back();
        const auto& curr = points[i];
        const auto& next = points[i + 1];

        // 检查三点是否共线
        gp_Vec v1(curr.XYZ() - prev.XYZ());
        gp_Vec v2(next.XYZ() - curr.XYZ());

        v1.Normalize();
        v2.Normalize();

        double dot = v1.Dot(v2);
        
        // 如果接近共线（dot 接近 1），跳过该点
        if (dot > 0.99) {
            continue;
        }

        simplified.push_back(curr);
    }

    simplified.push_back(points.back());

    // 检查首尾是否重复
    if (simplified.size() > 2 &&
        simplified.front().Distance(simplified.back()) < tolerance) {
        simplified.pop_back();
    }

    return simplified;
}

bool WireOffsetter::isPolygonSelfIntersecting(
    const std::vector<gp_Pnt>& points,
    double tolerance)
{
    if (points.size() < 4) {
        return false;
    }

    size_t n = points.size();

    // 检查是否有非相邻边相交
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 2; j < n; ++j) {
            // 跳过相邻边
            if (j == (i + n - 1) % n || j == (i + 1) % n) {
                continue;
            }

            gp_Pnt intersection;
            size_t nextI = (i + 1) % n;
            size_t nextJ = (j + 1) % n;

            if (computeLineIntersection(
                    points[i], points[nextI],
                    points[j], points[nextJ],
                    intersection)) {
                // 检查交点是否在两条线段上
                double distI = points[i].Distance(intersection) +
                               intersection.Distance(points[nextI]);
                double edgeLenI = points[i].Distance(points[nextI]);

                double distJ = points[j].Distance(intersection) +
                               intersection.Distance(points[nextJ]);
                double edgeLenJ = points[j].Distance(points[nextJ]);

                if (std::abs(distI - edgeLenI) < tolerance &&
                    std::abs(distJ - edgeLenJ) < tolerance) {
                    return true;
                }
            }
        }
    }

    return false;
}

} // namespace PathForge
