// WireOffsetter.h
#pragma once

#include <TopoDS_Wire.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>
#include <vector>
#include <memory>

namespace PathForge {

/**
 * @brief 偏置层数据结构
 * @description 存储单次偏置操作的结果
 */
struct OffsetResult {
    TopoDS_Wire wire;              ///< 偏置生成的轮廓线
    std::vector<gp_Pnt> points;    ///< 离散化点集
    double offsetDistance = 0.0;   ///< 累计偏置距离
    int iteration = 0;             ///< 迭代次数
    bool isValid = true;           ///< 是否有效
    std::string message;           ///< 状态信息
};

/**
 * @brief 偏置选项配置
 * @description 控制偏置行为的各种参数
 */
struct OffsetOptions {
    // 连接类型
    enum class JoinType {
        Arc,       ///< 圆弧连接
        Intersection, ///< 交点连接
        Round      ///< 圆角连接
    };

    double stepSize = 1.0;         ///< 每次偏置的步长
    double minArea = 1.0;          ///< 最小区域面积阈值
    int maxIterations = 100;       ///< 最大迭代次数
    double tolerance = 1e-6;       ///< 几何公差
    bool keepPoints = true;        ///< 是否保留离散化点集
    int discretizationSegments = 50; ///< 每边离散化段数
    JoinType joinType = JoinType::Arc;
};

/**
 * @brief Wire 向内多次偏置工具
 * @description 基于步长参数进行向内偏置，直到不能偏置为止
 * 
 * 核心特性：
 * 1. 不使用 BRepOffsetAPI_MakeOffset，采用 2D 投影 + 多边形偏置算法
 * 2. 支持递归/迭代多次偏置，直到区域消失
 * 3. 自动检测偏置失败（区域分裂、面积过小等）
 * 4. 可选保留每层偏置的离散化点集
 */
class WireOffsetter {
public:
    WireOffsetter();
    explicit WireOffsetter(const OffsetOptions& options);
    ~WireOffsetter() = default;

    /**
     * @brief 执行多次向内偏置
     * @param wire 输入轮廓
     * @return 所有偏置层结果（包含原始轮廓）
     */
    std::vector<OffsetResult> performMultiOffset(const TopoDS_Wire& wire);

    /**
     * @brief 执行单次向内偏置
     * @param wire 输入轮廓
     * @param distance 偏置距离
     * @return 偏置结果
     */
    OffsetResult performSingleOffset(const TopoDS_Wire& wire, double distance);

    /**
     * @brief 设置偏置选项
     * @param options 偏置选项
     */
    void setOptions(const OffsetOptions& options);

    /**
     * @brief 获取当前选项
     * @return 偏置选项
     */
    const OffsetOptions& getOptions() const;

    /**
     * @brief 获取最后一次操作的日志
     * @return 日志信息列表
     */
    const std::vector<std::string>& getLog() const;

    /**
     * @brief 清除日志
     */
    void clearLog();

    /**
     * @brief 将 Wire 投影到 XY 平面
     * @param wire 输入轮廓
     * @param discretizationSegments 每边离散化段数
     * @param tolerance 几何公差
     * @return 2D 点集（按顺序）
     */
    static std::vector<gp_Pnt> projectWireTo2D(
        const TopoDS_Wire& wire,
        int discretizationSegments = 50,
        double tolerance = 1e-6
    );

    /**
     * @brief 从 2D 点集重建 Wire
     * @param points 2D 点集（闭合）
     * @param zHeight Z 轴高度
     * @return 生成的 Wire
     */
    static TopoDS_Wire createWireFrom2DPoints(const std::vector<gp_Pnt>& points, double zHeight = 0.0);

    /**
     * @brief 计算多边形面积
     * @param points 多边形顶点（按顺序）
     * @return 面积值
     */
    static double computePolygonArea(const std::vector<gp_Pnt>& points);

    /**
     * @brief 计算多边形法向
     * @param points 多边形顶点（按顺序）
     * @return 法向量
     */
    static gp_Dir computePolygonNormal(const std::vector<gp_Pnt>& points);

    /**
     * @brief 检查点是否在多边形内
     * @param point 测试点
     * @param polygon 多边形顶点
     * @return true 如果在内部
     */
    static bool isPointInPolygon(const gp_Pnt& point, const std::vector<gp_Pnt>& polygon);

private:
    /**
     * @brief 计算单条边的向内偏置点
     * @param p1 边起点
     * @param p2 边终点
     * @param offsetDistance 偏置距离
     * @param normal 多边形法向（用于确定向内方向）
     * @return 偏置后的边（两个点）
     */
    std::pair<gp_Pnt, gp_Pnt> offsetEdge(
        const gp_Pnt& p1,
        const gp_Pnt& p2,
        double offsetDistance,
        const gp_Dir& normal
    );

    /**
     * @brief 计算两条线的交点
     * @param p1 第一条线起点
     * @param p2 第一条线终点
     * @param p3 第二条线起点
     * @param p4 第二条线终点
     * @param outPoint 交点输出
     * @return true 如果相交
     */
    static bool computeLineIntersection(
        const gp_Pnt& p1, const gp_Pnt& p2,
        const gp_Pnt& p3, const gp_Pnt& p4,
        gp_Pnt& outPoint
    );

    /**
     * @brief 对多边形进行向内偏置
     * @param polygon 输入多边形
     * @param distance 偏置距离
     * @param normal 法向量
     * @return 偏置后的多边形
     */
    std::vector<gp_Pnt> offsetPolygon(
        const std::vector<gp_Pnt>& polygon,
        double distance,
        const gp_Dir& normal
    );

    /**
     * @brief 使用圆弧连接偏置边
     * @param polygon 输入多边形
     * @param distance 偏置距离
     * @param normal 法向量
     * @return 连接后的多边形
     */
    std::vector<gp_Pnt> offsetWithArcJoin(
        const std::vector<gp_Pnt>& polygon,
        double distance,
        const gp_Dir& normal
    );

    /**
     * @brief 使用交点连接偏置边
     * @param polygon 输入多边形
     * @param distance 偏置距离
     * @param normal 法向量
     * @return 连接后的多边形
     */
    std::vector<gp_Pnt> offsetWithIntersectionJoin(
        const std::vector<gp_Pnt>& polygon,
        double distance,
        const gp_Dir& normal
    );

    /**
     * @brief 简化多边形（移除共线点）
     * @param points 输入点集
     * @param tolerance 公差
     * @return 简化后的点集
     */
    static std::vector<gp_Pnt> simplifyPolygon(
        const std::vector<gp_Pnt>& points,
        double tolerance
    );

    /**
     * @brief 检查多边形是否自相交
     * @param points 多边形顶点
     * @param tolerance 几何公差
     * @return true 如果自相交
     */
    static bool isPolygonSelfIntersecting(
        const std::vector<gp_Pnt>& points,
        double tolerance = 1e-6
    );

    /**
     * @brief 记录日志
     * @param message 日志信息
     */
    void log(const std::string& message);

    OffsetOptions m_options;
    std::vector<std::string> m_log;
};

using WireOffsetterPtr = std::shared_ptr<WireOffsetter>;

} // namespace PathForge
