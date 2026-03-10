// AdaptiveCleaningStrategy.h
#pragma once

#include <vector>
#include <memory>
#include <string>

#include <TopoDS_Wire.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <GeomAbs_JoinType.hxx>

#include "Strategy.h"
#include "Toolpath.h"
#include "Geometry/WireOffsetter.h"

namespace PathForge::Path {

/**
 * @brief 偏移层数据结构
 * @description 存储每一层偏移生成的几何信息和拓扑状态
 */
struct OffsetLayer {
    TopoDS_Wire wire;                      ///< 偏移生成的轮廓线
    std::vector<gp_Pnt> discretizedPoints; ///< 离散化点集
    double offsetDistance = 0.0;           ///< 偏移距离
    int subRegionIndex = 0;                ///< 子区域索引（处理多岛屿现象）
    bool isSplit = false;                  ///< 是否发生分裂
    std::vector<OffsetLayer> subLayers;    ///< 子区域偏移层（递归用）
};

/**
 * @brief 连接段数据结构
 * @description 用于连接相邻偏移环的切入点信息
 */
struct LinkSegment {
    gp_Pnt startPoint;      ///< 起点
    gp_Pnt endPoint;        ///< 终点
    double distance = 0.0;  ///< 欧氏距离
    int fromLayer = 0;      ///< 起始层索引
    int toLayer = 0;        ///< 目标层索引
};

/**
 * @brief 摆线路径参数
 * @description 用于生成摆线模式的加工路径
 */
struct TrochoidalParams {
    double loopRadius = 0.0;       ///< 摆线环半径
    double loopSpacing = 0.0;      ///< 环间距
    double entryAngle = 0.0;       ///< 切入角度
    bool isEnabled = false;        ///< 是否启用摆线模式
};

/**
 * @brief 2D 自适应清洁路径策略
 * @description 基于多级等距偏移与拓扑自适应处理的 CAM 区域填充算法
 * 
 * 核心特性：
 * 1. 多级等距偏移（Multi-level Offsetting）：递归向内收缩生成覆盖路径
 * 2. 拓扑自适应处理：自动识别并处理多岛屿现象
 * 3. 路径连续性：智能连接段生成，实现不抬刀加工
 * 4. 自适应模式切换：根据区域跨度动态切换内螺旋/摆线模式
 */
class AdaptiveCleaningStrategy : public PathStrategy {
public:
    AdaptiveCleaningStrategy();
    explicit AdaptiveCleaningStrategy(const PathStrategyContext& context);
    ~AdaptiveCleaningStrategy() override = default;

    StrategyType getType() const override;
    std::string getName() const override;
    std::string getDescription() const override;

    bool validate() const override;
    ToolpathPtr generate() override;

    // ==================== 偏移参数 ====================
    /**
     * @brief 设置偏移连接类型
     * @param joinType 连接类型：GeomAbs_Arc（圆弧）或 GeomAbs_Intersection（交点）
     */
    void setOffsetJoinType(GeomAbs_JoinType joinType);
    GeomAbs_JoinType offsetJoinType() const;

    /**
     * @brief 设置初始偏移步距
     * @param stepover 步距值（通常为刀具直径的百分比）
     */
    void setInitialStepover(double stepover);
    double initialStepover() const;

    /**
     * @brief 设置最小偏移步距
     * @param minStepover 最小步距值
     */
    void setMinStepover(double minStepover);
    double minStepover() const;

    // ==================== 离散化参数 ====================
    /**
     * @brief 设置离散化弦长
     * @param chordLength 等弦长离散化值
     */
    void setDiscretizationChordLength(double chordLength);
    double discretizationChordLength() const;

    /**
     * @brief 设置离散化点数上限
     * @param maxPoints 每层最大点数
     */
    void setMaxDiscretizationPoints(int maxPoints);
    int maxDiscretizationPoints() const;

    // ==================== 连接段参数 ====================
    /**
     * @brief 启用/禁用连接段优化
     * @param enabled 是否启用
     */
    void setLinkSegmentOptimization(bool enabled);
    bool isLinkSegmentOptimizationEnabled() const;

    /**
     * @brief 设置最大抬刀高度
     * @param height 抬刀高度
     */
    void setMaxLiftHeight(double height);
    double maxLiftHeight() const;

    // ==================== 自适应模式参数 ====================
    /**
     * @brief 设置摆线模式参数
     * @param params 摆线参数
     */
    void setTrochoidalParams(const TrochoidalParams& params);
    const TrochoidalParams& trochoidalParams() const;

    /**
     * @brief 设置内螺旋模式启用条件
     * @param maxSpan 最大跨度阈值（超过此值切换为内螺旋）
     */
    void setSpiralModeMaxSpan(double maxSpan);
    double spiralModeMaxSpan() const;

    /**
     * @brief 设置残余高度阈值
     * @param scallopHeight 允许的残余高度
     */
    void setMaxScallopHeight(double scallopHeight);
    double maxScallopHeight() const;

    // ==================== 递归控制参数 ====================
    /**
     * @brief 设置最大递归深度
     * @param depth 最大递归层数
     */
    void setMaxRecursionDepth(int depth);
    int maxRecursionDepth() const;

    /**
     * @brief 设置最小区域面积阈值
     * @param minArea 小于此面积的区域将被忽略
     */
    void setMinRegionArea(double minArea);
    double minRegionArea() const;

    // ==================== 调试与可视化 ====================
    /**
     * @brief 获取生成的所有偏移层
     * @return 偏移层列表
     */
    const std::vector<OffsetLayer>& getOffsetLayers() const;

    /**
     * @brief 获取所有连接段
     * @return 连接段列表
     */
    const std::vector<LinkSegment>& getLinkSegments() const;

    /**
     * @brief 获取算法执行日志
     * @return 日志信息
     */
    const std::vector<std::string>& getAlgorithmLog() const;

private:
    // ==================== 核心算法方法 ====================
    /**
     * @brief 执行多级等距偏移
     * @param boundaryWire 边界轮廓
     * @return 偏移层列表
     */
    std::vector<OffsetLayer> performMultiLevelOffset(const TopoDS_Wire& boundaryWire);

    /**
     * @brief 递归偏移处理
     * @param currentWire 当前轮廓
     * @param currentOffset 当前偏移量
     * @param depth 当前递归深度
     * @param subRegionIndex 子区域索引
     * @return 偏移层（包含子区域）
     */
    OffsetLayer recursiveOffset(
        const TopoDS_Wire& currentWire,
        double currentOffset,
        int depth,
        int subRegionIndex
    );

    /**
     * @brief 检测并处理多岛屿现象
     * @param offsetShape 偏移生成的形状
     * @param offsetDistance 偏移距离
     * @param depth 递归深度
     * @return 子区域偏移层列表
     */
    std::vector<OffsetLayer> detectAndHandleIslands(
        const TopoDS_Shape& offsetShape,
        double offsetDistance,
        int depth
    );

    // ==================== 几何处理方法 ====================
    /**
     * @brief 将 Wire 转化为 Face（参数化平面）
     * @param wire 输入轮廓
     * @return 生成的面
     */
    TopoDS_Face wireToFace(const TopoDS_Wire& wire);

    /**
     * @brief 使用 GCPnts_QuasiUniformAbscissa 进行等弦长离散化
     * @param wire 输入轮廓
     * @param chordLength 弦长
     * @return 离散化点集
     */
    std::vector<gp_Pnt> discretizeWire(const TopoDS_Wire& wire, double chordLength);

    /**
     * @brief 计算两个轮廓之间的最短切入点
     * @param points1 第一个轮廓点集
     * @param points2 第二个轮廓点集
     * @return 连接段信息
     */
    LinkSegment findShortestLinkSegment(
        const std::vector<gp_Pnt>& points1,
        const std::vector<gp_Pnt>& points2
    );

    /**
     * @brief 检测局部区域跨度
     * @param points 轮廓点集
     * @return 最大跨度值
     */
    double detectRegionSpan(const std::vector<gp_Pnt>& points);

    // ==================== 路径生成方法 ====================
    /**
     * @brief 生成标准偏移路径
     * @param layers 偏移层列表
     * @return 刀路轨迹
     */
    ToolpathPtr generateStandardPath(const std::vector<OffsetLayer>& layers);

    /**
     * @brief 生成内螺旋路径
     * @param boundaryWire 边界轮廓
     * @param depth 加工深度
     * @return 刀路轨迹
     */
    ToolpathPtr generateSpiralPath(const TopoDS_Wire& boundaryWire, double depth);

    /**
     * @brief 生成摆线路径
     * @param boundaryWire 边界轮廓
     * @param depth 加工深度
     * @return 刀路轨迹
     */
    ToolpathPtr generateTrochoidalPath(const TopoDS_Wire& boundaryWire, double depth);

    /**
     * @brief 生成连接段路径
     * @param path 刀路轨迹
     * @param linkSegments 连接段列表
     */
    void addLinkSegmentsToPath(ToolpathPtr path, const std::vector<LinkSegment>& linkSegments);

    /**
     * @brief 检查并处理未覆盖区域（Scallops）
     * @param layers 偏移层列表
     * @param toolDiameter 刀具直径
     * @return 是否需要补充路径
     */
    bool checkAndFillScallops(
        std::vector<OffsetLayer>& layers,
        double toolDiameter
    );

    // ==================== 辅助方法 ====================
    /**
     * @brief 计算点集包围盒
     * @param points 点集
     * @return 包围盒 (minX, maxX, minY, maxY)
     */
    std::tuple<double, double, double, double> computeBoundingBox2D(
        const std::vector<gp_Pnt>& points
    );

    /**
     * @brief 计算点到线段的最短距离
     * @param point 测试点
     * @param lineStart 线段起点
     * @param lineEnd 线段终点
     * @return 最短距离
     */
    double pointToLineDistance(
        const gp_Pnt& point,
        const gp_Pnt& lineStart,
        const gp_Pnt& lineEnd
    );

    /**
     * @brief 记录算法日志
     * @param message 日志信息
     */
    void logMessage(const std::string& message);

    // ==================== 成员变量 ====================
    GeomAbs_JoinType m_offsetJoinType;          ///< 偏移连接类型
    double m_initialStepover;                    ///< 初始偏移步距
    double m_minStepover;                        ///< 最小偏移步距
    double m_chordLength;                        ///< 离散化弦长
    int m_maxDiscretizationPoints;               ///< 最大离散化点数
    bool m_linkSegmentOptimization;              ///< 连接段优化启用
    double m_maxLiftHeight;                      ///< 最大抬刀高度
    TrochoidalParams m_trochoidalParams;         ///< 摆线参数
    double m_spiralModeMaxSpan;                  ///< 内螺旋模式最大跨度
    double m_maxScallopHeight;                   ///< 最大残余高度
    int m_maxRecursionDepth;                     ///< 最大递归深度
    double m_minRegionArea;                      ///< 最小区域面积阈值

    std::vector<OffsetLayer> m_offsetLayers;     ///< 生成的偏移层
    std::vector<LinkSegment> m_linkSegments;     ///< 连接段列表
    std::vector<std::string> m_algorithmLog;     ///< 算法日志
};

using AdaptiveCleaningStrategyPtr = std::shared_ptr<AdaptiveCleaningStrategy>;

} // namespace PathForge::Path
