#pragma once

#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>

namespace PathForge {
namespace Geometry {

/**
 * ShapeUtils:
 *  - 面积 / 面心（质心）计算
 *  - 面边界个数
 */
class ShapeUtils {
public:
    /// 计算 face 的面积
    static double faceArea(const TopoDS_Face& face);

    /// 计算 face 的质心（基于表面积积分）
    static gp_Pnt faceCentroid(const TopoDS_Face& face);

    /// 统计 face 边界的边数（外边界 + 内孔）
    static int faceEdgeCount(const TopoDS_Face& face);
};

} // namespace Geometry
} // namespace PathForge
