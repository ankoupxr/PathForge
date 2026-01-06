#pragma once

#include <gp_Vec.hxx>
#include <gp_Pnt.hxx>

namespace PathForge {
    namespace Geometry {

        struct MathUtils {
            /// 计算两个向量的夹角（弧度）
            static double angleBetween(const gp_Vec& a, const gp_Vec& b);

            /// 计算点到点的距离
            static double distance(const gp_Pnt& a, const gp_Pnt& b);

            /// 判断两个向量是否近似平行（使用 cos 相似度）
            static bool isParallel(const gp_Vec& a, const gp_Vec& b, double tol = 1e-6);

            /// 判断两个向量是否近似垂直
            static bool isPerpendicular(const gp_Vec& a, const gp_Vec& b, double tol = 1e-6);
        };

    } // namespace Geometry
} // namespace PathForge
