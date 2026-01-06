#pragma once

#include <TopoDS_Face.hxx>
#include <GeomAbs_Shape.hxx>
#include <GeomAbs_SurfaceType.hxx>

namespace PathForge {
    namespace Geometry {

        /**
         * FaceClassifier:
         *  - 判断面类型（Plane / Cylinder / Cone / BSpline / ...)
         *  - 计算面在参数域中心处的法向量
         *  - 判断两个面的法向量是否平行 / 垂直
         */
        class FaceClassifier {
        public:
            FaceClassifier() = default;
            ~FaceClassifier() = default;

            /// 获取 GeomAbs_SurfaceType
            GeomAbs_SurfaceType classifySurfaceType(const TopoDS_Face& face) const;

            /// 计算 face 在参数域中点处的法向量（单位向量）
            gp_Vec faceNormal(const TopoDS_Face& face) const;

            /// 判断两个面的法向量是否近似平行（tolerance: rad）
            bool areNormalsParallel(const TopoDS_Face& f1, const TopoDS_Face& f2, double tolRad = 1e-3) const;

            /// 判断两个面的法向量是否近似垂直
            bool areNormalsPerpendicular(const TopoDS_Face& f1, const TopoDS_Face& f2, double tolRad = 1e-3) const;
        };

    } // namespace Geometry
} // namespace PathForge
