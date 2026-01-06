#include "FaceClassifier.h"
#include "MathUtils.h"

#include <BRepAdaptor_Surface.hxx>
#include <GeomLProp_SLProps.hxx>
#include <gp_Vec.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <BRep_Tool.hxx>
#include <TopLoc_Location.hxx>
#include <BRepTools.hxx>

namespace PathForge {
    namespace Geometry {

        GeomAbs_SurfaceType FaceClassifier::classifySurfaceType(const TopoDS_Face& face) const {
            BRepAdaptor_Surface surf(face);
            return surf.GetType();
        }

        gp_Vec FaceClassifier::faceNormal(const TopoDS_Face& face) const {
            // 使用参数域中点 uMid, vMid，计算一阶导数构建法向量
            BRepAdaptor_Surface surf(face);
            Standard_Real u1 = surf.FirstUParameter();
            Standard_Real u2 = surf.LastUParameter();
            Standard_Real v1 = surf.FirstVParameter();
            Standard_Real v2 = surf.LastVParameter();
            Standard_Real u = 0.5 * (u1 + u2);
            Standard_Real v = 0.5 * (v1 + v2);

            GeomLProp_SLProps props(surf.Surface().Surface(), u, v, 1, 1e-6);
            if (!props.IsNormalDefined()) {
                // 退化情况：返回 (0,0,1)
                return gp_Vec(0, 0, 1);
            }
            gp_Vec n = props.Normal();
            n.Normalize();
            return n;
        }

        bool FaceClassifier::areNormalsParallel(const TopoDS_Face& f1, const TopoDS_Face& f2, double tolRad) const {
            gp_Vec n1 = faceNormal(f1);
            gp_Vec n2 = faceNormal(f2);
            double ang = MathUtils::angleBetween(n1, n2);
            return (std::abs(ang) < tolRad) || (std::abs(ang - M_PI) < tolRad);
        }

        bool FaceClassifier::areNormalsPerpendicular(const TopoDS_Face& f1, const TopoDS_Face& f2, double tolRad) const {
            gp_Vec n1 = faceNormal(f1);
            gp_Vec n2 = faceNormal(f2);
            double ang = MathUtils::angleBetween(n1, n2);
            return std::abs(ang - M_PI / 2.0) < tolRad;
        }

    } // namespace Geometry
} // namespace PathForge
