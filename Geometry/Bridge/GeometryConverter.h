// GeometryConverter.h
#pragma once

#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include "CamPoint.h"
#include "CamPoint2D.h"
#include "CamVector.h"
#include "CamDirection.h"

namespace PathForge {
namespace Bridge {

class GeometryConverter {
public:
    static CamPoint toCam(const gp_Pnt& p) {
        return CamPoint(p.X(), p.Y(), p.Z());
    }

    static gp_Pnt toOcc(const CamPoint& p) {
        return gp_Pnt(p.x, p.y, p.z);
    }

    static CamPoint2D toCam(const gp_Pnt2d& p) {
        return CamPoint2D(p.X(), p.Y());
    }

    static gp_Pnt2d toOcc(const CamPoint2D& p) {
        return gp_Pnt2d(p.u, p.v);
    }

    static CamVector toCam(const gp_Vec& v) {
        return CamVector(v.X(), v.Y(), v.Z());
    }

    static gp_Vec toOcc(const CamVector& v) {
        return gp_Vec(v.x, v.y, v.z);
    }

    static CamDirection toCam(const gp_Dir& d) {
        return CamDirection(d.X(), d.Y(), d.Z());
    }

    static gp_Dir toOcc(const CamDirection& d) {
        return gp_Dir(d.x, d.y, d.z);
    }

    static std::vector<CamPoint> toCamPointList(const std::vector<gp_Pnt>& points) {
        std::vector<CamPoint> result;
        result.reserve(points.size());
        for (const auto& p : points) {
            result.push_back(toCam(p));
        }
        return result;
    }

    static std::vector<gp_Pnt> toOccPointList(const std::vector<CamPoint>& points) {
        std::vector<gp_Pnt> result;
        result.reserve(points.size());
        for (const auto& p : points) {
            result.push_back(toOcc(p));
        }
        return result;
    }

    static std::vector<CamPoint2D> toCamPoint2DList(const std::vector<gp_Pnt2d>& points) {
        std::vector<CamPoint2D> result;
        result.reserve(points.size());
        for (const auto& p : points) {
            result.push_back(toCam(p));
        }
        return result;
    }

    static std::vector<gp_Pnt2d> toOccPoint2DList(const std::vector<CamPoint2D>& points) {
        std::vector<gp_Pnt2d> result;
        result.reserve(points.size());
        for (const auto& p : points) {
            result.push_back(toOcc(p));
        }
        return result;
    }
};

} // namespace Bridge
} // namespace PathForge