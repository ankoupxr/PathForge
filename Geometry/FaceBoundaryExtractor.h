// FaceBoundaryExtractor.h
#pragma once

#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <vector>

namespace PathForge {

/**
 * @brief Face boundary extractor
 * @description Extract boundary contour points from TopoDS_Face
 *
 * Used to generate machining boundaries from CAD model faces
 */
class FaceBoundaryExtractor {
public:
    FaceBoundaryExtractor() = default;

    /**
     * @brief Extract outer boundary contour from face
     * @param face Input face
     * @return std::vector<gp_Pnt> Boundary point list (ordered)
     */
    std::vector<gp_Pnt> extractOuterBoundary(const TopoDS_Face& face);

    /**
     * @brief Extract all boundary contours from face
     * @param face Input face
     * @return std::vector<std::vector<gp_Pnt>> All boundary rings
     */
    std::vector<std::vector<gp_Pnt>> extractAllBoundaries(const TopoDS_Face& face) const;

    /**
     * @brief Check if face has holes
     * @param face Input face
     * @return bool true if has holes
     */
    bool hasHoles(const TopoDS_Face& face) const;

    /**
     * @brief Get Z height range of face
     * @param face Input face
     * @return std::pair<double, double> (minZ, maxZ)
     */
    std::pair<double, double> getZRange(const TopoDS_Face& face) const;

    /**
     * @brief Get centroid of face
     * @param face Input face
     * @return gp_Pnt Centroid coordinates
     */
    gp_Pnt getCentroid(const TopoDS_Face& face) const;

    /**
     * @brief Get area of face
     * @param face Input face
     * @return double Area value
     */
    double getArea(const TopoDS_Face& face) const;

    /**
     * @brief Simplify boundary points
     * @param points Original point list
     * @param tolerance Tolerance value
     * @return std::vector<gp_Pnt> Simplified point list
     */
    std::vector<gp_Pnt> simplifyBoundary(
        const std::vector<gp_Pnt>& points,
        double tolerance = 0.01
    ) const;

    /**
     * @brief Check if point is inside face
     * @param point Test point
     * @param boundary Boundary point list
     * @return bool true if inside
     */
    bool isPointInside(const gp_Pnt& point, const std::vector<gp_Pnt>& boundary) const;

    /**
     * @brief Unfold face to 2D parameter space
     * @param face Input face
     * @return std::vector<gp_Pnt2d> 2D points in UV parameter space
     * @description Uses surface UV bounds to generate a grid of 2D points.
     *              For planar faces, this represents the natural parameterization.
     */
    std::vector<gp_Pnt2d> unfoldTo2D(const TopoDS_Face& face) const;

    /**
     * @brief Map 2D UV point to 3D on face surface
     * @param face Target face
     * @param uv 2D point in UV parameter space
     * @return gp_Pnt 3D point on the surface
     * @description Evaluates the surface at given UV coordinates to get 3D point.
     */
    gp_Pnt mapTo3D(const TopoDS_Face& face, const gp_Pnt2d& uv) const;

    /**
     * @brief Get surface UV parameter range
     * @param face Input face
     * @return std::pair<gp_Pnt2d, gp_Pnt2d> (umin, vmin), (umax, vmax)
     * @description Returns the natural parameter bounds of the underlying surface.
     */
    std::pair<gp_Pnt2d, gp_Pnt2d> getUVRange(const TopoDS_Face& face) const;

    /**
     * @brief Map 3D point to 2D UV parameter space
     * @param face Target face
     * @param point 3D point on the surface
     * @return gp_Pnt2d UV coordinates
     * @description Computes the UV parameters corresponding to the 3D point on the surface.
     */
    gp_Pnt2d mapTo2D(const TopoDS_Face& face, const gp_Pnt& point) const;

    /**
     * @brief Offset 2D points by given distance
     * @param points Original 2D points
     * @param offsetDistance Offset distance (positive = outward)
     * @return std::vector<gp_Pnt2d> Offset 2D points
     * @description Offsets points in the UV parameter space.
     */
    std::vector<gp_Pnt2d> offset2D(
        const std::vector<gp_Pnt2d>& points,
        double offsetDistance
    ) const;

private:
    std::vector<gp_Pnt> extractEdgeLoop(const TopoDS_Shape& edge) const;
    bool isOuterBoundary(const TopoDS_Shape& wire) const;
};

} // namespace PathForge
