// FaceBoundaryExtractor.h
#pragma once

#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>
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

private:
    std::vector<gp_Pnt> extractEdgeLoop(const TopoDS_Shape& edge) const;
    bool isOuterBoundary(const TopoDS_Shape& wire) const;
};

} // namespace PathForge
