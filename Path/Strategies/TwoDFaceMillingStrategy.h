#pragma once

#include "Strategy.h"
#include "PathStrategy.h"

namespace PathForge {
namespace Path {

class TwoDFaceMillingStrategy : public PathStrategy {
public:
    TwoDFaceMillingStrategy();
    explicit TwoDFaceMillingStrategy(const PathStrategyContext& context);

    StrategyType getType() const override;
    std::string getName() const override;
    std::string getDescription() const override;

    bool validate() const override;

    ToolpathPtr generate() override;

    void setToolRadiusCompensation(bool enabled);
    bool isToolRadiusCompensationEnabled() const;

    void setOverlap(double overlap);
    double getOverlap() const;

    void setMultiplePasses(bool enabled);
    bool isMultiplePassesEnabled() const;

    void setStepDown(double stepDown);
    double getStepDown() const;

private:
    struct Line2D {
        double x1, y1, x2, y2;
    };

    struct Point2D {
        double x, y;
        Point2D() : x(0), y(0) {}
        Point2D(double x_, double y_) : x(x_), y(y_) {}
    };

    struct BoundingBox {
        double minX, maxX, minY, maxY;
    };

    BoundingBox computeBoundingBox(const TopoDS_Wire& wire);
    std::vector<Point2D> extractPoints2D(const TopoDS_Wire& wire);
    
    std::vector<Point2D> generateZigzagLines(const BoundingBox& bbox, 
                                              double stepover, 
                                              double angle);
    std::vector<Point2D> generateOneDirectionLines(const BoundingBox& bbox,
                                                      double stepover,
                                                      double angle,
                                                      bool forward);
    
    std::vector<Point2D> clipLinesToBoundary(const std::vector<Point2D>& lines,
                                               const TopoDS_Wire& boundary);
    
    Point2D lineIntersection(const Point2D& p1, const Point2D& p2,
                               const Point2D& p3, const Point2D& p4);
    
    bool pointInPolygon(const Point2D& p, const std::vector<Point2D>& polygon);
    
    void addLeadInOut(PathPoint& startPoint, PathPoint& endPoint,
                       const Point2D& direction, bool addLeadIn, bool addLeadOut);

    bool m_toolRadiusCompensation = true;
    double m_overlap = 0.0;
    bool m_multiplePasses = false;
    double m_stepDown = 2.0;
};

}
}
