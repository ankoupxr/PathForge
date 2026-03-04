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
        double x1, y1, x2, y2, z1, z2;
    };


    struct BoundingBox {
        double minX, maxX, minY, maxY, minZ, maxZ;
    };

    BoundingBox computeBoundingBox(const TopoDS_Wire& wire);
    std::vector<gp_Pnt> extractPoints2D(const TopoDS_Wire& wire);
    
    std::vector<gp_Pnt> generateZigzagLines(const BoundingBox& bbox,
                                              double stepover,
                                              double angle);
    std::vector<gp_Pnt> generateOneDirectionLines(const BoundingBox& bbox,
                                                      double stepover,
                                                      double angle,
                                                      bool forward);


    gp_Pnt lineIntersection(const gp_Pnt& p1, const gp_Pnt& p2,
                               const gp_Pnt& p3, const gp_Pnt& p4);

    bool pointInPolygon(const gp_Pnt& p, const std::vector<gp_Pnt>& polygon);

    void addLeadInOut(PathPoint& startPoint, PathPoint& endPoint,
                       const gp_Dir& direction, bool addLeadIn, bool addLeadOut);

    bool m_toolRadiusCompensation = true;
    double m_overlap = 0.0;
    bool m_multiplePasses = false;
    double m_stepDown = 2.0;

	TopoDS_Wire m_offsetWire;
};

}
}
