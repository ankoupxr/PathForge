#pragma once

#include <vector>
#include <memory>
#include <functional>

#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gp_Dir.hxx>
#include <TopoDS_Face.hxx>

#include "../Toolpath.h"

namespace PathForge::Path::Core {

enum class DrivePatternType {
    ParallelLines,
    Zigzag,
    Contour,
    Spiral,
    Circular
};

enum class ToolDirection {
    Forward,
    Backward,
    Alternate
};

struct ToolpathOptions {
    double stepover = 1.0;
    double cuttingDepth = 0.0;
    double toolRadius = 0.0;
    double safetyHeight = 10.0;
    double tolerance = 0.01;
    bool useConstantZ = false;
    double constantZ = 0.0;
    bool smoothNormals = true;
    double normalSmoothingFactor = 0.2;
    ToolDirection direction = ToolDirection::Forward;
    bool addLeadIn = false;
    double leadInLength = 5.0;
    bool addLeadOut = false;
    double leadOutLength = 5.0;
};

class SurfaceToolpathCore {
public:
    virtual ~SurfaceToolpathCore() = default;

    virtual ToolpathPtr generateParameterLines(
        const TopoDS_Face& surface,
        const ToolpathOptions& options,
        double uStart, double uEnd,
        double vStart, double vEnd,
        int uDivisions,
        int vDivisions,
        double feedrate = 1000.0) = 0;

    virtual ToolpathPtr generateEquidistantSections(
        const TopoDS_Face& surface,
        const ToolpathOptions& options,
        const gp_Vec& planeNormal,
        double startOffset,
        double endOffset,
        double step,
        double feedrate = 1000.0) = 0;

    virtual ToolpathPtr generateProjection(
        const TopoDS_Face& surface,
        const ToolpathOptions& options,
        const std::vector<gp_Pnt>& drivePoints2D,
        const gp_Pnt& referencePoint,
        double feedrate = 1000.0) = 0;

    virtual ToolpathPtr generateWithDrivePattern(
        const TopoDS_Face& surface,
        const ToolpathOptions& options,
        DrivePatternType patternType,
        const gp_Pnt& boundingMin,
        const gp_Pnt& boundingMax,
        double feedrate = 1000.0) = 0;

    void setSurfaceEvaluator(std::function<gp_Pnt(const TopoDS_Face&, double, double)> evaluator);
    void setNormalEvaluator(std::function<gp_Vec(const TopoDS_Face&, double, double)> evaluator);

protected:
    SurfaceToolpathCore() = default;

    std::function<gp_Pnt(const TopoDS_Face&, double, double)> m_surfaceEvaluator;
    std::function<gp_Vec(const TopoDS_Face&, double, double)> m_normalEvaluator;
};

using SurfaceToolpathCorePtr = std::shared_ptr<SurfaceToolpathCore>;

}