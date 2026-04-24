#include "SurfaceToolpathCore.h"

#include <cmath>
#include <algorithm>

#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gp_Dir.hxx>
#include <gp_Trsf.hxx>
#include <gp_Mat.hxx>
#include <TopoDS_Face.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomAPI_IntSS.hxx>
#include <gp_Pln.hxx>
#include <Geom_Curve.hxx>

#include "DrivePatternGenerator.h"

namespace PathForge::Path::Core {

void SurfaceToolpathCore::setSurfaceEvaluator(
    std::function<gp_Pnt(const TopoDS_Face&, double, double)> evaluator)
{
    m_surfaceEvaluator = evaluator;
}

void SurfaceToolpathCore::setNormalEvaluator(
    std::function<gp_Vec(const TopoDS_Face&, double, double)> evaluator)
{
    m_normalEvaluator = evaluator;
}

class SurfaceToolpathCoreImpl : public SurfaceToolpathCore {
public:
    SurfaceToolpathCoreImpl() = default;
    ~SurfaceToolpathCoreImpl() override = default;

    ToolpathPtr generateParameterLines(
        const TopoDS_Face& surface,
        const ToolpathOptions& options,
        double uStart, double uEnd,
        double vStart, double vEnd,
        int uDivisions,
        int vDivisions,
        double feedrate = 1000.0) override;

    ToolpathPtr generateEquidistantSections(
        const TopoDS_Face& surface,
        const ToolpathOptions& options,
        const gp_Vec& planeNormal,
        double startOffset,
        double endOffset,
        double step,
        double feedrate = 1000.0) override;

    ToolpathPtr generateProjection(
        const TopoDS_Face& surface,
        const ToolpathOptions& options,
        const std::vector<gp_Pnt>& drivePoints2D,
        const gp_Pnt& referencePoint,
        double feedrate = 1000.0) override;

    ToolpathPtr generateWithDrivePattern(
        const TopoDS_Face& surface,
        const ToolpathOptions& options,
        DrivePatternType patternType,
        const gp_Pnt& boundingMin,
        const gp_Pnt& boundingMax,
        double feedrate = 1000.0) override;

private:
    gp_Pnt evaluateSurface(const TopoDS_Face& face, double u, double v);
    gp_Vec evaluateNormal(const TopoDS_Face& face, double u, double v);
    std::vector<gp_Pnt> generateDrivePoints2D(DrivePatternType patternType, const gp_Pnt& boundingMin, const gp_Pnt& boundingMax, double step);
};

gp_Pnt SurfaceToolpathCoreImpl::evaluateSurface(const TopoDS_Face& face, double u, double v)
{
    if (m_surfaceEvaluator) {
        return m_surfaceEvaluator(face, u, v);
    }
    BRepAdaptor_Surface adaptor(face);
    return adaptor.Value(u, v);
}

gp_Vec SurfaceToolpathCoreImpl::evaluateNormal(const TopoDS_Face& face, double u, double v)
{
    if (m_normalEvaluator) {
        return m_normalEvaluator(face, u, v);
    }
    BRepAdaptor_Surface adaptor(face);
    gp_Vec du = adaptor.D1U(u, v);
    gp_Vec dv = adaptor.D1V(u, v);
    gp_Vec normal = du.Crossed(dv);
    if (normal.Magnitude() > 1e-10) {
        normal.Normalize();
    }
    return normal;
}

std::vector<gp_Pnt> SurfaceToolpathCoreImpl::generateDrivePoints2D(
    DrivePatternType patternType,
    const gp_Pnt& boundingMin,
    const gp_Pnt& boundingMax,
    double step)
{
    std::vector<gp_Pnt> points;
    DrivePatternGenerator generator;
    generator.setPatternType(patternType);
    generator.setBoundingBox(boundingMin.X(), boundingMax.X(), boundingMin.Y(), boundingMax.Y());
    generator.setStep(step);
    auto drivePoints = generator.generate();
    for (const auto& dp : drivePoints) {
        points.push_back(dp.position);
    }
    return points;
}

ToolpathPtr SurfaceToolpathCoreImpl::generateParameterLines(
    const TopoDS_Face& surface,
    const ToolpathOptions& options,
    double uStart, double uEnd,
    double vStart, double vEnd,
    int uDivisions,
    int vDivisions,
    double feedrate)
{
    auto path = std::make_shared<Toolpath>("ParameterLineToolpath");
    
    if (uDivisions <= 0 || vDivisions <= 0) {
        return path;
    }

    bool reverse = (options.direction == ToolDirection::Backward);

    for (int i = 0; i <= uDivisions; ++i) {
        double u = uStart + (uEnd - uStart) * i / uDivisions;
        
        std::vector<gp_Pnt> linePoints;
        int vStartIdx = reverse ? vDivisions : 0;
        int vEndIdx = reverse ? 0 : vDivisions;
        int vStep = reverse ? -1 : 1;

        for (int j = vStartIdx; reverse ? j >= vEndIdx : j <= vEndIdx; j += vStep) {
            double v = vStart + (vEnd - vStart) * j / vDivisions;
            gp_Pnt point = evaluateSurface(surface, u, v);
            if (options.useConstantZ) {
                point.SetZ(options.constantZ);
            }
            linePoints.push_back(point);
        }

        for (size_t k = 0; k < linePoints.size(); ++k) {
            double v = vStart + (vEnd - vStart) * k / vDivisions;
            PathPoint pt(linePoints[k], MoveType::Linear);
            pt.feedrate = feedrate;
            pt.normal = evaluateNormal(surface, u, v);

            if (options.addLeadIn && i == 0 && k == 0) {
                pt.motionType = MotionType::LeadIn;
            } else if (options.addLeadOut && i == uDivisions && k == linePoints.size() - 1) {
                pt.motionType = MotionType::LeadOut;
            } else if (k == 0) {
                pt.motionType = MotionType::Plunge;
                pt.feedrate = feedrate * 0.5;
            } else {
                pt.motionType = MotionType::Cutting;
            }
            path->addPoint(pt);
        }
    }

    if (options.direction == ToolDirection::Alternate) {
        for (int i = 0; i <= uDivisions; i += 2) {
            double u = uStart + (uEnd - uStart) * i / uDivisions;
            bool reverseLine = (i % 2 == 1);

            for (int j = 0; j <= vDivisions; ++j) {
                double v = vStart + (vEnd - vStart) * j / vDivisions;
                gp_Pnt point = evaluateSurface(surface, u, v);
                
                PathPoint pt(point, MoveType::Linear);
                pt.feedrate = feedrate;
                pt.normal = evaluateNormal(surface, u, v);
                pt.motionType = (j == 0) ? MotionType::Lift : MotionType::Cutting;
                path->addPoint(pt);
            }
        }
    }

    return path;
}

ToolpathPtr SurfaceToolpathCoreImpl::generateEquidistantSections(
    const TopoDS_Face& surface,
    const ToolpathOptions& options,
    const gp_Vec& planeNormal,
    double startOffset,
    double endOffset,
    double step,
    double feedrate)
{
    auto path = std::make_shared<Toolpath>("EquidistantSectionToolpath");
    
    if (step <= 0) {
        return path;
    }

    BRepAdaptor_Surface adaptor(surface);
    gp_Pnt surfaceCenter = adaptor.Value(0.5, 0.5);
    gp_Dir normalDir(planeNormal);
    double currentOffset = startOffset;
    bool firstSection = true;

    while (currentOffset <= endOffset + 0.001) {
        gp_Pnt planeOrigin = surfaceCenter.XYZ() + planeNormal.XYZ() * currentOffset;
        gp_Pln plane(planeOrigin, normalDir);

        GeomAPI_IntSS intersection(surface, plane, 1.0e-6);

        if (intersection.IsDone() && intersection.NbLines() > 0) {
            for (int i = 1; i <= intersection.NbLines(); ++i) {
                Handle(Geom_Curve) curve = intersection.Line(i);
                if (curve.IsNull()) continue;

                double u1, u2;
                curve->Bounds(u1, u2);
                int numSamples = 50;
                bool firstPoint = true;

                for (int j = 0; j <= numSamples; ++j) {
                    double t = u1 + (u2 - u1) * j / numSamples;
                    gp_Pnt point = curve->Value(t);

                    if (options.useConstantZ) {
                        point.SetZ(options.constantZ);
                    }

                    PathPoint pt(point, MoveType::Linear);
                    pt.feedrate = feedrate;
                    pt.normal = firstSection ? planeNormal : planeNormal.Reversed();

                    if (firstSection && firstPoint) {
                        pt.motionType = MotionType::Plunge;
                        pt.feedrate = feedrate * 0.5;
                    } else if (!firstPoint) {
                        pt.motionType = MotionType::Cutting;
                    } else {
                        pt.motionType = MotionType::Lift;
                        pt.feedrate = feedrate * 1.5;
                    }

                    path->addPoint(pt);
                    firstPoint = false;
                }
            }
        }

        currentOffset += step;
        firstSection = false;
    }

    return path;
}

ToolpathPtr SurfaceToolpathCoreImpl::generateProjection(
    const TopoDS_Face& surface,
    const ToolpathOptions& options,
    const std::vector<gp_Pnt>& drivePoints2D,
    const gp_Pnt& referencePoint,
    double feedrate)
{
    auto path = std::make_shared<Toolpath>("ProjectionToolpath");
    
    if (drivePoints2D.empty()) {
        return path;
    }

    gp_Vec lastNormal;
    bool hasLastNormal = false;

    for (size_t idx = 0; idx < drivePoints2D.size(); ++idx) {
        const auto& point2D = drivePoints2D[idx];
        gp_Pnt point3D(point2D.X(), point2D.Y(), referencePoint.Z());

        GeomAPI_ProjectPointOnSurf projector(point3D, surface);

        if (projector.NbPoints() > 0) {
            gp_Pnt projectedPoint = projector.NearestPoint();

            if (options.useConstantZ) {
                projectedPoint.SetZ(options.constantZ);
            }

            double u = std::clamp((projectedPoint.X() - referencePoint.X()) / 10.0, 0.0, 1.0);
            double v = std::clamp((projectedPoint.Y() - referencePoint.Y()) / 10.0, 0.0, 1.0);
            gp_Vec normal = evaluateNormal(surface, u, v);

            if (options.smoothNormals && hasLastNormal) {
                normal = gp_Vec(
                    lastNormal.X() * (1 - options.normalSmoothingFactor) + normal.X() * options.normalSmoothingFactor,
                    lastNormal.Y() * (1 - options.normalSmoothingFactor) + normal.Y() * options.normalSmoothingFactor,
                    lastNormal.Z() * (1 - options.normalSmoothingFactor) + normal.Z() * options.normalSmoothingFactor
                );
            }

            PathPoint pt(projectedPoint, MoveType::Linear);
            pt.feedrate = feedrate;
            pt.normal = normal;
            lastNormal = normal;
            hasLastNormal = true;

            if (options.addLeadIn && idx == 0) {
                pt.motionType = MotionType::LeadIn;
            } else if (options.addLeadOut && idx == drivePoints2D.size() - 1) {
                pt.motionType = MotionType::LeadOut;
            } else if (idx == 0) {
                pt.motionType = MotionType::Plunge;
                pt.feedrate = feedrate * 0.5;
            } else {
                pt.motionType = MotionType::Cutting;
            }

            path->addPoint(pt);
        }
    }

    return path;
}

ToolpathPtr SurfaceToolpathCoreImpl::generateWithDrivePattern(
    const TopoDS_Face& surface,
    const ToolpathOptions& options,
    DrivePatternType patternType,
    const gp_Pnt& boundingMin,
    const gp_Pnt& boundingMax,
    double feedrate)
{
    auto path = std::make_shared<Toolpath>("DrivePatternToolpath");

    auto drivePoints2D = generateDrivePoints2D(patternType, boundingMin, boundingMax, options.stepover);

    if (drivePoints2D.empty()) {
        return path;
    }

    gp_Pnt referencePoint(
        (boundingMin.X() + boundingMax.X()) / 2.0,
        (boundingMin.Y() + boundingMax.Y()) / 2.0,
        0.0);

    return generateProjection(surface, options, drivePoints2D, referencePoint, feedrate);
}

SurfaceToolpathCorePtr createSurfaceToolpathCore()
{
    return std::make_shared<SurfaceToolpathCoreImpl>();
}

}