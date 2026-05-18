#pragma once

#include "MultiAxisConfig.h"
#include "MultiAxisPoint.h"
#include "ToolAxisController.h"

#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>

#include <vector>
#include <memory>

namespace PathForge::MultiAxis {

class Axis3Plus2Positioning {
public:
    Axis3Plus2Positioning();
    ~Axis3Plus2Positioning() = default;

    void setConfiguration(const MultiAxisConfig& config);
    void setToolAxisController(ToolAxisController* controller);

    void addMachiningPlane(const TopoDS_Face& face);
    void clearMachiningPlanes();

    MultiAxisPath generatePositioningPath(const gp_Pnt& start, const gp_Pnt& end);

    void setPositioningFeedrate(double feedrate);
    double getPositioningFeedrate() const { return m_positioningFeedrate; }

    bool isValid() const;
    std::string getLastError() const { return m_lastError; }

private:
    struct MachiningPlane {
        TopoDS_Face face;
        gp_Dir normal;
        gp_Ax1 rotationAxis;
        double angle;

        bool isMachiningPlane() const;
        gp_Pnt getCentroid() const;
    };

    MachiningPlane createMachiningPlane(const TopoDS_Face& face);
    double calculateRotationAngle(const gp_Dir& currentNormal, const gp_Dir& targetNormal);
    gp_Ax1 calculateRotationAxis(const gp_Dir& currentNormal, const gp_Dir& targetNormal);

    MultiAxisPoint createPositioningPoint(const gp_Pnt& position, const gp_Dir& axis,
                                          double aAngle, double bAngle);

    std::vector<MachiningPlane> m_machiningPlanes;
    ToolAxisController* m_toolAxisController = nullptr;
    MultiAxisConfig m_config;

    double m_positioningFeedrate = 1000.0;
    std::string m_lastError;
};

}
