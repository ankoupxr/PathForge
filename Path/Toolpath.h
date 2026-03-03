#pragma once

#include <vector>
#include <string>
#include <memory>

#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Edge.hxx>

namespace PathForge::Path {

enum class MoveType {
    Rapid,         // G0
    Linear,        // G1
    ArcCW,         // G2
    ArcCCW,        // G3
    Helix,
    Dwell          // G4
};

enum class MotionType {
    Cutting,
    LeadIn,
    LeadOut,
    Lift,
    Plunge,
    Rapid
};

struct PathPoint {
    gp_Pnt position;
    gp_Vec normal;
    double feedrate = 0.0;
    double spindleSpeed = 0.0;
    int toolIndex = 0;
    MoveType moveType = MoveType::Linear;
    MotionType motionType = MotionType::Cutting;

    PathPoint() = default;
    explicit PathPoint(const gp_Pnt& p, MoveType mt = MoveType::Linear)
        : position(p), moveType(mt) {}
};

class Toolpath {
public:
    Toolpath();
    explicit Toolpath(const std::string& name);

    void addPoint(const PathPoint& point);
    void addPoints(const std::vector<PathPoint>& points);

    const std::vector<PathPoint>& points() const;
    std::vector<PathPoint>& points();

    const std::string& name() const;
    void setName(const std::string& name);

    double totalLength() const;
    double cuttingLength() const;
    double duration() const;

    void setDepth(double depth);
    double depth() const;

    void setStartPoint(const gp_Pnt& p);
    gp_Pnt startPoint() const;

    void setEndPoint(const gp_Pnt& p);
    gp_Pnt endPoint() const;

    void reverse();
    void clear();

    bool isEmpty() const;
    size_t pointCount() const;

    // Edge 鎿嶄綔鏂规硶
    void addEdge(const TopoDS_Edge& edge);
    void addEdges(const std::vector<TopoDS_Edge>& edges);
    const std::vector<TopoDS_Edge>& edges() const;
    std::vector<TopoDS_Edge>& edges();
    void clearEdges();
    bool hasEdges() const;
    size_t edgeCount() const;

private:
    std::string m_name;
    std::vector<PathPoint> m_points;
    std::vector<TopoDS_Edge> m_edges;
    double m_depth = 0.0;
    gp_Pnt m_startPoint;
    gp_Pnt m_endPoint;
};

using ToolpathPtr = std::shared_ptr<Toolpath>;
using ToolpathList = std::vector<ToolpathPtr>;

}
