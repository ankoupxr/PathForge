#pragma once

#include "NCConfig.h"

#include <PathForge/Path/Toolpath.h>

#include <string>
#include <vector>
#include <memory>

namespace PathForge::Post {

class PostProcessor {
public:
    virtual ~PostProcessor() = default;

    virtual std::string name() const = 0;
    virtual std::string description() const = 0;

    virtual std::string generate(const Path::Toolpath& toolpath, const NCConfig& config) = 0;
    virtual std::string generate(const Path::ToolpathList& toolpaths, const NCConfig& config);

    virtual bool validate(const Path::Toolpath& toolpath) const;
    virtual std::vector<std::string> getValidationErrors(const Path::Toolpath& toolpath) const;

protected:
    PostProcessor() = default;
    explicit PostProcessor(const NCConfig& config);

    NCConfig m_config;
};

class GCodePostProcessor : public PostProcessor {
public:
    GCodePostProcessor();
    explicit GCodePostProcessor(const NCConfig& config);

    std::string name() const override { return "Generic G-Code"; }
    std::string description() const override { return "Generic G-code post processor"; }

    std::string generate(const Path::Toolpath& toolpath, const NCConfig& config) override;

private:
    std::string generateHeader(const Path::Toolpath& toolpath, const NCConfig& config);
    std::string generateFooter(const NCConfig& config);
    std::string generateToolpath(const Path::Toolpath& toolpath, const NCConfig& config);

    std::string formatLinearMove(const Path::PathPoint& point, const NCConfig& config);
    std::string formatRapidMove(const Path::PathPoint& point, const NCConfig& config);
    std::string formatArcMove(const Path::PathPoint& from, const Path::PathPoint& to,
                               const NCConfig& config);
    std::string formatHelix(const Path::PathPoint& from, const Path::PathPoint& to,
                             const NCConfig& config);

    std::string formatCoordinate(double x, double y, double z, const NCConfig& config);
    std::string formatFeedrate(double feedrate, const NCConfig& config);
    std::string formatSpindleSpeed(int rpm, const NCConfig& config);

    void addComment(const std::string& comment);
    void addLine(const std::string& line);

    int m_lineNumber = 0;
    std::vector<std::string> m_outputLines;
    Path::PathPoint m_lastPoint;
    bool m_firstPoint = true;
};

}
