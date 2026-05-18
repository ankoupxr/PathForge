#pragma once

#include "PostProcessor.h"

#include <string>
#include <memory>
#include <map>

namespace PathForge::Post {

class PostProcessorFactory {
public:
    static PostProcessorFactory& instance();

    std::shared_ptr<PostProcessor> create(const std::string& name);
    std::shared_ptr<PostProcessor> createGCode();

    void registerProcessor(const std::string& name, std::shared_ptr<PostProcessor> processor);

    std::vector<std::string> availablePostProcessors() const;

private:
    PostProcessorFactory();
    PostProcessorFactory(const PostProcessorFactory&) = delete;
    PostProcessorFactory& operator=(const PostProcessorFactory&) = delete;

    std::map<std::string, std::shared_ptr<PostProcessor>> m_processors;
};

std::string postProcessToolpath(const Path::Toolpath& toolpath, const NCConfig& config);
std::string postProcessToolpaths(const Path::ToolpathList& toolpaths, const NCConfig& config);

bool saveGCode(const std::string& gcode, const std::string& filename);
std::string loadGCode(const std::string& filename);

}
