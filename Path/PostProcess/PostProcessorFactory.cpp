#include "PostProcessorFactory.h"

#include <fstream>
#include <sstream>

namespace PathForge::Post {

PostProcessorFactory& PostProcessorFactory::instance() {
    static PostProcessorFactory factory;
    return factory;
}

PostProcessorFactory::PostProcessorFactory() {
    m_processors["generic"] = std::make_shared<GCodePostProcessor>();
    m_processors["gcode"] = std::make_shared<GCodePostProcessor>();
}

std::shared_ptr<PostProcessor> PostProcessorFactory::create(const std::string& name) {
    auto it = m_processors.find(name);
    if (it != m_processors.end()) {
        return it->second;
    }
    return nullptr;
}

std::shared_ptr<PostProcessor> PostProcessorFactory::createGCode() {
    return create("generic");
}

void PostProcessorFactory::registerProcessor(const std::string& name, std::shared_ptr<PostProcessor> processor) {
    m_processors[name] = processor;
}

std::vector<std::string> PostProcessorFactory::availablePostProcessors() const {
    std::vector<std::string> names;
    for (const auto& pair : m_processors) {
        names.push_back(pair.first);
    }
    return names;
}

std::string postProcessToolpath(const Path::Toolpath& toolpath, const NCConfig& config) {
    auto processor = PostProcessorFactory::instance().createGCode();
    return processor->generate(toolpath, config);
}

std::string postProcessToolpaths(const Path::ToolpathList& toolpaths, const NCConfig& config) {
    auto processor = PostProcessorFactory::instance().createGCode();
    return processor->generate(toolpaths, config);
}

bool saveGCode(const std::string& gcode, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    file << gcode;
    file.close();
    return true;
}

std::string loadGCode(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        return "";
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

}
