#pragma once

#include <memory>
#include "Strategy.h"

namespace PathForge::Path {

class PathStrategyFactory {
public:
    static PathStrategyPtr create(StrategyType type);
    static PathStrategyPtr create(const std::string& name);

    static std::string strategyName(StrategyType type);
    static StrategyType strategyType(const std::string& name);
};

} // namespace PathForge::Path
