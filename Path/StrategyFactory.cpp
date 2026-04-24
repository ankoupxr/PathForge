#include "StrategyFactory.h"
#include "Strategies/TwoDFaceMillingStrategy.h"
#include "Strategies/AdaptiveCleaningStrategy.h"
#include "Strategies/PParameterLineStrategy.h"
#include "Strategies/EquidistantSectionPlaneStrategy.h"
#include "Strategies/ProjectionStrategy.h"

namespace PathForge::Path {

PathStrategyPtr PathStrategyFactory::create(StrategyType type)
{
    switch (type) {
    case StrategyType::FaceMilling2D:
        return std::make_shared<TwoDFaceMillingStrategy>();
    case StrategyType::ContourMilling:
        return std::make_shared<AdaptiveCleaningStrategy>();
    case StrategyType::PocketMilling:
    case StrategyType::DrillCenter:
    case StrategyType::Engrave:
    case StrategyType::PParameterLine:
        return std::make_shared<PParameterLineStrategy>();
    case StrategyType::EquidistantSectionPlane:
        return std::make_shared<EquidistantSectionPlaneStrategy>();
    case StrategyType::Projection:
        return std::make_shared<ProjectionStrategy>();
    default:
        return nullptr;
    }
}

PathStrategyPtr PathStrategyFactory::create(const std::string& name)
{
    if (name == "2D Face Milling" || name == "FaceMilling2D") {
        return std::make_shared<TwoDFaceMillingStrategy>();
    }
    if (name == "Adaptive Cleaning" || name == "AdaptiveCleaning") {
        return std::make_shared<AdaptiveCleaningStrategy>();
    }
    // 其他策略类型待实现
    return nullptr;
}

std::string PathStrategyFactory::strategyName(StrategyType type)
{
    switch (type) {
    case StrategyType::FaceMilling2D:
        return "2D Face Milling";
    case StrategyType::PocketMilling:
        return "Pocket Milling";
    case StrategyType::ContourMilling:
        return "Adaptive Cleaning";
    case StrategyType::DrillCenter:
        return "Drill Center";
    case StrategyType::Engrave:
        return "Engrave";
    case StrategyType::PParameterLine:
        return "P Parameter Line";
    case StrategyType::EquidistantSectionPlane:
        return "Equidistant Section Plane";
    case StrategyType::Projection:
        return "Projection";
    default:
        return "Unknown";
    }
}

StrategyType PathStrategyFactory::strategyType(const std::string& name)
{
    if (name == "2D Face Milling" || name == "FaceMilling2D") {
        return StrategyType::FaceMilling2D;
    }
    if (name == "Adaptive Cleaning" || name == "AdaptiveCleaning") {
        return StrategyType::ContourMilling;
    }
    if (name == "P Parameter Line" || name == "PParameterLine") {
        return StrategyType::PParameterLine;
    }
    if (name == "Equidistant Section Plane" || name == "EquidistantSectionPlane") {
        return StrategyType::EquidistantSectionPlane;
    }
    if (name == "Projection" || name == "ProjectionStrategy") {
        return StrategyType::Projection;
    }
    return StrategyType::FaceMilling2D; // 默认
}

} // namespace PathForge::Path
