#include "StrategyFactory.h"
#include "Strategies/TwoDFaceMillingStrategy.h"

namespace PathForge::Path {

PathStrategyPtr PathStrategyFactory::create(StrategyType type)
{
    switch (type) {
    case StrategyType::FaceMilling2D:
        return std::make_shared<TwoDFaceMillingStrategy>();
    case StrategyType::PocketMilling:
    case StrategyType::ContourMilling:
    case StrategyType::DrillCenter:
    case StrategyType::Engrave:
        // 其他策略类型待实现
        return nullptr;
    default:
        return nullptr;
    }
}

PathStrategyPtr PathStrategyFactory::create(const std::string& name)
{
    if (name == "2D Face Milling" || name == "FaceMilling2D") {
        return std::make_shared<TwoDFaceMillingStrategy>();
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
        return "Contour Milling";
    case StrategyType::DrillCenter:
        return "Drill Center";
    case StrategyType::Engrave:
        return "Engrave";
    default:
        return "Unknown";
    }
}

StrategyType PathStrategyFactory::strategyType(const std::string& name)
{
    if (name == "2D Face Milling" || name == "FaceMilling2D") {
        return StrategyType::FaceMilling2D;
    }
    return StrategyType::FaceMilling2D; // 默认
}

} // namespace PathForge::Path
