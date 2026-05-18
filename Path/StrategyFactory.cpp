#include "StrategyFactory.h"
#include "Strategies/TwoDFaceMillingStrategy.h"
#include "Strategies/AdaptiveCleaningStrategy.h"
#include "Strategies/PParameterLineStrategy.h"
#include "Strategies/EquidistantSectionPlaneStrategy.h"
#include "Strategies/ProjectionStrategy.h"
#include "Strategies/DrillingStrategy.h"
#include "Strategies/PocketMillingStrategy.h"
#include "Strategies/ContourMillingStrategy.h"

namespace PathForge::Path {

PathStrategyPtr PathStrategyFactory::create(StrategyType type)
{
    switch (type) {
    case StrategyType::FaceMilling2D:
        return std::make_shared<TwoDFaceMillingStrategy>();
    case StrategyType::PocketMilling:
        return std::make_shared<PocketMillingStrategy>();
    case StrategyType::ContourMilling:
        return std::make_shared<ContourMillingStrategy>();
    case StrategyType::DrillCenter:
        return std::make_shared<DrillingStrategy>();
    case StrategyType::Engrave:
        return std::make_shared<PParameterLineStrategy>();
    case StrategyType::PParameterLine:
        return std::make_shared<PParameterLineStrategy>();
    case StrategyType::EquidistantSectionPlane:
        return std::make_shared<EquidistantSectionPlaneStrategy>();
    case StrategyType::Projection:
        return std::make_shared<ProjectionStrategy>();
    default:
        return std::make_shared<TwoDFaceMillingStrategy>();
    }
}

PathStrategyPtr PathStrategyFactory::create(const std::string& name)
{
    if (name == "2D Face Milling" || name == "FaceMilling2D") {
        return std::make_shared<TwoDFaceMillingStrategy>();
    }
    if (name == "Pocket Milling" || name == "PocketMilling") {
        return std::make_shared<PocketMillingStrategy>();
    }
    if (name == "Contour Milling" || name == "ContourMilling") {
        return std::make_shared<ContourMillingStrategy>();
    }
    if (name == "Drilling" || name == "DrillCenter" || name == "Drill") {
        return std::make_shared<DrillingStrategy>();
    }
    if (name == "Adaptive Cleaning" || name == "AdaptiveCleaning") {
        return std::make_shared<AdaptiveCleaningStrategy>();
    }
    if (name == "P Parameter Line" || name == "PParameterLine") {
        return std::make_shared<PParameterLineStrategy>();
    }
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
        return "Drilling";
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
    if (name == "Pocket Milling" || name == "PocketMilling") {
        return StrategyType::PocketMilling;
    }
    if (name == "Contour Milling" || name == "ContourMilling") {
        return StrategyType::ContourMilling;
    }
    if (name == "Drilling" || name == "DrillCenter" || name == "Drill") {
        return StrategyType::DrillCenter;
    }
    if (name == "Engrave" || name == "Engraving") {
        return StrategyType::Engrave;
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
    return StrategyType::FaceMilling2D;
}

} // namespace PathForge::Path
