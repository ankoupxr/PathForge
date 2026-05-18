#include "FeatureData.h"

#include <cmath>

namespace PathForge::Feature {

std::string FeatureData::getTypeName() const {
    switch (type) {
        case FeatureType::Plane: return "平面";
        case FeatureType::Cylinder: return "圆柱";
        case FeatureType::Cone: return "圆锥";
        case FeatureType::Sphere: return "球面";
        case FeatureType::FreeForm: return "自由曲面";
        case FeatureType::Hole: return "孔";
        case FeatureType::Slot: return "槽";
        case FeatureType::Pocket: return "型腔";
        case FeatureType::Boss: return "凸台";
        case FeatureType::Step: return "台阶";
        default: return "未知";
    }
}

std::string FeatureData::getMachiningTypeName() const {
    switch (machiningType) {
        case MachiningType::FaceMilling: return "面铣";
        case MachiningType::ContourMilling: return "轮廓铣";
        case MachiningType::PocketMilling: return "型腔铣";
        case MachiningType::Drilling: return "钻孔";
        case MachiningType::Engraving: return "雕刻";
        default: return "未知";
    }
}

}
