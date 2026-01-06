// FeatureType.h
#pragma once

#include <string>
#include <unordered_map>

namespace PathForge::Feature {

    // ============================================================
    // FeatureType
    // ============================================================

    enum class FeatureType : int
    {
        // ---------- 基础几何 ----------
        PlanarFace = 0,
        Cylinder,
        Cone,

        // ---------- 加工特征 ----------
        Hole,
        Pocket,
        Slot,

        // ---------- 边角特征 ----------
        Chamfer,
        Fillet,

        // ---------- 其他 ----------
        Unknown
    };

    // ============================================================
    // FeatureType -> string
    // ============================================================

    inline std::string FeatureTypeToString(FeatureType type)
    {
        switch (type) {
        case FeatureType::PlanarFace: return "PlanarFace";
        case FeatureType::Cylinder:   return "Cylinder";
        case FeatureType::Cone:       return "Cone";
        case FeatureType::Hole:       return "Hole";
        case FeatureType::Pocket:     return "Pocket";
        case FeatureType::Slot:       return "Slot";
        case FeatureType::Chamfer:    return "Chamfer";
        case FeatureType::Fillet:     return "Fillet";
        default:                      return "Unknown";
        }
    }

    // ============================================================
    // string -> FeatureType
    // ============================================================

    inline FeatureType FeatureTypeFromString(const std::string& name)
    {
        static const std::unordered_map<std::string, FeatureType> map = {
            {"PlanarFace", FeatureType::PlanarFace},
            {"Cylinder",   FeatureType::Cylinder},
            {"Cone",       FeatureType::Cone},
            {"Hole",       FeatureType::Hole},
            {"Pocket",     FeatureType::Pocket},
            {"Slot",       FeatureType::Slot},
            {"Chamfer",    FeatureType::Chamfer},
            {"Fillet",     FeatureType::Fillet}
        };

        auto it = map.find(name);
        if (it != map.end())
            return it->second;

        return FeatureType::Unknown;
    }

    // ============================================================
    // 分类判断（非常重要，规则系统会大量用）
    // ============================================================

    inline bool IsBaseSurface(FeatureType type)
    {
        return type == FeatureType::PlanarFace ||
            type == FeatureType::Cylinder ||
            type == FeatureType::Cone;
    }

    inline bool IsMachiningFeature(FeatureType type)
    {
        return type == FeatureType::Hole ||
            type == FeatureType::Pocket ||
            type == FeatureType::Slot;
    }

    inline bool IsEdgeFeature(FeatureType type)
    {
        return type == FeatureType::Chamfer ||
            type == FeatureType::Fillet;
    }

    // ============================================================
    // 可选：FeatureType 优先级（规则冲突时用）
    // ============================================================

    inline int FeaturePriority(FeatureType type)
    {
        switch (type) {
        case FeatureType::Hole:     return 100;
        case FeatureType::Pocket:  return 90;
        case FeatureType::Slot:    return 80;
        case FeatureType::Chamfer: return 70;
        case FeatureType::Fillet:  return 60;

        case FeatureType::Cylinder:return 20;
        case FeatureType::Cone:    return 15;
        case FeatureType::PlanarFace:
            return 10;
        default:
            return 0;
        }
    }

} // namespace PathForge::Feature
