#include "FeatureManager.h"

namespace PathForge::Feature {

FeatureManager::FeatureManager() = default;

FeatureManager::FeatureManager(const TopoDS_Shape& shape) : m_shape(shape) {
    initializeRecognizers();
}

void FeatureManager::setShape(const TopoDS_Shape& shape) {
    m_shape = shape;
    initializeRecognizers();
}

void FeatureManager::initializeRecognizers() {
}

bool FeatureManager::recognizeAll() {
    if (m_shape.IsNull()) {
        m_lastError = "Shape is null";
        return false;
    }

    m_features.clear();

    if (!m_planeRecognizer.recognize(m_shape, m_features)) {
    }

    if (!m_cylinderRecognizer.recognize(m_shape, m_features)) {
    }

    if (!m_holeRecognizer.recognize(m_shape, m_features)) {
    }

    if (!m_slotRecognizer.recognize(m_shape, m_features)) {
    }

    if (!m_pocketRecognizer.recognize(m_shape, m_features)) {
    }

    return !m_features.empty();
}

bool FeatureManager::recognizePlanes() {
    if (m_shape.IsNull()) {
        m_lastError = "Shape is null";
        return false;
    }

    return m_planeRecognizer.recognize(m_shape, m_features);
}

bool FeatureManager::recognizeHoles() {
    if (m_shape.IsNull()) {
        m_lastError = "Shape is null";
        return false;
    }

    return m_holeRecognizer.recognize(m_shape, m_features);
}

bool FeatureManager::recognizeSlots() {
    if (m_shape.IsNull()) {
        m_lastError = "Shape is null";
        return false;
    }

    return m_slotRecognizer.recognize(m_shape, m_features);
}

bool FeatureManager::recognizePockets() {
    if (m_shape.IsNull()) {
        m_lastError = "Shape is null";
        return false;
    }

    return m_pocketRecognizer.recognize(m_shape, m_features);
}

FeatureList FeatureManager::getFeaturesByType(FeatureType type) const {
    FeatureList filtered;
    for (const auto& feature : m_features) {
        if (feature->type == type) {
            filtered.push_back(feature);
        }
    }
    return filtered;
}

FeatureList FeatureManager::getFeaturesByMachiningType(MachiningType type) const {
    FeatureList filtered;
    for (const auto& feature : m_features) {
        if (feature->machiningType == type) {
            filtered.push_back(feature);
        }
    }
    return filtered;
}

HoleList FeatureManager::getHoles() const {
    HoleList holes;
    for (const auto& feature : m_features) {
        if (feature->type == FeatureType::Hole) {
            auto hole = std::dynamic_pointer_cast<HoleData>(feature);
            if (hole) {
                holes.push_back(hole);
            }
        }
    }
    return holes;
}

SlotList FeatureManager::getSlots() const {
    SlotList slots;
    for (const auto& feature : m_features) {
        if (feature->type == FeatureType::Slot) {
            auto slot = std::dynamic_pointer_cast<SlotData>(feature);
            if (slot) {
                slots.push_back(slot);
            }
        }
    }
    return slots;
}

PocketList FeatureManager::getPockets() const {
    PocketList pockets;
    for (const auto& feature : m_features) {
        if (feature->type == FeatureType::Pocket) {
            auto pocket = std::dynamic_pointer_cast<PocketData>(feature);
            if (pocket) {
                pockets.push_back(pocket);
            }
        }
    }
    return pockets;
}

int FeatureManager::getHoleCount() const {
    return static_cast<int>(getHoles().size());
}

int FeatureManager::getSlotCount() const {
    return static_cast<int>(getSlots().size());
}

int FeatureManager::getPocketCount() const {
    return static_cast<int>(getPockets().size());
}

void FeatureManager::clear() {
    m_features.clear();
    m_lastError.clear();
}

std::map<std::string, int> FeatureManager::getFeatureStatistics() const {
    std::map<std::string, int> stats;

    stats["平面"] = 0;
    stats["圆柱"] = 0;
    stats["孔"] = 0;
    stats["槽"] = 0;
    stats["型腔"] = 0;
    stats["其他"] = 0;

    for (const auto& feature : m_features) {
        switch (feature->type) {
            case FeatureType::Plane:
            case FeatureType::Cylinder:
                stats["平面"]++;
                break;
            case FeatureType::Hole:
                stats["孔"]++;
                break;
            case FeatureType::Slot:
                stats["槽"]++;
                break;
            case FeatureType::Pocket:
                stats["型腔"]++;
                break;
            default:
                stats["其他"]++;
                break;
        }
    }

    return stats;
}

}
