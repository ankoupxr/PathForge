#pragma once

#include "FeatureData.h"
#include "FeatureRecognizer.h"
#include "HoleRecognizer.h"
#include "SlotRecognizer.h"
#include "PocketRecognizer.h"

#include <TopoDS_Shape.hxx>

#include <vector>
#include <memory>
#include <map>
#include <string>

namespace PathForge::Feature {

class FeatureManager {
public:
    FeatureManager();
    explicit FeatureManager(const TopoDS_Shape& shape);

    void setShape(const TopoDS_Shape& shape);
    const TopoDS_Shape& getShape() const { return m_shape; }

    bool recognizeAll();
    bool recognizePlanes();
    bool recognizeHoles();
    bool recognizeSlots();
    bool recognizePockets();

    const FeatureList& getFeatures() const { return m_features; }
    FeatureList& getFeatures() { return m_features; }

    FeatureList getFeaturesByType(FeatureType type) const;
    FeatureList getFeaturesByMachiningType(MachiningType type) const;

    HoleList getHoles() const;
    SlotList getSlots() const;
    PocketList getPockets() const;

    int getTotalFeatureCount() const { return static_cast<int>(m_features.size()); }
    int getHoleCount() const;
    int getSlotCount() const;
    int getPocketCount() const;

    void clear();
    bool isEmpty() const { return m_features.empty(); }

    std::map<std::string, int> getFeatureStatistics() const;
    std::string getLastError() const { return m_lastError; }

private:
    void initializeRecognizers();

    TopoDS_Shape m_shape;
    FeatureList m_features;

    PlaneRecognizer m_planeRecognizer;
    CylinderRecognizer m_cylinderRecognizer;
    HoleRecognizer m_holeRecognizer;
    SlotRecognizer m_slotRecognizer;
    PocketRecognizer m_pocketRecognizer;

    std::string m_lastError;
};

}
