#pragma once

#include "FeatureRecognizer.h"
#include "FeatureData.h"

#include <TopoDS_Shape.hxx>

#include <vector>
#include <memory>

namespace PathForge::Feature {

class SlotRecognizer : public FeatureRecognizer {
public:
    SlotRecognizer();
    explicit SlotRecognizer(const TopoDS_Shape& shape);

    FeatureType getType() const override { return FeatureType::Slot; }
    std::string getName() const override { return "Slot Recognizer"; }
    std::string getDescription() const override { return "槽特征识别器"; }

    bool recognize(const TopoDS_Shape& shape, FeatureList& features) override;

    void setMinSlotWidth(double width) { m_minSlotWidth = width; }
    double getMinSlotWidth() const { return m_minSlotWidth; }

    void setMinSlotDepth(double depth) { m_minSlotDepth = depth; }
    double getMinSlotDepth() const { return m_minSlotDepth; }

    const SlotList& getSlots() const { return m_slots; }

private:
    bool detectSlotGeometry(const std::vector<TopoDS_Face>& faces,
                          double& width, double& depth, double& length);

    SlotList m_slots;
    double m_minSlotWidth = 1.0;
    double m_minSlotDepth = 0.5;
};

}
