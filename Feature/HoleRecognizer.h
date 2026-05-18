#pragma once

#include "FeatureRecognizer.h"
#include "FeatureData.h"

#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>

#include <vector>
#include <memory>

namespace PathForge::Feature {

class HoleRecognizer : public FeatureRecognizer {
public:
    HoleRecognizer();
    explicit HoleRecognizer(const TopoDS_Shape& shape);

    FeatureType getType() const override { return FeatureType::Hole; }
    std::string getName() const override { return "Hole Recognizer"; }
    std::string getDescription() const override { return "孔特征识别器"; }

    bool recognize(const TopoDS_Shape& shape, FeatureList& features) override;

    void setMinHoleRadius(double radius) { m_minHoleRadius = radius; }
    double getMinHoleRadius() const { return m_minHoleRadius; }

    void setMaxHoleRadius(double radius) { m_maxHoleRadius = radius; }
    double getMaxHoleRadius() const { return m_maxHoleRadius; }

    void setMinHoleDepth(double depth) { m_minHoleDepth = depth; }
    double getMinHoleDepth() const { return m_minHoleDepth; }

    const HoleList& getHoles() const { return m_holes; }

private:
    bool isCylindricalHole(const TopoDS_Face& face) const;
    double calculateHoleRadius(const TopoDS_Face& face) const;
    double calculateHoleDepth(const TopoDS_Shape& shape, const TopoDS_Face& face) const;
    bool isHoleThrough(const TopoDS_Shape& shape, const TopoDS_Face& face) const;

    HoleList m_holes;
    double m_minHoleRadius = 1.0;
    double m_maxHoleRadius = 100.0;
    double m_minHoleDepth = 0.5;
};

}
