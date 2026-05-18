#pragma once

#include "FeatureRecognizer.h"
#include "FeatureData.h"

#include <TopoDS_Shape.hxx>

#include <vector>
#include <memory>

namespace PathForge::Feature {

class PocketRecognizer : public FeatureRecognizer {
public:
    PocketRecognizer();
    explicit PocketRecognizer(const TopoDS_Shape& shape);

    FeatureType getType() const override { return FeatureType::Pocket; }
    std::string getName() const override { return "Pocket Recognizer"; }
    std::string getDescription() const override { return "型腔特征识别器"; }

    bool recognize(const TopoDS_Shape& shape, FeatureList& features) override;

    void setMinPocketArea(double area) { m_minPocketArea = area; }
    double getMinPocketArea() const { return m_minPocketArea; }

    void setMinPocketDepth(double depth) { m_minPocketDepth = depth; }
    double getMinPocketDepth() const { return m_minPocketDepth; }

    const PocketList& getPockets() const { return m_pockets; }

private:
    bool analyzePocketGeometry(const std::vector<TopoDS_Face>& faces,
                              double& width, double& height, double& depth);

    PocketList m_pockets;
    double m_minPocketArea = 10.0;
    double m_minPocketDepth = 0.5;
};

}
