#pragma once

#include "FeatureData.h"

#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>

#include <vector>
#include <memory>
#include <string>

namespace PathForge::Feature {

class FeatureRecognizer {
public:
    virtual ~FeatureRecognizer() = default;

    virtual FeatureType getType() const = 0;
    virtual std::string getName() const = 0;
    virtual std::string getDescription() const = 0;

    virtual bool recognize(const TopoDS_Shape& shape, FeatureList& features) = 0;
    virtual bool validate() const;

protected:
    FeatureRecognizer() = default;
    explicit FeatureRecognizer(const TopoDS_Shape& shape);

    TopoDS_Shape m_shape;
    std::string m_lastError;
};

class PlaneRecognizer : public FeatureRecognizer {
public:
    PlaneRecognizer();
    explicit PlaneRecognizer(const TopoDS_Shape& shape);

    FeatureType getType() const override { return FeatureType::Plane; }
    std::string getName() const override { return "Plane Recognizer"; }
    std::string getDescription() const override { return "平面识别器"; }

    bool recognize(const TopoDS_Shape& shape, FeatureList& features) override;
};

class CylinderRecognizer : public FeatureRecognizer {
public:
    CylinderRecognizer();
    explicit CylinderRecognizer(const TopoDS_Shape& shape);

    FeatureType getType() const override { return FeatureType::Cylinder; }
    std::string getName() const override { return "Cylinder Recognizer"; }
    std::string getDescription() const override { return "圆柱面识别器"; }

    bool recognize(const TopoDS_Shape& shape, FeatureList& features) override;
};

using FeatureRecognizerPtr = std::shared_ptr<FeatureRecognizer>;

}
