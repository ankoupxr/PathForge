// CamPathStrategy.h
#pragma once

#include <string>
#include <memory>
#include "Geometry/CAM/CamFace.h"
#include "Geometry/CAM/CamPathPoint.h"
#include "CamMachiningParameters.h"

namespace PathForge {
namespace CAM {

enum class CamStrategyType {
    FaceMilling2D,
    PocketMilling,
    ContourMilling,
    DrillCenter,
    Engrave
};

class CamPathStrategy {
public:
    virtual ~CamPathStrategy() = default;

    virtual CamStrategyType getType() const = 0;
    virtual std::string getName() const = 0;
    virtual std::string getDescription() const = 0;

    virtual bool validate() const = 0;
    virtual CamToolpathPtr generate() = 0;

    void setContext(const CamPathStrategyContext& ctx) { m_context = ctx; }
    const CamPathStrategyContext& getContext() const { return m_context; }

    void setFace(CamFacePtr face) { m_face = face; }
    CamFacePtr getFace() const { return m_face; }

    void setParams(const CamMachiningParameters& params) { m_params = params; }
    const CamMachiningParameters& getParams() const { return m_params; }

    std::string getLastError() const { return m_lastError; }

protected:
    CamPathStrategy() = default;
    explicit CamPathStrategy(CamFacePtr face) : m_face(face) {}

    CamPathStrategyContext m_context;
    CamMachiningParameters m_params;
    CamFacePtr m_face;
    mutable std::string m_lastError;
};

using CamPathStrategyPtr = std::shared_ptr<CamPathStrategy>;

} // namespace CAM
} // namespace PathForge