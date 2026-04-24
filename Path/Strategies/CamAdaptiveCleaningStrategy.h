// CamAdaptiveCleaningStrategy.h
#pragma once

#include "CamPathStrategy.h"
#include "CamWire.h"
#include <vector>
#include <string>

namespace PathForge {
namespace CAM {

struct CamOffsetLayer {
    CamWire wire;
    std::vector<CamPoint> discretizedPoints;
    double offsetDistance = 0.0;
    bool isSplit = false;
    std::vector<CamOffsetLayer> subLayers;
};

struct CamLinkSegment {
    CamPoint startPoint;
    CamPoint endPoint;
    double distance = 0.0;
};

struct CamTrochoidalParams {
    double loopRadius = 0.0;
    double loopSpacing = 0.0;
    bool isEnabled = false;
};

class CamAdaptiveCleaningStrategy : public CamPathStrategy {
public:
    CamAdaptiveCleaningStrategy();
    explicit CamAdaptiveCleaningStrategy(CamFacePtr face);

    CamStrategyType getType() const override;
    std::string getName() const override;
    std::string getDescription() const override;

    bool validate() const override;
    CamToolpathPtr generate() override;

    void setStepover(double s) { m_initialStepover = s; }
    double getStepover() const { return m_initialStepover; }

    void setMinStepover(double s) { m_minStepover = s; }
    double getMinSteopper() const { return m_minStepover; }

    void setMaxRecursionDepth(int d) { m_maxRecursionDepth = d; }
    int getMaxRecursionDepth() const { return m_maxRecursionDepth; }

    void setChordLength(double l) { m_chordLength = l; }
    double getChordLength() const { return m_chordLength; }

    const std::vector<CamOffsetLayer>& getOffsetLayers() const { return m_offsetLayers; }

private:
    std::vector<CamOffsetLayer> performOffset(const CamWire& boundary);
    CamOffsetLayer recursiveOffset(const CamWire& wire, double offset, int depth);

    std::vector<CamPoint> discretizeWire(const CamWire& wire, double chordLength);
    CamLinkSegment findShortestLink(const std::vector<CamPoint>& pts1,
                                     const std::vector<CamPoint>& pts2);

    std::tuple<double, double, double, double> computeBBox2D(const std::vector<CamPoint>& points);
    CamPoint wireCentroid(const CamWire& wire);
    double wireArea(const CamWire& wire);

    CamToolpathPtr generateSpiralPath(const CamWire& boundary, double depth);
    void addLinkSegments(CamToolpathPtr path, const std::vector<CamLinkSegment>& links);

    double m_initialStepover = 5.0;
    double m_minStepover = 1.0;
    double m_chordLength = 0.5;
    int m_maxRecursionDepth = 10;
    double m_toolRadius = 5.0;

    std::vector<CamOffsetLayer> m_offsetLayers;
    std::vector<CamLinkSegment> m_linkSegments;
};

} // namespace CAM
} // namespace PathForge