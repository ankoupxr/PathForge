#pragma once

#include <string>

namespace PathForge::Post {

struct NCConfig {
    std::string programName = "PathForge";
    std::string programNumber = "O0001";
    std::string toolName = "Endmill";
    double toolDiameter = 10.0;
    double toolLength = 50.0;
    int toolNumber = 1;

    int decimalPlaces = 4;
    bool absoluteMode = true;
    bool metricMode = true;

    bool lineNumbers = true;
    int startLineNumber = 1;
    int lineNumberIncrement = 10;

    bool outputComments = true;
    bool outputToolChange = true;
    bool outputCoolant = true;
    bool outputSpindle = true;

    double rapidFeedrate = 5000.0;
    double defaultFeedrate = 1000.0;
    int defaultSpindleSpeed = 3000;

    double safeZ = 100.0;
    double clearanceZ = 5.0;

    int maxLineLength = 80;

    double rapidTraverseRate = 5000.0;
    bool useG91Incremental = false;

    int workCoordinateSystem = 54;

    std::string postProcessorName = "Generic";
};

}
