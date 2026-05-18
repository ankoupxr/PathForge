#include "SimulationResults.h"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctime>

namespace PathForge::Simulation {

bool SimulationResults::exportToFile(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }

    file << generateReport();
    file.close();
    return true;
}

std::string SimulationResults::generateReport() const {
    std::ostringstream report;

    report << "==============================================\n";
    report << "          PathForge 仿真报告\n";
    report << "==============================================\n\n";

    std::time_t now = std::time(nullptr);
    report << "生成时间: " << std::ctime(&now) << "\n";

    report << "----------------------------------------------\n";
    report << "仿真状态\n";
    report << "----------------------------------------------\n";
    report << "仿真有效: " << (isValid ? "是" : "否") << "\n";
    report << "存在碰撞: " << (hasCollisions ? "是" : "否") << "\n";
    report << "存在过切: " << (hasGouges ? "是" : "否") << "\n";
    report << "综合风险: " << std::fixed << std::setprecision(2) << (overallRisk * 100) << "%\n";
    report << "\n";

    report << "----------------------------------------------\n";
    report << "统计信息\n";
    report << "----------------------------------------------\n";
    report << "总点数: " << statistics.totalPoints << "\n";
    report << "切削点数: " << statistics.cuttingPoints << "\n";
    report << "快速移动点数: " << statistics.rapidPoints << "\n";
    report << "总路径长度: " << std::fixed << std::setprecision(2) << statistics.totalPathLength << " mm\n";
    report << "切削长度: " << std::fixed << std::setprecision(2) << statistics.cuttingLength << " mm\n";
    report << "估算加工时间: " << std::fixed << std::setprecision(2) << statistics.estimatedTime << " min\n";
    report << "材料去除体积: " << std::fixed << std::setprecision(2) << statistics.materialRemovalVolume << " mm³\n";
    report << "\n";

    report << "----------------------------------------------\n";
    report << "碰撞信息 (共 " << collisions.size() << " 处)\n";
    report << "----------------------------------------------\n";
    if (collisions.empty()) {
        report << "未检测到碰撞\n";
    } else {
        for (const auto& col : collisions) {
            report << "点 #" << col.pointIndex
                   << " 位置: (" << std::fixed << std::setprecision(3)
                   << col.position.X() << ", " << col.position.Y() << ", " << col.position.Z() << ")\n";
            report << "  类型: " << col.description << "\n";
            report << "  距离: " << std::fixed << std::setprecision(3) << col.distance << " mm\n";
            report << "  严重度: " << (col.severity < 0.5 ? "轻微" : (col.severity < 1.0 ? "中等" : "严重")) << "\n";
        }
    }
    report << "\n";

    report << "----------------------------------------------\n";
    report << "过切信息 (共 " << gouges.size() << " 处)\n";
    report << "----------------------------------------------\n";
    if (gouges.empty()) {
        report << "未检测到过切\n";
    } else {
        for (const auto& gou : gouges) {
            report << "点 #" << gou.pointIndex
                   << " 位置: (" << std::fixed << std::setprecision(3)
                   << gou.position.X() << ", " << gou.position.Y() << ", " << gou.position.Z() << ")\n";
            report << "  过切深度: " << std::fixed << std::setprecision(3) << gou.gougeDepth << " mm\n";
            report << "  过切面积: " << std::fixed << std::setprecision(3) << gou.gougeArea << " mm²\n";
            report << "  严重度: " << (gou.severity == GougeSeverity::Minor ? "轻微" :
                                      (gou.severity == GougeSeverity::Moderate ? "中等" : "严重")) << "\n";
        }
    }
    report << "\n";

    report << "----------------------------------------------\n";
    report << "仿真时间: " << simulationTime.count() << " ms\n";
    report << "==============================================\n";

    return report.str();
}

}
