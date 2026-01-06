// ModelLoader.h
#pragma once

#include <TopoDS_Shape.hxx>
#include <string>
#include <vector>

class ModelLoader
{
public:
    ModelLoader();
    ~ModelLoader() = default;

    // ==================== 主要接口 ====================
    // 加载单个文件（自动识别格式）
    bool LoadFile(const std::string& filepath, TopoDS_Shape& outShape);

    // 批量加载多个文件（返回所有成功加载的形状）
    std::vector<TopoDS_Shape> LoadFiles(const std::vector<std::string>& filepaths);

    // 获取最后一次错误信息
    std::string GetLastError() const { return m_lastError; }

    // 支持的格式（可扩展）
    static const std::vector<std::string> SupportedExtensions()
    {
        return { ".step", ".stp", ".STEP", ".STP",
                 ".iges", ".igs", ".IGES", ".IGS",
                 ".stl", ".STL",
                 ".brep", ".BRep", ".BREP" };
    }

private:
    // 各种格式的专用加载函数
    bool LoadSTEP(const std::string& filepath, TopoDS_Shape& shape);
    bool LoadIGES(const std::string& filepath, TopoDS_Shape& shape);
    bool LoadSTL(const std::string& filepath, TopoDS_Shape& shape);
    bool LoadBRep(const std::string& filepath, TopoDS_Shape& shape);

    std::string m_lastError;
};