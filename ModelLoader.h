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

    // Load single file, auto-detect format
    bool LoadFile(const std::string& filepath, TopoDS_Shape& outShape);

    // Batch load multiple files
    std::vector<TopoDS_Shape> LoadFiles(const std::vector<std::string>& filepaths);

    // Get last error message
    std::string GetLastError() const { return m_lastError; }

    // Supported file extensions
    static const std::vector<std::string> SupportedExtensions()
    {
        return { ".step", ".stp", ".STEP", ".STP",
                 ".iges", ".igs", ".IGES", ".IGS",
                 ".stl", ".STL",
                 ".brep", ".BRep", ".BREP" };
    }

private:
    // Format-specific load functions
    bool LoadSTEP(const std::string& filepath, TopoDS_Shape& shape);
    bool LoadIGES(const std::string& filepath, TopoDS_Shape& shape);
    bool LoadSTL(const std::string& filepath, TopoDS_Shape& shape);
    bool LoadBRep(const std::string& filepath, TopoDS_Shape& shape);

    std::string m_lastError;
};
