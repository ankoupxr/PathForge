// ModelLoader.cpp
#include "ModelLoader.h"

#include <STEPControl_Reader.hxx>
#include <IGESControl_Reader.hxx>
#include <StlAPI_Reader.hxx>
#include <BRepTools.hxx>
#include <TopoDS_Shape.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <BRep_Builder.hxx>

#include <iostream>
#include <algorithm>
#include <cctype>

ModelLoader::ModelLoader()
{
    m_lastError.clear();
}

bool ModelLoader::LoadFile(const std::string& filepath, TopoDS_Shape& outShape)
{
    if (filepath.empty())
    {
        m_lastError = "文件路径为空";
        return false;
    }

    // 提取小写后缀
    std::string ext;
    size_t dotPos = filepath.find_last_of('.');
    if (dotPos != std::string::npos)
        ext = filepath.substr(dotPos);

    std::string lowerExt = ext;
    std::transform(lowerExt.begin(), lowerExt.end(), lowerExt.begin(), ::tolower);

    outShape = TopoDS_Shape(); // 清空

    if (lowerExt == ".step" || lowerExt == ".stp")
        return LoadSTEP(filepath, outShape);
    else if (lowerExt == ".iges" || lowerExt == ".igs")
        return LoadIGES(filepath, outShape);
    else if (lowerExt == ".stl")
        return LoadSTL(filepath, outShape);
    else if (lowerExt == ".brep")
        return LoadBRep(filepath, outShape);
    else
    {
        m_lastError = "不支持的文件格式: " + ext;
        return false;
    }
}

bool ModelLoader::LoadSTEP(const std::string& filepath, TopoDS_Shape& shape)
{
    STEPControl_Reader reader;
    IFSelect_ReturnStatus status = reader.ReadFile(filepath.c_str());

    if (status != IFSelect_RetDone)
    {
        m_lastError = "STEP 文件读取失败: " + filepath;
        return false;
    }

    int nbRoots = reader.NbRootsForTransfer();
    reader.TransferRoots();
    shape = reader.OneShape();

    if (shape.IsNull())
        m_lastError = "STEP 文件内容为空";
    else
        std::cout << "STEP 加载成功: " << filepath << " (共 " << reader.NbShapes() << " 个实体)\n";

    return !shape.IsNull();
}

bool ModelLoader::LoadIGES(const std::string& filepath, TopoDS_Shape& shape)
{
    IGESControl_Reader reader;
    int status = reader.ReadFile(filepath.c_str());

    if (status != IFSelect_RetDone)
    {
        m_lastError = "IGES 文件读取失败";
        return false;
    }

    reader.TransferRoots();
    shape = reader.OneShape();

    std::cout << "IGES 加载成功: " << filepath << "\n";
    return !shape.IsNull();
}

bool ModelLoader::LoadSTL(const std::string& filepath, TopoDS_Shape& shape)
{
    StlAPI_Reader stlReader;
    if (!stlReader.Read(shape,filepath.c_str()))
    {
        m_lastError = "STL 文件读取失败";
        return false;
    }

    std::cout << "STL 网格加载成功: " << filepath << "\n";
    return !shape.IsNull();
}

bool ModelLoader::LoadBRep(const std::string& filepath, TopoDS_Shape& shape)
{
    if (!BRepTools::Read(shape, filepath.c_str(), BRep_Builder()))
    {
        m_lastError = "BRep 文件读取失败";
        return false;
    }

    std::cout << "BRep 原生格式加载成功: " << filepath << "\n";
    return true;
}

std::vector<TopoDS_Shape> ModelLoader::LoadFiles(const std::vector<std::string>& filepaths)
{
    std::vector<TopoDS_Shape> shapes;
    shapes.reserve(filepaths.size());

    for (const auto& path : filepaths)
    {
        TopoDS_Shape s;
        if (LoadFile(path, s) && !s.IsNull())
            shapes.push_back(s);
        else
            std::cerr << "跳过失败文件: " << path << " (" << GetLastError() << ")\n";
    }

    return shapes;
}