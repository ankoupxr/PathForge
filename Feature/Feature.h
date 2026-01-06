#pragma once

#include "FeatureType.h"

#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>

#include <vector>
#include <string>
#include <unordered_map>

namespace PathForge::Feature {

    class Feature
    {
    public:
        // ========= 构造 =========
        explicit Feature(FeatureType type = FeatureType::Unknown);

        // ========= 基本属性 =========
        FeatureType type() const;
        const std::string& name() const;

        // ========= 几何组成 =========
        void addFace(const TopoDS_Face& face);
        void addEdge(const TopoDS_Edge& edge);
        void addVertex(const TopoDS_Vertex& vertex);

        const std::vector<TopoDS_Face>& faces() const;
        const std::vector<TopoDS_Edge>& edges() const;
        const std::vector<TopoDS_Vertex>& vertices() const;

        // ========= 语义判断 =========
        bool isHole() const;
        bool isPocket() const;
        bool isSlot() const;
        bool isChamfer() const;
        bool isFillet() const;

        // ========= 参数接口 =========
        void setParam(const std::string& key, double value);
        bool hasParam(const std::string& key) const;
        double param(const std::string& key, double defaultValue = 0.0) const;

        const std::unordered_map<std::string, double>& params() const;

        // ========= 特征关系 =========
        void merge(const Feature& other);
        bool isValid() const;

        // ========= 调试 =========
        std::string debugString() const;

        // ========= 类型设置（安全） =========
        void setType(FeatureType type);

        // ========= 快捷接口 =========
        void setPrimaryFace(const TopoDS_Face& face);

        // ========= 工厂方法（推荐用这个） =========
        static Feature CreateFromFace(
            FeatureType type,
            const TopoDS_Face& face
        );

    private:
        FeatureType m_type;
        std::string m_name;

        std::vector<TopoDS_Face>   m_faces;
        std::vector<TopoDS_Edge>   m_edges;
        std::vector<TopoDS_Vertex> m_vertices;

        std::unordered_map<std::string, double> m_params;
    };

} // namespace PathForge::Feature
