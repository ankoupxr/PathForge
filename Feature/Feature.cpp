#include "Feature.h"

#include <sstream>

namespace PathForge::Feature {

    // ========= 构造 =========

    Feature::Feature(FeatureType type)
        : m_type(type),
        m_name(FeatureTypeToString(type))
    {
    }

    // ========= 基本属性 =========

    FeatureType Feature::type() const
    {
        return m_type;
    }

    const std::string& Feature::name() const
    {
        return m_name;
    }

    // ========= 几何组成 =========

    void Feature::addFace(const TopoDS_Face& face)
    {
        m_faces.push_back(face);
    }

    void Feature::addEdge(const TopoDS_Edge& edge)
    {
        m_edges.push_back(edge);
    }

    void Feature::addVertex(const TopoDS_Vertex& vertex)
    {
        m_vertices.push_back(vertex);
    }

    const std::vector<TopoDS_Face>& Feature::faces() const
    {
        return m_faces;
    }

    const std::vector<TopoDS_Edge>& Feature::edges() const
    {
        return m_edges;
    }

    const std::vector<TopoDS_Vertex>& Feature::vertices() const
    {
        return m_vertices;
    }

    // ========= 语义判断 =========

    bool Feature::isHole() const
    {
        return m_type == FeatureType::Hole;
    }

    bool Feature::isPocket() const
    {
        return m_type == FeatureType::Pocket;
    }

    bool Feature::isSlot() const
    {
        return m_type == FeatureType::Slot;
    }

    bool Feature::isChamfer() const
    {
        return m_type == FeatureType::Chamfer;
    }

    bool Feature::isFillet() const
    {
        return m_type == FeatureType::Fillet;
    }

    // ========= 参数 =========

    void Feature::setParam(const std::string& key, double value)
    {
        m_params[key] = value;
    }

    bool Feature::hasParam(const std::string& key) const
    {
        return m_params.find(key) != m_params.end();
    }

    double Feature::param(const std::string& key, double defaultValue) const
    {
        auto it = m_params.find(key);
        if (it != m_params.end())
            return it->second;
        return defaultValue;
    }

    const std::unordered_map<std::string, double>& Feature::params() const
    {
        return m_params;
    }

    // ========= 合并 =========

    void Feature::merge(const Feature& other)
    {
        if (other.m_type != m_type)
            return;

        m_faces.insert(m_faces.end(), other.m_faces.begin(), other.m_faces.end());
        m_edges.insert(m_edges.end(), other.m_edges.begin(), other.m_edges.end());
        m_vertices.insert(m_vertices.end(), other.m_vertices.begin(), other.m_vertices.end());

        for (const auto& kv : other.m_params) {
            m_params[kv.first] = kv.second;
        }
    }

    // ========= 校验 =========

    bool Feature::isValid() const
    {
        // 最基本校验：必须至少有一个面
        return !m_faces.empty();
    }

    // ========= 调试 =========

    std::string Feature::debugString() const
    {
        std::ostringstream oss;
        oss << "Feature[" << m_name << "] ";
        oss << "Faces=" << m_faces.size();
        oss << ", Edges=" << m_edges.size();
        oss << ", Params={";

        for (const auto& kv : m_params) {
            oss << kv.first << ":" << kv.second << " ";
        }
        oss << "}";

        return oss.str();
    }

    // ========= 类型设置 =========

    void Feature::setType(FeatureType type)
    {
        m_type = type;
        m_name = FeatureTypeToString(type);
    }

    // ========= 主导面（你原来的 f.face = face） =========

    void Feature::setPrimaryFace(const TopoDS_Face& face)
    {
        m_faces.clear();
        m_faces.push_back(face);
    }

    // ========= 工厂方法（强烈推荐） =========

    Feature Feature::CreateFromFace(
        FeatureType type,
        const TopoDS_Face& face)
    {
        Feature f(type);
        f.addFace(face);
        return f;
    }



} // namespace PathForge::Feature
