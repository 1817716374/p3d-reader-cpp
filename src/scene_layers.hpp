#pragma once
#include "internal.hpp"
namespace p3d {
struct NativeSceneLayerCatalog {
    struct Table {
        bool known = false;
        std::string reason;
        std::map<std::uint32_t, std::vector<std::size_t>> ids;
    };
    Json data;
    std::map<std::size_t, Table> tables;
    std::map<std::string, std::vector<std::size_t>> models;
    NativeSceneLayerCatalog(const Json &native, const Json &table_catalog);
    Json bind(const Json &model, const Json &header, const Json &group_query = Json()) const;
};
} // namespace p3d
