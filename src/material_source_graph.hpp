#pragma once
#include <p3d/material_sources.hpp>
#include <map>
namespace p3d::detail {
Json material_source_graph(const Json &materials, const Json &catalogs,
                           const Json &graphics_records, const Json &native_records,
                           const StoredMaterialSourceOptions &);
void attach_native_element_sources(Json &graph, const Json &graphics_records,
                                   const Json &native_records,
                                   const std::map<std::string, std::uint64_t> &model_ids,
                                   const std::string &sm_path, const std::string &smga_path,
                                   const std::string &smg_path);
} // namespace p3d::detail
