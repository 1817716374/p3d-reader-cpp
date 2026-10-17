#pragma once
#include <p3d/material_sources.hpp>
namespace p3d::detail {
Json material_source_graph(const Json &materials, const Json &catalogs,
                           const Json &graphics_records, const Json &native_records,
                           const StoredMaterialSourceOptions &);
} // namespace p3d::detail
