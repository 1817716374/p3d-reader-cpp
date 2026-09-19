#pragma once
#include "internal.hpp"

namespace p3d {
Json native_material_texture_file(const Bytes &payload);
Json native_catalog_material_read(const Json &catalog, const Json &container,
                                  const Json &native_records,
                                  const MaterialCatalogOptions &options = {});
} // namespace p3d
