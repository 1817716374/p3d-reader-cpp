#pragma once
#include <p3d/graphics_material.hpp>

namespace p3d {
struct NativeMaterialNameContext {
    // The selected project's catalog AFTER any requested native update.
    // Keep its native order, duplicates and complete UTF-16 names.
    bool catalog_complete = false;
    std::vector<std::u16string> names;
    // Source catalog name first, query second, both cut at their first NUL.
    // Match the source runtime's _wcsicmp locale. nullopt means unknown.
    std::function<std::optional<int>(const std::u16string &, const std::u16string &)> compare;
    // Receives the original catalog index, once for every matching entry.
    // Must include native cache/provider loading, not merely XML availability.
    // The result indexes the caller's loaded material collection.
    std::function<GraphicsMaterialResult(std::size_t)> load;
};
// Searches a prepared catalog with an initially empty result list. Collects all
// matching candidates before loading any; continues after successful loads.
// Does not prepare/update a project, open resources or invent cache sharing.
Json lookup_native_material_name(const std::u16string &name,
                                 const NativeMaterialNameContext &context);
} // namespace p3d
