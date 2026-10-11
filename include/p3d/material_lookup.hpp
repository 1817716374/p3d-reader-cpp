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

struct NativeGraphicsMaterialNameContext {
    // The current project's prepared catalog, with stable native enumeration order.
    // load must preserve native cache/provider behavior across repeated calls.
    NativeMaterialNameContext catalog;
    // Display name from a successfully loaded material's settings (not the
    // catalog name or serialized XML label). nullopt means unavailable.
    std::function<std::optional<std::u16string>(std::size_t)> display_name;
};
// BPGraphics material registration/restoration name query: first load all catalog
// name matches. If none load, load the entire catalog, then select the first loaded
// display-name match. Both searches use the source locale's _wcsicmp semantics.
// Display comparison calls catalog.compare(query, display), unlike the first pass.
// Empty queries can match empty display names. Does not register a missing material
// or convert a project material back into BPMaterial. Not the entity-name query.
Json lookup_native_graphics_material_name(const std::u16string &name,
                                          const NativeGraphicsMaterialNameContext &context);

struct NativeMaterialIdContext {
    // Unset denotes the catalog's current resource context, not an empty key.
    // An explicit key is UTF-16 and ends at its first NUL.
    std::optional<std::u16string> resource_index_key;
    // Pure comparison of source context key first, query context key second.
    // Must match the source _wcsicmp locale AND the catalog's registration.
    // Only different, concrete keys need this callback; nullopt means unknown.
    std::function<std::optional<int>(const std::u16string &, const std::u16string &)> compare;
    // Receives the selected index in registered_catalog.entries, exactly once.
    // Must include actual native cache/provider loading; XML alone is insufficient.
    std::function<GraphicsMaterialResult(std::size_t)> load;
};
// registered_catalog is one complete, prepared catalog in the form returned by
// Document::native_material_catalog(), not that method's outer array. The caller
// must select the correct project and perform any required updates first.
// Selects by (full ID, resource index key), then loads the single indexed entry.
// Does not search by name or try another entry after a failed/unknown load.
// Symbolic current contexts refer to this catalog only, never another file.
Json lookup_native_material_id(std::uint64_t id, const Json &registered_catalog,
                               const NativeMaterialIdContext &context = {});
} // namespace p3d
