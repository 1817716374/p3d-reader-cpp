#pragma once
#include <p3d/reader.hpp>
#include <p3d/material_lookup.hpp>
#include <array>

namespace p3d {
struct NativeAssignmentTextContext {
    // Match the source runtime's _wcsicmp locale. Identical strings need no callback.
    std::function<std::optional<int>(const std::u16string &, const std::u16string &)> compare;
    // Match its towupper on a Windows UTF-16 code unit, not Unicode case folding.
    std::function<std::optional<char16_t>(char16_t)> uppercase;
};
// Reads the XML in group 0/key 20015 after attribute decompression. Keeps source
// entries, then reproduces insertion into a fresh table without editing callbacks.
// Unavailable locale comparisons leave normalization unresolved, never guessed.
Json decode_native_material_assignment_table(const std::string &xml,
                                             const NativeAssignmentTextContext &text = {});

// Decodes a paletteList source reference before resource-provider calls. Keeps
// the source spelling; never opens files or replaces a path with an absolute one.
// Native special-prefix comparisons use the same source text locale as tables.
Json decode_native_palette_reference(const std::string &reference,
                                     const NativeAssignmentTextContext &text = {});

struct NativeAssignmentQuery {
    std::u16string layer_name;
    std::uint32_t color = 0;
    // Fully loaded native extended color table, in its original order. nullopt
    // means unavailable, while an empty vector means confirmed empty.
    std::optional<std::vector<std::array<std::uint8_t, 3>>> extended_colors;
    NativeAssignmentTextContext text;
};
// Selects a rule in the normalized table. Does not load its referenced material,
// choose the owning model/table, or replace the earlier entity/part material path.
Json lookup_native_material_assignment(const Json &table, const NativeAssignmentQuery &query);

// Complete, attached model-head attributes in source order, including unrelated
// keys and duplicates. Selects the first readable table in the native sorted
// iterator order; an empty requested name accepts any name. Reads source payloads,
// not decoded annotations. Does not choose a model/parent or consult live caches.
Json select_native_material_assignment_table(const Json &attributes,
                                             const std::u16string &requested_name,
                                             const NativeAssignmentTextContext &text = {});

struct NativeAssignmentMaterialContext {
    NativeMaterialIdContext id;
    // Same prepared owning-project catalog as the ID query. Name queries load
    // all matches using the native NAME route's model/resource context, which
    // may differ from the ID route. Both callbacks use original catalog indices.
    std::function<std::optional<int>(const std::u16string &, const std::u16string &)> compare_name;
    std::function<GraphicsMaterialResult(std::size_t)> load_name;
};
// Selects a rule, then loads its ID. Only a known ID miss falls back to name;
// a found entry whose provider failed stops with load_failed. All successful
// name candidates are loaded before choosing the first. The owning project and
// any required catalog updates must already be established by the caller.
// A rule miss does not perform the outer parent/layer fallback or final styling.
Json resolve_native_assignment_material(const Json &table, const NativeAssignmentQuery &query,
                                        const Json &registered_catalog,
                                        const NativeAssignmentMaterialContext &context);
} // namespace p3d
