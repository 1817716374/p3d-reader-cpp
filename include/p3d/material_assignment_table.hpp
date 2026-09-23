#pragma once
#include <p3d/reader.hpp>
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
} // namespace p3d
