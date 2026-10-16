#pragma once
#include <p3d/reader.hpp>

namespace p3d {
struct NativeLayerNameContext {
    // Pure comparisons/conversions matching the source runtime's locale.
    // Lowercase operates on one Windows UTF-16 code unit, not Unicode folding.
    std::function<std::optional<char16_t>(char16_t)> lowercase;
    std::function<std::optional<int>(const std::u16string &, const std::u16string &)> compare;
};
// Reconstructs name lookup for a fresh local layer table after its saved records
// have loaded. Uses raw record bodies/linkages, not decoded layer annotations.
// Duplicate layer IDs load last; the native name index then visits unsigned IDs
// in ascending order, overwriting equivalent names. UINT32_MAX IDs are skipped.
// table_record_index indexes Document::native_records(), not layer_tables().tables.
// Does not select a model/parent, apply live edits or load a referenced material.
Json lookup_native_layer_name(const Json &native_records, std::size_t table_record_index,
                              const std::u16string &name,
                              const NativeLayerNameContext &context = {});
} // namespace p3d
