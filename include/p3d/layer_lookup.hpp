#pragma once
#include <p3d/reader.hpp>
#include <map>

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
// in ascending order, overwriting equivalent names. Fresh input rejects IDs
// zero and UINT32_MAX, without a prior name-to-ID repair map.
// table_record_index indexes Document::native_records(), not layer_tables().tables.
// Does not select a model/parent, apply live edits or load a referenced material.
Json lookup_native_layer_name(const Json &native_records, std::size_t table_record_index,
                              const std::u16string &name,
                              const NativeLayerNameContext &context = {});

// Reconstructs the independent native numeric-key index for one fresh local
// table. Builds once; const queries share that index without source copies.
// Equal keys select the greatest unsigned loaded layer ID, after same-ID
// records load last. Raw bodies/linkages are authoritative. No host services,
// prior name-to-ID repair map, parent discovery or runtime edits are included.
class NativeLayerNumericIndex {
  public:
    NativeLayerNumericIndex(const Json &native_records, std::size_t table_record_index);
    const Json &metadata() const noexcept {
        return data_;
    }
    Json lookup(std::uint32_t numeric_key) const;
    // Offset branch of the native parent-layer query. Layer zero bypasses
    // lookup. Otherwise add offset to the source key with uint32 wrap; a
    // confirmed source/key miss yields layer 64, not a fabricated definition.
    // A mapped ID of UINT32_MAX-1 is replaced by parent_layer_id. If that
    // actual parent value is unavailable, keep the result unresolved.
    Json remap_offset(std::uint32_t layer_id, std::uint32_t offset,
                      std::optional<std::uint32_t> parent_layer_id = {}) const;

  private:
    Json data_;
    std::map<std::uint32_t, std::size_t> keys_;
    std::map<std::uint32_t, std::pair<std::uint32_t, std::size_t>> ids_;
};
} // namespace p3d
