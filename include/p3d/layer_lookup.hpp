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
struct NativeLayerNameCacheEntry {
    std::uint32_t layer_id;
    std::u16string name;
    // Whether the actual loaded node has a registered name key. This is
    // independent of whether a saved name string is present.
    bool name_registered = true;
};
struct NativeLayerNameSyncRequest {
    std::uint32_t layer_id;
    std::u16string name;
};
// Evaluate non-null name synchronization in the supplied actual order, against
// a prepared cache after membership alignment. Inputs must already include
// native loading/default-name repair; saved record strings alone do not prove
// this state. A populated initial index must correspond to a fresh rebuild
// from these nodes in unsigned ID order; arbitrary stale/live indexes are
// outside this input profile. Empty and populated indexes behave differently.
// Does not mutate inputs or perform native resource allocation/notifications.
// Unknown context stops the sequence and withholds the final layers array.
Json evaluate_native_layer_name_sync(const std::vector<NativeLayerNameCacheEntry> &,
                                     const std::vector<NativeLayerNameSyncRequest> &,
                                     bool initial_name_index_populated,
                                     const NativeLayerNameContext &context = {});
// Prepare names from this document's validated saved tables, then evaluate
// each group with the supplied source locale. Does not require a caller-made
// cache. Generated/malformed initial names keep the affected group unresolved.
// Returns source table indices and a separate name_synchronization result;
// name evaluation is independent of the group's scalar synchronization mode.
Json read_native_layer_group_names(const Document &, const NativeLayerNameContext &context = {});
struct NativeLayerNameLoadContext {
    NativeLayerNameContext text;
    // Complete names already registered in the shared layer-name registry,
    // before this table loads. Empty is known empty; nullopt is unavailable.
    // Registration and lookup must use the same source text context.
    // Native transient numeric keys are not needed or synthesized for callers.
    std::optional<std::vector<std::u16string>> registered_names_before_load;
};
struct NativeLayerGroupNameLoadContext {
    NativeLayerNameContext text;
    // Complete registry contents at each table's own loading time. Neither
    // prefix can be inferred from the other table's final registry.
    std::optional<std::vector<std::u16string>> file_registered_names_before_load;
    std::optional<std::vector<std::u16string>> group_registered_names_before_load;
};
// Reconstruct one fresh writable local/group table's base name-loading phase,
// without an old name-to-ID repair map. Includes missing-name generation and
// the index actually captured while loading; it is not always a final rebuild.
// Does not perform later group synchronization, invalidation or host callbacks.
// State is owned and immutable after construction; const queries may share it
// across threads when the source text callbacks are themselves thread safe.
class NativeLayerNameCache {
  public:
    NativeLayerNameCache(const Json &native_records, std::size_t table_record_index,
                         const NativeLayerNameLoadContext &context = {});
    const Json &metadata() const noexcept;
    Json lookup(const std::u16string &) const;
    // Derive the group's membership/name phase from these base load snapshots.
    // The caller must establish the same file owner and source text context.
    // This preserves the actual captured index, including stale/dangling keys.
    // Const lookups are observations, not replayed index-population operations.
    NativeLayerNameCache synchronize_group_names(const NativeLayerNameCache &file,
                                                 bool confirmed_same_file_owner) const;
    // Select the unique same-owner local table from the document's catalog,
    // load both tables, then evaluate membership/name synchronization.
    // group_table_index indexes layer_tables().tables, not native_records().
    static NativeLayerNameCache load_group(const Document &, std::size_t group_table_index,
                                           const NativeLayerGroupNameLoadContext &context = {});

  private:
    struct Impl;
    explicit NativeLayerNameCache(std::shared_ptr<const Impl>);
    std::shared_ptr<const Impl> impl_;
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
    // Exact loaded layer-ID lookup, before numeric-key lookup and without a
    // fallback definition. Returns its numeric key and source record index.
    Json lookup_layer(std::uint32_t layer_id) const;
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
