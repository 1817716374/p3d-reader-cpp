#pragma once
#include <p3d/reader.hpp>
#include <map>
#include <set>

namespace p3d {
struct ReferenceModelQuery {
    std::optional<std::uint32_t> secondary_flags;
    std::optional<std::uint32_t> runtime_flags;
    std::optional<std::uint32_t> explicit_model_id;
    // Name already selected by the ordinary wrapper (primary or alternate).
    // Empty means a known empty/null name; nullopt means unknown.
    std::optional<std::u16string> requested_name;
    std::uint32_t requested_load_mask = 0;
};
struct ReferenceModelFileContext {
    std::optional<std::uint32_t> default_model_id;
    // Current native directory; a persisted directory is suitable only when it
    // describes the actual directory at this lookup. Borrowed, never modified.
    const Json *directory = nullptr;
    bool directory_known_absent = false;
    NativeModelNameEqual equal;
};
// Selects the target ID after a file has been acquired and before model lookup.
// Does not run the earlier file-query gates or the later attachment callbacks.
Json select_reference_model(const ReferenceModelQuery &query,
                            const ReferenceModelFileContext &file);
ReferenceModelQuery initial_reference_model_query(const Json &reference_record,
                                                  std::uint32_t requested_load_mask = 0);

struct NativeModelLookupContext {
    // Native signed-ID cache. Values are caller graph identities, not model IDs.
    // A known null entry falls through; a positive hit needs no complete suffix.
    std::map<std::int32_t, std::optional<std::size_t>> model_cache;
    bool model_cache_complete = false;
    // ID -1 uses this dedicated slot, bypassing the ordinary cache.
    bool special_model_known = false;
    std::optional<std::size_t> special_model;
    std::set<std::int32_t> suppressed_model_ids;
    bool suppressed_model_ids_complete = false;
    // Result of the complete native creation helper for this requested ID,
    // including any provider refresh/relookup. Not its raw provider output.
    bool creation_result_known = false;
    std::optional<std::size_t> created_model;
};
Json lookup_native_model(std::uint32_t id, const NativeModelLookupContext &context);
// Composes selection and lookup into one target identity. This does not attach
// that identity to a reference or assert that requested model data was loaded.
Json resolve_reference_model(const ReferenceModelQuery &query,
                             const ReferenceModelFileContext &file,
                             const NativeModelLookupContext &models);
enum class NativeModelReferenceOperation { add, remove };
// Complete ordered list of model back-reference pointer identities. Null is a
// known null pointer. Preserves the native compact-then-single-erase behavior,
// including its result on duplicate input. Does not execute attachment callbacks.
Json native_model_reference_list(const std::vector<std::optional<std::size_t>> &references,
                                 std::optional<std::size_t> reference,
                                 NativeModelReferenceOperation operation);
} // namespace p3d
