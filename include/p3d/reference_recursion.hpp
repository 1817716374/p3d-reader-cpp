#pragma once
#include <p3d/reader.hpp>

namespace p3d {
struct ReferenceAncestorState {
    bool valid = true;
    // A plain model has no reference object; unknown is distinct from absent.
    bool reference_known_absent = false;
    std::optional<std::uint32_t> reference_primary_flags;
    bool file_known_absent = false;
    std::optional<NativeFileReference> file;
    std::optional<bool> is_default_model;
    std::optional<std::u16string> model_name;
};
struct ReferenceRepetitionContext {
    std::optional<bool> native_setting_enabled;
    // Result of the native host-file fallback gate (provider present, kind 3/4).
    // This is not inferred from the filename extension or model directory kind.
    std::optional<bool> host_file_fallback_gate;
    std::optional<std::u16string> lookup_reference;
    // Loaded reference's alternate file string, not its saved key-35 string.
    std::optional<std::u16string> alternate_file_reference;
    // Current host first, then successive parents. Retain skipped nodes/depths.
    std::vector<ReferenceAncestorState> ancestors;
    bool complete_ancestor_chain = false;
    // The ordinary reference-loading call supplies 1. Other values are native
    // signed arguments, not a requested number of matching ancestors.
    std::int32_t limit = 1;
    NativeModelNameEqual equal;
};
// Computes the native repetition predicate with explicit loaded-state context.
// reference_target supplies decoded persisted string slots 21/36. File loading,
// settings lookup, parent discovery and native post-match callbacks are separate.
Json reference_ancestor_repetition(std::uint32_t primary_flags, std::uint32_t secondary_flags,
                                   const Json &reference_target,
                                   const ReferenceRepetitionContext &context);
} // namespace p3d
