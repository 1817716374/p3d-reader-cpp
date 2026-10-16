#pragma once
#include <p3d/reader.hpp>
namespace p3d {
// Materialize one complete record after the known file-input layout/header
// changes and ID assignment. The explicit values must come from its actual
// initial input occurrence, not from unrelated IDs or physical adjacency.
// Keeps source bytes; does not execute runtime callbacks or build a descriptor.
Json prepare_native_record_input(const Bytes &complete_record, std::uint64_t assigned_id,
                                 std::uint32_t accepted_descendant_count, bool is_child,
                                 bool system_container);
} // namespace p3d
