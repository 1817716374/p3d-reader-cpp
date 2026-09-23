#pragma once
#include <p3d/reader.hpp>

namespace p3d {
// Input is a UTF-8 Identifier truncated at its first NUL. Return the bytes
// produced by the originating Windows ANSI code page (flags 0), or nullopt
// when that conversion is unavailable. No current-machine code page is assumed.
using PersistentDataNameEncoder = std::function<std::optional<Bytes>(const std::string &)>;
struct PersistentDataIndexOptions {
    PersistentDataNameEncoder encode_name;
    // All DataUnit objects of one explicitly selected project are supplied.
    bool complete_project = false;
    // Set only if input order is the native dataset enumeration order.
    // Document::objects() order alone does not establish this.
    bool native_enumeration_order = false;
};
// Consumes Document::objects() or a subset with the same object representation.
// Indexes saved values only; runtime caches and uncommitted edits are excluded.
Json persistent_data_index(const Json &objects, const PersistentDataIndexOptions &options = {});
// reference is a decoded persistent_data_reference node. Resolves one hop;
// cycles and shared targets are retained rather than recursively expanded.
Json resolve_persistent_data_reference(const Json &reference, const Json &index);
} // namespace p3d
