#pragma once
#include <p3d/reader.hpp>

namespace p3d {
struct StoredMaterialSourceOptions {
    // Registration and name comparison use the same source text profile.
    // Default: native CRT C-locale comparison. Resources are never opened.
    MaterialCatalogOptions catalog;
    std::function<std::string(const Bytes &)> ansi_decoder;
};
// Recognized file-local declared references, not successful runtime loads or final draw
// bindings. Definition indices refer to document.materials()["definitions"].
// Keeps catalog namespaces, registered order, source attribute occurrences and
// rejected/shadowed inputs distinct. Does not deduplicate by name or contents.
// Part references retain rebuilt-entry indices; no triangle mapping is inferred.
Json read_stored_material_graph(const Document &, const StoredMaterialSourceOptions & = {});
} // namespace p3d
