#pragma once
#include <p3d/reader.hpp>

namespace p3d {
// Identity in a caller-owned collection of successfully loaded materials.
// known=true with no index means a confirmed null result, not an unavailable loader.
struct GraphicsMaterialResult {
    bool known = false;
    std::optional<std::size_t> material_index;
};
enum class GraphicsMaterialNameQuery {
    // Current project, native name search with its update argument enabled.
    advanced_part,
    advanced_entity,
    // Owning file (or the native null-file context), update argument disabled.
    legacy_part
};
struct GraphicsMaterialContext {
    // Complete getMaterialByEeh result, including its native-ID fallback.
    GraphicsMaterialResult native_entity_material;
    // Return the first successfully loaded native search result. Comparison and
    // provider loading belong to this callback; a name match alone is insufficient.
    std::function<GraphicsMaterialResult(const std::u16string &, GraphicsMaterialNameQuery)>
        lookup_name;
    // Converts a rebuilt Entry's BPMaterial to a material in the owning project.
    std::function<GraphicsMaterialResult(const Json &)> load_inline_material;
};
// entries is the ordered, already rebuilt BPGraphics Entry array, each containing
// inline_material (null means absent). attributes is the complete selected native
// attribute collection, in source order, with group/key/index/payload fields.
// Models material selection before the ordinary draw context resolves symbology.
// Its symbology state is embedded in the context; it is not an optional per-entry
// builder. This neither reconstructs Entry order nor maps triangles to parts.
// Serialized geometry_packets are suitable ONLY when they are independently known
// to be exactly this builder's input sequence. Empty/non-emitting entries still count.
// Unknown higher-priority choices stop that entry's fallback. Inputs are not changed.
// unassigned means that this selection writes a null material, not that the final
// draw has no material. Later symbology resolution may perform another lookup;
// draw_material_status remains not_evaluated, including for resolved selections.
// Advanced names accept JSON comments and string, int64/uint64, bool and null
// values; floats and non-scalar values remain unresolved. Empty names (including
// converted null) still invoke the callback for native project preparation;
// the native catalog search cannot match them.
Json resolve_rebuilt_graphics_materials(const Json &entries, const Json &attributes,
                                        const GraphicsMaterialContext &context);
} // namespace p3d
