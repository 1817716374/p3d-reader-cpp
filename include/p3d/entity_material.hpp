#pragma once
#include <p3d/graphics_material.hpp>

namespace p3d {
enum class NativeEntityMaterialIdQuery {
    primary_resource_update_true,
    fallback_resource_update_false
};
struct NativeEntityMaterialContext {
    // Whether the entity model is valid and supplies its native resource context.
    // This is NOT whether a material/table/name exists. nullopt means unknown.
    std::optional<bool> entity_model_resource_available;
    // Both routes use the selected owning-project material manager. Their
    // resource contexts and update arguments differ; do not merge the requests.
    // Results must include actual catalog/cache/provider loading, not ID equality.
    std::function<GraphicsMaterialResult(std::uint64_t, NativeEntityMaterialIdQuery)> lookup_id;
    // Complete primary name query with update=true, including project preparation
    // and loading ALL native candidates before returning the first successful one.
    std::function<GraphicsMaterialResult(const std::u16string &)> lookup_name;
    // NUL-excluded source Windows ANSI bytes -> UTF-8. Absent: ASCII only.
    std::function<std::string(const Bytes &)> ansi_decoder;
};
struct NativeEntityMaterialSelection {
    // Directly usable as GraphicsMaterialContext::native_entity_material.
    GraphicsMaterialResult material;
    Json report;
};
// native_record is a present entity's complete native record, including its
// ordered links array. Selects the first user ID/name linkage from source bytes;
// decoded annotations are not used for selection. The entity's owning project
// must already be selected; runtime project/resource discovery is not performed.
// Applies primary ID OR primary name, then the outer ID fallback on known null.
// Unavailable contexts/results stop fallback; successful choices do not inspect
// irrelevant lower-priority payloads. Inputs are not modified; callbacks run
// synchronously in native query order and are never retried automatically.
NativeEntityMaterialSelection
resolve_native_entity_material(const Json &native_record,
                               const NativeEntityMaterialContext &context);
} // namespace p3d
