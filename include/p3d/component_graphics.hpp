#pragma once
#include <p3d/reader.hpp>

namespace p3d {
// Inputs are decoded ParaCmptProperty and ParaCmptInstance of the same component.
// Resolves stored-state source order under built-in callbacks, valid project/model,
// no current-edit bypass or runtime property mutations, and normal service returns.
// Unknown earlier extraction never selects a later cache. Default-handler output
// is not synthesized. Returned entry indices remain separate from final draw parts.
Json resolve_stored_component_graphics(const Json &properties, const Json &instance);
// Selects the stored graphics source before applying its material override stages.
// Ignores context.graphics_source and manual Noumenon material fields. Loaded
// material names and ANSI decoding still come from context. Preserving an entry's
// material does not establish its final identity or mean that it has no material.
Json resolve_stored_component_materials(const Json &properties, const Json &instance,
                                        const ComponentMaterialContext &context);
} // namespace p3d
