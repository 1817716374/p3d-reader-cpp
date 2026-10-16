#pragma once
#include <p3d/layer_lookup.hpp>

namespace p3d {
// Layer component of a native parent-symbology stack entry. Other color,
// line-style and fill components are separate and are not evaluated here.
struct NativeParentLayerRule {
    bool force_layer = false;
    bool use_numeric_offset = false;
    std::uint32_t layer_id = 0;
    std::optional<std::uint32_t> numeric_offset;
};
struct NativeBlockParentLayerInput {
    Json data;
    std::optional<NativeParentLayerRule> rule;
};
// The selected instance (type 62), definition (type 32), and fresh local index
// must belong to the caller's actual native loading context. Reads raw bodies,
// not annotations. This does not select the definition, model or layer cache.
// Requires a present valid model; a known null model does not push this rule.
// Missing cache members contribute numeric key zero; unknown cache state does
// not. A partial result may retain a rule with an unknown numeric offset.
NativeBlockParentLayerInput prepare_native_block_parent_layer(const Json &instance_record,
                                                              const Json &definition_record,
                                                              const NativeLayerNumericIndex &);
// On stack push, the previous combined entry overrides enabled layer/offset
// fields in the current entry. Offsets are copied, never added together.
NativeParentLayerRule merge_native_parent_layer_rules(const NativeParentLayerRule &current,
                                                      const NativeParentLayerRule &previous);
// protected_layer means the native style's layer-protection bit is already
// known set after local overrides; it bypasses this entire parent layer query.
// Requires an actual parent stack entry; do not call for an absent parent.
// The result's protected_layer is the updated bit, including on an unresolved
// numeric lookup, so a prepared offset is not applied a second time.
Json apply_native_parent_layer_rule(std::uint32_t layer_id, bool protected_layer,
                                    const NativeParentLayerRule &, const NativeLayerNumericIndex &);
} // namespace p3d
