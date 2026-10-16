#pragma once
#include <p3d/layer_lookup.hpp>

namespace p3d {
// Color index, weight, style number and modifier state of an initial type-62
// parent entry. The fresh index and records must be from the actual selected
// model's cache. Requires a present valid model; does not discover it. Raw
// bodies and links, rather than decoded annotations, supply the values.
// Layer, priority and cross-file resource loading are separate components.
Json prepare_native_block_parent_symbols(const Json &instance_record, const Json &native_records,
                                         const NativeLayerNumericIndex &);
// Current entry plus the actual previous combined stack entry. First resolve
// by-parent scalar sentinels; then previous bits 0x08/0x10/0x20 replace color,
// weight, or style and the complete modifier state. A style sentinel alone
// does not copy modifiers. Does not apply this entry to a drawn entity.
Json merge_native_parent_symbols(const Json &current, const Json &previous);
} // namespace p3d
