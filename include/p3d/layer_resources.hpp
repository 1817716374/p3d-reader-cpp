#pragma once
#include <p3d/material_lookup.hpp>

namespace p3d {
// Decode the bounded text consumed by the native layer material-name reader.
// Does not interpret the first quoted component as a file to open.
Json decode_native_layer_material_name(const std::u16string &);
// Decode three signed decimal values and the native fixed-position name pair.
// RGB bytes are the low eight bits, not clamped values. Unsafe reads remain
// unresolved. Does not resolve a color index or assume a palette.
Json decode_native_layer_color_name(const std::u16string &);
// Builtin names are case sensitive. Other nonempty names require the native
// prepared-owner lookup with import=false, alternate=false. Its zero result
// becomes STYLE_Invalid. This helper does not discover/select that owner.
Json lookup_native_line_style_name(
    const std::u16string &,
    const std::function<std::optional<std::int32_t>(const std::u16string &)> &lookup = {});
// Evaluate the two material-name assignments for one saved layer. The owner
// context, resource service and catalog preparation must already have succeeded.
// catalog is the prepared catalog for that actual owner, not an arbitrary file.
// No request, empty parsed name, miss or selected UINT64_MAX preserves saved ID.
// This does not execute resource services, other symbology or group sync.
Json resolve_native_layer_material_names(const Json &native_records, std::size_t layer_record_index,
                                         const Json &prepared_catalog,
                                         const NativeMaterialReferenceNameContext &context = {});
} // namespace p3d
