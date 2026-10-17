#pragma once
#include "internal.hpp"

namespace p3d::detail {
enum class PropertyBinaryCodec {
    extension,
    component_instances,
    component_properties,
    secondary_properties,
    legacy_graphics,
    directed_mesh,
    analytic_profile
};
struct PropertyBinaryContract {
    PropertyBinaryCodec codec;
    const char *encoding;
};
std::optional<PropertyBinaryContract>
property_binary_contract(const std::string &declaring_namespace, const std::string &name);
// Strict known-codec entry, without decode_binary_field's unrelated fallback
// probes. No DataBlock/BfaTree recursion or caller-controlled codec dispatch.
Json decode_property_binary_value(PropertyBinaryCodec, const Bytes &,
                                  const std::string &root_class);
} // namespace p3d::detail
