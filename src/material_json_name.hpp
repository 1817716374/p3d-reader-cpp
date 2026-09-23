#pragma once
#include <p3d/reader.hpp>

namespace p3d::detail {
// Native JsonCpp asString subset used by advanced part-material names.
// Throws for unconfirmed floating conversion and non-convertible containers.
std::string material_json_name(const Json &value);
} // namespace p3d::detail
