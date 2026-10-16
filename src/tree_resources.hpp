#pragma once
#include <p3d/reader.hpp>
namespace p3d {
bool native_tree_resource_text(const Json &, std::string &);
int native_tree_decimal_digit(unsigned);
Json native_tree_property_bytes(const Json &);
Json native_tree_visibility(const Json &);
} // namespace p3d
