#pragma once
#include <p3d/reader.hpp>
namespace p3d {
bool native_tree_resource_text(const Json &, std::string &);
int native_tree_decimal_digit(unsigned);
struct NativeTreeInteger {
    std::int64_t value = 0;
    bool digits = false, saturated = false;
};
NativeTreeInteger native_tree_integer32(const std::string &);
Json native_tree_property_bytes(const Json &);
Json native_tree_visibility(const Json &);
Json native_tree_fold_input(const Json &);
Json native_tree_ui_parameters(const Json &);
} // namespace p3d
