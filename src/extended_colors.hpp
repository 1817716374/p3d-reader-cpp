#pragma once
#include "internal.hpp"
namespace p3d {
Json decode_native_extended_colors(const Json &tree);
Json initial_native_extended_color_table(const Json &list, const Json &records,
                                         const Json &ids, const Json &input);
}
