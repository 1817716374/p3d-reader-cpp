#pragma once
#include "internal.hpp"
namespace p3d {
Json decode_line_style_modifier_source(const Bytes &);
Json native_graphic_line_style_input(const Bytes &, const Json &);
Json native_layer_line_style_input(const Json &);
Json native_default_line_style_parameter_state();
} // namespace p3d
