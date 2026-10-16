#pragma once
#include "internal.hpp"
namespace p3d {
Json decode_native_line_style_resource(const Bytes &);
Json decode_native_compound_line_style(const Bytes &);
Json decode_line_style_modifier_source(const Bytes &);
Json native_graphic_line_style_input(const Bytes &, const Json &);
Json native_layer_line_style_input(const Json &);
Json native_layer_line_style_slots(const Json &, std::int32_t primary_style,
                                   std::int32_t by_layer_style);
Json native_default_line_style_parameter_state();
} // namespace p3d
