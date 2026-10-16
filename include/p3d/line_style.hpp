#pragma once
#include <p3d/reader.hpp>
namespace p3d {
// Native fresh 0xa8 parameter initialization plus one user linkage 0x79f9.
// Reconstructs the dimension-specific orientation and native rejected-flag
// reset. Does not choose a graphical caller or load a line-style resource.
Json load_native_line_style_parameter_link(const Json &link, bool three_dimensional);
// Graphic-record route: native extended-header/style gates and first matching
// user linkage. The record's raw dimension bit supplies the orientation mode.
// Type 63 requires its separate multiline reader and remains not_evaluated.
Json read_native_graphic_line_style_parameters(const Json &native_record);
} // namespace p3d
