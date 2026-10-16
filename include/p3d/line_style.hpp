#pragma once
#include <p3d/reader.hpp>
namespace p3d {
// Read saved line-style name/component tables and records from raw data.
// Keeps native component IDs and source fields; does not select an owner,
// execute resource services or resolve final graphical line styles.
Json read_native_line_style_resource(const Json &native_record);
// Native fresh 0xa8 parameter initialization plus one user linkage 0x79f9.
// Reconstructs the dimension-specific orientation and native rejected-flag
// reset. Does not choose a graphical caller or load a line-style resource.
Json load_native_line_style_parameter_link(const Json &link, bool three_dimensional);
// Graphic-record route: native extended-header/style gates, dimension mode and
// linkage selection. Type 63 uses the multiline default-parameter route: the
// first converted data-type-zero entry wins; a known miss writes zero state.
Json read_native_graphic_line_style_parameters(const Json &native_record);
} // namespace p3d
