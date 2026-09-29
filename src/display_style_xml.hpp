#pragma once
#include "internal.hpp"

namespace p3d {
// Confirmed XML input writes; does not register or select a runtime style.
Json decode_display_style_xml(const Json &tree);
Json decode_display_style_usages(const Json &attributes);
}
