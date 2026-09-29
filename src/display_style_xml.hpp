#pragma once
#include "internal.hpp"

namespace p3d {
// Confirmed XML input writes; does not register or select a runtime style.
Json decode_display_style_xml(const Json &tree);
Json decode_display_style_usages(const Json &attributes);
Json project_display_style_lite_copy(const Json &tree);
Json plan_initial_lite_style_list(const Json &common, const Json &lite);
std::array<std::uint32_t, 2> apply_display_style_flag_words(
    std::uint32_t first, std::uint32_t second, std::uint32_t flags48,
    std::uint32_t flags50, std::uint64_t material);
Json project_display_style_view_flags(const Json &record, const Json &imported);
Json project_initial_view_background(const Json &record);
}
