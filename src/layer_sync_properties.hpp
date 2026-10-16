#pragma once
#include <array>

namespace p3d::detail {
struct LayerSyncProperty {
    unsigned bit;
    const char *override_name;
    const char *result_name;
    const char *source_container;
    const char *source_member;
};
inline constexpr std::array<LayerSyncProperty, 14> layer_sync_properties{{
    {6, "primary_color", "primary_color_index", "primary_symbology", "color_index"},
    {7, "primary_line_style", "primary_line_style", "primary_symbology", "line_style"},
    {9, "primary_line_weight", "primary_line_weight", "primary_symbology", "line_weight"},
    {11, "color", "color_index", "by_layer_symbology", "color_index"},
    {12, "line_style", "line_style", "by_layer_symbology", "line_style"},
    {14, "line_weight", "line_weight", "by_layer_symbology", "line_weight"},
    {20, "access_mode", "access_mode", nullptr, "access_mode"},
    {25, "display", "display", nullptr, "display"},
    {26, "print", "print", nullptr, "print"},
    {32, "frozen", "frozen", nullptr, "frozen"},
    {34, "display_priority", "display_priority", nullptr, "display_priority"},
    {35, "transparency", "transparency", nullptr, "transparency"},
    {38, "primary_material_reference", "primary_material_id", "primary_material_reference",
     "material_id"},
    {39, "material_reference", "material_id", "material_reference", "material_id"},
}};
inline const LayerSyncProperty *layer_sync_property(unsigned bit) {
    for (const auto &property : layer_sync_properties)
        if (property.bit == bit)
            return &property;
    return nullptr;
}
} // namespace p3d::detail
