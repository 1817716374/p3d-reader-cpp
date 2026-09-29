#include <p3d/default_view_table.hpp>
#include <cmath>
#include <limits>
namespace p3d {
NativeCachedModelBoundsResult project_native_cached_model_bounds(const NativeCachedModelBoundsInput &in) {
    NativeCachedModelBoundsResult out;
    if (!in.range_pointer_known) { out.reason="cached_model_bounds_pointer_unknown"; return out; }
    if (!in.integer_range) { out.return_code=1; out.resolved=true; return out; }
    if (!in.spatial_byte_78) { out.reason="cached_model_bounds_spatial_state_unknown"; return out; }
    const auto &r=*in.integer_range;
    const unsigned dimensions=*in.spatial_byte_78?3:2;
    for (unsigned i=0;i<dimensions;++i)
        if (r[i]>r[i+3] || r[i]>=std::numeric_limits<std::int64_t>::max() ||
            r[i+3]<=std::numeric_limits<std::int64_t>::min()) {
            out.return_code=1; out.resolved=true; return out;
        }
    std::array<double,6> range{};
    for (unsigned i=0;i<6;++i) range[i]=static_cast<double>(r[i]);
    if (!*in.spatial_byte_78) range[2]=range[5]=0;
    out.range=range; out.return_code=0; out.resolved=true; return out;
}
NativeDefaultRangeResult select_native_default_view_range(
    const std::array<double,6> &range, const NativeDefaultRangeScaleContext &c) {
    NativeDefaultRangeResult out; out.range = range;
    bool fallback = range[0] == range[3] && range[1] == range[4] && range[2] == range[5];
    for (double value : range) fallback |= !(std::abs(value) < 1e100);
    if (!fallback) { out.resolved = true; return out; }
    out.used_fallback = true;
    if (!c.record_present) { out.reason = "default_range_scale_record_presence_unknown"; return out; }
    double scale = 100000;
    if (*c.record_present) {
        if (!c.value_28 || !c.code_30 || !c.code_68) {
            out.reason = "default_range_scale_base_or_codes_unknown"; return out;
        }
        scale = *c.value_28;
        if (*c.code_30 == *c.code_68) {
            bool use_ratio = true;
            for (const auto &value : {c.value_38,c.value_40,c.value_70,c.value_78}) {
                if (!value) { out.reason = "default_range_scale_ratio_field_unknown"; return out; }
                if (!(*value > 0)) { use_ratio = false; break; }
            }
            if (use_ratio) scale = ((*c.value_38 * scale) * *c.value_78) / (*c.value_70 * *c.value_40);
        }
    }
    if (!std::isfinite(scale)) { out.reason = "nonfinite_default_range_scale"; return out; }
    const double opposite = scale * -1.0;
    const double low = scale >= opposite ? opposite : scale;
    const double high = scale >= opposite ? scale : opposite;
    out.range = {low,low,low,high,high,high}; out.resolved = true; return out;
}
NativeDefaultViewTableResult project_native_default_view_table(const NativeDefaultViewTableInput &in) {
    NativeDefaultViewTableResult out;
    for (unsigned i=0;i<8;++i) {
        if (in.slots[i] == NativeViewSlotPresence::Unknown) { out.reason = "default_table_slot_presence_unknown"; return out; }
        if (in.slots[i] == NativeViewSlotPresence::Present) out.preserved_slot_mask |= std::uint8_t(1u<<i);
    }
    if (out.preserved_slot_mask == 255) { out.resolved = true; return out; }
    if (!in.model_present) { out.reason = "default_table_model_presence_unknown"; return out; }
    if (!*in.model_present) {
        if (!out.preserved_slot_mask) { out.reason = "empty_default_table_requires_nonnull_model"; return out; }
        unsigned first=0; while (!(out.preserved_slot_mask & (1u<<first))) ++first;
        if (in.layouts[first] != NativeViewSlotPresence::Present) {
            out.reason = in.layouts[first] == NativeViewSlotPresence::Absent ?
                "default_table_copy_source_layout_absent" : "default_table_copy_source_layout_unknown";
            return out;
        }
        for (unsigned i=0;i<8;++i) if (in.slots[i] == NativeViewSlotPresence::Absent) {
            auto &slot=out.slots[i]; slot.action=NativeDefaultTableSlotAction::Copy;
            slot.copy_source_slot=first; slot.copy_clear_flags_10=0x84;
            out.copied_slot_mask |= std::uint8_t(1u<<i);
        }
        out.resolved=true; return out;
    }
    if (!in.combined_model_range) { out.reason="combined_model_range_not_established"; return out; }
    out.range=select_native_default_view_range(*in.combined_model_range,in.fallback_scale);
    if (!out.range->resolved) { out.reason=out.range->reason; return out; }
    if (!in.model_query_68) { out.reason="default_table_model_query_68_unknown"; return out; }
    if (!in.suppress_selection) { out.reason="default_table_selection_mode_unknown"; return out; }
    const std::array<std::int8_t,8> presets{1,7,5,4,1,8,6,3};
    for (unsigned i=0;i<8;++i) if (in.slots[i] == NativeViewSlotPresence::Absent) {
        auto &slot=out.slots[i]; slot.action=NativeDefaultTableSlotAction::Create;
        slot.viewport={2,2,998,798}; slot.orientation_preset=1;
        if (*in.model_query_68) {
            slot.viewport={(i&1)?501:2,(i&2)?401:2,(i&1)?998:499,(i&2)?798:399};
            slot.orientation_preset=presets[i];
        }
        const auto &v=slot.viewport;
        slot.normalized_layout={v[0]/1000.0,v[1]/800.0,
                                (v[2]-v[0])/1000.0+v[0]/1000.0,
                                (v[3]-v[1])/800.0+v[1]/800.0};
        slot.layout_words={std::uint16_t(v[0]),std::uint16_t(v[1]),std::uint16_t(v[2]),std::uint16_t(v[3]),1,std::uint16_t(i>>2),0};
        NativeDefaultViewInput view;
        view.model_present=true; view.directory=in.directory;
        view.model_query_68=in.model_query_68; view.model_record_value_1c=in.model_record_value_1c;
        view.range=out.range->range; view.viewport=slot.viewport; view.slot=i;
        view.orientation_preset=slot.orientation_preset;
        view.selected=!*in.suppress_selection && (*in.model_query_68 ? i<4 : i==0);
        slot.created_view=project_native_default_view(view);
        if (!slot.created_view->resolved) { out.reason=slot.created_view->reason; return out; }
        out.created_slot_mask |= std::uint8_t(1u<<i);
    }
    out.resolved=true; return out;
}
} // namespace p3d
