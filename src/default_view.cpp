#include <p3d/default_view.hpp>
namespace p3d {
NativeViewDirectoryResult evaluate_native_view_directory(const NativeViewDirectoryContext &c) {
    NativeViewDirectoryResult out;
    if (!c.owning_file_present) { out.reason = "view_owning_file_presence_unknown"; return out; }
    if (!*c.owning_file_present) { out.preserve_spatial_orientation = false; return out; }
    if (!c.model_id) { out.reason = "bound_view_model_id_unknown"; return out; }
    for (std::size_t i = 0; i < c.entries.size(); ++i) {
        const auto &entry = c.entries[i];
        if (entry.excluded_55 || entry.model_id != *c.model_id) continue;
        out.entry_index = i;
        out.preserve_spatial_orientation = entry.state_54 != 0 &&
                                          (entry.state_59 != 0 || entry.value_50 != 1);
        return out;
    }
    if (!c.entries_complete) { out.reason = "view_model_directory_incomplete"; return out; }
    out.preserve_spatial_orientation = false; return out;
}
NativeDefaultViewResult project_native_default_view(const NativeDefaultViewInput &in) {
    NativeDefaultViewResult out;
    if (!in.model_present) { out.reason = "default_view_model_presence_unknown"; return out; }
    if (!*in.model_present) { out.reason = "default_view_requires_nonnull_model"; return out; }
    out.directory = evaluate_native_view_directory(in.directory);
    if (!out.directory.preserve_spatial_orientation) { out.reason = out.directory.reason; return out; }
    out.flags = {0xb0d8u, 0xc0050u, 0u};
    if (*out.directory.preserve_spatial_orientation) {
        out.flags[0] |= 0x50000u;
    } else {
        if (!in.model_query_68) { out.reason = "default_view_model_query_68_unknown"; return out; }
        if (!*in.model_query_68) {
            if (!in.model_record_value_1c) { out.reason = "default_view_model_record_value_1c_unknown"; return out; }
            if (*in.model_record_value_1c != 1) out.flags[0] |= 0x200u;
        }
        out.values_a8_to_c0[0] = -1;
    }
    out.flags[0] = (out.flags[0] & ~0x80u) | (in.selected ? 0x80u : 0u);
    out.flags[1] = (out.flags[1] & ~0x8000u) | (in.state_188 ? 0x8000u : 0u);
    out.state_188 = in.state_188;
    NativeViewFrameInput frame;
    frame.range = in.range; frame.viewport = in.viewport; frame.slot = in.slot;
    // 4b8050 writes the preset, then 4b69c0 clears [view+0x10,view+0xc8).
    // This zero matrix is passed by alias to 4b7550 for every int8 preset.
    frame.orientation = std::array<double, 9>{};
    frame.preserve_spatial_orientation = out.directory.preserve_spatial_orientation;
    out.frame = project_native_view_frame(frame);
    if (!out.frame.resolved) { out.reason = out.frame.reason; return out; }
    out.resolved = true; return out;
}
} // namespace p3d
