#pragma once
#include <p3d/view_frame.hpp>
#include <vector>
namespace p3d {
// Current runtime directory entries, in original order. These fields cannot
// be inferred from a source model ID alone or an unrefreshed persisted list.
struct NativeViewDirectoryEntry {
    std::int32_t model_id = 0;
    std::int32_t value_50 = 0;
    std::uint8_t state_54 = 0, excluded_55 = 0, state_59 = 0;
};
struct NativeViewDirectoryContext {
    std::optional<bool> owning_file_present;
    std::optional<std::int32_t> model_id;
    std::vector<NativeViewDirectoryEntry> entries;
    bool entries_complete = false;
};
struct NativeViewDirectoryResult {
    std::optional<bool> preserve_spatial_orientation;
    std::optional<std::size_t> entry_index;
    std::string reason;
};
// The first nonexcluded matching entry determines the predicate. A false
// result does not continue searching for a later duplicate with true flags.
NativeViewDirectoryResult evaluate_native_view_directory(const NativeViewDirectoryContext &context);

struct NativeDefaultViewInput {
    std::optional<bool> model_present;
    NativeViewDirectoryContext directory;
    // Results of the model's virtual +0x68 and virtual +0xc0 record +0x1c.
    // Only the ordinary directory path needs these; true +0x68 skips +0xc0.
    std::optional<bool> model_query_68;
    std::optional<std::int32_t> model_record_value_1c;
    std::array<double, 6> range{};
    std::optional<std::array<std::int32_t, 4>> viewport;
    std::int32_t slot = 0;
    std::int8_t orientation_preset = 1;
    bool selected = false;
    bool state_188 = false;
};
struct NativeDefaultViewResult {
    bool resolved = false;
    std::string reason;
    NativeViewDirectoryResult directory;
    NativeViewFrameResult frame;
    std::array<std::uint32_t, 3> flags{};
    std::array<double, 4> values_a8_to_c0{};
    bool state_188 = false;
};
// Scalar/frame projection of the newly allocated view from R1.18 4b8050.
// The original preset matrix is cleared by 4b69c0 before 4b7550 consumes it.
// Requires a known nonnull model and an already obtained range. Does not
// create an object, acquire model bounds, or manage resources/host/GUI state.
// Consume scalar/frame values only when resolved is true.
NativeDefaultViewResult project_native_default_view(const NativeDefaultViewInput &input);
} // namespace p3d
