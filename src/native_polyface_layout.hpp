#pragma once
#include "native_polyface_triangulate.hpp"
namespace p3d::swept_detail {
enum NativePolyfacePool : std::size_t {
    native_point_pool,
    native_parameter_pool,
    native_normal_pool,
    native_double_color_pool,
    native_float_color_pool,
    native_integer_color_pool,
    native_color_table_pool,
    native_face_data_pool,
    native_polyface_pool_count
};
struct NativePolyfacePoolLayout {
    std::size_t count = 0;
    std::uint32_t structs_per_row = 0;
    bool active = false;
};
// Runtime layout metadata and index buffers, not a binary file-layout struct.
// Pool values remain owned by the caller; these operations never rewrite them.
struct NativePolyfaceLayout {
    NativePolyfaceIndexState indices;
    std::array<std::uint32_t, polyface_channel_count> index_structs_per_row{};
    std::array<NativePolyfacePoolLayout, native_polyface_pool_count> pools{};
    std::uint32_t mesh_style = 1, num_per_face = 0, num_per_row = 0;
};
struct NativePolyfaceLayoutResult {
    NativePolyfaceLayout output;
    bool native_succeeded = false, complete = false;
    Json report;
};
// Native styles 1/3/4/5/6. Completeness concerns layout conversion and omitted
// partial source rows only; it does not validate references or polygon geometry.
NativePolyfaceLayoutResult convert_native_polyface_layout(const NativePolyfaceLayout &,
                                                          TubeBudget &);
// Separate original AvailableData step: flags reflect buffer presence, not
// valid references, matching lengths or geometric validity. No indices added.
NativePolyfaceLayoutResult activate_native_polyface_layout(const NativePolyfaceLayout &,
                                                           TubeBudget &);
} // namespace p3d::swept_detail
