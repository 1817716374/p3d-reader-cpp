#pragma once
#include "native_vu_graph.hpp"
namespace p3d::swept_detail {
// Native merge type: 0 removes duplicate pairs, 2002 keeps one; all other values
// retain duplicate edges as null-face bundles. Triangulation uses 1.
Json connect_native_vu_vertices(NativeVuGraph &, double tolerance, int merge_type,
                                std::uint32_t scratch_mask, TubeBudget &);
} // namespace p3d::swept_detail
