#pragma once
#include "native_vu_graph.hpp"
namespace p3d::swept_detail {
inline constexpr std::uint32_t native_vu_exterior_mask = 2;
// Native area-then-boundary-parity classification. The candidate array retains
// its read cursor across native pool reuse; pass the regularization report's
// candidate_array_read_index, or zero for a newly allocated array.
// Only masks change. Validation/budget failures leave the graph unchanged.
Json mark_native_vu_exterior(NativeVuGraph &, std::size_t candidate_read_index, TubeBudget &);
} // namespace p3d::swept_detail
