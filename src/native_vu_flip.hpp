#pragma once
#include "native_vu_graph.hpp"
namespace p3d::swept_detail {
// Fresh projected-loop graph: zero period/scale and no label inheritance.
// The predicate alone does not apply the fixed-edge mask filter.
bool native_vu_quadratic_flip_test(const NativeVuGraph &, std::size_t, TubeBudget &);
// 144500/1427d0: native fixed-edge mask, LIFO candidate order and iteration cap.
// No new nodes; a successful flip copies XYZ but retains both node payloads.
// Transactional on resource/numerical failure. Does not produce source indices.
Json flip_native_vu_triangles(NativeVuGraph &, TubeBudget &);
} // namespace p3d::swept_detail
