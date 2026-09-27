#pragma once
#include "native_tube_path_placement.hpp"
#include "native_bezier.hpp"
namespace p3d::swept_detail {
inline BsplineCurve *facet_working_path(TubeFacetPathBranches &p, TubePathBranch &branch) {
    require(!branch.empty_curve_object, "native facet path has no initialized controls");
    if (branch.reused_curve_index) {
        require(*branch.reused_curve_index < p.path.selection.curves.size(),
                "native facet path source index");
        return &p.path.selection.curves[*branch.reused_curve_index];
    }
    return branch.constructed ? &*branch.constructed : nullptr;
}
inline void update_facet_working_path(BsplineCurve *path,
                                      const std::shared_ptr<const BsplineCurve> &updated,
                                      TubeBudget &b) {
    if (!path || !updated)
        return;
    curve_detail::BezierWork work{b.work, b.max_work};
    work.charge(updated->knots().size());
    for (unsigned i = 0; i < 4; ++i)
        work.charge(updated->poles().size());
    *path = *updated;
}
} // namespace p3d::swept_detail
