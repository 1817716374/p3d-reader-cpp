#include "native_tube_mesh_trim_facets.hpp"
#include "native_bezier_support.hpp"
namespace p3d::swept_detail {
namespace {
std::int64_t declared_count(const TubeMeshTrimColumn &c) {
    return std::int64_t(c.last_interior) - c.first_interior + 3;
}
void validate(const TubeMeshTrimPlan &plan, TubeBudget &budget) {
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    require(plan.success && plan.columns.size() >= 2 && plan.columns.size() <= INT32_MAX &&
                plan.columns.size() <= budget.max_control_points &&
                plan.vertex_count <= budget.max_control_points && plan.vertex_count <= INT32_MAX,
            "native trimmed facet column plan");
    work.charge(plan.columns.size());
    std::size_t count = 0;
    for (const auto &c : plan.columns) {
        const auto n = declared_count(c);
        require(c.first_interior >= 0 && c.first_interior < INT32_MAX && c.last_interior >= -1 &&
                    c.last_interior <= INT32_MAX - 2 && c.count == (n > 0 ? std::size_t(n) : 0) &&
                    c.offset == count && c.count <= budget.max_control_points - count,
                "native trimmed facet column extent");
        count += c.count;
    }
    require(count == plan.vertex_count, "native trimmed facet vertex count");
}
std::int64_t native_int(std::int64_t x) {
    require(x >= INT32_MIN && x <= INT32_MAX, "native trimmed facet signed integer overflow");
    return x;
}
} // namespace
std::vector<Point2> evaluate_tube_mesh_trim_parameters(const TubeMeshTrimPlan &plan,
                                                       const std::vector<double> &v,
                                                       std::size_t patch, std::size_t patches,
                                                       TubeBudget &budget) {
    using curve_detail::bezier_support::finite;
    validate(plan, budget);
    require(patches && patches <= INT32_MAX && patch < patches && v.size() >= 2 &&
                v.size() <= budget.max_control_points,
            "native trimmed parameter patch or sample extent");
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    work.charge(plan.vertex_count);
    std::vector<Point2> out;
    out.reserve(plan.vertex_count);
    const auto offset = double(patch) / double(patches);
    for (const auto &c : plan.columns) {
        const auto first = std::int64_t(c.first_interior) - 1;
        for (std::size_t j = 0; j < c.count; ++j) {
            const auto row = first + std::int64_t(j);
            double f = c.lower;
            if (j != 0) {
                require(row >= 0 && std::size_t(row) < v.size(),
                        "native trimmed attribute sample index");
                f = v[std::size_t(row)];
            }
            out.push_back({finite(c.source_u), finite(offset + finite(f) / double(patches))});
        }
    }
    return out;
}
TubeMeshTrimFacetPlan prepare_tube_mesh_trim_facets(const TubeMeshTrimPlan &plan,
                                                    TubeBudget &budget) {
    validate(plan, budget);
    const curve_detail::BezierWork work{budget.work, budget.max_work};
    TubeMeshTrimFacetPlan out;
    std::int64_t left_offset = 0, right_offset = 0;
    std::size_t corners = 0, invalid_indices = 0, invalid_correspondences = 0;
    for (std::size_t i = 0; i + 1 < plan.columns.size(); ++i) {
        const auto &a = plan.columns[i], &b = plan.columns[i + 1];
        // Native arithmetic accumulates the signed declared count even for
        // empty/inverted columns. Do not substitute the clamped vertex count.
        if (i)
            left_offset = native_int(left_offset + declared_count(plan.columns[i - 1]));
        right_offset = native_int(right_offset + declared_count(a));
        const auto low = std::int64_t(std::max(a.first_interior, b.first_interior));
        const auto high = std::int64_t(std::min(a.last_interior, b.last_interior));
        auto begin = [&](TubeMeshTrimFacetKind kind) {
            work.charge(1);
            require(out.facets.size() < budget.max_control_points,
                    "native trimmed facet count budget");
            out.facets.push_back({i, kind, {}, true, true});
        };
        auto add = [&](bool right, std::int64_t row) {
            work.charge(1);
            require(corners < budget.max_control_points, "native trimmed facet corner budget");
            ++corners;
            const auto &c = right ? b : a;
            auto local = native_int(native_int(row - c.first_interior) + 2);
            const auto index = native_int(local + (right ? right_offset : left_offset));
            auto &f = out.facets.back();
            const bool in_range = index > 0 && std::uint64_t(index) <= plan.vertex_count;
            // A global index can land in a different column while still being
            // inside the flat array; report that separately instead of welding.
            const bool corresponds = local > 0 && std::uint64_t(local) <= c.count &&
                                     index == std::int64_t(c.offset) + local;
            invalid_indices += !in_range;
            invalid_correspondences += !corresponds;
            f.indices_in_range &= in_range;
            f.column_correspondence_valid &= corresponds;
            out.indices_in_range &= in_range;
            out.column_correspondence_valid &= corresponds;
            f.indices.push_back(std::int32_t(index));
        };
        // 88550: shared rows first, then lower and upper boundary polygons.
        for (auto row = low - 1; row < high + 1; ++row) {
            begin(TubeMeshTrimFacetKind::common_rows);
            add(false, row);
            add(true, row);
            add(true, row + 1);
            add(false, row + 1);
        }
        if (a.first_interior != b.first_interior) {
            begin(TubeMeshTrimFacetKind::lower_boundary);
            for (auto row = low - 1; row > std::int64_t(a.first_interior) - 2; --row)
                add(false, row);
            for (auto row = std::int64_t(b.first_interior) - 1; row < low; ++row)
                add(true, row);
        }
        if (a.last_interior != b.last_interior) {
            begin(TubeMeshTrimFacetKind::upper_boundary);
            for (auto row = std::int64_t(a.last_interior) + 1; row > high; --row)
                add(false, row);
            for (auto row = high + 1; row < std::int64_t(b.last_interior) + 2; ++row)
                add(true, row);
        }
    }
    out.report = {{"scope", "native_swept_trimmed_facet_arguments"},
                  {"status", "prepared"},
                  {"facets", out.facets.size()},
                  {"corners", corners},
                  {"invalid_indices", invalid_indices},
                  {"invalid_column_correspondences", invalid_correspondences},
                  {"triangulated", false},
                  {"shared_coordinates_applied", false},
                  {"work_used", budget.work}};
    return out;
}
} // namespace p3d::swept_detail
