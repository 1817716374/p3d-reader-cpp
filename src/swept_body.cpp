#include <p3d/swept_body.hpp>
#include "native_tube_facet_caps.hpp"
#include "native_surface_boundary.hpp"
#include <limits>
#include <unordered_map>

namespace p3d {
const SweptBodyFace *
SweptBodyResult::find_face(const std::array<std::int64_t, 3> &id) const noexcept {
    if (status != "reconstructed" || id[2] != 0 || id[1] < 0 ||
        static_cast<std::uint64_t>(id[1]) > std::numeric_limits<std::size_t>::max())
        return nullptr;
    const auto index = static_cast<std::size_t>(id[1]);
    std::size_t position;
    if (id[0] == -1) {
        if (index >= caps.size())
            return nullptr;
        position = index;
    } else if (id[0] == 0) {
        if (caps.size() > faces.size() || index >= faces.size() - caps.size())
            return nullptr;
        position = caps.size() + index;
    } else
        return nullptr;
    return position < faces.size() && faces[position].indices == id ? &faces[position] : nullptr;
}

SweptBodyResult reconstruct_bgfb_swept_body(const Json &table, const SweptBodyOptions &options) {
    SweptBodyResult out;
    out.source = table;
    swept_detail::TubeBudget budget{options.max_control_points, options.max_work, 0};
    out.report = {{"scope", "grouped_swept_surfaces_and_cap_regions"},
                  {"native_result", nullptr},
                  {"mesh_status", "not_evaluated"}};
    try {
        require(table.is_object() && table.value("_type", Json()) == "P3DSweptBody",
                "swept body source type");
        require(options.max_control_points > 0 && options.max_control_points <= UINT32_MAX &&
                    options.max_work > 0 && options.max_faces > 0,
                "swept body resource limits must be positive; control limit must fit uint32");
        const auto &profile = table.at("profile"), &path = table.at("path");
        const bool capped = table.value("capped", false);
        auto native =
            swept_detail::prepare_swept_tube_facets_with_caps(profile, path, capped, budget);
        out.report["generation"] = std::move(native.sides.generation.report);
        out.report["preparation"] = std::move(native.sides.generation.preparation.report);
        out.report["classification"] = std::move(native.sides.classification.report);
        out.report["assembly"] = std::move(native.sides.report);
        out.report["cap_selection"] = std::move(native.report);
        out.report["cap_construction"] = std::move(native.attempted.report);
        std::size_t count = native.caps.size();
        require(count <= options.max_faces, "swept body face budget");
        require(native.sides.groups.size() <= options.max_faces, "swept body group budget");
        for (const auto &group : native.sides.groups) {
            require(group.size() <= options.max_faces, "swept body member budget");
            for (const auto &member : group) {
                require(member.size() <= options.max_faces - count, "swept body face budget");
                count += member.size();
            }
        }
        // Move each referenced working object once. Repeated native references
        // retain identity; equality of the geometric payload is never examined.
        for (std::size_t g = 0; g < native.sides.groups.size(); ++g) {
            curve_detail::BezierWork{budget.work, budget.max_work}.charge(1);
            auto &group = out.groups.emplace_back();
            for (std::size_t m = 0; m < native.sides.groups[g].size(); ++m) {
                curve_detail::BezierWork{budget.work, budget.max_work}.charge(1);
                auto &member = group.emplace_back();
                std::unordered_map<std::size_t, std::size_t> retained;
                auto &working = native.sides.generation.groups.at(g).at(m);
                for (const auto i : native.sides.groups[g][m]) {
                    curve_detail::BezierWork{budget.work, budget.max_work}.charge(1);
                    auto previous = retained.find(i);
                    if (previous == retained.end()) {
                        auto &surface = working.surfaces.at(i);
                        const auto index = out.surfaces.size();
                        out.surfaces.push_back({std::move(surface.geometry),
                                                std::move(surface.boundaries),
                                                std::move(surface.pcurves),
                                                {g, m, i},
                                                working.source_member});
                        retained.emplace(i, index);
                        member.push_back(index);
                    } else
                        member.push_back(previous->second);
                }
            }
        }
        if (native.success) {
            out.caps = std::move(native.caps);
            const auto &ids = native.face_indices.at("face_indices");
            const auto &locations = native.face_indices.at("side_locations");
            require(ids.size() == count && locations.size() == count - out.caps.size(),
                    "swept body native face enumeration mismatch");
            out.faces.reserve(count);
            for (std::size_t i = 0; i < ids.size(); ++i) {
                curve_detail::BezierWork{budget.work, budget.max_work}.charge(1);
                SweptBodyFace face;
                face.indices = ids[i].get<std::array<std::int64_t, 3>>();
                if (i < out.caps.size()) {
                    face.kind = SweptBodyFaceKind::cap;
                    face.index = i;
                } else {
                    const auto &where = locations[i - out.caps.size()];
                    std::array<std::size_t, 3> location{where.at("group"), where.at("member"),
                                                        where.at("patch")};
                    face.location = location;
                    face.index = out.groups.at(location[0]).at(location[1]).at(location[2]);
                }
                out.faces.push_back(std::move(face));
            }
        }
        out.status = native.success ? "reconstructed" : "native_failure";
        out.report["native_result"] = native.success;
        out.report["surface_count"] = out.surfaces.size();
        out.report["face_count"] = out.faces.size();
    } catch (const std::bad_alloc &) {
        throw;
    } catch (const std::exception &e) {
        out.surfaces.clear();
        out.groups.clear();
        out.caps.clear();
        out.faces.clear();
        out.status = "not_reconstructed";
        out.report["native_result"] = nullptr;
        out.report["reason"] = e.what();
    }
    out.report["work_used"] = budget.work;
    return out;
}
SweptBodyBoundaryResult
extract_swept_body_surface_boundary(const SweptBodySurface &source,
                                    const SweptBodyBoundaryOptions &options) {
    SweptBodyBoundaryResult out;
    swept_detail::TubeBudget budget{options.max_control_points, options.max_work, 0};
    try {
        require(options.max_work > 0, "swept surface boundary work limit must be positive");
        auto boundary = swept_detail::native_surface_boundary(
            BsplineSurface::from_bgfb(source.geometry), source.boundary_points,
            options.include_outer, budget, options.max_curves);
        out.curves = std::move(boundary.curves);
        out.report = std::move(boundary.report);
        out.status = out.curves.empty() ? "native_empty" : "extracted";
    } catch (const std::bad_alloc &) {
        throw;
    } catch (const std::exception &e) {
        out.curves.clear();
        out.status = "not_extracted";
        out.report["reason"] = e.what();
    }
    out.report["work_used"] = budget.work;
    return out;
}
} // namespace p3d
