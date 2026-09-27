#include <p3d/swept_mesh.hpp>
#include "native_tube_mesh_source.hpp"
#include "native_tube_mesh_assembly.hpp"
#include "native_tube_patches.hpp"
namespace p3d {
SweptBodyMeshResult mesh_bgfb_swept_body(const Json &table, const SweptBodyMeshOptions &options) {
    SweptBodyMeshResult out;
    out.source = table;
    out.report = {{"scope", "native_source_sweep_mesh"},
                  {"material_part_mapping_status", "not_evaluated"}};
    swept_detail::TubeBudget budget{options.max_control_points, options.max_work, 0};
    try {
        require(table.is_object() && table.value("_type", "") == "P3DSweptBody",
                "sweep mesh source type");
        require(options.max_control_points > 0 && options.max_control_points <= UINT32_MAX &&
                    options.max_work > 0 && options.max_patches > 0 &&
                    options.max_sample_nodes > 0 && std::isfinite(options.chord_tolerance) &&
                    std::isfinite(options.angle_tolerance),
                "sweep mesh options or resource limits");
        const auto capped = table.value("capped", false);
        auto conditions = swept_detail::tube_mesh_source_conditions(
            table.at("profile"), table.at("path"), capped, budget);
        out.report["source_conditions"] = conditions.report;
        if (!conditions.accepted) {
            out.status = "native_failure";
        } else {
            auto patches = swept_detail::generate_tube_patch_groups(table.at("profile"),
                                                                    table.at("path"), budget);
            out.report["patch_generation"] = patches.report;
            out.report["patch_preparation"] = patches.preparation.report;
            if (!patches.success || patches.groups.empty()) {
                out.status = "native_failure";
            } else {
                require(patches.groups.size() <= options.max_patches, "sweep mesh group budget");
                std::vector<swept_detail::TubeMeshGridPreparation> grids;
                out.report["sampling"] = Json::array();
                std::size_t patch_count = 0;
                bool prepared = true;
                for (auto &g : patches.groups) {
                    require(g.surfaces.size() <= options.max_patches - patch_count,
                            "sweep mesh patch budget");
                    patch_count += g.surfaces.size();
                    SweptBodyPatchGroup group;
                    group.source_curve = g.source_curve;
                    for (auto &s : g.surfaces) {
                        curve_detail::BezierWork{budget.work, budget.max_work}.charge(1);
                        require(s.pcurves.empty(), "native sweep patch has unexpected pcurves");
                        group.patches.push_back({std::move(s.geometry), std::move(s.boundaries)});
                    }
                    auto grid = swept_detail::prepare_tube_mesh_grid(
                        group, conditions.profile_closed, options.chord_tolerance,
                        options.angle_tolerance, options.max_sample_nodes, budget);
                    out.report["sampling"].push_back(grid.report);
                    if (!grid.success) {
                        prepared = false;
                        break;
                    }
                    grids.push_back(std::move(grid));
                }
                out.report["patch_count"] = patch_count;
                if (!prepared) {
                    out.status = "native_failure";
                } else {
                    auto result = swept_detail::assemble_native_tube_mesh_groups(
                        grids,
                        {options.normals, options.parameters, conditions.profile_closed,
                         conditions.path_closed, conditions.cap_eligible, options.parameter_mode},
                        budget);
                    out.report["assembly"] = result.report;
                    out.report["groups"] = Json::array();
                    for (auto &g : result.groups)
                        out.report["groups"].push_back(std::move(g.report));
                    if (!result.complete || !result.finalized) {
                        out.status = "incomplete";
                    } else {
                        auto &m = result.finalized->output;
                        out.points = std::move(m.data.coordinates.points);
                        out.normals = std::move(m.data.coordinates.normals);
                        out.parameters = std::move(m.data.coordinates.parameters);
                        out.point_indices =
                            std::move(m.data.indices.indices[swept_detail::point_channel]);
                        out.normal_indices =
                            std::move(m.data.indices.indices[swept_detail::normal_channel]);
                        out.parameter_indices =
                            std::move(m.data.indices.indices[swept_detail::parameter_channel]);
                        out.face_data_indices =
                            std::move(m.data.indices.indices[swept_detail::face_channel]);
                        for (const auto &f : m.data.face_data) {
                            curve_detail::BezierWork{budget.work, budget.max_work}.charge(1);
                            out.face_data.push_back({f.parameter_distance_range, f.parameter_range,
                                                     f.point_range, f.normal_range, f.source_index,
                                                     f.face_indices});
                        }
                        out.two_sided = m.data.two_sided;
                        out.layout = {{"mesh_style", m.mesh_style},
                                      {"num_per_face", m.data.num_per_face},
                                      {"num_per_row", m.num_per_row},
                                      {"pools", Json::object()},
                                      {"indices", Json::object()}};
                        auto metadata = [](bool active, std::uint32_t rows,
                                           const swept_detail::NativePolyfaceVectorTags &t) {
                            return Json{{"active", active},
                                        {"structs_per_row", rows},
                                        {"num_per_struct", t.num_per_struct},
                                        {"tag", t.tag},
                                        {"index_family", t.index_family},
                                        {"indexed_by", t.indexed_by}};
                        };
                        const std::array<const char *, 8> pools{
                            "points",       "parameters",     "normals",     "double_colors",
                            "float_colors", "integer_colors", "color_table", "face_data"};
                        const std::array<const char *, 5> channels{
                            "points", "parameters", "normals", "colors", "face_data"};
                        for (std::size_t i = 0; i < pools.size(); ++i)
                            out.layout["pools"][pools[i]] =
                                metadata(m.pool_active[i], m.pool_rows[i], m.pool_tags[i]);
                        for (std::size_t i = 0; i < channels.size(); ++i)
                            out.layout["indices"][channels[i]] = metadata(
                                m.data.indices.active[i], m.index_rows[i], m.index_tags[i]);
                        out.layout["edge_chains"] =
                            metadata(m.edge_chain_active, m.edge_chain_rows, m.edge_chain_tags);
                        out.status = "meshed";
                    }
                }
            }
        }
    } catch (const std::bad_alloc &) {
        throw;
    } catch (const std::exception &e) {
        out.points.clear();
        out.normals.clear();
        out.parameters.clear();
        out.point_indices.clear();
        out.normal_indices.clear();
        out.parameter_indices.clear();
        out.face_data_indices.clear();
        out.face_data.clear();
        out.layout = Json::object();
        out.two_sided = false;
        out.status = "not_meshed";
        out.report["reason"] = e.what();
    }
    out.report["work_used"] = budget.work;
    out.report["status"] = out.status;
    return out;
}
} // namespace p3d
