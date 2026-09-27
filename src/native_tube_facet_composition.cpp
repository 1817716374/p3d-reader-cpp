#include "native_tube_facet_composition.hpp"
#include "native_tube_orientation.hpp"
#include "native_tube_path_placement.hpp"
#include "native_curve_affine.hpp"
#include "native_curve_conversion.hpp"
#include "native_pcurve_points.hpp"
#include "native_bezier.hpp"
#include <map>
namespace p3d::swept_detail {
namespace {
void charge(TubeBudget &b, std::size_t n) {
    curve_detail::BezierWork{b.work, b.max_work}.charge(n);
}
double finite(double v) {
    require(std::isfinite(v), "native facet composition nonfinite arithmetic");
    return v;
}
Point3 negative(Point3 p) {
    for (auto &v : p)
        v = finite(-v);
    return p;
}
Point3 unit(Point3 p) {
    const double length = finite(std::sqrt((p[0] * p[0] + p[1] * p[1]) + p[2] * p[2]));
    if (!length)
        return {1, 0, 0};
    const double inverse = finite(1 / length);
    for (auto &v : p)
        v = finite(v * inverse);
    return p;
}
detail::NativePCurvePointTangent query(const BsplineCurve &c, double f, TubeBudget &b) {
    require(c.order() <= 26 && c.poles().size() <= b.max_control_points,
            "native facet composition curve limits");
    charge(b, c.knots().size() + 8 * std::size_t(c.order()) * c.order());
    auto q = detail::pcurve_point_tangent(c, f);
    const auto domain = c.knot_domain();
    const double span = finite(domain[1] - domain[0]);
    for (auto &v : q.tangent)
        v = finite(v * span);
    return q;
}
template <class T> void assign(std::shared_ptr<TubeFacetSeamStorage<T>> &p, const T &value) {
    if (!p)
        p = std::make_shared<TubeFacetSeamStorage<T>>(TubeFacetSeamStorage<T>{value, true});
    else {
        require(p->alive, "native facet composition writes freed seam storage");
        p->value = value;
    }
}
TubeFacetSeamReferences references(const TubeFacetSeam &s) {
    TubeFacetSeamReferences out;
    out.classifier = s.classifier;
    if (s.incoming)
        assign(out.incoming, *s.incoming);
    if (s.outgoing)
        assign(out.outgoing, *s.outgoing);
    if (s.plane)
        assign(out.plane, *s.plane);
    return out;
}
TubeFacetSeam snapshot(const TubeFacetSeamReferences &r) {
    TubeFacetSeam out;
    out.classifier = r.classifier;
    if (r.incoming && r.incoming->alive)
        out.incoming = r.incoming->value;
    if (r.outgoing && r.outgoing->alive)
        out.outgoing = r.outgoing->value;
    if (r.plane && r.plane->alive)
        out.plane = r.plane->value;
    return out;
}
void negate(TubeFacetSeamReferences &s) {
    if (s.incoming) {
        require(s.incoming->alive, "native freed seam tangent");
        s.incoming->value = negative(s.incoming->value);
    }
    if (s.outgoing) {
        require(s.outgoing->alive, "native freed seam tangent");
        s.outgoing->value = negative(s.outgoing->value);
    }
    if (s.plane) {
        require(s.plane->alive, "native freed seam plane");
        s.plane->value[1] = negative(s.plane->value[1]);
    }
}
void plane(TubeFacetSeamReferences &r, const Point3 &point) {
    if (!r.incoming || !r.outgoing)
        return;
    require(r.incoming->alive && r.outgoing->alive && (!r.plane || r.plane->alive),
            "native facet plane references freed seam storage");
    auto s = snapshot(r);
    update_native_tube_facet_plane(s, point);
    r.classifier = s.classifier;
    if (!s.incoming) {
        r.incoming->alive = false;
        r.outgoing->alive = false;
        r.incoming.reset();
        r.outgoing.reset();
    } else if (s.plane)
        assign(r.plane, *s.plane);
}
Json initial_report() {
    return {{"scope", "native_facet_branch_composition"},
            {"status", "native_failure"},
            {"seams_applied", false},
            {"pending", {"adjacent_surface_seams", "chain_finalization"}}};
}
void account(const std::vector<TubeFacetNode> &nodes, std::size_t &total, TubeBudget &b) {
    charge(b, nodes.size());
    for (const auto &node : nodes) {
        const auto &s = node.surface;
        require(s.is_object() && s.contains("poles") && s["poles"].is_array() &&
                    s["poles"].size() % 3 == 0,
                "native facet composition surface table");
        const auto count = s["poles"].size() / 3;
        require(count <= b.max_control_points - total,
                "native facet composition cumulative control budget");
        total += count;
        for (unsigned i = 0; i < 8; ++i)
            charge(b, count);
        for (const auto &p : {node.end_seam.incoming, node.end_seam.outgoing})
            if (p)
                for (double v : *p)
                    finite(v);
        if (node.end_seam.plane)
            for (const auto &p : *node.end_seam.plane)
                for (double v : p)
                    finite(v);
        BsplineSurface::from_bgfb(s);
    }
}
} // namespace
TubeFacetComposition compose_tube_facet_nodes(const std::vector<TubeFacetNode> &prefix,
                                              const std::vector<TubeFacetNode> &suffix,
                                              const BsplineCurve *prefix_path,
                                              const BsplineCurve *suffix_path, TubeBudget &b) {
    TubeFacetComposition out;
    out.report = initial_report();
    require((prefix_path != nullptr) == !prefix.empty() &&
                (suffix_path != nullptr) == !suffix.empty(),
            "native facet composition path/chain presence mismatch");
    std::size_t controls = 0;
    account(prefix, controls, b);
    account(suffix, controls, b);
    if (prefix.empty() && suffix.empty()) {
        out.report["reason"] = "absent_branches";
        return out;
    }
    bool joint = false;
    Point3 joint_in{}, joint_out{}, joint_point{};
    if (prefix_path && suffix_path) {
        const auto first = query(*prefix_path, 0, b), second = query(*suffix_path, 0, b);
        joint = !native_path_vectors_parallel(first.tangent, second.tangent);
        joint_in = negative(first.tangent);
        joint_out = second.tangent;
        joint_point = second.value.point; // second query overwrites the shared point slot
    }
    out.report["joint_requested"] = joint;
    Json sources = Json::array(), reversals = Json::array();
    for (auto it = prefix.rbegin(); it != prefix.rend(); ++it)
        out.seams.push_back(references(it->end_seam));
    for (const auto &node : suffix)
        out.seams.push_back(references(node.end_seam));
    if (!prefix.empty()) {
        auto saved = prefix.back().end_seam;
        const bool pair = saved.incoming && saved.outgoing;
        if (!pair) {
            saved.incoming.reset();
            saved.outgoing.reset();
        }
        const bool closing_seam = pair || saved.plane;
        if (closing_seam && joint) {
            out.seams.clear();
            out.report["reason"] = "prefix_closing_seam_conflicts_with_branch_joint";
            out.report["work_used"] = b.work;
            return out;
        }
        out.seams.front().incoming.reset();
        out.seams.front().outgoing.reset();
        out.seams.front().plane.reset();
        out.nodes.reserve(prefix.size() + suffix.size());
        for (std::size_t r = prefix.size(); r > 0; --r) {
            const std::size_t i = r - 1;
            auto surface =
                reverse_tube_surface_both(BsplineSurface::from_bgfb(prefix[i].surface), b);
            auto &seam = out.seams[prefix.size() - r];
            if (i) {
                const auto next = out.seams[prefix.size() - r + 1];
                seam.incoming = next.outgoing;
                seam.outgoing = next.incoming;
                seam.plane = next.plane;
                seam.classifier = next.classifier;
                negate(seam);
            } else if (closing_seam) {
                seam.classifier = saved.classifier;
                if (pair) {
                    assign(seam.incoming, *saved.outgoing);
                    assign(seam.outgoing, *saved.incoming);
                }
                if (saved.plane)
                    assign(seam.plane, *saved.plane);
                negate(seam);
            } else if (joint) {
                seam.classifier = -2;
                assign(seam.incoming, unit(joint_in));
                assign(seam.outgoing, unit(joint_out));
                plane(seam, joint_point);
            } else {
                seam.classifier = 2;
                seam.incoming.reset();
                seam.outgoing.reset();
                seam.plane.reset();
            }
            out.nodes.push_back({std::move(surface.surface), {}});
            reversals.push_back(std::move(surface.report));
            sources.push_back({{"branch", "prefix"}, {"node", i}});
        }
    }
    const auto suffix_begin = out.nodes.size();
    out.nodes.insert(out.nodes.end(), suffix.begin(), suffix.end());
    for (std::size_t i = 0; i < suffix.size(); ++i)
        sources.push_back({{"branch", "suffix"}, {"node", i}});
    if (prefix_path && suffix_path) {
        const auto first = query(*prefix_path, 1, b), second = query(*suffix_path, 1, b);
        const auto outgoing = negative(first.tangent);
        auto &seam = out.seams.back();
        seam.classifier = -2;
        const bool closed =
            curve_detail::endpoint_pair_closed(first.value.point, second.value.point);
        const bool needed = closed && !native_path_vectors_parallel(outgoing, second.tangent);
        out.report["ends_near"] = closed;
        out.report["end_seam_requested"] = needed;
        if (needed) {
            // This branch unconditionally allocates a new pair; only the plane
            // reuses its previous storage.
            seam.incoming.reset();
            seam.outgoing.reset();
            assign(seam.incoming, unit(second.tangent));
            assign(seam.outgoing, unit(outgoing));
            plane(seam, first.value.point);
        } else {
            // Native code changes only the classifier here, not existing data.
            seam.classifier = 2;
        }
    }
    if (prefix_path)
        out.working_prefix_path = *prefix_path;
    if (suffix_path)
        out.working_suffix_path = *suffix_path;
    Json storage = Json::array(), identities = Json::array();
    std::map<const void *, std::size_t> ids;
    auto identity = [&](const auto &cell) -> Json {
        if (!cell)
            return nullptr;
        const auto inserted = ids.emplace(cell.get(), ids.size());
        if (inserted.second)
            storage.push_back({{"alive", cell->alive}, {"value", cell->value}});
        return inserted.first->second;
    };
    bool evaluable = true;
    for (std::size_t i = 0; i < out.seams.size(); ++i) {
        const auto &r = out.seams[i];
        out.nodes[i].end_seam = snapshot(r);
        identities.push_back({{"incoming", identity(r.incoming)},
                              {"outgoing", identity(r.outgoing)},
                              {"plane", identity(r.plane)}});
        evaluable &= (!r.incoming || r.incoming->alive) && (!r.outgoing || r.outgoing->alive) &&
                     (!r.plane || r.plane->alive);
    }
    out.report["seam_storage"] = std::move(storage);
    out.report["seam_references"] = std::move(identities);
    out.report["evaluable_seam_references"] = evaluable;
    out.prepared = true;
    out.report["status"] = "prepared_before_seam_processing";
    out.report["source_nodes"] = std::move(sources);
    out.report["prefix_reversals"] = std::move(reversals);
    out.report["suffix_begin"] = suffix_begin;
    out.report["control_points"] = controls;
    out.report["work_used"] = b.work;
    return out;
}
TubeFacetComposition prepare_tube_facet_composition(const BsplineCurve *prefix_path,
                                                    const BsplineCurve *prefix_section,
                                                    const BsplineCurve *suffix_path,
                                                    const BsplineCurve *suffix_section, bool rigid,
                                                    TubeBudget &b) {
    TubeFacetComposition failed;
    failed.report = initial_report();
    if ((!prefix_path && !suffix_path) || (prefix_path && !prefix_section) ||
        (suffix_path && !suffix_section)) {
        failed.report["reason"] = "missing_path_or_section";
        return failed;
    }
    std::map<const BsplineCurve *, BsplineCurve> working;
    auto get = [&](const BsplineCurve *source) -> BsplineCurve & {
        auto found = working.find(source);
        if (found != working.end())
            return found->second;
        require(source && source->order() <= 26 && source->poles().size() <= b.max_control_points,
                "native facet composition source limits");
        charge(b, source->knots().size() + 4 * source->poles().size());
        return working.emplace(source, *source).first->second;
    };
    TubeFacetChain prefix, suffix;
    Json branches = Json::object();
    auto capture = [&](TubeFacetComposition &result) {
        const std::array<const BsplineCurve *, 4> sources{prefix_path, prefix_section, suffix_path,
                                                          suffix_section};
        std::map<const BsplineCurve *, std::pair<std::size_t, std::shared_ptr<const BsplineCurve>>>
            copies;
        Json references = Json::array();
        for (std::size_t i = 0; i < sources.size(); ++i) {
            if (!sources[i]) {
                references.push_back(nullptr);
                continue;
            }
            auto found = copies.find(sources[i]);
            if (found == copies.end()) {
                const auto updated = working.find(sources[i]);
                const auto &value = updated == working.end() ? *sources[i] : updated->second;
                require(value.poles().size() <= b.max_control_points,
                        "native facet source state control budget");
                charge(b, value.knots().size());
                for (unsigned pass = 0; pass < 8; ++pass)
                    charge(b, value.poles().size());
                found = copies
                            .emplace(sources[i],
                                     std::make_pair(i, std::make_shared<const BsplineCurve>(value)))
                            .first;
            }
            result.working_sources[i] = found->second.second;
            references.push_back(found->second.first);
        }
        result.report["source_reference_indices"] = std::move(references);
        result.report["work_used"] = b.work;
    };
    auto build = [&](const BsplineCurve *path, const BsplineCurve *section, TubeFacetChain &chain,
                     const char *name) {
        if (!path)
            return true;
        auto &path_working = get(path);
        auto &section_working = get(section);
        chain = build_tube_facet_chain(section_working, path_working, rigid, b);
        path_working = curve_detail::with_poles(path_working, chain.working_source_poles);
        branches[name] = std::move(chain.report);
        return chain.success;
    };
    // Builds are deliberately sequential, including all native shared aliases.
    if (!build(prefix_path, prefix_section, prefix, "prefix") ||
        !build(suffix_path, suffix_section, suffix, "suffix")) {
        failed.report["reason"] = "branch_generation_failed";
        failed.report["branches"] = std::move(branches);
        capture(failed);
        return failed;
    }
    auto out = compose_tube_facet_nodes(prefix.nodes, suffix.nodes,
                                        prefix_path ? &get(prefix_path) : nullptr,
                                        suffix_path ? &get(suffix_path) : nullptr, b);
    out.report["branches"] = std::move(branches);
    out.report["shared_path"] = prefix_path && prefix_path == suffix_path;
    capture(out);
    return out;
}
} // namespace p3d::swept_detail
