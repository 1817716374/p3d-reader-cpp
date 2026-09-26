#include "native_tube_transform.hpp"
#include "native_curve_affine.hpp"
#include <unordered_map>
namespace p3d::swept_detail {
namespace {
void charge(TubeBudget &b, std::size_t n) {
    require(b.work <= b.max_work && n <= b.max_work - b.work,
            "native curve list transform work budget exceeded");
    b.work += n;
}
} // namespace
TubeCurveTransform transform_tube_curve_list(const TubeCurveViews &source, const Matrix4 *m,
                                             bool clone, TubeBudget &b) {
    charge(b, source.size());
    bool skip = true, evaluated = false;
    // P3D's GeTransform stores only three rows. The Matrix4 carrier's last row
    // is not consumed here and does not turn the operation into projective math.
    TubeCurveTransform out;
    if (!clone)
        out.curves = source;
    std::unordered_map<const BsplineCurve *, std::shared_ptr<BsplineCurve>> working;
    std::size_t controls = 0, processed = 0, copies = 0;
    std::optional<std::size_t> failed;
    for (std::size_t i = 0; i < source.size(); ++i) {
        charge(b, 1);
        if (!source[i]) {
            failed = i;
            break;
        }
        if (m && !evaluated) {
            for (unsigned r = 0; r < 3; ++r)
                for (double x : (*m)[r])
                    require(std::isfinite(x), "native curve list transform nonfinite matrix");
            skip = curve_detail::bspline_identity(*m);
            evaluated = true;
        }
        const auto &s = *source[i];
        const auto n = s.poles().size();
        require(n <= b.max_control_points, "native curve list source control budget");
        std::shared_ptr<BsplineCurve> curve;
        if (clone || !skip) {
            auto existing = working.find(&s);
            if (!clone && existing != working.end())
                curve = existing->second;
            else {
                require(n <= b.max_control_points - controls,
                        "native curve list output control budget");
                controls += n;
                for (unsigned k = 0; k < 8; ++k)
                    charge(b, n);
                curve = std::make_shared<BsplineCurve>(s);
                ++copies;
                if (!clone)
                    working.emplace(&s, curve);
            }
            if (!skip) {
                for (unsigned k = 0; k < 32; ++k)
                    charge(b, n);
                auto poles = curve->poles();
                for (std::size_t j = 0; j < n; ++j)
                    poles[j] = curve->rational()
                                   ? curve_detail::affine_point(*m, poles[j], curve->weights()[j])
                                   : curve_detail::affine_polynomial_point(*m, poles[j]);
                *curve = curve_detail::with_poles(*curve, poles);
            }
        }
        if (clone)
            out.curves.push_back(std::move(curve));
        ++processed;
    }
    if (!clone && !skip)
        for (std::size_t i = 0; i < source.size(); ++i) {
            const auto found = working.find(source[i].get());
            if (found != working.end())
                out.curves[i] = found->second;
        }
    Json aliases = Json::array();
    std::unordered_map<const BsplineCurve *, std::size_t> first;
    for (std::size_t i = 0; i < out.curves.size(); ++i) {
        if (!out.curves[i])
            aliases.push_back(nullptr);
        else {
            const auto entry = first.emplace(out.curves[i].get(), i);
            aliases.push_back(entry.first->second);
        }
    }
    out.report = {{"scope", "native_swept_curve_list_transform"},
                  {"native_result", !failed},
                  {"clone_each_occurrence", clone},
                  {"transform_skipped", skip},
                  {"transform_evaluated", evaluated},
                  {"processed_occurrences", processed},
                  {"working_copies", copies},
                  {"failure_index", failed ? Json(*failed) : Json(nullptr)},
                  {"first_reference_indices", std::move(aliases)},
                  {"work_used", b.work}};
    return out;
}
TubeFacetBranches prepare_tube_facet_branches(const TubeFacetMember &member, const Matrix4 *m,
                                              bool preserve_original, TubeBudget &b) {
    auto prepared = prepare_tube_facet_member(member, b);
    TubeCurveViews source{std::make_shared<const BsplineCurve>(std::move(prepared.curve))};
    auto transformed = transform_tube_curve_list(source, m, preserve_original, b);
    TubeFacetBranches out;
    out.original = preserve_original ? std::move(source) : transformed.curves;
    out.transformed = std::move(transformed.curves);
    out.report = {{"scope", "native_swept_facet_member_branches"},
                  {"source_preparation", std::move(prepared.report)},
                  {"transform", std::move(transformed.report)},
                  {"work_used", b.work}};
    return out;
}
} // namespace p3d::swept_detail
