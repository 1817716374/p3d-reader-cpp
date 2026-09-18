#include "internal.hpp"

namespace p3d {
namespace {
std::vector<std::size_t> removed_rows(const BsplineSurface &surface) {
    if (surface.v().order() != 2)
        return {};
    const auto &knots = surface.v().knots();
    struct Group {
        double value;
        std::size_t count;
    };
    std::vector<Group> groups{{knots.front(), 1}};
    std::size_t start = 0, end = 0;
    // Native compressKnots compares to the current group representative, not
    // to the preceding input knot. Active-domain ends reset representatives.
    for (std::size_t i = 1; i < knots.size(); ++i) {
        const auto a = groups.back().value, b = knots[i];
        if (std::abs(a - b) < ((std::abs(a) + 1) + std::abs(b)) * 1e-14)
            ++groups.back().count;
        else
            groups.push_back({b, 1});
        if (i == 1) {
            start = groups.size() - 1;
            groups.back().value = b;
        }
        if (i == knots.size() - 2) {
            end = groups.size() - 1;
            groups.back().value = b;
        }
    }
    std::vector<std::size_t> rows;
    std::size_t row = 1;
    for (auto g = start + 1; g < end; ++g) {
        for (std::size_t repeat = 1; repeat < groups[g].count; ++repeat)
            rows.push_back(++row);
        ++row;
    }
    return rows;
}
BsplineSurface prepare(const BsplineSurface &source, Json &note) {
    const auto rows = removed_rows(source);
    if (rows.empty())
        return source;
    const auto nu = source.u().pole_count(), nv = source.v().pole_count();
    require(rows.size() < nv, "native loft cleanup removes all V poles");
    std::vector<bool> remove_poles(source.poles().size()), remove_knots(source.v().knots().size()),
        remove_weights(source.weights().size());
    std::size_t erased_weights = 0;
    Json weight_indices = Json::array();
    for (auto row : rows) {
        require(row < nv && row + 1 < remove_knots.size(), "native loft cleanup index range");
        remove_knots[row + 1] = true;
        for (std::size_t col = 0; col < nu; ++col) {
            remove_poles[row * nu + col] = true;
            if (source.rational()) {
                // Native coordinates subtract the cumulative erasure count;
                // weights do not. Erasure positions increase monotonically,
                // so current index + prior erasures is the original index.
                const auto original = row * nu + col + erased_weights;
                require(original < remove_weights.size(),
                        "native loft weight cleanup would access outside the array");
                remove_weights[original] = true;
                weight_indices.push_back(original);
                ++erased_weights;
            }
        }
    }
    std::vector<Point3> poles;
    std::vector<double> weights, knots;
    for (std::size_t i = 0; i < source.poles().size(); ++i)
        if (!remove_poles[i])
            poles.push_back(source.poles()[i]);
    for (std::size_t i = 0; i < source.weights().size(); ++i)
        if (!remove_weights[i])
            weights.push_back(source.weights()[i]);
    for (std::size_t i = 0; i < source.v().knots().size(); ++i)
        if (!remove_knots[i])
            knots.push_back(source.v().knots()[i]);
    Json xyz = Json::array();
    for (std::size_t i = 0; i < poles.size(); ++i)
        for (auto x : poles[i]) {
            // getPoles copies stored XYZ. The replacement constructor is
            // called with inputAlreadyWeighted=false and multiplies again.
            if (source.rational())
                x *= weights.at(i);
            require(std::isfinite(x), "native loft reweighted pole outside finite range");
            xyz.push_back(x);
        }
    note = {{"removed_v_pole_rows", rows},
            {"removed_source_weight_indices", weight_indices},
            {"rational_poles_reweighted", source.rational()},
            {"source_v_pole_count", nv},
            {"result_v_pole_count", nv - rows.size()}};
    return BsplineSurface::from_bgfb(
        {{"_type", "BsplineSurface"},
         {"orderU", source.u().order()},
         {"orderV", 2},
         {"closedU", source.u().closed()},
         {"closedV", source.v().closed()},
         {"numPolesU", nu},
         {"numPolesV", nv - rows.size()},
         {"knotsU", source.u().knots()},
         {"knotsV", knots},
         {"poles", xyz},
         {"weights", source.rational() ? Json(weights) : Json(nullptr)},
         {"numRulesU", 0},
         {"numRulesV", 0},
         {"boundaries", nullptr},
         {"holeOrigin", 0}});
}
} // namespace

LoftNativeFaces SectionLoft::native_faces(unsigned max_cap_control_points) const {
    require(max_cap_control_points > 0, "native loft cap control budget must be positive");
    LoftNativeFaces out;
    out.report = {{"status", "incomplete"},
                  {"representation", "native_loft_face_geometry"},
                  {"cap_status", "not_evaluated"},
                  {"side_adjustments", Json::array()}};
    try {
        SectionLoft prepared = *this;
        for (std::size_t i = 0; i < sides_.size(); ++i) {
            Json note;
            prepared.sides_[i].surface = prepare(sides_[i].surface, note);
            if (!note.is_null()) {
                note["side_index"] = i;
                out.report["side_adjustments"].push_back(std::move(note));
            }
        }
        out.caps = prepared.cap_regions(max_cap_control_points);
        out.report["cap_status"] = out.caps.report.at("status");
        if (source_.at("capped").get<bool>() && out.caps.report.at("status") != "complete") {
            out.report["status"] = out.caps.report.at("status");
            out.report["cap_failure"] = out.caps.report;
            return out;
        }
        out.sides = std::move(prepared.sides_);
        out.report["status"] = "complete";
        out.report["side_count"] = out.sides.size();
    } catch (const std::exception &e) {
        out.sides.clear();
        out.caps.bottom = nullptr;
        out.caps.top = nullptr;
        out.report["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
