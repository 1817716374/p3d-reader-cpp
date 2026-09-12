#include "geometry.hpp"

unsigned mesh_buffer_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto triangle = [](const std::array<Point3, 3> &points, std::int32_t group,
                       std::uint64_t material) {
        Json out = {{"normal_group", group}, {"material_id", material}, {"corners", Json::array()}};
        for (const auto &p : points)
            out["corners"].push_back({{"point", p}, {"uv", {0., 0.}}});
        return out;
    };
    const std::array<Point3, 3> xy{{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}};
    auto a = triangle(xy, 1, UINT64_MAX);
    auto b = triangle(xy, 1, 0);
    auto input = Json::array({a, b});
    const auto original = input;
    for (bool smooth : {false, true}) {
        auto out = assemble_native_mesh_buffers(input, smooth);
        check(out["points"] == Json(xy) && out["faces"] == Json({{0, 1, 2}, {0, 1, 2}}),
              "native vertex registry shares identical corners across materials");
        check(out["face_material_ids"] == Json({UINT64_MAX, std::uint64_t(0)}) &&
                  out["normals"] == Json({{0., 0., 1.}, {0., 0., 1.}, {0., 0., 1.}}),
              "full material IDs remain per face and normals align with positions");
    }
    check(input == original, "derived buffers leave source triangle corners unchanged");
    b["corners"][0]["uv"] = {2., 3.};
    for (bool smooth : {false, true}) {
        auto out = assemble_native_mesh_buffers(Json::array({a, b}), smooth);
        check(out["points"].size() == 4 && out["faces"] == Json({{0, 1, 2}, {3, 1, 2}}) &&
                  out["uvs"][3] == Json({2., 3.}),
              "UV seam splits the native final vertex key");
    }
    b["corners"][0]["uv"] = {double(float(5e-7)), 0.};
    auto near_uv = assemble_native_mesh_buffers(Json::array({a, b}), true);
    check(near_uv["registry_counts"]["uvs"] == 1 && near_uv["points"].size() == 3 &&
              near_uv["uvs"][0] == Json({0., 0.}),
          "UV registry retains its first tolerance representative");
    b["corners"][0]["uv"] = {double(float(2e-6)), 0.};
    check(assemble_native_mesh_buffers(Json::array({a, b}), true)["registry_counts"]["uvs"] == 2,
          "UV outside the native tolerance keeps a separate index");

    auto shifted = xy;
    for (auto &p : shifted)
        p[0] += 5e-7;
    b = triangle(shifted, 1, 2);
    check(assemble_native_mesh_buffers(Json::array({a, b}), false)["points"].size() == 3 &&
              assemble_native_mesh_buffers(Json::array({a, b}), true)["points"].size() == 6,
          "flat packed float and smooth double position registries use different tolerances");

    const std::array<Point3, 3> large{{{1e8, 0, 0}, {1e8 + 16, 0, 0}, {1e8, 16, 0}}};
    shifted = large;
    for (auto &p : shifted)
        p[0] += 1;
    auto large_input = Json::array({triangle(large, 1, 4), triangle(shifted, 1, 5)});
    auto large_smooth = assemble_native_mesh_buffers(large_input, true);
    auto large_flat = assemble_native_mesh_buffers(large_input, false);
    check(large_smooth["points"].size() == 6 && large_flat["points"].size() == 3,
          "smooth registry identities precede float export, flat keys follow float export");
    check(large_smooth["points"][0] == large_smooth["points"][3] &&
              large_smooth["faces"][1] == Json({3, 4, 5}),
          "equal exported float values must not trigger extra smooth deduplication");

    b = triangle({{{0, 0, 0}, {0, 0, 2}, {1, 0, 0}}}, 1, 42);
    auto angled = assemble_native_mesh_buffers(Json::array({a, b}), true);
    const double diagonal = double(float(1 / std::sqrt(2.)));
    check(angled["normals"][0] == Json({0., diagonal, diagonal}) &&
              angled["normals"][1] == angled["normals"][0] && angled["points"].size() == 4,
          "smooth normal uses unit face contributions across material boundaries");
    b["normal_group"] = 3;
    auto separate = assemble_native_mesh_buffers(Json::array({a, b}), true);
    check(separate["points"].size() == 6 && separate["normals"][3] == Json({0., 1., 0.}),
          "overlapping group bits are not a shared smoothing group");
    a["normal_group"] = -3;
    b["normal_group"] = -3;
    check(assemble_native_mesh_buffers(Json::array({a, b}), true)["normals"] == angled["normals"],
          "negative group keys remain active in the selected smooth path");
    a["normal_group"] = 0;
    b["normal_group"] = 0;
    check(assemble_native_mesh_buffers(Json::array({a, b}), true)["points"].size() == 6,
          "zero groups use independent face normals inside the smooth branch");
    check(assemble_native_mesh_buffers(Json::array({a, b}), false)["points"].size() == 6,
          "flat vertex registry includes the face normal in all eight float components");

    a = triangle(xy, 1, 1);
    b = triangle({{xy[0], xy[2], xy[1]}}, 1, 1);
    auto cancel = assemble_native_mesh_buffers(Json::array({a, b}), true);
    check(cancel["normals"] == Json({{1., 0., 0.}, {1., 0., 0.}, {1., 0., 0.}}),
          "opposite unit normal sums retain the native positive X fallback");
    b = triangle({{{10, 0, 0}, {11, 0, 0}, {10, 1, 5e-8}}}, 0, 2);
    a["normal_group"] = 0;
    auto near_normal = assemble_native_mesh_buffers(Json::array({a, b}), true);
    check(near_normal["registry_counts"]["normals"] == 1 &&
              near_normal["normals"][3] == Json({0., 0., 1.}),
          "normal registry has an independent double tolerance and representative");
    b["corners"][2]["point"][2] = 2e-7;
    check(assemble_native_mesh_buffers(Json::array({a, b}), true)["registry_counts"]["normals"] ==
              2,
          "normal vectors outside tolerance are not merged");
    for (bool smooth : {false, true}) {
        auto empty = assemble_native_mesh_buffers(Json::array(), smooth);
        check(empty["faces"].empty() && empty["points"].empty(),
              "empty tessellation exports empty pools");
    }
    return checks;
}
