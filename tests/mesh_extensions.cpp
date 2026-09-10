#include "geometry.hpp"
using namespace p3d;
namespace {
template <class T> void put(Bytes &b, T value) {
    const auto *data = reinterpret_cast<const std::uint8_t *>(&value);
    b.insert(b.end(), data, data + sizeof(value));
}
template <class T> void array(Bytes &b, const std::vector<T> &values) {
    put(b, std::uint32_t(values.size()));
    for (auto value : values)
        put(b, value);
}
Bytes prefix(std::uint32_t width = 0) {
    Bytes b;
    put(b, width);
    const std::vector<std::int32_t> points =
        width ? std::vector<std::int32_t>{1, 2, 3, 1, 3, 4}
              : std::vector<std::int32_t>{1, 2, 3, 0, 1, 3, 4, 0};
    put(b, std::uint32_t(points.size()));
    array(b, points);
    array(b, std::vector<std::int32_t>{});
    array(b, width ? std::vector<std::int32_t>{1, 2, 3, 4, 3, 2}
                   : std::vector<std::int32_t>{1, 2, 3, 0, 4, 3, 2, 0});
    const std::vector<Point3> vertices = {{0, 0, 0}, {2, 0, 0}, {2, 2, 0}, {0, 2, 0}};
    put(b, std::uint32_t(vertices.size()));
    for (auto p : vertices)
        for (double x : p)
            put(b, x);
    put(b, std::uint32_t(0));
    const std::vector<Point2> uv = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
    put(b, std::uint32_t(uv.size()));
    for (auto p : uv)
        for (double x : p)
            put(b, x);
    return b;
}
Json command(const Bytes &extra, std::uint32_t width = 0) {
    auto b = prefix(width);
    b.insert(b.end(), extra.begin(), extra.end());
    return {{"op", 25}, {"offset", 2}, {"body", rawbytes(b)}, {"decoded", command_fields(25, b)}};
}
Bytes extension(std::vector<std::size_t> *ends = nullptr) {
    Bytes b;
    auto end = [&] {
        if (ends)
            ends->push_back(b.size());
    };
    put(b, std::uint32_t(5));
    for (unsigned i = 0; i < 5; ++i)
        for (float x : {float(i), .25f, -.5f})
            put(b, x);
    end();
    // Deliberately differs from parameter indices; the native reader skips it.
    array(b, std::vector<std::int32_t>{-5, 5, 5, 0, 5, 5, 5, 0});
    end();
    array(b, std::vector<std::uint16_t>{0x6728, 0x6750, 0});
    end();
    put(b, std::uint32_t(6));
    for (unsigned i = 0; i < 6; ++i) {
        put(b, 10. + i);
        put(b, -double(i));
    }
    end();
    array(b, std::vector<std::int32_t>{-7, 3});
    end();
    array(b, std::vector<std::uint64_t>{0, UINT64_MAX - 123});
    end();
    return b;
}
Json channels(const Geometry &g) {
    return g.primitive_ranges.at(0).at("mesh_channels");
}
} // namespace
unsigned mesh_extension_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };
    std::vector<std::size_t> ends;
    auto extra = extension(&ends);
    auto cmd = command(extra);
    const auto &d = cmd["decoded"];
    check(!d.contains("field_decode_error") && d["num_per_face"] == 0 &&
              d["reported_point_index_count"] == 8 && d["point_index_count_matches_header"] == true,
          "mesh header fields have confirmed meanings");
    auto c = d["mesh_channels"];
    check(c["status"] == "decoded" && c["fields"].size() == 6 && c["raw_hex"] == hex(extra),
          "six extension fields decoded with complete raw bytes");
    check(c["float_colors"][4] == Json({4., .25, -.5}) && c["color_indices"][0] == -5,
          "FloatRgb and signed independent source color indices retained");
    check(c["illumination_name"] == u8"木材" && c["illumination_name_status"] == "decoded" &&
              c["fields"][2]["count"] == 3,
          "UTF16 illumination name count includes terminator");
    check(c["face_uv_points"][5] == Json({15., -5.}) &&
              c["face_smoothing_groups"] == Json({-7, 3}) &&
              c["face_material_ids"][1].get<std::uint64_t>() == UINT64_MAX - 123,
          "face UV groups and full-width material IDs retained");
    const auto &bindings = c["bindings"];
    check(bindings["serialized_color_indices_used_by_reader"] == false &&
              bindings["polygons"][0]["color_indices"] == Json({0, 1, 2}) &&
              bindings["polygons"][1]["color_indices"] == Json({3, 2, 1}),
          "actual reader color lookup uses parameter indices");
    check(bindings["polygons"][1]["face_uv_point_indices"] == Json({3, 4, 5}) &&
              bindings["polygons"][0]["material_id"] == 0 &&
              bindings["polygons"][0]["smoothing_group"] == -7,
          "face channels bind in source nonempty polygon order");
    auto g = reconstruct(Json::array({cmd}), {});
    check(g.unknown.empty() && g.faces.size() == 2 && channels(g)["source"] == c,
          "geometry carries all nonempty mesh channels");
    auto triangles = channels(g)["triangles"];
    check(triangles[0]["source_corners"] == Json({0, 1, 2}) &&
              triangles[1]["material_id"].get<std::uint64_t>() == UINT64_MAX - 123 &&
              triangles[1]["face_uv_point_indices"] == Json({3, 4, 5}),
          "triangle bindings retain full source material IDs and corner identity");
    check((*g.face_uvs[0])[0] == Point2{0, 0}, "additional face UV does not overwrite primary UV");
    auto native = c["native_triangulation"];
    check(native["status"] == "mapped" && native["uv_source"] == "face_uv_points" &&
              native["normal_mode"] == "smoothing_groups" &&
              native["explicit_normals_used"] == false,
          "native triangulation selects additional UV and generated normals");
    check(native["polygons"][0]["normal_group"] == -7 &&
              native["polygons"][1]["normal_group"] == 3 &&
              native["normal_rule"]["group_comparison"] == "signed_integer_equality" &&
              native["normal_rule"]["position_tolerance"] == 1e-7,
          "one positive group activates exact signed group matching including negative groups");
    check(triangles[1]["native_triangulation"]["material_id"] ==
              native["polygons"][1]["material_id"],
          "native material routing follows source polygon into triangles");
    auto route = [&](const std::vector<std::int32_t> &groups,
                     const std::vector<std::uint64_t> &materials, std::size_t uv_count = 6) {
        Bytes data(12);
        put(data, std::uint32_t(uv_count));
        for (std::size_t i = 0; i < uv_count * 2; ++i)
            put(data, double(i));
        array(data, groups);
        array(data, materials);
        return command(data)["decoded"]["mesh_channels"];
    };
    auto no_groups = route({}, {81, 82});
    check(no_groups["bindings"]["polygons"][0]["material_id"] == 81 &&
              no_groups["native_triangulation"]["polygons"][0]["material_id"] == 0 &&
              no_groups["native_triangulation"]["polygons"][0]["source_material_index"].is_null() &&
              no_groups["native_triangulation"]["zero_padded_material_count"] == 2 &&
              no_groups["native_triangulation"]["unused_source_material_count"] == 2,
          "source material association differs from native zero padding when no groups exist");
    auto short_groups = route({7}, {81, 82})["native_triangulation"];
    check(short_groups["normal_mode"] == "flat_triangles" &&
              short_groups["polygons"][0]["material_id"] == 81 &&
              short_groups["polygons"][1]["material_id"] == 0 &&
              short_groups["copied_material_count"] == 1,
          "short group list copies only its count of materials and disables smoothing");
    auto long_groups = route({7, 8, 9}, {81, 82, 83, 84})["native_triangulation"];
    check(long_groups["normal_mode"] == "flat_triangles" && long_groups["polygons"].size() == 2 &&
              long_groups["discarded_copied_material_count"] == 1 &&
              long_groups["unused_source_material_count"] == 1,
          "native copied materials are resized to face count after bounded copying");
    auto unsafe = route({7, 8, 9}, {81, 82})["native_triangulation"];
    check(unsafe["status"] == "unsafe_material_copy" && unsafe["polygons"].empty(),
          "native out-of-range material copy is diagnosed without reading or inventing values");
    for (auto groups : {std::vector<std::int32_t>{0, 0}, std::vector<std::int32_t>{-7, -7}}) {
        auto r = route(groups, {81, 82})["native_triangulation"];
        check(r["normal_mode"] == "flat_triangles" && r["polygons"][0]["normal_group"].is_null(),
              "zero or only negative groups do not activate native smoothing");
    }
    auto mixed = route({0, 7}, {81, 82})["native_triangulation"];
    check(mixed["normal_mode"] == "smoothing_groups" &&
              mixed["polygons"][0]["normal_group"].is_null() &&
              mixed["polygons"][1]["normal_group"] == 7,
          "zero-group triangles stay flat when another group activates smoothing");
    check(route({1, 2}, {})["native_triangulation"]["status"] == "no_face_materials" &&
              route({1, 2}, {81, 82}, 5)["native_triangulation"]["status"] ==
                  "face_uv_count_mismatch",
          "native path requires both face materials and complete corner UVs");
    auto signed_arrays = d["index_arrays"];
    signed_arrays[0][0] = -1;
    check(decode_mesh_channels(extra, signed_arrays, d["polygons"],
                               0)["native_triangulation"]["status"] == "unsafe_signed_point_lookup",
          "native direct point lookup does not silently adopt absolute-value source indexing");
    auto mirror = identity();
    mirror[0][0] = -1;
    Geometry placed, twice;
    merge_geometry(placed, g, mirror);
    auto mapped = channels(placed);
    check(mapped["source"] == c && mapped["triangles"][0]["source_corners"] == Json({0, 2, 1}) &&
              mapped["triangles"][1]["color_indices"] == Json({3, 1, 2}) &&
              mapped["triangles"][1]["face_uv_point_indices"] == Json({3, 5, 4}),
          "mirror changes triangle corner bindings without changing source arrays");
    merge_geometry(twice, placed, mirror);
    check(channels(twice) == channels(g), "two mirrors restore extension bindings");
    Bytes transform_bytes;
    for (unsigned i = 0; i < 3; ++i)
        for (unsigned j = 0; j < 4; ++j)
            put(transform_bytes, mirror[i][j]);
    Json push = {{"op", 13},
                 {"offset", 0},
                 {"body", rawbytes(transform_bytes)},
                 {"decoded", command_fields(13, transform_bytes)}};
    Json pop = {{"op", 14}, {"offset", 1}, {"body", rawbytes({})}, {"decoded", Json::object()}};
    auto command_mirror = reconstruct(Json::array({push, cmd, pop}), {});
    check(command_mirror.unknown.empty() && channels(command_mirror) == mapped,
          "command matrix mirrors all source corner bindings once");
    Geometry merged;
    merge_geometry(merged, g, identity());
    merge_geometry(merged, placed, identity());
    check(merged.primitive_ranges[1]["start"] == 2 &&
              merged.primitive_ranges[1]["mesh_channels"] == mapped,
          "instance concatenation preserves command-local source indices and rebases range");
    auto unknown = extra;
    unknown.push_back(0x5a);
    auto extended = command(unknown);
    check(extended["decoded"]["mesh_channels"]["remaining_hex"] == "5a" &&
              extended["decoded"]["mesh_channels"]["status"] == "partial" &&
              reconstruct(Json::array({extended}), {}).faces.size() == 2,
          "future suffix retained while known geometry remains available");
    for (std::size_t n = 0; n < extra.size(); ++n) {
        auto truncated = command(slice(extra, 0, n));
        auto &decoded = truncated["decoded"];
        const bool optional_boundary = n == ends[2] || n == ends[3] || n == ends[4];
        check(
            !decoded.contains("field_decode_error") &&
                decoded["mesh_channels"]["status"] == (optional_boundary ? "decoded" : "invalid") &&
                decoded["mesh_channels"]["raw_hex"] == hex(slice(extra, 0, n)),
            "all extension truncation boundaries are safe and distinguish omitted optional fields");
    }
    auto legacy = command(slice(extra, 0, ends[2]));
    check(legacy["decoded"]["mesh_channels"]["fields"][3]["status"] == "omitted" &&
              legacy["decoded"]["mesh_channels"]["bindings"]["material_id_status"] == "absent" &&
              reconstruct(Json::array({legacy}), {}).unknown.empty(),
          "legacy suffix omits later face channels");
    auto mismatch = slice(extra, 0, ends[2]);
    put(mismatch, std::uint32_t(1));
    put(mismatch, 7.);
    put(mismatch, 8.);
    array(mismatch, std::vector<std::int32_t>{9});
    array(mismatch, std::vector<std::uint64_t>{42});
    auto mis = command(mismatch);
    auto bad_geo = reconstruct(Json::array({mis}), {});
    check(bad_geo.faces.size() == 2 && bad_geo.unknown.size() == 3 &&
              channels(bad_geo)["triangles"][0]["material_id"].is_null() &&
              channels(bad_geo)["source"]["face_material_ids"] == Json({42}),
          "count mismatch preserves raw arrays without inventing face bindings");
    auto empty = command(Bytes(24));
    auto plain = reconstruct(Json::array({empty}), {});
    check(plain.unknown.empty() && !plain.primitive_ranges[0].contains("mesh_channels"),
          "all-empty extension pools do not allocate per-triangle metadata");
    auto fixed = command(Bytes(24), 3);
    auto fixed_geo = reconstruct(Json::array({fixed}), {});
    check(fixed["decoded"]["polygon_layout"] == "fixed_width" && fixed_geo.unknown.empty() &&
              fixed_geo.faces == g.faces,
          "fixed-width indexed triangles need no zero separators");
    auto padded_bytes = prefix();
    padded_bytes[0] = 4;
    padded_bytes.resize(padded_bytes.size() + 24);
    Json padded = {{"op", 25},
                   {"offset", 2},
                   {"body", rawbytes(padded_bytes)},
                   {"decoded", command_fields(25, padded_bytes)}};
    auto padded_geo = reconstruct(Json::array({padded}), {});
    check(padded_geo.unknown.empty() && padded_geo.faces == g.faces &&
              padded["decoded"]["index_arrays"][0].size() == 8,
          "fixed-width zero padding is excluded from corners but retained in source arrays");
    auto fixed_ext = reconstruct(Json::array({command(extra, 3)}), {});
    check(fixed_ext.faces.size() == 2 && !fixed_ext.unknown.empty() &&
              channels(fixed_ext)["source"]["bindings"]["face_uv_status"] ==
                  "not_evaluated_for_fixed_width",
          "fixed-width extension grouping does not borrow zero-terminated consumer assumptions");
    check(
        channels(fixed_ext)["source"]["native_triangulation"]["status"] == "no_terminated_faces" &&
            channels(
                fixed_ext)["source"]["native_triangulation"]["ignored_unterminated_corner_count"] ==
                6,
        "native face getter ignores an unterminated fixed-width index sequence");
    auto padded_ext = command(extra);
    auto padded_ext_bytes = prefix();
    padded_ext_bytes[0] = 4;
    padded_ext_bytes.insert(padded_ext_bytes.end(), extra.begin(), extra.end());
    padded_ext["body"] = rawbytes(padded_ext_bytes);
    padded_ext["decoded"] = command_fields(25, padded_ext_bytes);
    auto padded_ext_geo = reconstruct(Json::array({padded_ext}), {});
    check(padded_ext_geo.unknown.empty() && padded_ext_geo.faces.size() == 2 &&
              channels(padded_ext_geo)["source"]["bindings"]["face_uv_status"] == "mapped" &&
              channels(padded_ext_geo)["triangles"] == triangles,
          "zero-padded fixed-width faces share the confirmed native corner and material layout");
    // Different consumer segmentation must not be assigned to source triangles.
    auto misleading_arrays = d["index_arrays"];
    misleading_arrays[0] = {1, 0, 2, 0, 3, 0, 4, 0};
    auto misleading = decode_mesh_channels(extra, misleading_arrays, d["polygons"], 4);
    check(misleading["bindings"]["face_uv_status"] == "not_evaluated_for_fixed_width" &&
              misleading["native_triangulation"]["source_polygon_layout_matches"] == false,
          "more zero-delimited faces than source polygons are bounded and kept separate");
    // Name payload and later fields remain locatable even for invalid text.
    Bytes name;
    put(name, std::uint32_t(0));
    put(name, std::uint32_t(0));
    array(name, std::vector<std::uint16_t>{'A'});
    name.resize(name.size() + 12);
    auto unterminated = command(name);
    check(unterminated["decoded"]["mesh_channels"]["illumination_name_status"] == "unterminated" &&
              unterminated["decoded"]["mesh_channels"]["status"] == "partial",
          "unterminated illumination name is not reported as a valid native string");
    name.clear();
    put(name, std::uint32_t(0));
    put(name, std::uint32_t(0));
    array(name, std::vector<std::uint16_t>{0xd800, 0});
    name.resize(name.size() + 12);
    auto unicode = command(name);
    check(unicode["decoded"]["mesh_channels"]["illumination_name_status"] == "invalid_unicode" &&
              unicode["decoded"]["mesh_channels"]["illumination_name"].is_null(),
          "invalid UTF16 source remains raw without invalid UTF8 output");
    auto source_polygons = d["polygons"];
    source_polygons.insert(source_polygons.begin(), Json{{"point_indices", Json::array()},
                                                         {"normal_indices", Json::array()},
                                                         {"uv_indices", Json::array()}});
    auto empty_face = decode_mesh_channels(extra, d["index_arrays"], source_polygons, 0);
    check(empty_face["bindings"]["polygons"][0]["material_id"].is_null() &&
              empty_face["bindings"]["polygons"][1]["material_id"] == 0 &&
              empty_face["bindings"]["polygons"][2]["face_uv_point_indices"] == Json({3, 4, 5}),
          "empty separators do not consume face material or corner UV entries");
    Bytes fewer_colors;
    put(fewer_colors, std::uint32_t(1));
    for (float x : {1.f, 0.f, 0.f})
        put(fewer_colors, x);
    fewer_colors.insert(fewer_colors.end(), extra.begin() + ends[0], extra.end());
    auto color_bad = reconstruct(Json::array({command(fewer_colors)}), {});
    check(color_bad.faces.size() == 2 && color_bad.unknown.size() == 1 &&
              channels(color_bad)["triangles"][0]["color_indices"].is_null() &&
              channels(color_bad)["source"]["bindings"]["color_status"] == "invalid_reader_indices",
          "invalid native color references retain geometry and source colors without indexing out "
          "of bounds");
    // A concave source polygon must keep corner-dependent fields after ear clipping.
    auto concave = cmd;
    concave["decoded"]["points"] = {{0, 0, 0}, {2, 0, 0}, {1, .5, 0}, {0, 2, 0}};
    concave["decoded"]["polygons"] = Json::array({{{"point_indices", {1, 2, 3, 4}},
                                                   {"normal_indices", Json::array()},
                                                   {"uv_indices", {4, 3, 2, 1}}}});
    Bytes one = slice(extra, 0, ends[2]);
    put(one, std::uint32_t(4));
    for (unsigned i = 0; i < 4; ++i) {
        put(one, 10. + i);
        put(one, -double(i));
    }
    array(one, std::vector<std::int32_t>{17});
    array(one, std::vector<std::uint64_t>{88});
    concave["decoded"]["mesh_channels"] =
        decode_mesh_channels(one, d["index_arrays"], concave["decoded"]["polygons"], 0);
    auto concave_geo = reconstruct(Json::array({concave}), {});
    check(concave_geo.unknown.empty() && concave_geo.faces.size() == 2,
          "concave extension mesh triangulates");
    const auto concave_channels = channels(concave_geo);
    for (std::size_t f = 0; f < concave_geo.faces.size(); ++f) {
        const auto &binding = concave_channels["triangles"][f];
        check(binding["material_id"] == 88 && binding["smoothing_group"] == 17,
              "all derived triangles retain source polygon material and smoothing group");
        for (unsigned corner = 0; corner < 3; ++corner) {
            auto p = concave_geo.faces[f][corner];
            check(binding["source_corners"][corner] == p &&
                      binding["face_uv_point_indices"][corner] == p &&
                      binding["color_indices"][corner] == 3 - p,
                  "ear clipping keeps independent color and additional UV corner associations");
        }
    }
    NativeScene scene;
    auto def = std::make_shared<GeometryDefinition>();
    def->geometry = g;
    def->source_key = "mesh";
    scene.definitions.push_back(def);
    scene.metadata = {{"color_tables", Json::array()},
                      {"materials", {{"definitions", Json::array()}}},
                      {"document_graph", Json::object()}};
    SceneElement element;
    element.metadata = {{"unknown", Json::array()},
                        {"notes", Json::array()},
                        {"model_id", 1},
                        {"binding_status", "missing"}};
    GeometryInstance instance;
    instance.definition = 0;
    instance.matrix = mirror;
    element.instances = {instance, instance};
    scene.elements.push_back(element);
    const auto output = scene.expanded()["elements"][0];
    check(output["primitive_ranges"][1]["start"] == 2 &&
              output["primitive_ranges"][1]["mesh_channels"] == mapped &&
              channels(def->geometry) == channels(g),
          "expanded scene retains independent face material and corner metadata without mutating "
          "definition");
    check(channels(g)["triangles"][1]["source_material_index"] == 1 &&
              channels(placed)["triangles"][1]["source_material_index"] == 1,
          "source material list index survives placement and triangle routing");
    const auto large_id = UINT64_MAX - 123;
    scene.metadata["materials"]["definitions"] =
        Json::array({{{"scope", "model:1"}, {"id", large_id}, {"base_color_rgb", {1., 0., 0.}}},
                     {{"scope", "global"}, {"id", large_id}, {"base_color_rgb", {0., 1., 0.}}},
                     {{"scope", "model:2"}, {"id", large_id}, {"base_color_rgb", {0., 0., 1.}}},
                     {{"scope", "global"}, {"id", 0}}});
    // One shared geometry definition is used by elements in distinct model scopes.
    scene.elements[0].instances[0].style = {{"material_id", 91}};
    auto second = scene.elements[0];
    second.metadata["model_id"] = 2;
    scene.elements.push_back(second);
    auto third = second;
    third.metadata["model_id"] = 3;
    scene.elements.push_back(third);
    const auto original_range = def->geometry.primitive_ranges;
    auto scoped_output = scene.expanded();
    std::size_t visits = 0;
    scene.for_each_primitive([&](const PrimitiveView &view) {
        ++visits;
        const auto &refs = view.mesh_material_references;
        auto expected = view.element_index == 0 ? 0u : view.element_index == 1 ? 2u : 1u;
        check(refs.size() == 2 && refs[0].material_id == 0 && refs[0].status == "unassigned" &&
                  refs[0].candidates.empty(),
              "zero face material stays unassigned even if a zero-ID definition exists");
        check(refs[1].material_id == large_id && refs[1].status == "resolved" &&
                  refs[1].candidates == std::vector<std::size_t>{expected},
              "face material resolves in each instance model scope before global scope");
        const auto &expanded_ref =
            scoped_output["elements"][view.element_index]["primitive_ranges"][view.instance_index]
                         ["mesh_material_references"][1];
        check(expanded_ref["material_id"] == large_id && expanded_ref["status"] == refs[1].status &&
                  expanded_ref["candidates"] == Json(refs[1].candidates),
              "expanded and borrowed views agree on mesh material candidates");
        check(view.instance_index != 0 ||
                  (view.material_status == "missing" && view.style["material_id"] == 91 &&
                   view.appearance["material_id"] == 91),
              "resolved face references do not silently override unresolved range material");
    });
    check(visits == 6 && def->geometry.primitive_ranges == original_range,
          "reference lookup preserves shared geometry and visits all model instances");
    scene.metadata["materials"]["definitions"].push_back(
        scene.metadata["materials"]["definitions"][0]);
    scene.for_each_primitive([&](const PrimitiveView &view) {
        if (view.element_index == 0)
            check(view.mesh_material_references[1].status == "ambiguous" &&
                      view.mesh_material_references[1].candidates ==
                          std::vector<std::size_t>({0, 4}),
                  "ambiguous local face materials retain every candidate without global fallback");
    });
    scene.metadata["materials"]["definitions"] = Json::array();
    scene.for_each_primitive([&](const PrimitiveView &view) {
        check(view.mesh_material_references[1].status == "missing" &&
                  view.mesh_material_references[1].candidates.empty(),
              "missing face materials remain explicit references");
    });
    auto duplicates = slice(extra, 0, ends[4]);
    array(duplicates, std::vector<std::uint64_t>{large_id, large_id});
    def->geometry = reconstruct(Json::array({command(duplicates)}), {});
    scene.elements.resize(1);
    scene.elements[0].instances.resize(1);
    scene.for_each_primitive([&](const PrimitiveView &view) {
        check(view.mesh_material_references.size() == 2 &&
                  view.mesh_material_references[0].material_id == large_id &&
                  view.mesh_material_references[1].material_id == large_id &&
                  (*view.source_range)["mesh_channels"]["triangles"][0]["source_material_index"] ==
                      0 &&
                  (*view.source_range)["mesh_channels"]["triangles"][1]["source_material_index"] ==
                      1,
              "duplicate source material entries keep distinct source list indices");
    });
    def->geometry = plain;
    scene.elements[0].metadata.erase("model_id");
    scene.elements[0].instances[0].style = Json::object();
    scene.for_each_primitive([&](const PrimitiveView &view) {
        check(view.mesh_material_references.empty(), "ordinary geometry has no extra references");
    });
    return checks;
}
