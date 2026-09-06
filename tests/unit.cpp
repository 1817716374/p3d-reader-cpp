#include "geometry.hpp"
#include <iostream>
#include <cstring>
using namespace p3d;
static unsigned checks = 0;
static void check(bool value, const char *message) {
    ++checks;
    require(value, message);
}
template <class F> static void rejects(F f, const char *message) {
    bool threw = false;
    try {
        f();
    } catch (const std::exception &) {
        threw = true;
    }
    check(threw, message);
}
template <class T> static void put(Bytes &b, T x) {
    auto off = b.size();
    b.resize(off + sizeof(x));
    std::memcpy(b.data() + off, &x, sizeof(x));
}
static Json command(unsigned op, const Bytes &body, std::size_t offset = 2) {
    return {{"op", op},
            {"offset", offset},
            {"body", rawbytes(body)},
            {"decoded", command_fields(op, body)}};
}
static Geometry triangle() {
    Geometry g;
    g.vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    g.faces = {{0, 1, 2}};
    g.face_uvs.push_back(std::array<Point2, 3>{{{0, 0}, {1, 0}, {0, 1}}});
    g.face_source_polygons.push_back(0);
    g.primitive_ranges.push_back(
        {{"channel", "faces"}, {"start", 0}, {"count", 1}, {"style", Json::object()}});
    return g;
}
int main() {
    try {
        Bytes zipped = {3,    0,    0,    0,    3,    0,    0,    0,    0x78, 0x9c,
                        0x4b, 0x4c, 0x4a, 0x06, 0x00, 0x02, 0x4d, 0x01, 0x27};
        auto inflated = decode_attribute(7, 99, zipped);
        check(inflated["codec"] == "zlib" && bytesof(inflated["data"]) == Bytes({'a', 'b', 'c'}),
              "zlib attribute payload decoded in every build mode");
        zipped.back() ^= 1;
        check(decode_attribute(7, 99, zipped)["encoding"] == "opaque",
              "corrupt zlib checksum is not accepted as decoded data");
        for (std::size_t n = 0; n < 50; ++n) {
            Bytes b;
            for (std::size_t i = 0; i < n; ++i)
                b.push_back(std::uint8_t(i * 37));
            check(unbase64(base64(b)) == b, "base64 roundtrip");
        }
        check(utf16({0x3d, 0xd8, 0x00, 0xde}) == "\xf0\x9f\x98\x80", "UTF16 surrogate pair");
        check(latin1({0xfc}) == "\xc3\xbc", "DEX Latin1");
        rejects([]() { utf16({0, 0xd8}); }, "UTF16 truncation rejected");
        rejects([]() { parse_commands({2, 0, 25, 0, 255, 255, 255, 127}); },
                "command overrun rejected");
        rejects([]() { parse_native(Bytes(36)); }, "native invalid base rejected");
        Bytes dex = {'P', '3', 'D',  'D', 'E',  'X', 0,   255, 10, 13,   0,    0, 0,    0,
                     0,   0,   0x30, 1,   0xfa, 1,   'R', 2,   0,  0x10, 0xfa, 1, 0xfc, 4};
        std::size_t pos = 0;
        auto obj = parse_dex(dex, pos);
        check(pos == dex.size() && obj["root"]["value"] == "\xc3\xbc", "DEX literal text");
        for (std::size_t n = 0; n < dex.size(); ++n) {
            auto short_dex = slice(dex, 0, n);
            rejects(
                [&]() {
                    std::size_t p = 0;
                    parse_dex(short_dex, p);
                },
                "DEX truncation rejected");
        }
        auto src = triangle();
        auto mirror = identity();
        mirror[0][0] = -1;
        Geometry placed;
        merge_geometry(placed, src, mirror);
        check(placed.faces[0] == Triangle{0, 2, 1}, "mirror winding");
        check((*placed.face_uvs[0])[1] == Point2{0, 1}, "mirror UV attachment");
        Geometry twice;
        merge_geometry(twice, placed, mirror);
        check(twice.faces == src.faces && twice.face_uvs == src.face_uvs &&
                  twice.vertices == src.vertices,
              "two mirrors restore geometry");
        src.texts.push_back({{"text", "annotation"},
                             {"origin", {1., 2., 3.}},
                             {"quaternion", {1., 0., 0., 0.}},
                             {"width", 3.},
                             {"height", 4.},
                             {"placement_matrix", identity()}});
        auto translation = identity();
        translation[1][3] = 5;
        Geometry text_placed;
        merge_geometry(text_placed, src, translation);
        check(text_placed.texts[0]["origin"] == Json({1., 7., 3.}), "text world position");
        check(text_placed.texts[0]["source_origin"] == Json({1., 2., 3.}),
              "text source frame retained");
        auto bad = identity();
        bad[3][0] = 1;
        rejects(
            [&]() {
                Geometry g;
                merge_geometry(g, src, bad);
            },
            "projective matrix rejected");
        Tessellation tolerance;
        tolerance.chord_tolerance = .01;
        auto ns = tolerance.segments(100);
        check(100 * (1 - std::cos(3.141592653589793 / ns)) <= .0100000001, "chord error bound");
        tolerance.max_segments = 2;
        rejects([&]() { tolerance.segments(100); }, "caller segment limit");
        Bytes poly;
        put(poly, std::uint8_t(0));
        put(poly, std::uint32_t(5));
        for (Point3 p : std::vector<Point3>{{0, 0, 0}, {2, 0, 0}, {2, 2, 0}, {0, 2, 0}, {0, 0, 0}})
            for (auto x : p)
                put(poly, x);
        auto g = reconstruct(Json::array({command(9, poly)}), {});
        check(g.faces.size() == 2 && g.unknown.empty(), "polygon triangulation");
        double area = 0;
        for (auto f : g.faces) {
            auto a = g.vertices[f[0]], b = g.vertices[f[1]], c = g.vertices[f[2]];
            area += ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) / 2;
        }
        check(std::abs(area - 4) < 1e-12, "polygon area and winding");
        Bytes pf;
        put(pf, std::uint32_t(0));
        put(pf, std::uint32_t(4));
        put(pf, std::uint32_t(4));
        for (int i : {1, 2, 3, 0})
            put(pf, std::int32_t(i));
        put(pf, std::uint32_t(0));
        put(pf, std::uint32_t(0));
        put(pf, std::uint32_t(3));
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                put(pf, 0.);
        put(pf, std::uint32_t(0));
        put(pf, std::uint32_t(0));
        pf.insert(pf.end(), 24, 0);
        auto degenerate = reconstruct(Json::array({command(25, pf)}), {});
        check(degenerate.faces.size() == 1 && degenerate.unknown.empty(),
              "source degenerate triangle retained");
        Bytes enum_raw;
        put(enum_raw, std::int32_t(-7));
        auto value = decode_binary_field("BeamType", enum_raw);
        check(value["decoded"]["value"] == -7 && value["decoded"]["enum_label"].is_null(),
              "unnamed enum retained");
        auto opaque = decode_binary_field("FutureField", {0x88, 0x77, 0x66});
        check(!opaque.contains("encoding") && opaque["binary_base64"] == base64({0x88, 0x77, 0x66}),
              "unknown bytes retained");
        auto object = [](unsigned cls, unsigned id, const std::string &name, const Json &fields) {
            Json children = Json::array();
            for (auto it = fields.begin(); it != fields.end(); ++it)
                children.push_back({{"name", it.key()}, {"value", it.value()}});
            return Json{{"class_id", cls},
                        {"object_id", id},
                        {"stream", {"test"}},
                        {"offset", id},
                        {"length", 1},
                        {"prefix", rawbytes({})},
                        {"root", {{"name", name}, {"children", children}}}};
        };
        auto one = object(2, 7, "Thing", {{"Value", 1}});
        auto other = object(2, 7, "Thing", {{"Value", 2}});
        auto conflict = build_graph_records(Json::array({one, one, other}), Json::array(),
                                            Json::array(), {}, {});
        check(get_object(conflict, 2, 7)["status"] == "ambiguous" &&
                  get_object(conflict, 2, 7)["records"].size() == 3,
              "conflicting objects never selected");
        auto copies =
            build_graph_records(Json::array({one, one}), Json::array(), Json::array(), {}, {});
        check(get_object(copies, 2, 7)["status"] == "identical_copies",
              "same identity copies retained");
        Json trees = Json::array();
        for (unsigned tree : {100, 200}) {
            trees.push_back(object(
                1, tree, "BimBaseTreeNodeData",
                {{"TreeId", tree}, {"NodeId", 1}, {"ParentNodeId", 65534}, {"NodeType", 9}}));
            trees.push_back(
                object(1, tree + 1, "BimBaseTreeNodeData",
                       {{"TreeId", tree},
                        {"NodeId", 2},
                        {"ParentNodeId", 1},
                        {"NodeType", 11},
                        {"ResourceRefers",
                         {nullptr, nullptr, nullptr, std::to_string((13ull << 32) | 99) + ","}}}));
        }
        auto graph = build_graph_records(trees, Json::array(), Json::array(), {13}, {"13:99"});
        check(graph["hierarchy"]["roots"].size() == 2 &&
                  graph["hierarchy"]["parent_edges"].size() == 2,
              "separate source trees");
        check(element_context(graph, 13, 99)["tree_nodes"] == Json({"1:101", "1:201"}),
              "element reachable through both source trees");
        Json model_nodes = Json::array(
            {object(1, 1, "BPModelTreeSet",
                    {{"Name", "root"}, {"ModelSetChilds", {"Archi"}}, {"ModelItems", {"Archi"}}}),
             object(1, 2, "BPModelTreeSet", {{"Name", "Archi"}}),
             object(2, 3, "BPModelTreeItem", {{"Name", "Archi"}, {"ModelId", 4}})});
        auto mg = build_graph_records(model_nodes, Json::array(), Json::array(), {4}, {});
        check(mg["hierarchy"]["model_edges"][0]["candidates"] == Json({"1:2"}) &&
                  mg["hierarchy"]["model_edges"][1]["candidates"] == Json({"2:3"}),
              "model set/item namespaces remain separate");
        auto cycle = build_graph_records(
            Json::array({object(1, 20, "BimBaseTreeNodeData",
                                {{"TreeId", 1}, {"NodeId", 0}, {"ParentNodeId", 1}}),
                         object(1, 21, "BimBaseTreeNodeData",
                                {{"TreeId", 1}, {"NodeId", 1}, {"ParentNodeId", 0}})}),
            Json::array(), Json::array(), {}, {});
        check(cycle["hierarchy"]["cycle_affected_nodes"].size() == 2, "source tree cycle reported");
        Bytes floor(96, 0);
        floor[0] = 0xa5;
        floor[92] = 0x7e;
        auto design = decode_binary_field("DesignPara", floor, "StructStandardFloorModel");
        check(design["decoded"]["fields"].size() == 21 &&
                  design["decoded"]["abi_fields"][1]["raw_hex"] == "7e000000",
              "ABI residue and padding preserved");
        Bytes floor_values = slice(floor, 0, 8);
        for (std::int32_t v :
             {175, 45, 35, 40, 30, 50, 21, 22, 25, 17, 4, 7, 6, 1, 2, 3, 4, 5, 6, 7, 8})
            put(floor_values, v);
        put(floor_values, std::uint32_t(0x01020304));
        auto floor_result =
            decode_binary_field("DesignPara", floor_values, "StructStandardFloorModel");
        const auto &fd = floor_result["decoded"];
        check(fd["named_values"] == Json({{"slab_thickness", 175},
                                          {"column_concrete_grade", 45},
                                          {"beam_concrete_grade", 35},
                                          {"shear_wall_concrete_grade", 40},
                                          {"slab_concrete_grade", 30},
                                          {"brace_concrete_grade", 50},
                                          {"slab_rebar_cover", 25},
                                          {"column_main_rebar_type", 4},
                                          {"beam_main_rebar_type", 7},
                                          {"wall_main_rebar_type", 6}}),
              "floor fields follow native member offsets");
        check(fd["identified_field_count"] == 10 && fd["unassigned_field_count"] == 11 &&
                  fd["fields"][6]["name"].is_null() && fd["fields"][6]["storage_uint32"] == 21,
              "floor unassigned members are not guessed or discarded");
        check(fd["fields"][0]["unit"] == "mm" && fd["fields"][8]["unit"] == "mm" &&
                  fd["fields"][1]["display_value"] == "C45" &&
                  fd["fields"][11]["enum_label"] == "HPB235" &&
                  unbase64(floor_result["binary_base64"]) == floor_values,
              "floor units, grade display and raw bytes preserved");
        const char *expected_rebar[] = {"HPB300",      "HRB335",      "HRB400",  "HRB500",
                                        "冷轧带肋550", "冷轧带肋600", "HTRB600", "HPB235"};
        for (std::int32_t v = -1; v <= 8; ++v) {
            for (auto off : {48, 52, 56})
                std::memcpy(floor_values.data() + off, &v, sizeof(v));
            auto result = decode_binary_field("DesignPara", floor_values,
                                              "StructStandardFloorModel")["decoded"];
            bool valid = true;
            for (auto index : {10, 11, 12}) {
                const auto &f = result["fields"][index];
                valid &=
                    f["value"] == v &&
                    (v >= 0 && v < 8
                         ? f["enum_label"] == expected_rebar[v] && f["enum_status"] == "identified"
                         : f["enum_label"].is_null() && f["enum_status"] == "unknown_value");
            }
            check(valid, "floor rebar indices include unknown and negative values");
        }
        check(
            !decode_binary_field("DesignPara", floor_values, "UnrelatedClass").contains("decoded"),
            "floor semantics are scoped to the source class");
        floor_values.pop_back();
        check(!decode_binary_field("DesignPara", floor_values, "StructStandardFloorModel")
                   .contains("decoded"),
              "truncated floor memory image is not accepted");
        auto cap_fixture = [](std::int32_t concrete, std::int32_t rebar, std::int32_t cap_type,
                              std::int32_t shape, std::int32_t steps,
                              const std::vector<std::int32_t> &heights) {
            Bytes b(24, 0); // Cap, pile section and pile base cereal/version pairs.
            for (std::int32_t v : {1, concrete, 701, 702})
                put(b, v);
            for (double v : {1.25, 2.5, 3.75})
                put(b, v);
            for (std::int32_t v : {71, 72, 73, 74, 75})
                put(b, v);
            put(b, 10.5);
            put(b, 20.5);
            put(b, std::int32_t(111));
            put(b, std::int32_t(-222));
            put(b, 12.5);
            put(b, 9.75);
            put(b, std::int64_t(123456789));
            for (std::int32_t v : {cap_type, 4321, steps})
                put(b, v);
            put(b, std::uint64_t(heights.size()));
            for (auto v : heights)
                put(b, v);
            put(b, std::int32_t(4));
            put(b, std::uint64_t(0)); // Primary points.
            put(b, std::int32_t(4));
            put(b, std::uint64_t(0)); // Secondary point arrays.
            for (std::int32_t v : {shape, 31, -32, 0, 0, rebar, 175, 22, 1200, 0, 3, 225, 16, 900})
                put(b, v);
            put(b, std::uint8_t(1));
            put(b, std::int32_t(126));
            return b;
        };
        auto decode_cap = [](const Bytes &b) {
            return decode_binary_field("BinaryData", b, "PBStandardSectionProfile",
                                       {{"Type", 0x40002}});
        };
        auto cap_bytes = cap_fixture(3, 0, 2, 1, 2, {400, 600});
        auto cap_result = decode_cap(cap_bytes);
        check(cap_result.value("encoding", "") == "pilecap_section_cereal",
              "cap fixture is decoded in its native class and type");
        const auto &cap = cap_result.at("decoded");
        const auto &pile = cap["pile_section"]["base"];
        check(pile["named_values"] == Json({{"concrete_grade", 3},
                                            {"x_offset", 111},
                                            {"y_offset", -222},
                                            {"rotation", 12.5},
                                            {"top_elevation", 9.75}}) &&
                  pile["members"][1]["enum_label"] == "C30" &&
                  pile["members"][14]["unit"] == "mm" && pile["members"][17]["unit"] == "m" &&
                  !pile["members"][16].contains("unit"),
              "pile getters identify offsets and units without guessing angle units");
        check(cap["named_values"] == Json({{"cap_type", 2},
                                           {"plan_shape", 1},
                                           {"step_count", 2},
                                           {"top_offset_x", 31},
                                           {"top_offset_y", -32}}) &&
                  cap["members"][0]["enum_label"] == "阶形现浇" &&
                  cap["members"][3]["enum_label"] == "矩形" &&
                  cap["step_heights"] == Json({400, 600}) &&
                  cap["step_heights_order"] == "lower_to_upper",
              "cap dialog bindings preserve lower and upper step order");
        check(cap["reinforcement"][0]["named_values"] == Json({{"rebar_type", 0},
                                                               {"spacing", 175},
                                                               {"diameter", 22},
                                                               {"distribution_width", 1200}}) &&
                  cap["reinforcement"][0]["members"][0]["enum_label"] == "HPB235" &&
                  cap["reinforcement"][1]["members"][0]["enum_label"] == "HRB400" &&
                  cap["reinforcement"][1]["named_values"]["diameter"] == 16,
              "reinforcement diameter and spacing follow native storage, with scoped enum");
        check(pile["identified_member_count"] == 5 && pile["unassigned_member_count"] == 13 &&
                  cap["identified_member_count"] == 5 && cap["unassigned_member_count"] == 3 &&
                  cap["members"][1]["name"].is_null() && cap["members"][1]["value"] == 4321 &&
                  unbase64(cap_result["binary_base64"]) == cap_bytes,
              "cap annotations retain unknown fields and every original byte");
        const char *cap_rebar[] = {"HPB235", "HPB300", "HRB335",  "HRB400", "HRB500",
                                   "CRB550", "CRB600", "HTRB600", "T63"};
        const char *cap_types[] = {"阶形预制", "锥形预制", "阶形现浇", "锥形现浇"};
        const char *cap_shapes[] = {"圆形", "矩形", "正多边形", "多边形"};
        for (std::int32_t v = -1; v <= 14; ++v) {
            auto d = decode_cap(cap_fixture(v, v, v, v, 1, {800})).at("decoded");
            auto enum_is = [&](const Json &field, int count, const std::string &label) {
                return field["value"] == v &&
                       (v >= 0 && v < count
                            ? field["enum_label"] == label && field["enum_status"] == "identified"
                            : field["enum_label"].is_null() &&
                                  field["enum_status"] == "unknown_value");
            };
            check(enum_is(d["pile_section"]["base"]["members"][1], 14,
                          "C" + std::to_string(15 + 5 * v)) &&
                      enum_is(d["reinforcement"][0]["members"][0], 9,
                              v >= 0 && v < 9 ? cap_rebar[v] : "") &&
                      enum_is(d["members"][0], 4, v >= 0 && v < 4 ? cap_types[v] : "") &&
                      enum_is(d["members"][3], 4, v >= 0 && v < 4 ? cap_shapes[v] : ""),
                  "cap enum tables preserve negative and out-of-range codes");
        }
        check(decode_cap(cap_fixture(0, 0, 0, 0, 1, {800}))["decoded"]["step_heights"] ==
                  Json({800}),
              "single cap step retains its height");
        for (auto steps : {0, 1, 3}) {
            auto d = decode_cap(cap_fixture(0, 0, 0, 0, steps, {400, 600})).at("decoded");
            check(!d.contains("step_heights") &&
                      d["step_heights_status"] == "unsupported_step_layout" &&
                      d["integer_array_0x250"] == Json({400, 600}),
                  "unrecognized cap step layout is retained without a semantic alias");
        }
        check(decode_binary_field("BinaryData", cap_bytes, "PBStandardSectionProfile",
                                  {{"Type", 0x40001}})
                      .value("encoding", "") != "pilecap_section_cereal",
              "cap semantics require the native type discriminator");
        cap_bytes[0] = 1;
        check(decode_cap(cap_bytes).value("encoding", "") != "pilecap_section_cereal",
              "unsupported cap cereal version is not accepted");
        cap_bytes[0] = 0;
        cap_bytes.pop_back();
        check(decode_cap(cap_bytes).value("encoding", "") != "pilecap_section_cereal",
              "truncated cap is not accepted");
        NativeScene views;
        auto shared = std::make_shared<GeometryDefinition>();
        shared->source_key = "stream@1";
        shared->geometry = triangle();
        auto independent = std::make_shared<GeometryDefinition>(*shared);
        independent->source_key = "stream@2";
        views.definitions = {shared, independent};
        views.metadata = {
            {"color_tables", Json::array()},
            {"materials",
             {{"definitions",
               Json::array(
                   {{{"scope", "model:13"}, {"id", 7}, {"base_color_rgb", {1., 0., 0.}}},
                    {{"scope", "global"}, {"id", 7}, {"base_color_rgb", {0., 1., 0.}}}})}}}};
        SceneElement ve;
        ve.metadata = {{"model_id", 13}, {"unknown", Json::array()}};
        GeometryInstance vi;
        vi.definition = 0;
        vi.matrix = mirror;
        vi.style = {{"material_id", 7}};
        ve.instances = {vi, vi};
        vi.definition = 1;
        ve.instances.push_back(vi);
        views.elements.push_back(ve);
        std::vector<const Geometry *> borrowed;
        views.for_each_primitive([&](const PrimitiveView &view) {
            borrowed.push_back(view.geometry);
            check(view.material_status == "resolved" &&
                      view.material_candidates == std::vector<std::size_t>{0},
                  "model material takes precedence over global");
            check(view.winding_reversed && view.uv_status == "explicit_source",
                  "primitive mirror and source UV contract");
        });
        check(borrowed.size() == 3 && borrowed[0] == borrowed[1] && borrowed[0] != borrowed[2],
              "native references share arrays; equal independent definitions remain distinct");
        check(views.summary()["reused_definitions"] == 1 &&
                  views.summary()["stored_referenced_triangles"] == 2,
              "native reuse summary counts source identities");
        views.metadata["materials"]["definitions"].push_back(
            views.metadata["materials"]["definitions"][0]);
        views.for_each_primitive([&](const PrimitiveView &view) {
            check(view.material_status == "ambiguous" &&
                      view.material_candidates == std::vector<std::size_t>({0, 2}),
                  "ambiguous local materials not replaced by a global candidate");
        });
        shared->geometry.primitive_ranges[0]["count"] = 2;
        rejects([&]() { views.for_each_primitive([](const PrimitiveView &) {}); },
                "out of bounds primitive range rejected");
        std::cout << checks << " checks passed\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
    return 0;
}
