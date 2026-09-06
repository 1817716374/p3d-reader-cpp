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
static Bytes drawing_fixture(bool substation, unsigned version) {
    Bytes b;
    put<std::uint32_t>(b, 1);
    for (int i = 0; i < 13; ++i)
        put<std::uint64_t>(b, 0);
    put<std::uint32_t>(b, 0);
    auto ids =
        substation ? std::vector<int>{0, 1, 2, 3, 4, 5, 6, 7} : std::vector<int>{4, 5, 3, 6, 7};
    for (std::size_t index = 0; index < ids.size(); ++index) {
        int id = ids[index];
        put<std::uint32_t>(b, 0);
        auto v = !substation && id != 6 ? version : 0;
        put<std::uint32_t>(b, v);
        if (!index)
            put<std::uint32_t>(b, 0);
        put<std::uint32_t>(b, 0);
        put<std::uint32_t>(b, id);
        put<std::uint32_t>(b, 2);
        if (substation) {
            put<double>(b, 901);
            put<std::uint32_t>(b, 902);
        }
        for (double x : {101., 102., 103.})
            put(b, x);
        if (substation) {
            auto n = id == 2 || id == 3 ? 4 : id == 7 ? 3 : 2;
            for (int i = 0; i < n; ++i)
                put<double>(b, 201 + i);
        } else {
            put<std::uint32_t>(b, 3);
            if (id == 4 || id == 5) {
                put<std::uint32_t>(b, 1);
                if (id == 5)
                    put<std::uint32_t>(b, 2);
                for (int i = 0; i < (id == 4 ? 4 : 3); ++i)
                    put<double>(b, 201 + i);
                for (auto x : id == 4 ? Bytes{1, 0, 1, 0} : Bytes{0, 1})
                    put(b, x);
                if (id == 5) {
                    put<double>(b, 205);
                    put<std::uint8_t>(b, 1);
                }
            } else if (id == 3) {
                put<std::uint8_t>(b, 1);
                put<double>(b, 206);
            } else {
                put<std::uint8_t>(b, 0);
                put<std::uint8_t>(b, 1);
                if (id == 7)
                    put<std::uint8_t>(b, 0);
            }
            if (v) {
                if (id == 7)
                    put<std::uint8_t>(b, 0);
                else
                    put<std::uint32_t>(b, 1);
            }
        }
    }
    return b;
}
int main() {
    try {
        for (unsigned version : {0u, 1u}) {
            auto payload = drawing_fixture(false, version);
            auto decoded = decode_binary_field("CerealDatas", payload, "PSDrawingManager");
            auto &sheets = decoded.at("decoded").at("records");
            auto &plan = sheets[0]["named_values"];
            check(plan["scale_denominator"] == 101 &&
                      plan["connection_number_text_height"] == 103 &&
                      plan["member_number_text_height"] == 203 && plan["beam_end_gap"] == 204,
                  "plan scales and text heights follow native members");
            check(plan["draw_column_leader"] == 1 && plan["draw_section_table"] == 0 &&
                      plan["merged_output"] == 0 && plan["beam_drawing_mode"] == 1,
                  "native drawing flags are not UI combo indices");
            auto &elevation = sheets[1]["named_values"];
            check(elevation["column_drawing_mode"] == 2 && elevation["beam_end_gap"] == 205 &&
                      elevation["show_member_numbers"] == 1 && elevation["merged_output"] == 1,
                  "elevation parameters preserve noncontiguous native layout");
            check(sheets[3]["named_values"]["leader_text_height"] == 102 &&
                      sheets[4]["named_values"]["connection_drawing_type"] == 1 &&
                      sheets[4]["named_values"]["generate_connection_drawing"] == 0,
                  "node and connection settings have distinct meanings");
            check(sheets[0].contains("node_number_mode_code") == (version == 1) &&
                      sheets[4].contains("splice_annotation_origin_flag") == (version == 1),
                  "drawing version-specific parameters are only present in their archive version");
            if (version == 1)
                check(plan["node_number_mode"] == 1 &&
                          sheets[2]["named_values"]["node_number_mode"] == 1 &&
                          sheets[4]["named_values"]["splice_annotation_origin"] == 0,
                      "drawing version 1 tail fields decoded");
            payload.pop_back();
            check(!decode_binary_field("CerealDatas", payload, "PSDrawingManager")
                       .contains("encoding"),
                  "truncated drawing tail is not accepted");
        }
        auto subs =
            decode_binary_field("CerealDatas", drawing_fixture(true, 0), "SubsDrawingManager")
                .at("decoded");
        for (auto &sheet : subs["records"])
            check(sheet["named_values"]["dimension_text_height"] == 101 &&
                      sheet["named_values"]["name_text_height"] == 102 &&
                      sheet["named_values"]["table_text_height"] == 103 &&
                      sheet["unassigned_scale"] == 901,
                  "substation text heights are independent of the retained base double");
        check(subs["records"][2]["named_values"]["section_scale_denominator"] == 203 &&
                  subs["records"][3]["named_values"]["connection_scale_denominator"] == 204 &&
                  subs["records"][7]["named_values"]["elevation_scale_denominator"] == 201,
              "substation per-type scales retain native ordering");
        auto drawing_unknown = drawing_fixture(false, 0);
        // First sheet: 112-byte manager prefix, 16-byte cereal/base header.
        std::uint32_t unknown_paper = 99;
        std::memcpy(drawing_unknown.data() + 132, &unknown_paper, sizeof(unknown_paper));
        drawing_unknown[200] = 7;
        auto unknown_sheet = decode_binary_field("CerealDatas", drawing_unknown, "PSDrawingManager")
                                 .at("decoded")
                                 .at("records")[0];
        check(unknown_sheet["fields"][0]["enum_status"] == "unknown_value" &&
                  unknown_sheet["named_values"]["paper_size"] == 99 &&
                  unknown_sheet["named_values"]["draw_column_leader"] == 7,
              "unknown drawing enums and flag bytes remain lossless");
        std::uint32_t future_sheet_version = 2;
        std::memcpy(drawing_unknown.data() + 116, &future_sheet_version,
                    sizeof(future_sheet_version));
        check(!decode_binary_field("CerealDatas", drawing_unknown, "PSDrawingManager")
                   .contains("encoding"),
              "unsupported future drawing version is not decoded as version zero");
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
                                            {"section_shape", 1},
                                            {"section_width", 702},
                                            {"section_height", 701},
                                            {"pile_length", 10.5},
                                            {"x_offset", 111},
                                            {"y_offset", -222},
                                            {"rotation", 12.5},
                                            {"top_elevation", 9.75}}) &&
                  pile["members"][1]["enum_label"] == "C30" &&
                  pile["members"][14]["unit"] == "mm" && pile["members"][17]["unit"] == "m" &&
                  pile["members"][16]["unit"] == "deg",
              "pile getters and native angle conversion identify units");
        check(cap["named_values"] == Json({{"cap_type", 2},
                                           {"layout_preset_index", 4321},
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
        check(pile["identified_member_count"] == 9 && pile["unassigned_member_count"] == 9 &&
                  cap["identified_member_count"] == 6 && cap["unassigned_member_count"] == 2 &&
                  cap["members"][7]["name"].is_null() && cap["members"][7]["value"] == 126 &&
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
        for (std::int32_t shape : {-1, 0, 2, 3}) {
            auto b = cap_fixture(3, 0, 2, 1, 2, {400, 600});
            std::memcpy(b.data() + 24, &shape, sizeof(shape));
            const auto base = decode_cap(b)["decoded"]["pile_section"]["base"];
            const auto &values = base["named_values"];
            check(!values.contains("section_width") && !values.contains("section_height") &&
                      (shape == 2 ? values["section_diameter"] == 701 &&
                                        base["members"][0]["enum_label"] == "圆形" &&
                                        base["members"][3]["name"].is_null()
                                  : !values.contains("section_diameter") &&
                                        base["members"][0]["enum_label"].is_null()),
                  "pile dimensions are scoped to the source shape, with unknown codes retained");
        }
        auto cap_geometry = [&](int shape, int count, int edges, const Json &positions,
                                const Json &profiles) {
            auto raw = cap_fixture(3, 0, 2, shape, 2, {400, 600});
            Bytes b = slice(raw, 0, 160);
            auto points = [&](const Json &array) {
                put(b, std::uint64_t(array.size()));
                for (const auto &p : array) {
                    put(b, std::uint64_t(3));
                    for (const auto &v : p)
                        put(b, v.get<double>());
                }
            };
            put(b, std::int32_t(count));
            points(positions);
            put(b, std::int32_t(edges));
            put(b, std::uint64_t(profiles.size()));
            for (const auto &profile : profiles)
                points(profile);
            auto tail = slice(raw, 184, raw.size() - 184);
            b.insert(b.end(), tail.begin(), tail.end());
            return decode_cap(b);
        };
        const Json positions = {{-500., 100., 0.}, {500., -100., 2.}};
        const Json contours = {{{-900., -800., 0.}, {900., -800., 0.}, {0., 800., 0.}},
                               {{-600., -500., 0.}, {600., -500., 0.}, {0., 500., 0.}}};
        auto parameterized = cap_geometry(3, 2, 3, positions, contours)["decoded"];
        check(parameterized["pile_layout"]["positions"] == positions &&
                  parameterized["pile_layout"]["count_status"] == "consistent" &&
                  parameterized["cap_profiles"]["step_count_status"] == "consistent" &&
                  parameterized["cap_profiles"]["profiles"][0]["vertices"] == contours[0] &&
                  parameterized["cap_profiles"]["profiles"][1]["vertices"] == contours[1],
              "pile locations and lower-to-upper cap polygons remain distinct");
        const Json circles = {{{1200., 0., 0.}}, {{800., 0., 0.}}};
        auto circular = cap_geometry(0, 2, 0, positions, circles)["decoded"];
        check(circular["cap_profiles"]["profiles"][0]["kind"] == "circle" &&
                  circular["cap_profiles"]["profiles"][0]["radius"] == 1200. &&
                  circular["cap_profiles"]["profiles"][1]["radius"] == 800. &&
                  !circular["cap_profiles"]["profiles"][0].contains("vertices") &&
                  circular["secondary_point_arrays"] == circles,
              "native circular radius tuples are not interpreted as polygon vertices");
        auto future_circle = circles;
        future_circle[0][0][1] = 3.;
        auto unknown_profile = cap_geometry(0, 2, 0, positions, future_circle)["decoded"];
        check(unknown_profile["cap_profiles"]["profiles"][0]["kind"] == "unassigned" &&
                  unknown_profile["cap_profiles"]["profiles"][0]["source_values"] ==
                      future_circle[0],
              "unrecognized circular tuple keeps every value without guessing its layout");
        auto mismatch = cap_geometry(3, -1, 4, positions, Json({contours[0]}))["decoded"];
        check(mismatch["pile_layout"]["count_status"] == "mismatch" &&
                  mismatch["cap_profiles"]["step_count_status"] == "mismatch" &&
                  mismatch["cap_profiles"]["profiles"][0]["edge_count_status"] == "mismatch" &&
                  mismatch["pile_layout"]["positions"] == positions,
              "inconsistent source counts are reported without truncation or synthesis");
        auto future_shape = cap_geometry(99, 2, 3, positions, contours)["decoded"];
        check(future_shape["cap_profiles"]["profiles"][0]["kind"] == "unassigned" &&
                  future_shape["cap_profiles"]["profiles"][0]["source_values"] == contours[0],
              "unknown cap shapes retain their raw profile arrays");
        auto link_fixture = [](int version, const std::vector<std::vector<int>> &floors,
                               const std::vector<std::vector<int>> &links,
                               const std::vector<int> &codes) {
            Bytes b;
            for (int v : {version, 0, 0, codes.at(0), codes.at(1)})
                put(b, std::int32_t(v));
            bool first_group = true;
            for (auto *groups : {&floors, &links}) {
                put(b, std::uint64_t(groups->size()));
                for (const auto &values : *groups) {
                    if (first_group) {
                        put(b, std::uint32_t(0));
                        first_group = false;
                    }
                    put(b, std::uint32_t(0));
                    put(b, std::uint64_t(values.size()));
                    for (int value : values)
                        put(b, std::int32_t(value));
                }
            }
            for (int v : {0, 0, codes.at(2), codes.at(3), codes.at(4), codes.at(5)})
                put(b, std::int32_t(v));
            put(b, std::uint64_t(3));
            for (auto v : {0, 1, 0})
                put(b, std::uint8_t(v));
            if (version == 1)
                put(b, std::int64_t(1234567890123));
            return b;
        };
        const std::vector<int> link_codes{1, 3, 2, 4, 5, 3};
        auto link_bytes = link_fixture(1, {{7, 3, 7}, {}}, {{-1, 2147483647}}, link_codes);
        auto link = decode_binary_field("CerealDatas", link_bytes, "AssemblyLinkModel");
        check(link.value("encoding", "") == "assembly_settings_cereal",
              "nonempty native link merge groups are decoded");
        const auto &ld = link["decoded"];
        const auto &lg = ld["link_merge_set"]["collections"];
        check(lg[0]["name"] == "floor_groups" && lg[1]["name"] == "link_groups" &&
                  lg[0]["entries"][0]["values"] == Json({7, 3, 7}) &&
                  lg[0]["entries"][1]["values"] == Json::array() &&
                  lg[1]["entries"][0]["values"] == Json({-1, 2147483647}),
              "merge groups preserve signed values, duplicates and source ordering");
        check(lg[0]["entries"][0]["cereal_version"] == 0 &&
                  !lg[0]["entries"][1].contains("cereal_version") &&
                  !lg[1]["entries"][0].contains("cereal_version") &&
                  ld["boolean_vector"] == Json({0, 1, 0}) &&
                  ld["storey_design_flags"]["source_values"] == Json({0, 1, 0}) &&
                  ld["load_g_para_id"] == 1234567890123LL &&
                  unbase64(link["binary_base64"]) == link_bytes,
              "group type version is shared across collections and trailing data is preserved");
        check(ld["link_merge_set"]["fields"][0]["enum_label"] == "1/1,2/1,1/2,3/2..." &&
                  ld["link_merge_set"]["fields"][1]["enum_label"] == "任选楼层节点连接归并" &&
                  ld["display_control"]["fields"][0]["enum_label"] == "半透明显示" &&
                  ld["display_control"]["fields"][1]["enum_label"] == "精细显示-带焊接标记" &&
                  ld["display_control"]["fields"][2]["enum_label"] == "简化显示" &&
                  ld["display_control"]["fields"][3]["enum_label"] == "精细显示",
              "native merge and display enums use their field-specific mappings");
        auto second_first =
            decode_binary_field("CerealDatas", link_fixture(0, {}, {{42}, {9}}, link_codes),
                                "AssemblyLinkModel")["decoded"];
        check(second_first["link_merge_set"]["collections"][1]["entries"][0]["cereal_version"] ==
                      0 &&
                  second_first["link_merge_set"]["collections"][1]["entries"][1]["values"] ==
                      Json({9}) &&
                  !second_first.contains("load_g_para_id"),
              "first nonempty merge collection carries the shared cereal type version");
        auto unknown_link =
            decode_binary_field("CerealDatas", link_fixture(0, {}, {}, {-1, 99, 3, 2, 4, 0}),
                                "AssemblyLinkModel")["decoded"];
        bool unknown_modes = true;
        for (const auto *block : {"link_merge_set", "display_control"})
            for (const auto &f : unknown_link[block]["fields"])
                unknown_modes &= f["enum_status"] == "unknown_value" && f["enum_label"].is_null();
        check(unknown_modes && unknown_link["display_control"]["enum_codes"] == Json({3, 2, 4, 0}),
              "unrecognized display and merge enum values are not coerced");
        for (auto offset : {28, 32}) {
            auto unsupported = link_bytes;
            unsupported[offset] = 1;
            check(decode_binary_field("CerealDatas", unsupported, "AssemblyLinkModel")
                          .value("encoding", "") != "assembly_settings_cereal",
                  "unsupported merge group cereal or payload version is rejected");
        }
        auto truncated_group = slice(link_bytes, 0, 46);
        check(decode_binary_field("CerealDatas", truncated_group, "AssemblyLinkModel")
                      .value("encoding", "") != "assembly_settings_cereal",
              "truncated group members do not produce a partial success");
        auto oversized_group = link_bytes;
        for (auto offset = 36; offset < 44; ++offset)
            oversized_group[offset] = 0xff;
        check(decode_binary_field("CerealDatas", oversized_group, "AssemblyLinkModel")
                      .value("encoding", "") != "assembly_settings_cereal",
              "group member count is bounded by remaining input");
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
