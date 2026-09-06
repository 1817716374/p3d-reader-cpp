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
