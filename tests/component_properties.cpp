#include "component_properties.hpp"
#include <cstring>

unsigned component_property_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    constexpr std::uint64_t U = 0x2477702224190092ULL, S = 0x1316456973520092ULL;
    constexpr std::uint64_t P = 0x0141762724222207ULL, N = 0x0260975554222207ULL;
    constexpr std::uint64_t B = 0x0059665104550025ULL, V = 0x3131099204415903ULL;
    constexpr std::uint64_t M = 0x3131099213160368ULL, F = 0x0181635124290115ULL;
    auto qword = [](Bytes &b, std::uint64_t v) {
        for (unsigned i = 0; i < 8; ++i)
            b.push_back(std::uint8_t(v >> (i * 8)));
    };
    auto frame = [&](std::uint64_t id, Bytes b) {
        const auto size = b.size();
        qword(b, size);
        qword(b, id);
        return b;
    };
    auto integer = [&](std::uint64_t n, std::uint64_t id = 0x2477702224190092ULL) {
        Bytes b;
        qword(b, n);
        return frame(id, b);
    };
    auto string = [&](std::string s) { return frame(S, Bytes(s.begin(), s.end())); };
    // List arguments are in reader order. A stack places the first requested value last.
    auto stack = [](const std::vector<Bytes> &values) {
        Bytes b;
        for (auto i = values.rbegin(); i != values.rend(); ++i)
            b.insert(b.end(), i->begin(), i->end());
        return b;
    };
    auto property = [&](Bytes value, std::vector<std::pair<Bytes, Bytes>> attributes = {}) {
        std::vector<Bytes> items{value, integer(attributes.size())};
        for (auto &a : attributes) {
            items.push_back(a.first);
            items.push_back(a.second);
        }
        return frame(P, stack(items));
    };
    auto object = [&](std::vector<std::pair<Bytes, Bytes>> properties) {
        std::vector<Bytes> items{integer(properties.size())};
        for (auto &p : properties) {
            items.push_back(p.first);
            items.push_back(p.second);
        }
        return frame(N, stack(items));
    };
    auto decode = [&](const Bytes &b) {
        return decode_binary_field("ParaCmptProperty", b).at("decoded");
    };
    auto one = [&](const Bytes &v) { return decode(object({{string("value"), property(v)}})); };
    auto value = [&](const Bytes &v) {
        return one(v).at("root").at("entries").at(0).at("value").at("value");
    };

    // Literal wire fixture: a zero uint64 count followed by the Noumenon footer.
    const Bytes empty = {0, 0, 0,    0, 0,    0,    0,    0,    8,    0,    0,    0, 0, 0,
                         0, 0, 0x92, 0, 0x19, 0x24, 0x22, 0x70, 0x77, 0x24, 24,   0, 0, 0,
                         0, 0, 0,    0, 7,    0x22, 0x22, 0x54, 0x55, 0x97, 0x60, 2};
    auto result = decode(empty);
    check(result.at("status") == "decoded" && result.at("root").at("entries").empty(),
          "literal empty component stack");
    check(result.at("component_graphics_input").at("status") == "absent",
          "empty graph source absent");
    check(value(integer(UINT64_MAX)).at("value").get<std::uint64_t>() == UINT64_MAX,
          "unsigned width retained");
    check(value(integer(UINT64_MAX, 0x4569571470222419ULL)).at("value").get<std::int64_t>() == -1,
          "signed value remains signed");
    check(value(frame(0x1580142273520992ULL, {2})).at("value") == true,
          "bool byte preserved as nonzero input");
    check(value(frame(0x4500450011720048ULL, {0x31})).at("value").is_null(), "None is not boolean");
    check(value(frame(0x4500450011720048ULL, {0x31})).at("storage_uint8") == 0x31,
          "None storage preserved");
    check(value(frame(0x7175318778202422ULL, {0, 0, 0, 0, 0, 0, 0xf4, 0x3f})).at("value") == 1.25,
          "double value");
    check(value(string(std::string("a\0b", 3))).at("text") == std::string("a\0b", 3),
          "narrow string embedded NUL retained");
    check(!value(frame(S, {0xff, 0xfe})).contains("text"), "unknown code page not guessed");
    check(value(frame(B, {0xff, 0, 0x42})).at("bytes_base64") == "/wBC",
          "bytes not interpreted as text");
    check(value(frame(0x4827000104282422ULL, stack({string("mod"), string("op")})))
                  .at("module_name")
                  .at("text") == "mod",
          "unified function module order");
    check(value(frame(0x4827000104282422ULL, stack({string("mod"), string("op")})))
                  .at("operator_name")
                  .at("text") == "op",
          "unified function operator order");

    const auto vector = frame(V, stack({integer(3), integer(9), string("x"), object({})}));
    auto list = value(vector).at("entries");
    check(list.size() == 3 && list[0].at("value") == 9 && list[1].at("text") == "x" &&
              list[2].at("kind") == "noumenon",
          "heterogeneous vector order");
    auto map =
        value(frame(M, stack({integer(2), string("b"), integer(4), string("a"), integer(5)})));
    check(map.at("entries")[0].at("key").at("text") == "b" &&
              map.at("entries")[1].at("value").at("value") == 5,
          "map stored entries retained without sorting or deduplication");
    check(map.at("runtime_key_comparison") == "not_evaluated", "map semantics not fabricated");
    auto nested =
        object({{string("child"),
                 property(object({{string("inside"), property(string("v"))}}),
                          {{string("show"), property(frame(0x1580142273520992ULL, {1}))}})}});
    result = decode(nested);
    const auto &prop = result.at("root").at("entries")[0].at("value");
    check(prop.at("attributes")[0].at("key").at("text") == "show",
          "property attributes are distinct from value");
    check(prop.at("value").at("entries")[0].at("key").at("text") == "inside",
          "nested Noumenon preserved");

    // Folders are unframed recursion inside the DependentFile payload. True denotes folder.
    auto files =
        frame(F, stack({integer(2), string("dir"), frame(0x1580142273520992ULL, {1}), integer(1),
                        string("file.bin"), frame(0x1580142273520992ULL, {0}), frame(B, {1, 2, 3}),
                        string("empty"), frame(0x1580142273520992ULL, {1}), integer(0)}));
    auto folder = value(files).at("root");
    check(folder.at("entries").size() == 2, "dependent root members");
    check(folder.at("entries")[0].at("value").at("entries")[0].at("value").at("bytes_base64") ==
              "AQID",
          "dependent file data remains in memory");
    check(folder.at("entries")[1].at("value").at("entries").empty(),
          "empty dependent folder retained");

    auto sources = object(
        {{string("componentGraphics"), property(frame(B, {}))},
         {string("componentGraphics"), property(frame(B, {1}))},
         {string("\x07_component_material"), property(string(std::string("wood\0suffix", 11)))}});
    result = decode(sources);
    check(result.at("component_graphics_input").at("property_index") == 0 &&
              result.at("component_graphics_input").at("bytes") == 0,
          "first duplicate graphics property including empty bytes");
    check(result.at("component_material_input").at("property_index") == 2 &&
              result.at("component_material_input").at("bytes") == 11,
          "material source bytes retained");
    result = decode(object({{string("componentGraphics"), property(string("wrong"))},
                            {string("componentGraphics"), property(frame(B, {}))}}));
    check(result.at("component_graphics_input").at("status") == "wrong_value_type",
          "wrong first value type does not select later duplicate");
    check(result.at("active_graphics_selection") == "not_established",
          "stored graph input is not active scene selection");

    Bytes graphics(142, 0);
    graphics[0] = 1; // valid empty BPGraphics, without serialized model
    result = decode(object({{string("componentGraphics"), property(frame(B, graphics))}}));
    check(result.at("component_graphics_input").contains("graphics"),
          "component graphics delegates to existing decoder");
    check(result.at("component_graphics_input").at("graphics").at("geometry_packets").empty(),
          "empty graphics does not invent entries");
    result = decode(object({{string("componentGraphics"), property(frame(B, {1, 2, 3}))}}));
    check(result.at("component_graphics_input").contains("decode_error") &&
              result.at("status") == "decoded",
          "invalid graph distinguished from property decoding");

    result = one(frame(0x12345, {1, 2, 3}));
    check(result.at("status") == "partial", "unknown registered type is not semantic completion");
    check(result.at("root").at("entries")[0].at("value").at("value").at("payload_base64") == "AQID",
          "unknown value payload retained");
    for (std::size_t i = 0; i < empty.size(); ++i) {
        Bytes truncated(empty.begin(), empty.begin() + i);
        check(decode(truncated).at("status") == "invalid", "truncated root refused");
    }
    auto bad = empty;
    bad[24] = 255;
    check(decode(bad).at("status") == "invalid", "root size cannot escape source");
    check(decode(frame(N, integer(UINT64_MAX))).at("status") == "invalid",
          "hostile count bounded before allocation");
    check(decode(frame(N, integer(0, 0x4569571470222419ULL))).at("status") == "invalid",
          "signed count is not uint64 count");
    check(one(frame(0x4569571470222419ULL, {1})).at("status") == "partial",
          "truncated scalar preserved without unsafe read");
    auto leading = empty;
    leading.insert(leading.begin(), {0xab, 0xcd});
    result = decode(leading);
    check(result.at("consumed_bytes") == empty.size() &&
              result.at("leading_bytes").at("bytes") == 2,
          "outer offset consumption and leading bytes");
    auto inner = integer(0);
    inner.insert(inner.begin(), {0xab});
    check(decode(frame(N, inner)).at("status") == "partial", "unread container prefix is reported");
    Bytes deep = integer(0);
    for (unsigned i = 0; i < 100; ++i)
        deep = frame(V, stack({integer(1), deep}));
    check(one(deep).at("status") == "partial", "recursive input budget produces incomplete result");

    constexpr std::uint64_t I = 0x4569571470222419ULL, K = 0x1395017306582671ULL;
    constexpr std::uint64_t KS = 0x4809735200591395ULL, KU = 0x1311011548825714ULL;
    auto data_key = [&](std::int64_t cl, std::int64_t object_id) {
        return frame(K,
                     stack({integer(std::uint64_t(object_id), I), integer(std::uint64_t(cl), I)}));
    };
    auto unit = [&](const Bytes &b) { return decode_binary_field("DataUnit", b).at("decoded"); };
    auto key = value(data_key(-2, INT64_MIN));
    check(key.at("kind") == "data_key" && key.at("class_id") == -2 &&
              key.at("object_id").get<std::int64_t>() == INT64_MIN,
          "DataKey class and object roles retain signed width");
    check(key.at("object_id_source").at("offset") > key.at("class_id_source").at("offset"),
          "DataKey native object-first reading reverses physical stack order");
    const auto keyed =
        frame(KU, stack({integer(5), data_key(2, 3), integer(8), data_key(-1, 4), integer(9),
                         data_key(2, -5), integer(UINT64_MAX), data_key(2, 3), integer(10),
                         data_key(-1, INT64_MIN), integer(11)}));
    auto root = unit(keyed).at("root");
    check(root.at("kind") == "data_key_uint64_map" && root.at("entries").size() == 5,
          "typed DataKey map preserves duplicates");
    check(root.at("entries")[0].at("key").at("object_id") == 3 &&
              root.at("entries")[0].at("value").at("value") == 8,
          "DataUnit map key and value follow native read order");
    check(root.at("entries")[2].at("value").at("value").get<std::uint64_t>() == UINT64_MAX,
          "DataUnit map values remain unsigned");
    check(root.at("native_index").at("selected_entry_indices") == Json::array({4, 1, 2, 3}),
          "DataKey map uses signed class/object ordering and last duplicate");
    check(value(keyed) == unit(keyed).at("root") ||
              value(keyed).at("native_index") == root.at("native_index"),
          "component properties share registered DataKey map semantics");
    root = unit(frame(KS, stack({integer(2), data_key(1195, 49), string("first"),
                                 data_key(1195, 49), string(std::string("last\0x", 6))})))
               .at("root");
    check(root.at("entries")[0].at("key").at("class_id") == 1195 &&
              root.at("entries")[0].at("value").at("text") == "first",
          "DataKey-to-string roles are not reversed");
    check(root.at("native_index").at("selected_entry_indices") == Json::array({1}) &&
              root.at("entries")[1].at("value").at("text") == std::string("last\0x", 6),
          "string assignment uses last value including embedded NUL");
    check(unit(integer(UINT64_MAX, I)).at("root").at("value") == -1,
          "DataUnit signed root is not coerced to uint64");
    for (auto id : {KS, KU}) {
        check(unit(frame(id, integer(0)))
                  .at("root")
                  .at("native_index")
                  .at("selected_entry_indices")
                  .empty(),
              "empty typed DataKey map");
        result = unit(frame(id, stack({integer(1), string("wrong-key"), integer(0)})));
        check(result.at("status") == "invalid" && !result.at("root").contains("native_index"),
              "wrong typed map key cannot produce a native index");
        result = unit(frame(id, stack({integer(1), data_key(1, 2), frame(V, integer(0))})));
        check(result.at("status") == "invalid" && !result.at("root").contains("native_index"),
              "wrong typed map value cannot produce a native index");
    }
    check(unit(frame(K, stack({integer(1), integer(2, I)}))).at("status") == "invalid",
          "DataKey requires signed object ID type");
    check(unit(frame(K, stack({integer(1, I), integer(2)}))).at("status") == "invalid",
          "DataKey requires signed class ID type");
    auto extra = keyed;
    extra.insert(extra.begin(), 0xaa);
    check(unit(extra).at("status") == "partial",
          "DataUnit leading bytes are not complete decoding");
    check(unit(frame(0x1234, {1})).at("status") == "partial", "unknown DataUnit type retained");

    constexpr std::uint64_t MAT = 0x2624634702211655ULL;
    auto real = [&](double number) {
        Bytes b(sizeof(number));
        std::memcpy(b.data(), &number, sizeof(number));
        return frame(0x7175318778202422ULL, b);
    };
    // Independent wire order: flags, six factors, RGB/factors, UV y/x, enums, strings.
    std::vector<Bytes> material_members;
    for (unsigned i = 0; i < 13; ++i)
        material_members.push_back(frame(0x1580142273520992ULL, {std::uint8_t(i % 2)}));
    for (unsigned i = 0; i < 23; ++i)
        material_members.push_back(real(0.25 + i));
    material_members.push_back(integer(0x100000006ULL, I));
    material_members.push_back(integer(UINT64_MAX, I));
    material_members.push_back(string(std::string("a.jpg\0trailing", 14)));
    material_members.push_back(frame(S, {0xc4, 0xbe, 0, 'x'}));
    const auto material_bytes = frame(MAT, stack(material_members));
    result = unit(material_bytes);
    root = result.at("root");
    check(result.at("status") == "decoded" && root.at("kind") == "material",
          "registered material complete stored payload");
    auto fields = root.at("fields");
    const char *flags[] = {"is_valid",
                           "has_transparency",
                           "has_specular_factor",
                           "has_specular_color",
                           "has_roughness_factor",
                           "has_refract_factor",
                           "has_reflect_factor",
                           "has_map",
                           "has_glow_factor",
                           "has_glow_color",
                           "has_diffuse_factor",
                           "has_color",
                           "has_ambient_factor"};
    for (unsigned i = 0; i < 13; ++i)
        check(fields.at(flags[i]).at("value") == bool(i % 2), "material flag wire order");
    const char *factors[] = {"refract_factor", "reflect_factor", "roughness_factor",
                             "diffuse_factor", "ambient_factor", "glow_factor"};
    for (unsigned i = 0; i < 6; ++i)
        check(fields.at(factors[i]).at("value") == 0.25 + i,
              "material factors retained including disabled and out-of-range inputs");
    check(fields.at("glow_color").at("value") == Json::array({6.25, 7.25, 8.25}),
          "material glow RGB read order");
    check(fields.at("specular_factor").at("value") == 9.25 &&
              fields.at("specular_color").at("value") == Json::array({10.25, 11.25, 12.25}),
          "material specular factor and RGB");
    check(fields.at("transparency").at("value") == 13.25 &&
              fields.at("color").at("value") == Json::array({14.25, 15.25, 16.25}),
          "material transparency and RGB");
    check(fields.at("bump_factor").at("value") == 17.25 &&
              fields.at("w_rotation").at("value") == 18.25 &&
              fields.at("w_rotation").at("unit") == "degrees",
          "material bump and rotation");
    check(fields.at("uv_offset").at("value") == Json::array({20.25, 19.25}) &&
              fields.at("uv_scale").at("value") == Json::array({22.25, 21.25}),
          "material UV read y before x");
    check(fields.at("uv_scale").at("components")[0].at("offset") <
              fields.at("uv_scale").at("components")[1].at("offset"),
          "material vector components retain exact source positions");
    check(fields.at("map_mode").at("value") == 0x100000006LL &&
              fields.at("map_mode").at("runtime_value_int32") == 6 &&
              fields.at("map_unit").at("runtime_value_int32") == -1,
          "material enums retain int64 source and native signed low32");
    check(fields.at("map_file").at("text") == std::string("a.jpg\0trailing", 14) &&
              fields.at("map_file").at("native_string_input").at("text") == "a.jpg",
          "material string source and native NUL termination differ");
    check(fields.at("name").at("native_string_input").at("bytes") == 2 &&
              !fields.at("name").at("native_string_input").contains("text") &&
              fields.at("name").at("native_string_input").at("encoding") ==
                  "windows_ansi_code_page",
          "material ANSI text not guessed as UTF8");
    check(root.at("not_serialized_fields").size() == 6 && fields.size() == 32 &&
              !fields.contains("material_id") && !fields.contains("pbr_maps"),
          "registered material does not invent omitted SDK members");
    check(one(material_bytes).at("status") == "decoded", "material nested in component property");
    for (std::size_t i = 0; i < material_members.size(); ++i) {
        auto broken = material_members;
        broken.resize(i);
        check(unit(frame(MAT, stack(broken))).at("status") == "invalid",
              "registered material missing trailing member rejected");
    }
    for (auto index : {0, 13, 36, 38}) {
        auto wrong = material_members;
        wrong[index] = frame(B, {1, 2, 3});
        check(unit(frame(MAT, stack(wrong))).at("status") == "invalid",
              "registered material rejects wrong flag, number, enum or string type");
    }
    auto material_extra = stack(material_members);
    material_extra.insert(material_extra.begin(), 0xff);
    check(unit(frame(MAT, material_extra)).at("status") == "partial",
          "registered material does not ignore unexplained prefix");

    constexpr std::uint64_t MODEL = 0x0956146148825714ULL, ENTITY = 0x1395755506582671ULL;
    constexpr std::uint64_t COLOR = 0x4767484556636631ULL;
    const std::pair<std::uint64_t, std::int64_t> model_cases[] = {
        {0, 0},
        {1, 1},
        {0xfffffffffffffffeULL, -2},
        {0x180000000ULL, INT32_MIN},
        {0xffffffff7fffffffULL, INT32_MAX},
        {0x10000002aULL, 42}};
    for (const auto &item : model_cases) {
        root = unit(frame(MODEL, integer(item.first, I))).at("root");
        check(root.at("kind") == "model_id" && root.at("model_id") == item.second,
              "registered model ID uses signed low32");
    }
    const auto entity_bytes =
        frame(ENTITY, stack({integer(UINT64_MAX, I), integer(0x100000007ULL, I)}));
    root = unit(entity_bytes).at("root");
    check(root.at("model_id") == 7 && root.at("entity_id").get<std::uint64_t>() == UINT64_MAX,
          "entity ID stores unsigned64 and model ID signed32");
    check(root.at("entity_id_source").at("value") == -1 &&
              root.at("model_id_source").at("value") == 0x100000007LL,
          "entity ID preserves distinct signed wire values");
    check(root.at("entity_id_source").at("offset") > root.at("model_id_source").at("offset"),
          "entity ID read before model ID on stack");
    check(one(entity_bytes).at("status") == "decoded", "entity identifier in nested property");
    check(unit(frame(MODEL, integer(1))).at("status") == "invalid",
          "model identifier rejects unsigned wire type");
    check(unit(frame(ENTITY, stack({integer(1), integer(2, I)}))).at("status") == "invalid",
          "entity identifier requires signed wire type despite unsigned runtime storage");
    check(unit(frame(ENTITY, integer(1, I))).at("status") == "invalid",
          "entity identifier requires both source members");

    const auto color_bytes = frame(COLOR, stack({real(.5), real(1), real(.25), real(0)}));
    root = unit(color_bytes).at("root");
    check(root.at("kind") == "color" && root.at("rgba_uint8") == Json::array({0, 63, 255, 127}),
          "color ABGR native read order becomes RGBA without rounding");
    check(root.at("components")[0].at("value") == 0 && root.at("components")[3].at("value") == .5 &&
              root.at("components")[0].at("offset") < root.at("components")[3].at("offset"),
          "color normalized sources and positions retain component association");
    root = unit(frame(COLOR, stack({real(2), real(-1), real(-.5), real(0.5)}))).at("root");
    check(root.at("rgba_uint8") == Json::array({127, 129, 1, 254}),
          "native color conversion truncates toward zero then wraps low byte without clamping");
    check(one(color_bytes).at("status") == "decoded", "color in nested component property");
    const auto nan = frame(0x7175318778202422ULL, {0x42, 0, 0, 0, 0, 0, 0xf8, 0x7f});
    const auto inf = frame(0x7175318778202422ULL, {0, 0, 0, 0, 0, 0, 0xf0, 0x7f});
    for (const auto &bad_channel : {nan, inf, real(1e300), real(-1e300)}) {
        result = unit(frame(COLOR, stack({real(1), real(0), real(0), bad_channel})));
        root = result.at("root");
        check(result.at("status") == "partial" && root.at("rgba_uint8")[0].is_null() &&
                  root.at("rgba_uint8")[3] == 255,
              "invalid int32 color conversion cannot fabricate a normal byte");
        check(root.at("components")[0].contains("bits_hex") &&
                  root.at("components")[0].at("conversion_status") == "invalid_int32_conversion",
              "nonfinite color source bits and conversion reason retained");
    }
    for (unsigned count = 0; count < 4; ++count) {
        std::vector<Bytes> parts(count, real(1));
        check(unit(frame(COLOR, stack(parts))).at("status") == "invalid",
              "color requires all four components");
    }
    check(unit(frame(COLOR, stack({real(1), real(0), integer(0, I), real(0)}))).at("status") ==
              "invalid",
          "color components require double wire type");

    constexpr std::uint64_t PORT = 0x6647782002071873ULL, CALLOUT = 0x2871484802071873ULL;
    constexpr std::uint64_t INFO = 0x6647782004452207ULL, INTERACT = 0x0074782073520992ULL;
    std::vector<Bytes> port_members;
    for (int i = 9; i > 0; --i)
        port_members.push_back(real(i));
    for (auto id : {PORT, CALLOUT}) {
        root = unit(frame(id, stack(port_members))).at("root");
        check(root.at("kind") == (id == PORT ? "terminal_port" : "callout_line"),
              "same point layout retains distinct registered classes");
        const auto &f = root.at("fields");
        check(f.at("center").at("value") == Json::array({1., 2., 3.}) &&
                  f.at("second").at("value") == Json::array({4., 5., 6.}) &&
                  f.at("direction").at("value") == Json::array({7., 8., 9.}),
              "port and callout center second direction components");
        check(f.at("direction").at("components")[0].at("offset") <
                  f.at("direction").at("components")[2].at("offset"),
              "port vector source coordinates remain associated");
        check(one(frame(id, stack(port_members))).at("status") == "decoded",
              "connection geometry in nested component property");
        for (std::size_t n = 0; n < port_members.size(); ++n) {
            auto short_members = port_members;
            short_members.resize(n);
            check(unit(frame(id, stack(short_members))).at("status") == "invalid",
                  "connection geometry requires all nine doubles");
        }
    }
    root = unit(frame(INFO, stack({real(7.5), real(-3), real(2.25)}))).at("root");
    check(root.at("kind") == "terminal_port_info" && root.at("distance").at("value") == 2.25 &&
              root.at("depth").at("value") == -3 && root.at("angle").at("value") == 7.5,
          "viewport port information angle depth distance stack order");
    check(root.at("angle").at("unit") == "radians",
          "port angle unit follows native viewport calculation");

    // Fixture assembled in physical writer order, not using the reader-order stack helper.
    Bytes interaction;
    for (auto n : {10, 11, 12, 20, 21, 22, 30, 31, 32, 40, 41, 42, 1, 2, 3}) {
        auto bytes = real(n);
        interaction.insert(interaction.end(), bytes.begin(), bytes.end());
    }
    auto type = string(std::string("line\0line", 9));
    interaction.insert(interaction.end(), type.begin(), type.end());
    root = unit(frame(INTERACT, interaction)).at("root");
    check(root.at("kind") == "interact_point" && root.at("status") == "decoded",
          "intersection record physical writer fixture");
    auto points = root.at("fields");
    check(points.at("point").at("value") == Json::array({1., 2., 3.}) &&
              points.at("start_a_point").at("value") == Json::array({10., 11., 12.}) &&
              points.at("end_a_point").at("value") == Json::array({20., 21., 22.}) &&
              points.at("start_b_point").at("value") == Json::array({30., 31., 32.}) &&
              points.at("end_b_point").at("value") == Json::array({40., 41., 42.}),
          "all five saved points preserved independently of native reader overwrite");
    check(points.at("interaction_type").at("text") == std::string("line\0line", 9),
          "interaction narrow string does not use material NUL truncation");
    auto effect = root.at("native_reader_effect");
    check(effect.at("start_a_point").at("origin") == "constructor" &&
              effect.at("start_a_point").at("value") == Json::array({0, 0, 0}) &&
              effect.at("start_b_point").at("source_field") == "start_a_point" &&
              effect.at("overwritten_source_field") == "start_b_point",
          "native interaction reader overwrites B start and leaves A start default");
    auto extra_interaction = interaction;
    extra_interaction.insert(extra_interaction.begin(), 0xff);
    root = unit(frame(INTERACT, extra_interaction)).at("root");
    check(root.at("status") == "partial" && !root.contains("native_reader_effect"),
          "incomplete interaction payload cannot claim native result");
    for (std::size_t offset = 24; offset <= 15 * 24; offset += 24) {
        Bytes missing(interaction.begin() + offset, interaction.end());
        check(unit(frame(INTERACT, missing)).at("status") == "invalid",
              "interaction record requires all five complete points");
    }
    check(unit(frame(PORT, stack({integer(0, I)}))).at("status") == "invalid",
          "connection geometry double type is checked");
    check(unit(frame(INFO, stack({real(1), real(2)}))).at("status") == "invalid",
          "viewport port information requires three doubles");

    constexpr std::uint64_t REF = 0x2422220717143938ULL, NM = 0x7352099224222207ULL;
    root = unit(frame(REF, integer(UINT64_MAX))).at("root");
    check(root.at("kind") == "persistent_data_reference" && root.at("selector_kind") == "id" &&
              root.at("id").get<std::uint64_t>() == UINT64_MAX,
          "persistent data ID keeps unsigned width");
    check(root.at("value_resolution") == "not_performed" &&
              root.at("target").at("identifier_field") == "Identifier" &&
              root.at("target").at("value_field") == "DataUnit",
          "persistent reference does not fabricate its target value");
    root = unit(frame(REF, string(std::string("data\0name", 9)))).at("root");
    check(root.at("selector_kind") == "name" &&
              root.at("selector").at("text") == std::string("data\0name", 9) &&
              !root.contains("id"),
          "persistent name is a complete byte string distinct from numeric ID");
    root = unit(frame(REF, integer(UINT64_MAX, I))).at("root");
    check(root.at("id") == 0 && root.at("selector").at("value") == -1 &&
              root.at("selector_read_status") == "defaulted_wrong_type",
          "native persistent reader resets wrong signed selector type to ID zero");
    result = unit(frame(REF, frame(0x12345, {1})));
    check(result.at("status") == "partial" &&
              result.at("root").at("selector_kind") == "not_evaluated" &&
              !result.at("root").contains("id"),
          "unknown selector cannot claim native fallback");
    check(unit(frame(REF, {})).at("status") == "invalid", "persistent selector frame required");
    check(one(frame(REF, string("shared"))).at("status") == "decoded",
          "persistent references in nested component property");

    const auto old_object = object({{string("old"), property(integer(1))}});
    const auto new_object = object({{string("new"), property(integer(2))}});
    const auto named_map =
        frame(NM, stack({integer(6), string("b"), old_object, frame(S, {0x80}), object({}),
                         string("b"), new_object, string(std::string("a\0x", 3)), object({}),
                         string("a"), object({}), string(""), object({})}));
    root = unit(named_map).at("root");
    check(root.at("kind") == "string_noumenon_map" && root.at("entries").size() == 6,
          "named Noumenon map preserves every source entry");
    check(root.at("native_index").at("selected_entry_indices") == Json::array({5, 4, 3, 2, 1}),
          "named map byte ordering includes prefix NUL high-bit byte and last duplicate");
    check(root.at("native_index").at("value_assignment") == "replace_noumenon" &&
              root.at("entries")[0].at("value").at("entries")[0].at("key").at("text") == "old" &&
              root.at("entries")[2].at("value").at("entries").size() == 1 &&
              root.at("entries")[2].at("value").at("entries")[0].at("key").at("text") == "new",
          "duplicate Noumenon replacement does not merge old property members");
    check(unit(frame(NM, integer(0)))
              .at("root")
              .at("native_index")
              .at("selected_entry_indices")
              .empty(),
          "empty named map has an empty native index");
    check(unit(frame(NM, stack({integer(1), integer(0), object({})}))).at("status") == "invalid",
          "named map requires a string key");
    check(unit(frame(NM, stack({integer(1), string("key"), integer(0)}))).at("status") == "invalid",
          "named map requires a Noumenon value");
    const auto unresolved_object = object({{string("unknown"), property(frame(0x12345, {1}))}});
    root = unit(frame(NM, stack({integer(1), string("key"), unresolved_object}))).at("root");
    check(root.at("status") == "partial" && !root.contains("native_index"),
          "incomplete nested map value cannot claim complete native restoration");
    result = unit(frame(V, stack({integer(2), frame(0x12345, {1}), named_map})));
    check(result.at("status") == "partial" && result.at("root")
                                                      .at("entries")[1]
                                                      .at("native_index")
                                                      .at("selected_entry_indices")
                                                      .size() == 5,
          "earlier incomplete sibling does not invalidate independent named map index");
    check(one(named_map).at("status") == "decoded", "named Noumenon map in component property");
    return checks;
}
