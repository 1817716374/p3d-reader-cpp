#include "blob_internal.hpp"
namespace p3d {
static Json collect(Reader &r, const std::function<Json()> &fn, char fmt = 'Q',
                    std::size_t minimum = 1) {
    auto n = r.count(fmt, minimum);
    Json a = Json::array();
    for (std::uint64_t i = 0; i < n; ++i)
        a.push_back(fn());
    return a;
}
static std::string packed_string(Reader &r) {
    require(hex(r.take(3)) == "fffeff", "packed string marker");
    std::uint32_t n = r.u8();
    if (n == 255) {
        n = r.u16();
        if (n == 65535)
            n = r.u32();
    }
    require(n <= r.left() / 2, "packed string extent");
    return utf16(r.take(n * 2));
}
static Json unknown(Reader &r, std::size_t n) {
    auto off = r.p;
    return {{"offset", off}, {"bytes", n}, {"raw_hex", hex(r.take(n))}};
}
static Json section_points(Reader &r) {
    return collect(
        r,
        [&]() {
            r.expect("Q", 3);
            return r.doubles(3);
        },
        'Q', 32);
}
static Json parameters(Reader &r) {
    auto groups = collect(
        r,
        [&]() {
            auto name = r.string('I');
            auto entries = collect(
                r,
                [&]() {
                    auto off = r.p;
                    auto key = r.string('I');
                    auto type = r.u32();
                    auto storage = r.take(16);
                    auto flags = r.uints(5, 1);
                    Reader s(storage);
                    Json value;
                    std::string kind;
                    if (type == 1) {
                        s.p = 8;
                        value = s.i32();
                        kind = "int32";
                    } else if (type == 2) {
                        value = s.f64();
                        kind = "float64";
                    } else if (type == 3) {
                        require(storage[12] <= 1, "parameter bool");
                        value = bool(storage[12]);
                        kind = "bool";
                    } else
                        throw std::runtime_error("parameter variant");
                    return Json{{"offset", off},
                                {"name", key},
                                {"variant_type", type},
                                {"kind", kind},
                                {"value", value},
                                {"flags", flags},
                                {"variant_storage_hex", hex(storage)}};
                },
                'Q', 29);
            return Json{{"name", name}, {"entries", entries}};
        },
        'I', 12);
    return {
        {"groups", groups},
        {"note", "Inactive variant storage is retained; flags and enum labels are unassigned."}};
}
static Json group_info(Reader &r) {
    auto version = r.expect("I", 0);
    std::function<Json(unsigned)> node = [&](unsigned depth) {
        require(depth <= 64, "group tree depth");
        auto off = r.p;
        auto name = r.string('I');
        auto size = r.u32();
        auto end = r.p + size;
        auto data = r.take(r.u32());
        Json v = {{"offset", off}, {"name", name}, {"bytes", size}, {"data_hex", hex(data)}};
        Reader d(data);
        if (name == "m_eInfoType" && data.size() == 4)
            v["value"] = d.u32();
        if (name == "m_setName") {
            v["names"] = collect(d, [&]() { return Json(d.string('I')); }, 'I', 4);
            d.finish();
        }
        v["children"] = collect(r, [&]() { return node(depth + 1); }, 'I', 16);
        require(r.p == end, "group node extent");
        return v;
    };
    return {{"version", version}, {"nodes", collect(r, [&]() { return node(0); }, 'I', 16)}};
}
static Json data_unit(const Bytes &b) {
    std::function<std::pair<Json, std::size_t>(const Bytes &, std::size_t, unsigned)> value =
        [&](const Bytes &data, std::size_t end, unsigned depth) -> std::pair<Json, std::size_t> {
        require(depth <= 64 && end >= 16 && end <= data.size(), "DataUnit frame");
        Reader r(data, end - 16);
        auto size = r.u64();
        auto tag = hex(r.take(8));
        require(size <= end - 16, "DataUnit size");
        auto start = end - 16 - size;
        auto body = slice(data, start, size);
        Reader br(body);
        Json v = {{"type_token", tag}, {"bytes", size}};
        if (tag == "1924227014576945" || tag == "9200192422707724") {
            require(size == 8, "DataUnit integer width");
            v.update({{"kind", "uint64"}, {"value", br.u64()}});
        } else if (tag == "9200527369451613")
            v.update({{"kind", "string"}, {"value", utf8(body)}});
        else if (tag == "7126580673019513") {
            auto second = value(body, body.size(), depth + 1);
            auto first = value(body, second.second, depth + 1);
            require(first.second == 0, "DataUnit pair extent");
            v.update({{"kind", "pair"}, {"first", first.first}, {"second", second.first}});
        } else if (tag == "9513590052730948" || tag == "1457824815011113") {
            auto count = value(body, body.size(), depth + 1);
            require(count.first["type_token"] == "9200192422707724", "DataUnit count tag");
            auto n = count.first["value"].get<std::uint64_t>();
            require(n <= body.size() / 16, "DataUnit count");
            auto pos = count.second;
            Json entries = Json::array();
            for (std::uint64_t i = 0; i < n; ++i) {
                auto val = value(body, pos, depth + 1);
                auto key = value(body, val.second, depth + 1);
                if (tag == "1457824815011113")
                    require(key.first["type_token"] == "9200192422707724",
                            "DataUnit association key");
                entries.push_back({{"key", key.first}, {"value", val.first}});
                pos = key.second;
            }
            require(pos == 0, "DataUnit container extent");
            std::reverse(entries.begin(), entries.end());
            v.update({{"kind", tag == "9513590052730948" ? "map" : "association_sequence"},
                      {"entries", entries}});
        } else
            throw std::runtime_error("DataUnit type token");
        return {v, start};
    };
    auto out = value(b, b.size(), 0);
    require(out.second == 0, "DataUnit leading bytes");
    return out.first;
}
static Json secondary(Reader &r) {
    auto ver = r.expect("I", 1);
    auto cats = collect(r, [&]() { return Json(packed_string(r)); }, 'I', 4);
    auto props = collect(
        r,
        [&]() {
            auto name = packed_string(r);
            auto type = r.expect("I", 4);
            auto val = packed_string(r);
            auto cat = r.u32(), ordinal = r.u32();
            require(cat < cats.size(), "secondary category");
            return Json{{"name", name},          {"value", val},          {"variant_type", type},
                        {"category_index", cat}, {"category", cats[cat]}, {"ordinal", ordinal}};
        },
        'I', 4);
    return {{"version", ver}, {"categories", cats}, {"properties", props}};
}
static Json corridor(Reader &r) {
    auto v = r.expect("I", 1);
    auto kind = r.string();
    auto cv = r.expect("I", 1);
    require(kind == "RoadCorridor", "corridor class");
    auto maps = [&]() {
        return collect(r, [&]() {
            return collect(r, [&]() {
                auto k = r.string(), val = r.string();
                Json entry = {{"key", k}, {"value", val}};
                auto ix = val.find_first_not_of(" \t\r\n");
                if (ix != std::string::npos && (val[ix] == '{' || val[ix] == '[')) {
                    auto j = Json::parse(val, nullptr, false);
                    if (!j.is_discarded())
                        entry["decoded_json"] = j;
                }
                return entry;
            });
        });
    };
    auto groups = collect(r, [&]() {
        auto t = r.u32();
        auto stations = collect(r, [&]() {
            auto key = r.f64();
            auto ver = r.expect("I", 3);
            auto s = r.f64();
            auto name = r.string();
            auto points = maps(), surfaces = maps();
            return Json{{"key", key},
                        {"version", ver},
                        {"station", s},
                        {"name", name},
                        {"point_properties", points},
                        {"surface_properties", surfaces}};
        });
        return Json{{"type", t}, {"stations", stations}};
    });
    return {{"version", v}, {"class", kind}, {"class_version", cv}, {"groups", groups}};
}
static Json standard_floor(const Bytes &b) {
    require(b.size() == 96, "standard floor size");
    Reader r(b);
    Json abi = Json::array({{{"offset", 0},
                             {"bytes", 8},
                             {"kind", "serialized_vtable_pointer"},
                             {"raw_hex", hex(slice(b, 0, 8))}},
                            {{"offset", 92},
                             {"bytes", 4},
                             {"kind", "tail_alignment_padding"},
                             {"raw_hex", hex(slice(b, 92, 4))}}});
    auto ptr = r.u64();
    std::ostringstream addr;
    addr << "0x" << std::hex;
    addr.width(16);
    addr.fill('0');
    addr << ptr;
    abi[0]["address_hex"] = addr.str();
    // Offsets are relative to TagStdFlrDesignPara, including its serialized vptr.
    // The standard-floor dialog and grid bind these members to named controls.
    // These are library field names, not recovered C++ member identifiers.
    struct Field {
        int offset;
        const char *name;
        const char *label;
        const char *kind;
    };
    static constexpr Field known[] = {
        {8, "slab_thickness", "板厚", "length"},
        {12, "column_concrete_grade", "柱混凝土强度等级", "concrete_grade"},
        {16, "beam_concrete_grade", "梁混凝土强度等级", "concrete_grade"},
        {20, "shear_wall_concrete_grade", "剪力墙混凝土强度等级", "concrete_grade"},
        {24, "slab_concrete_grade", "板混凝土强度等级", "concrete_grade"},
        {28, "brace_concrete_grade", "斜杆混凝土强度等级", "concrete_grade"},
        {40, "slab_rebar_cover", "板钢筋保护层厚度", "length"},
        {48, "column_main_rebar_type", "柱主筋类别", "rebar_type"},
        {52, "beam_main_rebar_type", "梁主筋类别", "rebar_type"},
        {56, "wall_main_rebar_type", "墙主筋类别", "rebar_type"}};
    // Native combo indices, also used directly by the standard-floor grid.
    static constexpr const char *rebar[] = {"HPB300",      "HRB335",      "HRB400",  "HRB500",
                                            "冷轧带肋550", "冷轧带肋600", "HTRB600", "HPB235"};
    Json fields = Json::array(), values = Json::object();
    for (int i = 0; i < 21; ++i) {
        auto u = r.u32();
        Json field = {{"offset", 8 + 4 * i},
                      {"bytes", 4},
                      {"name", nullptr},
                      {"storage_uint32", u},
                      {"int32_view", std::int32_t(u)},
                      {"semantic_status", "unassigned"}};
        for (const auto &k : known) {
            if (k.offset != 8 + 4 * i)
                continue;
            auto value = std::int32_t(u);
            field.update({{"name", k.name},
                          {"label", k.label},
                          {"kind", k.kind},
                          {"value", value},
                          {"storage_type", "int32"},
                          {"semantic_status", "identified"}});
            values[k.name] = value;
            if (std::string(k.kind) == "length")
                field["unit"] = "mm";
            if (std::string(k.kind) == "concrete_grade")
                field["display_value"] = "C" + std::to_string(value);
            if (std::string(k.kind) == "rebar_type") {
                field["enum_label"] = nullptr;
                field["enum_status"] = "unknown_value";
                if (u < sizeof(rebar) / sizeof(*rebar)) {
                    field["enum_label"] = rebar[u];
                    field["enum_status"] = "identified";
                }
            }
        }
        fields.push_back(std::move(field));
    }
    return {{"native_type", "PBBim::PBBimStruct::TagStdFlrDesignPara"},
            {"layout", "observed_msvc_x64_memory_image_96"},
            {"abi_fields", abi},
            {"fields", fields},
            {"named_values", values},
            {"identified_field_count", values.size()},
            {"unassigned_field_count", fields.size() - values.size()},
            {"layout_status", "complete_for_observed_abi"},
            {"semantic_status", "partially_identified"},
            {"unresolved_spans", Json::array()},
            {"note",
             "The pointer is source-process residue and is never dereferenced or treated as an "
             "object ID. All member storage and nonzero padding are retained. Named values use "
             "the standard-floor UI bindings; other members remain unassigned. Field units do "
             "not establish the document's geometry units."}};
}
static Json pilecap(Reader &r) {
    auto version = [&]() {
        auto c = r.expect("I", 0), v = r.expect("I", 0);
        return Json{{"cereal_version", c}, {"version", v}};
    };
    auto scalar = [&](std::string member, std::string fmt) {
        auto off = r.p;
        auto v = r.number(fmt);
        auto storage = slice(r.b, off, r.p - off);
        Reader sr(storage);
        Json result = {{"offset", off},
                       {"bytes", storage.size()},
                       {"native_member", member},
                       {"numeric_view_format", fmt},
                       {"value", v},
                       {"name", nullptr},
                       {"storage_uint", storage.size() == 8   ? sr.u64()
                                        : storage.size() == 4 ? sr.u32()
                                                              : sr.u8()},
                       {"declared_type", nullptr}};
        if (storage.size() == 8) {
            sr.p = 0;
            result["float64_view"] = sr.f64();
        }
        return result;
    };
    Json result = version();
    result["native_type"] = "PBBim::StructBaseModel::PileCapSectionPara";
    auto sec = version(), base = version();
    base["native_type"] = "PileBasePara";
    base["members"] = Json::array();
    for (auto pair : std::vector<std::pair<std::string, std::string>>{{"0x0", "i"},
                                                                      {"0x4", "i"},
                                                                      {"0x1d0", "i"},
                                                                      {"0x1d4", "i"},
                                                                      {"0x1d8", "d"},
                                                                      {"0x1e0", "d"},
                                                                      {"0x1e8", "d"},
                                                                      {"0x1f0", "i"},
                                                                      {"0x1f4", "i"},
                                                                      {"0x1f8", "i"},
                                                                      {"0x1fc", "i"},
                                                                      {"0x200", "i"},
                                                                      {"0x208", "d"},
                                                                      {"0x210", "d"},
                                                                      {"0x218", "i"},
                                                                      {"0x21c", "i"},
                                                                      {"0x220", "d"},
                                                                      {"0x228", "d"}})
        base["members"].push_back(scalar(pair.first, pair.second));
    sec.update({{"native_type", "PileSectionPara"}, {"base", base}, {"section_id", r.i64()}});
    result["pile_section"] = sec;
    result["members"] = Json::array();
    for (auto m : {"0x2a0", "0x2a8", "0x248"})
        result["members"].push_back(scalar(m, "i"));
    result["integer_array_0x250"] = collect(r, [&]() { return Json(r.i32()); }, 'Q', 4);
    result["primary_code"] = r.i32();
    result["primary_points"] = section_points(r);
    result["secondary_code"] = r.i32();
    result["secondary_point_arrays"] = collect(r, [&]() { return section_points(r); }, 'Q', 8);
    for (auto m : {"0x2a4", "0x2b0", "0x2b4"})
        result["members"].push_back(scalar(m, "i"));
    result["reinforcement_cereal_version"] = r.expect("I", 0);
    result["reinforcement"] = Json::array();
    for (auto m : {"0x2bc", "0x2d0"}) {
        Json item = {{"native_type", "StructBaseReinForce"},
                     {"native_member", m},
                     {"version", r.expect("I", 0)},
                     {"members", Json::array()}};
        for (auto k : {"0x0", "0x4", "0x8", "0xc"})
            item["members"].push_back(scalar(k, "i"));
        result["reinforcement"].push_back(item);
    }
    result["members"].push_back(scalar("0x2e4", "B"));
    result["members"].push_back(scalar("0x2e8", "i"));
    // Each native type has its own member namespace and enum numbering.
    // PileBasePara: managed property getters. PileCapSectionPara / reinforcement:
    // native cap-dialog data exchange and its resource lists.
    struct Meaning {
        const char *member;
        const char *name;
        const char *label;
        const char *unit;
        std::vector<std::string> choices;
    };
    auto identify = [](Json &block, const std::vector<Meaning> &meanings) {
        Json values = Json::object();
        for (auto &field : block["members"]) {
            for (const auto &m : meanings) {
                if (field["native_member"] != m.member)
                    continue;
                field.update(
                    {{"name", m.name},
                     {"label", m.label},
                     {"semantic_status", "identified"},
                     {"declared_type", field["numeric_view_format"] == "d" ? "float64" : "int32"}});
                values[m.name] = field["value"];
                if (*m.unit)
                    field["unit"] = m.unit;
                if (!m.choices.empty()) {
                    auto value = field["value"].get<std::int64_t>();
                    field["enum_label"] = nullptr;
                    field["enum_status"] = "unknown_value";
                    if (value >= 0 && std::uint64_t(value) < m.choices.size()) {
                        field["enum_label"] = m.choices[std::size_t(value)];
                        field["enum_status"] = "identified";
                    }
                }
            }
        }
        block["named_values"] = values;
        block["identified_member_count"] = values.size();
        block["unassigned_member_count"] = block["members"].size() - values.size();
    };
    static const std::vector<Meaning> base_meanings = {
        {"0x4",
         "concrete_grade",
         "混凝土强度等级",
         "",
         {"C15", "C20", "C25", "C30", "C35", "C40", "C45", "C50", "C55", "C60", "C65", "C70", "C75",
          "C80"}},
        {"0x218", "x_offset", "x轴偏移", "mm", {}},
        {"0x21c", "y_offset", "y轴偏移", "mm", {}},
        {"0x220", "rotation", "旋转角度", "", {}},
        {"0x228", "top_elevation", "顶部标高", "m", {}}};
    static const std::vector<Meaning> cap_meanings = {
        {"0x2a0", "cap_type", "承台类型", "", {"阶形预制", "锥形预制", "阶形现浇", "锥形现浇"}},
        {"0x2a4", "plan_shape", "平面形状", "", {"圆形", "矩形", "正多边形", "多边形"}},
        {"0x248", "step_count", "承台阶数", "", {}},
        {"0x2b0", "top_offset_x", "承台顶面相对底面X偏心", "", {}},
        {"0x2b4", "top_offset_y", "承台顶面相对底面Y偏心", "", {}}};
    static const std::vector<Meaning> reinforcement_meanings = {
        {"0x0",
         "rebar_type",
         "钢筋级别",
         "",
         {"HPB235", "HPB300", "HRB335", "HRB400", "HRB500", "CRB550", "CRB600", "HTRB600", "T63"}},
        {"0x4", "spacing", "间距", "mm", {}},
        {"0x8", "diameter", "直径", "mm", {}},
        {"0xc", "distribution_width", "布置宽度", "mm", {}}};
    identify(result["pile_section"]["base"], base_meanings);
    identify(result, cap_meanings);
    for (auto &reinforcement : result["reinforcement"])
        identify(reinforcement, reinforcement_meanings);
    auto steps = result["named_values"]["step_count"].get<int>();
    if ((steps == 1 || steps == 2) && result["integer_array_0x250"].size() == std::size_t(steps)) {
        result["step_heights"] = result["integer_array_0x250"];
        result["step_heights_order"] = "lower_to_upper";
        result["step_heights_status"] = "identified";
    } else
        result["step_heights_status"] = "unsupported_step_layout";
    result.update({{"layout_status", "complete_for_supported_versions"},
                   {"semantic_status", "some_parameter_names_unassigned"},
                   {"unresolved_spans", Json::array()}});
    return result;
}
static Json station(Reader &r) {
    return {{"version", r.expect("I", 1)}, {"label", r.string()}, {"distance", r.f64()}};
}
static Json component(Reader &r) {
    auto v = r.expect("I", 3);
    auto kind = r.string();
    auto cv = r.u32();
    Json result = {{"version", v}, {"class", kind}, {"class_version", cv}};
    if (kind == "RoadCorridor" && cv == 6) {
        result["prefix_version"] = r.expect("I", 1);
        result["unassigned_header"] = r.number("QQIIQQ");
        result["template_version"] = r.expect("I", 1);
        result["template_path"] = r.string();
        result["relative_template_path"] = r.string();
        result["start_station"] = station(r);
        result["end_station"] = station(r);
        result["sampling_interval"] = r.f64();
        result["unassigned_settings"] = r.number("QQIQQIQIQIQQIQIQ");
        result["stations"] = collect(
            r,
            [&]() {
                return Json{
                    {"version", r.expect("I", 2)}, {"flags", r.u32()}, {"station", station(r)}};
            },
            'Q', 28);
        result["footer_header"] = r.number("II");
        result["footer_values"] = Json::array();
        for (int i = 0; i < 2; ++i)
            result["footer_values"].push_back({{"version", r.expect("I", 1)}, {"value", r.f64()}});
    } else if (kind == "BridgeStruct" && cv == 3) {
        result["name"] = r.string();
        result["station_positions"] = r.doubles(3);
        result["start_station_label"] = r.string();
        result["unassigned_strings"] = Json::array({r.string(), r.string()});
        result["span_expression"] = r.string();
        result["length"] = r.f64();
        result["unassigned_values"] = r.doubles(2);
        result["span_lengths"] = collect(r, [&]() { return Json(r.f64()); }, 'Q', 8);
        result["supports"] = collect(
            r,
            [&]() {
                return Json{{"version", r.expect("I", 1)},
                            {"label", r.string()},
                            {"station", r.f64()},
                            {"unassigned_value", r.f64()}};
            },
            'Q', 28);
        result["sections"] = collect(
            r,
            [&]() {
                Json sec = {{"version", r.expect("I", 1)},  {"type_code", r.u32()},
                            {"name", r.string()},           {"template_name", r.string()},
                            {"start_station", r.f64()},     {"length", r.f64()},
                            {"span_expression", r.string()}};
                sec["span_lengths"] = collect(r, [&]() { return Json(r.f64()); }, 'Q', 8);
                return sec;
            },
            'Q', 48);
        result["piers"] = collect(
            r,
            [&]() {
                return Json{{"version", r.expect("I", 1)},
                            {"unassigned_uint64", r.u64()},
                            {"label", r.string()},
                            {"angle", r.f64()},
                            {"script_reference", r.string()},
                            {"unassigned_values", r.doubles(3)}};
            },
            'Q', 56);
        result["barrier_version"] = r.expect("I", 1);
        result["barrier_name"] = r.string();
        result["barrier_parameter"] = r.string();
    } else
        throw std::runtime_error("component definition class/version");
    return result;
}
static Json feature_catalog(Reader &r) {
    auto v = r.expect("I", 2);
    std::function<Json(unsigned)> node = [&](unsigned depth) {
        require(depth <= 64, "feature tree depth");
        auto v = r.expect("I", 1);
        auto name = r.string();
        auto children = collect(r, [&]() { return node(depth + 1); }, 'Q', 20);
        return Json{{"version", v}, {"path", name}, {"children", children}};
    };
    auto smap = [&]() {
        return collect(
            r, [&]() { return Json{{"key", r.string()}, {"value", r.string()}}; }, 'Q', 16);
    };
    Json out = {{"version", v},
                {"header", r.expect("I", 1)},
                {"root_name", r.string()},
                {"collection_version", r.expect("I", 1)}};
    out["paths"] = collect(r, [&]() { return node(0); }, 'Q', 20);
    out["styles"] = collect(
        r,
        [&]() {
            return Json{{"header", r.expect("II", {1, 1})},    {"parent_path", r.string()},
                        {"unassigned_string", r.string()},     {"name", r.string()},
                        {"display_name", r.string()},          {"unassigned_uint64", r.u64()},
                        {"unassigned_uint32", r.number("III")}};
        },
        'Q', 60);
    out["feature_tree_version"] = r.expect("I", 1);
    out["feature_paths"] = collect(r, [&]() { return node(0); }, 'Q', 20);
    out["features"] = collect(
        r,
        [&]() {
            return Json{{"header", r.expect("II", {2, 1})},
                        {"parent_path", r.string()},
                        {"unassigned_string", r.string()},
                        {"name", r.string()},
                        {"alias", r.string()},
                        {"template_path", r.string()},
                        {"component_paths", smap()},
                        {"related_feature_paths", smap()}};
        },
        'Q', 64);
    return out;
}
static Json drawing(Reader &r, const std::string &cl) {
    Json out = {{"class", cl}, {"semantic_status", "partial"}, {"unresolved_spans", Json::array()}};
    if (cl == "PSDrawingGlobalParametrer") {
        out["version"] = r.expect("I", 0);
        out["phone"] = r.string();
        out["print_drawing_scale"] = r.f64();
        out["range_low"] = r.doubles(3);
        out["range_high"] = r.doubles(3);
    } else if (cl == "PSDrawingManager" || cl == "SubsDrawingManager") {
        out["base_version"] = r.expect("I", 1);
        auto names = collect(r, [&]() { return Json(r.string()); }, 'Q', 8);
        Json columns = Json::array();
        for (int i = 0; i < 12; ++i) {
            auto col = collect(r, [&]() { return Json(r.f64()); }, 'Q', 8);
            require(col.size() == names.size(), "drawing transform collection length");
            columns.push_back(col);
        }
        out["block_transforms"] = Json::array();
        for (std::size_t i = 0; i < names.size(); ++i) {
            Json values = Json::array();
            for (auto &c : columns)
                values.push_back(c[i]);
            out["block_transforms"].push_back({{"name", names[i]}, {"native_components", values}});
        }
        out["version"] = r.expect("I", 0);
        std::vector<int> ids = cl == "PSDrawingManager" ? std::vector<int>{4, 5, 3, 6, 7}
                                                        : std::vector<int>{0, 1, 2, 3, 4, 5, 6, 7};
        Json records = Json::array();
        for (auto id : ids) {
            auto ho = r.p;
            auto cv = r.expect("I", 0), v = r.expect("I", 0);
            Json bcv = records.empty() ? r.expect("I", 0) : Json();
            auto bv = r.expect("I", 0);
            Json item = {{"header_offset", ho}, {"cereal_version", cv},
                         {"version", v},        {"base_version", bv},
                         {"offset", r.p},       {"type_code", r.expect("I", id)},
                         {"mode_code", r.u32()}};
            if (cl == "SubsDrawingManager") {
                item["unassigned_scale"] = r.f64();
                item["unassigned_code"] = r.u32();
                item["unassigned_values"] = r.doubles(3);
                item["type_specific_values"] = r.doubles(id == 2 || id == 3 ? 4 : id == 7 ? 3 : 2);
            } else {
                std::map<int, std::string> names = {{4, "ConLayoutPlanSheetInfo"},
                                                    {5, "ConLayoutEleSheetInfo"},
                                                    {3, "ConLayoutColDownSheetInfo"},
                                                    {6, "ConNodeSheetInfo"},
                                                    {7, "ConLinkSheetInfo"}};
                item["native_type"] = names.at(id);
                item["unassigned_values"] = r.doubles(3);
                item["unassigned_code"] = r.u32();
                if (id == 4) {
                    item["type_specific_code"] = r.u32();
                    item["type_specific_values"] = r.doubles(4);
                    item["flags"] = r.uints(4, 1);
                } else if (id == 5) {
                    item["type_specific_codes"] = r.uints(2);
                    item["type_specific_values"] = r.doubles(3);
                    item["flags"] = r.uints(2, 1);
                    item["unassigned_value"] = r.f64();
                    item["trailing_flag"] = r.u8();
                } else if (id == 3) {
                    item["flag"] = r.u8();
                    item["unassigned_value"] = r.f64();
                } else
                    item["flags"] = r.uints(id == 6 ? 2 : 3, 1);
            }
            if (!bcv.is_null())
                item["base_cereal_version"] = bcv;
            records.push_back(item);
        }
        out["records"] = records;
    } else if (cl == "PSDrawingManagerUI") {
        out["version"] = r.expect("I", 0);
        out["paper_clocked_or_frozen"] =
            collect(r, [&]() { return Json{{"name", r.string()}, {"value", r.u8()}}; }, 'Q', 9);
        out["paper_serial_numbers"] =
            collect(r, [&]() { return Json{{"name", r.string()}, {"value", r.i32()}}; }, 'Q', 12);
    } else
        throw std::runtime_error("drawing class");
    out["layout_status"] = "complete_for_supported_versions";
    out["semantic_status"] = cl == "PSDrawingGlobalParametrer" || cl == "PSDrawingManagerUI"
                                 ? "decoded"
                                 : "some_parameter_names_unassigned";
    return out;
}
static Json assembly(Reader &r, const std::string &cl) {
    Json out = {{"class", cl},
                {"unresolved_spans", Json::array()},
                {"layout_status", "complete_for_supported_versions"},
                {"semantic_status", "some_parameter_names_unassigned"}};
    auto empty = [&](std::string member, std::string kind) {
        auto off = r.p;
        r.expect("Q", 0);
        return Json{{"offset", off},
                    {"native_member", member},
                    {"kind", kind},
                    {"count", 0},
                    {"entries", Json::array()}};
    };
    auto ver = r.u32();
    require(ver <= 1, "assembly version");
    out["version"] = ver;
    if (cl == "AssemblyStatus") {
        std::vector<std::string> names = {"have_force", "is_changed_struct_model",
                                          "need_refresh_design_model", "if_set_para"};
        if (ver == 1)
            names.push_back("if_edit");
        for (auto &n : names)
            out["flags"][n] = r.u8();
        out["semantic_status"] = "decoded";
    } else if (cl == "AssemblyLinkModel") {
        out["link_merge_set"] = {
            {"cereal_version", r.expect("I", 0)},
            {"version", r.expect("I", 0)},
            {"enum_codes", r.number("2i")},
            {"collections", Json::array({empty("0x1a8", "vector"), empty("0x1c0", "vector")})}};
        out["display_control"] = {{"cereal_version", r.expect("I", 0)},
                                  {"version", r.expect("I", 0)},
                                  {"enum_codes", r.number("4i")}};
        out["boolean_vector"] = collect(r, [&]() { return Json(r.u8()); });
        if (ver == 1)
            out["load_g_para_id"] = r.i64();
    } else if (cl == "ElecParaData") {
        out["cereal_version"] = r.expect("I", 0);
        out["wj_version"] = r.expect("I", 2);
        out["strings"] = Json::array({r.string(), r.string()});
        out["section_parameters"] = {{"native_type", "PSXSectManager"},
                                     {"cereal_version", r.expect("I", 0)},
                                     {"version", r.expect("I", 0)},
                                     {"collection", empty("PBWJData+0x48", "vector")}};
        out["collections"] = Json::array();
        for (auto m : {"PBWJData+0x60", "PBWJData+0x70", "PBWJData+0x80", "PBWJData+0x90:wj_joints",
                       "PBWJData+0xa0:wj_bars"})
            out["collections"].push_back(empty(m, "map"));
        out["boolean_member_0x110"] = {
            {"offset", r.p}, {"storage_uint8", r.u8()}, {"name", nullptr}};
        for (auto pair :
             std::vector<std::pair<std::string, std::string>>{{"PBWJData+0x118", "map"},
                                                              {"PBWJData+0x138", "map"},
                                                              {"PBWJData+0x168", "vector"},
                                                              {"PBWJData+0x180", "vector"},
                                                              {"PBWJData+0x1e0", "vector"},
                                                              {"PBWJData+0x1f8", "vector"}})
            out["collections"].push_back(empty(pair.first, pair.second));
        if (ver == 1)
            out["changed_joints"] = empty("ElecParaData+0x20", "map");
    } else
        throw std::runtime_error("assembly class");
    return out;
}
Json application_blob(const std::string &name, const Bytes &b, const std::string &cl) {
    Reader r(b);
    Json v;
    if (name == "Parameter")
        v = parameters(r);
    else if (name == "GroupInfoBinary")
        v = group_info(r);
    else if (name == "ExtendPro") {
        auto h = r.expect("QBI", {0, 1, 5});
        auto text = utf8(r.take(r.u64()));
        v = {{"header", h}, {"text", text}, {"value", Json::parse(text)}};
    } else if (name == "ModelIdMap") {
        auto ver = r.expect("I", 1);
        auto n = r.count('H', 4);
        require(r.u32() == n, "model map count");
        Json entries = Json::array();
        for (std::uint64_t i = 0; i < n; ++i)
            entries.push_back({{"first_model_id", r.u32()},
                               {"second_model_id", r.u32()},
                               {"name", packed_string(r)},
                               {"unassigned_uint32", r.u32()}});
        v = {{"version", ver}, {"entries", entries}};
    } else if (name == "DataIdMap") {
        require(b.size() % 24 == 0, "DataIdMap stride");
        Json entries = Json::array();
        while (r.left())
            entries.push_back(
                {{"parameter_handle", r.u64()}, {"class_id", r.u64()}, {"object_id", r.u64()}});
        v = {{"entries", entries}};
    } else if (name == "DataUnit")
        return data_unit(b);
    else if (name == "SecondaryDevelopmentProperty")
        v = secondary(r);
    else if (name == "CustomExtensionProperty")
        v = corridor(r);
    else if (name == "DesignPara")
        return standard_floor(b);
    else if (name == "Pilecap")
        v = pilecap(r);
    else if (name == "BinaryData") {
        Json spans = Json::array({unknown(r, 156)});
        auto pc = r.expect("I", 4);
        auto points = section_points(r);
        auto sc = r.expect("I", 4);
        auto secondary = collect(r, [&]() { return section_points(r); }, 'Q', 8);
        require(r.left() == 61, "standard section suffix");
        spans.push_back(unknown(r, 61));
        v = {{"primary_code", pc},
             {"primary_points", points},
             {"secondary_code", sc},
             {"secondary_point_arrays", secondary},
             {"semantic_status", "partial"},
             {"unresolved_spans", spans},
             {"note", "Contour roles, header and design suffix are unassigned; these are "
                      "definition data, not additional scene instances."}};
    } else if (name == "CerealDatas")
        v = drawing(r, cl);
    else if (name == "AssemblyCereal")
        v = assembly(r, cl);
    else if (name == "Definition" && cl == "ComponentElement")
        v = component(r);
    else if (name == "Definition" && cl == "FeatureStyleDataElement")
        v = feature_catalog(r);
    else
        throw std::runtime_error("application field unsupported");
    r.finish();
    return v;
}
Json profile(const Bytes &b, unsigned depth) {
    require(depth <= 50, "profile depth");
    Reader r(b);
    auto version = r.u32();
    require(version <= 4, "profile boundary");
    Json commands = Json::array();
    while (r.left()) {
        auto off = r.p;
        auto op = r.u32(), n = r.u32();
        auto raw = r.take(n);
        Reader p(raw);
        Json item = {{"type", op}, {"offset", off}, {"bytes", n}};
        if (op == 1 && n == 48)
            item.update({{"kind", "line"}, {"start", p.doubles(3)}, {"end", p.doubles(3)}});
        else if (op == 2 && n && n % 24 == 0) {
            Json pts = Json::array();
            while (p.left())
                pts.push_back(p.doubles(3));
            item.update({{"kind", "polyline"}, {"points", pts}});
        } else if (op == 3 && n == 88)
            item.update({{"kind", "elliptical_arc"},
                         {"origin", p.doubles(3)},
                         {"vector0", p.doubles(3)},
                         {"vector90", p.doubles(3)},
                         {"start_angle", p.f64()},
                         {"sweep_angle", p.f64()}});
        else if (op == 8)
            item.update({{"kind", "nested_profile"}, {"profile", profile(raw, depth + 1)}});
        else
            item.update({{"kind", "unknown"}, {"binary_base64", base64(raw)}});
        commands.push_back(item);
    }
    return {{"version", version}, {"boundary_code", version}, {"primitives", commands}};
}
} // namespace p3d
