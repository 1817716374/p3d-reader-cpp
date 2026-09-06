#include "internal.hpp"
namespace p3d {
namespace {
template <class F> Json collect(Reader &r, F decode, char format = 'Q', std::size_t minimum = 1) {
    auto count = r.count(format, minimum);
    Json values = Json::array();
    for (std::uint64_t i = 0; i < count; ++i)
        values.push_back(decode());
    return values;
}
static const std::map<int, const char *> section_type_labels = {
    {1, "矩"},
    {2, "工"},
    {3, "圆"},
    {4, "多边"},
    {5, "槽"},
    {6, "十"},
    {7, "L"},
    {8, "T"},
    {9, "双槽"},
    {10, "梯"},
    {11, "基础梁"},
    {15, "箱"},
    {16, "H"},
    {17, "圆管"},
    {18, "翼悬箱"},
    {19, "强箱1"},
    {20, "强箱2"},
    {23, "双工"},
    {25, "矩变"},
    {26, "箱变"},
    {27, "工变"},
    {28, "加腋矩"},
    {29, "加腋箱"},
    {30, "加腋工"},
    {36, "欧洲H"},
    {37, "日本H"},
    {38, "美国H"},
    {39, "国标H"},
    {40, "高频焊H"},
    {41, "剖分T"},
    {42, "波折腹H"},
    {43, "波浪腹H"},
    {44, "波浪腹H"},
    {71, "薄壁L"},
    {72, "薄壁槽"},
    {73, "薄壁Z"},
    {74, "薄壁L组合"},
    {75, "薄壁槽组合"},
    {77, "薄壁钢管"},
    {78, "薄壁几"},
    {79, "薄壁水槽"},
    {101, "工型钢混凝土"},
    {102, "箱型钢混凝土"},
    {103, "对称十字型钢混凝土"},
    {104, "不对称十字型钢混凝土"},
    {105, "外方内圆管型钢混凝土"},
    {106, "外圆内工型钢混凝土"},
    {107, "外圆内圆型钢混凝土"},
    {108, "外圆内不对称十字型钢混凝土"},
    {201, "矩型钢混凝土"},
    {202, "圆管型钢混凝土"},
    {302, "矩"},
    {303, "T"},
    {304, "C"},
};
// Native short labels include orientation symbols and duplicate labels.
static const std::map<int, std::vector<const char *>> section_subtype_labels = {
    {31, {"工", "工"}},
    {32, {"槽", "槽"}},
    {33, {"L", "不等边L"}},
    {34,
     {"等边角钢┓┏", "不等边角钢长边┒┎", "不等边角钢短边┒┎", "等边角钢┓┗", "等边角钢┎  ┒",
      "不等边等边角钢┎  ┒"}},
    {35, {"槽][", "槽[]", "槽][", "槽[]", "槽[]", "槽[]"}},
};
struct ElecCollections {
    Reader &r;
    std::set<std::string> seen;
    Json record(const char *type, unsigned max_version) {
        Json out = {{"offset", r.p}, {"native_type", type}};
        if (seen.insert(type).second)
            out["cereal_version"] = r.expect("I", 0);
        auto version = r.u32();
        require(version <= max_version,
                std::string("unsupported electrical element version: ") + type);
        out["version"] = version;
        return out;
    }
    Json word(unsigned width) {
        auto offset = r.p;
        if (width == 1)
            return {{"offset", offset}, {"storage_uint8", r.u8()}};
        if (width == 2)
            return {{"offset", offset},
                    {"storage_uint16", r.u16()},
                    {"int16_view", r.at<std::int16_t>(offset)}};
        if (width == 4)
            return {{"offset", offset},
                    {"storage_uint32", r.u32()},
                    {"int32_view", r.at<std::int32_t>(offset)},
                    {"float32_view", r.at<float>(offset)}};
        return {{"offset", offset},
                {"storage_uint64", r.u64()},
                {"int64_view", r.at<std::int64_t>(offset)},
                {"float64_view", r.at<double>(offset)}};
    }
    Json words64() {
        return collect(r, [&]() { return word(8); }, 'Q', 8);
    }
    void member(Json &out, unsigned offset, Json value) {
        value["native_member_offset"] = offset;
        out["members"].push_back(std::move(value));
    }
    void scalar(Json &out, unsigned offset, unsigned width) {
        member(out, offset, word(width));
    }
    void named_scalar(Json &out, unsigned offset, unsigned width, const char *name,
                      const char *view, const char *type) {
        auto value = word(width);
        value["name"] = name;
        value["value"] = value.at(view);
        value["storage_type"] = type;
        value["semantic_status"] = "identified";
        member(out, offset, std::move(value));
    }
    void vectors32(Json &out, std::initializer_list<unsigned> offsets) {
        for (auto offset : offsets) {
            auto start = r.p;
            auto entries = collect(r, [&]() { return word(4); }, 'Q', 4);
            member(out, offset, {{"offset", start}, {"kind", "vector"}, {"entries", entries}});
        }
    }
    Json column_parameter() {
        auto out = record("SubsColPar", 6);
        auto version = out["version"].get<unsigned>();
        out["members"] = Json::array();
        scalar(out, 0, 8);
        scalar(out, 8, 1);
        for (auto offset : {0x10u, 0x20u, 0x30u, 0x38u})
            scalar(out, offset, 8);
        named_scalar(out, 0x40, 4, "column_section_count", "int32_view", "int32");
        if (version >= 1) {
            named_scalar(out, 0x44, 4, "column_type", "int32_view", "int32");
            auto &type = out["members"].back();
            const auto code = type["value"].get<std::int32_t>();
            type["enum_status"] = "unknown_value";
            if (code == 0 || code == 1) {
                type["enum_status"] = "identified";
                type["enum_label"] = code == 0 ? "人字柱" : "格构柱";
            }
            scalar(out, 0x18, 8);
            scalar(out, 0x28, 8);
        }
        if (version >= 2)
            scalar(out, 0x48, 8);
        if (version >= 3) {
            scalar(out, 0x50, 4);
            scalar(out, 0x54, 4);
            named_scalar(out, 0x58, 8, "fire_wall_height", "float64_view", "float64");
            out["members"].back()["native_property_name"] = "fireWallH";
            for (auto offset : {0x60u, 0x68u, 0x70u, 0x78u})
                scalar(out, offset, 8);
        }
        if (version >= 4) {
            scalar(out, 0xa0, 4);
            scalar(out, 0xa8, 8);
            scalar(out, 0xb0, 8);
        }
        if (version >= 5) {
            auto start = r.p;
            auto entries = collect(
                r,
                [&]() {
                    auto key = word(8);
                    key.update({{"name", "upper_elevation"},
                                {"value", key.at("float64_view")},
                                {"storage_type", "float64"}});
                    auto value = word(4);
                    value.update({{"name", "segment_index"},
                                  {"value", value.at("int32_view")},
                                  {"storage_type", "int32"},
                                  {"index_base", 1}});
                    return Json{{"key", key}, {"value", value}};
                },
                'Q', 12);
            member(out, 0xb8,
                   {{"offset", start},
                    {"kind", "map"},
                    {"name", "bar_segments_by_elevation"},
                    {"semantic_status", "identified"},
                    {"entries", entries}});
        }
        if (version >= 6)
            for (auto offset : {0x80u, 0x90u}) {
                auto start = r.p;
                auto entries = collect(
                    r,
                    [&]() {
                        auto key = word(4);
                        key["value"] = key.at("int32_view");
                        key["storage_type"] = "int32";
                        auto values = words64();
                        for (auto &value : values) {
                            value["value"] = value.at("float64_view");
                            value["storage_type"] = "float64";
                        }
                        return Json{{"key", key}, {"value", values}};
                    },
                    'Q', 12);
                member(out, offset,
                       {{"offset", start},
                        {"kind", "map"},
                        {"name",
                         offset == 0x80 ? "domain_column_parameters" : "domain_beam_parameters"},
                        {"native_property_name", offset == 0x80 ? "colParam" : "beamParam"},
                        {"entries", entries}});
            }
        return out;
    }
    Json columns() {
        return collection(
            "PBWJData+0x118", "map",
            [&]() {
                auto start = r.p;
                auto key = r.i32();
                return Json{{"offset", start}, {"key", key}, {"value", column_parameter()}};
            },
            53);
    }
    Json truss_beam() {
        auto out = record("TrussBeam", 5);
        auto version = out["version"].get<unsigned>();
        out["members"] = Json::array();
        member(out, 8, point());
        member(out, 0x38, point());
        member(out, 0x68, truss_parameter());
        // The native archive serializes member 0x138 twice. Keep both values
        // in stream order, including when they differ in the source archive.
        vectors32(out, {0x90, 0xa8, 0xc0, 0xd8, 0xf0, 0x108, 0x120, 0x138, 0x138, 0x150, 0x168,
                        0x180, 0x198});
        if (version >= 1) {
            for (auto offset : {0x1e0u, 0x1f8u}) {
                auto start = r.p;
                auto entries = collect(r, [&]() { return point(); }, 'Q', 16);
                member(out, offset, {{"offset", start}, {"kind", "vector"}, {"entries", entries}});
            }
            scalar(out, 0, 4);
        }
        if (version >= 2) {
            vectors32(out, {0x1b0});
            for (auto offset : {0x288u, 0x28cu, 0x290u, 0x294u})
                scalar(out, offset, 4);
        }
        if (version >= 3)
            vectors32(out, {0x210, 0x228, 0x240, 0x258, 0x270});
        if (version >= 4)
            vectors32(out, {0x298, 0x1c8});
        if (version >= 5)
            vectors32(out, {0x2b0, 0x2c8});
        return out;
    }
    Json human_column() {
        auto out = record("HumanColumn", 4);
        auto version = out["version"].get<unsigned>();
        out["members"] = Json::array();
        member(out, 8, point());
        out["members"].back()["name"] = "start_point";
        member(out, 0x38, point());
        out["members"].back()["name"] = "end_point";
        named_scalar(out, 0x68, 8, "rotation", "float64_view", "float64");
        out["members"].back()["unit"] = "rad";
        member(out, 0x70, column_parameter());
        // This repeated member is also present in the native reader/writer.
        vectors32(out, {0x140, 0x158, 0x170, 0x188, 0x1a0, 0x1a0, 0x1b8, 0x1d0});
        if (version >= 1)
            scalar(out, 0, 4);
        if (version >= 3)
            scalar(out, 4, 4);
        if (version >= 1)
            vectors32(out, {0x338});
        if (version >= 2)
            vectors32(out, {0x350, 0x368, 0x380, 0x398, 0x3b0, 0x3c8, 0x3e0, 0x3f8, 0x410, 0x428,
                            0x440, 0x458, 0x470, 0x488, 0x4a0});
        if (version >= 4)
            vectors32(out, {0x1e8, 0x200, 0x218, 0x230, 0x248, 0x260, 0x278, 0x290, 0x2a8, 0x2c0,
                            0x2d8, 0x2f0, 0x308, 0x320});
        return out;
    }
    Json force() {
        auto out = record("PointForce", 1);
        out["members"] = Json::array();
        for (unsigned member = 0; member < 24; member += 4) {
            auto value = word(4);
            value["native_member_offset"] = member;
            out["members"].push_back(value);
        }
        if (out["version"] == 1) {
            auto value = word(4);
            value["native_member_offset"] = 24;
            out["members"].push_back(value);
            out["flag_member_0x1c"] = r.u8();
        }
        return out;
    }
    Json point() {
        auto out = record("Pt3D_ST", 1);
        out["position"] = r.number("3f");
        if (out["version"] == 1)
            out["point_forces"] = collect(r, [&]() { return force(); }, 'Q', 28);
        return out;
    }
    template <class F>
    Json collection(const char *member, const char *kind, F decode, std::size_t minimum) {
        auto offset = r.p;
        auto entries = collect(r, decode, 'Q', minimum);
        return {{"offset", offset},
                {"native_member", member},
                {"kind", kind},
                {"count", entries.size()},
                {"entries", entries}};
    }
    Json integer_map(const char *member) {
        return collection(
            member, "map",
            [&]() {
                auto offset = r.p;
                auto key = r.i32(), value = r.i32();
                return Json{{"offset", offset}, {"key", key}, {"value", value}};
            },
            8);
    }
    Json material_map() {
        return collection(
            "PBWJData+0x60", "map",
            [&]() {
                auto offset = r.p;
                auto key = r.i32();
                auto value = record("PSsimpleMat", 0);
                value["members"] = Json::array({word(4), word(4)});
                return Json{{"offset", offset}, {"key", key}, {"value", value}};
            },
            16);
    }
    Json point_map(const char *member) {
        return collection(
            member, "map",
            [&]() {
                auto offset = r.p;
                auto key = r.i32();
                return Json{{"offset", offset}, {"key", key}, {"value", point()}};
            },
            20);
    }
    Json bars() {
        return collection(
            "PBWJData+0xa0:wj_bars", "map",
            [&]() {
                auto offset = r.p;
                auto key = r.i32();
                auto out = record("WJ_Bar", 1);
                out["members_0x0_to_0xc"] = Json::array({word(4), word(4), word(4), word(4)});
                out["point_member_0x10"] = point();
                out["point_member_0x40"] = point();
                out["member_0x70"] = word(4);
                if (out["version"] == 1)
                    out["bar_forces"] = collect(
                        r,
                        [&]() {
                            auto value = record("WJ_BarForce", 1);
                            value["force_member_0x0"] = force();
                            value["force_member_0x24"] = force();
                            value["flag_member_0x48"] = r.u8();
                            if (value["version"] == 1)
                                value["member_0x4c"] = word(4);
                            return value;
                        },
                        'Q', 61);
                return Json{{"offset", offset}, {"key", key}, {"value", out}};
            },
            60);
    }
    Json truss_parameter() {
        auto out = record("TrussBeamPar", 3);
        auto version = out["version"].get<unsigned>();
        out["members"] = Json::array();
        // The native reader and writer both omit the member payload for version 2.
        // Values in the runtime object are not serialized and must not be invented here.
        if (version == 2) {
            out["member_payload_status"] = "not_serialized";
            return out;
        }
        // Version 0 omits native member 0x4. Versions 1 and 3
        // include it; only version 1 serializes member 0x20.
        for (unsigned member = 0; member <= (version == 1 ? 32u : 28u); member += 4) {
            if (version == 0 && member == 4)
                continue;
            auto value = word(4);
            value["native_member_offset"] = member;
            out["members"].push_back(value);
        }
        return out;
    }
    Json truss_beam_parameters() {
        return collection(
            "PBWJData+0x138", "map",
            [&]() {
                auto offset = r.p;
                auto key = r.i32();
                return Json{{"offset", offset}, {"key", key}, {"value", truss_parameter()}};
            },
            36);
    }
    Json sections() {
        return collection(
            "PBWJData+0x48", "vector",
            [&]() {
                auto out = record("PSXsectAndMatType", 0);
                auto start = r.p;
                auto raw = r.take(800);
                Reader group(raw);
                auto type = group.get<std::int16_t>();
                auto header = group.get<std::int16_t>();
                auto version = header < 10 ? header : 0;
                auto length = header < 10 ? group.get<std::int16_t>() : header;
                require(version >= 0 && length >= static_cast<int>(group.p) && length <= 800,
                        "invalid section group header");
                // The native converter bounds every field by this embedded length.
                // The enclosing archive still stores the entire 800-byte capacity.
                auto body = slice(raw, 0, static_cast<std::size_t>(length));
                Reader bounded(body, group.p);
                ElecCollections fields{bounded, {}};
                Json decoded = {{"offset", start},
                                {"offset_basis", "section_group_start"},
                                {"section_type", type},
                                {"version", version},
                                {"serialized_length", length},
                                {"members", Json::array()}};
                auto label = section_type_labels.find(type);
                if (label != section_type_labels.end())
                    decoded["section_type_label"] = label->second;
                auto count_offset = bounded.p;
                auto extra_count = bounded.get<std::int16_t>();
                require(extra_count >= 0 &&
                            static_cast<std::size_t>(extra_count) <= bounded.left() / 4,
                        "section extension count overrun");
                Json extra = Json::array();
                for (int i = 0; i < extra_count; ++i)
                    extra.push_back(fields.word(4));
                fields.member(decoded, 0xc2,
                              {{"offset", count_offset},
                               {"count", extra_count},
                               {"native_count_member_offset", 0xc2},
                               {"native_values_member_offset", 0xc4},
                               {"entries", extra},
                               {"kind", "vector"}});
                if (version > 0)
                    fields.scalar(decoded, 0x48, 2);
                fields.scalar(decoded, 0, 4);
                for (auto offset : {0x4au, 0x72u, 0x9au}) {
                    auto pos = bounded.p;
                    auto size = bounded.get<std::int16_t>();
                    require(size >= 0, "negative section string length");
                    auto bytes = bounded.take(static_cast<std::size_t>(size));
                    Json value = {{"offset", pos},
                                  {"kind", "string"},
                                  {"source_bytes", rawbytes(bytes)},
                                  {"encoding", "native_ansi"}};
                    // The archive does not carry a Windows code-page identifier.
                    // Keep original bytes even when a display string can be decoded.
                    if (std::all_of(bytes.begin(), bytes.end(),
                                    [](std::uint8_t c) { return c < 128; })) {
                        value["text"] = latin1(bytes);
                        value["text_encoding"] = "ascii";
                    }
                    fields.member(decoded, offset, std::move(value));
                }
                std::vector<std::pair<unsigned, unsigned>> layout;
                // XSect's converter has explicit layouts and a native generic branch.
                switch (type) {
                case 1:
                    layout = {{0x8, 2}, {0xa, 2}};
                    break;
                case 2:
                    layout = {{0x16, 2}, {0x1c, 2}, {0x24, 4}, {0x28, 4}, {0x2c, 4}};
                    break;
                case 3:
                    layout = {{0x8, 2}};
                    break;
                case 4:
                    layout = {{0x8, 2}, {0xa, 2}};
                    break;
                case 5:
                    layout = {{0x10, 2}, {0xc, 2}, {0x8, 2},  {0xa, 2},
                              {0x12, 2}, {0xe, 2}, {0x16, 2}, {0x18, 2}};
                    break;
                case 6:
                    layout = {{0x16, 2}, {0x18, 2}, {0x1c, 2}, {0x1e, 2}, {0x24, 4}, {0x28, 4}};
                    break;
                case 7:
                    layout = {{0x10, 2}, {0xa, 2}, {0x8, 2}, {0x12, 2}, {0x16, 2}, {0x18, 2}};
                    break;
                case 8:
                    layout = {{0x16, 2}, {0x18, 2}, {0xa, 2}, {0x24, 4}, {0x28, 4}, {0x1a, 2}};
                    break;
                case 9:
                    layout = {{0x10, 2}, {0xc, 2}, {0xa, 2}, {0x8, 2}, {0xe, 2}, {0x12, 2}};
                    break;
                case 10:
                    layout = {{0xa, 2}, {0x8, 2}, {0xe, 2}, {0x12, 2}};
                    break;
                case 15:
                    layout = {{0x16, 2}, {0x1c, 2}, {0x3c, 4}, {0x40, 4}};
                    break;
                case 16:
                case 40:
                    layout = {{0x16, 2}, {0x1c, 2}, {0x18, 2}, {0x3c, 4}, {0x40, 4}, {0x44, 4}};
                    break;
                case 17:
                    layout = {{0x16, 2}, {0x3c, 4}, {0x12, 2}};
                    break;
                case 18:
                    layout = {{0x16, 2}, {0x1c, 2}, {0x18, 2}, {0x3c, 4}, {0xe, 2}};
                    break;
                case 19:
                case 20:
                    layout = {{0x8, 2}, {0xa, 2}, {0xc, 2}, {0xe, 2}, {0x10, 2}, {0x12, 2}};
                    break;
                case 23:
                    layout = {{0x16, 2}, {0x18, 2}, {0x1c, 2}, {0x1e, 2},
                              {0x24, 4}, {0x28, 4}, {0x2c, 4}, {0x30, 4}};
                    break;
                case 25:
                    layout = {{0x16, 2}, {0x1c, 2}, {0x1e, 2}};
                    break;
                case 26:
                    layout = {{0x16, 2}, {0x1c, 2}, {0x1e, 2}, {0x3c, 4}, {0x40, 4}};
                    break;
                case 27:
                    layout = {{0x16, 2}, {0x18, 2}, {0x1c, 2}, {0x1e, 2},
                              {0x3c, 4}, {0x40, 4}, {0x44, 4}, {0x12, 2}};
                    break;
                case 31:
                case 32:
                case 36:
                case 37:
                case 38:
                case 39:
                case 41:
                    layout = {{0x8, 2}, {0x16, 2}, {0x1c, 2}, {0x3c, 4}, {0x40, 4}, {0x44, 4}};
                    break;
                case 33:
                    layout = {{0x8, 2}, {0x16, 2}, {0x1c, 2}, {0x3c, 4}, {0x44, 4}};
                    break;
                case 34:
                    layout = {{0x8, 2},  {0x16, 2}, {0x1c, 2}, {0x3c, 4}, {0x44, 4},
                              {0x30, 4}, {0x1a, 2}, {0x2c, 4}, {0x20, 2}, {0x34, 4}};
                    break;
                case 35:
                    layout = {{0x8, 2},  {0x16, 2}, {0x1c, 2}, {0x3c, 4},
                              {0x40, 4}, {0x44, 4}, {0x30, 4}};
                    break;
                case 101:
                case 102:
                    layout = {{0x8, 2}, {0xa, 2}, {0x16, 2}, {0x1c, 2}, {0x3c, 4}, {0x40, 4}};
                    break;
                case 103:
                    layout = {{0x8, 2}, {0xa, 2}, {0x16, 2}, {0x1c, 2}, {0x24, 4}, {0x30, 4}};
                    break;
                case 104:
                    layout = {{0x8, 2},  {0xa, 2},  {0x16, 2}, {0x1c, 2}, {0x18, 2},
                              {0x1e, 2}, {0x24, 4}, {0x28, 4}, {0x2c, 4}, {0x30, 4}};
                    break;
                case 105:
                    layout = {{0x8, 2}, {0xa, 2}, {0x16, 2}, {0x3c, 4}, {0x12, 2}};
                    break;
                case 106:
                    layout = {{0x8, 2}, {0x3c, 4}, {0x1c, 2}, {0x16, 2}, {0x40, 4}};
                    break;
                case 107:
                    layout = {{0x8, 2}, {0x16, 2}, {0x3c, 4}, {0x12, 2}};
                    break;
                case 108:
                    layout = {{0x8, 2},  {0x24, 4}, {0x30, 4}, {0x1c, 2}, {0x18, 2},
                              {0x28, 4}, {0x2c, 4}, {0x16, 2}, {0x1e, 2}};
                    break;
                case 201:
                    layout = {{0x8, 2}, {0xa, 2}, {0x3c, 4}, {0x40, 4}, {0x12, 2}};
                    break;
                case 202:
                    layout = {{0x16, 2}, {0x3c, 4}, {0x12, 2}};
                    break;
                case 302:
                    layout = {{0x8, 2}, {0xa, 2}};
                    break;
                case 303:
                    layout = {{0x16, 2}, {0x18, 2}, {0xa, 2}, {0x24, 4}, {0x28, 4}, {0x1a, 2}};
                    break;
                case 304:
                    layout = {{0x10, 2}, {0xc, 2}, {0x8, 2},  {0xa, 2},
                              {0x12, 2}, {0xe, 2}, {0x16, 2}, {0x18, 2}};
                    break;
                default:
                    layout = {{0x8, 2},  {0xa, 2},  {0xc, 2},  {0xe, 2},  {0x10, 2}, {0x12, 2},
                              {0x16, 2}, {0x18, 2}, {0x1a, 2}, {0x1c, 2}, {0x1e, 2}, {0x20, 2},
                              {0x24, 4}, {0x28, 4}, {0x2c, 4}, {0x30, 4}, {0x34, 4}, {0x38, 4},
                              {0x3c, 4}, {0x40, 4}, {0x44, 4}};
                    break;
                }
                for (const auto &field : layout)
                    fields.scalar(decoded, field.first, field.second);
                auto subtypes = section_subtype_labels.find(type);
                if (subtypes != section_subtype_labels.end()) {
                    for (auto &field : decoded["members"]) {
                        if (field["native_member_offset"] != 8)
                            continue;
                        const auto code = field["int16_view"].get<std::int16_t>();
                        field.update({{"name", "section_subtype"},
                                      {"value", code},
                                      {"storage_type", "int16"},
                                      {"semantic_status", "identified"},
                                      {"enum_status", "unknown_value"}});
                        decoded["section_subtype"] = code;
                        if (code > 0 && std::size_t(code) <= subtypes->second.size()) {
                            const auto label = subtypes->second[std::size_t(code) - 1];
                            field["enum_label"] = label;
                            field["enum_status"] = "identified";
                            decoded["section_type_label"] = label;
                        }
                        break;
                    }
                }
                bounded.finish();
                decoded["unused_capacity"] = {
                    {"offset", length},
                    {"source_bytes", rawbytes(slice(raw, length, 800 - length))}};
                out["section_group"] = std::move(decoded);
                out["member_0xcc"] = word(4);
                return out;
            },
            808);
    }
    Json line_groups() {
        return collection(
            "PBWJData+0x1e0", "vector",
            [&]() {
                auto out = record("LineSubsBeamCols", 0);
                out["member_0x0"] = words64();
                out["member_0x18"] = words64();
                return out;
            },
            20);
    }
    Json grid_axes() {
        return collection(
            "PBWJData+0x1f8", "vector",
            [&]() {
                auto out = record("GridAxisData", 0);
                out["member_0x0"] = word(4);
                out["member_0x4"] = word(4);
                out["point_member_0x8"] = point();
                out["member_0x38"] = word(8);
                out["member_0x40"] = words64();
                out["member_0x58"] = words64();
                return out;
            },
            52);
    }
};
} // namespace
Json electrical_parameters(Reader &r, unsigned ver) {
    Json out = Json::object();
    ElecCollections elements{r, {}};
    out["cereal_version"] = r.expect("I", 0);
    auto wj_version = r.u32();
    require(wj_version <= 2, "unsupported PBWJData version");
    out["wj_version"] = wj_version;
    out["strings"] = Json::array({r.string(), r.string()});
    out["section_parameters"] = {{"native_type", "PSXSectManager"},
                                 {"cereal_version", r.expect("I", 0)},
                                 {"version", r.expect("I", 0)},
                                 {"collection", elements.sections()}};
    out["collections"] = Json::array();
    out["collections"].push_back(elements.material_map());
    out["collections"].push_back(elements.integer_map("PBWJData+0x70"));
    out["collections"].push_back(elements.integer_map("PBWJData+0x80"));
    out["collections"].push_back(elements.point_map("PBWJData+0x90:wj_joints"));
    out["collections"].push_back(elements.bars());
    out["boolean_member_0x110"] = {{"offset", r.p}, {"storage_uint8", r.u8()}, {"name", nullptr}};
    out["collections"].push_back(elements.columns());
    out["collections"].push_back(elements.truss_beam_parameters());
    out["collections"].push_back(elements.collection(
        "PBWJData+0x168", "vector", [&]() { return elements.truss_beam(); }, 172));
    out["collections"].push_back(elements.collection(
        "PBWJData+0x180", "vector", [&]() { return elements.human_column(); }, 157));
    if (wj_version >= 1)
        out["collections"].push_back(elements.line_groups());
    if (wj_version >= 2)
        out["collections"].push_back(elements.grid_axes());
    if (ver == 1)
        out["changed_joints"] = elements.point_map("ElecParaData+0x20");
    return out;
}
} // namespace p3d
