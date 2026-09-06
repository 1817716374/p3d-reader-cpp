#include "blob_internal.hpp"
namespace p3d {
static Json list(Reader &r, std::function<Json()> f, std::size_t min = 1, char fmt = 'Q') {
    auto n = r.count(fmt, min);
    Json out = Json::array();
    for (std::uint64_t i = 0; i < n; ++i)
        out.push_back(f());
    return out;
}
static Json ref(Reader &r) {
    auto v = r.expect("I", 1);
    auto guid = r.string();
    return {
        {"version", v}, {"document_guid", guid}, {"model_id", r.u32()}, {"element_id", r.u64()}};
}
static Json tokens(const Bytes &b) {
    Reader r(b);
    Json ts = Json::array();
    while (r.left()) {
        auto off = r.p;
        auto kind = r.u32();
        Json item = {{"offset", off}, {"type_code", kind}};
        std::map<unsigned, std::pair<std::string, std::string>> fmt = {
            {0, {"B", "unassigned_byte"}},
            {1, {"i", "int32"}},
            {2, {"d", "double"}},
            {3, {"q", "int64"}},
            {6, {"3d", "point3d"}}};
        if (fmt.count(kind)) {
            item["value"] = r.number(fmt[kind].first);
            item["kind"] = fmt[kind].second;
        } else if (kind == 4 || kind == 5) {
            auto data = r.take(r.u32());
            if (kind == 5)
                item.update(
                    {{"kind", "binary"}, {"binary_base64", base64(data)}, {"bytes", data.size()}});
            else {
                std::string text, enc = "utf8";
                try {
                    text = utf8(data);
                } catch (...) {
                    text = gb18030(data);
                    enc = "gb18030";
                }
                item.update({{"kind", "string"}, {"value", text}, {"encoding", enc}});
            }
        } else
            throw std::runtime_error("property token type");
        ts.push_back(item);
    }
    require(ts.size() >= 2 && ts[0]["kind"] == "string" && ts[1]["kind"] == "string",
            "property block namespace/class");
    return {{"namespace", ts[0]["value"]},
            {"class_name", ts[1]["value"]},
            {"tokens", ts},
            {"note", "Ordered typed values, including property names and declared types. Object "
                     "scopes and type-0 byte semantics remain unassigned."}};
}
static Json bfa(const Bytes &b) {
    Reader r(b);
    auto reference = [&](Reader &r) {
        require(hex(r.take(3)) == "7c2340", "BFA reference");
        return r.u64();
    };
    Json records = Json::array();
    while (r.left()) {
        auto off = r.p;
        auto tag = utf8(r.take(3));
        require(tag == "~$^" || tag == "@#$" || tag == "!_#" || tag == "&@`", "BFA type");
        auto oid = reference(r);
        auto body = r.take(r.u64());
        auto children = list(r, [&]() { return Json(reference(r)); }, 11, 'I');
        Reader br(body);
        require(body.size() >= 19 && reference(br) == oid, "BFA body identity");
        Json item = {{"offset", off},
                     {"type_tag", tag},
                     {"id", oid},
                     {"children", children},
                     {"unassigned_header_uint64", br.u64()},
                     {"body_base64", base64(body)}};
        if (tag == "&@`") {
            if (br.left())
                try {
                    item["property_block"] = tokens(br.take(br.left()));
                } catch (const std::exception &e) {
                    item["decode_error"] = e.what();
                }
            else
                item["empty_property_block"] = true;
        } else {
            if (tag == "!_#")
                item["unassigned_prefix_byte"] = br.u8();
            Json names = Json::array();
            for (int i = 0; i < (tag == "@#$" ? 1 : 2); ++i)
                names.push_back(gb18030(br.take(br.u32())));
            item["names"] = names;
            item["unassigned_suffix_hex"] = hex(br.take(br.left()));
        }
        records.push_back(item);
    }
    std::set<std::uint64_t> ids, external;
    for (auto &v : records)
        ids.insert(v["id"].get<std::uint64_t>());
    for (auto &v : records)
        for (auto &c : v["children"])
            if (!ids.count(c))
                external.insert(c.get<std::uint64_t>());
    return {{"records", records},
            {"external_child_ids", external},
            {"note", "All graph records and embedded property tokens retained. Parameter-node "
                     "suffixes and some node header semantics remain unassigned."}};
}
static Json cached(const Bytes &b) {
    Reader r(b);
    require(b.size() >= 80, "cached geometry length");
    auto version = r.f64();
    auto count = r.u32(), kind = r.u32();
    require(version == 3.1 && count == 1, "cached geometry version/count");
    Json out = {{"version", version},
                {"count", count},
                {"kind_code", kind},
                {"display_header_hex", hex(slice(b, 16, 56))},
                {"display_rgb", r.uints(3)}};
    r.p = 72;
    if (kind == 3) {
        auto header = r.number("3IB");
        auto points = list(r, [&]() { return r.doubles(3); }, 24, 'I');
        require(r.take(24) == Bytes(24), "cached auxiliary channels");
        auto indices = list(r, [&]() { return Json(r.i32()); }, 4, 'I');
        require(r.left() == 68, "cached mesh footer");
        require(r.take(12) == Bytes(12), "cached additional indices");
        Json polygons = Json::array(), face = Json::array();
        for (auto &index : indices) {
            auto i = index.get<std::int64_t>();
            require(std::llabs(i) <= static_cast<long long>(points.size()), "cached point index");
            if (i)
                face.push_back(i);
            else if (!face.empty()) {
                polygons.push_back(face);
                face = Json::array();
            }
        }
        require(face.empty(), "cached polygon termination");
        out.update(
            {{"kind", "indexed_mesh"},
             {"points", points},
             {"indices", indices},
             {"polygons", polygons},
             {"mesh_header_fields", header},
             {"empty_auxiliary_channels", 6},
             {"empty_additional_indices", 3},
             {"footer_hex", hex(r.take(r.left()))},
             {"unassigned_regions", {"display_header_hex", "mesh_header_fields", "footer_hex"}}});
    } else if (kind == 2) {
        auto boundary = r.u32();
        auto curves = list(
            r,
            [&]() {
                auto op = r.u32();
                if (op == 2)
                    return Json{{"kind", "polyline"},
                                {"points", list(r, [&]() { return r.doubles(3); }, 24, 'I')}};
                require(op == 3, "cached curve opcode");
                return Json{{"kind", "elliptical_arc"}, {"origin", r.doubles(3)},
                            {"vector0", r.doubles(3)},  {"vector90", r.doubles(3)},
                            {"start_angle", r.f64()},   {"sweep_angle", r.f64()}};
            },
            4, 'I');
        require(r.left() == 40, "cached curve footer");
        auto footer = hex(slice(b, r.p, r.left()));
        out.update({{"kind", "curve_collection"},
                    {"boundary_code", boundary},
                    {"curves", curves},
                    {"footer_rgb", r.uints(3)},
                    {"footer_hex", footer},
                    {"unassigned_regions", {"display_header_hex", "boundary_code", "footer_hex"}}});
        r.skip(r.left());
    } else
        throw std::runtime_error("cached geometry kind");
    return out;
}
Json decode_inline_material(const Bytes &b) {
    Reader r(b);
    auto string = [&]() {
        auto n = r.u64();
        require(n % 2 == 0, "inline material string width");
        return utf16(r.take(n));
    };
    auto boolean = [&]() {
        auto flag = r.u8();
        require(flag <= 1, "inline material boolean");
        return flag;
    };
    auto marked = [&](const std::string &label, unsigned n) {
        auto flag = boolean();
        return Json{{"kind", label},
                    {"flag", flag},
                    {"enabled", flag != 0},
                    {"value", n == 1 ? Json(r.f64()) : r.doubles(n)}};
    };
    auto valid = boolean();
    auto name = string();
    Json parameters = Json::array({marked("color", 3), marked("transparency", 1)});
    auto flag = boolean();
    auto unit = r.i32(), mode = r.i32();
    auto filename = string();
    auto scale = r.doubles(2), offset = r.doubles(2);
    auto rotation = r.f64();
    auto bump_filename = string();
    auto bump_factor = r.f64();
    for (auto k :
         {"specular_color", "specular_factor", "glow_color", "glow_factor", "ambient_factor",
          "diffuse_factor", "roughness_factor", "reflect_factor", "refract_factor"})
        parameters.push_back(marked(k, std::string(k).find("color") != std::string::npos ? 3 : 1));
    auto footer_start = r.p;
    Json out = {
        {"encoding", "inline_material_v1"},
        {"wire_format", "BPMaterial_unversioned"},
        {"is_valid", valid != 0},
        {"name", name},
        {"parameters", parameters},
        {"has_map", flag != 0},
        {"map_unit", unit},
        {"map_mode", mode},
        {"uv_scale", scale},
        {"uv_offset", offset},
        {"rotation_degrees", rotation},
        {"bump_factor", bump_factor},
        {"texture_references",
         Json::array(
             {{{"role", "pattern"}, {"filename", filename}, {"flag", flag}, {"enabled", flag != 0}},
              {{"role", "bump"}, {"filename", bump_filename}}})},
        {"display_name", name},
        {"display_name_source", "name_fallback"}};
    static const std::map<int, std::string> units = {{0, "relative"}, {3, "absolute"}};
    static const std::map<int, std::string> modes = {{0, "parametric"}, {1, "elevation_drape"},
                                                     {2, "planar"},     {4, "cubic"},
                                                     {5, "spherical"},  {6, "cylindrical"}};
    out["map_unit_status"] = units.count(unit) ? "identified" : "unknown_value";
    out["map_mode_status"] = modes.count(mode) ? "identified" : "unknown_value";
    if (units.count(unit))
        out["map_unit_name"] = units.at(unit);
    if (modes.count(mode))
        out["map_mode_name"] = modes.at(mode);
    if (r.left()) {
        r.expect("I", 0xabcd);
        out["display_name"] = string();
        out["display_name_source"] = "serialized";
    }
    if (r.left() >= 4 && Reader(b, r.p).u32() == 0xabce) {
        auto start = r.p;
        r.u32();
        auto n = r.u64();
        require(n >= 2 && n % 2 == 0, "inline material extension string width");
        auto raw = r.take(n);
        require(raw[raw.size() - 2] == 0 && raw.back() == 0,
                "inline material extension string terminator");
        auto text = utf16(slice(raw, 0, raw.size() - 2));
        Json extension = {{"offset", start}, {"text", text}, {"source_bytes", rawbytes(raw)}};
        if (text.empty())
            extension["json_status"] = "empty";
        else {
            auto value = Json::parse(text, nullptr, false);
            if (value.is_discarded())
                extension["json_status"] = "invalid_json";
            else {
                extension["json_status"] = "parsed";
                extension["value"] = std::move(value);
            }
        }
        out["extended_data"] = std::move(extension);
    }
    // Preserve future extensions beyond the known display-name and JSON blocks.
    if (r.left())
        out["unassigned_suffix_hex"] = hex(r.take(r.left()));
    out["footer_hex"] = hex(slice(b, footer_start, b.size() - footer_start));
    out["note"] = "Source material flags and parameters; texture paths are references. "
                  "The encoding label is a library identifier, not a serialized version byte.";
    return out;
}
static Json instance_body(const Bytes &b) {
    Reader r(b);
    require(b.size() >= 155 && r.u32() == 1, "component instance body");
    r.p = 33;
    Json matrix = Json::array();
    for (int i = 0; i < 3; ++i)
        matrix.push_back(r.doubles(4));
    auto flag = r.u8();
    auto word = r.u32();
    auto n = r.u64();
    require(n <= b.size() / 8, "component geometry count");
    r.p = 142;
    Json geo = Json::array();
    for (std::uint64_t i = 0; i < n; ++i) {
        auto data = r.take(r.u32());
        auto index = r.i32();
        Json packet = {{"unassigned_index", index}, {"raw_base64", base64(data)}};
        try {
            Reader p(data);
            auto head = hex(p.take(32));
            auto size = p.u64();
            auto g = p.take(size);
            packet.update(
                {{"header_hex", head},
                 {"geometry_bytes", size},
                 {"geometry", decode_bgfb(g)},
                 {"suffix_hex", hex(p.take(p.left()))},
                 {"note", "Packet display/header and suffix semantics remain unassigned."}});
        } catch (const std::exception &e) {
            packet["decode_error"] = e.what();
        }
        geo.push_back(packet);
    }
    auto off = r.p;
    auto ver = r.u8();
    auto size = r.u32();
    require(ver == 3 && r.left() == std::uint64_t(size) + 8, "component footer");
    auto matraw = r.take(size);
    Json mat = nullptr;
    if (size) {
        mat = {{"raw_base64", base64(matraw)}};
        try {
            mat.update(decode_inline_material(matraw));
        } catch (const std::exception &e) {
            mat["decode_error"] = e.what();
        }
    }
    return {{"version", 1},
            {"header_hex", hex(slice(b, 0, 33))},
            {"transform_3x4_rows", matrix},
            {"unassigned_flag", flag},
            {"unassigned_uint32", word},
            {"geometry_packets", geo},
            {"inline_material", mat},
            {"footer_version", ver},
            {"footer_hex", hex(slice(b, off, b.size() - off))}};
}
static std::uint32_t be32(Reader &r) {
    auto b = r.take(4);
    return (unsigned(b[0]) << 24) | (unsigned(b[1]) << 16) | (unsigned(b[2]) << 8) | b[3];
}
Json boolean_record(const Bytes &b) {
    Reader r(b);
    auto payload = r.take(be32(r));
    Reader p(payload);
    Json operands = Json::array();
    while (p.left())
        operands.push_back(instance_body(p.take(be32(p))));
    std::function<Json(const Bytes &, unsigned)> tree = [&](const Bytes &b, unsigned depth) {
        require(depth <= 80, "boolean tree depth");
        Reader t(b);
        auto header = t.take(be32(t));
        Reader h(header);
        auto op = be32(h), leaf = be32(h);
        require(leaf <= 1 && header.size() == 8 + 4 * leaf, "boolean node header");
        Json node = {{"operation_code", op}, {"is_leaf", bool(leaf)}, {"children", Json::array()}};
        if (leaf) {
            auto ix = be32(h);
            require(ix < operands.size(), "boolean operand index");
            node["operand_index"] = ix;
        }
        while (t.left())
            node["children"].push_back(tree(t.take(be32(t)), depth + 1));
        require(leaf ? node["children"].empty() : node["children"].size() == 2,
                "boolean child arity");
        return node;
    };
    auto expression = tree(r.take(be32(r)), 0);
    r.finish();
    return {{"operands", operands},
            {"expression", expression},
            {"note", "Operand geometry and expression topology retained. Operation codes and "
                     "inactive-transform/display flags need renderer confirmation; no boolean "
                     "re-evaluation is substituted for stored scene geometry."}};
}
static Json feature_setting(Reader &r) {
    auto mapping = [&](std::function<Json()> fn) {
        return list(r, [&]() { return Json{{"key", r.string()}, {"value", fn()}}; }, 8);
    };
    auto strmap = [&]() { return mapping([&]() { return Json(r.string()); }); };
    auto feature = [&]() {
        return Json{{"header", r.expect("II", {2, 1})},
                    {"parent_path", r.string()},
                    {"unassigned_string", r.string()},
                    {"name", r.string()},
                    {"alias", r.string()},
                    {"template_path", r.string()},
                    {"component_paths", strmap()},
                    {"related_feature_paths", strmap()}};
    };
    auto style = [&]() {
        return Json{{"header", r.expect("II", {1, 1})},    {"parent_path", r.string()},
                    {"unassigned_string", r.string()},     {"name", r.string()},
                    {"display_name", r.string()},          {"unassigned_uint64", r.u64()},
                    {"unassigned_uint32", r.number("III")}};
    };
    return {{"version", r.expect("I", 3)},    {"path", r.string()},
            {"feature", feature()},           {"style", style()},
            {"named_styles", mapping(style)}, {"named_features", mapping(feature)}};
}
static Json vector_mesh(Reader &r) {
    auto v = r.get<float>();
    auto nf = r.u32(), nv = r.u32();
    require(v == 1.f && nv <= r.b.size() / 52 && nf <= r.b.size() / 32, "vector mesh header");
    std::set<std::uint32_t> ids;
    Json vertices = Json::array(), faces = Json::array();
    for (unsigned i = 0; i < nv; ++i) {
        auto p = r.doubles(3), u = r.doubles(3);
        auto id = r.u32();
        require(ids.insert(id).second, "vector duplicate ID");
        vertices.push_back({{"id", id}, {"position", p}, {"unassigned_vector", u}});
    }
    for (unsigned i = 0; i < nf; ++i) {
        auto u = r.doubles(3);
        auto word = r.u32();
        Json edges = Json::array(), verts = Json::array();
        while (true) {
            auto id = r.u32();
            if (!id)
                break;
            auto a = r.u32(), b = r.u32();
            require(ids.count(a) && ids.count(b), "vector vertex missing");
            edges.push_back({{"face_id", id}, {"start", a}, {"end", b}});
            verts.push_back(a);
        }
        require(!edges.empty(), "vector empty face");
        for (std::size_t j = 0; j < edges.size(); ++j)
            require(edges[j]["face_id"] == edges[0]["face_id"] &&
                        edges[j]["end"] == edges[(j + 1) % edges.size()]["start"],
                    "vector face identity/closure");
        faces.push_back({{"id", edges[0]["face_id"]},
                         {"unassigned_vector", u},
                         {"unassigned_uint32", word},
                         {"edges", edges},
                         {"vertex_ids", verts}});
    }
    return {{"version", v}, {"vertices", vertices}, {"faces", faces}};
}
Json complex_blob(const std::string &name, const Bytes &b) {
    Reader r(b);
    Json out;
    if (name == "GraphicsGeom")
        return cached(b);
    if (name == "DataBlock")
        return tokens(b);
    if (name == "BfaTree")
        return bfa(b);
    if (name == "FeatureSetting")
        out = feature_setting(r);
    else if (name == "GraphicsVecExtend")
        out = vector_mesh(r);
    else if (name == "RuleRelation") {
        auto ver = r.expect("I", 2);
        auto first = list(
            r, [&]() { return Json{{"reference", ref(r)}, {"unassigned_uint32", r.u32()}}; }, 24);
        auto second = list(r, [&]() { return ref(r); }, 24);
        auto third = r.expect("Q", 0);
        out = {{"version", ver},
               {"reference_map", first},
               {"references", second},
               {"unassigned_empty_collection_count", third}};
    } else if (name == "NestRelation") {
        auto ver = r.expect("I", 1);
        auto children = list(r, [&]() { return ref(r); }, 24);
        auto parent = ref(r);
        out = {{"version", ver}, {"children", children}, {"parent", parent}};
    } else if (name == "Definition") {
        r.expect("4I", {1, 302, 1, 1});
        auto points = list(
            r,
            [&]() {
                r.expect("I", 1);
                return r.doubles(3);
            },
            28);
        out = {{"version", 1},
               {"kind_code", 302},
               {"nested_versions", {1, 1}},
               {"point_version", 1},
               {"points", points},
               {"note", "Ordered exact source points; kind-code 302 is preserved. No additional "
                        "interpolation rule is inferred."}};
    } else if (name == "ParaCmptInstance") {
        auto reference = [&]() -> Json {
            auto data = r.take(11);
            if (data == Bytes(11))
                return nullptr;
            Reader dr(data);
            require(hex(dr.take(3)) == "7c2340", "component reference");
            return dr.u64();
        };
        auto parent = reference();
        auto instances = list(
            r,
            [&]() {
                auto oid = reference();
                auto body = instance_body(r.take(r.u32()));
                body["object_id"] = oid;
                return body;
            },
            15, 'I');
        require(r.left() == 25, "component list footer");
        out = {
            {"parent_id", parent},
            {"instances", instances},
            {"footer_hex", hex(r.take(r.left()))},
            {"note",
             "Reference list, 3x4 transforms and geometry packets decoded. Display/header/footer "
             "semantics remain unassigned; caches are not added again to the visible scene."}};
    } else
        throw std::runtime_error("unsupported complex field");
    r.finish();
    return out;
}
} // namespace p3d
