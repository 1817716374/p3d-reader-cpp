#include "blob_internal.hpp"
namespace p3d {
Json binary_json(const Bytes &b) {
    Reader r(b);
    auto uint = [&](std::size_t p, unsigned w, std::size_t limit) {
        require(p <= limit && w <= limit - p && limit <= b.size(), "binary JSON bounds");
        Reader t(b, p);
        return w == 1 ? t.u8() : w == 2 ? t.u16() : t.u32();
    };
    auto take = [&](std::size_t p, std::size_t n, std::size_t end) {
        require(p <= end && n <= end - p, "binary JSON bounds");
        return slice(b, p, n);
    };
    auto varint = [&](std::size_t &p, std::size_t end) {
        unsigned n = 0;
        for (unsigned s = 0; s < 35; s += 7) {
            auto v = uint(p++, 1, end);
            n |= (v & 127) << s;
            if (!(v & 128))
                return n;
        }
        throw std::runtime_error("binary JSON varint");
    };
    std::function<std::pair<Json, std::size_t>(unsigned, std::size_t, std::size_t, unsigned)>
        value = [&](unsigned kind, std::size_t pos, std::size_t limit,
                    unsigned depth) -> std::pair<Json, std::size_t> {
        require(depth <= 150, "binary JSON depth");
        if (kind <= 3) {
            bool large = kind == 1 || kind == 3, obj = kind == 0 || kind == 1;
            unsigned w = large ? 4 : 2;
            auto count = uint(pos, w, limit), size = uint(pos + w, w, limit);
            auto end = pos + size;
            require(count <= 1000000 && end <= limit && size >= 2 * w, "binary JSON container");
            auto ks = pos + 2 * w, vs = ks + (obj ? std::uint64_t(count) * (w + 2) : 0);
            require(vs + std::uint64_t(count) * (w + 1) <= end, "binary JSON entries");
            Json keys = Json::array(), values = obj ? Json::object() : Json::array();
            if (obj)
                for (unsigned i = 0; i < count; ++i) {
                    auto entry = ks + i * (w + 2);
                    keys.push_back(
                        utf8(take(pos + uint(entry, w, end), uint(entry + w, 2, end), end)));
                }
            for (unsigned i = 0; i < count; ++i) {
                auto entry = vs + i * (w + 1);
                auto tag = uint(entry, 1, end);
                bool in = tag == 4 || tag == 5 || tag == 6 || (large && (tag == 7 || tag == 8));
                auto target = in ? entry + 1 : pos + uint(entry + 1, w, end);
                auto v = value(tag, target, in ? entry + 1 + w : end, depth + 1).first;
                if (obj) {
                    auto key = keys[i].get<std::string>();
                    require(!values.contains(key), "binary JSON duplicate key");
                    values[key] = v;
                } else
                    values.push_back(v);
            }
            return {values, end};
        }
        if (kind == 4) {
            auto v = uint(pos, 1, limit);
            require(v <= 2, "binary JSON literal");
            return {v == 0 ? Json() : Json(v == 1), pos + 1};
        }
        std::map<unsigned, std::pair<std::string, unsigned>> fmt = {
            {5, {"h", 2}}, {6, {"H", 2}},  {7, {"i", 4}}, {8, {"I", 4}},
            {9, {"q", 8}}, {10, {"Q", 8}}, {11, {"d", 8}}};
        if (fmt.count(kind)) {
            auto raw = take(pos, fmt[kind].second, limit);
            Reader v(raw);
            return {v.number(fmt[kind].first), pos + raw.size()};
        }
        if (kind == 12) {
            auto p = pos;
            auto n = varint(p, limit);
            return {utf8(take(p, n, limit)), p + n};
        }
        if (kind == 15) {
            auto type = uint(pos, 1, limit);
            auto p = pos + 1;
            auto n = varint(p, limit);
            return {Json{{"mysql_opaque_type", type}, {"binary_base64", base64(take(p, n, limit))}},
                    p + n};
        }
        throw std::runtime_error("binary JSON type");
    };
    require(!b.empty(), "empty binary JSON");
    auto v = value(b[0], 1, b.size(), 0);
    require(v.second == b.size(), "binary JSON trailing bytes");
    return v.first;
}
Json binary_xml(const Bytes &b) {
    Reader r(b);
    auto str = [&]() {
        unsigned n = 0;
        for (unsigned s = 0; s < 35; s += 7) {
            auto v = r.u8();
            n |= (v & 127) << s;
            if (v < 128)
                return utf8(r.take(n));
        }
        throw std::runtime_error("NBFX string length");
    };
    auto text = [&](unsigned code) -> Json {
        auto kind = code & 0xfe;
        switch (kind) {
        case 0x80:
            return 0;
        case 0x82:
            return 1;
        case 0x84:
            return false;
        case 0x86:
            return true;
        case 0xa8:
            return "";
        case 0x88:
            return r.get<std::int8_t>();
        case 0x8a:
            return r.get<std::int16_t>();
        case 0x8c:
            return r.i32();
        case 0x8e:
            return r.i64();
        case 0x90:
            return r.get<float>();
        case 0x92:
            return r.f64();
        case 0xb2:
            return r.u64();
        case 0xb4:
            return bool(r.u8());
        default:
            break;
        }
        if ((kind >= 0x98 && kind <= 0xa2) || (kind >= 0xb6 && kind <= 0xba)) {
            unsigned start = kind < 0x9e ? 0x98 : kind < 0xb6 ? 0x9e : 0xb6;
            auto w = (kind - start) / 2;
            auto n = w == 0 ? r.u8() : w == 1 ? r.u16() : r.u32();
            auto raw = r.take(n);
            if (start == 0x9e)
                return Json{{"binary_base64", base64(raw)}};
            return start == 0xb6 ? utf16(raw) : utf8(raw);
        }
        throw std::runtime_error("NBFX text type");
    };
    Json roots = Json::array();
    std::vector<Json> stack;
    auto finish = [&]() {
        require(!stack.empty(), "NBFX unmatched end");
        Json n = std::move(stack.back());
        stack.pop_back();
        if (stack.empty())
            roots.push_back(std::move(n));
        else {
            auto &parent = stack.back();
            parent["content"].push_back({{"child_index", parent["children"].size()}});
            parent["children"].push_back(std::move(n));
        }
    };
    while (r.left()) {
        auto off = r.p;
        auto code = r.u8();
        if (code == 1)
            finish();
        else if (code == 0x40 || code == 0x41 || (code >= 0x5e && code <= 0x77)) {
            std::string prefix = code == 0x41   ? str()
                                 : code >= 0x5e ? std::string(1, char('a' + code - 0x5e))
                                                : "";
            require(stack.size() <= 200, "NBFX depth");
            stack.push_back({{"name", str()},
                             {"prefix", prefix},
                             {"attributes", Json::array()},
                             {"children", Json::array()},
                             {"content", Json::array()},
                             {"offset", off}});
        } else if (code == 4 || code == 5 || code == 8 || code == 9) {
            require(!stack.empty(), "NBFX attribute without element");
            auto prefix = code == 5 || code == 9 ? str() : "";
            Json attr;
            if (code == 8 || code == 9)
                attr = {{"name", "xmlns"}, {"prefix", prefix}, {"value", str()}};
            else {
                auto name = str();
                auto t = r.u8();
                require(t >= 0x80 && !(t & 1), "NBFX attribute text");
                attr = {{"name", name}, {"prefix", prefix}, {"value", text(t)}, {"record_type", t}};
            }
            stack.back()["attributes"].push_back(attr);
        } else if (code >= 0x80) {
            require(!stack.empty(), "NBFX text outside element");
            stack.back()["content"].push_back(
                {{"value", text(code)}, {"record_type", code}, {"offset", off}});
            if (code & 1)
                finish();
        } else
            throw std::runtime_error("NBFX record type");
    }
    require(stack.empty() && roots.size() == 1, "NBFX incomplete/multiple roots");
    return roots[0];
}
Json decode_bgfb(const Bytes &b) {
    require(b.size() >= 12 && utf8(slice(b, 0, 8)) == "bg0001fb", "BGFB signature");
    const auto &types = bgfb_schema().at("types");
    const std::map<std::string, std::pair<std::string, unsigned>> scalars = {
        {"bool", {"B", 1}},   {"byte", {"B", 1}},  {"ubyte", {"B", 1}}, {"short", {"h", 2}},
        {"ushort", {"H", 2}}, {"int", {"i", 4}},   {"uint", {"I", 4}},  {"long", {"q", 8}},
        {"ulong", {"Q", 8}},  {"float", {"f", 4}}, {"double", {"d", 8}}};
    struct Layout {
        std::vector<std::tuple<std::string, std::string, std::size_t>> fields;
        std::size_t size = 0, align = 1;
    };
    std::map<std::string, Layout> layouts;
    std::function<Layout(const std::string &)> layout = [&](const std::string &name) {
        if (layouts.count(name))
            return layouts.at(name);
        Layout l;
        for (auto &f : types.at(name).at("fields")) {
            auto t = f.at("type").get<std::string>();
            std::size_t size, align;
            if (scalars.count(t))
                size = align = scalars.at(t).second;
            else {
                auto nested = layout(t);
                size = nested.size;
                align = nested.align;
            }
            l.size = (l.size + align - 1) / align * align;
            l.fields.emplace_back(f.at("name").get<std::string>(), t, l.size);
            l.size += size;
            l.align = std::max(l.align, align);
        }
        l.size = (l.size + l.align - 1) / l.align * l.align;
        layouts[name] = l;
        return l;
    };
    auto checked = [&](std::size_t p, std::size_t n) {
        Reader r(b, p);
        r.need(n);
    };
    auto u32 = [&](std::size_t p) {
        Reader r(b, p);
        return r.u32();
    };
    auto u16 = [&](std::size_t p) {
        Reader r(b, p);
        return r.u16();
    };
    auto indirect = [&](std::size_t p) {
        auto off = u32(p);
        require(off >= 4, "BGFB relative pointer");
        checked(p, off);
        return p + off;
    };
    auto vtable = [&](std::size_t p) {
        Reader r(b, p);
        auto off = r.i32();
        auto target = static_cast<std::int64_t>(p) - off;
        require(target >= 0, "BGFB vtable pointer");
        checked(target, 4);
        return std::size_t(target);
    };
    std::set<std::pair<std::string, std::size_t>> active;
    std::function<Json(const std::string &, std::size_t, unsigned)> read =
        [&](const std::string &typ, std::size_t pos, unsigned depth) -> Json {
        require(depth <= 80, "BGFB depth");
        if (scalars.count(typ)) {
            Reader r(b, pos);
            if (typ == "bool")
                return bool(r.u8());
            if (typ == "byte")
                return r.get<std::int8_t>();
            return r.number(scalars.at(typ).first);
        }
        if (typ == "string") {
            auto p = indirect(pos);
            auto n = u32(p);
            checked(p + 4, n + 1);
            require(b[p + 4 + n] == 0, "BGFB string termination");
            return utf8(slice(b, p + 4, n));
        }
        if (!typ.empty() && typ[0] == '[') {
            auto it = typ.substr(1, typ.size() - 2);
            auto start = indirect(pos);
            auto count = u32(start);
            start += 4;
            bool scalar = scalars.count(it), vector = !it.empty() && it[0] == '[',
                 st = !scalar && !vector && types.at(it).at("kind") == "struct";
            std::size_t size = scalar ? scalars.at(it).second : st ? layout(it).size : 4;
            require(count <= b.size() / size, "BGFB array count");
            checked(start, size * count);
            Json a = Json::array();
            for (unsigned i = 0; i < count; ++i)
                a.push_back(
                    read(it, scalar || st || vector ? start + i * size : indirect(start + i * size),
                         depth + 1));
            return a;
        }
        auto &def = types.at(typ);
        if (def["kind"] == "struct") {
            auto l = layout(typ);
            checked(pos, l.size);
            Json v = {{"_type", typ}};
            for (auto &f : l.fields)
                v[std::get<0>(f)] = read(std::get<1>(f), pos + std::get<2>(f), depth + 1);
            return v;
        }
        require(active.insert({typ, pos}).second, "BGFB table cycle");
        auto vt = vtable(pos);
        auto size = u16(vt), os = u16(vt + 2);
        require(size >= 4 && size % 2 == 0 && os >= 4, "BGFB vtable size");
        checked(vt, size);
        checked(pos, os);
        Json fields = Json::array();
        for (auto &f : def["fields"]) {
            auto t = f["type"].get<std::string>();
            if (types.contains(t) && types[t]["kind"] == "union")
                fields.push_back(
                    {{"name", f["name"].get<std::string>() + "Type"}, {"type", "ubyte"}});
            fields.push_back(f);
        }
        Json result = {{"_type", typ}, {"_present_fields", Json::array()}};
        for (std::size_t i = 0; i < fields.size(); ++i) {
            auto off = 4 + 2 * i < size ? u16(vt + 4 + 2 * i) : 0;
            auto name = fields[i]["name"].get<std::string>(),
                 t = fields[i]["type"].get<std::string>();
            if (!off) {
                result[name] = t == "bool" ? Json(false) : scalars.count(t) ? Json(0) : Json();
                continue;
            }
            require(off >= 4 && off < os, "BGFB field extent");
            const std::size_t width = scalars.count(t) ? scalars.at(t).second
                                      : types.contains(t) && types[t]["kind"] == "struct"
                                          ? layout(t).size
                                          : 4;
            require(width <= std::size_t(os - off), "BGFB field crosses table extent");
            result["_present_fields"].push_back(name);
            if (types.contains(t) && types[t]["kind"] == "union") {
                auto tag = result[name + "Type"].get<unsigned>();
                auto key = std::to_string(tag);
                require(types[t]["fields"].contains(key), "BGFB union type");
                result[name] = read(types[t]["fields"][key].get<std::string>(), indirect(pos + off),
                                    depth + 1);
            } else if (types.contains(t) && types[t]["kind"] == "table")
                result[name] = read(t, indirect(pos + off), depth + 1);
            else
                result[name] = read(t, pos + off, depth + 1);
        }
        result["_unknown_field_slots"] = Json::array();
        for (std::size_t i = fields.size(); i < (size - 4) / 2u; ++i) {
            auto off = u16(vt + 4 + 2 * i);
            if (off)
                result["_unknown_field_slots"].push_back({{"slot", i}, {"offset", off}});
        }
        if (typ == "BsplineCurve") {
            try {
                const auto curve = BsplineCurve::from_bgfb(result);
                result["_spline"] = {
                    {"status", "valid"},
                    {"pole_coordinates", curve.rational() ? "weighted_xyz" : "cartesian_xyz"},
                    {"rational", curve.rational()},
                    {"knots_source", curve.source_knots().empty() ? "generated_uniform" : "stored"},
                    {"knot_domain", curve.knot_domain()},
                    {"periodic_pole_shift", curve.periodic_pole_shift()}};
            } catch (const std::exception &e) {
                result["_spline"] = {{"status", "invalid"}, {"error", e.what()}};
            }
        }
        if (typ == "BsplineSurface") {
            try {
                const auto surface = BsplineSurface::from_bgfb(result);
                auto direction = [](const BsplineDirection &axis) {
                    return Json{{"knots_source",
                                 axis.source_knots().empty() ? "generated_uniform" : "stored"},
                                {"knot_domain", axis.knot_domain()},
                                {"periodic_pole_shift", axis.periodic_pole_shift()}};
                };
                result["_spline"] = {
                    {"status", "valid"},
                    {"pole_coordinates", surface.rational() ? "weighted_xyz" : "cartesian_xyz"},
                    {"rational", surface.rational()},
                    {"pole_order", "u_fastest"},
                    {"u", direction(surface.u())},
                    {"v", direction(surface.v())},
                    {"outer_boundary_active", surface.outer_boundary_active()},
                    {"trim_region_evaluation", "not_evaluated"}};
            } catch (const std::exception &e) {
                result["_spline"] = {{"status", "invalid"}, {"error", e.what()}};
            }
        }
        active.erase({typ, pos});
        return result;
    };
    return read("VariantGeometry", indirect(8), 0);
}
} // namespace p3d
