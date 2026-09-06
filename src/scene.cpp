#include "geometry.hpp"
#include <atomic>
#include <thread>
namespace p3d {
static std::string scoped(const Json &model, const Json &id) {
    return model.dump() + ":" + id.dump();
}
static std::string source_key(const Json &row) {
    return row["stream"].dump() + "@" + row["offset"].dump();
}
static std::string block_name(const Json &n) {
    for (auto &link : n["links"]) {
        if (link["app"] != 0x56d2)
            continue;
        auto b = bytesof(link["payload"]);
        if (b.size() < 8)
            continue;
        Reader r(b);
        auto key = r.u32(), size = r.u32();
        if (key == 1)
            return native_string(r.take(size));
    }
    return "";
}
static void flatten(const Json &node, Json &out, const std::string &prefix = "",
                    const std::string &segment = "") {
    auto path = prefix + "/" + (segment.empty() ? node["name"].get<std::string>() : segment);
    if (node.contains("value"))
        out[path] = node["value"];
    if (!node.contains("children"))
        return;
    std::map<std::string, unsigned> count, seen;
    for (auto &c : node["children"])
        count[c["name"]]++;
    for (auto &c : node["children"]) {
        auto name = c["name"].get<std::string>();
        auto seg = count[name] > 1 ? name + "[" + std::to_string(seen[name]++) + "]" : name;
        flatten(c, out, path, seg);
    }
}
static Json color(std::uint64_t index, const Json &entries) {
    if (index == 0x7fffffff || index == 0xffffffff)
        return {{"index", index}, {"rgb", nullptr}, {"source", "inherited_color_requires_context"}};
    if (index >= 0x1000000)
        return {{"index", index}, {"rgb", nullptr}, {"source", "unverified_color_encoding"}};
    auto ordinal = static_cast<std::int64_t>(index >> 8) - 1;
    if (ordinal >= 0 && std::size_t(ordinal) < entries.size() && !entries[ordinal]["rgb"].is_null())
        return {{"index", index},
                {"rgb", entries[ordinal]["rgb"]},
                {"source", "extended_color"},
                {"extended_ordinal", ordinal}};
    return {{"index", index},
            {"rgb", default_palette().at("rgb").at(index & 255)},
            {"source", "palette"},
            {"palette_index", index & 255},
            {"unresolved_extended_ordinal", ordinal >= 0 ? Json(ordinal) : Json()}};
}
static Json style_color(const Json &style, const Json &entries) {
    if (style.contains("native_symbology_extension") &&
        style["native_symbology_extension"].contains("inheritance_flags"))
        return {{"rgb", nullptr}, {"source", "native_inheritance_rules_not_evaluated"}};
    if (style.contains("native_symbology_extension") &&
        style["native_symbology_extension"].contains("fill_style_block") &&
        style["native_symbology_extension"]["fill_style_block"].value("status", "") != "clear")
        return {{"rgb", nullptr}, {"source", "native_gradient_fill"}};
    if (style.contains("true_color_packed")) {
        auto packed = style["true_color_packed"].get<std::uint32_t>();
        return {{"packed", packed},
                {"rgb", {packed & 255u, (packed >> 8) & 255u, (packed >> 16) & 255u}},
                {"unassigned_high_byte", packed >> 24},
                {"source", "native_packed_color"}};
    }
    for (auto key : {"field_0001", "field_0002", "field_0004", "field_0008"})
        if (style.contains(key))
            return {{"rgb", nullptr}, {"source", "native_display_color_requires_context"}};
    return color(style.value("color_index", std::uint64_t(0)), entries);
}
NativeScene build_native_scene(const Document &doc, const Tessellation &policy, unsigned threads) {
    policy.segments(1);
    NativeScene scene;
    std::map<std::string, std::uint64_t> model_ids;
    Json model_map = Json::object();
    for (auto it = doc.index().begin(); it != doc.index().end(); ++it)
        if (it.key()[0] == '#' && it.key().find('%') == std::string::npos) {
            auto id = std::stoull(it.key().substr(1), nullptr, 16);
            model_ids[it.value()] = id;
            model_map[std::to_string(id)] = it.value();
        }
    auto canonical = [&](const char *key) { return doc.index().value(key, std::string()); };
    std::vector<std::shared_ptr<GeometryDefinition>> definitions;
    std::vector<std::function<Geometry()>> tasks;
    std::map<std::string, std::size_t> definition_lookup;
    auto define = [&](const std::string &key, const Json &source, std::function<Geometry()> task) {
        auto it = definition_lookup.find(key);
        if (it != definition_lookup.end())
            return it->second;
        auto ix = definitions.size();
        auto d = std::make_shared<GeometryDefinition>();
        d->source_key = key;
        d->source = source;
        definitions.push_back(d);
        tasks.push_back(std::move(task));
        definition_lookup[key] = ix;
        return ix;
    };
    std::map<std::string, std::vector<const Json *>> geometry_records, attribute_records,
        system_attributes;
    std::map<StreamPath, std::vector<const Json *>> system_records;
    std::vector<const Json *> live;
    for (auto &n : doc.native_records()) {
        auto s = n["stream"].get<StreamPath>();
        if (std::find(s.begin(), s.end(), canonical("P3D-SSYS")) != s.end())
            system_records[s].push_back(&n);
        if (s.size() == 4 && s[0] == canonical("P3D-SM") && s[2] == canonical("P3D-SMG") &&
            model_ids.count(s[1]))
            live.push_back(&n);
    }
    for (auto &g : doc.graphics_records()) {
        auto s = g["stream"].get<StreamPath>();
        if (std::find(s.begin(), s.end(), canonical("P3D-SSYSA")) != s.end())
            system_attributes[g["id"].dump()].push_back(&g);
        if (s.size() < 4 || s[0] != canonical("P3D-SM") || !model_ids.count(s[1]))
            continue;
        auto k = scoped(model_ids[s[1]], g["id"]);
        if (s[2] == canonical("P3D-SMCA"))
            geometry_records[k].push_back(&g);
        else if (s[2] == canonical("P3D-SMGA"))
            attribute_records[k].push_back(&g);
    }
    Json errors = Json::array(), library_errors = Json::array(), blocks = Json::array();
    std::map<std::string, std::vector<std::size_t>> block_lookup;
    std::vector<std::vector<const Json *>> block_children;
    for (auto &pair : system_records) {
        auto &rows = pair.second;
        for (std::size_t i = 0; i < rows.size(); ++i) {
            auto &n = *rows[i];
            if (n["element_type"] != 32)
                continue;
            try {
                auto b = bytesof(n["data"]);
                auto count = Reader(b, 108).u32();
                require(count <= rows.size() - i - 1, "block child record count");
                auto name = block_name(n);
                auto ix = blocks.size();
                blocks.push_back({{"key", source_key(n)},
                                  {"name", name},
                                  {"id", n["id"]},
                                  {"source", {{"stream", n["stream"]}, {"offset", n["offset"]}}},
                                  {"children", Json::array()}});
                block_children.emplace_back(rows.begin() + i + 1, rows.begin() + i + 1 + count);
                if (!name.empty())
                    block_lookup[name].push_back(ix);
                for (auto child : block_children.back())
                    blocks[ix]["children"].push_back({{"source_key", source_key(*child)},
                                                      {"id", child->at("id")},
                                                      {"native_type", child->at("element_type")}});
            } catch (const std::exception &e) {
                library_errors.push_back({{"block_id", n["id"]}, {"error", e.what()}});
            }
        }
    }
    std::map<std::string, std::vector<std::size_t>> geometry_defs;
    std::map<std::string, Json> terrain_defs;
    for (auto &pair : geometry_records) {
        for (auto gr : pair.second) {
            const auto &g = *gr;
            auto rk = source_key(g);
            std::vector<std::size_t> ids;
            if (g.contains("terrain")) {
                const auto &t = g["terrain"];
                auto ix = define(
                    rk + ":terrain",
                    {{"kind", "native_shared_terrain"},
                     {"record_key", rk},
                     {"geometry_id", g["id"]},
                     {"stream", g["stream"]},
                     {"offset", g["offset"]}},
                    [gr]() {
                        const auto &t = gr->at("terrain");
                        Geometry geo;
                        if (t["transform"]["matrix"] != Json(identity())) {
                            geo.unknown.push_back(
                                {{"reason",
                                  "nonidentity terrain definition transform needs verification"}});
                            return geo;
                        }
                        geo.vertices = t["vertices"].get<std::vector<Point3>>();
                        geo.faces = t["faces"].get<std::vector<Triangle>>();
                        geo.face_uvs.resize(geo.faces.size());
                        geo.face_source_polygons.resize(geo.faces.size());
                        geo.primitive_ranges.push_back({{"channel", "faces"},
                                                        {"start", 0},
                                                        {"count", geo.faces.size()},
                                                        {"style", Json::object()},
                                                        {"source", "terrain_neighbor_rings"}});
                        geo.notes.push_back("terrain faces reconstructed from source neighbor "
                                            "rings; source winding retained");
                        return geo;
                    });
                ids.push_back(ix);
                auto detail = t;
                for (auto k : {"vertices", "faces", "neighbor_rings", "adjacency_entries",
                               "vertex_metadata"})
                    detail.erase(k);
                terrain_defs[pair.first] = detail;
            }
            if (g.contains("terrain_decode_error"))
                errors.push_back({{"stream", g["stream"]},
                                  {"terrain_id", g["id"]},
                                  {"error", g["terrain_decode_error"]}});
            for (std::size_t ai = 0; ai < g["attributes"].size(); ++ai) {
                auto &a = g["attributes"][ai];
                if (a["group"] == 0 && a["key"] == 20031 && a["decoded"].contains("commands")) {
                    auto ap = &a;
                    ids.push_back(define(rk + ":attribute:" + std::to_string(ai),
                                         {{"kind", "native_shared_geometry"},
                                          {"record_key", rk},
                                          {"geometry_id", g["id"]},
                                          {"stream", g["stream"]},
                                          {"offset", g["offset"]},
                                          {"attribute_index", a["index"]}},
                                         [ap, &policy]() {
                                             return reconstruct(ap->at("decoded").at("commands"),
                                                                policy);
                                         }));
                }
            }
            geometry_defs[rk] = ids;
        }
    }
    auto native_definition = [&](const Json &n, bool in_block) {
        auto np = &n;
        auto rk = source_key(n) + (in_block ? ":block-child" : ":direct");
        return define(
            rk,
            {{"kind", in_block ? "native_block_child" : "native_direct_element"},
             {"record_key", source_key(n)},
             {"native_element_id", n["id"]},
             {"stream", n["stream"]},
             {"offset", n["offset"]}},
            [&, np, in_block]() {
                auto geo =
                    np->at("element_type") == 62 ? Geometry{} : reconstruct_native(*np, policy);
                if (in_block) {
                    auto found = system_attributes.find(np->at("id").dump());
                    if (found != system_attributes.end()) {
                        if (found->second.size() > 1)
                            geo.unknown.push_back(
                                {{"reason", "ambiguous block child graphics records"},
                                 {"record_count", found->second.size()}});
                        else
                            for (auto &a : found->second[0]->at("attributes"))
                                if (a["group"] == 0 && a["key"] == 20031 &&
                                    a["decoded"].contains("commands"))
                                    merge_geometry(geo,
                                                   reconstruct(a["decoded"]["commands"], policy),
                                                   identity());
                    }
                }
                return geo;
            });
    };
    std::function<void(const Json &, const Matrix4 &, std::vector<GeometryInstance> &, Json &,
                       std::vector<std::size_t>, Json)>
        block_reference = [&](const Json &n, const Matrix4 &parent,
                              std::vector<GeometryInstance> &instances, Json &meta,
                              std::vector<std::size_t> ancestors, Json chain) {
            try {
                auto name = block_name(n);
                auto candidates = block_lookup[name];
                Json ref = {{"source_key", source_key(n)},
                            {"name", name},
                            {"candidates", candidates},
                            {"status", candidates.size() == 1 ? "resolved"
                                       : candidates.empty()   ? "missing"
                                                              : "ambiguous"}};
                if (candidates.size() != 1) {
                    meta["unknown"].push_back(
                        {{"native_type", 62},
                         {"reason", candidates.empty() ? "missing block definition"
                                                       : "ambiguous block definition"},
                         {"name", name},
                         {"candidates", candidates}});
                    meta["block_references"].push_back(ref);
                    return;
                }
                auto ix = candidates[0];
                require(std::find(ancestors.begin(), ancestors.end(), ix) == ancestors.end(),
                        "cyclic block reference");
                ancestors.push_back(ix);
                auto b = bytesof(n["data"]);
                Reader r(b, 164);
                auto m = identity();
                for (unsigned i = 0; i < 3; ++i)
                    for (unsigned j = 0; j < 3; ++j)
                        m[i][j] = r.f64();
                for (unsigned i = 0; i < 3; ++i)
                    m[i][3] = r.f64();
                auto world = multiply(parent, m);
                ref["matrix"] = m;
                ref["definition_key"] = blocks[ix]["key"];
                meta["block_references"].push_back(ref);
                chain.push_back(ref);
                for (auto child : block_children[ix]) {
                    if (child->at("element_type") == 62) {
                        block_reference(*child, world, instances, meta, ancestors, chain);
                        // Reference-owned display commands use the parent block frame.
                        auto found = system_attributes.find(child->at("id").dump());
                        bool has_commands = false;
                        if (found != system_attributes.end())
                            for (auto record : found->second)
                                for (auto &attribute : record->at("attributes"))
                                    has_commands |=
                                        attribute["group"] == 0 && attribute["key"] == 20031;
                        if (has_commands) {
                            GeometryInstance display;
                            display.definition = native_definition(*child, true);
                            display.matrix = world;
                            display.source = {{"block_chain", chain},
                                              {"reference_display_source", source_key(*child)}};
                            instances.push_back(std::move(display));
                        }
                    } else {
                        GeometryInstance instance;
                        instance.definition = native_definition(*child, true);
                        instance.matrix = world;
                        instance.source = {{"block_chain", chain}};
                        instance.apply_placement = true;
                        instances.push_back(instance);
                    }
                }
                meta["notes"].push_back("shared block expanded: " + name +
                                        "; native visibility rules require verification");
            } catch (const std::exception &e) {
                meta["unknown"].push_back({{"native_type", 62}, {"reason", e.what()}});
            }
        };
    for (std::size_t bi = 0; bi < block_children.size(); ++bi)
        for (std::size_t ci = 0; ci < block_children[bi].size(); ++ci) {
            auto child = block_children[bi][ci];
            auto &entry = blocks[bi]["children"][ci];
            if (child->at("element_type") != 62) {
                entry["geometry_definition"] = native_definition(*child, true);
            } else {
                try {
                    auto name = block_name(*child);
                    auto candidates = block_lookup[name];
                    auto raw = bytesof(child->at("data"));
                    Reader reader(raw, 164);
                    auto matrix = identity();
                    for (unsigned i = 0; i < 3; ++i)
                        for (unsigned j = 0; j < 3; ++j)
                            matrix[i][j] = reader.f64();
                    for (unsigned i = 0; i < 3; ++i)
                        matrix[i][3] = reader.f64();
                    entry["block_reference"] = {{"name", name},
                                                {"matrix", matrix},
                                                {"candidates", candidates},
                                                {"status", candidates.size() == 1 ? "resolved"
                                                           : candidates.empty()   ? "missing"
                                                                                  : "ambiguous"}};
                } catch (const std::exception &error) {
                    entry["decode_error"] = error.what();
                }
            }
        }
    std::map<std::string, Json> binding_groups;
    for (auto &b : doc.bindings()) {
        auto k = scoped(b["model_id"], b["element_id"]);
        if (!binding_groups.count(k))
            binding_groups[k] = Json::array();
        binding_groups[k].push_back(b);
    }
    auto graph = doc.object_graph();
    for (auto np : live) {
        auto &n = *np;
        auto s = n["stream"].get<StreamPath>();
        auto model = model_ids[s[1]], id = n["id"].get<std::uint64_t>();
        auto ek = scoped(model, id);
        SceneElement element;
        auto &meta = element.metadata;
        const auto native_data = bytesof(n["data"]);
        meta = {
            {"model_id", model},
            {"element_id", id},
            {"composite_id", (model << 32) | id},
            {"native_type", n["element_type"]},
            {"native_flags", n["element_flags"]},
            {"display_state", native_display_state(n["element_type"].get<unsigned>(), native_data)},
            {"class", nullptr},
            {"document_element_key", ek},
            {"native_source_key", source_key(n)},
            {"native_header_hex", hex(native_data)},
            {"unknown", Json::array()},
            {"notes", Json::array()},
            {"block_references", Json::array()},
            {"instance_commands", Json::array()}};
        auto matrix = identity();
        Json instance_style = Json::object(), terrain_children = Json::array(),
             terrain_attrs = Json::array();
        std::string terrain_error;
        auto attr = attribute_records.find(ek);
        if (attr != attribute_records.end()) {
            if (attr->second.size() != 1) {
                meta["unknown"].push_back({{"reason", "ambiguous element graphics records"},
                                           {"record_count", attr->second.size()}});
            } else {
                auto gr = attr->second[0];
                for (std::size_t ai = 0; ai < gr->at("attributes").size(); ++ai) {
                    auto &a = gr->at("attributes")[ai];
                    auto &d = a["decoded"];
                    if ((a["group"] == 4 || a["group"] == 2) && a["key"] == 10001) {
                        if (!meta.contains("advanced_material_assignments"))
                            meta["advanced_material_assignments"] = Json::array();
                        meta["advanced_material_assignments"].push_back(
                            {{"source_key", source_key(*gr)},
                             {"group", a["group"]},
                             {"key", a["key"]},
                             {"attribute_ordinal", ai},
                             {"attribute_index", a["index"]},
                             {"attribute_offset", a["offset"]},
                             {"payload", a["payload"]},
                             {"decoded", d}});
                    }
                    if (a["key"] == 23223) {
                        terrain_attrs.push_back({{"group", a["group"]},
                                                 {"index", a["index"]},
                                                 {"payload_hex", hex(bytesof(a["payload"]))}});
                        if (a["group"] == 300 && d.contains("geometry_id"))
                            terrain_children.push_back(d["geometry_id"]);
                        if (a["group"] == 313 &&
                            (!d.contains("matrix") || d["matrix"] != Json(identity())))
                            terrain_error =
                                "nonidentity terrain instance transform needs verification";
                    }
                    if (a["group"] == 1 && a["key"] == 10001 && d.contains("text"))
                        meta["class"] = d["text"];
                    if (a["group"] == 0 && a["key"] == 20031 && d.contains("commands")) {
                        bool direct = false;
                        for (auto &cmd : d["commands"]) {
                            auto op = cmd["op"].get<unsigned>();
                            auto body = bytesof(cmd["body"]);
                            meta["instance_commands"].push_back({{"opcode", op},
                                                                 {"offset", cmd["offset"]},
                                                                 {"body_hex", hex(body)},
                                                                 {"decoded", cmd["decoded"]}});
                            if (op == 34)
                                matrix = multiply(matrix, instance_transform(body));
                            if (op == 28) {
                                auto style = decode_symbology(body);
                                apply_symbology(instance_style, style);
                            }
                            if (op == 40)
                                apply_symbology_extension(instance_style, cmd["decoded"]);
                            if (op != 28 && op != 29 && op != 34 && op != 40)
                                direct = true;
                        }
                        if (direct) {
                            auto ap = &a;
                            auto ix = define(
                                source_key(*gr) + ":direct-attribute:" + std::to_string(ai),
                                {{"kind", "native_direct_commands"},
                                 {"record_key", source_key(*gr)},
                                 {"element_key", ek},
                                 {"attribute_index", a["index"]}},
                                [ap, &policy]() {
                                    return reconstruct(ap->at("decoded").at("commands"), policy);
                                });
                            GeometryInstance inst;
                            inst.definition = ix;
                            inst.matrix = identity();
                            inst.apply_placement = false;
                            element.instances.push_back(inst);
                        }
                    }
                }
            }
        }
        if (n["element_type"] == 62)
            block_reference(n, identity(), element.instances, meta, {}, Json::array());
        else if (n["element_type"] != 97) {
            GeometryInstance inst;
            inst.definition = native_definition(n, false);
            inst.matrix = identity();
            inst.apply_placement = false;
            element.instances.push_back(inst);
        }
        auto geometry_ids = n["children"];
        for (auto &child : terrain_children)
            if (std::find(geometry_ids.begin(), geometry_ids.end(), child) == geometry_ids.end())
                geometry_ids.push_back(child);
        for (auto &child : geometry_ids) {
            auto k = scoped(model, child);
            if (!terrain_error.empty() &&
                std::find(terrain_children.begin(), terrain_children.end(), child) !=
                    terrain_children.end()) {
                meta["unknown"].push_back(
                    {{"geometry_id", child}, {"terrain_transform_error", terrain_error}});
                continue;
            }
            auto found = geometry_records.find(k);
            if (found == geometry_records.end() || found->second.empty()) {
                meta["unknown"].push_back({{"missing_geometry", child}});
                continue;
            }
            if (found->second.size() > 1) {
                meta["unknown"].push_back(
                    {{"ambiguous_geometry", child}, {"record_count", found->second.size()}});
                continue;
            }
            auto gr = found->second[0];
            for (auto ix : geometry_defs[source_key(*gr)]) {
                GeometryInstance inst;
                inst.definition = ix;
                inst.matrix = matrix;
                inst.geometry_id = child;
                inst.source = {{"native_child_reference", child}, {"record_key", source_key(*gr)}};
                element.instances.push_back(inst);
            }
        }
        for (auto &inst : element.instances)
            inst.style = instance_style;
        meta["geometry_ids"] = geometry_ids;
        meta["instance_matrix"] = matrix;
        meta["instance_style"] = instance_style;
        auto candidates = binding_groups.count(ek) ? binding_groups[ek] : Json::array();
        Json binding = nullptr;
        bool identical = !candidates.empty();
        for (auto &b : candidates)
            if (b != candidates[0])
                identical = false;
        if (identical)
            binding = candidates[0];
        meta["binding"] = binding;
        meta["binding_candidates"] = candidates;
        meta["binding_status"] =
            binding.is_null() ? (candidates.empty() ? "unbound" : "ambiguous") : "missing";
        meta["object_key"] = nullptr;
        if (!binding.is_null()) {
            auto ok = scoped(binding["schema_id"], binding["object_id"]);
            meta["object_key"] = ok;
            auto found = graph["object_index"].find(ok);
            if (found != graph["object_index"].end())
                meta["binding_status"] = found->at("status");
        }
        if (!terrain_attrs.empty()) {
            meta["terrain_attributes"] = terrain_attrs;
            meta["terrain_definitions"] = Json::array();
            for (auto child : terrain_children) {
                auto k = scoped(model, child);
                if (terrain_defs.count(k)) {
                    auto detail = terrain_defs[k];
                    detail["geometry_id"] = child;
                    meta["terrain_definitions"].push_back(detail);
                }
            }
        }
        scene.elements.push_back(std::move(element));
    }
    std::atomic<std::size_t> work{0};
    auto worker = [&]() {
        while (true) {
            auto ix = work.fetch_add(1);
            if (ix >= tasks.size())
                break;
            try {
                definitions[ix]->geometry = tasks[ix]();
            } catch (const std::exception &e) {
                definitions[ix]->geometry.unknown.push_back({{"reason", e.what()}});
            }
        }
    };
    auto nt = std::min<unsigned>(std::min(64u, std::max(1u, threads)),
                                 std::max(std::size_t(1), tasks.size()));
    std::vector<std::thread> pool;
    for (unsigned i = 1; i < nt; ++i) {
        try {
            pool.emplace_back(worker);
        } catch (const std::system_error &) {
            break;
        }
    }
    worker();
    for (auto &t : pool)
        t.join();
    for (auto &d : definitions)
        scene.definitions.push_back(d);
    Json mats = doc.materials();
    for (auto &m : mats["definitions"]) {
        m["payload_hex"] = hex(bytesof(m["payload"]));
        m.erase("payload");
    }
    Json colors = doc.color_tables();
    for (auto &c : colors)
        if (c.contains("data")) {
            c["decoded_data_hex"] = hex(bytesof(c["data"]));
            c.erase("data");
        }
    Json schemas = doc.schemas();
    for (auto &s : schemas) {
        s["header_hex"] = hex(bytesof(s["header"]));
        s.erase("header");
    }
    Json relations = doc.relationships(), related = Json::object();
    for (auto &rel : relations)
        for (std::string side : {"source", "target"}) {
            auto k = scoped(rel[side + "_class_id"], rel[side + "_object_id"]);
            auto found = graph["object_index"].find(k);
            std::string status = found == graph["object_index"].end()
                                     ? "missing"
                                     : found->at("status").get<std::string>();
            rel[side + "_status"] = status;
            rel[side + "_resolved"] = status == "resolved" || status == "identical_copies";
            if (status == "resolved" || status == "identical_copies") {
                auto rk = (*found)["records"][0].get<std::string>();
                related[k] =
                    graph["objects"][graph["record_index"][rk].get<std::size_t>()]["property_tree"];
            }
        }
    scene.metadata = {
        {"models", model_map},
        {"model_info", doc.models()},
        {"materials", std::move(mats)},
        {"color_tables", std::move(colors)},
        {"relationships", std::move(relations)},
        {"relationship_nodes", std::move(related)},
        {"errors", errors},
        {"library_errors", library_errors},
        {"coordinate_units", "source-native; geometry scale must be verified per file"},

        {"schema_definitions", std::move(schemas)},
        {"native_block_definitions", blocks},
        {"winding_policy", "Source front-face orientation preserved when affine transforms are "
                           "baked, including mirror transforms; UV corners follow vertex order. "
                           "Absolute source front-face conventions are retained."},
        {"bindings", doc.bindings().size()},
        {"geometry_definitions", geometry_records.size()},
        {"reuse_policy",
         "Source identities and references only. No content-based geometry deduplication."}};
    scene.metadata["document_graph"] = std::move(graph);
    scene.metadata["schema_objects"] = scene.metadata["document_graph"]["object_index"].size();
    return scene;
}
Json NativeScene::expanded() const {
    Json result = metadata, output = Json::array();
    Json extended = metadata["color_tables"].empty()
                        ? Json::array()
                        : metadata["color_tables"].back().value("color_entries", Json::array());
    std::map<std::string, std::vector<const Json *>> mats;
    for (auto &m : metadata["materials"]["definitions"])
        mats[m["scope"].get<std::string>() + ":" + m["id"].dump()].push_back(&m);
    auto &graph = metadata["document_graph"];
    for (auto &element : elements) {
        auto item = element.metadata;
        Geometry geo;
        for (auto &u : item["unknown"])
            geo.unknown.push_back(u);
        for (auto &n : item["notes"])
            geo.notes.push_back(n);
        for (auto &instance : element.instances) {
            auto &source = definitions.at(instance.definition)->geometry;
            Geometry placed;
            if (instance.apply_placement)
                merge_geometry(placed, source, instance.matrix);
            else
                placed = source;
            std::map<std::string, std::size_t> offsets = {{"faces", geo.faces.size()},
                                                          {"lines", geo.lines.size()},
                                                          {"texts", geo.texts.size()}};
            for (auto range : placed.primitive_ranges) {
                auto style = instance.style;
                style.update(range["style"]);
                Json appearance = {
                    {"color", style_color(style, extended)},
                    {"style_status",
                     range.value("style_status", std::string("decoded_command_style"))}};
                auto mid = style.value("material_id", Json());
                if (!mid.is_null() && mid != 0) {
                    auto key = "model:" + item["model_id"].dump() + ":" + mid.dump();
                    auto candidates = mats[key];
                    if (candidates.empty())
                        candidates = mats["global:" + mid.dump()];
                    appearance["material_id"] = mid;
                    appearance["material_resolved"] = candidates.size() == 1;
                    if (candidates.size() == 1) {
                        appearance["material_scope"] = candidates[0]->at("scope");
                        appearance["material_rgb"] = candidates[0]->value("base_color_rgb", Json());
                    }
                }
                range["start"] = range["start"].get<std::size_t>() +
                                 offsets.at(range["channel"].get<std::string>());
                range["geometry_id"] = instance.geometry_id;
                range["instance_style"] = instance.style;
                range["appearance"] = appearance;
                geo.primitive_ranges.push_back(range);
            }
            auto vbase = geo.vertices.size();
            geo.vertices.insert(geo.vertices.end(), placed.vertices.begin(), placed.vertices.end());
            for (auto f : placed.faces) {
                for (auto &v : f)
                    v += std::uint32_t(vbase);
                geo.faces.push_back(f);
            }
            geo.face_uvs.insert(geo.face_uvs.end(), placed.face_uvs.begin(), placed.face_uvs.end());
            geo.face_source_polygons.insert(geo.face_source_polygons.end(),
                                            placed.face_source_polygons.begin(),
                                            placed.face_source_polygons.end());
            geo.lines.insert(geo.lines.end(), placed.lines.begin(), placed.lines.end());
            for (auto &t : placed.texts)
                geo.texts.push_back(t);
            for (auto &u : placed.unknown)
                geo.unknown.push_back(u);
            for (auto &n : placed.notes)
                geo.notes.push_back(n);
        }
        item.update(geometry_json(geo));
        Json root = nullptr, props = Json::object();
        auto status = item["binding_status"].get<std::string>();
        if ((status == "resolved" || status == "identical_copies") &&
            item["object_key"].is_string()) {
            auto ok = item["object_key"].get<std::string>();
            auto rk = graph["object_index"][ok]["records"][0].get<std::string>();
            root = graph["objects"][graph["record_index"][rk].get<std::size_t>()]["property_tree"];
            flatten(root, props);
            if (item["class"].is_null())
                item["class"] = root["name"];
        }
        item["property_tree"] = root;
        item["properties"] = props;
        item["guid"] = nullptr;
        if (!root.is_null()) {
            bool found_guid = false;
            std::function<void(const Json &)> find_guid = [&](const Json &n) {
                if (found_guid)
                    return;
                if (n["name"] == "Guid" && n.contains("value")) {
                    item["guid"] = n["value"];
                    found_guid = true;
                    return;
                }
                if (n.contains("children"))
                    for (auto &c : n["children"])
                        find_guid(c);
            };
            find_guid(root);
        }
        if (!geo.vertices.empty()) {
            Point3 lo = geo.vertices[0], hi = lo;
            for (auto p : geo.vertices)
                for (unsigned j = 0; j < 3; ++j) {
                    lo[j] = std::min(lo[j], p[j]);
                    hi[j] = std::max(hi[j], p[j]);
                }
            item["bounds"] = {lo, hi};
        }
        output.push_back(std::move(item));
    }
    result["elements"] = std::move(output);
    return result;
}
Json NativeScene::summary() const {
    std::vector<std::size_t> uses(definitions.size(), 0);
    std::size_t instances = 0, expanded_v = 0, expanded_f = 0, unique_v = 0, unique_f = 0,
                unknown = 0;
    for (auto &e : elements) {
        unknown += e.metadata["unknown"].size();
        for (auto &i : e.instances) {
            uses.at(i.definition)++;
            instances++;
            auto &g = definitions[i.definition]->geometry;
            expanded_v += g.vertices.size();
            expanded_f += g.faces.size();
            unknown += g.unknown.size();
        }
    }
    std::size_t reused = 0, used = 0;
    for (std::size_t i = 0; i < definitions.size(); ++i) {
        if (uses[i]) {
            used++;
            unique_v += definitions[i]->geometry.vertices.size();
            unique_f += definitions[i]->geometry.faces.size();
        }
        if (uses[i] > 1)
            reused++;
    }
    return {{"elements", elements.size()},      {"definitions", definitions.size()},
            {"referenced_definitions", used},   {"reused_definitions", reused},
            {"instances", instances},           {"stored_referenced_vertices", unique_v},
            {"expanded_vertices", expanded_v},  {"stored_referenced_triangles", unique_f},
            {"expanded_triangles", expanded_f}, {"unknown", unknown}};
}
Json build_scene(const Document &doc, unsigned segments) {
    Tessellation p;
    p.full_circle_segments = segments;
    return doc.native_scene(p).expanded();
}
void NativeScene::for_each_primitive(
    const std::function<void(const PrimitiveView &)> &callback) const {
    std::map<std::string, std::vector<std::size_t>> materials;
    const auto &defs = metadata.at("materials").at("definitions");
    for (std::size_t i = 0; i < defs.size(); ++i)
        materials[defs[i]["scope"].get<std::string>() + ":" + defs[i]["id"].dump()].push_back(i);
    auto &colors = metadata.at("color_tables");
    Json extended =
        colors.empty() ? Json::array() : colors.back().value("color_entries", Json::array());
    for (std::size_t ei = 0; ei < elements.size(); ++ei) {
        auto &element = elements[ei];
        for (std::size_t ii = 0; ii < element.instances.size(); ++ii) {
            auto &inst = element.instances[ii];
            auto &geo = definitions.at(inst.definition)->geometry;
            for (auto &range : geo.primitive_ranges) {
                PrimitiveView v;
                v.element_index = ei;
                v.instance_index = ii;
                v.definition_index = inst.definition;
                v.geometry = &geo;
                v.source_range = &range;
                v.matrix = inst.matrix;
                auto channel = range["channel"].get<std::string>();
                auto start = range["start"].get<std::size_t>(),
                     count = range["count"].get<std::size_t>();
                auto size = channel == "faces"   ? geo.faces.size()
                            : channel == "lines" ? geo.lines.size()
                            : channel == "texts" ? geo.texts.size()
                                                 : 0;
                require(start <= size && count <= size - start, "primitive range extent");
                v.winding_reversed =
                    inst.apply_placement && determinant(inst.matrix) < 0 && channel == "faces";
                v.style = inst.style;
                v.style.update(range["style"]);
                v.appearance = {
                    {"color", style_color(v.style, extended)},
                    {"style_status",
                     range.value("style_status", std::string("decoded_command_style"))}};
                auto mid = v.style.value("material_id", Json());
                bool assigned = !mid.is_null() && mid != 0;
                if (assigned) {
                    v.appearance["material_id"] = mid;
                    auto k = "model:" + element.metadata["model_id"].dump() + ":" + mid.dump();
                    v.material_candidates = materials[k];
                    if (v.material_candidates.empty())
                        v.material_candidates = materials["global:" + mid.dump()];
                }
                v.material_status = !assigned                           ? "unassigned"
                                    : v.material_candidates.empty()     ? "missing"
                                    : v.material_candidates.size() == 1 ? "resolved"
                                                                        : "ambiguous";
                if (assigned)
                    v.appearance["material_resolved"] = v.material_candidates.size() == 1;
                if (v.material_candidates.size() == 1) {
                    auto &m = defs[v.material_candidates[0]];
                    v.appearance["material_scope"] = m["scope"];
                    v.appearance["material_rgb"] = m.value("base_color_rgb", Json());
                }
                v.topology = channel == "faces"              ? "triangles"
                             : channel == "texts"            ? "annotations"
                             : range.value("opcode", 0) == 3 ? "points"
                                                             : "polylines";
                v.uv_status = "not_applicable";
                if (channel == "faces") {
                    std::size_t present = 0;
                    for (auto i = start; i < start + count; ++i)
                        present += geo.face_uvs.at(i).has_value();
                    v.uv_status = present == count ? "explicit_source"
                                  : present == 0   ? "absent"
                                                   : "partly_explicit";
                }
                callback(v);
            }
        }
    }
}
} // namespace p3d
