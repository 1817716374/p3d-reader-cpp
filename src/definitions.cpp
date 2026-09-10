#include "internal.hpp"
namespace p3d {
Json material_settings(const Json &tree) {
    auto fields = [](const Json &attrs) {
        Json o = Json::object();
        for (auto i = attrs.begin(); i != attrs.end(); ++i)
            try {
                auto s = i.value().get<std::string>();
                char *tail = nullptr;
                double v = std::strtod(s.c_str(), &tail);
                require(tail != s.c_str() &&
                            s.find_first_not_of(" \t\r\n", std::size_t(tail - s.c_str())) ==
                                std::string::npos &&
                            std::isfinite(v),
                        "material number");
                o[i.key()] = v;
            } catch (...) {
            }
        return o;
    };
    Json maps = Json::array();
    std::function<void(const Json &, const std::string &, unsigned)> walk =
        [&](const Json &n, const std::string &path, unsigned depth) {
            const auto tag = n["tag"].get<std::string>();
            if (tag.size() == 3 && (tag[0] == 'M' || tag[0] == 'm') &&
                (tag[1] == 'A' || tag[1] == 'a') && (tag[2] == 'P' || tag[2] == 'p')) {
                auto &a = n["attributes"];
                auto numeric = fields(a);
                Json layers = Json::array();
                for (std::size_t i = 0; i < n["children"].size(); ++i) {
                    const auto &child = n["children"][i];
                    // The native layer reader examines attributes, not the tag.
                    // Retain source order; this is not a compositing expression.
                    if (!child["attributes"].contains("LayerType"))
                        continue;
                    layers.push_back({{"child_index", i},
                                      {"xml_path", path + "/" + child["tag"].get<std::string>() +
                                                       "[" + std::to_string(i) + "]"},
                                      {"source_parameters", child["attributes"]},
                                      {"semantics", material_layer_semantics(child["attributes"])},
                                      {"procedures", material_procedure_nodes(child)},
                                      {"replicators", material_replicator_nodes(child)}});
                }
                maps.push_back(
                    {{"index", maps.size()},
                     {"xml_path", path},
                     {"native_table_member", depth == 1},
                     {"source_parameters", a},
                     {"numeric_parameters", numeric},
                     {"semantics", material_map_semantics(a)},
                     {"procedures", material_procedure_nodes(n)},
                     {"replicators", material_replicator_nodes(n)},
                     {"texture_layers",
                      {{"entries", layers},
                       {"order", "source_child_order"},
                       {"activation_status", "not_evaluated"},
                       {"composition_status", "not_evaluated"}}},
                     {"filename", a.value("Filename", Json())},
                     {"source_type", a.value("Type", Json())},
                     {"source_pattern_off", numeric.value("pattern_off", Json())},
                     {"source_projection_matrix_on",
                      numeric.value("origin_uv_pro_matrix_on", Json())},
                     {"uv_evaluation_status", "native_mapping_semantics_not_evaluated"}});
            }
            for (std::size_t i = 0; i < n["children"].size(); ++i) {
                auto &c = n["children"][i];
                walk(c, path + "/" + c["tag"].get<std::string>() + "[" + std::to_string(i) + "]",
                     depth + 1);
            }
        };
    walk(tree, "/" + tree["tag"].get<std::string>(), 0);
    auto reader_profile = material_reader_paths(tree, maps);
    return {{"source_parameters", tree["attributes"]},
            {"numeric_parameters", fields(tree["attributes"])},
            {"semantics", material_parameter_semantics(tree["attributes"])},
            {"reader_profile", reader_profile},
            {"maps", maps},
            {"map_bindings", material_map_bindings(maps)},
            {"shader_policy", "Native parameter flags and map roles; no conversion to another "
                              "renderer or color space."}};
}
Json material_texture_references(const Json &tree, const Json &settings) {
    Json out = Json::array();
    std::function<void(const Json &, const std::string &)> walk = [&](const Json &n,
                                                                      const std::string &path) {
        if (n["attributes"].contains("Filename"))
            out.push_back({{"xml_path", path},
                           {"filename", n["attributes"]["Filename"]},
                           {"parameters", n["attributes"]}});
        for (std::size_t i = 0; i < n["children"].size(); ++i) {
            const auto &child = n["children"][i];
            walk(child,
                 path + "/" + child["tag"].get<std::string>() + "[" + std::to_string(i) + "]");
        }
    };
    walk(tree, "/" + tree["tag"].get<std::string>());
    // Preserve the legacy Filename inventory first. Additional references carry
    // explicit ownership; their presence does not assert renderer activation.
    for (const auto &map : settings["maps"]) {
        auto append = [&](const Json &owner, const Json &child_index) {
            const auto &sem = owner["semantics"];
            if (!sem.contains("additional_texture_references"))
                return;
            for (const auto &ref : sem["additional_texture_references"]["entries"])
                out.push_back({{"xml_path", owner["xml_path"]},
                               {"filename", ref["value"]},
                               {"parameters", owner["source_parameters"]},
                               {"source_attribute", "M556"},
                               {"reference_kind", "additional_texture"},
                               {"map_index", map["index"]},
                               {"layer_child_index", child_index},
                               {"append_index", ref["append_index"]},
                               {"source_fragment", ref["source_fragment"]},
                               {"activation_status", "not_evaluated"}});
        };
        append(map, nullptr);
        for (const auto &layer : map["texture_layers"]["entries"]) {
            const auto &type = layer["semantics"]["type"];
            if (type.value("argument_role", Json()) == "texture_reference")
                out.push_back({{"xml_path", layer["xml_path"]},
                               {"filename", type["argument"]},
                               {"parameters", layer["source_parameters"]},
                               {"source_attribute", "LayerType"},
                               {"reference_kind", "layer_primary"},
                               {"map_index", map["index"]},
                               {"layer_child_index", layer["child_index"]},
                               {"activation_status", "not_evaluated"}});
            append(layer, layer["child_index"]);
        }
    }
    return out;
}
Json read_materials(const Document &doc) {
    Json definitions = Json::array(), errors = Json::array();
    std::map<std::string, Json> names;
    auto key = [](const StreamPath &p, const Json &id) { return Json(p).dump() + ":" + id.dump(); };
    for (auto &n : doc.native_records()) {
        if (n["element_type"] != 49)
            continue;
        auto b = bytesof(n["data"]);
        if (b.size() < 20 || Reader(b, 16).u32() != 18)
            continue;
        Json strings = Json::array();
        for (auto &l : n["links"]) {
            auto p = bytesof(l["payload"]);
            if (l["app"] != 0x56d2 || p.size() < 8)
                continue;
            Reader r(p);
            auto k = r.u32(), len = r.u32();
            if (len <= r.left())
                try {
                    strings.push_back({{"key", k}, {"text", native_string(r.take(len))}});
                } catch (const std::exception &e) {
                    errors.push_back({{"stream", n["stream"]}, {"error", e.what()}});
                }
        }
        auto s = n["stream"].get<StreamPath>();
        s.resize(2);
        names[key(s, n["id"])] = strings;
    }
    std::map<std::string, std::string> models;
    for (auto e = doc.index().begin(); e != doc.index().end(); ++e)
        if (e.key()[0] == '#' && e.key().find('%') == std::string::npos)
            models[e.value()] = std::to_string(std::stoull(e.key().substr(1), nullptr, 16));
    for (auto &g : doc.graphics_records())
        for (auto &a : g["attributes"])
            if (a["group"] == 0 && a["key"] == 20014) {
                auto s = g["stream"].get<StreamPath>();
                require(s.size() >= 2, "material stream scope");
                bool global = std::find(s.begin(), s.end(),
                                        doc.index().value("P3D-SSYSA", std::string())) != s.end();
                std::string scope =
                    global ? "global" : "model:" + (models.count(s[1]) ? models[s[1]] : s[1]);
                auto native = s;
                native.resize(2);
                if (global)
                    native[1] = doc.index().value("P3D-SSYS", std::string());
                auto nk = key(native, g["id"]);
                Json item = {{"id", g["id"]},
                             {"scope", scope},
                             {"stream", s},
                             {"record_offset", g["offset"]},
                             {"attribute_index", a["index"]},
                             {"payload", a["payload"]},
                             {"native_strings", names.count(nk) ? names[nk] : Json::array()}};
                try {
                    auto &dec = a["decoded"];
                    const auto encoding = dec.value("encoding", std::string());
                    require(encoding == "compressed_utf16_xml" ||
                                encoding == "uncompressed_utf16_xml",
                            "material XML envelope");
                    auto &tree = dec["tree"];
                    require(tree["tag"] == "Material", "material XML root");
                    auto settings = material_settings(tree);
                    auto textures = material_texture_references(tree, settings);
                    item.update({{"version", dec["version"]},
                                 {"xml_byte_count", dec["decoded_bytes"]},
                                 {"xml", dec["xml"]},
                                 {"tree", tree},
                                 {"texture_references", textures},
                                 {"settings", settings}});
                    auto &at = tree["attributes"];
                    if (at.contains("color.r") && at.contains("color.g") && at.contains("color.b"))
                        item["base_color_rgb"] = {std::stod(at["color.r"].get<std::string>()),
                                                  std::stod(at["color.g"].get<std::string>()),
                                                  std::stod(at["color.b"].get<std::string>())};
                } catch (const std::exception &e) {
                    item["decode_error"] = e.what();
                    errors.push_back({{"stream", s}, {"id", g["id"]}, {"error", e.what()}});
                }
                definitions.push_back(item);
            }
    return {
        {"definitions", definitions},
        {"errors", errors},
        {"assignment_rules",
         {{"applies_to", "native_graphics_rebuild"},
          {"part_lookup_order",
           {"advanced_part_name", "legacy_part_name", "element_material",
            "graphics_entry_material"}},
          {"element_lookup_order", {"native_element_material", "advanced_element_name"}},
          {"part_index_basis", "zero_based_graphics_entry_order"},
          {"unresolved_name_policy", "continue_to_next_layer"},
          {"name_comparison", "native_case_insensitive_locale_dependent"},
          {"stored_command_styles", "already_materialized_do_not_reapply_by_triangle_index"}}},
        {"texture_policy", "opaque source references; resolution and loading belong to caller"}};
}
Json embedded_texture_records(const Json &graphics, const Json &materials) {
    auto identity = [](const Json &stream, const Json &id) {
        return stream.dump() + ":" + id.dump();
    };
    std::map<std::string, Json> candidates;
    const auto &definitions = materials.at("definitions");
    for (std::size_t i = 0; i < definitions.size(); ++i) {
        const auto &m = definitions[i];
        auto key = identity(m.at("stream"), m.at("id"));
        if (!candidates.count(key))
            candidates[key] = Json::array();
        candidates[key].push_back(i);
    }
    Json out = Json::array();
    for (const auto &g : graphics)
        for (std::size_t i = 0; i < g.at("attributes").size(); ++i) {
            const auto &a = g["attributes"][i];
            if (a["key"] != 22913)
                continue;
            auto key = identity(g.at("stream"), g.at("id"));
            auto matches = candidates.count(key) ? candidates.at(key) : Json::array();
            out.push_back({{"stream", g["stream"]},
                           {"record_id", g["id"]},
                           {"record_offset", g["offset"]},
                           {"attribute_ordinal", i},
                           {"attribute_index", a["index"]},
                           {"attribute_offset", a["offset"]},
                           {"group", a["group"]},
                           {"key", a["key"]},
                           {"material_candidates", matches},
                           {"material_status", matches.empty()       ? "missing"
                                               : matches.size() == 1 ? "resolved"
                                                                     : "ambiguous"},
                           {"payload", a["payload"]},
                           {"decoded", a["decoded"]}});
        }
    return out;
}
Json decode_terrain(const Json &attributes) {
    constexpr unsigned END = 2139999999, UNSET = 2138888888;
    std::map<unsigned, std::vector<const Json *>> groups;
    for (auto &a : attributes)
        if (a["key"] == 23223)
            groups[a["group"]].push_back(&a);
    auto single = [&](unsigned group) {
        require(groups[group].size() == 1,
                "terrain single attribute group " + std::to_string(group));
        return bytesof(groups[group][0]->at("payload"));
    };
    auto header = single(301);
    require(header.size() == 316 && hex(slice(header, 0, 8)) == "4d4454452c010000",
            "MDTE terrain header");
    auto rawp = single(303), rawm = single(304);
    require(rawp.size() % 24 == 0 && rawp.size() == rawm.size(), "terrain vertex metadata stride");
    Reader pr(rawp), mr(rawm), hr(header);
    Json points = Json::array(), metadata = Json::array(), adj = Json::array();
    std::vector<std::array<unsigned, 6>> vertices;
    std::array<double, 3> low = {INFINITY, INFINITY, INFINITY},
                          high = {-INFINITY, -INFINITY, -INFINITY};
    while (pr.left()) {
        auto p = pr.doubles(3);
        for (unsigned i = 0; i < 3; ++i) {
            double v = p[i];
            require(std::isfinite(v), "terrain nonfinite point");
            low[i] = std::min(low[i], v);
            high[i] = std::max(high[i], v);
        }
        points.push_back(p);
        std::array<unsigned, 6> v;
        for (auto &x : v)
            x = mr.u32();
        vertices.push_back(v);
        metadata.push_back(v);
    }
    require(!points.empty(), "terrain empty points");
    auto chunks = groups[305];
    std::sort(chunks.begin(), chunks.end(),
              [](auto a, auto b) { return a->at("index") < b->at("index"); });
    require(!chunks.empty(), "terrain missing adjacency");
    std::vector<std::array<unsigned, 2>> edges;
    Json chunk_sizes = Json::array(), chunk_indices = Json::array();
    for (std::size_t i = 0; i < chunks.size(); ++i) {
        require(chunks[i]->at("index") == i, "terrain chunk index");
        auto b = bytesof(chunks[i]->at("payload"));
        require(b.size() % 8 == 0, "terrain adjacency stride");
        chunk_sizes.push_back(b.size());
        chunk_indices.push_back(i);
        Reader r(b);
        while (r.left()) {
            std::array<unsigned, 2> e = {r.u32(), r.u32()};
            edges.push_back(e);
            adj.push_back(e);
        }
    }
    for (auto off : {104, 108, 120, 124, 128})
        require(hr.at<std::uint32_t>(off) == points.size(), "terrain header point count");
    for (auto off : {132, 136})
        require(hr.at<std::uint32_t>(off) == edges.size(), "terrain header adjacency count");
    hr.p = 8;
    for (auto v : low)
        require(std::abs(v - hr.f64()) <= 1e-8, "terrain low bounds");
    for (auto v : high)
        require(std::abs(v - hr.f64()) <= 1e-8, "terrain high bounds");
    std::map<unsigned, unsigned> previous, next;
    for (unsigned i = 0; i < vertices.size(); ++i) {
        auto &v = vertices[i];
        require(v[0] == 0 && v[3] == UNSET && v[4] == UNSET && v[5] == END,
                "terrain metadata flags");
        if (v[1] == UNSET)
            continue;
        require(v[1] < points.size() && !previous.count(v[1]), "terrain boundary successor");
        previous[v[1]] = i;
        next[i] = v[1];
    }
    for (auto &p : previous)
        require(next.count(p.first), "terrain open boundary");
    std::vector<int> owners(edges.size(), -1);
    std::vector<std::vector<unsigned>> neighbors;
    std::set<std::pair<unsigned, unsigned>> directed;
    for (unsigned i = 0; i < vertices.size(); ++i) {
        auto edge = vertices[i][2];
        std::vector<unsigned> ring;
        std::set<unsigned> unique;
        while (edge != END) {
            require(edge < edges.size() && owners[edge] == -1, "terrain invalid/reused adjacency");
            owners[edge] = int(i);
            auto other = edges[edge][0];
            edge = edges[edge][1];
            require(other < points.size() && other != i && unique.insert(other).second,
                    "terrain invalid neighbor");
            ring.push_back(other);
            directed.insert({i, other});
        }
        require(ring.size() >= 2, "terrain neighbor ring");
        neighbors.push_back(ring);
    }
    require(std::find(owners.begin(), owners.end(), -1) == owners.end(),
            "terrain unreachable adjacency");
    for (auto e : directed)
        require(directed.count({e.second, e.first}), "terrain asymmetric adjacency");
    std::map<std::array<unsigned, 3>, std::size_t> lookup;
    std::vector<std::array<unsigned, 3>> faces;
    std::vector<unsigned> counts;
    for (unsigned i = 0; i < neighbors.size(); ++i) {
        auto &ring = neighbors[i];
        for (std::size_t j = 0; j < ring.size(); ++j) {
            auto a = ring[j], b = ring[(j + 1) % ring.size()];
            if (next.count(i) && a == next[i] && b == previous[i])
                continue;
            require(directed.count({a, b}), "terrain face edge missing");
            std::array<unsigned, 3> face = {i, a, b}, key = face;
            std::sort(key.begin(), key.end());
            if (!lookup.count(key)) {
                lookup[key] = faces.size();
                faces.push_back(face);
                counts.push_back(1);
            } else {
                auto ix = lookup[key];
                auto orig = faces[ix];
                bool cyclic = face == orig;
                std::rotate(orig.begin(), orig.begin() + 1, orig.end());
                cyclic |= face == orig;
                std::rotate(orig.begin(), orig.begin() + 1, orig.end());
                cyclic |= face == orig;
                require(cyclic, "terrain inconsistent winding");
                counts[ix]++;
            }
        }
    }
    for (auto c : counts)
        require(c == 3, "terrain three rings per face");
    std::map<std::pair<unsigned, unsigned>, unsigned> mesh_edges;
    auto edgekey = [](unsigned a, unsigned b) {
        return std::make_pair(std::min(a, b), std::max(a, b));
    };
    for (auto face : faces)
        for (unsigned i = 0; i < 3; ++i)
            mesh_edges[edgekey(face[i], face[(i + 1) % 3])]++;
    std::set<std::pair<unsigned, unsigned>> boundary, actual;
    std::size_t interior = 0;
    for (auto p : next)
        boundary.insert(edgekey(p.first, p.second));
    for (auto e : mesh_edges) {
        require(e.second == 1 || e.second == 2, "terrain nonmanifold edge");
        if (e.second == 1)
            actual.insert(e.first);
        else
            interior++;
    }
    require(boundary == actual, "terrain boundary disagreement");
    auto ne = hr.at<std::uint32_t>(80), nf = hr.at<std::uint32_t>(84);
    require(ne == mesh_edges.size() && nf == faces.size(), "terrain topology header counts");
    std::set<unsigned> remaining;
    for (auto p : next)
        remaining.insert(p.first);
    Json loops = Json::array();
    while (!remaining.empty()) {
        auto start = *remaining.begin(), current = start;
        Json loop = Json::array();
        while (remaining.erase(current)) {
            loop.push_back(current);
            current = next.at(current);
        }
        require(current == start, "terrain boundary loop");
        loops.push_back(loop);
    }
    auto transform = decode_attribute(313, 23223, single(313));
    transform.erase("encoding");
    Json source = Json::array();
    for (auto &a : attributes)
        if (a["key"] == 23223 && a["group"] != 303 && a["group"] != 304 && a["group"] != 305)
            source.push_back({{"group", a["group"]},
                              {"index", a["index"]},
                              {"payload_hex", hex(bytesof(a["payload"]))}});
    return {{"encoding", "terrain_neighbor_rings"},
            {"vertices", points},
            {"faces", faces},
            {"neighbor_rings", neighbors},
            {"vertex_metadata", metadata},
            {"adjacency_entries", adj},
            {"boundary_loops", loops},
            {"bounds", {low, high}},
            {"transform", transform},
            {"header_hex", hex(header)},
            {"header_edge_count", ne},
            {"header_face_count", nf},
            {"chunk_indices", chunk_indices},
            {"chunk_sizes", chunk_sizes},
            {"source_attributes", source},
            {"topology_checks",
             {{"all_adjacencies_owned", true},
              {"reciprocal_adjacencies", true},
              {"three_rings_per_face", true},
              {"boundary_edges", boundary.size()},
              {"interior_edges", interior},
              {"euler_characteristic", static_cast<std::int64_t>(points.size()) -
                                           static_cast<std::int64_t>(mesh_edges.size()) +
                                           static_cast<std::int64_t>(faces.size())}}},
            {"unresolved_header_spans", Json::array({{{"offset", 56}, {"bytes", 24}},
                                                     {{"offset", 88}, {"bytes", 16}},
                                                     {{"offset", 112}, {"bytes", 8}},
                                                     {{"offset", 140}, {"bytes", 176}}})},
            {"note", "Source ring winding retained. Header capacities/flags, vertex sentinels and "
                     "rendering settings are not fully interpreted."}};
}
} // namespace p3d
