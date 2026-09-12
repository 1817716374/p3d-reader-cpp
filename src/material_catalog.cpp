#include "internal.hpp"

namespace p3d {
Json native_material_catalog_records(const Json &native_records) {
    Json out = Json::array();
    for (std::size_t ni = 0; ni < native_records.size(); ++ni) {
        const auto &n = native_records[ni];
        if (n.at("element_type") != 49)
            continue;
        const auto base = bytesof(n.at("data"));
        if (base.size() < 20 || Reader(base, 16).u32() != 18)
            continue;
        Json strings = Json::array();
        Json name = {{"status", "missing"}, {"entry_index", nullptr}, {"value", nullptr}};
        Json resource = name;
        const auto &links = n.at("links");
        for (std::size_t li = 0; li < links.size(); ++li) {
            const auto &l = links[li];
            if (l.at("app") != 0x56d2)
                continue;
            const auto b = bytesof(l.at("payload"));
            const bool user = (l.at("header").get<unsigned>() & 0x1000) != 0;
            Json item = {{"linkage_index", li},
                         {"linkage_offset", l.at("offset")},
                         {"payload", l.at("payload")},
                         {"key", nullptr},
                         {"unassigned_word", nullptr},
                         {"status", "invalid"},
                         {"text", nullptr},
                         {"role", "unassigned"},
                         {"reader_selection", "not_selected"}};
            Json *field = nullptr;
            if (user && b.size() >= 2) {
                const auto key = Reader(b).u16();
                item["key"] = key;
                if (key == 1 || key == 3) {
                    item["role"] = key == 1 ? "catalog_name" : "resource_reference";
                    field = key == 1 ? &name : &resource;
                    if ((*field)["entry_index"].is_null()) {
                        (*field)["entry_index"] = strings.size();
                        item["reader_selection"] = "first_match";
                    } else {
                        field = nullptr;
                        item["reader_selection"] = "shadowed";
                    }
                }
            }
            try {
                require(user, "native catalog string requires user linkage");
                require(b.size() >= 8, "native catalog string header is truncated");
                Reader r(b, 2);
                item["unassigned_word"] = r.u16();
                const auto count = r.u32();
                item["declared_byte_count"] = count;
                require(count <= r.left(), "native catalog string exceeds linkage payload");
                const auto text = r.take(count);
                item["text"] = native_string(text);
                item["status"] = "decoded";
                // String length excludes the linkage's alignment padding. Preserve
                // the suffix without making it part of the source string.
                item["suffix_hex"] = hex(r.take(r.left()));
            } catch (const std::exception &e) {
                item["decode_error"] = e.what();
            }
            if (field) {
                (*field)["status"] = item["status"];
                (*field)["value"] = item["text"];
            }
            strings.push_back(std::move(item));
        }
        out.push_back({{"stream", n.value("stream", Json())},
                       {"native_record_index", ni},
                       {"record_id", n.at("id")},
                       {"record_offset", n.at("offset")},
                       {"strings", std::move(strings)},
                       {"catalog_name", std::move(name)},
                       {"resource_reference", std::move(resource)},
                       {"resource_lookup_status", "not_performed"}});
    }
    return out;
}
} // namespace p3d
