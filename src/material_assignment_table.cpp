#include "internal.hpp"
#include <p3d/material_assignment_table.hpp>
#include <pugixml/pugixml.hpp>
#include <codecvt>
#include <locale>

namespace p3d {
namespace {
std::u16string wide(const std::string &s) {
    std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> codec;
    auto result = codec.from_bytes(s);
    require(codec.converted() == s.size(), "assignment_invalid_utf8");
    result.resize(std::min(result.size(), result.find(u'\0')));
    return result;
}
bool case_equal(const std::string &a, const std::string &b,
                const NativeAssignmentTextContext &text) {
    const auto x = wide(a), y = wide(b);
    if (x == y)
        return true;
    require(bool(text.compare), "assignment_locale_comparison_required");
    auto order = text.compare(x, y);
    require(order.has_value(), "assignment_locale_comparison_unknown");
    return *order == 0;
}
std::string content(pugi::xml_node node) {
    std::string out;
    for (auto child : node.children()) {
        if (child.type() == pugi::node_pcdata || child.type() == pugi::node_cdata)
            out += child.value();
        else if (child.type() == pugi::node_element)
            out += content(child);
    }
    return out;
}
bool space(unsigned char c) {
    return c == ' ' || (c >= 9 && c <= 13);
}
// UCRT unsigned scanf conversion: optional sign, prefix, saturation before narrowing.
std::optional<std::uint64_t> number(const std::string &s, std::size_t &p, unsigned base) {
    while (p < s.size() && space(s[p]))
        ++p;
    const bool negative = p < s.size() && s[p] == '-';
    if (p < s.size() && (s[p] == '+' || negative))
        ++p;
    if (base == 16 && p + 1 < s.size() && s[p] == '0' && (s[p + 1] == 'x' || s[p + 1] == 'X'))
        p += 2;
    std::uint64_t value = 0;
    const auto start = p;
    bool overflow = false;
    for (; p < s.size(); ++p) {
        unsigned digit = s[p] >= '0' && s[p] <= '9'   ? s[p] - '0'
                         : s[p] >= 'a' && s[p] <= 'f' ? s[p] - 'a' + 10
                         : s[p] >= 'A' && s[p] <= 'F' ? s[p] - 'A' + 10
                                                      : 99;
        if (digit >= base)
            break;
        if (value > (UINT64_MAX - digit) / base) {
            value = UINT64_MAX;
            overflow = true;
        } else if (!overflow)
            value = value * base + digit;
    }
    if (start == p)
        return {};
    return negative && !overflow ? 0 - value : value;
}
std::set<std::uint32_t> colors(const Json &entry) {
    return entry.at("rgb_keys").get<std::set<std::uint32_t>>();
}
void combine(Json &destination, const Json &source, bool subtract) {
    for (unsigned i = 0; i < 8; ++i) {
        auto a = destination.at("color_mask_words").at(i).get<std::uint32_t>();
        auto b = source.at("color_mask_words").at(i).get<std::uint32_t>();
        destination["color_mask_words"][i] = subtract ? a & ~b : a | b;
    }
    auto values = colors(destination);
    for (auto c : colors(source)) {
        if (subtract)
            values.erase(c);
        else
            values.insert(c);
    }
    destination["rgb_keys"] = values;
}
bool empty(const Json &entry) {
    for (const auto &word : entry.at("color_mask_words"))
        if (word.get<std::uint32_t>())
            return false;
    return entry.at("rgb_keys").empty();
}
bool color_match(const Json &entry, const NativeAssignmentQuery &query) {
    const auto c = query.color;
    if (c <= 255)
        return (entry.at("color_mask_words").at(c / 32).get<std::uint32_t>() >> (c % 32)) & 1;
    if (c == 0xffffff00u || c >= 0xfffffffeu)
        return false;
    const auto index = (c >> 8) & 0xfffffu;
    if (!index)
        return false; // The native default-color call's result is discarded here.
    require(query.extended_colors.has_value(), "assignment_extended_colors_required");
    if (index > query.extended_colors->size())
        return false;
    const auto rgb = query.extended_colors->at(index - 1);
    const auto key = (std::uint32_t(rgb[0]) << 16) | (std::uint32_t(rgb[1]) << 8) | rgb[2];
    const auto &keys = entry.at("rgb_keys");
    return std::find(keys.begin(), keys.end(), key) != keys.end();
}
bool pattern_match(const std::u16string &pattern, const std::u16string &name,
                   const NativeAssignmentTextContext &text) {
    auto folded_equal = [&](char16_t a, char16_t b) {
        if (a == b)
            return true;
        require(bool(text.uppercase), "assignment_uppercase_required");
        auto x = text.uppercase(a), y = text.uppercase(b);
        require(x.has_value() && y.has_value(), "assignment_uppercase_unknown");
        return x == y;
    };
    std::size_t p = 0, n = 0;
    while (p < pattern.size()) {
        if (pattern[p] == u'*') {
            while (p < pattern.size() && pattern[p] == u'*')
                ++p;
            if (p == pattern.size())
                return true;
            // Native code probes name[n + 1] if this first comparison fails.
            // At the terminator this would read beyond the logical string.
            require(n < name.size(), "assignment_native_pattern_reads_past_name");
            while (!folded_equal(name[n], pattern[p]))
                if (++n == name.size())
                    return false;
        } else if (n == name.size() || !folded_equal(pattern[p], name[n]))
            return false;
        ++p;
        ++n;
    }
    // No test for the query's terminator and no wildcard backtracking.
    return true;
}
} // namespace

Json decode_native_material_assignment_table(const std::string &xml,
                                             const NativeAssignmentTextContext &text) {
    Json out = {{"status", "unresolved"},
                {"scope", "fresh_assignment_list_without_edit_callbacks"},
                {"source_entries", Json::array()},
                {"entries", Json::array()},
                {"source_xml", xml},
                {"material_loading", "not_evaluated"}};
    try {
        pugi::xml_document document;
        auto parsed = document.load_buffer(
            xml.data(), xml.size(), pugi::parse_full | pugi::parse_ws_pcdata, pugi::encoding_utf8);
        require(bool(parsed), "assignment_table_invalid_xml");
        for (auto n : document.children())
            require(n.type() != pugi::node_doctype, "assignment_table_dtd_not_supported");
        auto root = document.document_element();
        require(bool(root), "assignment_table_root_required");
        std::function<void(pugi::xml_node, unsigned)> inspect = [&](pugi::xml_node n,
                                                                    unsigned depth) {
            require(depth <= 64, "assignment_table_xml_depth_limit");
            require(std::string(n.name()).find(':') == std::string::npos,
                    "assignment_table_namespaces_not_supported");
            for (auto a : n.attributes())
                require(std::string(a.name()) != "xmlns" &&
                            std::string(a.name()).find(':') == std::string::npos,
                        "assignment_table_namespaces_not_supported");
            for (auto c : n.children())
                if (c.type() == pugi::node_element)
                    inspect(c, depth + 1);
        };
        inspect(root, 0);
        out["source_tree"] = xml_tree(xml);
        out["name"] = content(root.child("name"));
        out["description"] = content(root.child("description"));
        out["rendering_parameters"] = Json::array();
        for (auto node : root.child("rendering_parameters").children())
            if (node.type() == pugi::node_element)
                out["rendering_parameters"].push_back(content(node));
        bool complete = true;
        std::size_t group_index = 0;
        for (auto group : root.child("assignments").children()) {
            if (group.type() != pugi::node_element)
                continue;
            const auto source_group = group_index++;
            std::size_t pos = 0;
            auto id = number(group.attribute("id").value(), pos, 10);
            const bool valid_id = id && *id != 0 && *id != UINT64_MAX;
            const std::string name = group.attribute("materialName").value();
            if (!valid_id && name.empty())
                continue;
            std::size_t row_index = 0;
            for (auto row : group.children()) {
                if (row.type() != pugi::node_element)
                    continue;
                Json entry = {{"source_group_index", source_group},
                              {"source_row_index", row_index++},
                              {"material_id", valid_id ? *id : UINT64_MAX},
                              {"material_name", name},
                              {"layer_name", ""},
                              {"color_mask_words", Json::array()},
                              {"rgb_keys", Json::array()},
                              {"status", "decoded"}};
                for (auto child : row.children())
                    if (child.type() == pugi::node_pcdata) {
                        entry["layer_name"] = child.value();
                        break;
                    }
                Json attributes = Json::object();
                for (auto a : row.attributes())
                    attributes[a.name()] = a.value();
                for (unsigned i = 0; i < 8; ++i) {
                    auto value =
                        material_xml_integer(attributes, "cMsk" + std::to_string(i), false);
                    entry["color_mask_words"].push_back(
                        value.at("status") == "decoded" ? value.at("value") : Json(0));
                }
                std::set<std::uint32_t> rgb;
                for (auto child : row.children()) {
                    const std::string tag = child.name();
                    if (child.type() != pugi::node_element || tag.size() != 3 ||
                        (tag[0] != 'r' && tag[0] != 'R') || (tag[1] != 'g' && tag[1] != 'G') ||
                        (tag[2] != 'b' && tag[2] != 'B'))
                        continue;
                    const auto value = content(child);
                    std::uint32_t packed = 0;
                    pos = 0;
                    bool assigned = true;
                    for (unsigned i = 0; i < 3; ++i) {
                        auto channel = number(value, pos, 16);
                        if (!channel || (i < 2 && (pos == value.size() || value[pos++] != ','))) {
                            assigned = false;
                            break;
                        }
                        packed = (packed << 8) | (std::uint32_t(*channel) & 255);
                    }
                    if (assigned)
                        rgb.insert(packed);
                    else {
                        entry["status"] = "unresolved";
                        entry["reason"] = "native_rgb_scan_leaves_unassigned_components";
                        complete = false;
                    }
                }
                entry["rgb_keys"] = rgb;
                out["source_entries"].push_back(std::move(entry));
            }
        }
        require(complete, "assignment_table_has_unresolved_entries");
        Json entries = Json::array();
        for (const auto &source : out.at("source_entries")) {
            bool merged = false;
            for (auto &existing : entries) {
                if (!case_equal(existing.at("layer_name"), source.at("layer_name"), text))
                    continue;
                if (existing.at("material_id") == source.at("material_id") &&
                    case_equal(existing.at("material_name"), source.at("material_name"), text)) {
                    combine(existing, source, false);
                    merged = true;
                } else
                    combine(existing, source, true);
            }
            entries.erase(std::remove_if(entries.begin(), entries.end(), empty), entries.end());
            // Native cleanup precedes append, so a new empty rule survives until
            // the next insertion. Do not globally discard empty source rows.
            if (!merged)
                entries.push_back(source);
        }
        out["entries"] = std::move(entries);
        out["status"] = "resolved";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}

Json lookup_native_material_assignment(const Json &table, const NativeAssignmentQuery &query) {
    Json out = {{"status", "unresolved"}, {"material_loading", "not_evaluated"}};
    try {
        require(table.at("status") == "resolved", "normalized_assignment_table_required");
        auto name = query.layer_name;
        name.resize(std::min(name.size(), name.find(u'\0')));
        const auto &entries = table.at("entries");
        for (unsigned pass = 0; pass < 2; ++pass)
            for (std::size_t i = 0; i < entries.size(); ++i) {
                const auto &entry = entries.at(i);
                const auto layer = wide(entry.at("layer_name"));
                bool match = pass == 0 ? layer == name : pattern_match(layer, name, query.text);
                if (!match || !color_match(entry, query))
                    continue;
                out.update({{"status", "matched"},
                            {"entry_index", i},
                            {"pass", pass == 0 ? "exact" : "pattern"},
                            {"material_id", entry.at("material_id")},
                            {"material_name", entry.at("material_name")}});
                return out;
            }
        out["status"] = "not_found";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
