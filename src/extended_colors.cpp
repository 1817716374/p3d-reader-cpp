#include "extended_colors.hpp"
namespace p3d {
namespace {
Json scan_color(const std::string &text) {
    // 2667bb: initialize ints to 255; UCRT L"(%d,%d,%d)"; retain low bytes.
    std::array<unsigned, 3> rgb{255, 255, 255};
    std::size_t p = 0;
    unsigned assigned = 0;
    if (!text.empty() && text[p++] == '(') {
        for (unsigned channel = 0; channel < 3; ++channel) {
            while (p < text.size() && (text[p] == ' ' || (text[p] >= '\t' && text[p] <= '\r'))) ++p;
            bool negative = false;
            if (p < text.size() && (text[p] == '+' || text[p] == '-')) negative = text[p++] == '-';
            const auto start = p;
            const auto limit = negative ? 0x8000000000000000ull : 0x7fffffffffffffffull;
            std::uint64_t value = 0;
            while (p < text.size() && text[p] >= '0' && text[p] <= '9') {
                const auto digit = unsigned(text[p++] - '0');
                value = value > (limit - digit) / 10 ? limit : value * 10 + digit;
            }
            if (p == start) break;
            rgb[channel] = unsigned(negative ? std::uint64_t(0) - value : value) & 255u;
            ++assigned;
            // Literal commas do not skip whitespace. A missing closing ')'
            // cannot undo successful assignments; the native return is ignored.
            if (channel < 2 && (p == text.size() || text[p++] != ',')) break;
        }
    }
    return {{"rgb", Json::array({rgb[0], rgb[1], rgb[2]})}, {"assigned_channels", assigned}};
}
}
Json decode_native_extended_colors(const Json &tree) {
    Json out = {{"status", "unresolved"}, {"scope", "R1.18_extended_color_XML_input"},
                {"runtime_table_selection", "not_evaluated"}, {"source_locale", "UCRT_C"},
                {"source_entries", Json::array()}, {"entries", Json::array()}, {"rgb_lookup", Json::array()}};
    try {
        // Native XPath //Entry includes the matching root and descendants,
        // excludes namespace-qualified nodes, and retains document order.
        struct Pending { const Json *node; Json path; };
        std::vector<Pending> pending{{&tree, Json::array()}};
        std::map<std::uint32_t, std::size_t> last_by_rgb;
        while (!pending.empty()) {
            auto current = std::move(pending.back()); pending.pop_back();
            const auto &node = *current.node;
            if (node.at("tag") == "Entry") {
                const auto &attributes = node.at("attributes");
                require(attributes.is_object(), "invalid_extended_color_attributes");
                Json source = {{"source_path", current.path}, {"source_attributes", attributes},
                    {"source_ordinal", out["source_entries"].size()}, {"action", "skip_missing_color"}};
                if (attributes.contains("Color")) {
                    require(attributes.at("Color").is_string(), "extended_color_text_required");
                    const auto &color = attributes.at("Color").get_ref<const std::string &>();
                    source["action"] = "skip_empty_color";
                    if (!color.empty()) {
                        auto entry = scan_color(color);
                        entry["ordinal"] = out["entries"].size();
                        entry["native_index"] = out["entries"].size() + 1;
                        entry["source_ordinal"] = source.at("source_ordinal");
                        entry["source_path"] = current.path;
                        entry["source_attributes"] = attributes;
                        entry["book_name"] = nullptr;
                        const auto book = attributes.value("Book", std::string());
                        const auto name = attributes.value("Name", std::string());
                        if (!book.empty() && !name.empty()) entry["book_name"] = {{"book", book}, {"name", name}};
                        const auto &rgb = entry.at("rgb");
                        const auto key = (rgb[0].get<unsigned>() << 16) | (rgb[1].get<unsigned>() << 8) | rgb[2].get<unsigned>();
                        // 266990 appends even duplicates and updates RGB->index
                        // to the most recently appended one-based slot.
                        last_by_rgb[key] = out["entries"].size();
                        source.update({{"action", "append"}, {"native_index", entry.at("native_index")}});
                        out["entries"].push_back(std::move(entry));
                    }
                }
                out["source_entries"].push_back(std::move(source));
            }
            const auto &children = node.at("children");
            require(children.is_array(), "invalid_extended_color_children");
            for (std::size_t i = children.size(); i; --i) {
                auto path = current.path; path.push_back(i - 1);
                pending.push_back({&children[i - 1], std::move(path)});
            }
        }
        for (const auto &[key, ordinal] : last_by_rgb)
            out["rgb_lookup"].push_back({{"rgb_key", key}, {"native_index", ordinal + 1}});
        out["slot_count"] = out["entries"].size();
        out["status"] = "resolved";
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
}
