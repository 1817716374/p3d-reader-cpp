#include "internal.hpp"
#include "native_attribute_sort.hpp"
#include <numeric>

namespace p3d {
Json native_attribute_lookup(const Json &attributes) {
    Json result = {{"status", "unresolved"},
                   {"reader_profile", "bimbase_2025_attribute_key_lookup"},
                   {"query_scope", "exact_key_and_index"},
                   {"sorted_source_ordinals", Json::array()},
                   {"keys", Json::array()}};
    try {
        using Key = std::pair<std::uint32_t, std::uint32_t>;
        std::vector<Key> keys;
        for (const auto &a : attributes) {
            const auto group = a.at("group").get<std::uint64_t>();
            const auto key = a.at("key").get<std::uint64_t>();
            const auto index = a.at("index").get<std::uint64_t>();
            require(group <= 0xffff && key <= 0xffff && index <= 0xffffffff,
                    "attribute lookup key outside native field width");
            keys.emplace_back(static_cast<std::uint32_t>((key << 16) | group),
                              static_cast<std::uint32_t>(index));
        }
        std::vector<std::size_t> order(keys.size());
        std::iota(order.begin(), order.end(), 0);
        auto less = [&](std::size_t a, std::size_t b) { return keys[a] < keys[b]; };
        NativeAttributeSort<decltype(less)> sorter(order, less);
        sorter.sort(0, order.size(), order.size());
        result["sorted_source_ordinals"] = order;
        const bool reverse_scan = order.size() <= 5;
        result["lookup_method"] = reverse_scan ? "reverse_scan" : "lower_bound";
        for (std::size_t first = 0; first < order.size();) {
            auto last = first + 1;
            const auto key = keys[order[first]];
            while (last < order.size() && keys[order[last]] == key)
                ++last;
            Json matches = Json::array();
            for (auto i = first; i < last; ++i)
                matches.push_back(order[i]);
            result["keys"].push_back(
                {{"group", key.first & 0xffff},
                 {"key", key.first >> 16},
                 {"index", key.second},
                 {"matching_source_ordinals", std::move(matches)},
                 {"selected_source_ordinal", order[reverse_scan ? last - 1 : first]}});
            first = last;
        }
        result["status"] = "resolved";
    } catch (const std::exception &e) {
        result["error"] = e.what();
    }
    return result;
}

Json native_material_attribute(const Json &member, const Json &input) {
    for (std::size_t i = 0; i < input.at("attachments").size(); ++i) {
        const auto &attachment = input.at("attachments").at(i);
        if (attachment.at("target").at("input_occurrence_index") !=
            member.at("input_occurrence_index"))
            continue;
        const auto &lookup = attachment.at("lookup");
        if (lookup.at("status") != "resolved")
            return {{"status", "unresolved"}, {"reason", "attribute_lookup_unresolved"}};
        for (const auto &key : lookup.at("keys")) {
            if (key.at("group") == 0 && key.at("key") == 20014 && key.at("index") == 0)
                return {{"status", "selected"},
                        {"attachment_index", i},
                        {"source_ordinal", key.at("selected_source_ordinal")}};
        }
        return {{"status", "absent"},
                {"attachment_index", i},
                {"reason", "material_attribute_key_not_found"}};
    }
    const auto status = input.value("status", Json());
    if (status == "resolved" || status == "absent" || status == "not_loaded")
        return {{"status", "absent"}, {"reason", "no_attached_collection"}};
    return {{"status", "unresolved"}, {"reason", "attribute_input_incomplete"}};
}
} // namespace p3d
